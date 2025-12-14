# -*- coding: utf-8 -*-
from os import link, remove
from xml.sax.handler import property_dom_node
from ryu.base import app_manager
from ryu.controller import ofp_event
from ryu.controller.handler import set_ev_cls, MAIN_DISPATCHER, CONFIG_DISPATCHER
from ryu.ofproto import ofproto_v1_3
from ryu.lib.packet import packet, ethernet, arp
from ryu.topology import event, switches
from ryu.topology.api import get_switch, get_link, get_host
from ryu.lib.packet import arp, ether_types
from ryu.lib import hub
import heapq
import time
import subprocess

#host_to_switch_number
host_switch={
    '1':0,
    '2':8,
    '3':5,
    '4':6,
}

class MyTopologyApp(app_manager.RyuApp):
    OFP_VERSIONS = [ofproto_v1_3.OFP_VERSION]

    def __init__(self, *args, **kwargs):
        super(MyTopologyApp, self).__init__(*args, **kwargs)
        self.topology_api_app = self
        self.myswitches = []
        self.adjacency = {}
        self.datapath_list = {}
        self.bandwidth = {}
        self.original_bandwidth = {}  # 保存原始带宽
        self.host_to_switch = {}
        self.mac_to_port = {}
        self.port_stats = {}  # save port data
        self.removed_bandwidth = {}
        self.removed_adjacency ={}
        self.new_path=[]
        self.old_path=[]
        self.monitor_thread = hub.spawn(self._monitor)
        self.ip_counter = 1
        self.read_bandwidth_file("bw.txt")
        self.subflow_counter = 1
        self.subflow_limit = 30
        self.arp_src ="10.0.0.0"
        self.arp_dst ="10.0.0.0"
        self.src_ip=1
        self.dst_ip=2
        self.arp_flow_table={}
        self.ip_flow_table={}
        self.check_src = '1'
        self.allocated_flows = {}  # 记录已分配的流及其占用的带宽

    def _monitor(self):
        reset_interval = 1
        elapsed_time = 0
        while True:
            if elapsed_time > reset_interval:
                self.port_stats.clear()
                elapsed_time = 0
            print("--------------------")
            for datapath in self.datapath_list.values():
                self.request_stats(datapath)
            hub.sleep(1)
            elapsed_time += 1

    def read_bandwidth_file(self, filename):
        """
        read bandwidth file save it in 'self.bandwidth' and 'self.original_bandwidth'
        """
        try:
            with open(filename, "r") as file:
                for line in file:
                    parts = line.strip().split()
                    if len(parts) == 3:
                        node1 = int(parts[0])
                        node2 = int(parts[1])
                        bw = int(parts[2])
                        # save bandwidth data
                        if node1 not in self.bandwidth:
                            self.bandwidth[node1] = {}
                            self.original_bandwidth[node1] = {}
                        self.bandwidth[node1][node2] = bw
                        self.original_bandwidth[node1][node2] = bw
                        # bidirection connect(if needed)
                        if node2 not in self.bandwidth:
                            self.bandwidth[node2] = {}
                            self.original_bandwidth[node2] = {}
                        self.bandwidth[node2][node1] = bw
                        self.original_bandwidth[node2][node1] = bw
                    else:
                        print ("Invalid line format in bw.txt:", line)
        except IOError:
            print ("getting bandwidth of each links")
            #print ("Error: Could not read file", filename)

    def install_flow(self, datapath ,in_port, out_port, src_ip, dst_ip):
        ofproto = datapath.ofproto
        parser = datapath.ofproto_parser
        match = parser.OFPMatch(eth_type=0x0800, in_port=in_port, ipv4_src=src_ip, ipv4_dst=dst_ip)
        actions = [parser.OFPActionOutput(out_port)]
        inst = [parser.OFPInstructionActions(ofproto.OFPIT_APPLY_ACTIONS, actions)]
        mod = parser.OFPFlowMod(datapath=datapath, match=match, instructions=inst)
        datapath.send_msg(mod)

    def install_flow_arp(self, datapath, arp_tpa, in_port, out_port):
        ofproto = datapath.ofproto
        parser = datapath.ofproto_parser
        key = (datapath.id,in_port,arp_tpa)
        if not hasattr(self, 'arp_flow_table'):
            self.arp_flow_table = {}
        if key not in self.arp_flow_table:
            self.arp_flow_table[key] = set()
        if out_port in self.arp_flow_table[key]:
            return
        self.arp_flow_table[key].add(out_port)
        match = parser.OFPMatch(eth_type=0x0806,arp_tpa=arp_tpa ,in_port=in_port)
        actions = [parser.OFPActionOutput(p) for p in sorted(self.arp_flow_table[key])]
        self.logger.info(match)
        self.logger.info(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>")
        inst = [parser.OFPInstructionActions(ofproto.OFPIT_APPLY_ACTIONS, actions)]
        mod = parser.OFPFlowMod(datapath=datapath, match=match, instructions=inst, command=ofproto.OFPFC_ADD)
        datapath.send_msg(mod)

    def delete_all_flows(self, datapath):
        """
        Delete all flows in the switch except those related to the controller.
        """
        ofproto = datapath.ofproto
        parser = datapath.ofproto_parser
        # Create a flow mod message to delete all flows except those with output to controller
        match = parser.OFPMatch(priority=65534)  # Match all flows
        mod = parser.OFPFlowMod(
            datapath=datapath,
            command=ofproto.OFPFC_DELETE,
            out_port=ofproto.OFPP_ANY,
            out_group=ofproto.OFPG_ANY,
            match=match,
            instructions=[],
        )
        datapath.send_msg(mod)
        print ("Deleted all flows in switch", datapath.id, "except controller-related flows")

    def get_path_bottleneck_bandwidth(self, path):
        """
        找到路径上的瓶颈带宽（最小带宽）
        """
        if len(path) <= 1:
            return float('inf')
        
        min_bandwidth = float('inf')
        for i in range(len(path) - 1):
            sw1 = path[i]
            sw2 = path[i + 1]
            if sw1 in self.bandwidth and sw2 in self.bandwidth[sw1]:
                link_bw = self.bandwidth[sw1][sw2]
                min_bandwidth = min(min_bandwidth, link_bw)
                print("Link",sw1,"<->",sw2,": current bandwidth =" ,link_bw)
        
        print("Path bottleneck bandwidth: ",min_bandwidth,)
        return min_bandwidth if min_bandwidth != float('inf') else 0

    def allocate_bandwidth_for_path(self, path, allocated_bandwidth):
        """
        为路径分配带宽：从路径上每个链路减去分配的带宽
        """
        print("Allocating ",allocated_bandwidth, "bandwidth for path:",' -> '.join(map(str, path)))
        
        for i in range(len(path) - 1):
            sw1 = path[i]
            sw2 = path[i + 1]
            if sw1 in self.bandwidth and sw2 in self.bandwidth[sw1]:
                # 减去分配的带宽，但不能低于0
                old_bw = self.bandwidth[sw1][sw2]
                self.bandwidth[sw1][sw2] = max(0, self.bandwidth[sw1][sw2] - allocated_bandwidth)
                self.bandwidth[sw2][sw1] = max(0, self.bandwidth[sw2][sw1] - allocated_bandwidth)
                print("Link" ,sw1,"<->",sw2,old_bw," -> ",self.bandwidth[sw1][sw2]," (allocated: ",allocated_bandwidth,")")

    def deallocate_bandwidth_for_path(self, path, allocated_bandwidth):
        """
        为路径释放带宽：恢复路径上每个链路的带宽（不超过原始带宽）
        """
        print("Deallocating" ,allocated_bandwidth, "bandwidth for path: ",' -> '.join(map(str, path)))
        
        for i in range(len(path) - 1):
            sw1 = path[i]
            sw2 = path[i + 1]
            if sw1 in self.bandwidth and sw2 in self.bandwidth[sw1]:
                # 恢复带宽，但不超过原始带宽
                original_bw = self.original_bandwidth.get(sw1, {}).get(sw2, float('inf'))
                old_bw = self.bandwidth[sw1][sw2]
                self.bandwidth[sw1][sw2] = min(original_bw, self.bandwidth[sw1][sw2] + allocated_bandwidth)
                self.bandwidth[sw2][sw1] = min(original_bw, self.bandwidth[sw2][sw1] + allocated_bandwidth)
                print("Link ",sw1,"<->",sw2,": ",old_bw," -> ",self.bandwidth[sw1][sw2]," (deallocated: ",allocated_bandwidth,")")

    def find_max_bandwidth_path_recalculate(self, src, dst, src_ip, dst_ip):
        """
        Dijkstra algorithm to find the path with maximum bandwidth.
        After finding the path, allocate bandwidth for the path.
        """
        # initial
        max_bandwidth = {switch: float('-inf') for switch in self.adjacency}
        prev = {switch: None for switch in self.adjacency}
        max_bandwidth[src] = float('inf')
        pq = [(-float('inf'), src)]

        while pq:
            bw, u = heapq.heappop(pq)
            bw = -bw  # take back positive

            if u == dst:
                break  # find dst, end search

            for v in self.adjacency[u]:
                if self.bandwidth[u][v] <= 0: 
                    continue 
                min_bw = min(bw, self.bandwidth[u][v])  # calculate bw
                if min_bw > max_bandwidth[v]:  # find better route
                    max_bandwidth[v] = min_bw
                    prev[v] = u
                    heapq.heappush(pq, (-min_bw, v))  # negative into stack

        self.old_path = self.new_path
        # recreate path
        path = []
        current = dst
        while current is not None:
            path.append(current)
            current = prev[current]
        path.reverse()

        if not path or len(path) == 1:
            print("No valid path found from", src, "to", dst)
            return

        print("Max bandwidth path:", " -> ".join(map(str, path)))
        self.new_path = path
        print("Old path:", self.old_path)
        print("New path:", self.new_path)

        if self.old_path != self.new_path:
            # 如果有旧路径，先释放其带宽
            if self.old_path and len(self.old_path) > 1:
                flow_key = (src_ip, dst_ip)
                if flow_key in self.allocated_flows:
                    old_allocated_bw = self.allocated_flows[flow_key]['allocated_bandwidth']
                    self.deallocate_bandwidth_for_path(self.old_path, old_allocated_bw)
                    del self.allocated_flows[flow_key]

            # **install after all best found**
            self.install_flows(path, src_ip, dst_ip)
            
            # 计算新路径的瓶颈带宽并分配
            bottleneck_bw = self.get_path_bottleneck_bandwidth(path)
            if bottleneck_bw > 0:
                self.allocate_bandwidth_for_path(path, bottleneck_bw)
                # 记录分配的流
                flow_key = (src_ip, dst_ip)
                self.allocated_flows[flow_key] = {
                    'path': path,
                    'allocated_bandwidth': bottleneck_bw
                }

            self.ip_counter += 1

    def find_max_bandwidth_path(self, src, dst, src_ip, dst_ip):
        """
        Dijkstra algorithm to find the path with maximum bandwidth.
        After finding the path, allocate bandwidth for the path.
        """
        # initial
        max_bandwidth = {switch: float('-inf') for switch in self.adjacency}
        prev = {switch: None for switch in self.adjacency}
        max_bandwidth[src] = float('inf')
        pq = [(-float('inf'), src)]

        while pq:
            bw, u = heapq.heappop(pq)
            bw = -bw  # take back positive

            if u == dst:
                break  # find dst, end search

            for v in self.adjacency[u]:

                if self.bandwidth[u][v] <= 0:
                    continue
                min_bw = min(bw, self.bandwidth[u][v])  # calculate bw
                if min_bw > max_bandwidth[v]:  # find better route
                    max_bandwidth[v] = min_bw
                    prev[v] = u
                    heapq.heappush(pq, (-min_bw, v))  # negative into stack

        self.old_path = self.new_path
        # recreate path
        path = []
        current = dst
        while current is not None:
            path.append(current)
            current = prev[current]
        path.reverse()

        if not path or len(path) == 1:
            print("No valid path found from", src, "to", dst)
            return

        print("Max bandwidth path:", " -> ".join(map(str, path)))
        self.new_path = path

        # **install after all best found**
        self.install_flows(path, src_ip, dst_ip)
        
        # 计算路径的瓶颈带宽并分配
        bottleneck_bw = self.get_path_bottleneck_bandwidth(path)
        if bottleneck_bw > 0:
            self.allocate_bandwidth_for_path(path, bottleneck_bw)
            # 记录分配的流
            flow_key = (src_ip, dst_ip)
            self.allocated_flows[flow_key] = {
                'path': path,
                'allocated_bandwidth': bottleneck_bw
            }

    def install_flows(self, path, src_ip, dst_ip):
        """
        install after best found
        """
        for i in range(len(path)):
            current_switch = path[i]
            if i == 0:  # first
                next_switch = path[i + 1]
                in_port = 1
                out_port = self.adjacency[current_switch][next_switch]
                self.install_flow(self.datapath_list[current_switch], in_port, out_port, src_ip, dst_ip)
                self.install_flow(self.datapath_list[current_switch], out_port, in_port, dst_ip, src_ip)
                self.install_flow_arp(self.datapath_list[current_switch], dst_ip, in_port, out_port)
                self.install_flow_arp(self.datapath_list[current_switch], src_ip, out_port, in_port)
                print("Installed flow: Switch %d, in_port: %d -> out_port: %d" % (current_switch, in_port, out_port))
            elif i == len(path) - 1:  # last
                prev_switch = path[i - 1]
                in_port = self.adjacency[current_switch][prev_switch]
                out_port = 1
                self.install_flow(self.datapath_list[current_switch], in_port, out_port, src_ip, dst_ip)
                self.install_flow(self.datapath_list[current_switch], out_port, in_port, dst_ip, src_ip)
                self.install_flow_arp(self.datapath_list[current_switch], dst_ip, in_port, out_port)
                self.install_flow_arp(self.datapath_list[current_switch], src_ip, out_port, in_port)
                print("Installed flow: Switch %d, in_port: %d -> out_port: %d" % (current_switch, in_port, out_port))
            else:  # mid
                prev_switch = path[i - 1]
                next_switch = path[i + 1]
                in_port = self.adjacency[current_switch][prev_switch]
                out_port = self.adjacency[current_switch][next_switch]
                self.install_flow(self.datapath_list[current_switch], in_port, out_port, src_ip, dst_ip)
                self.install_flow(self.datapath_list[current_switch], out_port, in_port, dst_ip, src_ip)
                self.install_flow_arp(self.datapath_list[current_switch], dst_ip, in_port, out_port)
                self.install_flow_arp(self.datapath_list[current_switch], src_ip, out_port, in_port)
                print("Installed flow: Switch %d, in_port: %d -> out_port: %d" % (current_switch, in_port, out_port))

        print("Largest bandwidth route:", " -> ".join(map(str, path)))

    @set_ev_cls([event.EventSwitchEnter, event.EventSwitchLeave, event.EventPortAdd, 
                event.EventPortDelete, event.EventPortModify, event.EventLinkAdd, 
                event.EventLinkDelete, event.EventHostAdd])
    def get_topology_data(self, ev):
        """
        Get Topology Data
        Switch total
        Links total
        Adjacency matrix
        """
        print ("get_topology_data() is called")
        switch_list = get_switch(self.topology_api_app, None)
        self.myswitches = [switch.dp.id for switch in switch_list]
        for switch in switch_list:
            self.datapath_list[switch.dp.id] = switch.dp
            self.adjacency[switch.dp.id] = {}

        print ("Switches in the topology:")
        for switch_id in self.myswitches:
            print ("Switch ID: %d" % switch_id)

        links_list = get_link(self.topology_api_app, None)
        mylinks = [(link.src.dpid, link.dst.dpid, link.src.port_no, link.dst.port_no) 
                  for link in links_list]
        print ("Links in the topology:")
        for s1, s2, port1, port2 in mylinks:
            self.adjacency[s1][s2] = port1
            self.adjacency[s2][s1] = port2
            print ("Switch %d: Port %d <---> Switch %d: Port %d" % (s1, port1, s2, port2))

        print ("Adjacency matrix:")
        for s1 in self.adjacency:
            for s2 in self.adjacency[s1]:
                print ("Switch %d -> Switch %d via Port %d" % (s1, s2, self.adjacency[s1][s2]))

        print("-------------------------")
        print(self.adjacency)
        print("-------------------------")

    def request_stats(self, datapath):
        """
        send req for datapath info
        """
        parser = datapath.ofproto_parser
        req = parser.OFPPortStatsRequest(datapath, 0, ofproto_v1_3.OFPP_ANY)
        datapath.send_msg(req)

    @set_ev_cls(ofp_event.EventOFPPacketIn, MAIN_DISPATCHER)
    def packet_in_handler(self, ev):
        msg = ev.msg
        pkt = packet.Packet(msg.data)
        eth = pkt.get_protocol(ethernet.ethernet)

        if eth.ethertype != 0x0806:  # 0x0806 是 ARP 封包
            return

        self.logger.info(eth)
        arp_pkt = pkt.get_protocol(arp.arp)
        if arp_pkt:
            self.arp_src = arp_pkt.src_ip
            self.arp_dst = arp_pkt.dst_ip
            self.src_ip = self.arp_src[-1]
            self.dst_ip = self.arp_dst[-1]
            print("ARP Packet - Src IP:", self.arp_src, "Dst IP:", self.arp_dst)

            if len(self.myswitches) > 1:
                # First path for src_ip and dst_ip
                tmp_src_ip = self.arp_src
                tmp_dst_ip = self.arp_dst
                tmp_host_src = self.arp_src[-1]
                tmp_host_dst = self.arp_dst[-1]
                if tmp_host_src != 0 and tmp_host_dst != 0:
                    self.find_max_bandwidth_path(self.myswitches[host_switch[tmp_host_src]], 
                                                self.myswitches[host_switch[tmp_host_dst]], 
                                                tmp_src_ip, tmp_dst_ip)

    @set_ev_cls(ofp_event.EventOFPSwitchFeatures, CONFIG_DISPATCHER)
    def switch_features_handler(self, ev):
        datapath = ev.msg.datapath
        ofproto = datapath.ofproto
        parser = datapath.ofproto_parser

        # 讓所有 ARP 封包都送進 Controller
        match = parser.OFPMatch(eth_type=0x0806)
        actions = [parser.OFPActionOutput(ofproto.OFPP_CONTROLLER, ofproto.OFPCML_NO_BUFFER)]
        inst = [parser.OFPInstructionActions(ofproto.OFPIT_APPLY_ACTIONS, actions)]
        mod = parser.OFPFlowMod(datapath=datapath, priority=1, match=match, instructions=inst)
        datapath.send_msg(mod)

    @set_ev_cls(ofp_event.EventOFPPortStatsReply, MAIN_DISPATCHER)
    def port_stats_reply_handler(self, ev):
        """
        Handle switch received data - simplified version focusing on bandwidth management
        """
        if not hasattr(self, 'removed_link_stats'):
            self.removed_link_stats = {}
        if not hasattr(self, 'link_inactive_since'):
            self.link_inactive_since = {}

        body = ev.msg.body
        THRESHOLD_MB = 1.0  # 門檻大小（MB）
        dpid = ev.msg.datapath.id
        need_recalculate = False
        current_time = time.time()

        # 简化的流量监控，主要用于触发重新计算
        for stat in body:
            port_no = stat.port_no
            tx_bytes = stat.tx_bytes
            rx_bytes = stat.rx_bytes

            prev_tx, prev_rx = self.port_stats.get((dpid, port_no), (tx_bytes, rx_bytes))
            delta_tx = tx_bytes - prev_tx
            delta_rx = rx_bytes - prev_rx
            self.port_stats[(dpid, port_no)] = (tx_bytes, rx_bytes)

            delta_tx_mb = delta_tx / (1024.0 * 1024.0)
            delta_rx_mb = delta_rx / (1024.0 * 1024.0)

            # 当流量超过阈值时，标记需要重新计算
            if port_no < 99999 and (delta_rx_mb > THRESHOLD_MB or delta_tx_mb > THRESHOLD_MB):
                print("[INFO] High traffic detected: DPID={dpid} port={port_no} ")
                print("TX={round(delta_tx_mb, 2)}MB RX={round(delta_rx_mb, 2)}MB")
                need_recalculate = True

        # 如果需要重新计算路径
        if need_recalculate:
            print("Recalculating paths due to traffic changes...")
            if self.check_src != self.src_ip:
                self.ip_counter = 1
                self.check_src = self.src_ip
            
            temp_src_ip = "10.0.%d.%s" % (self.ip_counter, self.src_ip)
            temp_dst_ip = "10.0.0.%s" % (self.dst_ip)
            self.find_max_bandwidth_path_recalculate(
                self.myswitches[host_switch[self.src_ip]], 
                self.myswitches[host_switch[self.dst_ip]], 
                temp_src_ip, temp_dst_ip)
