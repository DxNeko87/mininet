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

port_h1=1
host_switch={
    1:0,
    2:4,
    3:3
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
        self.host_to_switch = {}
        self.mac_to_port = {} 
        self.port_stats = {}  # save port data
        self.removed_bandwidth = {}
        self.removed_adjacency ={}
        self.new_path=[]
        self.old_path=[]
        self.monitor_thread = hub.spawn(self._monitor)
        self.ip_counter = 1
        self.src_subflow_counter = 0
        self.dst_subflow_counter = 0
        self.read_bandwidth_file("bw.txt")
        self.subflow_counter = 1 
        self.subflow_limit = 30

        print("Please enter your source host number: ")
        self.src_ip = int(input())
        print("Please enter your destination host number: ")
        self.dst_ip = int(input())
        if self.dst_ip == 3:
            self.port_h2=3
        elif self.dst_ip == 2:
            self.port_h2=4

    def add_subflow_to_host(self, host_name, new_ip, interface):
        """
        host_name_ex:h1
        new_ip_ex:10.0.0.1/24
        interface_ex:h1-eth0:1
        """
        try:
            with open("/tmp/%s.pid" % host_name, "r") as f:
                pid = f.read().strip()

            cmd_add_ip = "mnexec -a %s ifconfig %s %s up" % (pid, interface, new_ip)
            subprocess.check_call(cmd_add_ip, shell=True)
            print ("[INFO] 成功新增 IP %s 到 %s 的 %s" % (new_ip, host_name, interface))

        except Exception as e:
            print ("[ERROR] 無法新增 subflow: %s" % str(e))



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
        read bandwidth file save it in 'self.bandwidth'
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
                        self.bandwidth[node1][node2] = bw
                        # bidirection connect(if needed)
                        if node2 not in self.bandwidth:
                            self.bandwidth[node2] = {}
                        self.bandwidth[node2][node1] = bw
                    else:
                        print ("Invalid line format in bw.txt:", line)
        except IOError:
            print ("Error: Could not read file", filename)

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
	
        match = parser.OFPMatch(eth_type=0x0806,arp_tpa=arp_tpa ,in_port=in_port)   
        actions = [parser.OFPActionOutput(out_port)]

        inst = [parser.OFPInstructionActions(ofproto.OFPIT_APPLY_ACTIONS, actions)]
        mod = parser.OFPFlowMod(datapath=datapath, match=match, instructions=inst)
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

    def find_max_bandwidth_path_recalculate(self, src, dst, src_ip, dst_ip):
        """
        Dijkstra algorithm to find the path with maximum bandwidth.
        After finding the path, install flow entries.
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
                min_bw = min(bw, self.bandwidth[u][v])  # calculate bw
                if min_bw > max_bandwidth[v]:  # find better route
                    max_bandwidth[v] = min_bw
                    prev[v] = u
                    heapq.heappush(pq, (-min_bw, v))  # negative into stack
        self.old_path=self.new_path
        # recreate
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
        self.new_path=path
        print(self.old_path)
        print(self.new_path)
        if(self.old_path!=self.new_path):
            # **install after all best found**
            self.install_flows(path, src_ip,dst_ip)
            self.ip_counter+=1

    def find_max_bandwidth_path(self, src, dst, src_ip, dst_ip):
        """
        Dijkstra algorithm to find the path with maximum bandwidth.
        After finding the path, install flow entries.
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
                min_bw = min(bw, self.bandwidth[u][v])  # calculate bw
                if min_bw > max_bandwidth[v]:  # find better route
                    max_bandwidth[v] = min_bw
                    prev[v] = u
                    heapq.heappush(pq, (-min_bw, v))  # negative into stack
        self.old_path=self.new_path
        # recreate
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
        self.new_path=path
        # **install after all best found**
        self.install_flows(path, src_ip,dst_ip)

    def install_flows(self, path, src_ip, dst_ip):
        """
        install after best found
        """
        for i in range(len(path)):
            current_switch = path[i]

            if i == 0:
                # first
                next_switch = path[i + 1]
                in_port = port_h1
                out_port = self.adjacency[current_switch][next_switch]
                self.install_flow(self.datapath_list[current_switch], in_port, out_port, src_ip, dst_ip)
                self.install_flow(self.datapath_list[current_switch], out_port, in_port, dst_ip, src_ip)
                self.install_flow_arp(self.datapath_list[current_switch], dst_ip, in_port, out_port)
                self.install_flow_arp(self.datapath_list[current_switch], src_ip, out_port, in_port)
                print("Installed flow: Switch %d, in_port: %d -> out_port: %d" % (current_switch, in_port, out_port))

            elif i == len(path) - 1:
                # last
                prev_switch = path[i - 1]
                in_port = self.adjacency[current_switch][prev_switch]
                out_port = self.port_h2
                self.install_flow(self.datapath_list[current_switch], in_port, out_port, src_ip, dst_ip)
                self.install_flow(self.datapath_list[current_switch], out_port, in_port, dst_ip, src_ip)
                self.install_flow_arp(self.datapath_list[current_switch], dst_ip, in_port, out_port)
                self.install_flow_arp(self.datapath_list[current_switch], src_ip, out_port, in_port)
                print("Installed flow: Switch %d, in_port: %d -> out_port: %d" % (current_switch, in_port, out_port))

            else:
                # mid
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
    
    @set_ev_cls([event.EventSwitchEnter,
                 event.EventSwitchLeave,
                 event.EventPortAdd,
                 event.EventPortDelete,
                 event.EventPortModify,
                 event.EventLinkAdd,
                 event.EventLinkDelete,
                 event.EventHostAdd])
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
        mylinks = [(link.src.dpid, link.dst.dpid, link.src.port_no, link.dst.port_no) for link in links_list]
	
        print ("Links in the topology:")
        for s1, s2, port1, port2 in mylinks:
            self.adjacency[s1][s2] = port1
            self.adjacency[s2][s1] = port2
            print ("Switch %d: Port %d <---> Switch %d: Port %d" % (s1, port1, s2, port2))

        print ("Adjacency matrix:")
        for s1 in self.adjacency:
            for s2 in self.adjacency[s1]:
                print ("Switch %d -> Switch %d via Port %d" % (s1, s2, self.adjacency[s1][s2]))

        if len(self.myswitches) > 1:
            # First path for src_ip and dst_ip
            tmp_src_ip = "10.0.0.%d" % self.src_ip
            tmp_dst_ip = "10.0.0.%d" % self.dst_ip
            self.find_max_bandwidth_path(self.myswitches[host_switch[self.src_ip]], 
                                        self.myswitches[host_switch[self.dst_ip]],
                                        tmp_src_ip, 
                                        tmp_dst_ip)

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
    
    @set_ev_cls(ofp_event.EventOFPPortStatsReply, MAIN_DISPATCHER)
    def port_stats_reply_handler(self, ev):
        """
        Handle switch received data
        """
        if not hasattr(self, 'removed_link_stats'):
            self.removed_link_stats = {}
        if not hasattr(self, 'link_inactive_since'):
            self.link_inactive_since = {}

        body = ev.msg.body
        THRESHOLD_MB = 20.0  # 門檻大小（MB）

        dpid = ev.msg.datapath.id
        need_recalculate = False
        removed_links = []

        link_loads = {}
        current_time = time.time()

        # 第一步：收集所有 port 的流量變化量
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

            if delta_rx_mb > self.subflow_limit or delta_tx_mb > self.subflow_limit:
                print ("[INFO] DPID=%s port=%s delta=%d bytes" % (dpid, port_no, delta_tx_mb))
                self.add_subflow_to_host("h1", "10.0.%s.1/24" % self.subflow_counter, "h1-eth0:%s" % self.subflow_counter)
                self.add_subflow_to_host("h2", "10.0.%s.2/24" % self.subflow_counter, "h2-eth0:%s" % self.subflow_counter)
                self.subflow_counter += 1
                self.subflow_limit += (self.subflow_limit/2)
                self.logger.info("[INFO] %s" % self.subflow_limit)


            if port_no < 99999:
                neighbor = None
                for n, p in self.adjacency.get(dpid, {}).items():
                    if p == port_no:
                        neighbor = n
                        break
                if neighbor is None:
                    for link in self.removed_adjacency:
                        if dpid in link:
                            other_sw = link[0] if link[1] == dpid else link[1]
                            port1, port2 = self.removed_adjacency[link]
                            if dpid == link[0] and port_no == port1:
                                neighbor = other_sw
                                break
                            elif dpid == link[1] and port_no == port2:
                                neighbor = other_sw
                                break
                if neighbor is not None:
                    link_key = tuple(sorted((dpid, neighbor)))
                    current_load = link_loads.get(link_key, (0, 0))
                    max_tx = max(current_load[0], delta_tx_mb)
                    max_rx = max(current_load[1], delta_rx_mb)
                    link_loads[link_key] = (max_tx, max_rx)
          
        # 第二步：決定要刪除連結或恢復連結
        for (sw1, sw2), (max_tx, max_rx) in link_loads.iteritems():
            #if max_tx > 1 or max_rx > 1:
                #print(sw1,"TX:", round(max_tx, 2))
                #print(sw2,"RX:", round(max_rx, 2))
          
            if max_tx > THRESHOLD_MB or max_rx > THRESHOLD_MB:
                # 流量過大，要刪除連結
                if sw1 in self.adjacency and sw2 in self.adjacency[sw1]:
                    print("remove route: Switch %d <--> Switch %d" % (sw1, sw2))

                    # 保留流量負載數據，即使刪除 adjacency
                    link_loads[(sw1, sw2)] = (max_tx, max_rx)

                    # 保存 removed_adjacency 和 removed_bandwidth
                    self.removed_adjacency[(sw1, sw2)] = (self.adjacency[sw1][sw2], self.adjacency[sw2][sw1])

                    if sw1 in self.bandwidth and sw2 in self.bandwidth[sw1]:
                        bw_value = self.bandwidth[sw1][sw2]
                        self.removed_bandwidth[(sw1, sw2)] = bw_value

                    # 刪除 adjacency 和 bandwidth
                    del self.adjacency[sw1][sw2]
                    del self.adjacency[sw2][sw1]
                    if sw1 in self.bandwidth and sw2 in self.bandwidth[sw1]:
                        del self.bandwidth[sw1][sw2]
                        del self.bandwidth[sw2][sw1]

                    # 記錄移除時間
                    self.link_inactive_since[(sw1, sw2)] = current_time
                    removed_links.append((sw1, sw2))
                    need_recalculate = True

            else:
                # 流量小，考慮恢復之前刪除的連結（需超過7秒無明顯改動）
                if (sw1, sw2) in self.removed_adjacency :
                    last_inactive = self.link_inactive_since.get((sw1, sw2), 0)
                    print(current_time - last_inactive)
                    if current_time - last_inactive >= 7:
                        need_recalculate = True
                        print("restore route: Switch %d <--> Switch %d" % (sw1, sw2))

                        # re from removed_adjacency 中恢復連接
                        port1, port2 = self.removed_adjacency[(sw1, sw2)]

                        # re adjacency
                        self.adjacency.setdefault(sw1, {})[sw2] = port1
                        self.adjacency.setdefault(sw2, {})[sw1] = port2

                        # re bandwidth
                        if (sw1, sw2) in self.removed_bandwidth:
                            bw_value = self.removed_bandwidth[(sw1, sw2)]
                            self.bandwidth.setdefault(sw1, {})[sw2] = bw_value
                            self.bandwidth.setdefault(sw2, {})[sw1] = bw_value
                            del self.removed_bandwidth[(sw1, sw2)]

                        # re then delete
                        del self.removed_adjacency[(sw1, sw2)]
                        del self.link_inactive_since[(sw1, sw2)]
                        break  # re 1
            
            if (sw1, sw2) in self.removed_adjacency :
                if max_tx > THRESHOLD_MB or max_rx > THRESHOLD_MB:
                    self.link_inactive_since[(sw1, sw2)] = current_time

        # recalculate
        if need_recalculate:
            print("recalculate")
            temp_src_ip = "10.0.%d.%d" % (self.ip_counter,self.src_ip)
            temp_dst_ip = "10.0.%d.%d" % (self.ip_counter,self.dst_ip)
            self.find_max_bandwidth_path_recalculate(self.myswitches[host_switch[self.src_ip]], self.myswitches[host_switch[self.dst_ip]], temp_src_ip, temp_dst_ip)
