#!/usr/bin/python
# -*- coding: utf-8 -*-

from mininet.topo import Topo
from mininet.net import Mininet
from mininet.node import RemoteController
from mininet.cli import CLI
from mininet.log import setLogLevel

class CustomTopo(Topo):
    def __init__(self):
        Topo.__init__(self)
        
        # 添加2台主机和1台交换机
        h1 = self.addHost('h1',ip="10.0.0.1")
        h2 = self.addHost('h2',ip="10.0.0.2")
        s1 = self.addSwitch('s1')
        s2 = self.addSwitch('s2')
        s3 = self.addSwitch('s3')
        s4 = self.addSwitch('s4')
        s5 = self.addSwitch('s5')
        # 添加链路
        self.addLink(h1, s1)
        self.addLink(s1, s2)
        self.addLink(s1, s3)
        self.addLink(s1, s4)
        self.addLink(s2, s5)
        self.addLink(s3, s5)
        self.addLink(s4, s5)
        self.addLink(s5, h2)

def setup_host_bandwidth(host, interface, bw):
    """为主机接口设置带宽限制"""
    # 先清除现有qdisc
    host.cmd('tc qdisc del dev %s root 2>/dev/null || true' % interface)
    # 添加新的限制
    cmd = 'tc qdisc add dev %s root tbf rate %smbit burst 100kb latency 100ms' % (interface, bw)
    host.cmd(cmd)

def setup_switch_bandwidth(switch, interface, bw):
    """为交换机接口设置带宽限制"""
    # 方法1：使用tc命令（适用于所有交换机类型）
    switch.cmd('tc qdisc del dev %s root 2>/dev/null || true' % interface)
    cmd = 'tc qdisc add dev %s root tbf rate %smbit burst 100kb latency 100ms' % (interface, bw)
    switch.cmd(cmd)
    
    # 方法2：使用OVS QoS（仅适用于OVS交换机）
    # setup_ovs_qos(switch, interface, bw)


def run():
    setLogLevel('info')
    
    # 创建拓扑并连接远程控制器
    topo = CustomTopo()
    net = Mininet(topo=topo, 
                 controller=lambda name: RemoteController(name, ip='127.0.0.1', port=6653),
                 autoSetMacs=True)
    
    net.start()
   
    
    # 获取网络元素
    h1, h2, s1, s2, s3, s4, s5 = net.get('h1', 'h2', 's1', 's2', 's3', 's4', 's5')

    with open("/tmp/h1.pid", "w") as f:
        f.write(str(h1.pid))
    with open("/tmp/h2.pid", "w") as f:
        f.write(str(h2.pid))
    
    # 设置带宽限制
    # 主机侧限制
    setup_host_bandwidth(h1, 'h1-eth0', 1000)
    setup_host_bandwidth(h2, 'h2-eth0', 1000)
    
    # 交换机侧限制 - 对每个接口设置
    setup_switch_bandwidth(s2, 's2-eth1', 100) 
    setup_switch_bandwidth(s2, 's2-eth2', 100) 
    setup_switch_bandwidth(s3, 's3-eth1', 200)  
    setup_switch_bandwidth(s3, 's3-eth2', 200)
    setup_switch_bandwidth(s4, 's4-eth1', 300) 
    setup_switch_bandwidth(s4, 's4-eth2', 300)  
    
    # 进入CLI
    CLI(net)
    

    net.stop()

if __name__ == '__main__':
    run()
