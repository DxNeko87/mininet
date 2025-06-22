# -*- coding: utf-8 -*-

import time
import os

THRESHOLD = 0

IFACE = "h1-eth0"

def get_tx_rate():
    with open("/sys/class/net/%s/statistics/tx_bytes" % IFACE) as f:
        tx1 = int(f.read())
    time.sleep(1)
    with open("/sys/class/net/%s/statistics/tx_bytes" % IFACE) as f:
        tx2 = int(f.read())
    rate_mbps = (tx2 - tx1) * 8 / 1000000.0
    return rate_mbps


while True:
    rate = get_tx_rate()
    print("[auto] Data sent from client %s: %.2f Mbps"%(IFACE,rate))

    if rate > THRESHOLD:
        print("[auto] exceed THRESHOLD，add new interface")
        os.system("ifconfig h1-eth0:1 10.0.1.1/24 up")
        THRESHOLD = rate + 50

    time.sleep(5)
