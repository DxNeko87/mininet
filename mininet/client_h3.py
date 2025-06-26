# -*- coding: utf-8 -*-

import socket
import time
import os

HOST = '10.0.0.4'
PORT = 5001
THRESHOLD = 150  # MBps
TMP = 0
LOCAL_IP = ['10.0.0.3']
COUNTER = 0
SOCKET_LIST = []

message = b'x' * 5000 

def create_socket(ip):
    global COUNTER

    print("lol")
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.bind((ip, 0)) 
    s.settimeout(5.0)  
    try:
        print("[INFO] 嘗試連線：%s ➜ %s" % (ip, HOST))
        s.connect((HOST, PORT))
        COUNTER += 1
        print("[OK] 成功建立新連線：%s ➜ %s" % (ip, HOST))
        return s
    except socket.error as e:
        print("[ERROR] 無法連線 %s ➜ %s，錯誤：%s" % (ip, HOST, e))
        s.close()
        return None

SOCKET_LIST.append(create_socket(LOCAL_IP[0]))

total_sent_list = [0] 
start_time = time.time()

try:
    while True:
        for idx, sock in enumerate(SOCKET_LIST):
            sock.sendall(message)
            total_sent_list[idx] += len(message)

        elapsed = time.time() - start_time
        if elapsed >= 1.0:
            total_bytes = sum(total_sent_list)
            rate_mbps = (total_bytes * 8) / 1024 / 1024 / elapsed

            print("Send rate: %.2f MBPS" % rate_mbps)
            if rate_mbps - TMP >= THRESHOLD:
                TMP = rate_mbps
                tmp_ip = '10.0.%d.3' % COUNTER
                os.system("ifconfig h1-eth0:%d %s/24 up"%(COUNTER,tmp_ip))
                rd = COUNTER
                print("test")
                new_sock = create_socket(tmp_ip)
                print(elapsed)
                if COUNTER != rd:
 		    SOCKET_LIST.append(new_sock)
                    LOCAL_IP.append(tmp_ip)
                    total_sent_list.append(0)

            # reset counter
            total_sent_list = [0 for _ in SOCKET_LIST]
            start_time = time.time()

except KeyboardInterrupt:
    for s in SOCKET_LIST:
        s.close()
    print("Stopped by user")
