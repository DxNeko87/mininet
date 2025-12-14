# -*- coding: utf-8 -*-
import socket
import time
import os

# server IP and port
HOST = '10.0.0.2'
PORT = 5001

# threshold to see if need to add a new interface
THRESHOLD = 20          # 速率門檻 (MBps)
ADD_INTERVAL = 5      

TMP = 0
LOCAL_IP = ['10.0.0.1']  # 初始IP
COUNTER = 0
SOCKET_LIST = []        # 已建立的 socket 清單

message = b'x' * 5000
tmp_ip = ""

def create_socket(ip):
    global COUNTER
    global tmp_ip
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.bind((ip, 0))
    s.settimeout(10.0)
    try:
        print("[INFO] 嘗試連線：%s ➜ %s" % (ip, HOST))
        s.connect((HOST, PORT))
        COUNTER += 1
        print("[OK] 成功建立新連線：%s ➜ %s" % (ip, HOST))
        return s
    except socket.error as e:
        print("[ERROR] 無法連線 %s ➜ %s，錯誤：%s" % (ip, HOST, e))
        os.system("ifconfig h1-eth0:%d %s/24 down" % (COUNTER, tmp_ip))
        s.close()
        return None

# 建立第一條連線
first_sock = create_socket(LOCAL_IP[0])
if first_sock:
    SOCKET_LIST.append(first_sock)

total_sent_list = [0]
start_time = time.time()
last_add_time = time.time()  # **記錄最後一次新增線路的時間**

try:
    while True:
        # 傳輸資料
        for idx, sock in enumerate(SOCKET_LIST):
            sock.sendall(message)
            total_sent_list[idx] += len(message)

        elapsed = time.time() - start_time
        if elapsed >= 1.0:
            total_bytes = sum(total_sent_list)
            rate_mbps = (total_bytes * 8) / 1024 / 1024 / elapsed

            print(total_sent_list)
            print("總傳輸量: %d bytes" % total_bytes)
            print("Send rate: %.2f MBPS" % rate_mbps)

            # **新增線路的條件**
            now = time.time()
            if rate_mbps - TMP >= THRESHOLD and (now - last_add_time >= ADD_INTERVAL):
                TMP = rate_mbps
                tmp_ip = '10.0.%d.1' % COUNTER
                os.system("ifconfig h1-eth0:%d %s/24 up" % (COUNTER, tmp_ip))
                rd = COUNTER
                print("[INFO] 嘗試新增第二條線路: %s" % tmp_ip)
                new_sock = create_socket(tmp_ip)
                if new_sock and COUNTER != rd:
                    SOCKET_LIST.append(new_sock)
                    LOCAL_IP.append(tmp_ip)
                    total_sent_list.append(0)
                    last_add_time = now   # **更新最後新增時間**

            # 重置統計
            total_sent_list = [0 for _ in SOCKET_LIST]
            start_time = time.time()

except KeyboardInterrupt:
    for s in SOCKET_LIST:
        s.close()
    print("Stopped by user")
