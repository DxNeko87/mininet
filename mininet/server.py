# -*- coding: utf-8 -*-
import socket
import threading

HOST = '0.0.0.0'
PORT = 5001

def handle_client(conn, addr):
    print("New connection from:", addr)
    while True:
        try:
            data = conn.recv(4096)
            if not data:
                print("%s disconnect"% addr)
                break
        except socket.error as e:
            print("[ERROR] from %s connection error：%s"%(addr,e))
            break
    conn.close()

def main():
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.bind((HOST, PORT))
    s.listen(10)
    print("Server start：%s:%s" % (HOST, PORT))

    while True:
        conn, addr = s.accept()
        t = threading.Thread(target=handle_client, args=(conn, addr))
        t.daemon = True  
        t.start()

if __name__ == '__main__':
    main()
