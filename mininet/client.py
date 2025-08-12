# -*- coding: utf-8 -*-
# client.py (Python 2)
import socket
import time
import sys

def start_client(fake_ip):
    HOST = '10.0.0.2'
    PORT = 5000

    print "[%s] Connecting to %s:%s..."%(fake_ip, HOST, PORT)
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    try:
        s.connect((HOST, PORT))
    except Exception as e:
        print "[%s] Failed to connect: %s"%(fake_ip, e)
        return

    try:
        while True:
            msg = 'x' * 1000  # Python 2: str 就是 bytes
            s.sendall(msg)
            print "[%s] Sent 1000 bytes"%(fake_ip)
    except KeyboardInterrupt:
        print "[%s] Stopped by user."%(fake_ip)
    except Exception as e:
        print "[%s] Error: %s"%(fake_ip, e)
    finally:
        s.close()

if __name__ == "__main__":
    fake_ip = sys.argv[1] if len(sys.argv) > 1 else "10.0.0.1"
    start_client(fake_ip)
