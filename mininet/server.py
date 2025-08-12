# -*- coding: utf-8 -*-
# server.py (Python 2)
import socket
import select
import time

HOST = '10.0.0.2'
PORT = 5000

server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_socket.bind((HOST, PORT))
server_socket.listen(5)
server_socket.setblocking(0)

sockets_list = [server_socket]
clients = {}

total_bytes_received = 0
client_threshold = 300  # 每 300MB 加一個 client

print "Server listening on %s:%s"%(HOST, PORT)

while True:
    read_sockets, _, exception_sockets = select.select(sockets_list, [], sockets_list)

    for notified_socket in read_sockets:
        if notified_socket == server_socket:
            client_socket, client_address = server_socket.accept()
            client_socket.setblocking(0)
            sockets_list.append(client_socket)
            clients[client_socket] = client_address
            print ("Accepted new connection from",client_address)
        else:
            try:
                data = notified_socket.recv(4096)
                if not data:
                    print "%s disconnected"%(clients[notified_socket])
                    sockets_list.remove(notified_socket)
                    del clients[notified_socket]
                    notified_socket.close()
                    continue

                total_bytes_received += len(data)
                print "Received %s bytes"%(total_bytes_received)

                if total_bytes_received >= client_threshold * 1000000:
                    new_ip_index = len(clients)
                    simulated_ip = "10.0.%s.1"%(new_ip_index)
                    print "*** Threshold reached, simulate new client IP: %s ***"%(simulated_ip)
                    client_threshold += 3  # 下一個門檻

            except Exception as e:
                print "Error from %s: %s"%(clients.get(notified_socket, "unknown"), e)
                if notified_socket in sockets_list:
                    sockets_list.remove(notified_socket)
                if notified_socket in clients:
                    del clients[notified_socket]
                notified_socket.close()

    for notified_socket in exception_sockets:
        if notified_socket in sockets_list:
            sockets_list.remove(notified_socket)
        if notified_socket in clients:
            del clients[notified_socket]
        notified_socket.close()
