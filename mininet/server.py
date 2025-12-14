# -*- coding: utf-8 -*-
# -*- coding: utf-8 -*-
import socket
import select
import errno
import time

HOST = '0.0.0.0'
PORT = 5001

def main():
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server_socket.bind((HOST, PORT))
    server_socket.listen(10)
    server_socket.setblocking(False)  # 非阻塞
    print("Server start：%s:%s" % (HOST, PORT))

    sockets_list = [server_socket]  # 監控的 socket 清單
    clients = {}  # 儲存 client socket 對應 addr

    while True:
        # 使用 select 監控 sockets 可讀
        read_sockets, _, exception_sockets = select.select(sockets_list, [], sockets_list, 1)

        for notified_socket in read_sockets:
            if notified_socket == server_socket:
                # 有新的連線進來
                client_socket, client_address = server_socket.accept()
                client_socket.setblocking(False)
                sockets_list.append(client_socket)
                clients[client_socket] = client_address
                print("New connection from:", client_address)
            else:
                # 來自 client 的資料
                try:
                    data = notified_socket.recv(4096)
                    if not data:
                        # 客戶端關閉連線
                        print("%s disconnect" % clients[notified_socket])
                        sockets_list.remove(notified_socket)
                        del clients[notified_socket]
                        notified_socket.close()
                        continue
                    # 你可以在這裡處理收到的資料
                    #print("Received from %s: %s" % (clients[notified_socket], data.decode(errors='ignore')))
                except socket.error as e:
                    if e.errno == errno.EWOULDBLOCK or e.errno == errno.EAGAIN:
                        continue
                    else:
                        print("[ERROR] from %s connection error：%s" % (clients[notified_socket], e))
                        sockets_list.remove(notified_socket)
                        del clients[notified_socket]
                        notified_socket.close()

        for notified_socket in exception_sockets:
            # 發生錯誤的 socket
            print("Exception from %s" % clients[notified_socket])
            sockets_list.remove(notified_socket)
            del clients[notified_socket]
            notified_socket.close()

if __name__ == '__main__':
    main()
