import socket
import struct

MCAST_GRP = '224.0.0.5'
MCAST_PORT = 5000
IS_ALL_GROUPS = True

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
if IS_ALL_GROUPS:
    sock.bind(('', MCAST_PORT))
else:
    sock.bind((MCAST_GRP, MCAST_PORT))

mreq = struct.pack("4sl", socket.inet_aton(MCAST_GRP), socket.INADDR_ANY)
sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)

while True:
  # For Python 3, change next line to "print(sock.recv(10240))"
  data = sock.recv(2048)
  print('start : {}, Product line : {}, ID : {}'.format(hex(data[0]), hex(data[1]), hex(data[2])))
  print(bin(data[7]))




