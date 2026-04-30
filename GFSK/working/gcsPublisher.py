import socket


class GCSPublisher:
    def __init__(self, addr):
        
        self.addr = addr
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)  # needed for broadcast


    def publish(self, data: bytes):
        self.sock.sendto(data, self.addr)
