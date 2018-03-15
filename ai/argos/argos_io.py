import socket

in_port = 6030
out_port = 5030
max_msg_size = 4096
ip_address = '127.0.0.1'


class ArgosIO:
    def __init__(self, num_robots=1, verbose=False):
        self.num_robots = num_robots
        self.in_socket = []
        self.out_socket = []
        self.verbose = verbose
        self.setup()

    def setup(self):
        print("Robots: ", self.num_robots)
        for i in range(self.num_robots):
            self.out_socket.append(socket.socket(socket.AF_INET, socket.SOCK_DGRAM))
            self.in_socket.append(socket.socket(socket.AF_INET, socket.SOCK_DGRAM))
            self.in_socket[-1].bind((ip_address, in_port + i))

    def send_to(self, message, robot_id=0):
        self.out_socket[robot_id].sendto(message.encode(), (ip_address, out_port + robot_id))
        if self.verbose:
            print("MSG out P --> A[", robot_id, "]: ", message)

    def receive_from(self, robot_id=0):
        message, addr = self.in_socket[robot_id].recvfrom(max_msg_size)
        message = message.decode('utf-8')
        if self.verbose:
            print("MSG in P <-- A[", robot_id, "]: ", message)
        return message