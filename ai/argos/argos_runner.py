import subprocess
import threading
import psutil
from time import sleep
import socket

argos_path = '/usr/local/bin/argos3'
argos_experiment_path = '../argos/footbot-ai.argos'

in_port = 6030
out_port = 5030
ip_address = '127.0.0.1'

class Argos:
    def __init__(self, num_robots=1):
        self.argos_process = None
        self.num_robots = num_robots
        self.in_socket = []
        self.out_socket = []
        self.setup()

    def setup(self):
        print("Robots: ", self.num_robots)
        for i in range(self.num_robots):
            self.out_socket.append(socket.socket(socket.AF_INET, socket.SOCK_DGRAM))
            self.in_socket.append(socket.socket(socket.AF_INET, socket.SOCK_DGRAM))
            self.in_socket[-1].bind((ip_address, in_port + i))

        self.argos_process = subprocess.Popen([argos_path, '-c', argos_experiment_path], stdout=subprocess.PIPE)
        threading.Thread(target=self.start(), daemon=True).start()

    def start(self, robot_id=0):
        while not self.argos_process.poll():
            # send message
            self.out_socket[robot_id].sendto(("From Python").encode(), (ip_address, 5030+robot_id))
            print("Message sent...")
            #print("Memory usage: ", self.memory_usage(self.argos_process.pid) / 1024**3)

            sleep(.1)
            # get message
            for i in range(self.num_robots):
                print("size: ", len(self.in_socket), ", ", len(self.out_socket))
                data, addr = self.in_socket[i].recvfrom(4096)
                print(i, " Received: ", data.decode('utf-8'))



    def memory_usage(self, pid):
        proc = psutil.Process(pid)
        mem = proc.memory_info().rss  # resident memory
        for child in proc.children(recursive=True):
            try:
                mem += child.memory_info().rss
            except psutil.NoSuchProcess:
                pass
        return mem
