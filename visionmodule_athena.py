# -*- coding: utf-8 -*-
"""
@Brief: This is a vision module(single robot) for RoboCup Small Size League
@Version: grSim 4 camera version
@author: Wang Yunkai, altered by rkterence@zju.edu.cn
"""

import socket
from time import sleep
import proto.vision_detection_pb2 as detection_pb
import numpy as np

MULTI_GROUP = '224.5.23.2'


class VisionModule:
    def __init__(self, VISION_PORT=23333, SENDERIP='0.0.0.0'):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM,
                                  socket.IPPROTO_UDP)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind((SENDERIP, VISION_PORT))
        self.sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP,
                             socket.inet_aton(MULTI_GROUP) +
                             socket.inet_aton(SENDERIP))
        self.robot_info = -100*np.ones(6)
        # self.ball_info = [-100, -100]
        self.other_robots = -100*np.ones([7, 3])  # All the defending robots

    def receive(self):
        data, addr = self.sock.recvfrom(65535)
        # sleep(0.001)  # wait for reading
        return data

    def get_info(self):
        """
        get the info of our robot and the info of all the robots available in the pitch
        :return: None
        """
        data = self.receive()

        detection = detection_pb.Vision_DetectionFrame()
        detection.ParseFromString(data)

        robot = detection.robots_blue[0]  # repeat object. Our robot is set to be No.0 robot
        self.robot_info = np.array([robot.x/1000, robot.y/1000, robot.orientation,
                                    robot.vel_x/1000, robot.vel_y/1000, robot.rotate_vel])

        robots = detection.robots_blue
        self.other_robots = np.array([[robot.x/1000, robot.y/1000, robot.orientation] for robot in \
                robots])
        self.other_robots = np.delete(self.other_robots, 0, axis=0)


if __name__ == '__main__':
    print("Hello world!")
