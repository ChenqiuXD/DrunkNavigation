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
    def __init__(self, VISION_PORT=23333, SENDERIP='0.0.0.0', sim=True):
        self.sim = sim
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM,
                                  socket.IPPROTO_UDP)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind((SENDERIP, VISION_PORT))
        self.sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP,
                             socket.inet_aton(MULTI_GROUP) +
                             socket.inet_aton(SENDERIP))
        self.robot_info = -100*np.ones(6)
        # self.ball_info = [-100, -100]
        # self.other_robots = []  # All the defending robots

    def receive(self):
        data, addr = self.sock.recvfrom(65535)
        # sleep(0.001)  # wait for reading
        return data

    def get_info(self, robot_id):
        """
        get the info of our robot and the info of all the robots available in the pitch
        :return: None
        """
        data = self.receive()

        detection = detection_pb.Vision_DetectionFrame()
        detection.ParseFromString(data)

        blue_robots = detection.robots_blue
        yellow_robots = detection.robots_yellow
        self.other_robots = []
        for i in range(len(blue_robots)):
            if blue_robots[i].robot_id == robot_id:
                self.robot_info = np.array([blue_robots[i].x/1000,
                                            blue_robots[i].y/1000,
                                            blue_robots[i].orientation,
                                           blue_robots[i].vel_x/1000,
                                           blue_robots[i].vel_y/1000,
                                           blue_robots[i].rotate_vel])
            else:
                self.other_robots.append([blue_robots[i].x/1000, blue_robots[
                    i].y/1000, blue_robots[i].orientation])
        for i in range(len(yellow_robots)):
            self.other_robots.append([yellow_robots[i].x / 1000, yellow_robots[
                i].y / 1000, yellow_robots[i].orientation])

        self.other_robots = np.array(self.other_robots)
        if not self.sim:
            self.robot_info[3] /= 10
            self.robot_info[4] /= 10

        # self.other_robots = np.array([[robot.x/1000, robot.y/1000,
        #                   robot.orientation] for robot in robots])
        # self.other_robots = np.delete(self.other_robots, 0, axis=0)


if __name__ == '__main__':
    print("Hello world!")
