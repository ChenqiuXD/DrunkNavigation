# -*- coding: utf-8 -*-
"""
@Brief: This is a test run script
@Version: Test Run 2.0.0
@author: Wang Yunkai
"""

import numpy as np
from visionmodule_grsim import VisionModule
from actionmodule import ActionModule

LOG = False

VISION_PORT = 10076
ACTION_IP = '127.0.0.1'
ACTION_PORT = 20011
ROBOT_ID = 6

# max velocity
max_v = 3.0
# max angle
max_w = 2*np.pi
# max acceleration
robot_max_acc = 3
robot_max_w_acc = 2*np.pi
# Max brake distance
l_max = (max_v)*(max_v)/(2*robot_max_acc)
dt = 1.0 / 60
max_step_acc = robot_max_acc * dt
#center2ball = 0.09
        
def angle_normalize(angle):
        if angle > np.pi:
            angle -= 2*np.pi
        elif angle < -np.pi:
            angle += 2*np.pi
        return angle

'''
0   robot2ball_dist
1   sin(delt_theta)
2   cos(delt_theta)
3   v_t
4   v_n
5   sin(robot2ball_theta)
6   cos(robot2ball_theta)
'''
def target_policy(state):
    theta = np.arctan2(state[1], state[2])

    if state[0] > l_max:
        v_t = max_v
    else:
        v_t = np.sqrt(2 * robot_max_acc * state[0])
        
    if state[4] > 0:
        v_n = state[4] - max_step_acc
        v_n = np.clip(v_n, 0, max_v)
    else:
        v_n = state[4] + max_step_acc
        v_n = np.clip(v_n, -max_v, 0)
        
    if theta > 0:
        # 计算角速度
        w = np.sqrt(2 * robot_max_w_acc * theta)
    else:
        w = -np.sqrt(-2 * robot_max_w_acc * theta)
        
    """ mode 2: turn first
    if abs(theta) > np.pi/2:
        v_t = 0 
        v_t = 0
    else:
        v_t = v_t * np.exp(-0.75*(abs(theta)/(np.pi/6)))
        v_n = v_n * np.exp(-0.75*(abs(theta)/(np.pi/6)))
    """  
    # 前面是以球为目标做速度规划，后面将其返回到全局坐标
    vx = v_t*state[2] - v_n*state[1]
    vy = v_t*state[1] + v_n*state[2]
    return [vx, vy, w]


def run_loop(vision, sender):
    action = [0, 0, 0]
    sender.reset(robot_num=6)
    
    i = 0
    while i < 1e7:
        i += 1
        if i % 200 == 0:
            sender.reset_ball()
            
        vision.get_info(ROBOT_ID)

        state = []
        robot2ball_dist = np.sqrt((vision.robot_info[0]-vision.ball_info[0])**2 + (vision.robot_info[1]-vision.ball_info[1])**2)
        state.append(robot2ball_dist) # 0
        robot2ball_theta = np.arctan2((vision.ball_info[1] - vision.robot_info[1]), (vision.ball_info[0] - vision.robot_info[0]))
        delt_theta = robot2ball_theta - vision.robot_info[2]
        delt_theta = angle_normalize(delt_theta)
        state.append(np.sin(delt_theta)) # 1
        state.append(np.cos(delt_theta)) # 2
        v_t = vision.robot_info[3]*np.cos(robot2ball_theta) + vision.robot_info[4]*np.sin(robot2ball_theta)
        v_n = -vision.robot_info[3]*np.sin(robot2ball_theta) + vision.robot_info[4]*np.cos(robot2ball_theta)
        state.append(v_t) # 3
        state.append(v_n) # 4
        state.append(np.sin(robot2ball_theta)) # 5
        state.append(np.cos(robot2ball_theta)) # 6
        
        action = target_policy(state)
        if LOG:
            file.write(str(state)[1:-1] + ', ' + str(action)[1:-1] + '\n')
        sender.send_action(ROBOT_ID, vx=action[0], vy=action[1], w=action[2])


if __name__ == '__main__':
    vision = VisionModule(VISION_PORT)
    sender = ActionModule(ACTION_IP, ACTION_PORT)
    
    if LOG:
        file = open('data.txt', 'w')
    
    run_loop(vision, sender)
    
    if LOG:
        file.close()
