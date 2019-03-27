import threading
import numpy as np
import math
from visionmodule_athena import VisionModule
from actionmodule import ActionModule
from debugmodule import DebugModule
import dwa

# ROBOT_ID = 6  # The specific robot that we control
VISION_PORT = 23333  # The port to read Vision messages

class MyThread(threading.Thread):
    def __init__(self, threadID, name, counter):
        threading.Thread.__init__(self)
        self.threadID = threadID
        self.name = name
        self.counter = counter

    def run(self):  # 把要执行的代码写到run函数里面 线程在创建后会直接运行run函数
        while True:
            vision.get_info()
            global ob
            ob = vision.other_robots[:, :-1]  # Dessert the orientation of other robots


class Config():
    # parameters
    def __init__(self):
        # robot parameter
        self.max_speed = 1.0  # [m/s]
        self.min_speed = -1.0  # [m/s]
        self.max_yawrate = 40.0 * math.pi / 180.0  # [rad/s]
        self.max_accel = 0.2  # [m/ss]
        self.max_dyawrate = 40.0 * math.pi / 180.0  # [rad/ss]
        self.v_reso = 0.01  # [m/s]
        self.yawrate_reso = math.pi / 180.0  # [rad/s]
        self.dt = 0.3  # [s]
        self.predict_time = 9  # [s]
        self.to_goal_cost_gain = 1.0
        self.speed_cost_gain = 1.0
        self.robot_radius = 0.2  # [m]
        self.to_path_cost_gain = 1.0  # [m]


def thread_read_info():
    """
    This function uses another thread in python to read the output of GrSim.
    The frequency is 50Hz, a.k.a. read once every 20ms.
    Through test, this function costs approximately 2ms to be executed,
    and has only small influence on the main function.
    :return: None
    """
    # start = time.clock()  # This is for debug. We will see how much time will
    # pass for our python module to read a set of data from GrSim.
    vision.get_info()
    global ob
    ob = vision.other_robots[:, :-1]  # Dessert the orientation of other robots
    # Restart the timer
    # print("Elapsed Time: "+str(time.clock()-start))


if __name__ == "__main__":
    vision = VisionModule(VISION_PORT)
    sender = ActionModule('127.0.0.1', 20011)
    debug = DebugModule()
    sender.reset(robot_num=0)

    # Initialize the initial situation of our robot and other robots:
    vision.get_info()
    pos = vision.robot_info
    goal = np.array([4.8, 0])
    ob = vision.other_robots[:, :-1]  # Dessert the orientation of other robots
    min_u = np.zeros(3)
    config = Config()
    traj = np.array(pos)
    path = None  # For now, there is no global path

    count = 0  # This will be a flag in the running loop

    # Initialize the debug module
    debug.dwa_msg_init()
    # Start the timer:
    my_thread = MyThread(1, "get_info", 1)
    my_thread.start()

    # Get into the running loop
    while True:
        u, ltraj, cost = dwa.dwa_control(vision.robot_info, min_u, config, goal, path, ob)
        sender.send_action(0, u[0], u[1], u[2])
        traj = np.vstack((traj, pos))  # store state history

        # display debug msg
        debug.dwa_msgs[0].text.text = "POS: " + str(vision.robot_info)
        debug.dwa_msgs[1].text.text = "u: " + str(u)
        debug.dwa_msgs[2].text.text = "Final Cost: " + str(cost)
        debug.dwa_msgs[3].text.text = "Round" + str(count+1)
        debug.dwa_msgs[4].line.start.x = int(100*vision.robot_info[0])
        debug.dwa_msgs[4].line.start.y = int(-100*vision.robot_info[1])
        debug.dwa_msgs[4].line.end.x = int(100*ltraj[-1, 0])
        debug.dwa_msgs[4].line.end.y = int(-100*ltraj[-1, 1])
        debug.send_msg(debug.messages)

        # check goal
        if count < 1 and math.sqrt((pos[0] - goal[0]) ** 2 + (pos[1] - goal[1]) ** 2) <= \
                config.robot_radius:
            count += 1
            goal = np.array([0, 0])

        if count >= 1 and math.sqrt((pos[0] - goal[0]) ** 2 + (pos[1] - goal[1]) ** 2):
            break

        # time.sleep(0.1)  # update control signal every 100 mille seconds
