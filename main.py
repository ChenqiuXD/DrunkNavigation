import threading
import numpy as np
import math
from visionmodule_athena import VisionModule
from actionmodule import ActionModule
from debugmodule import DebugModule
import dwa
import time
from Fast_Planning import Functions as func
from Fast_Planning import Classes as cls
from Fast_Planning.run_viewer import run_viewer

# global configurations
ROBOT_ID = 1
SIM = True
GOAL = np.array([5.0, 0])
RUNVIEWER_SIZE = (600, 900)
ROBOT_RADIUS = 0.1


class GetVision(threading.Thread):
    def __init__(self, thread_id, name, counter):
        threading.Thread.__init__(self)
        self.threadID = thread_id
        self.name = name
        self.counter = counter

    def run(self):
        while thread_running_flag:
            vision.get_info(ROBOT_ID)
            global ob
            ob = vision.other_robots[:, :-1]


class SendAction(threading.Thread):
    def __init__(self, thread_id, name, counter):
        threading.Thread.__init__(self)
        self.threadID = thread_id
        self.name = name
        self.counter = counter

    def run(self):
        while thread_running_flag:
            if SIM:
                sender.send_action(ROBOT_ID, u[0], u[1], u[2])
            else:
                sender.send_action_real(robot_id=ROBOT_ID+1, vx=800*u[0], vy=800*u[1],
                                        vw=40*u[2])
                # sender.send_action_real(robot_id=ROBOT_ID+1, vx=5, vy=5, vw=0)
                print("send success")
                time.sleep(0.2)  # 16ms of delay.


# def timer_read_info():
#     """
#     This function uses another thread in python to read the output of GrSim.
#     The frequency is 50Hz, a.k.a. read once every 20ms.
#     Through test, this function costs approximately 2ms to be executed,
#     and has only small influence on the main function.
#
#     UPDATE: This function now is deprecated for the horrible delay.
#     :return: None
#     """
#     # start = time.clock()  # This is for debug. We will see how much time will
#     # pass for our python module to read a set of data from GrSim.
#     vision.get_info(robot_id)
#     global ob, pos
#     ob = vision.other_robots[:, :-1]  # Dessert the orientation of other robots
#     pos = vision.robot_info
#     # Restart the timer
#     global timer
#     timer = threading.Timer(0.02, timer_read_info)
#     timer.start()
#     # print("Elapsed Time: "+str(time.clock()-start))


if __name__ == "__main__":
    vision = VisionModule(sim=SIM)
    sender = ActionModule(sim=SIM)
    debug = DebugModule()
    # sender.reset(robot_num=1)

    # Initialize the initial situation of our robot and other robots:
    vision.get_info(ROBOT_ID)
    goal = GOAL
    ob = vision.other_robots[:, :-1]  # Dessert the orientation of other robots
    min_u = np.zeros(3)
    config = dwa.Config()
    traj = np.array(vision.robot_info)
    path = None  # For now, there is no global path
    u = np.array([0, 0, 0])
    count = 0  # This will be a flag in the running loop

    # Initialize the debug module
    debug.dwa_msg_init()

    # Initialize the serial connector
    # serial_connector = Ser()

    # define the start the other 2 threads
    thread_running_flag = True  # while True, the two threads below will be running all the time
    thread_get_vision = GetVision(1, "GetVisionInfo", 1)
    thread_get_vision.start()
    thread_send_action = SendAction(2, "SendVelocity", 2)
    thread_send_action.start()

    # Get into the running loop
    while True:
        # print("ROBOT:", vision.robot_info)
        # print("Other Robots:")
        # print(vision.other_robots)
        u, ltraj, cost = dwa.dwa_control(vision.robot_info, min_u, config, goal, path, ob)
        # sender.send_action(1, u[0], u[1], u[2])
        traj = np.vstack((traj, vision.robot_info))  # store state history

        # Initialize the parameter of the fast path planning
        trajectory = cls.Trajectory([cls.Point(vision.robot_info[0], vision.robot_info[1]), cls.Point(goal[0], goal[1])])
        obstacles = []
        for i in np.arange(len(vision.other_robots)):
            obstacles.append(cls.Obstacle(vision.other_robots[i][0], vision.other_robots[i][1], ROBOT_RADIUS))
        environment = cls.Environment(obstacles, 1000,1000, ROBOT_RADIUS)
        trajectory, unused = func.FastPathPlanning(environment, trajectory, 0)
        # run_viewer(trajectory, environment, RUNVIEWER_SIZE[0]/GOAL[0])

        # display debug msg
        debug.dwa_msgs[0].text.text = "POS: " + str(vision.robot_info.round(3))
        debug.dwa_msgs[1].text.text = "u: " + str(np.array(u).round(3))
        debug.dwa_msgs[2].text.text = "Final Cost: " + str(round(cost,3))
        debug.dwa_msgs[3].text.text = "Round: " + str(count+1)
        debug.dwa_msgs[4].line.start.x = int(100*vision.robot_info[0])
        debug.dwa_msgs[4].line.start.y = int(-100*vision.robot_info[1])
        debug.dwa_msgs[4].line.end.x = int(100*ltraj[-1, 0])
        debug.dwa_msgs[4].line.end.y = int(-100*ltraj[-1, 1])
        for i in np.arange(len(trajectory.points)+5):
            debug.dwa_msgs[i].line.start.x = trajectory.points[i-5].x
            debug.dwa_msgs[i].line.start.y = trajectory.points[i-5].y
            debug.dwa_msgs[i].line.end.x = trajectory.points[i-5].x
            debug.dwa_msgs[i].line.end.y = trajectory.points[i-5].y
        debug.send_msg(debug.messages)

        # check goal
        if count % 2 == 0 and math.sqrt((vision.robot_info[0] - goal[0]) ** 2 + (
                vision.robot_info[1]
                                                                            - goal[1])
                                   ** 2) <= config.robot_radius:
            count += 1
            goal = np.array([0, 0])

        if count % 2 == 1 and math.sqrt((vision.robot_info[0] - goal[0]) ** 2 + (
                vision.robot_info[1]
                                                                             - goal[1]) ** 2) <= \
                config.robot_radius:
            # thread_running_flag = False
            count += 1
            goal = GOAL
            # break

    print ("Finished")
