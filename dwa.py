"""
DWA implementation in python.
Author: rkterence@zju.edu.cn
Reference:
"""

import math
import numpy as np
# import matplotlib.pyplot as plt

show_animation = True


class Config():
    # parameters
    def __init__(self):
        # robot parameter
        self.max_speed = 1.0  # [m/s]
        self.min_speed = -1.0  # [m/s]
        self.max_yawrate = 40.0 * math.pi / 180.0  # [rad/s]
        self.max_accel = 2  # [m/ss]
        self.max_dyawrate = 40 * math.pi / 180.0  # [rad/ss]
        self.v_reso = 0.1  # [m/s]
        self.yawrate_reso = math.pi / 180.0  # [rad/s]
        self.dt = 0.1  # [s]
        self.predict_time = 1.5  # [s]
        self.to_goal_cost_gain = 1.0
        self.speed_cost_gain = 0.5
        self.to_path_cost_gain = 1.0  # [m]
        self.to_obstacle_cost_gain = 0.4
        self.robot_radius = 0.1  # [m]


def cal_next_point(pos, u, dt):
    """

    :param pos: [x, y, yaw, vx, vy, vw]
    :param u: [vx, vy, vw]
    :param dt: Time interval
    :return: The new point which has the same format as input x
    """
    pos[2] += u[2] * dt
    pos[0] += u[0] * math.cos(pos[2]) * dt - u[1] * math.sin(pos[2]) * dt
    pos[1] += u[0] * math.sin(pos[2]) * dt + u[1] * math.cos(pos[2]) * dt
    # vx, vy, and vw are the same as u
    pos[3] = u[0]
    pos[4] = u[1]
    pos[5] = u[2]
    return pos


def calc_dynamic_window(pos, config):
    """

    :param pos: the status of a robot
    :param config: The Config class defined above.
    :return: dw: dynamic window.
    """
    # Dynamic window from robot specification
    robot_limits = [config.min_speed, config.max_speed,
          -config.max_yawrate, config.max_yawrate]

    # Dynamic window from motion model
    dynamic_limits = [pos[3] - config.max_accel * config.dt, pos[3] +
                      config.max_accel * config.dt, pos[4] -
                      config.max_accel * config.dt, pos[4] + config.max_accel
                      * config.dt, pos[5] - config.max_dyawrate * config.dt,
                      pos[5] + config.max_dyawrate * config.dt]

    #  [vxmin, vxmax, vymin, vymax, yawrate min, yawrate max]
    dw = [max(robot_limits[0], dynamic_limits[0]), min(robot_limits[1], dynamic_limits[1]), max(robot_limits[0], dynamic_limits[2]),
          min(robot_limits[1], dynamic_limits[3]), max(robot_limits[2], dynamic_limits[4]), min(robot_limits[3], dynamic_limits[5])]

    return dw


def calc_trajectory(pos_init, vx, vy, vw, config):
    """

    :param pos_init: Start status of the robot
    :param ...
    :return: series of status of robot.
    """
    pos = np.array(pos_init)
    traj = np.array(pos)
    time = 0
    while time <= config.predict_time:
        pos = cal_next_point(pos, [vx, vy, vw], config.dt)
        traj = np.vstack((traj, pos))
        time += config.dt

    return traj


def calc_final_input(pos, u, dw, config, goal, path, ob):
    """
    Find the optimum control signal through iteration.
    :param pos: current status of the robot
    :param u: min_u
    :param dw: dynamic window
    :param config: Config class
    :param goal: goal position
    :param ob: obstacles
    :return: min_u and the best trajectory
    """
    pos_init = pos[:]
    min_cost = float('inf')
    min_u = u
    min_u[0] = min_u[1] = 0.0
    best_traj = np.array([pos])

    # evalucate all trajectory with sampled input in dynamic window
    for vx in np.arange(dw[0], dw[1], config.v_reso):
        for vy in np.arange(dw[2], dw[3], config.v_reso):
            for vw in np.arange(dw[4], dw[5], config.yawrate_reso):
                traj = calc_trajectory(pos_init, vx, vy, vw, config)

                # calc cost
                to_goal_cost = calc_to_goal_cost(traj, goal, config)
                speed_cost = config.speed_cost_gain * \
                    (config.max_speed - (math.fabs(traj[-1, 3]) + math.fabs(traj[-1, 4]))/2)
                ob_cost = calc_obstacle_cost(traj, ob, config)
                # to_path_cost = calc_to_path_cost(traj, path, config)

                # final_cost = to_goal_cost + ob_cost + speed_cost + to_path_cost
                final_cost = to_goal_cost + ob_cost + speed_cost
                # search minimum trajectory
                if min_cost >= final_cost:
                    min_cost = final_cost
                    min_u = [vx, vy, vw]
                    best_traj = traj

    return min_u, best_traj, min_cost


def calc_obstacle_cost(traj, ob, config):
    # calc obstacle cost inf: collistion, 0:free

    skip_n = 2
    minr = float("inf")

    for ii in range(0, len(traj[:, 1]), skip_n):
        for i in range(len(ob[:, 0])):
            ox = ob[i, 0]
            oy = ob[i, 1]
            dx = traj[ii, 0] - ox
            dy = traj[ii, 1] - oy

            r = math.sqrt(dx**2 + dy**2)
            if r <= config.robot_radius:
                return float("Inf")  # collision

            if minr >= r:
                minr = r

    return 1.0 / minr * config.to_obstacle_cost_gain


def calc_to_goal_cost(traj, goal, config):
    # calc to goal cost. It is 2D norm.

    # goal_magnitude = math.sqrt(goal[0]**2 + goal[1]**2)
    # traj_magnitude = math.sqrt(traj[-1, 0]**2 + traj[-1, 1]**2)
    # dot_product = (goal[0]*traj[-1, 0]) + (goal[1]*traj[-1, 1])
    # error = dot_product / (goal_magnitude*traj_magnitude)
    # error = int(error) if math.fabs(error) > 1 else error
    # error_angle = math.acos(error)
    # cost = config.to_goal_cost_gain * error_angle
    straight_line = np.array([goal[0] - traj[0, 0], goal[1] - traj[0, 1]])
    velocity_line = np.array([traj[0, 3] * math.cos(traj[0, 2]) - traj[0, 4] * math.sin(traj[0,
                                                                                             2]),
                              traj[0, 3] * math.sin(traj[0, 2]) + traj[0, 4] * math.cos(traj[0,
                                                                                             2])])

    direction_cost = np.dot(straight_line, velocity_line) / (np.linalg.norm(straight_line) *
                                                             np.linalg.norm(velocity_line))

    cost = config.to_goal_cost_gain * (math.sqrt((traj[-1, 0] - goal[0])**2 + (traj[-1,
                                                                                    1] - goal[1])
                                                 ** 2) + direction_cost)

    return cost


def calc_to_path_cost(traj, path, config):
    """
    This function can calculate the cost to the global path.
    This is similar to the to_obstacle cost. But in all cost, this is weighted
    more.
    :param traj: ndarray. The trajectory calculated from the dynamic window.
    :param path: the global path calculated by upper navigation algorithms,
    e.g. A*.
    :param config: The configurations for this dwa function
    :return: The cost.
    """
    return config.to_path_cost_gain * cal_to_path_dist(traj, path)


def cal_to_path_dist(traj, path):
    dist_min = float("inf")
    for i in range(len(path)):
        dist = math.sqrt((traj[-1, 0]-path[i].x)**2+(traj[-1, 1]-path[i].y)**2)
        if dist <= dist_min:
            dist_min = dist

    return dist_min


def dwa_control(pos, u, config, goal, path, ob):
    """
    The interface of this module. For a current robot status and other necessary
    info, it will compute the necessary robot velocity input (vx, vy, vw).
    :param pos: the current
    :param u: the minimum u (vx, vy, vw)
    :param config: Config class
    :param goal: goal pos
    :param ob: obstacles
    :param path: the global path
    :return:
    """

    dw = calc_dynamic_window(pos, config)

    u, traj, cost = calc_final_input(pos, u, dw, config, goal, path, ob)

    return u, traj, cost


# def plot_arrow(x, y, yaw, length=0.5, width=0.1):
#     plt.arrow(x, y, length * math.cos(yaw), length * math.sin(yaw),
#               head_length=width, head_width=width)
#     plt.plot(x, y)


def main():
    # initial state [x(m), y(m), yaw(rad), v(m/s), omega(rad/s)]
    pos = np.array([0.0, 0.0, 0, 0.0, 0.0, 0.0])
    # goal position [x(m), y(m)]
    goal = np.array([10, 10])
    # obstacles [x(m) y(m), ....]
    ob = np.matrix([[-2, -2],
                    [0, 2],
                    [4.0, 2.0],
                    [5.0, 4.0],
                    [5.0, 5.0],
                    [5.0, 6.0],
                    [5.0, 9.0],
                    [8.0, 9.0],
                    [7.0, 9.0],
                    [12.0, 12.0]
                    ])

    u = np.array([0.0, 0.0, 0.0])  # u = (vx, vy, vw)
    config = Config()
    traj = np.array(pos)
    path = np.array([[0,0],[2.5,0],[5,0],[7.5,2],[10,4],[10,6],[10,8],[10,10]])
    for i in range(1000):
        u, ltraj, cost = dwa_control(pos, u, config, goal, path, ob)

        pos = cal_next_point(pos, u, config.dt)
        traj = np.vstack((traj, pos))  # store state history

        # if show_animation:
        #     plt.cla()
        #     plt.plot(path[:, 0], path[:, 1])
        #     plt.plot(ltraj[:, 0], ltraj[:, 1], "-g")
        #     plt.text(ltraj[-1, 0], ltraj[-1, 1]+0.5, str(round(cost, 2)),
        #              color='red')
        #     plt.text(ltraj[-1, 0], ltraj[-1, 1]+1, str([round(i, 2) for i in
        #                                                 u]),
        #              color='blue')
        #     plt.plot(pos[0], pos[1], "xr")
        #     plt.plot(goal[0], goal[1], "xb")
        #     plt.plot(ob[:, 0], ob[:, 1], "ok")
        #     plot_arrow(pos[0], pos[1], pos[2])
        #     plt.axis("equal")
        #     plt.grid(True)
        #     plt.pause(0.01)

        # check goal
        if math.sqrt((pos[0] - goal[0])**2 + (pos[1] - goal[1])**2) <= \
                config.robot_radius:
            print("Goal!!")
            break

    print("Done")
    # if show_animation:
    #     plt.plot(traj[:, 0], traj[:, 1], "-r")
    #     plt.show()


if __name__ == '__main__':
    main()
