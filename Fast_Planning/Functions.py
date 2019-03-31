# Importing area
from Classes import *
import math
import sys

# Constant Variable definition
MAX_RECURSIVE_DEPTH = 100
subgoalAdd = Trajectory([Point(0,0)])

# Function Definition Area
def SearchPoint(point1, point2, obstacle, environment, destination):
    """This function return the achievable subgoal"""
    global subgoalAdd

    MAX_ITER = 100
    iterCount = 0
    deltalength = 5
    # calculate the slope of the line
    if point1.x == point2.x:
        slope = math.inf
    else:
        slope = (point2.y-point1.y)/(point2.x-point1.x)

    # search through the obstacle's point goes -1/tangent times the radius of the deltaLen
    deltaLen = obstacle.radius + environment.robotRadius + deltalength
    for i in np.arange(MAX_ITER):
        if slope == math.inf:
            subgoal1 = Point(obstacle.pos.x-(i+1)*deltaLen, obstacle.pos.y)
            subgoal2 = Point(obstacle.pos.x+(i+1)*deltaLen, obstacle.pos.y)
        elif slope == 0:
            subgoal1 = Point(obstacle.pos.x, obstacle.pos.y-(i+1)*deltaLen)
            subgoal2 = Point(obstacle.pos.x, obstacle.pos.y+(i+1)*deltaLen)
        else:
            deltaY = 1/math.sqrt(slope**2+1)*deltaLen*(i+1)
                
            subgoal1 = Point(obstacle.pos.x-slope*deltaY, obstacle.pos.y+deltaY)
            subgoal2 = Point(obstacle.pos.x+slope*deltaY, obstacle.pos.y-deltaY)

        # iteration ending condition
        isCollideSubgoal1 = 0
        isCollideSubgoal2 = 0
        for obstacle in environment.obstacles:
            if isCollideSubgoal1 == 0:
                if subgoal1.dist_point_to_point(obstacle.pos) < obstacle.radius + environment.robotRadius:
                    isCollideSubgoal1 = 1
            if isCollideSubgoal2 == 0:
                if subgoal2.dist_point_to_point(obstacle.pos) < obstacle.radius + environment.robotRadius:
                    isCollideSubgoal2 = 1

            if (isCollideSubgoal1 == 1 and isCollideSubgoal2 == 1) or iterCount >= MAX_ITER:
                break
        # if one subgoal is detected no collision, then break out of the for loop
        if isCollideSubgoal1 == 0 or isCollideSubgoal2 == 0:
            break

    # return the plausible subgoal
    if iterCount >= MAX_ITER:
        print("no plausible subgoal found in the area")
        subgoal = 0
    elif isCollideSubgoal2 == 0 and isCollideSubgoal1 == 0:
        distSub1 = subgoal1.dist_point_to_point(destination)
        distSub2 = subgoal2.dist_point_to_point(destination)
        if distSub1 > distSub2:
            subgoal = subgoal2
        else:
            subgoal = subgoal1
    elif isCollideSubgoal1 == 0:
        subgoal = subgoal1
    else:
        subgoal = subgoal2

    subgoalAdd.points.append(subgoal)

    return subgoal


def FastPathPlanning(environment, trajectory, depth):
    """This function would be executed recursively, """
    global subgoalAdd

    [isCollide, point1, point2, obstacle] = trajectory.collide(environment)
    destination = trajectory.points[-1]

    if isCollide != 0 and depth <= MAX_RECURSIVE_DEPTH :
        subgoal = SearchPoint(point1, point2, obstacle, environment, destination)
        print("sugboal generated-- %f, %f, between Point1: %f, %f, and Point2: %f, %f" % (subgoal.x, subgoal.y, point1.x, point1.y, point2.x, point2.y))

        if subgoal == 0:
            print("No plausible trajectory found, system exit with 1")
            sys.exit(1)

        # the following two line divide the trajectory points into two trajectory
        points1 = trajectory.points[:trajectory.points.index(point1)+1] + [subgoal]
        trajectory1, unused = FastPathPlanning(environment, Trajectory(points1), depth+1)
        points2 = [subgoal] + trajectory.points[trajectory.points.index(point2):]
        trajectory2, unused = FastPathPlanning(environment, Trajectory(points2), depth+1)

        # The following line merge two trajectory into one traj
        trajectory = trajectory1.trajMerge(trajectory2)
    return trajectory, subgoalAdd


if __name__ == '__main__':
    # env = Environment([Obstacle(10, 10, 5), Obstacle(25, 25, 5)], 40, 40, 2)
    # traj = Trajectory([Point(0, 0), Point(10, 10), Point(25, 25)])
    # FastPathPlanning(env, traj, 0)
    a = Point(0,0)
    print(a)
