# Importing area
from Fast_Planning.Classes import *
import math
import sys

# Constant Variable definition
MAX_RECURSIVE_DEPTH = 20
MAX_ITER = 20
IGNOREDIST = 1.5
# subgoalAdd = Trajectory([Point(0,0)])


# Function Definition Area
def SearchPoint(point1, point2, obstacle, environment):
    """This function return the achievable subgoal"""
    # global subgoalAdd

    iterCount = 0
    # calculate the slope of the line
    if point1.x == point2.x:
        slope = math.inf
    else:
        slope = (point2.y-point1.y)/(point2.x-point1.x)

    # search through the obstacle's point goes -1/tangent times the radius of the deltaLen
    deltaLen = obstacle.radius + environment.robotRadius
    deltaLen *= 1.5
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
        obstacleDist1 = 0
        obstacleDist2 = 0
        for other_obstacle in environment.obstacles:
            # The obstacle itself's distance is not accountable
            if other_obstacle == obstacle:
                continue
            # If the dist of the obstacle is too far, it is not accountable
            if obstacle.pos.dist_point_to_point(other_obstacle.pos) >= IGNOREDIST:
                continue

            if isCollideSubgoal1 == 0:
                dist1 = subgoal1.dist_point_to_point(other_obstacle.pos)
                obstacleDist1 += dist1
                if dist1 < obstacle.radius + environment.robotRadius:
                    isCollideSubgoal1 = 1
            if isCollideSubgoal2 == 0:
                dist2 = subgoal2.dist_point_to_point(other_obstacle.pos)
                obstacleDist2 += dist2
                if dist2 < obstacle.radius + environment.robotRadius:
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
        if obstacleDist1 >= obstacleDist2:
            subgoal = subgoal1
        else:
            subgoal = subgoal2
    elif isCollideSubgoal1 == 0:
        subgoal = subgoal1
    else:
        subgoal = subgoal2

    # subgoalAdd.points.append(subgoal)

    return subgoal


def FastPathPlanning(environment, trajectory, depth, destination):
    """This function would be executed recursively, """
    # global subgoalAdd
    notUsed = 0

    [isCollide, point1, point2, obstacle] = trajectory.collide(environment)

    if isCollide != 0 and depth <= MAX_RECURSIVE_DEPTH :
        subgoal = SearchPoint(point1, point2, obstacle, environment)
        print("sugboal generated-- %f, %f, between Point1: %f, %f, and Point2: %f, %f" % (subgoal.x, subgoal.y, point1.x, point1.y, point2.x, point2.y))

        if subgoal == 0:
            print("No plausible trajectory found, system exit with 1")
            sys.exit(1)

        # the following two line divide the trajectory points into two trajectory
        points1 = trajectory.points[:trajectory.points.index(point1)+1] + [subgoal]
        trajectory1 = FastPathPlanning(environment, Trajectory(points1), depth+1, notUsed)
        points2 = [subgoal] + trajectory.points[trajectory.points.index(point2):]
        trajectory2 = FastPathPlanning(environment, Trajectory(points2), depth+1, notUsed)

        # The following line merge two trajectory into one traj
        trajectory = trajectory1.trajMerge(trajectory2)

    # in case that no collision detected between start point and the end point
    if depth == 0:
        if trajectory.points[-1] == destination:
            trajectory.isPlanSuccess = True
            print("Path found")
        else:
            trajectory.isPlanSuccess = False
            print("Out of maximum iteration")

    # return trajectory
    return trajectory


if __name__ == '__main__':
    # env = Environment([Obstacle(10, 10, 5), Obstacle(25, 25, 5)], 40, 40, 2)
    # traj = Trajectory([Point(0, 0), Point(10, 10), Point(25, 25)])
    # FastPathPlanning(env, traj, 0)
    a = Point(0, 0)
    print(a)
