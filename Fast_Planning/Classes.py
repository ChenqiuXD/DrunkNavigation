# Document explanation
# Author      :       Li JianHui
# Date        :       2019/03/20
# Function    :       Classes for implementing "Fast Path Planning Algorithm"

# Importing area
import math
import numpy as np

# Constant variable definition
START_POINT = (0, 0)
END_POINT = (5000, 5000)
MAX_RECURSIVE_DEPTH = 100


# Class definition
class Point:
    def __init__(self, x=0, y=0):
        self.x = x
        self.y = y

    def __str__(self):
        return '(Point Location: %f, %f)' % (self.x, self.y)

    def dist_point_to_point(self, point):
        return math.sqrt((self.x-point.x)**2+(self.y-point.y)**2)

    def dist_point_to_line(self, line_point1, line_point2):
        """This method calculate the distance from the point (obstacles in this program)
        to the line which is representing by line_point1 and line_point2 using vector method
        and would return the relative position of the line with point (look at vector method)"""
        # Tested : trustworthy Ver0.0
        # reference : https://blog.csdn.net/u012138730/article/details/79779996
        distVector = [line_point2.x-line_point1.x, line_point2.y-line_point1.y]
        distTargetVector = [self.x-line_point1.x, self.y-line_point1.y]
        factor = ((distVector[0]*distTargetVector[0]+distVector[1]*distTargetVector[1])/
                (distVector[0]**2+distVector[1]**2))
        if factor >= 1:
            return [1, math.sqrt((self.x-line_point2.x)**2+(self.y-line_point2.y)**2)]
        elif factor <= 0:
            return [2, math.sqrt((self.x-line_point1.x)**2+(self.y-line_point1.y)**2)]
        else:
            parallelVector = [0, 0]
            parallelVector[0] = factor*distVector[0]
            parallelVector[1] = factor*distVector[1]
            dist = [0, 0]
            dist[0] = distTargetVector[0] - parallelVector[0]
            dist[1] = distTargetVector[1] - parallelVector[1]
            return [3, math.sqrt(dist[0]**2+dist[1]**2)]


class Obstacle:
    """Representing the general obstacles. By 'general' I mean 'RoundObstacle'
    and 'RectObstacle' could inherit this class"""
    def __init__(self, x, y, radius=30, is_moving=False):
        self.pos = Point(x, y)
        self.radius = radius
        self.isMoving = is_moving

    def __str__(self):
        return "The pos of obstacle is %f, %f" % (self.pos.x, self.pos.y)


class Environment:
    """Representing the whole environment, mainly for access to perimeter and obstacles."""
    def __init__(self, obstacles, perimeter_x, perimeter_y, robotRadius):
        self.obstacles = obstacles
        self.perimeter = Point(perimeter_x, perimeter_y)
        self.robotRadius = robotRadius

    def __str__(self):
        for obstacle in self.obstacles:
            print(obstacle.pos)
            print(str(obstacle.isMoving))
        return "Environment's obstacle is printed above"


class Trajectory:
    """Could be conceived as a list of Point"""
    def __init__(self, points):
        self.points = points
        self.isPlanSuccess = False

    def __str__(self):
        for i in np.arange(len(self.points)):
            print(self.points[i])
        return "trajectory plan is %s" %str(self.isPlanSuccess)

    def trajMerge(self, traj):
        """This method """
        self.points[len(self.points):len(self.points)+len(traj.points)-1] = traj.points[1:]
        return self

    # this function detect the collision between obstacles and trajectory
    # and would return typeOfCollide, point1, point2 and obstacle
    # by typeOfCollide means:  0-no collide, 1-near the point1, 2-near the point2, 3-on the projection of line.
    def collide(self, environment):
        typeOfCollide = 0
        i = 0
        for i in np.arange(len(self.points)-1):
            if typeOfCollide != 0:
                i = i-1
                break
            for obstacle in environment.obstacles:
                [typeOfDist, distance] = obstacle.pos.dist_point_to_line(self.points[i], self.points[i + 1])
                if distance < obstacle.radius + environment.robotRadius:
                    typeOfCollide = typeOfDist
                    break
                else:
                    typeOfCollide = 0
        return [typeOfCollide, self.points[i], self.points[i+1], obstacle]


# main loop
if __name__ == "__main__":
    initPoints = [Point(START_POINT[0], START_POINT[1]), Point(END_POINT[0], END_POINT[1])]
    trajectory = Trajectory(initPoints)
    depth = 1

    addedPoints = [Point(END_POINT[0], END_POINT[1]), Point(1,1), Point(0,0)]
    traj2 = Trajectory(addedPoints)
    trajectory.trajMerge(traj2)

    print(trajectory)

