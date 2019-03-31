# Import Area
from Classes import Point, Obstacle, Environment, Trajectory
from Functions import FastPathPlanning
import numpy as np
import cv2
import time

# Constant Variable Definition Area
START_POINT = (0, 0)
END_POINT = (550, 550)
MAX_RECURSIVE_DEPTH = 100
OBSTACLE_RADIUS = 30
ROBOT_RADIUS = 30
PERIMETER = (900, 600)
BLACK = (0, 0, 0)
YELLOW = (30, 30, 255)
BLUE = (255, 50, 50)
RED = (255, 0, 0)
cvWindowSize = (600, 900)
DOTSIZE = 5


# Function Definition
def run_viewer(trajectory, environment):
    img = np.zeros((600, 900, 3), dtype="uint8")

    # while True:
    cv2.rectangle(img, (0, 0), (900, 600), BLACK, -1)
    for obstacle in environment.obstacles:
        cv2.circle(img, (obstacle.pos.x, obstacle.pos.y), obstacle.radius, YELLOW)
    for i in np.arange(len(trajectory.points)-1):
        cv2.line(img, (int(trajectory.points[i].x), int(trajectory.points[i].y)),
                 (int(trajectory.points[i+1].x), int(trajectory.points[i+1].y)), RED)
        cv2.circle(img, (int(trajectory.points[i].x), int(trajectory.points[i].y)), DOTSIZE, YELLOW)
        cv2.imshow("Viewer", img)
        cv2.waitKey(0)
    cv2.circle(img, (int(trajectory.points[-1].x), int(trajectory.points[-1].y)), DOTSIZE, YELLOW)
    # cv2.line(img, (START_POINT), (END_POINT), BLUE)
    cv2.imshow("Viewer", img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()


def run_background_viewer(img, environment):
    cv2.rectangle(img, (0,0), (900,600), BLACK, -1)
    for obstacle in environment.obstacles:
        cv2.circle(img, (obstacle.pos.x, obstacle.pos.y), obstacle.radius, YELLOW)


def run_debug_viewer(start, end, subgoalAdd, environment):
    img = np.zeros((600, 900 , 3), dtype="uint8")

    # while True:
    for i in np.arange(len(subgoalAdd.points)-1):
        run_background_viewer(img, environment)

        cv2.circle(img, (int(start.x), int(start.y)), DOTSIZE, YELLOW)

        for j in np.arange(i)+1:
            cv2.circle(img, (int(subgoalAdd.points[j].x), int(subgoalAdd.points[j].y)), DOTSIZE, YELLOW)

        cv2.circle(img, (int(end.x), int(end.y)), DOTSIZE, YELLOW)

        cv2.imshow("Viewer", img)
        cv2.waitKey(0)
    cv2.circle(img, (int(subgoalAdd.points[-1].x), int(subgoalAdd.points[-1].y)), DOTSIZE, YELLOW)
    # cv2.line(img, (START_POINT), (END_POINT), BLUE)
    cv2.imshow("Viewer", img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()


# Main loop
if __name__ == '__main__':
    points = [Point(START_POINT[0], START_POINT[1]), Point(END_POINT[0], END_POINT[1])]
    trajectory = Trajectory(points)
    depth = 1

    # Wrong testing examples
    # obstacles = [Obstacle(100, 100), Obstacle(69, 159), Obstacle(145, 226), Obstacle(159, 285), Obstacle(200, 100), Obstacle(300, 300)]
    obstacles = [Obstacle(300,300), Obstacle(206,281), Obstacle(274,145), Obstacle(155,257)]

    # obstacles = [Obstacle(300, 300)]
    # for i in np.arange(3):
    #     obstacles.append(Obstacle(np.random.randint(100+OBSTACLE_RADIUS, int(PERIMETER[0]/2)),
    #                               np.random.randint(100+OBSTACLE_RADIUS, int(PERIMETER[1]/2))))

    environment = Environment(obstacles, PERIMETER[0], PERIMETER[1], ROBOT_RADIUS)

    trajectory, subgoalAdd = FastPathPlanning(environment, trajectory, depth)
    # run_viewer(trajectory, environment)
    run_debug_viewer(trajectory.points[0], trajectory.points[-1], subgoalAdd, environment)
    print("All done")
