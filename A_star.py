from actionmodule import  ActionModule
from visionmodule_athena import  VisionModule
import numpy as np
import time as t
from Map import MAP


ROBOT_ME = 0

max_length = 50
max_width = 36
MAP_SIZE = (max_length, max_width)
# map_grid = np.zeros((120, 90))
# obstacles_list = np.array([[6,16,25,33,44],[5,14,43,21,28]])
class find_global_path():

    def __init__(self, me_location, obstacles, goal, map_size=MAP_SIZE):
        self.base_location = [-2.5,-1.8]
        self.destination = (goal-self.base_location)*10
        self.start = (me[0:2]-self.base_location)*10
        l = []
        for i in range(ob.__len__()):
            l.append(self.base_location)
        l_ = np.array(l)
        self.obstacles = (ob[:,0:2] - l_)*10
        self.open_list = np.array([[],[],[],[],[],[]])
        self.close_list = np.array([[],[],[],[],[],[]])
        self.best_path_array = np.array([[], []])
        self.map = np.zeros(map_size)
        self.generate_map(self.obstacles)
        self.run()
        self.path_backtrace()
    def cal_g(self, current,father_point):
        g1 = father_point[0] - current[0]
        g2 = father_point[1] - current[1]
        g = g1 ** 2 + g2 ** 2
        g = np.sqrt(g) + father_point[4]  # 加上累计的g值
        return g

    def cal_h(self,current):
        h = (current[0] - self.destination[0]) ** 2 + (current[1] - self.destination[1]) ** 2
        h = np.sqrt(h)  # 计算h
        return h

    def cal_f(self,current,father_point):
        return self.cal_g(father_point=father_point,current=current)+self.cal_h(current)

    def child_point(self, x):
        """
        拓展的子节点坐标
        :param x: 父节点坐标
        :return: 子节点存入open表，返回值是每一次拓展出的子节点数目，用于撞墙判断
        当搜索的节点撞墙后，如果不加处理，会陷入死循环
        """
        # 开始遍历周围8个节点
        for j in range(-1, 2, 1):
            for q in range(-1, 2, 1):

                if j == 0 and q == 0:  # 搜索到父节点去掉
                    continue
                m = [x[0] + j, x[1] + q]
                # print(m)
                if m[0] < 0 or m[0] > max_length-1 or m[1] < 0 or m[1] > max_width-1:  # 搜索点出了边界去掉
                    continue

                if self.map[int(m[0])][int(m[1])] == 1:  # 搜索到障碍物去掉
                    continue


                record_g = self.cal_g(current= m, father_point=x)
                record_f = self.cal_f(current=m, father_point=x)  # 计算每一个节点的f值

                x_direction, y_direction = self.direction(x, m)  # 每产生一个子节点，记录一次方向

                para = [m[0], m[1], x_direction, y_direction, record_g, record_f]  # 将参数汇总一下
                # print(para)

                # 在open表中，则去掉搜索点，但是需要更新方向指针和self.g值
                # 而且只需要计算并更新self.g即可，此时建立一个比较g值的函数
                a, index = self.judge_location(m, self.open_list)
                if a == 1:
                    # 说明open_list中已经存在这个点

                    if record_f <= self.open_list[5][index]:
                        self.open_list[5][index] = record_f
                        self.open_list[4][index] = record_g
                        self.open_list[3][index] = y_direction
                        self.open_list[2][index] = x_direction

                    continue

                # 在close_list表中,则去掉搜索点
                b, index2 = self.judge_location(m, self.close_list)
                if b == 1:

                    if record_f <= self.close_list[5][index2]:
                        self.close_list[5][index2] = record_f
                        self.close_list[4][index2] = record_g
                        self.close_list[3][index2] = y_direction
                        self.close_list[2][index2] = x_direction
                        self.close_list = np.delete(self.close_list, index2, axis=1)
                        self.open_list = np.c_[self.open_list, para]
                    continue

                self.open_list = np.c_[self.open_list, para]  # 参数添加到open_list中
                # print(self.open_list)

    def judge_location(self, m, list_co):
        """
        判断拓展点是否在open_list表或者close_list表中
        :return:返回判断是否存在，和如果存在，那么存在的位置索引
        """
        jud = 0
        index = 0
        for i in range(list_co.shape[1]):

            if m[0] == list_co[0, i] and m[1] == list_co[1, i]:

                jud = jud + 1

                index = i
                break
            else:
                jud = jud
        # if a != 0:
        #     continue
        return jud, index
    def judge_in_map(self,location):
        x = location[0]
        y = location[1]
        if x <=MAP_SIZE[0]-1 and x>=0 and y <=MAP_SIZE[1] and y >=0:
            return True
        else:
            return False
    def direction(self, father_point, son_point):
        """
        建立每一个节点的方向，便于在close_list表中选出最佳路径
        非常重要的一步，不然画出的图像参考1.1版本
        x记录子节点和父节点的x轴变化
        y记录子节点和父节点的y轴变化
        如（0，1）表示子节点在父节点的方向上变化0和1
        :return:
        """
        x = son_point[0] - father_point[0]
        y = son_point[1] - father_point[1]
        return x, y

    def flatten_obstacles(self,obstacle):
        x = int(obstacle[0])
        y = int(obstacle[1])
        if not self.judge_in_map(np.array([x,y])):
            return []
        # vx = obstacle_velocity[2]
        # vy = obstacle_velocity[3]
        # me = self.get_self()
        # me_x = me[0]
        # me_y = me[1]
        # me_vx = me[2]
        # me_vy = me[3]
        # distance = np.sqrt((x-me_x)**2+(y-me_y)**2)
        # t = distance/np.sqrt(me_vx**2+me_vy**2)
        # x += vx*t
        # y += vy*t

        point_list = []
        for i in range(-1,1):
            for k in range(-1,1):
                if self.judge_in_map(np.array([i+x,k+y])):
                    point_list.append([i+x,k+y])
        point_list = np.array(point_list)
        return point_list

    def generate_map(self,obstacles):
        for [x, y]  in obstacles:
            point_list = self.flatten_obstacles(np.array([x, y]))
            for [x,y] in point_list:
                self.map[int(x)][int(y)] = 1


    def path_backtrace(self):
        """
        回溯close_list表中的最短路径
        :return:
        """
        best_path = [max_length-1, max_width-1]  # 回溯路径的初始化
        self.best_path_array = np.array([[max_length-1], [max_width-1]])
        j = 0
        while j <= self.close_list.shape[1]:
            for i in range(self.close_list.shape[1]):
                if best_path[0] == self.close_list[0][i] and best_path[1] == self.close_list[1][i]:
                    x = self.close_list[0][i]-self.close_list[2][i]
                    y = self.close_list[1][i]-self.close_list[3][i]
                    best_path = [x, y]
                    self.best_path_array = np.c_[self.best_path_array, best_path]
                    break  # 如果已经找到，退出本轮循环，减少耗时
                else:
                    continue
            j = j+1
            self.best_path_array = self.best_path_array/10.0
        return self.best_path_array

    def run(self):
        """
        main函数
        :return:
        """
        self.generate_map(self.obstacles)
        x_start = int(self.start[0])
        y_start = int(self.start[1])
        best = np.array([x_start,y_start]) # 起点放入当前点，作为父节点
        h0 = self.cal_h(best)
        init_open = np.array([[best[0]], [best[1]], [0], [0], [0], [h0]])  # 将方向初始化为（0，0），g_init=0,f值初始化h0
        self.open_list = np.hstack((self.open_list, init_open))  # 起点放入open,open初始化

        ite = 1  # 设置迭代次数小于200，防止程序出错无限循环
        while ite <= 1000000:
                # open列表为空，退出
                if self.open_list.shape[1] == 0:
                    print('没有搜索到路径！')
                    break

                self.open_list = self.open_list.T[np.lexsort(self.open_list)].T  # open表中最后一行排序(联合排序）

                # 选取open表中最小f值的节点作为best，放入close_list表

                best = self.open_list[:, 0]
                # print('检验第%s次当前点坐标*******************' % ite)
                # print(best)
                self.close_list = np.c_[self.close_list, best]

                if best[0] == int(self.destination[0]) and best[1] == int(self.destination[1]):  # 如果best是目标点，退出
                    print('搜索成功！')
                    break

                self.child_point(best)  # 生成子节点并判断数目
                # print(self.open_list)
                self.open_list = np.delete(self.open_list, 0, axis=1)  # 删除open中最优点


                ite = ite+1

if __name__ =="__main__":
    SIM = True
    vision = VisionModule(sim=SIM)
    vision.get_info(0)
    me = vision.robot_info
    ob = vision.other_robots
    goal = np.array([2.45,-1.7])
    finder = find_global_path(me, ob, goal, map_size=MAP_SIZE)
    plot = MAP(map = finder.map, finder=finder)
    plot.draw_path_open(finder)
    plot.draw_path_closed(finder)
    # plt.show()
