import numpy
from pylab import *
import copy


class MAP(object):
    """
    画出地图
    """
    def __init__(self,map, finder):
        self.map = map
        self.finder = finder
    def draw_init_map(self):
        """
        画出起点终点图
        :return:
        """
        plt.imshow(self.map, cmap=plt.cm.hot, interpolation='nearest', vmin=0, vmax=10)
        # plt.colorbar()
        xlim(-1, 50)  # 设置x轴范围
        ylim(-1, 50)  # 设置y轴范围
        my_x_ticks = numpy.arange(0, 50, 1)
        my_y_ticks = numpy.arange(0, 50, 1)
        plt.xticks(my_x_ticks)
        plt.yticks(my_y_ticks)
        plt.grid(True)
        plt.show()

    def draw_path_open(self, a):
        """
        画出open表中的坐标点图
        :return:
        """
        map_open = copy.deepcopy(self.map)
        for i in range(a.close_list.shape[1]):
            x = a.close_list[:, i]

            map_open[int(x[0]), int(x[1])] = 1

        plt.imshow(map_open, cmap=plt.cm.hot, interpolation='nearest', vmin=0, vmax=10)
        # plt.colorbar()
        xlim(-1, 50)  # 设置x轴范围
        ylim(-1, 50)  # 设置y轴范围
        my_x_ticks = numpy.arange(0, 50, 1)
        my_y_ticks = numpy.arange(0, 50, 1)
        plt.xticks(my_x_ticks)
        plt.yticks(my_y_ticks)
        plt.grid(True)
        plt.show()

    def draw_path_closed(self, a):
        """
        画出closed表中的坐标点图
        :return:
        """
        print('打印close_list长度：')
        print(a.close_list.shape[1])
        map_closed = copy.deepcopy(self.map)
        for i in range(a.close_list.shape[1]):
            x = a.close_list[:, i]

            map_closed[int(x[0]), int(x[1])] = 5

        plt.imshow(map_closed, cmap=plt.cm.hot, interpolation='nearest', vmin=0, vmax=10)
        # plt.colorbar()
        xlim(-1, 50)  # 设置x轴范围
        ylim(-1, 50)  # 设置y轴范围
        my_x_ticks = numpy.arange(0, 50, 1)
        my_y_ticks = numpy.arange(0, 50, 1)
        plt.xticks(my_x_ticks)
        plt.yticks(my_y_ticks)
        plt.grid(True)
        plt.show()

    def draw_direction_point(self, a):
        """
        从终点开始，根据记录的方向信息，画出搜索的路径图
        :return:
        """
        print('打印direction长度：')
        print(a.best_path_array.shape[1])
        map_direction = copy.deepcopy(self.map)
        for i in range(a.best_path_array.shape[1]):
            x = a.best_path_array[:, i]

            map_direction[int(x[0]), int(x[1])] = 6

        plt.imshow(map_direction, cmap=plt.cm.hot, interpolation='nearest', vmin=0, vmax=10)
        # plt.colorbar()
        xlim(-1, 50)  # 设置x轴范围
        ylim(-1, 50)  # 设置y轴范围
        my_x_ticks = numpy.arange(0, 50, 1)
        my_y_ticks = numpy.arange(0, 50, 1)
        plt.xticks(my_x_ticks)
        plt.yticks(my_y_ticks)
        plt.grid(True)

    def draw_three_axes(self, a):
        """
        将三张图画在一个figure中
        :return:
        """
        plt.figure()
        ax1 = plt.subplot(221)

        ax2 = plt.subplot(222)
        ax3 = plt.subplot(223)
        ax4 = plt.subplot(224)
        plt.sca(ax1)
        self.draw_init_map()
        plt.sca(ax2)
        self.draw_path_open(a)
        plt.sca(ax3)
        self.draw_path_closed(a)
        plt.sca(ax4)
        self.draw_direction_point(a)

        plt.show()
