# DrunkNavigation

本项目目的是在有移动（并具有攻击性）的障碍物的前提下进行避障。

目前借助学长的Python代码，已经实现了传感器信息的接收以及运动控制指令的下达。

## 避障规划思路

暂时采用Global Planner + Local Planner的思路。Global Planner拟采用递归的直线路径；而Local Planner拟采用DWA。

考虑到Python可能会出现运算速度不够快的问题，我们可能需要Python和C++结合，甚至最后转移到C++。

## 待解决的问题

目前，使用GrSim进行通信，存在读取的Vision信号中Robot波动、信息不全的问题。排除之后应该不是UDP传输的问题，可能是GrSim自己的问题，而Athena自己就进行了滤波，可能这个问题可以得到解决。

1. 完成仿真环境和机器人控制算法的DWA的连接（以尽量低的复杂度）；
2. 完成Python程序和Athena的通信。

## FastPathPlanning
1. 可以运行FastPathPlanning文件夹中的run_viewer文件来查看算法的效果，需要opencv、numpy，会出现代表障碍物的红色圆圈（会有重叠）和象征规划路径的蓝线，按空格键陆续出现规划轨迹。
2. 运行grSim与Athena，在运行main.py可以看到Athena上面会出现FPP规划的路径。
3. 待解决问题为：算法规划后会震荡，规划的路径在某个地方会画好几条重复的线。