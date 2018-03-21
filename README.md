# crazyflie_ecust

Crazyflie Ros platform for ecust

## 依赖库
  - [crazyflie_ros](https://github.com/whoenig/crazyflie_ros)
  - vicon_bridge
  - ~~hector_quadrotor~~ (现在用我的这套方案应该不需要这个库了)

## 硬件检查：
在软件连接之前先检查硬件连接
  - vicon有没有打开 
  - vicon的服务器的软件有没有打开
  - 网线有没有插好
  - crazyradio
  - xbox360（我后面会尝试写个/弄来个键盘控制的）算了吧这个鸽了= 0

## 驱动检查：
有硬件的地方就有驱动，驱动主要包括crazyradio的驱动和手柄的驱动。
1. crazyradio的驱动，按照[这篇帖子](http://blog.csdn.net/banzhuan133/article/details/76166160)进行：
    
2. 


## 如果运行不出时候的解决办法：

2. 就算连接上之后，也得看下是否软件连上了吧
    - vicon的电脑连的局域网是不是和vicon一个网？
    - crazyradio pa的驱动下了没有？好像用到libusb1.0什么

3. 然后就看报什么错了吧= 0，bug可在issue里提嘛～虽然我觉得不会有人


4. 可能会遇到报错AgentU.h：没有那个文件或者目录
这个应该是编译顺序有问题，应该先编译消息。

2017年12月4日

修复csvhelper的一个bug 在Kinetic上编译通过,indigo完美通过.但是以后的趋势肯定还是Kinetic呀

运行的方式:

单架无人机
`roslaunch hover_vicon hover_vicon.launch`
在参数列表里面修改地址和vicon对应的哪一架飞机.

1-6架无人机都有做,最多6架无人机
运行相应的launch文件就可以运行.

多架无人机

`roslaunch leader_follower centralize.launch`


bug 说明: 要使用手柄还需要hector_quadrotor这个git包,地址是:
git@github.com:tu-darmstadt-ros-pkg/hector_quadrotor.git

因为和其中gazebo的版本不兼容,所以在这个包里面需要吧所有关于gazebo的包都删除,否则编译不通过.

