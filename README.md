# crazyflie_ecust

Crazyflie Ros platform for ecust

依赖库：crazyflie_ros vicon_bridge

## 如果运行不出时候的解决办法：
1. 检查硬件设备是否连接成功，要连接这么些设备
    - vicon的服务器
    - crazyradio
    - xbox360（我后面会尝试写个/弄来个键盘控制的）

2. 就算连接上之后，也得看下是否软件连上了吧
    - vicon的电脑连的局域网是不是和vicon一个网？
    - crazyradio pa的驱动下了没？好像用到libusb1.0什么的

3. 然后就看报什么错了吧= 0，bug可在issue里提嘛～虽然我觉得不会有人的