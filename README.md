** 程序运行手册
*** 程序开启顺序
1. control. 根据策略选择 launch 文件, 如果想要查找段错误, 使用 rosrun
2. odom. 
3. navigation. 必须等 odom 的数据稳定之后才能开启, 注意看规划的路径对不对
4. realsense. 如果杀掉本程序, 需要重新插拔 realsense
5. infrared_detection.
6. vision. 
7. predict. infrared_detection 成功开启之后再开启本程序

*** 运行前检查
1. 检查各个设备是否在线 (ls /dev), 如果有不在线的设备, 使用 U 盘插拔
2. 检查两台机器人是否连接上 (socket 通信是否正常工作)
3. 校准 yaw 轴

*** 检测程序训练


*** 导航部分
根据不同的策略，选择到达的终点坐标。
// zhongdian: 3-34; 4.0, 2.5 
// point 1:  3-11; 3.3, 3.2
//point 3: 3-13; 4.00,3.8
//point 2(robot 2): 9-8; 2.6, 2.1
注意，修改完*go_center*后,需要修改robo_navigation_node.cpp中get_vel函数中的420行和427行中*path[0]==的序号为终点。
其次，设置的目标点的方向都是0度，因此，对于 point 13需要在control函数中返回true后立马转到对应的角度才行。
之后，要到的位置需要给对应点的坐标和转角才行：
对于跟踪瞄准敌方车辆，同时给角度和x,y；
对于长距离到达某一个固定点，需要分两步进行，第一步给0度角和x,y值；第二部给到达目标点的x,y和yaw值。