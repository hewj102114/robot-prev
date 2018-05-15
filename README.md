** 程序运行手册
*** 程序开启顺序
1. control. 根据策略选择 launch 文件, 如果想要查找段错误, 使用 rosrun
2. odom. 
3. navigation.
4. realsense. 如果杀掉本程序, 需要重新插拔 realsense
5. infrared_detection.
6. vision. 
7. predict. infrared_detection 成功开启之后再开启本程序

*** 运行前检查
1. 检查各个设备是否在线 (ls /dev), 如果有不在线的设备, 使用 U 盘插拔
2. 检查两台机器人是否连接上 (socket 通信是否正常工作)
3. 校准 yaw 轴

*** 