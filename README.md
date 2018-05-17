## 程序运行手册
### 程序开启顺序
1. control. 根据策略选择 launch 文件, 如果想要查找段错误, 使用 rosrun
2. odom. 
3. navigation. 必须等 odom 的数据稳定之后才能开启, 注意看规划的路径对不对
4. realsense. 如果杀掉本程序, 需要重新插拔 realsense
5. infrared_detection.
6. vision. 
7. predict. infrared_detection 成功开启之后再开启本程序

### 运行前检查
1. 检查各个设备是否在线 (ls /dev), 如果有不在线的设备, 使用 U 盘插拔
2. 检查两台机器人是否连接上 (socket 通信是否正常工作)
3. 校准 yaw 轴
4. 使用的 launch 文件是否正确

*** 检测程序训练


### 导航部分
1. 根据不同的策略，选择到达的终点坐标。  
    zhongdian: 3-34; 4.0, 2.5   
    point 1:  3-11; 3.3, 3.2  
    point 3: 3-13; 4.00,3.8  
    point 2(robot 2): 9-8; 2.6, 2.1  
2. 注意，修改完*go_center*后,需要修改robo_navigation_node.cpp中get_vel函数中的420行和427行中*path[0]==的序号为终点。
3. 其次，设置的目标点的方向都是0度，因此，对于 point 13需要在control函数中返回true后立马转到对应的角度才行。
4. 之后，要到的位置需要给对应点的坐标和转角才行：  
(1) 对于跟踪瞄准敌方车辆，同时给角度和x,y。  
(2) 对于长距离到达某一个固定点，需要分两步进行，第一步给0度角和x,y值；第二步给到达目标点的x,y和yaw值。

### USB 设备名称
1. 串口 ->
2. PX4 ->
3. 激光雷达 ->
4. 云台相机 ->
5. 鱼眼相机 1 ->
6. 鱼眼相机 2 ->
7. realsense ->

### 参数服务器
#### Control
##### YAW 控制
DEATH_AREA = 10                         -> 旋转打击死区角度

##### track_enemy
SWITCH_FORWARD_BACKWARD_DIS = 0.6       -> 前进后退的切换距离
MIN_TRACK_ENEMY_DIS = 0.7               -> 小于这个距离的导航点过滤 (DISTANCE = 1000)
MAX_TRACK_ENEMY_DIS = 3.0               -> 大于这个距离的导航点过滤 (DISTANCE = 1000)

##### stack_enemy
ARMOR_MAX_LOST_NUM = 150                -> armor 最大丢帧数量
ARMOR_AROUND_MAX_LOST_NUM = 150         -> armor 最大丢帧数量 (切换转头)
REALSENSE_AROUND_MAX_LOST_NUM = 150     -> realsense 最大丢帧数量 (切换转头)

LOW_SHOT_SPEED_DISTANCE = 2.0           -> 低速射击最小距离
HIGH_SHOT_SPEED_DISTANCE = 1.5          -> 高速射击最大距离

ARMOR_LOST_PITCH = 5.0                  -> armor 丢帧的时候, pitch 给的角度



### 策略
#### 策略 1
测试用文件. 最贱策略, 见到敌人跟踪打击, 没有看到敌人回到蹲守的地方蹲守

#### 策略 2
假设: 敌人 1 车开局抢占中点, 2 车站在敌方区域防守
应对: 全力攻击抢中点的车, 打死敌人或者敌人拿到 buff 之后回家蹲守

#### 策略 3
假设: 敌人 1 车开局抢占中点, 2 车站跑到我方区域
应对: 全力攻击抢中点的车, 打死敌人或者敌人拿到 buff 之后回家蹲守 (不管非抢中点的车, 与策略 2 站位不同)

#### 策略 4
假设: 敌人 1, 2 车不抢中点, 全部过来攻击我们
应对: 攻击距离自己最近的车, 队友死亡或者敌方死亡之后回家蹲守

#### 策略 5
假设: 我方抢中点的速度快于敌方, 我们要抢中点
应对: 1 车抢中点, 2 车在我方区域进行辅助攻击, 抢到中点之后跟随敌人进行攻击, 直到视野里面没有敌军, 然后回到中点附近蹲守, 不回家蹲守

#### 策略 6
假设: 我方抢中点的速度快于敌方, 我们要抢中点
应对: 1 车抢中点, 2 车在敌方区域进行辅助攻击, 抢到中点之后跟随敌人进行攻击, 直到视野里面没有敌军, 然后回到中点附近蹲守, 不回家蹲守

#### 策略 7
假设: 我方抢中点的速度快于敌方, 我们要抢中点
应对: 1, 2 车都不抢中点, 开场直接跑到敌方区域进行攻击, 直到视野里面没有敌军, 然后回到中点附近蹲守, 不回家蹲守

#### 策略 8
假设: 全攻击模式
应对: 1, 2 车都不抢中点, 开场直接跑到敌方区域进行攻击, 直到视野里面没有敌军, 敌军消失之后, 进行巡图, 看到敌人就去打敌人

#### 策略 9
假设: 全防御模式(基本不可能会用到)
应对: 基本原则: 看到敌人就跑, 直到视野里面没有任何敌人
