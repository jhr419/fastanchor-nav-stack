这份文档是一个最简的介绍，只能对系统的基本框架和使用方法做一个介绍，具体内容还是看README_old.md user.md等文档(更新不及时)以及源代码

这个项目是一个三维导航栈，尽量做的模块化

有需要讨论可以联系：

15603579123（微信同号）

# 主要模块介绍：


1. FastAnchor(ssh://git@192.168.0.118:2222/YuKun/FastAnchor.git) 

提供定位，基于ICP+lio，目前fastlio2适配比较好；yifanlio目前有比较多问题，主要是imu数据阻塞。仓库主分支的icp+fastlio2适配的比较好，定位速度和准度比较可靠

yifanlio是一个非公开的lio，是上交张一帆(同拼音yifan，不确定具体字)同学提供的lio，据说是基于fastlio2修改而来，主要是改善代码可读性，Elevator-LIO是基于这项工作继续做的一个lio(https://github.com/xiaofan4122/Elevator-LIO.git)也是张一帆同学的工作

2. FastPlanner

主要是为了整合主流三维全局规划器方案，主要整合了普通的a*，pct planner，jie-3d-nav中的三维栅格地图规划(https://github.com/6-robot/jie_3d_nav.git)

目前主要使用的是a*，简单好用

3. SCAN-Planner(https://github.com/wuyi2121/SCAN-Planner.git)

是一个基于激光点云的三维局部规划器，核心算法在于路径优化：

在基础的路径上，采样点并进行b-sline优化；在路径遇到障碍物情况下，会把障碍物路段进行 投影-a* 操作，具体可以看一下论文中的算法介绍，主要是把原本的三维查询空间压缩到一个与斜面相交的平面进行A*查询，降低计算成本

# 主要启动方式和参数介绍


1. 基础启动指令

ros2 launch navigation_bringup navigation_system.launch.py   

这个默认会启动：定位模块，全局规划器，局部规划器

默认不启动rviz2，减少机载端压力

可选参数：

选择地图，传入地图路径即可

map_pcd_path:=$PWD/maps/map_preprocessed2.pcd   

选择雷达传感器型号，目前只有mid360 mid360s两个参数可选

lidar_model:=mid360 

选择lio的实现，有fastlio2 和yifanlio, yifanlio目前问题比较多，截至到我写文档时，没有排查完。为了预防我离开时还没有完善这部分功能，这里先分享一个gpt链接可以看下前面的分析(https://chatgpt.com/share/6a855bcb-4c40-83ea-91b2-9e3d9ca9703b)

lio_backend:=yifanlio

log功能参数，前两个是检测指标，当低于这个频率，会记录在日志文件

runtime_log_map_target_rate_hz:=10.0 \
runtime_log_report_interval_sec:=5.0 \
runtime_log_csv_path:=/tmp/navigation-runtime.csv

2. 速度桥接

ros2 launch go2_twist_bridge twist_bridge.launch.py

用于将导航系统的速度话题桥接到go2需要的消息格式和话题，对应有一个yaml文件，可以调整速度缩放比例，最大速度限制等

3. 以服务的形式实现的功能，主要是给web/app端的功能：

(具体看user.md)

启动/停止/暂停/恢复/取消 导航任务

发送多个路径点(以ros2 aciton机制实现，使用action后，可以不再区分单点导航和多点导航)

速度桥接开关

# 目前问题

nuc运行，mid360

一个很大的问题现象是：

机器人在某些时候(在办公楼楼道的双开消防门最明显，单开门却没有很严重)，会存在定位pose更新正常，但是scan planner中的滑动窗口和局部障碍物点云不更新

目前没有定位到具体原因