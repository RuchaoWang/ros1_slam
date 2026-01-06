#!/usr/bin/env bash
set -e

# 1) 清理旧进程（允许失败）
rosnode kill -a 2>/dev/null || true
killall -9 gzserver gzclient 2>/dev/null || true
killall -9 roscore rosmaster 2>/dev/null || true

yes | rosclean purge

sleep 2


# 2) 启动 roscore
gnome-terminal -t "roscore" -- bash -lc "roscore"

# 等待 roscore 就绪
until rostopic list >/dev/null 2>&1; do
  sleep 0.2
done

# 3) 启动 Gazebo + robot
gnome-terminal -t "gazebo_start" -- bash -lc "source ~/Gitkraken/ros1_slam/devel/setup.bash; roslaunch tri_steer_gazebo sim_gazebo_rviz.launch"

# 等待 Gazebo 节点起来（/gazebo/model_states 出现再继续）
until rostopic list 2>/dev/null | grep -q "/gazebo/model_states"; do
  sleep 0.5
done

# 4) 启动 teleop + robot
#gnome-terminal -t "teleop_start" -- bash -lc "source ~/Gitkraken/ros1_slam/devel/setup.bash; roslaunch tri_steer_keyboard keyboard_tri_steer.launch"

# 5）启动地图节点
gnome-terminal -t "map start" -- bash -lc "source ~/Gitkraken/ros1_slam/devel/setup.bash; roslaunch robot_costmap map_deal.launch;exec bash"

# 6) 运动规划节点
gnome-terminal -t "motion Plan start" -- bash -lc "source ~/Gitkraken/ros1_slam/devel/setup.bash; roslaunch robot_navigation motionPlan_sim.launch"

# 7) 雷达点云转换成point-cloud2
gnome-terminal -t "laser to cloud" -- bash -lc \
"source ~/Gitkraken/ros1_slam/devel/setup.bash; 
roslaunch read_laser_data laser_to_cloud.launch; exec bash"

# 8）局部规划***********
gnome-terminal -t "DWA_new" -- bash -lc \
"source ~/Gitkraken/ros1_slam/devel/setup.bash; 
roslaunch diff_pure_pursuit_local_planner diff_local_planner.launch; exec bash"

# 9) 启动并发布机器人真值里程计消息
gnome-terminal -t "truth Odom start" -- bash -lc \
"source ~/Gitkraken/ros1_slam/devel/setup.bash; 
roslaunch robot_locatization truth_odometry.launch;exec bash"

# 10）订阅控制消息
gnome-terminal -t "control  start" -- bash -lc "source ~/Gitkraken/ros1_slam/devel/setup.bash; roslaunch tri_steer_drive bringup.launch use_teleop:=true;exec bash"

# 当重新编译工作空间的时候，需要先编译通信包catkin_make --pkg robot_conmmunication