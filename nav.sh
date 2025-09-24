#!/bin/bash

chmod 777 /dev/ttyACM0

# 启动第一个ROS节点（新终端）
gnome-terminal -- bash -c \
  "source /opt/ros/humble/setup.bash && \
   source /home/rm/fzsd2025_V2/install/setup.bash && \
   ros2 run tf2_ros static_transform_publisher --frame-id map --child-frame-id odom \
   --x 0 --y 0 --z 0 --roll 0 --pitch 0 --yaw 0 \
   --ros-args -r /tf:=tf -r /tf_static:=tf_static -r __ns:=/red_standard_robot1; \
   exec bash"

# 等待10秒确保第一个节点启动完成
sleep 3

# 启动第二个ROS节点（新终端）
gnome-terminal -- bash -c \
  "source /opt/ros/humble/setup.bash && \
   source /home/rm/fzsd2025_V2/install/setup.bash && \
   ros2 launch pb2025_nav_bringup rm_sentry_reality_launch.py world:=rmul_2025 slam:=False use_robot_state_pub:=True
   exec bash"
