ros2 run rqt_tf_tree rqt_tf_tree --ros-args -r /tf:=tf -r /tf_static:=tf_static -r __ns:=/red_standard_robot1


ros2 launch pb2025_nav_bringup rm_sentry_reality_launch.py slam:=True use_robot_state_pub:=True

ros2 run nav2_map_server map_saver_cli -f rmul_2025 --ros-args -r __ns:=/red_standard_robot1

ros2 launch pb2025_nav_bringup rm_sentry_reality_launch.py world:=rmul_2025 slam:=False use_robot_state_pub:=True

