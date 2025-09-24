#!/bin/bash
source install/setup.bash

ros2 launch pb2025_nav_bringup rm_sentry_simulation_launch.py \
world:=rmul_2025 \
slam:=False
