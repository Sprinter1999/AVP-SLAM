#! /bin/bash

rosbag record -o src/AVP-SLAM-PLUS/parse_rosbag/data/rosbag/ /odom /vertex_odom /edge_odom /currentPose /ideal_cmd_vel
# rosbag record /odom /vertex_odom /edge_odom /currentFeatureInWorld /currentPose /GT /ideal_cmd_vel
