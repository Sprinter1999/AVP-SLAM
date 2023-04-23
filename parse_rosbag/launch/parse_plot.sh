#! /bin/bash

roslaunch parse_rosbag parse_rosbag.launch
python3 src/AVP-SLAM-PLUS/parse_rosbag/src/2D_graph_slam.py
