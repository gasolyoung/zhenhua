#!/bin/bash
cd "$( dirname "${BASH_SOURCE[0]}" )"

gnome-terminal -t "zh" -x bash -c "source install/setup.bash && roslaunch zh_demo zh_lidar.launch;exec bash;" 
# -t代表标题；-x是在一个主终端打开多个子终端，18.04推荐用--替代-x，但是--会每一条gmoe-terminal打开一个分离终端；exec bash代表在终端内容执行完也不关闭终端
sleep 3s
gnome-terminal -t "rosbag" -x bash -c "rosbag play -l /media/lab/wsc_data/rosbag/zhenhua/exlog/perception_log.zpmc109.root.20230818-105105.bag /perception/log/points_raw:=/points_raw;exec bash;"

rviz -d zh_wrong.rviz
