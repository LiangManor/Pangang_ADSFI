#!/bin/bash

# 启动A3口的HW激光雷达
sleep 1
cd /home/sshuser/lidar_abstract_A3
python3 /opt/platform/mdc_platform/script/net_config_tool.py -M master_slave_set -m slave -p 2
python3 /opt/platform/mdc_platform/script/net_config_tool.py -M bandwidth_set -b 1000M -p 2
export CM_CONFIG_FILE_PATH=./LidarCmProcessA3/
export LD_LIBRARY_PATH=/opt/usr/root/usr/lib64:$LD_LIBRARY_PATH
export LD_LIBRARY_PATH=/home/sshuser/lidar_abstract_A3/lib/:${LD_LIBRARY_PATH}
./lidar_abstract_A3 &


# 启动B2口rs激光雷达
sleep 1
cd /home/sshuser/lidar_abstract_B2_rs16
python3 /opt/platform/mdc_platform/script/net_config_tool.py -M master_slave_set -m master -p 5
python3 /opt/platform/mdc_platform/script/net_config_tool.py -M bandwidth_set -b 100M -p 5
export CM_CONFIG_FILE_PATH=./LidarCmProcessB2/
export LD_LIBRARY_PATH=/opt/usr/root/usr/lib64:$LD_LIBRARY_PATH
export LD_LIBRARY_PATH=/home/sshuser/lidar_abstract_B2_rs16/lib/:${LD_LIBRARY_PATH}
route add -host 192.168.1.23/32 dev eth0.15
./lidar_abstract_B2_rs16 &

# 启动摄像头
#sleep 1
#/opt/platform/mdc_platform/script/camera_mviz_start.sh 71
# /opt/platform/mdc_platform/script/camera_mviz_start.sh 71 79 83 84

# 启动camera_det节点
#sleep 3
#/home/sshuser/camera_det.sh &


# 启动slam节点
sleep 3
/home/sshuser/lidar_slam.sh &

# 启动location节点
sleep 3
/home/sshuser/location.sh &

# 启动lidar_det节点
#sleep 3
#/home/sshuser/lidar_det_cluster.sh

# 启动lidar_tracker节点
sleep 1
/home/sshuser/lidar_tracker.sh &

# 启动mutisensor_fusion节点
sleep 2
/home/sshuser/mutisensor_fusion.sh &

# chassis
sleep 1 
/home/sshuser/chassis.sh &









echo "
