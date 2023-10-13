#!/bin/bash

set -e

# 设置ros的环境
source "/opt/ros/melodic/setup.bash"
source "/root/catkin_ws/devel/setup.bash"

# build mmdet
echo "=============================="

cd /root/catkin_ws/src/mms_slam/dependency/mmdet
python setup.py install


echo "================ MMS-SLAM Ready =============="


cd /root/catkin_ws

exec "$@"
