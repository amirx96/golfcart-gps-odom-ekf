#!/bin/bash

rosdep install -y -r --from-path /robot_ekf_ws
cd /robot_ekf_ws
echo "building project"
catkin build
echo 'source /robot_ekf_ws/devel/setup.bash' >> /root/.bashrc

