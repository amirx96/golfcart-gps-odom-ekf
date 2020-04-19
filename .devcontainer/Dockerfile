FROM ros:kinetic-ros-base

RUN apt-get update && apt-get install -q -y \
   python-catkin-tools python-pip mesa-utils ros-kinetic-pacmod

RUN echo 'source /opt/ros/$ROS_DISTRO/setup.bash' >> /root/.bashrc