# Use the official ROS 2 Jazzy base image
FROM ros:jazzy-ros-base

# Set noninteractive frontend
ENV DEBIAN_FRONTEND=noninteractive

# Setup locales
RUN apt-get update && apt-get install -y locales \
    && locale-gen en_US en_US.UTF-8 \
    && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 \
    && export LANG=en_US.UTF-8

RUN apt-get update && apt-get install -y \
    git wget curl gnupg2 lsb-release \
    build-essential cmake udev sudo \
    software-properties-common \
    && rm -rf /var/lib/apt/lists/

# Add Intel RealSense package repository
#RUN apt-key adv --keyserver keyserver.ubuntu.com --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE && \
#    add-apt-repository "deb http://realsense-hw-public.s3.amazonaws.com/Debian/apt-repo $(lsb_release -cs) main"

# Install necessary packages
RUN apt-get update && apt-get install -y \
	build-essential \
	ros-jazzy-ros2controlcli \
	ros-jazzy-rqt \
	ros-jazzy-rqt-graph \
	ros-jazzy-ros2-control \
	ros-jazzy-controller-manager \
	ros-jazzy-rviz2 \
	python3-vcstool \
	libeigen3-dev \
	libopencv-dev \
	libboost-all-dev \
	libpcl-dev \
	vim \
	nano \
	git \
	iputils-ping \
	python3-colcon-common-extensions \
	x11-apps \	
        ros-jazzy-realsense2-camera \
        ros-jazzy-image-tools \
        #ros-jazzy-apriltag-ros \
	&& rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y \
    python3-scipy \
    ros-jazzy-rmw-zenoh-cpp \
    && rm -rf /var/lib/apt/lists/*

# RUN   sed -i "s|size: 0.173|size: 0.69|g" /opt/ros/jazzy/share/apriltag_ros/cfg/tags_36h11.yaml

# Create workspace directory
WORKDIR /home/P7_drone_sit_awareness_ws/

# Automatically source ROS and workspace setups in bashrc
RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc && \
    echo "source /home/P7_drone_sit_awareness_ws/install/setup.bash" >> /root/.bashrc

SHELL ["/bin/bash", "-c"]
ENV ROS_DOMAIN_ID=42

#CMD ["bash", "-c", "source", "opt/ros/jazzy/setup.bash"] 
#CMD ["bash", "-c", "cd", "home/P5-Dual_Arm"]
#CMD ["bash", "-c", "source", "install/setup.bash"]
#ENTRYPOINT ["export", "ROS_DOMAIN_ID=69"]
