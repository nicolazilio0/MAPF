FROM amd64/ubuntu:jammy
ARG DEBIAN_FRONTEND=noninteractive

# copy all the files to the container
ADD ./shelfino_node/ /root/ros2_ws/src/shelfino_node
ADD ./shelfino_gazebo/ /root/ros2_ws/src/shelfino_gazebo
ADD ./shelfino_navigation/ /root/ros2_ws/src/shelfino_navigation
ADD ./shelfino_description/ /root/ros2_ws/src/shelfino_description
ADD ./remote_control/ /root/ros2_ws/src/remote_control
ADD startup.sh /root/startup.sh

# setup entrypoint
COPY ./ros_entrypoint.sh /
RUN bash -c "chmod +x /ros_entrypoint.sh"

# Set locale
RUN apt-get update && apt-get install -y locales && \
    apt-get install apt-utils && \
    locale-gen en_US en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 && \
    export LANG=en_US.UTF-8

# Setup source
RUN apt-get install -y software-properties-common && \
	add-apt-repository -y universe && \
    apt-get update && apt-get install -y curl gnupg lsb-release && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu jammy main" |  tee /etc/apt/sources.list.d/ros2.list > /dev/null 

# Install ROS2 packages
RUN apt-get update && apt-get upgrade -y && \
    apt-get install -y ros-humble-desktop && \
    apt-get install -y python3-colcon-common-extensions && \
    apt-get install -y libzmq3-dev && \
    apt-get install -y libncurses5-dev && \
    apt-get install -y mesa-utils libgl1-mesa-glx && \
    apt-get install -y ros-humble-nav2-bringup && \
    apt-get install -y ros-humble-cartographer && \
    apt-get install -y ros-humble-cartographer-ros && \
    apt-get install -y ros-humble-gazebo-ros && \
    apt-get install -y ros-humble-gazebo-ros-pkgs && \
    apt-get install -y ros-humble-test-msgs && \
    apt-get install -y ros-humble-xacro && \
    apt-get install -y net-tools && \
    apt-get install -y iproute2 

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]

# export LIBGL_ALWAYS_INDIRECT=1
# docker run --rm -it --gpus all --env="DISPLAY=host.docker.internal:0" -v /tmp/.X11-unix:/tmp/.X11-unix:rw f7c15b19d565 /bin/bash