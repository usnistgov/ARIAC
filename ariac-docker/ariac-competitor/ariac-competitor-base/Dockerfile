FROM osrf/ros:melodic-desktop-full

USER root

ENV DEBIAN_FRONTEND noninteractive


# RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

# install bootstrap tools
RUN apt-get update -y
RUN apt-get install -y apt-utils

RUN apt-get update && apt-get install --no-install-recommends -y \
    python-rosdep \
    lsb-release \
    ros-melodic-rqt-image-view \
    ros-melodic-image-pipeline \
    mesa-utils \
    python-rosinstall \
    libcanberra-gtk* \
    python-vcstools \
    python-rospkg \
    wget \
    && rm -rf /var/lib/apt/lists/*
RUN wget http://packages.osrfoundation.org/gazebo.key -O - | apt-key add -
RUN echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list



RUN apt-get update && apt-get install -y \
        ros-${ROS_DISTRO}-desktop-full \
        ros-${ROS_DISTRO}-ros-controllers \
        ros-${ROS_DISTRO}-ros-control \
        ros-${ROS_DISTRO}-moveit \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*


RUN rosdep update

# Create a new user called ariac-user and give them sudo priviledges.
ENV USERNAME ariac-user
RUN adduser --gecos "Development User" --disabled-password $USERNAME
RUN adduser $USERNAME sudo

#Download and compile ARIAC 2022
RUN mkdir -p /home/ariac-user/ariac_ws/src
RUN mkdir -p /tmp/gazebo_ws

RUN git clone \
      https://github.com/osrf/ariac-gazebo_ros_pkgs.git /tmp/gazebo_ws \
      -b ariac-network-melodic
RUN git clone \ 
    https://github.com/usnistgov/ARIAC.git /home/ariac-user/ariac_ws/src \
    -b ariac2022
RUN /bin/bash -c "cp -rf /tmp/gazebo_ws /home/ariac-user/ariac_ws/src"

RUN /bin/bash -c "source /opt/ros/melodic/setup.bash && \
                  cd /home/ariac-user/ariac_ws && \
                  catkin_make"                 

COPY ./competitor_base_entrypoint.sh /

WORKDIR /home/$USERNAME

ENTRYPOINT ["/competitor_base_entrypoint.sh"]
