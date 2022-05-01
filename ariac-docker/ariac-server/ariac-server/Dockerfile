FROM ubuntu:bionic

# Hint to dpkg and apt that packages should be installed without asking for human intervention
ENV DEBIAN_FRONTEND noninteractive


# setup timezone
RUN echo 'Etc/UTC' > /etc/timezone && \
    ln -s /usr/share/zoneinfo/Etc/UTC /etc/localtime && \
    apt-get update && apt-get install -q -y tzdata && rm -rf /var/lib/apt/lists/*

# install packages
# RUN apt-get update && apt-get install -q -y \
#     dirmngr \
#     gnupg2 \
#     lsb-release \
#     && rm -rf /var/lib/apt/lists/*

RUN apt-get update \
 && apt-get install -y \
    gnupg2 \
 && apt-get clean \
 && rm -rf /var/lib/apt/lists/*


# Need these before sources can be set up
# RUN apt update \
#  && apt install -y \
#     curl \
#     gnupg2 \
#  && apt-get clean \
#  && rm -rf /var/lib/apt/lists/*

# RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -

# sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-keys F88F6D313016330404F710FC9A2FD067A2E3EF7B

# setup keys and sources for official Gazebo and ROS debian packages
RUN apt-key adv --keyserver keyserver.ubuntu.com --recv-keys D2486D2DD83DB69272AFE98867170598AF249743 \
 && echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable bionic main" > /etc/apt/sources.list.d/gazebo-latest.list \
 && apt-key adv --keyserver keyserver.ubuntu.com --recv-keys C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654 \
 && echo "deb http://packages.ros.org/ros/ubuntu bionic main" > /etc/apt/sources.list.d/ros-latest.list

# install bootstrap tools
RUN apt-get update && apt-get install --no-install-recommends -y \
    python-rosdep \
    python-rosinstall \
    python-vcstools \
    && rm -rf /var/lib/apt/lists/*

# setup environment
# ENV LANG C.UTF-8
# ENV LC_ALL C.UTF-8


# bootstrap rosdep
# RUN rosdep init \
#     && rosdep update


# install ROS and Gazebo packages
RUN apt-get update && apt-get install -q -y \
    bash-completion \
    gazebo9 \
    libgazebo9-dev \
    libopencv-contrib-dev \
    locales \
    xvfb \
    psmisc \
    ros-melodic-rqt-image-view \
    python-catkin-tools \
    ros-melodic-image-pipeline \
    ros-melodic-ros-control* \
    ros-melodic-control* \
    ros-melodic-gazebo-msgs \
    ros-melodic-gazebo-plugins \
    ros-melodic-gazebo-ros-control* \
    ros-melodic-gazebo-ros \
    ros-melodic-gazebo-ros-pkgs \
    ros-melodic-robot-state-publisher \
    ros-melodic-camera-info-manager \
    ros-melodic-ros-control \
    ros-melodic-ros-controllers \
    ros-melodic-perception \
    ros-melodic-ros-core \
    ros-melodic-desktop-full \
    ros-melodic-image-transport \
    ros-melodic-cv-bridge \
    ros-melodic-joint-limits-interface \
    ros-melodic-polled-camera \
    ros-melodic-diagnostic-updater \
    ros-melodic-transmission-interface \
    ros-melodic-ros-base \
    ros-melodic-desktop-full \
    ros-melodic-moveit \
    libcanberra-gtk* \
    software-properties-common \
    apt-utils \
    git \
    # curl \
    ca-certificates \
    bzip2 \
    tree \
    htop \
    bmon \
    wget \
    mesa-utils \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# RUN add-apt-repository ppa:oibaf/graphics-drivers && apt-get update 
# RUN apt-get install mesa && apt-get clean && rm -rf /var/lib/apt/lists/*


# Other things for ROS
RUN apt-get update && apt-get install -q -y \
	ninja-build python-pip python-dev python-wstool \
    && rm -rf /var/lib/apt/lists/*

# RUN pip install -U \
#     catkin-tools \
#     jinja2

# bootstrap rosdep
RUN rosdep init

# Need to install a specific version of gazebo_ros_pkgs
# 1. Remove official packages
RUN export GZ_VERSION=9 && \
    dpkg -r --force-depends ros-melodic-gazebo${GZ_VERSION}-ros-pkgs \
                            ros-melodic-gazebo${GZ_VERSION}-ros \
                            ros-melodic-gazebo${GZ_VERSION}-plugins \
                            ros-melodic-gazebo${GZ_VERSION}-msgs \
                            ros-melodic-gazebo${GZ_VERSION}-ros-control


RUN sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
RUN wget https://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
RUN apt-get update
RUN apt-get update -y




# 3. Build the version from source
RUN mkdir -p /tmp/ros_ws/src
RUN /bin/bash -c "source /opt/ros/melodic/setup.bash && \
                  cd /tmp/ros_ws/src && \
                  catkin_init_workspace"



# Expose port used to communicate with gzserver
EXPOSE 11345

# CPU rendering with LLVMPipe
# from https://www.openrobots.org/morse/doc/stable/headless.html

# RUN apt-get update && apt-get install -q -y \
# 	llvm-dev scons python-mako libedit-dev flex wget bison \
#     && rm -rf /var/lib/apt/lists/*

# COPY ./build_mesa.sh /tmp
# RUN /tmp/build_mesa.sh && rm /tmp/build_mesa.sh

# RUN apt-get update && apt-get install -q -y \
# 	ffmpeg xvfb vim tmux htop xdotool rsync \
#     && rm -rf /var/lib/apt/lists/*

# RUN pip install -U \
#     numpy toml

# Stuff for nvidia-docker
LABEL com.nvidia.volumes.needed="nvidia_driver"
ENV PATH /usr/local/nvidia/bin:${PATH}
ENV LD_LIBRARY_PATH /usr/local/nvidia/lib:/usr/local/nvidia/lib64:${LD_LIBRARY_PATH}

# Set encoding to use unicode characters
RUN locale-gen en_US.UTF-8
ENV LANG en_US.UTF-8

# COPY ./build_mesa.sh /tmp
# RUN /tmp/build_mesa.sh && rm /tmp/build_mesa.sh

# Create a new user called ariac. Note: we don't add them to the sudo group
ENV USERNAME ariac
ARG USERID=1000
RUN adduser -u $USERID --gecos "Development User" --disabled-password $USERNAME
RUN echo "export QT_X11_NO_MITSHM=1" >> /home/$USERNAME/.bashrc

USER $USERNAME
WORKDIR /home/$USERNAME

# Get gazebo models early since it is big

RUN mkdir -p $HOME/.gazebo/models
RUN git clone \
    https://github.com/osrf/gazebo_models $HOME/.gazebo/models


 #Download and compile ARIAC 2022
 RUN mkdir -p /home/ariac/ariac_ws/src
 RUN mkdir -p /tmp/gazebo_ws
 RUN mkdir -p /tmp/ariac_ws

 RUN git clone \
      https://github.com/osrf/ariac-gazebo_ros_pkgs.git /tmp/gazebo_ws \
      -b ariac-network-melodic

 RUN git clone \
    https://github.com/usnistgov/ARIAC.git  /tmp/ariac_ws \
    -b ariac2022
 RUN /bin/bash -c "cp -rf /tmp/gazebo_ws /home/ariac/ariac_ws/src"
 RUN /bin/bash -c "cp -rf /tmp/ariac_ws /home/ariac/ariac_ws/src"

 RUN /bin/bash -c "rm -rf /tmp/ariac_ws"
 RUN /bin/bash -c "rm -rf /tmp/gazebo_ws"

 RUN /bin/bash -c "source /opt/ros/melodic/setup.bash && \
                   cd /home/ariac/ariac_ws && \
                   catkin_make"


# setup entrypoint
COPY ./ariac_entrypoint.sh /
COPY ./run_ariac_task.sh /


ENTRYPOINT ["/ariac_entrypoint.sh"]
