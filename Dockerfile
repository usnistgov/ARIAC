# ================================================================
# Base ARIAC image with ROS 2 Jazzy and Gazebo Harmonic
# ================================================================

FROM osrf/ros:jazzy-desktop

# Locale
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8

# Install dependencies: build tools, rosdep, colcon, vcs, etc.
RUN apt-get update && apt-get install -y \
    build-essential git python3-pip \
    python3-colcon-common-extensions python3-rosdep python3-vcstool \
    curl lsb-release gnupg

# Install gz-harmonic
RUN curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] https://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null && \
    apt-get update && apt-get install -y gz-harmonic

# Initialize rosdep
RUN rosdep init || true

RUN apt-get update && apt-get upgrade -y && rosdep update

# ================================================================
# Create ARIAC workspace
# ================================================================
ENV ROS_WS=/ariac_ws
RUN mkdir -p $ROS_WS/src
WORKDIR $ROS_WS

# Copy ARIAC packages into workspace
COPY ./ $ROS_WS/src/

RUN pip3 install -r $ROS_WS/src/ariac_app/requirements.txt --break-system-packages

# ================================================================
# Resolve dependencies with rosdep
# ================================================================
RUN rosdep install --from-paths src --ignore-src -r -y

# ================================================================
# Build workspace
# ================================================================
RUN /bin/bash -c "source /opt/ros/jazzy/setup.bash && colcon build --symlink-install"

# ================================================================
# Default setup
# ================================================================
ENV ROS_DISTRO=jazzy
RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc && \
    echo "source $ROS_WS/install/setup.bash" >> /root/.bashrc

# Now clean up apt caches (safe here, after rosdep ran)
RUN rm -rf /var/lib/apt/lists/*

WORKDIR $ROS_WS
CMD ["bash"]
