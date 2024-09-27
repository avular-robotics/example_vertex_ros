FROM ros:humble

SHELL ["/bin/bash", "-c"]

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update \
    && DEBIAN_FRONTEND=noninteractive apt-get install -qq -y --no-install-recommends \
    build-essential \
    cmake \
    git \
    libssl-dev \
    libusb-1.0-0-dev \
    pkg-config \
    libgtk-3-dev \
    libglfw3-dev \
    libgl1-mesa-dev \
    libglu1-mesa-dev \
    curl \
    nano iproute2 vim htop \
    net-tools \
    python3 \
    python3-dev \
    python3-pip \
    ca-certificates \
    software-properties-common \
    python3-rosdep \
    ros-${ROS_DISTRO}-rviz2 \
    ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
    ros-${ROS_DISTRO}-nav-msgs \
    bash-completion \
    && rm -rf /etc/apt/apt.conf.d/docker-clean \
    # Setup Rosdep
    && rm /etc/ros/rosdep/sources.list.d/20-default.list \
    && rosdep init \
    && rm -rf /var/lib/apt/lists/*


# Add user user and switch to home directory
RUN useradd --create-home --shell /bin/bash --groups sudo user \
    # Enable passwordless sudo
    && echo "user ALL=(ALL) NOPASSWD: ALL" >> /etc/sudoers.d/90-user \
    # Show the container name in the terminal
    && echo 'export PS1="\[\033[01;32m\]\u@\h\[\033[01;33m\][user]\[\033[00m\]:\[\033[01;34m\]\w\[\033[00m\]$ "' >> /home/user/.bashrc 
USER user

# Setup ros
RUN source /opt/ros/${ROS_DISTRO}/setup.sh \
    && rosdep update \
    && echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc

USER root

# Setup entrypoint
COPY entrypoint.sh /
RUN sudo chmod 0755 /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]

# Setup message definitions
COPY autonomy-msgs_arm64_2.2.0.deb /
COPY cmake-avular_arm64_3.0.0.deb / 
COPY ament-copyright-avular_arm64_3.0.0.deb /
RUN apt update && apt install -y /ament-copyright-avular_arm64_3.0.0.deb
RUN apt update && apt install -y /cmake-avular_arm64_3.0.0.deb
RUN apt update && apt install -y /autonomy-msgs_arm64_2.2.0.deb

WORKDIR /home/user/ws
RUN source /opt/ros/${ROS_DISTRO}/setup.sh

# Install extra dependencies
# RUN sudo apt update && sudo apt install -y \
#     <package you want to install>

USER user
WORKDIR /home/user/ws
CMD ["/bin/bash"]