FROM ros:humble

ARG USERNAME=USERNAME
ARG USER_UID=1000
ARG USER_GID=$USER_UID
ARG TARGETARCH
ARG CREOS_VERSION

SHELL ["/bin/bash", "-c"]

# Delete user if it exists in container (e.g Ubuntu Noble: ubuntu)
RUN if id -u $USER_UID ; then userdel `id -un $USER_UID` ; fi

# Switch to root user to install dependencies
USER root
COPY ros.dependencies.txt /tmp/ros.dependencies.txt
RUN apt update \
    && apt install -y $(cat /tmp/ros.dependencies.txt) \
    && apt clean \
    && rm -rf /var/lib/apt/lists/* /tmp/ros.dependencies.txt \
    # Enable bash completion for apt
    && rm -rf /etc/apt/apt.conf.d/docker-clean \
    # Setup Rosdep
    && rm /etc/ros/rosdep/sources.list.d/20-default.list \
    && rosdep init \
    && rm -rf /var/lib/apt/lists/*

# Create the user
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && apt-get update \
    && apt-get install -y sudo \
    # Enable sudo without password
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME \
    # Show the container name in the terminal
    && echo 'export PS1="\[\033[01;32m\]\u@\h\[\033[01;33m\][user]\[\033[00m\]:\[\033[01;34m\]\w\[\033[00m\]$ "' >> /home/$USERNAME/.bashrc

USER $USERNAME

# Setup ros
RUN source /opt/ros/${ROS_DISTRO}/setup.sh \
    && rosdep update \
    && echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc

USER root

# Setup entrypoint
COPY docker/ros.entrypoint.sh /entrypoint.sh
RUN sudo chmod 0755 /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]

# Install Creos
RUN wget https://avular.blob.core.windows.net/creos/creos_sdk_${CREOS_VERSION}_jammy_${TARGETARCH}.zip \
    && unzip creos_sdk_${CREOS_VERSION}_jammy_${TARGETARCH}.zip \
    && apt update \
    && cd creos_sdk_client_package_Release_jammy_${TARGETARCH} \
    && dpkg -i creos-utils_*_${TARGETARCH}.deb \
    && dpkg -i creos-client_*_${TARGETARCH}.deb \
    && dpkg -i creos-sdk-ros_*_${TARGETARCH}.deb \
    && cd .. \
    && rm -rf creos_sdk_client_package_Release_jammy_${TARGETARCH} \
    && rm creos_sdk_${CREOS_VERSION}_jammy_${TARGETARCH}.zip

USER $USERNAME
WORKDIR /home/$USERNAME/ws
CMD ["/bin/bash"]
