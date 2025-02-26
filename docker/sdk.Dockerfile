FROM ubuntu:jammy

ARG TARGETARCH

SHELL ["/bin/bash", "-c"]

# Switch to root user to install dependencies
USER root
COPY sdk.dependencies.txt /tmp/sdk.dependencies.txt
RUN apt update \
    && apt install -y $(cat /tmp/sdk.dependencies.txt) \
    && apt clean \
    && rm -rf /var/lib/apt/lists/* /tmp/sdk.dependencies.txt \
    # Enable bash completion for apt
    && rm -rf /etc/apt/apt.conf.d/docker-clean

# Add user user and switch to home directory
RUN useradd --create-home --shell /bin/bash --groups sudo user \
    # Enable passwordless sudo
    && echo "user ALL=(ALL) NOPASSWD: ALL" >> /etc/sudoers.d/90-user \
    # Show the container name in the terminal
    && echo 'export PS1="\[\033[01;32m\]\u@\h\[\033[01;33m\][user]\[\033[00m\]:\[\033[01;34m\]\w\[\033[00m\]$ "' >> /home/user/.bashrc 

USER root

# Setup entrypoint
COPY docker/sdk.entrypoint.sh /entrypoint.sh
RUN sudo chmod 0755 /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]

# Install creos-sdk
COPY sdk/sdk_bin/creos-*_${TARGETARCH}.deb /
RUN apt update && apt install -y /creos-*_${TARGETARCH}.deb

USER user
WORKDIR /home/user/ws
CMD ["/bin/bash"]
