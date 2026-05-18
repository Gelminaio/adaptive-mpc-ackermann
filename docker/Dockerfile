FROM osrf/ros:jazzy-desktop

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    git \
    wget \
    curl \
    nano \
    vim \
    python3-pip \
    python3-dev \
    python3-venv \
    python3-numpy \
    python3-scipy \
    python3-matplotlib \
    python3-yaml \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    libopencv-dev \
    python3-opencv \
    ros-jazzy-rmw-cyclonedds-cpp \
    ros-jazzy-rviz2 \
    ros-jazzy-rqt \
    ros-jazzy-tf2-tools \
    ros-jazzy-robot-state-publisher \
    ros-jazzy-xacro \
    && rm -rf /var/lib/apt/lists/*

ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

ARG USERNAME=dev
ARG USER_UID=1000
ARG USER_GID=1000

RUN apt-get update && apt-get install -y sudo && rm -rf /var/lib/apt/lists/* \
    && EXISTING_USER=$(getent passwd $USER_UID | cut -d: -f1) \
    && if [ -n "$EXISTING_USER" ]; then \
         echo "Renaming existing user $EXISTING_USER -> $USERNAME"; \
         usermod -l $USERNAME -d /home/$USERNAME -m $EXISTING_USER; \
         groupmod -n $USERNAME $(getent group $USER_GID | cut -d: -f1); \
       else \
         groupadd --gid $USER_GID $USERNAME; \
         useradd --uid $USER_UID --gid $USER_GID -m -s /bin/bash $USERNAME; \
       fi \
    && echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

RUN echo "source /opt/ros/jazzy/setup.bash" >> /home/$USERNAME/.bashrc \
    && echo "[ -f /workspace/ros2_ws/install/setup.bash ] && source /workspace/ros2_ws/install/setup.bash" >> /home/$USERNAME/.bashrc

USER $USERNAME
WORKDIR /workspace

CMD ["bash"]