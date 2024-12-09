FROM osrf/ros:humble-desktop
MAINTAINER Marcin Czajka <czajka.m.marcin@gmail.com>
# Based on https://github.com/RafalStaszak/NIMPRA_Docker
RUN echo "Europe/Utc" > /etc/timezone
# RUN ln -fs /usr/share/zoneinfo/Europe/Rome /etc/localtime
RUN apt-get update -q && \
        export DEBIAN_FRONTEND=noninteractive && \
    apt-get install -y --no-install-recommends tzdata
RUN dpkg-reconfigure -f noninteractive tzdata
# Install packages
# Install necessary packages
RUN apt-get update -q && \
    export DEBIAN_FRONTEND=noninteractive && \
    apt-get install -y --no-install-recommends \
        apt-utils \
        software-properties-common \
        wget \
        curl \
        rsync \
        netcat \
        mg \
        vim \
        bzip2 \
        zip \
        unzip \
        libxtst6 \
        bash-completion \
        nano \
        net-tools \
        iputils-ping \
        terminator \
        ros-dev-tools && \
    apt-get install -y ros-humble-turtlebot3-gazebo && \
    apt-get install -y ros-humble-nav2-bringup && \
    apt-get install -y ros-humble-navigation2 && \
    apt-get autoclean -y && \
    apt-get autoremove -y && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*
RUN apt-get update -q && \
        export DEBIAN_FRONTEND=noninteractive && \
    rm -rf /var/lib/apt/lists/*
RUN sed -i 's/--no-generate//g' /usr/share/bash-completion/completions/apt-get && \
    sed -i 's/--no-generate//g' /usr/share/bash-completion/completions/apt-cache
WORKDIR /root/

RUN sed -i "s/#force_color_prompt=yes/force_color_prompt=yes/g" /root/.bashrc

RUN echo 'if [ -f /etc/bash_completion ] && ! shopt -oq posix; then \n\
    . /etc/bash_completion \n\
fi \n\
\n\
export USER=root \n\
source /opt/ros/humble/setup.bash \n\
export TURTLEBOT3_MODEL=waffle \n\
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/humble/share/turtlebot3_gazebo/models \n\
source /usr/share/gazebo/setup.bash \n\
source /usr/share/gazebo-11/setup.bash' >> /root/.bashrc

RUN touch /root/.Xauthority
