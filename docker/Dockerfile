ARG BASE_ROS_IMAGE=osrf/ros
ARG ROS_DISTRO=foxy
ARG BASE_GAZEBO_IMAGE=gazebo
ARG GAZEBO_TAG=libgazebo11

FROM ${BASE_GAZEBO_IMAGE}:${GAZEBO_TAG}
FROM ${BASE_ROS_IMAGE}:${ROS_DISTRO}-desktop

# Update apt lists and install some basics
RUN apt-get update && apt-get upgrade -y
RUN apt-get install -y \
    wget \
    git \
    bash-completion \
    build-essential \
    sudo \
    libgl1-mesa-glx libgl1-mesa-dri

# Create the same user as the host itself
RUN addgroup ros
RUN adduser --gecos "ROS User" --disabled-password --ingroup ros ros
RUN usermod -a -G dialout ros
ADD docker/config/99_aptget /etc/sudoers.d/99_aptget
RUN chmod 0440 /etc/sudoers.d/99_aptget && chown root:root /etc/sudoers.d/99_aptget

# Run as created user
ENV USER ros
USER ros
ENV HOME /home/${USER} 

# Workspace setup
RUN mkdir -p ${HOME}/ros_ws/src
WORKDIR ${HOME}/ros_ws
RUN cd ${HOME}/ros_ws && . /opt/ros/${ROS_DISTRO}/setup.sh
COPY ./jackal_ws/src ${HOME}/ros_ws/src
RUN sudo apt-get update \
    && rosdep update \
    && rosdep install --from-paths src --ignore-src --rosdistro ${ROS_DISTRO} -y --skip-keys "console_bridge fastcdr fastrtps rti-connext-dds-5.3.1 urdfdom_headers" \
    &&. /opt/ros/${ROS_DISTRO}/setup.sh \
    && colcon build --symlink-install

# Set up environment
COPY ./docker/config/update_bashrc /sbin/update_bashrc
RUN sudo chmod +x /sbin/update_bashrc ; sudo chown ros /sbin/update_bashrc ; sync ; /bin/bash -c /sbin/update_bashrc ; sudo rm /sbin/update_bashrc
# Change entrypoint to source ~/.bashrc and start in ~
COPY ./docker/config/entrypoint.sh /ros_entrypoint.sh
RUN sudo chmod +x /ros_entrypoint.sh ; sudo chown ros /ros_entrypoint.sh ;

# Clean image
RUN sudo apt-get clean && sudo rm -rf /var/lib/apt/lists/* 
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]