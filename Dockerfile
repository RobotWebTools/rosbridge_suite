ARG DOCKER_ROS_DISTRO=noetic
FROM ros:${DOCKER_ROS_DISTRO}

# Booststrap workspace.
ENV CATKIN_DIR=/catkin_ws
RUN . /opt/ros/$ROS_DISTRO/setup.sh \
 && mkdir -p $CATKIN_DIR/src \
 && cd $CATKIN_DIR/src \
 && catkin_init_workspace
WORKDIR $CATKIN_DIR

# Install dependencies first.
COPY rosbridge_suite/package.xml $CATKIN_DIR/src/rosbridge_suite/
COPY rosbridge_library/package.xml $CATKIN_DIR/src/rosbridge_library/
COPY rosbridge_server/package.xml $CATKIN_DIR/src/rosbridge_server/
COPY rosbridge_msgs/package.xml $CATKIN_DIR/src/rosbridge_msgs/
COPY rosapi/package.xml $CATKIN_DIR/src/rosapi/
RUN . /opt/ros/$ROS_DISTRO/setup.sh \
 && apt-get update \
 && rosdep update \
 && rosdep install \
    --from-paths src \
    --ignore-src \
    --rosdistro $ROS_DISTRO \
    -y \
 && rm -rf /var/lib/apt/lists/*

# Build rosbridge packages.
COPY rosbridge_suite $CATKIN_DIR/src/rosbridge_suite
COPY rosbridge_library $CATKIN_DIR/src/rosbridge_library
COPY rosbridge_server $CATKIN_DIR/src/rosbridge_server
COPY rosbridge_msgs $CATKIN_DIR/src/rosbridge_msgs
COPY rosapi $CATKIN_DIR/src/rosapi
RUN . /opt/ros/$ROS_DISTRO/setup.sh \
 && catkin_make

# We want the development workspace active all the time.
RUN echo "#!/bin/bash\n\
set -e\n\
source \"${CATKIN_DIR}/devel/setup.bash\"\n\
exec \"\$@\"" > /startup.sh \
 && chmod a+x /startup.sh \
 && echo "source ${CATKIN_DIR}/devel/setup.bash" >> /root/.bashrc
ENTRYPOINT ["/startup.sh"]
CMD ["bash"]
