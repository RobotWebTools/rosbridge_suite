#!/usr/bin/env bash
# This script will build a Docker image with rosbridge in a catkin workspace,
# then run it with host networking and the sources mounted read-only.

cd "$(dirname "${BASH_SOURCE[0]}")"

CATKIN_DIR=/catkin_ws

docker build -t rosbridge_dev .
docker run -it --rm --net=host \
  -v "$(pwd)/rosbridge_suite:${CATKIN_DIR}/src/rosbridge_suite:ro" \
  -v "$(pwd)/rosbridge_library:${CATKIN_DIR}/src/rosbridge_library:ro" \
  -v "$(pwd)/rosbridge_server:${CATKIN_DIR}/src/rosbridge_server:ro" \
  -v "$(pwd)/rosbridge_msgs:${CATKIN_DIR}/src/rosbridge_msgs:ro" \
  -v "$(pwd)/rosapi:${CATKIN_DIR}/src/rosapi:ro" \
  rosbridge_dev $@
