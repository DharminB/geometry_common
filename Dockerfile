FROM ros:jazzy

SHELL [ "/bin/bash", "-c" ]

# Install dependencies
RUN apt-get -qq update > /dev/null

# Copy the geometry_common source code to the docker container
WORKDIR /workspace/ros2_ws/src/geometry_common
COPY . /workspace/ros2_ws/src/geometry_common/
