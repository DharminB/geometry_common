FROM ros:noetic

SHELL [ "/bin/bash", "-c" ]

# Install dependencies
RUN apt-get -qq update > /dev/null && \
    apt-get -yqq install sudo \
                         python3-catkin-tools \
                         ros-$ROS_DISTRO-catkin \
                         ros-$ROS_DISTRO-tf \
                         ros-$ROS_DISTRO-tf2-geometry-msgs > /dev/null && \
    apt-get clean > /dev/null

# Copy the geometry_common source code to the docker container
WORKDIR /workspace/catkin_ws/src/geometry_common
COPY . /workspace/catkin_ws/src/geometry_common/

# Compile the ROS catkin workspace
# RUN cd /workspace/catkin_ws && \
#     /ros_entrypoint.sh catkin build --no-status

# Run unit tests
# RUN source /workspace/catkin_ws/devel/setup.bash && \
#     cd /workspace/catkin_ws/src/geometry_common && \
#     /ros_entrypoint.sh catkin build --this --no-status --catkin-make-args run_tests -- && \
#     rosrun geometry_common geometry_common_test
