FROM ros:jazzy

SHELL [ "/bin/bash", "-c" ]

# Install dependencies
RUN apt-get -qq update > /dev/null

# Copy the geometry_common source code to the docker container
WORKDIR /workspace/ros2_ws/src/geometry_common
COPY . .

# Build
WORKDIR /workspace/ros2_ws
RUN /ros_entrypoint.sh colcon build

# Test
RUN . install/setup.bash && colcon test --packages-up-to geometry_common --event-handlers console_direct+
RUN mkdir /test_results
RUN cp /workspace/ros2_ws/build/geometry_common/test_results/geometry_common/geometry_common_test.gtest.xml /test_results/
