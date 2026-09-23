FROM ros:jazzy-ros-core
RUN apt-get update \
    && apt-get -y install --no-install-recommends \
       build-essential \
       cmake \
       python3-colcon-common-extensions \
       ros-jazzy-action-msgs \
       ros-jazzy-rmw-cyclonedds-cpp \
       ros-jazzy-rmw-fastrtps-cpp \
       ros-jazzy-rosidl-default-generators \
       ros-jazzy-tf2-msgs \
       ros-jazzy-service-msgs \
       ros-jazzy-unique-identifier-msgs \
       wget \
    && apt-get autoremove -y \
    && apt-get clean -y \
    && rm -rf /var/lib/apt/lists/* \
    && wget https://dot.net/v1/dotnet-install.sh -O dotnet-install.sh  \
    && chmod +x ./dotnet-install.sh \
    && ./dotnet-install.sh --channel 10.0 --runtime dotnet \
    && ./dotnet-install.sh --channel 9.0 --runtime dotnet \
    && ./dotnet-install.sh --channel 8.0 --runtime dotnet

COPY src/CodegenTests/packages/ros2cs_abi_test_msgs /opt/rclnet-test-ws/src/ros2cs_abi_test_msgs
RUN . /opt/ros/jazzy/setup.sh \
    && cd /opt/rclnet-test-ws \
    && colcon build --merge-install --packages-select ros2cs_abi_test_msgs

ENV AMENT_PREFIX_PATH=/opt/rclnet-test-ws/install
ENV LD_LIBRARY_PATH=/opt/rclnet-test-ws/install/lib
ENV DOTNET_ROOT=/root/.dotnet
ENV PATH=$PATH:$DOTNET_ROOT:$DOTNET_ROOT/tools
