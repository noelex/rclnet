FROM ros:foxy-ros-core
RUN echo "from urllib import request\nrequest.urlretrieve('https://packages.microsoft.com/config/ubuntu/20.04/packages-microsoft-prod.deb', 'packages-microsoft-prod.deb')" | python3  \
    && dpkg -i packages-microsoft-prod.deb \
    && rm packages-microsoft-prod.deb \
    && apt-get update \
    && apt-get -y install --no-install-recommends \
       build-essential \
       cmake \
       python3-colcon-common-extensions \
       ros-foxy-action-msgs \
       ros-foxy-rmw-cyclonedds-cpp \
       ros-foxy-rmw-fastrtps-cpp \
       ros-foxy-rosidl-default-generators \
       ros-foxy-tf2-msgs \
       ros-foxy-unique-identifier-msgs \
       wget \
    && apt-get autoremove -y \
    && apt-get clean -y \
    && rm -rf /var/lib/apt/lists/*
RUN wget https://dot.net/v1/dotnet-install.sh -O dotnet-install.sh  \
    && chmod +x ./dotnet-install.sh \
    && ./dotnet-install.sh --channel 10.0 \
    && ./dotnet-install.sh --channel 9.0 --runtime dotnet \
    && ./dotnet-install.sh --channel 8.0 --runtime dotnet

COPY src/CodegenTests/packages/ros2cs_abi_test_msgs /opt/rclnet-test-ws/src/ros2cs_abi_test_msgs
RUN . /opt/ros/foxy/setup.sh \
    && cd /opt/rclnet-test-ws \
    && colcon build --merge-install --packages-select ros2cs_abi_test_msgs

ENV AMENT_PREFIX_PATH=/opt/rclnet-test-ws/install
ENV LD_LIBRARY_PATH=/opt/rclnet-test-ws/install/lib
ENV DOTNET_ROOT=/root/.dotnet
ENV PATH=$PATH:$DOTNET_ROOT:$DOTNET_ROOT/tools
