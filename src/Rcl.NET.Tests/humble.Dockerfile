FROM ros:humble-ros-core
RUN apt-get update \
    && apt-get -y install --no-install-recommends \
       ros-humble-rmw-cyclonedds-cpp \
       ros-humble-rmw-fastrtps-cpp \
       ros-humble-tf2-msgs \
       ros-humble-rcutils \
       wget \
    && apt-get autoremove -y \
    && apt-get clean -y \
    && rm -rf /var/lib/apt/lists/*
RUN wget https://dot.net/v1/dotnet-install.sh -O dotnet-install.sh  \
    && chmod +x ./dotnet-install.sh \
    && ./dotnet-install.sh --channel 10.0 --runtime dotnet \
    && ./dotnet-install.sh --channel 9.0 --runtime dotnet \
    && ./dotnet-install.sh --channel 8.0 --runtime dotnet
ENV DOTNET_ROOT=/root/.dotnet
ENV PATH=$PATH:$DOTNET_ROOT:$DOTNET_ROOT/tools
