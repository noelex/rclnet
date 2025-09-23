FROM ros:foxy-ros-core
RUN echo "from urllib import request\nrequest.urlretrieve('https://packages.microsoft.com/config/ubuntu/20.04/packages-microsoft-prod.deb', 'packages-microsoft-prod.deb')" | python3  \
    && dpkg -i packages-microsoft-prod.deb \
    && rm packages-microsoft-prod.deb \
    && apt-get update \
    && apt-get -y install --no-install-recommends \
       ros-foxy-rmw-cyclonedds-cpp \
       ros-foxy-rmw-fastrtps-cpp \
       ros-foxy-tf2-msgs \
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