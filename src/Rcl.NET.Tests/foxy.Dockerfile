FROM ros:foxy-ros-core
RUN echo "from urllib import request\nrequest.urlretrieve('https://packages.microsoft.com/config/ubuntu/20.04/packages-microsoft-prod.deb', 'packages-microsoft-prod.deb')" | python3  \
    && dpkg -i packages-microsoft-prod.deb \
    && rm packages-microsoft-prod.deb \
    && apt-get update \
    && apt-get -y install --no-install-recommends \
       ros-foxy-rmw-cyclonedds-cpp \
       ros-foxy-rmw-fastrtps-cpp \
       ros-foxy-tf2-msgs \
       dotnet-runtime-7.0 \
       dotnet-runtime-8.0 \
    && apt-get autoremove -y \
    && apt-get clean -y \
    && rm -rf /var/lib/apt/lists/*