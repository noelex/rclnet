FROM ros:rolling-ros-core
RUN echo "from urllib import request\nrequest.urlretrieve('https://packages.microsoft.com/config/ubuntu/22.04/packages-microsoft-prod.deb', 'packages-microsoft-prod.deb')" | python3  \
    && dpkg -i packages-microsoft-prod.deb \
    && rm packages-microsoft-prod.deb \
    && apt-get update \
    && apt-get -y install --no-install-recommends \
       ros-rolling-rmw-cyclonedds-cpp \
       ros-rolling-rmw-fastrtps-cpp \
       ros-rolling-tf2-msgs \
       dotnet-runtime-7.0 \
       ros-rolling-service-msgs \
    && apt-get install -y --only-upgrade ros-rolling-* \
    && apt-get autoremove -y \
    && apt-get clean -y \
    && rm -rf /var/lib/apt/lists/*