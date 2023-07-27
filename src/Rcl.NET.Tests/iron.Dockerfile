FROM ros:iron-ros-core
RUN echo "from urllib import request\nrequest.urlretrieve('https://packages.microsoft.com/config/ubuntu/22.04/packages-microsoft-prod.deb', 'packages-microsoft-prod.deb')" | python3  \
    && dpkg -i packages-microsoft-prod.deb \
    && rm packages-microsoft-prod.deb \
    && apt-get update \
    && apt-get -y install --no-install-recommends \
       ros-iron-rmw-cyclonedds-cpp \
       ros-iron-rmw-fastrtps-cpp \
       ros-iron-tf2-msgs \
       dotnet-runtime-7.0 \
       ros-iron-service-msgs \
    && apt-get autoremove -y \
    && apt-get clean -y \
    && rm -rf /var/lib/apt/lists/*