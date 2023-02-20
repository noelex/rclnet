ARG DISTRO

FROM althack/ros2:${DISTRO}-dev

ARG MS_PACKAGE_FEED
ARG DISTRO

# Install .NET SDK
ENV DEBIAN_FRONTEND=noninteractive
COPY "sources/${DISTRO}/ros2.list" "/etc/apt/sources.list.d/"
COPY "sources/${DISTRO}/sources.list" "/etc/apt/"

RUN apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
RUN wget "${MS_PACKAGE_FEED}" -O packages-microsoft-prod.deb \
    && dpkg -i packages-microsoft-prod.deb \
    && rm packages-microsoft-prod.deb \
    && apt-get update \
    && apt-get -y install --no-install-recommends \
    ros-${DISTRO}-rmw-cyclonedds-cpp \
    ros-${DISTRO}-turtlesim \
    dotnet-sdk-7.0 \
    && apt-get autoremove -y \
    && apt-get clean -y \
    && rm -rf /var/lib/apt/lists/*
RUN pip install rosdepc
ENV DEBIAN_FRONTEND=dialog

ARG WORKSPACE
RUN echo "if [ -f ${WORKSPACE}/install/setup.bash ]; then source ${WORKSPACE}/install/setup.bash; fi" >> /home/ros/.bashrc
RUN git config --global --add safe.directory $WORKSPACE

ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

ENTRYPOINT []
CMD tail -f /dev/null
