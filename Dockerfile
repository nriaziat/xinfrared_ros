ARG FROM_IMAGE=ros:noetic
ARG OVERLAY_WS=/opt/overlay_ws

FROM ${FROM_IMAGE} as cacher
ARG OVERLAY_WS
WORKDIR ${OVERLAY_WS}/src
COPY . ./src
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
    apt-get update && \
    apt-get install -y python3-dev && \
    rosdep install -y \
    --from-paths \
    ./ \
    --ignore-src \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /opt
RUN mkdir -p /tmp/opt && \
    find ./ -name "package.xml" | \
    xargs cp --parents -t /tmp/opt

# multi-stage for building
FROM $FROM_IMAGE AS builder

# install overlay dependencies
ARG OVERLAY_WS
WORKDIR $OVERLAY_WS
COPY --from=cacher /tmp/$OVERLAY_WS/src ./src
RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
    apt-get update && \
    apt-get install -y python3-dev && \
    rosdep install -y \
    --from-paths \
    ./ \
    --ignore-src \
    && rm -rf /var/lib/apt/lists/* 

# build overlay source
COPY --from=cacher $OVERLAY_WS/src ./src
RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
    catkin_make

# source entrypoint setup
ENV OVERLAY_WS $OVERLAY_WS
RUN sed --in-place --expression \
    '$isource "$OVERLAY_WS/devel/setup.bash"' \
    /ros_entrypoint.sh
CMD ["roslaunch", "xinfrared_ros", "xinfrared.launch"]