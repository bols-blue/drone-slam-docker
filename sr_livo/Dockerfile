# SR-LIVO (LiDAR-Inertial-Visual Odometry with Sweep Reconstruction) rosbag リプレイ用イメージ
#   upstream: https://github.com/ZikangYuan/sr_livo (Ubuntu 20.04 / ROS Noetic で検証済みと記載)
FROM ros:noetic-perception

ARG MAKE_JOBS=4
ARG SR_LIVO_REPO=https://github.com/ZikangYuan/sr_livo.git
ARG SR_LIVO_COMMIT=97a26660ca0d3723e1556b447d3b7d34f2866555

ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
      git build-essential cmake \
      libceres-dev libeigen3-dev libpcl-dev libopencv-dev libgoogle-glog-dev \
      python3-yaml python3-numpy \
      ros-noetic-eigen-conversions ros-noetic-tf ros-noetic-pcl-ros ros-noetic-cv-bridge \
      ros-noetic-rviz \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /opt/drone_slam/ws
COPY common/ros1_ws/src src/common
RUN git clone ${SR_LIVO_REPO} src/sr_livo \
    && git -C src/sr_livo checkout ${SR_LIVO_COMMIT} \
    && rm -rf src/sr_livo/.git
COPY sr_livo/patch_sr_livo.py /opt/drone_slam/sr_livo/patch_sr_livo.py
RUN python3 /opt/drone_slam/sr_livo/patch_sr_livo.py src/sr_livo/src/lioOptimization.cpp
RUN . /opt/ros/noetic/setup.sh \
    && catkin_make -DCMAKE_BUILD_TYPE=Release -j${MAKE_JOBS} \
    && rm -rf build

COPY common/scripts/slam-replay /usr/local/bin/slam-replay
COPY common/scripts/calib.py /opt/drone_slam/lib/calib.py
COPY common/config/calib.yaml /opt/drone_slam/config/calib.yaml
COPY sr_livo/gen_config.py /opt/drone_slam/sr_livo/gen_config.py
COPY sr_livo/profile.sh /opt/drone_slam/profile.sh
RUN chmod +x /usr/local/bin/slam-replay

LABEL org.opencontainers.image.source="https://github.com/bols-blue/drone-slam-docker" \
      org.opencontainers.image.description="SR-LIVO (ROS Noetic) rosbag replay -> colored point cloud map, for Livox Mid-360S + camera" \
      org.opencontainers.image.licenses="MIT"

WORKDIR /data
ENTRYPOINT ["slam-replay"]
CMD ["--help"]
