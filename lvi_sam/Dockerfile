# =============================================================================
# LVI-SAM (LiDAR-Visual-Inertial SLAM, factor graph) — rosbag replay image
#   upstream : TixiaoShan/LVI-SAM を外参設定を汎用化した Cc19245/LVI-SAM-Easyused ("new" branch:
#              最新 LIO-SAM ベース・Livox 対応) を使用
#   target   : Ubuntu 18.04 / ROS Melodic / GTSAM 4.0.2 / Ceres 1.14 (上流の動作確認環境)
# build (repo root):
#   docker build -f lvi_sam/Dockerfile -t drone-slam:lvi-sam .
# =============================================================================
FROM ros:melodic-perception

ARG MAKE_JOBS=4
ARG GTSAM_VERSION=4.0.2
ARG CERES_VERSION=1.14.0
ARG LVI_SAM_REPO=https://github.com/Cc19245/LVI-SAM-Easyused.git
ARG LVI_SAM_COMMIT=29a21b3e543c02ae7a540f5dc3d2b6939412321d

ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-c"]

RUN apt-get update && apt-get install -y --no-install-recommends \
      git wget build-essential cmake \
      libgoogle-glog-dev libgflags-dev libatlas-base-dev libeigen3-dev libsuitesparse-dev \
      libboost-all-dev libtbb-dev \
      ros-melodic-xacro ros-melodic-robot-state-publisher ros-melodic-rviz \
      ros-melodic-image-transport-plugins \
      python3-numpy python3-pip python3-setuptools \
    && python3 -m pip install --no-cache-dir "PyYAML>=5.1,<7" \
    && rm -rf /var/lib/apt/lists/*

# ---- Ceres 1.14 ----
RUN git clone --depth 1 -b ${CERES_VERSION} https://github.com/ceres-solver/ceres-solver.git /tmp/ceres \
    && mkdir /tmp/ceres/build && cd /tmp/ceres/build \
    && cmake .. -DCMAKE_BUILD_TYPE=Release \
         -DBUILD_TESTING=OFF -DBUILD_EXAMPLES=OFF -DBUILD_BENCHMARKS=OFF \
    && make -j${MAKE_JOBS} install && cd / \
    && rm -rf /tmp/ceres

# ---- GTSAM 4.0.2 (march=native 無効: 別 CPU での Illegal instruction 回避) ----
RUN git clone --depth 1 -b ${GTSAM_VERSION} https://github.com/borglab/gtsam.git /tmp/gtsam \
    && mkdir /tmp/gtsam/build && cd /tmp/gtsam/build \
    && cmake .. -DCMAKE_BUILD_TYPE=Release \
         -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF -DGTSAM_USE_SYSTEM_EIGEN=ON \
         -DGTSAM_BUILD_TESTS=OFF -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF -DGTSAM_BUILD_UNSTABLE=ON \
         -DGTSAM_WITH_TBB=OFF \
    && make -j${MAKE_JOBS} install && cd / \
    && rm -rf /tmp/gtsam && ldconfig

# ---- workspace ----
WORKDIR /opt/drone_slam/ws
COPY common/ros1_ws/src src/common
COPY lvi_sam/patches /tmp/lvi_patches
RUN git clone ${LVI_SAM_REPO} src/lvi_sam \
    && cd src/lvi_sam && git checkout ${LVI_SAM_COMMIT} \
    && git apply /tmp/lvi_patches/*.patch \
    && rm -rf .git /tmp/lvi_patches
RUN source /opt/ros/melodic/setup.bash \
    && catkin_make -DCMAKE_BUILD_TYPE=Release -j${MAKE_JOBS} \
    && rm -rf build

# ---- runner ----
COPY common/scripts/slam-replay /usr/local/bin/slam-replay
COPY common/scripts/calib.py /opt/drone_slam/lib/calib.py
COPY common/config/calib.yaml /opt/drone_slam/config/calib.yaml
COPY lvi_sam/profile.sh /opt/drone_slam/profile.sh
COPY lvi_sam/gen_config.py lvi_sam/bag_to_tum.py /opt/drone_slam/lvi_sam/
COPY lvi_sam/launch /opt/drone_slam/lvi_sam/launch
RUN chmod +x /usr/local/bin/slam-replay /opt/drone_slam/lvi_sam/*.py

LABEL org.opencontainers.image.source="https://github.com/bols-blue/drone-slam-docker" \
      org.opencontainers.image.description="LVI-SAM (LVI-SAM-Easyused) rosbag replay for Livox Mid-360S + camera + FC 9-axis IMU" \
      org.opencontainers.image.licenses="BSD-3-Clause"

WORKDIR /data
ENTRYPOINT ["slam-replay"]
CMD ["--help"]
