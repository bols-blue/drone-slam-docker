# R3LIVE (hku-mars/r3live) + Livox Mid-360S bag リプレイ用イメージ
#   build:  docker build -f r3live/Dockerfile -t drone-slam:r3live .   (コンテキストはリポジトリルート)
FROM ros:melodic-perception

ARG MAKE_JOBS=4
ARG R3LIVE_COMMIT=6143a38537f28cb36eb24e9bbe2e39c8f7967157

LABEL org.opencontainers.image.source="https://github.com/bols-blue/drone-slam-docker" \
      org.opencontainers.image.description="R3LIVE (LiDAR-inertial-visual SLAM, colored map + textured mesh) for Livox Mid-360S rosbag replay" \
      org.opencontainers.image.licenses="GPL-2.0"

ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
      git libcgal-dev python3-numpy python3-pip python3-setuptools \
      ros-melodic-rviz ros-melodic-tf ros-melodic-message-filters ros-melodic-image-transport-plugins \
    && pip3 install --no-cache-dir "PyYAML>=5.1,<6" \
    && rm -rf /var/lib/apt/lists/* \
    # Melodic 既知問題: flann 同梱の lz4.h が roslz4 (rosbag) の lz4.h と衝突するのでシステム版に差し替え
    && mv /usr/include/flann/ext/lz4.h /usr/include/flann/ext/lz4.h.bak \
    && mv /usr/include/flann/ext/lz4hc.h /usr/include/flann/ext/lz4hc.h.bak \
    && ln -s /usr/include/lz4.h /usr/include/flann/ext/lz4.h \
    && ln -s /usr/include/lz4hc.h /usr/include/flann/ext/lz4hc.h

# R3LIVE 本体 (コミット固定) + パッチ
WORKDIR /opt/drone_slam/ws/src
RUN git clone https://github.com/hku-mars/r3live.git r3live \
    && cd r3live && git checkout ${R3LIVE_COMMIT} && rm -rf .git papers
COPY r3live/patch_r3live.py /opt/drone_slam/r3live/patch_r3live.py
RUN python3 /opt/drone_slam/r3live/patch_r3live.py /opt/drone_slam/ws/src/r3live/r3live

# 共通パッケージ (livox_ros_driver msgs, drone_slam_utils)
COPY common/ros1_ws/src /opt/drone_slam/ws/src/common
WORKDIR /opt/drone_slam/ws
# r3live は livox_ros_driver のメッセージ生成に依存宣言していないため先に単独でビルドする
RUN . /opt/ros/melodic/setup.sh \
    && catkin_make -DCMAKE_BUILD_TYPE=Release -DCATKIN_WHITELIST_PACKAGES="livox_ros_driver" -j${MAKE_JOBS} \
    && catkin_make -DCMAKE_BUILD_TYPE=Release -DCATKIN_WHITELIST_PACKAGES="" -j${MAKE_JOBS} \
    && rm -rf build

COPY common/scripts/slam-replay /usr/local/bin/slam-replay
COPY common/scripts/calib.py /opt/drone_slam/lib/calib.py
COPY common/config/calib.yaml /opt/drone_slam/config/calib.yaml
COPY r3live/gen_config.py /opt/drone_slam/r3live/gen_config.py
COPY r3live/launch /opt/drone_slam/r3live/launch
COPY r3live/profile.sh /opt/drone_slam/profile.sh
RUN chmod +x /usr/local/bin/slam-replay

WORKDIR /data
ENTRYPOINT ["slam-replay"]
CMD ["--help"]
