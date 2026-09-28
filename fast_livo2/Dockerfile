# FAST-LIVO2 (hku-mars) rosbag リプレイ用イメージ — ROS1 Noetic
FROM ros:noetic-perception

ARG MAKE_JOBS=4
# 上流は -march=native でビルドされるため、ビルド機以外の CPU で Illegal instruction になる。
# 可搬性のため x86-64-v2 相当 (gcc9 には v2 指定が無いので nehalem) に固定 (AVX を使わないので Sophus/Eigen のアライメント不整合も起きない)
ARG CPU_ARCH_FLAGS="-march=nehalem -mtune=generic"
ARG FAST_LIVO2_COMMIT=0d2c0346107b75b59934975adec9a6eeeb913c64
ARG VIKIT_COMMIT=6c886c8e5d83997806e00294826d528cea3581dd
ARG SOPHUS_COMMIT=a621ff2e56c56c839a6c40418d42c3c254424b5c

ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
      git cmake build-essential python3-yaml python3-numpy \
      libeigen3-dev libpcl-dev libopencv-dev libboost-thread-dev \
      ros-noetic-pcl-ros ros-noetic-cv-bridge ros-noetic-image-transport \
      ros-noetic-image-transport-plugins ros-noetic-eigen-conversions ros-noetic-tf \
      ros-noetic-rviz \
    && rm -rf /var/lib/apt/lists/*

# Sophus (非テンプレート版 a621ff。新しい gcc でビルドできない箇所を修正)
RUN git clone https://github.com/strasdat/Sophus.git /tmp/Sophus \
    && cd /tmp/Sophus && git checkout ${SOPHUS_COMMIT} \
    && sed -i 's/unit_complex_.real() = 1.;/unit_complex_ = std::complex<double>(1., 0.);/; /unit_complex_.imag() = 0.;/d' sophus/so2.cpp \
    && mkdir build && cd build && cmake .. -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTS=OFF \
    && make -j${MAKE_JOBS} Sophus && make install && rm -rf /tmp/Sophus \
    && mkdir -p /usr/local/lib/cmake/Sophus \
    && printf '%s\n' \
      'set(Sophus_INCLUDE_DIR /usr/local/include)' 'set(Sophus_INCLUDE_DIRS /usr/local/include)' \
      'set(Sophus_LIBRARY /usr/local/lib/libSophus.so)' 'set(Sophus_LIBRARIES /usr/local/lib/libSophus.so)' \
      'set(Sophus_LIBRARY_DIR /usr/local/lib)' 'set(Sophus_LIBRARY_DIRS /usr/local/lib)' \
      > /usr/local/lib/cmake/Sophus/SophusConfig.cmake \
    && ldconfig

WORKDIR /opt/drone_slam/ws
RUN mkdir -p src \
    && git clone https://github.com/xuankuzcr/rpg_vikit.git src/rpg_vikit \
    && git -C src/rpg_vikit checkout ${VIKIT_COMMIT} \
    && git clone https://github.com/hku-mars/FAST-LIVO2.git src/FAST-LIVO2 \
    && git -C src/FAST-LIVO2 checkout ${FAST_LIVO2_COMMIT} \
    && sed -i "s/-march=native/${CPU_ARCH_FLAGS}/g" src/rpg_vikit/vikit_common/CMakeLists.txt src/rpg_vikit/vikit_ros/CMakeLists.txt \
    && sed -i "s/-O3 -march=native -mtune=native -funroll-loops/-O3 ${CPU_ARCH_FLAGS} -funroll-loops/" src/FAST-LIVO2/CMakeLists.txt \
    && grep -rn "march=native" src/ ; true

COPY common/ros1_ws/src src/common
RUN . /opt/ros/noetic/setup.sh \
    && catkin_make -DCMAKE_BUILD_TYPE=Release -j${MAKE_JOBS} \
    && rm -rf build

COPY common/scripts/slam-replay /usr/local/bin/slam-replay
COPY common/scripts/calib.py /opt/drone_slam/lib/calib.py
COPY common/config/calib.yaml /opt/drone_slam/config/calib.yaml
COPY fast_livo2/profile.sh /opt/drone_slam/profile.sh
COPY fast_livo2/gen_config.py fast_livo2/voxel_downsample.py fast_livo2/fast_livo2_template.yaml fast_livo2/replay.launch /opt/drone_slam/fast_livo2/
RUN chmod +x /usr/local/bin/slam-replay

LABEL org.opencontainers.image.source="https://github.com/bols-blue/drone-slam-docker" \
      org.opencontainers.image.description="FAST-LIVO2 (LiDAR-Inertial-Visual SLAM) rosbag replay for Livox Mid-360S + camera -> colored point cloud map" \
      org.opencontainers.image.licenses="GPL-2.0"
WORKDIR /data
ENTRYPOINT ["slam-replay"]
CMD ["--help"]
