# =============================================================================
# FAST-LIO2 (LiDAR-慣性SLAM) + 後処理テクスチャリング (Open3D Poisson メッシュ + OpenMVS TextureMesh)
#   build context = リポジトリルート
#   docker build -f fast_lio2_openmvs/Dockerfile -t drone-slam:fast-lio2-openmvs .
# =============================================================================
ARG MAKE_JOBS=4

# -----------------------------------------------------------------------------
# Stage 1: OpenMVS (CPU, CUDA 無し)
#   v2.3 以降は OpenCV>=4.10 / gcc>=10 前提のコードを含み focal でビルドできないため v2.2.0 を使う。
#   v2.2.0 は Eigen >= 3.4 を要求するので、ビルドステージでのみ Eigen 3.4 を /opt/eigen34 に入れる
#   (OpenMVS 本体は静的ライブラリでリンクされ、最終イメージに Eigen 3.4 は残らない)
# -----------------------------------------------------------------------------
FROM ros:noetic-perception AS openmvs-build
ARG MAKE_JOBS
ARG OPENMVS_TAG=v2.2.0
ARG EIGEN_TAG=3.4.0
ARG VCG_SHA=658ba36d0a5666650da6e066b4794efc5a463407
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
      git build-essential cmake pkg-config \
      libboost-iostreams-dev libboost-program-options-dev libboost-system-dev libboost-serialization-dev \
      libcgal-dev libgmp-dev libmpfr-dev libopencv-dev \
      libpng-dev libjpeg-dev libtiff-dev zlib1g-dev \
    && rm -rf /var/lib/apt/lists/*
WORKDIR /build
RUN git clone --depth 1 -b ${EIGEN_TAG} https://gitlab.com/libeigen/eigen.git \
    && cmake -S eigen -B eigen/build -DCMAKE_INSTALL_PREFIX=/opt/eigen34 -DBUILD_TESTING=OFF -DEIGEN_BUILD_DOC=OFF \
    && cmake --install eigen/build \
    && git clone https://github.com/cdcseacave/VCG.git vcglib && git -C vcglib checkout ${VCG_SHA}
RUN git clone --depth 1 -b ${OPENMVS_TAG} https://github.com/cdcseacave/openMVS.git \
    && cmake -S openMVS -B openMVS/make \
         -DCMAKE_BUILD_TYPE=Release \
         -DEIGEN3_INCLUDE_DIR=/opt/eigen34/include/eigen3 \
         -DVCG_ROOT=/build/vcglib \
         -DOpenMVS_USE_CUDA=OFF -DOpenMVS_USE_PYTHON=OFF -DOpenMVS_USE_BREAKPAD=OFF \
         -DOpenMVS_USE_OPENGL=OFF -DOpenMVS_USE_CERES=OFF \
         -DCMAKE_INSTALL_PREFIX=/opt/openmvs \
    && cmake --build openMVS/make -j${MAKE_JOBS} \
    && cmake --install openMVS/make \
    && rm -rf openMVS

# -----------------------------------------------------------------------------
# Stage 2: FAST-LIO2 + 実行環境
# -----------------------------------------------------------------------------
FROM ros:noetic-perception
ARG MAKE_JOBS
ARG FAST_LIO_SHA=7cc4175de6f8ba2edf34bab02a42195b141027e9
ENV DEBIAN_FRONTEND=noninteractive
LABEL org.opencontainers.image.source="https://github.com/bols-blue/drone-slam-docker" \
      org.opencontainers.image.description="drone-slam: FAST-LIO2 rosbag replay + OpenMVS texture mapping (Livox Mid-360S + camera)" \
      org.opencontainers.image.licenses="GPL-2.0"

RUN apt-get update && apt-get install -y --no-install-recommends \
      git build-essential python3-pip python3-catkin-tools \
      libeigen3-dev libpcl-dev ros-noetic-pcl-ros ros-noetic-rviz \
      libboost-iostreams1.71.0 libboost-program-options1.71.0 libboost-serialization1.71.0 \
      libgmp10 libmpfr6 libgomp1 \
    && rm -rf /var/lib/apt/lists/*
# open3d-cpu 0.18 は py3.8 wheel (manylinux_2_27) があり軽量。focal 標準の pip 20 はこのタグを認識しないので先に更新
RUN pip3 install --no-cache-dir --upgrade "pip<24.1" \
    && python3 -m pip install --no-cache-dir --ignore-installed PyYAML "open3d-cpu==0.18.0" "numpy>=1.20,<1.25" "scipy<1.11"

COPY --from=openmvs-build /opt/openmvs /opt/openmvs
ENV PATH=/opt/openmvs/bin/OpenMVS:/opt/openmvs/bin:$PATH

WORKDIR /opt/drone_slam/ws
COPY common/ros1_ws/src src/common
RUN git clone https://github.com/hku-mars/FAST_LIO.git src/FAST_LIO \
    && git -C src/FAST_LIO checkout ${FAST_LIO_SHA} \
    && git -C src/FAST_LIO submodule update --init --recursive \
    && . /opt/ros/noetic/setup.sh \
    && catkin_make -DCMAKE_BUILD_TYPE=Release -j${MAKE_JOBS} \
    && rm -rf build \
    # FAST-LIO は ROOT_DIR(ソースディレクトリ)/PCD/scans.pcd に地図を保存する。非 root 実行でも書けるようにする
    && mkdir -p src/FAST_LIO/PCD src/FAST_LIO/Log && chmod 777 src/FAST_LIO/PCD src/FAST_LIO/Log

COPY common/scripts/slam-replay /usr/local/bin/slam-replay
COPY common/scripts/calib.py /opt/drone_slam/lib/calib.py
COPY common/config/calib.yaml /opt/drone_slam/config/calib.yaml
COPY fast_lio2_openmvs/launch /opt/drone_slam/launch
COPY fast_lio2_openmvs/gen_config.py /opt/drone_slam/lib/gen_config.py
COPY fast_lio2_openmvs/clock_keeper.py /opt/drone_slam/lib/clock_keeper.py
COPY fast_lio2_openmvs/texture_pipeline.py /opt/drone_slam/lib/texture_pipeline.py
COPY fast_lio2_openmvs/profile.sh /opt/drone_slam/profile.sh
RUN chmod +x /usr/local/bin/slam-replay /opt/drone_slam/lib/*.py \
    && ln -s /opt/drone_slam/lib/texture_pipeline.py /usr/local/bin/texture-pipeline
ENV PYTHONPATH=/opt/drone_slam/lib:/opt/ros/noetic/lib/python3/dist-packages \
    LD_LIBRARY_PATH=/opt/ros/noetic/lib

WORKDIR /data
ENTRYPOINT ["slam-replay"]
CMD ["--help"]
