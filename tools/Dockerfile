# ROS2 bag (mcap/sqlite3) -> ROS1 bag 変換ツール
FROM python:3.12-slim
LABEL org.opencontainers.image.source="https://github.com/bols-blue/drone-slam-docker" \
      org.opencontainers.image.description="drone-slam tools: ROS2 bag -> ROS1 bag converter for Livox Mid-360S + camera" \
      org.opencontainers.image.licenses="MIT"
RUN pip install --no-cache-dir "rosbags>=0.10,<0.11" "opencv-python-headless>=4.9,<5" "numpy<3"
COPY tools/ros2bag_to_ros1.py /usr/local/bin/ros2bag-to-ros1
RUN chmod +x /usr/local/bin/ros2bag-to-ros1
WORKDIR /data
ENTRYPOINT ["ros2bag-to-ros1"]
CMD ["--help"]
