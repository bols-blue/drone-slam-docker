# ROS2 bag の画像と LiDAR 点群を RViz2 で見るためのイメージ (ros2_tools/view_bag_images.sh 用)
FROM ros:jazzy-ros-base
LABEL org.opencontainers.image.source="https://github.com/bols-blue/drone-slam-docker" \
      org.opencontainers.image.description="drone-slam ros2-viewer: RViz2 + image_transport plugins + livox_ros_driver2 msgs for viewing ROS2 bags" \
      org.opencontainers.image.licenses="MIT"
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-jazzy-rviz2 ros-jazzy-rviz-default-plugins ros-jazzy-image-transport-plugins ros-jazzy-tf2-ros \
      ros-jazzy-rosbag2-storage-mcap python3-opencv python3-numpy python3-yaml \
    && rm -rf /var/lib/apt/lists/*
COPY ros2_tools/livox_ros_driver2 /opt/viewer_ws/src/livox_ros_driver2
RUN bash -c "source /opt/ros/jazzy/setup.bash && cd /opt/viewer_ws && colcon build --install-base /opt/viewer_ws/install && rm -rf build log"
COPY ros2_tools/make_rviz.py ros2_tools/compressed_relay.py ros2_tools/livox_to_pointcloud2.py /opt/ros2_tools/
RUN echo 'source /opt/viewer_ws/install/setup.bash' >> /etc/bash.bashrc
