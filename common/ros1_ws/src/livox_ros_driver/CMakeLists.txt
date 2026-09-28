# livox_ros_driver (ROS1) のメッセージ定義のみを提供する最小パッケージ。
# FAST-LIVO2 / FAST-LIO2 / R3LIVE / SR-LIVO はこの型 (livox_ros_driver/CustomMsg) を購読する。
# Livox-SDK 本体やドライバは bag リプレイには不要なのでビルドしない。
cmake_minimum_required(VERSION 3.0.2)
project(livox_ros_driver)
find_package(catkin REQUIRED COMPONENTS message_generation std_msgs)
add_message_files(FILES CustomPoint.msg CustomMsg.msg)
generate_messages(DEPENDENCIES std_msgs)
catkin_package(CATKIN_DEPENDS message_runtime std_msgs)
