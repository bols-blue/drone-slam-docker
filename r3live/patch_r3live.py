#!/usr/bin/env python3
"""R3LIVE (hku-mars/r3live) を Docker 内で bag リプレイ + Mid-360S 用に使うためのソースパッチ。

1. LiDAR front end: Avia 前提の「x > 0.7 m の点だけ使う」制限を距離 (blind) 判定に変更 (360°の Mid-360S 対応)
2. LiDAR-IMU 並進 (Lidar_offset_to_IMU, Avia 固定値) を ROS パラメータ /r3live_lio/lidar_offset_to_imu で指定可能に
3. IMU 加速度の g -> m/s^2 変換を /r3live_lio/acc_mul_G で有効化 (Mid-360S の内蔵IMUは g 単位)
4. IMU 初期化時に重力方向へ姿勢を合わせる (/r3live_lio/gravity_align。元実装は初期姿勢=単位行列固定)
5. カメラ-IMU 時刻オフセット初期値 /r3live_vio/camera_time_offset_td
6. GUI 無し (DISPLAY 無し) でも動くように cv::imshow / waitKey を無効化
7. 環境変数 R3LIVE_SAVE_TRIGGER のファイルが作られたら地図 (rgb_pt.pcd + test.r3live) を保存し、
   <trigger>.done を作る (キーボードの 's' と同等。pcl_viewer は起動しない)
"""
import sys
from pathlib import Path

root = Path(sys.argv[1])


def patch(rel, old, new, count=1):
    p = root / rel
    s = p.read_text()
    n = s.count(old)
    if n != count:
        sys.exit(f"patch failed: {rel}: expected {count} match(es), found {n}:\n{old}")
    p.write_text(s.replace(old, new))
    print(f"patched {rel}")


# 1. front end
patch("src/loam/LiDAR_front_end.cpp",
      "&& msg->points[ i ].x > 0.7 )",
      "&& ( msg->points[ i ].x * msg->points[ i ].x + msg->points[ i ].y * msg->points[ i ].y"
      " + msg->points[ i ].z * msg->points[ i ].z ) > blind * blind )")
patch("src/loam/LiDAR_front_end.cpp",
      "if ( ( msg->points[ i ].x > 2.0 )",
      "if ( ( ( msg->points[ i ].x * msg->points[ i ].x + msg->points[ i ].y * msg->points[ i ].y"
      " + msg->points[ i ].z * msg->points[ i ].z ) > 4.0 )")

# 2. LiDAR-IMU offset
patch("src/loam/include/common_lib.h",
      "static const Eigen::Vector3d Lidar_offset_to_IMU(0.04165, 0.02326, -0.0284); // Avia",
      """#include <ros/ros.h>
// drone-slam-docker: /r3live_lio/lidar_offset_to_imu で上書き可能 (既定は Avia の値)
inline const Eigen::Vector3d &lidar_offset_to_imu_param()
{
    static const Eigen::Vector3d v = []() {
        std::vector< double > t;
        if ( ros::param::get( "/r3live_lio/lidar_offset_to_imu", t ) && t.size() == 3 )
            return Eigen::Vector3d( t[ 0 ], t[ 1 ], t[ 2 ] );
        return Eigen::Vector3d( 0.04165, 0.02326, -0.0284 );
    }();
    return v;
}
#define Lidar_offset_to_IMU lidar_offset_to_imu_param()""")

# 3. acc in g
patch("src/r3live.hpp",
      'get_ros_parameter( m_ros_node_handle, "r3live_lio/lidar_time_delay", m_lidar_imu_time_delay, 0.0 );',
      'get_ros_parameter( m_ros_node_handle, "r3live_lio/lidar_time_delay", m_lidar_imu_time_delay, 0.0 );\n'
      '            get_ros_parameter( m_ros_node_handle, "r3live_lio/acc_mul_G", g_camera_lidar_queue.m_if_acc_mul_G, 0 );')

# 4. gravity alignment
patch("src/loam/IMU_Processing.cpp",
      "    state_inout.gravity = Eigen::Vector3d( 0, 0, 9.805 );\n    state_inout.rot_end = Eye3d;",
      """    state_inout.gravity = Eigen::Vector3d( 0, 0, 9.805 );
    state_inout.rot_end = Eye3d;
    bool gravity_align = false;
    ros::param::param( "/r3live_lio/gravity_align", gravity_align, false );
    if ( gravity_align && mean_acc.norm() > 1e-3 )
    {
        // 静止時の平均加速度 (IMU座標) がワールドの +z を向くように初期姿勢を設定
        state_inout.rot_end = Eigen::Quaterniond::FromTwoVectors( mean_acc.normalized(), Eigen::Vector3d( 0, 0, 1 ) ).toRotationMatrix();
        std::cout << "[drone-slam] gravity aligned initial rotation:\\n" << state_inout.rot_end << std::endl;
    }""")

# 5. camera time offset
patch("src/r3live_vio.cpp",
      "    m_inital_rot_ext_i2c = state.rot_ext_i2c;",
      '    ros::param::param( "/r3live_vio/camera_time_offset_td", state.td_ext_i2c, 0.0 );\n'
      "    m_inital_rot_ext_i2c = state.rot_ext_i2c;")

# 6 & 7. headless + save trigger
patch("src/r3live_vio.cpp",
      """char R3LIVE::cv_keyboard_callback()
{
    char c = cv_wait_key( 1 );
    // return c;
    if ( c == 's' || c == 'S' )
    {
        scope_color( ANSI_COLOR_GREEN_BOLD );
        cout << "I capture the keyboard input!!!" << endl;
        m_mvs_recorder.export_to_mvs( m_map_rgb_pts );
        // m_map_rgb_pts.save_and_display_pointcloud( m_map_output_dir, std::string("/rgb_pt"), std::max(m_pub_pt_minimum_views, 5) );
        m_map_rgb_pts.save_and_display_pointcloud( m_map_output_dir, std::string("/rgb_pt"), m_pub_pt_minimum_views  );
    }
    return c;
}""",
      """static bool r3live_has_display()
{
    static const bool v = ( getenv( "DISPLAY" ) != nullptr ) && ( getenv( "R3LIVE_HEADLESS" ) == nullptr );
    return v;
}

char R3LIVE::cv_keyboard_callback()
{
    char c = r3live_has_display() ? cv_wait_key( 1 ) : 0;
    static const std::string trigger = getenv( "R3LIVE_SAVE_TRIGGER" ) ? getenv( "R3LIVE_SAVE_TRIGGER" ) : "";
    bool by_trigger = false;
    if ( !trigger.empty() && Common_tools::if_file_exist( trigger ) )
    {
        remove( trigger.c_str() );
        by_trigger = true;
        c = 'S';
    }
    if ( c == 's' || c == 'S' )
    {
        scope_color( ANSI_COLOR_GREEN_BOLD );
        cout << "Saving map (" << ( by_trigger ? "trigger file" : "keyboard" ) << ") ..." << endl;
        if ( m_if_record_mvs )
            m_mvs_recorder.export_to_mvs( m_map_rgb_pts );
        m_map_rgb_pts.save_to_pcd( m_map_output_dir, std::string( "/rgb_pt" ), m_pub_pt_minimum_views );
        if ( by_trigger )
        {
            FILE *fp = fopen( ( trigger + ".done" ).c_str(), "w" );
            if ( fp )
                fclose( fp );
        }
    }
    return c;
}""")
patch("src/r3live_vio.cpp",
      '    cv::imshow( "Control panel", generate_control_panel_img().clone() );',
      '    if ( r3live_has_display() )\n        cv::imshow( "Control panel", generate_control_panel_img().clone() );')

# 8. OpenCV 3.2 (Melodic) には cv::parallel_for_ のラムダ版オーバーロードが無い (3.3 で追加) ので補う
(root / "src/tools/cv_parallel_compat.hpp").write_text("""#pragma once
// drone-slam-docker: OpenCV < 3.3 向け cv::parallel_for_(Range, lambda) 互換
#include <functional>
#include <opencv2/core.hpp>
#if (CV_VERSION_MAJOR == 3 && CV_VERSION_MINOR < 3) || CV_VERSION_MAJOR < 3
namespace cv
{
class DroneSlamLambdaBody : public ParallelLoopBody
{
  public:
    explicit DroneSlamLambdaBody( const std::function< void( const Range & ) > &f ) : m_f( f ) {}
    void operator()( const Range &r ) const override { m_f( r ); }

  private:
    std::function< void( const Range & ) > m_f;
};
inline void parallel_for_( const Range &range, std::function< void( const Range & ) > functor, double nstripes = -1. )
{
    parallel_for_( range, DroneSlamLambdaBody( functor ), nstripes );
}
} // namespace cv
#endif
""")
patch("src/optical_flow/lkpyramid.cpp", '#include "lkpyramid.hpp"\n',
      '#include "lkpyramid.hpp"\n#include "../tools/cv_parallel_compat.hpp"\n')
patch("src/rgb_map/pointcloud_rgbd.cpp", '#include "pointcloud_rgbd.hpp"\n',
      '#include "pointcloud_rgbd.hpp"\n#include "../tools/cv_parallel_compat.hpp"\n')

# 9. r3live_mapping が pcl_filters (VoxelGrid) をリンクしていない問題 (環境によってリンクエラー)
patch("CMakeLists.txt", "                          pcl_common \n                          pcl_io)",
      "                          pcl_common \n                          pcl_io\n                          ${PCL_LIBRARIES})")

# 10. SIGINT で r3live_mapping が終了せずハングする (ワーカースレッドの futex 待ち) ので、
#     SIGINT 受信後 5 秒で強制終了する。地図は 7. のトリガで終了前に保存済みの想定。
patch("src/r3live.cpp", '    ros::init(argc, argv, "R3LIVE_main");',
      """    ros::init(argc, argv, "R3LIVE_main", ros::init_options::NoSigintHandler);
    signal( SIGINT, []( int ) {
        ros::requestShutdown();
        signal( SIGALRM, []( int ) { _exit( 0 ); } );
        alarm( 5 );
    } );""")
patch("src/r3live.cpp", "    ros::spin();\n}", "    ros::spin();\n    _exit( 0 );\n}")
patch("src/r3live.cpp", "#include <omp.h>\n", "#include <omp.h>\n#include <csignal>\n#include <unistd.h>\n")
