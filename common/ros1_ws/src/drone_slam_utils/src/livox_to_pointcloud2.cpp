// livox_ros_driver/CustomMsg -> sensor_msgs/PointCloud2 (Velodyne 互換フィールド) 変換ノード
//
// 出力フィールド: x, y, z, intensity (float32), ring (uint16), time (float32, スキャン先頭からの相対秒)
// LVI-SAM など ring / time 付きの PointCloud2 を前提とする手法に Mid-360 の点群を入力するために使う。
#include <cstring>
#include <livox_ros_driver/CustomMsg.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

namespace {

struct VeloPoint {
  float x, y, z, intensity;
  uint16_t ring;
  float time;
} __attribute__((packed));

ros::Publisher g_pub;
double g_blind = 0.1;
int g_num_lines = 4;

void addField(sensor_msgs::PointCloud2& msg, const std::string& name, uint32_t offset, uint8_t type) {
  sensor_msgs::PointField f;
  f.name = name;
  f.offset = offset;
  f.datatype = type;
  f.count = 1;
  msg.fields.push_back(f);
}

void callback(const livox_ros_driver::CustomMsg::ConstPtr& in) {
  sensor_msgs::PointCloud2 out;
  out.header = in->header;
  addField(out, "x", offsetof(VeloPoint, x), sensor_msgs::PointField::FLOAT32);
  addField(out, "y", offsetof(VeloPoint, y), sensor_msgs::PointField::FLOAT32);
  addField(out, "z", offsetof(VeloPoint, z), sensor_msgs::PointField::FLOAT32);
  addField(out, "intensity", offsetof(VeloPoint, intensity), sensor_msgs::PointField::FLOAT32);
  addField(out, "ring", offsetof(VeloPoint, ring), sensor_msgs::PointField::UINT16);
  addField(out, "time", offsetof(VeloPoint, time), sensor_msgs::PointField::FLOAT32);
  out.point_step = sizeof(VeloPoint);
  out.is_bigendian = false;
  out.is_dense = true;
  out.height = 1;

  std::vector<VeloPoint> pts;
  pts.reserve(in->points.size());
  const double blind2 = g_blind * g_blind;
  for (const auto& p : in->points) {
    // FAST-LIO と同じタグフィルタ (ノイズ判定された点を除外)
    if ((p.tag & 0x30) != 0x10 && (p.tag & 0x30) != 0x00) continue;
    if (p.line >= g_num_lines) continue;
    const double r2 = p.x * p.x + p.y * p.y + p.z * p.z;
    if (r2 < blind2) continue;
    VeloPoint v;
    v.x = p.x;
    v.y = p.y;
    v.z = p.z;
    v.intensity = p.reflectivity;
    v.ring = p.line;
    v.time = static_cast<float>(p.offset_time * 1e-9);
    pts.push_back(v);
  }
  out.width = pts.size();
  out.row_step = out.point_step * out.width;
  out.data.resize(out.row_step);
  if (!pts.empty()) std::memcpy(out.data.data(), pts.data(), out.row_step);
  g_pub.publish(out);
}

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "livox_to_pointcloud2");
  ros::NodeHandle nh, pnh("~");
  std::string in_topic, out_topic;
  pnh.param<std::string>("input_topic", in_topic, "/livox/lidar");
  pnh.param<std::string>("output_topic", out_topic, "/livox/points");
  pnh.param("blind", g_blind, 0.1);
  pnh.param("num_lines", g_num_lines, 4);
  g_pub = nh.advertise<sensor_msgs::PointCloud2>(out_topic, 10);
  ros::Subscriber sub = nh.subscribe(in_topic, 10, callback, ros::TransportHints().tcpNoDelay());
  ros::spin();
  return 0;
}
