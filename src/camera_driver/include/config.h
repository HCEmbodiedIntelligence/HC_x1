#ifndef CONFIG_H
#define CONFIG_H

#include <string>
#include <vector>
#include <builtin_interfaces/msg/time.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <yaml-cpp/yaml.h>
#include "custom_log.h"

// 内参：畸变系数和焦距等参数
struct Intrinsics
{
    float fx, fy;     // 焦距
    float cx, cy;     // 主点坐标
    float coeffs[5];  // 畸变系数 (k1, k2, p1, p2, k3)
};
// 外参：旋转矩阵和平移向量
struct Extrinsics
{
    float rotation[9];    // 3x3 旋转矩阵
    float translation[3]; // 平移向量
};
// 相机参数
struct InExtrinsics
{
    Intrinsics rgb_intrinsics;
    Intrinsics dep_intrinsics;
    Extrinsics extrinsics;
};

// 相机配置结构
struct CameraConfig
{
    std::string name;
    std::string id;  // IP地址或USB端口
    std::string type;
    std::string qos;
    std::string frame_id;
    std::string color_topic;
    std::string depth_topic;
    std::string param_topic;
    std::string cloud_topic;
    std::string rgb_cloud_topic;
    std::string chassis_depth_topic;
    std::string chassis_param_topic;
    int width;
    int height;
    int pointcloud_type;
    InExtrinsics param;
    uint32_t expos_us;
    uint32_t expos_gain;
    bool use_param_from_file = true;
    bool send_chassis_depth = false;
};

// redis
struct RedisInfo
{
    std::string host;
    std::string pw;
    int port;
};

// 点云选项
struct PointCloudOptions
{
    bool enable_color = false;  // 是否生成彩色点云
    bool ordered      = true;   // 是否保持有序结构
    float min_depth   = 0.1f;   // 最小有效深度 (米)，博升：0.3，安思疆：0.2
    float max_depth   = 10.0f;  // 最大有效深度 (米)，博升：8.0，安思疆：4.0
    float depth_scale = 1.0f / 1000.0f;  // 深度值缩放因子
};

class CameraDriver;
extern std::shared_ptr<CameraDriver>   g_driver;
extern std::shared_ptr<spdlog::logger> g_logger;

void signalHandler(int signal);
void bgr2rgb(std::vector<uint8_t>& rgb);
std::string currentTimeStr();
std::string tsNsToStr(uint64_t timestamp_ns);
std::string rosTimeToStr(const builtin_interfaces::msg::Time& ros_time);
uint64_t rosTimeToNs(const builtin_interfaces::msg::Time& ros_time);
// 深度图转点云
void convertDepthToPointCloud(const uint16_t* depth_data, const uint8_t* color_data,
    int width, int height, const Intrinsics& intrinsics,
    const PointCloudOptions& options, sensor_msgs::msg::PointCloud2 &cloud_msg);

template<typename T>
T getYamlValue(const YAML::Node& node, const std::string& key, const T& default_value)
{
    if(node[key] && node[key].IsScalar())
    {
        try
        {
            return node[key].as<T>();
        }
        catch(...)
        {
            return default_value;
        }
    }
    return default_value;
}

#endif