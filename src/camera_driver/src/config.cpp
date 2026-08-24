#include "config.h"

#include <vector>
#include <cstdint>
#include <cmath>
#include <limits>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <rclcpp/rclcpp.hpp>
#include "camera_driver.h"

std::shared_ptr<CameraDriver>   g_driver;
std::shared_ptr<spdlog::logger> g_logger;

void signalHandler(int signal)
{
    if (signal == SIGINT || signal == SIGTERM)
    {
        g_logger->warn("Received shutdown signal: {}, exiting...", signal);
        if (g_driver) g_driver->stop();
    }
}

void bgr2rgb(std::vector<uint8_t>& rgb)
{
    uint8_t* p = rgb.data();
    uint8_t* end = p + rgb.size();
    while (p != end)
    {
        std::swap(p[0], p[2]);
        p += 3;
    }
}

std::string currentTimeStr()
{
    // 获取当前时间
    auto now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();

    // 原始时间戳转换逻辑
    uint64_t seconds = now_ms / 1000ULL;
    time_t time_seconds = static_cast<time_t>(seconds);
    struct tm *timeinfo = localtime(&time_seconds);
    if (timeinfo == nullptr) return "时间错误";

    char buffer[50];
    strftime(buffer, sizeof(buffer), "[%Y.%m.%d %H:%M:%S", timeinfo);

    int milliseconds = static_cast<int>(now_ms % 1000);

    std::stringstream ss;
    ss << buffer << "." << std::setw(3) << std::setfill('0') << milliseconds <<"]";

    return ss.str();
}

std::string tsNsToStr(uint64_t timestamp_ns)
{
    // 获取当前时间（纳秒）
    auto now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();

    // 计算时间差（秒）
    int64_t diff_ns = static_cast<int64_t>(timestamp_ns) - static_cast<int64_t>(now_ns);
    double diff_ms = diff_ns / 1000000.0;

    // 原始时间戳转换逻辑
    uint64_t seconds = timestamp_ns / 1000000000ULL;
    uint64_t nanoseconds = timestamp_ns % 1000000000ULL;

    time_t time_seconds = static_cast<time_t>(seconds);
    struct tm *timeinfo = localtime(&time_seconds);
    if (timeinfo == nullptr) return "时间错误";

    char buffer[50];
    strftime(buffer, sizeof(buffer), "%Y.%m.%d %H:%M:%S", timeinfo);
    // strftime(buffer, sizeof(buffer), "[%Y.%m.%d %H:%M:%S", timeinfo);

    int milliseconds = static_cast<int>(nanoseconds / 1000000);

    std::stringstream ss;
    ss << buffer << "." << std::setw(3) << std::setfill('0') << milliseconds
       << " (快: " << std::fixed << std::setprecision(3)
       << diff_ms << " ms)]";

    return ss.str();
}

std::string rosTimeToStr(const builtin_interfaces::msg::Time& ros_time)
{
    // 将ROS时间转换为time_t
    time_t seconds = static_cast<time_t>(ros_time.sec);
    long nanoseconds = ros_time.nanosec;

    // 转换为tm结构体
    struct tm *timeinfo = localtime(&seconds);

    // 格式化为字符串
    char buffer[50];
    strftime(buffer, sizeof(buffer), "%H:%M:%S", timeinfo);
    // strftime(buffer, sizeof(buffer), "%Y.%m.%d %H:%M:%S", timeinfo);

    // 添加毫秒部分
    int milliseconds = nanoseconds / 1000000;  // 纳秒转毫秒
    std::stringstream ss;
    ss << buffer << "." << std::setw(3) << std::setfill('0') << milliseconds;

    return ss.str();
}

uint64_t rosTimeToNs(const builtin_interfaces::msg::Time& ros_time)
{
    // ROS 2 Time 包含秒和纳秒部分
    uint64_t total_ns = static_cast<uint64_t>(ros_time.sec) * 1000000000ULL +
                        static_cast<uint64_t>(ros_time.nanosec);
    return total_ns;
}

void convertDepthToPointCloud(const uint16_t* depth_data, const uint8_t* color_data,
    int width, int height, const Intrinsics& intrinsics,
    const PointCloudOptions& options, sensor_msgs::msg::PointCloud2 &cloud_msg)
{    
    // ==================== 1. 预计算查找表 ====================
    std::vector<float> x_lookup(width);
    std::vector<float> y_lookup(height);

    for (int x = 0; x < width; ++x)
        x_lookup[x] = (static_cast<float>(x) - intrinsics.cx) / intrinsics.fx;    
    for (int y = 0; y < height; ++y) 
        y_lookup[y] = (static_cast<float>(y) - intrinsics.cy) / intrinsics.fy;    

    // ==================== 2. 统计有效点数 ====================
    size_t valid_count = 0;
    const size_t total_pixels = width * height;

    if (!options.ordered)
        // 无序点云：统计有效点数
        for (size_t i = 0; i < total_pixels; ++i)
        {
            float depth = static_cast<float>(depth_data[i]) * options.depth_scale;
            if (depth >= options.min_depth && depth <= options.max_depth) 
                ++valid_count;
        }

    // ==================== 3. 设置点云字段 ====================
    sensor_msgs::PointCloud2Modifier modifier(cloud_msg);
    if (options.enable_color && color_data)
    {
        // 彩色点云 (XYZ + RGB)
        modifier.setPointCloud2Fields(4,
            "x", 1, sensor_msgs::msg::PointField::FLOAT32,
            "y", 1, sensor_msgs::msg::PointField::FLOAT32,
            "z", 1, sensor_msgs::msg::PointField::FLOAT32,
            "rgb", 1, sensor_msgs::msg::PointField::UINT32
        );
    } 
    else
    {
        // 非彩色点云 (仅XYZ)
        modifier.setPointCloud2FieldsByString(1, "xyz");
    }

    // ==================== 4. 分配内存 ====================
    if (options.ordered)
    {
        // 有序点云：保留所有位置
        cloud_msg.width = width;
        cloud_msg.height = height;
        cloud_msg.is_dense = false;  // 可能包含无效点
        modifier.resize(total_pixels);
    } 
    else
    {
        // 无序点云：只分配有效点
        cloud_msg.width = valid_count;
        cloud_msg.height = 1;
        cloud_msg.is_dense = true;
        modifier.resize(valid_count);
    }

    // ==================== 5. 填充数据 ====================
    if (options.enable_color && color_data)
    {
        // 彩色点云
        sensor_msgs::PointCloud2Iterator<float> iter_x(cloud_msg, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(cloud_msg, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(cloud_msg, "z");
        sensor_msgs::PointCloud2Iterator<uint32_t> iter_rgb(cloud_msg, "rgb");

        for (size_t idx = 0; idx < total_pixels; ++idx)
        {
            int y = idx / width;
            int x = idx % width;
            float depth_m = static_cast<float>(depth_data[idx]) * options.depth_scale;
            // 检查深度有效性
            bool is_valid = (depth_m >= options.min_depth && depth_m <= options.max_depth);
            if (options.ordered)
            {
                // 有序点云：填充所有位置
                if (is_valid)
                {
                    *iter_x = x_lookup[x] * depth_m;
                    *iter_y = y_lookup[y] * depth_m;
                    *iter_z = depth_m;

                    size_t color_idx = idx * 3;
                    // 打包RGB：R在高字节，G在中字节，B在低字节
                    uint32_t rgb_packed = 
                        (static_cast<uint32_t>(color_data[color_idx]) << 16) | 
                        (static_cast<uint32_t>(color_data[color_idx + 1]) << 8) | 
                        static_cast<uint32_t>(color_data[color_idx + 2]);
                    *iter_rgb = rgb_packed;
                } 
                else
                {
                    // 无效点填充NaN和黑色
                    *iter_x = std::numeric_limits<float>::quiet_NaN();
                    *iter_y = std::numeric_limits<float>::quiet_NaN();
                    *iter_z = std::numeric_limits<float>::quiet_NaN();
                    *iter_rgb = 0;
                }

                ++iter_x; ++iter_y; ++iter_z;
                ++iter_rgb;
            } 
            else
            {
                // 无序点云：只填充有效点
                if (is_valid)
                {
                    *iter_x = x_lookup[x] * depth_m;
                    *iter_y = y_lookup[y] * depth_m;
                    *iter_z = depth_m;

                    size_t color_idx = idx * 3;
                    // 打包RGB：R在高字节，G在中字节，B在低字节
                    uint32_t rgb_packed = 
                        (static_cast<uint32_t>(color_data[color_idx + 2]) << 16) | 
                        (static_cast<uint32_t>(color_data[color_idx + 1]) << 8) | 
                        static_cast<uint32_t>(color_data[color_idx]);
                    *iter_rgb = rgb_packed;

                    ++iter_x; ++iter_y; ++iter_z;
                    ++iter_rgb;
                }
            }
        }
    } 
    else 
    {
        // 非彩色点云
        sensor_msgs::PointCloud2Iterator<float> iter_x(cloud_msg, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(cloud_msg, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(cloud_msg, "z");

        for (size_t idx = 0; idx < total_pixels; ++idx)
        {
            int y = idx / width;
            int x = idx % width;
            float depth_m = static_cast<float>(depth_data[idx]) * options.depth_scale;
            // 检查深度有效性
            bool is_valid = (depth_m >= options.min_depth && depth_m <= options.max_depth);

            if (options.ordered)
            {
                // 有序点云：填充所有位置
                *iter_x = is_valid ? x_lookup[x] * depth_m : std::numeric_limits<float>::quiet_NaN();
                *iter_y = is_valid ? y_lookup[y] * depth_m : std::numeric_limits<float>::quiet_NaN();
                *iter_z = is_valid ? depth_m : std::numeric_limits<float>::quiet_NaN();
                ++iter_x; ++iter_y; ++iter_z;
            } 
            else
            {
                // 无序点云：只填充有效点
                if (is_valid)
                {
                    *iter_x = x_lookup[x] * depth_m;
                    *iter_y = y_lookup[y] * depth_m;
                    *iter_z = depth_m;
                    ++iter_x; ++iter_y; ++iter_z;
                }
            }
        }
    }
}

//预计算点云转换（用于SDK已提供点云的情况）
sensor_msgs::msg::PointCloud2 createPointCloudFromXYZ(
    const float* xyz_data, size_t point_count,
    const std::string& frame_id = "camera_depth_optical_frame")
{
    sensor_msgs::msg::PointCloud2 cloud_msg;
    cloud_msg.header.stamp = rclcpp::Clock().now();
    cloud_msg.header.frame_id = frame_id;
    cloud_msg.height = 1;
    cloud_msg.width = point_count;
    cloud_msg.is_dense = true;
    // 设置字段
    sensor_msgs::PointCloud2Modifier modifier(cloud_msg);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(point_count);
    // 直接拷贝数据
    size_t data_size = point_count * 3 * sizeof(float);
    cloud_msg.data.resize(data_size);
    memcpy(cloud_msg.data.data(), xyz_data, data_size);
    return cloud_msg;
}
