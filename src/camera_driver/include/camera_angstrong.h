#ifndef CAMERA_ANGSTRONG_H
#define CAMERA_ANGSTRONG_H

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <as_camera_sdk_api.h>
#include "config.h"
#include "camera_msgs/msg/camera_parameters.hpp"
#include <map>
#include <mutex>
#include <memory>

class PoseEstimator;
class CameraAngstrong
{
public:
    CameraAngstrong(std::shared_ptr<rclcpp::Node> parent,
        const std::vector<CameraConfig>& configs, const std::string& config_path,
        PoseEstimator *pose_estimator);
    ~CameraAngstrong();

    bool initialize();
    void stop();

    // 相机信息结构
    struct AngstrongInfo
    {
        CameraConfig config;

        // ROS发布器
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr color_pub;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_pub, chassis_depth_pub;
        rclcpp::Publisher<camera_msgs::msg::CameraParameters>::SharedPtr param_pub, chassis_param_pub;
        // 参数发布定时器
        rclcpp::TimerBase::SharedPtr param_timer;
        size_t last_param, last_chassis_param;
        uint64_t color_count, depth_count;

        // 相机参数
        bool param_ready;
        Intrinsics rgb_intrinsics;
        camera_msgs::msg::CameraParameters param_msg;

        AngstrongInfo() : last_param(0), last_chassis_param(0),
            color_count(0), depth_count(0), param_ready(false) {}
    };

private:
    void createPublishers(AngstrongInfo& info);
    bool getParameter(AS_CAM_PTR device, AngstrongInfo& info);
    sensor_msgs::msg::Image downsampleDepth(const sensor_msgs::msg::Image& msg);
    // SDK回调函数
    static void onAttachedCallback(AS_CAM_ATTR_S* attr, void* private_data);
    static void onDetachedCallback(AS_CAM_ATTR_S* attr, void* private_data);
    static void streamCallbackStatic(AS_CAM_PTR device,
        const AS_SDK_Data_s* pst_data, void* private_data);

    // 内部处理函数
    void processCameraAttached(AS_CAM_ATTR_S* attr);
    void processCameraDetached(AS_CAM_ATTR_S* attr);
    void processCameraFrame(AS_CAM_PTR device, const AS_SDK_Data_s* pst_data);

    // 成员变量
    std::shared_ptr<rclcpp::Node> parent_;
    std::vector<CameraConfig> configs_;
    std::string config_path_;

    PoseEstimator* pose_estimator_{nullptr};

    std::map<AS_CAM_PTR, AngstrongInfo> angstrongs_;
    std::list<AS_CAM_PTR> device_list_;
    std::mutex mutex_;
    std::atomic<bool> running_;
};

#endif // ASCAMERA_MANAGER_HPP