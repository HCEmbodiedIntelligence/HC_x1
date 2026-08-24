#ifndef CAMERA_DRIVER_H
#define CAMERA_DRIVER_H

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <memory>
#include <map>
#include <string>
#include <vector>
#include <mutex>
#include "config.h"
#include "pose_estimator.h"
#include "pose3d.h"

// 前向声明相机管理器
class CameraBerxel;
class CameraAngstrong;

class CameraDriver : public rclcpp::Node 
{
public:
    CameraDriver();
    ~CameraDriver();

    bool initialize();
    void stop();

private:
    void loadBasicInfo();
    void loadCameraConfigs();
    bool loadCameraParameters();
    bool loadCalibrations();
    void setupDefaultBasicInfo();
    void setupDefaultCameraConfigs();
    void saveBasicInfo();

    // camera pose estimator
    std::unique_ptr<PoseEstimator> pose_estimator_;

    // camera driver
    std::map<std::string, std::unique_ptr<CameraBerxel>> berxels_;
    std::unique_ptr<CameraAngstrong> angstrongs_;
    std::vector<std::string> camera_names_;
    std::map<std::string, CameraConfig> configs_;
    std::string angstrong_file_;

    // redis info
    RedisInfo body_info_, chassis_info_;

    // use body, chassis data
    bool use_body_, use_chassis_;
    // offset of chassis and body
    int64_t body_offset_ms_, chassis_offset_ms_;

    // debug
    bool debug_mode_;

    // body info
    bool fold_, fixed_head_;
    double calf_length_, thigh_length_;

    // calibration
    std::map<std::string, Pose3D> calib_map_;

    std::atomic<bool> running_;
    std::string version_;
};

#endif // COMBINED_CAMERA_DRIVER_HPP