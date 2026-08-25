#ifndef CAMERA_BERXEL_H
#define CAMERA_BERXEL_H 

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <BerxelHawkContext.h>
#include <BerxelHawkDevice.h>
#include <BerxelHawkFrame.h>
#include <atomic>
#include <condition_variable>
#include <deque>
#include <thread>
#include "config.h"
#include "camera_msgs/msg/camera_parameters.hpp"

using namespace berxel;

class PoseEstimator;
class CameraBerxel
{
public:
    CameraBerxel(std::shared_ptr<rclcpp::Node> parent,
        const CameraConfig& config, PoseEstimator *pose_estimator);
    ~CameraBerxel();

    bool initialize();
    bool start();
    void stop();

    static void destroy();

private:
    struct FrameCopy
    {
        uint64_t timestamp{0};
        int width{0};
        int height{0};
        BerxelHawkPixelType pixel_type{BERXEL_HAWK_PIXEL_INVALID_TYPE};
        std::vector<uint8_t> data;
    };

    void createPublishers();
    bool getParameter();
    // SDK回调函数
    static void __attribute__((__stdcall__)) onDeviceStatusChange(
        const char* address, const char* serial_number,
        BerxelHawkDeviceStatus device_state, void* user_data);
    static void __attribute__((__stdcall__)) onNewFrameCallback(
        BerxelHawkStreamType stream_type,
        BerxelHawkFrame* p_frame,
        void* user_data);
    int32_t processDeviceStatusChange(const char* address,
        const char* serial_number,
        BerxelHawkDeviceStatus device_state);
    // 内部处理函数
    void processNewFrame(BerxelHawkStreamType stream_type,
        BerxelHawkFrame* p_frame);
    void enqueueFrame(BerxelHawkStreamType stream_type,
        BerxelHawkFrame* p_frame);
    void startFrameWorkers();
    void stopFrameWorkers();
    void colorWorkerMain();
    void depthWorkerMain();
    void processColorFrame(const FrameCopy& frame);
    void processDepthFrame(const FrameCopy& frame);
    void processIrFrame(BerxelHawkFrame* pFrame);
    // 图像转换函数
    void convertDepthToCv16UC(uint16_t* pDepth, uint16_t* pCv16UC1,
        int width, int height, BerxelHawkPixelType pixelType);
    void tryPublishRgbCloud();
    bool isRgbDepPair(
        const std::vector<uint16_t> &dep, const std::vector<uint8_t> &rgb,
        const uint64_t dep_ts, const uint64_t rgb_ts,
        sensor_msgs::msg::PointCloud2 &cloud_msg);
    void removeFrames(bool rgb, size_t num);

    // 相机参数
    bool param_ready_;
    Intrinsics rgb_intrinsics_;
    camera_msgs::msg::CameraParameters param_msg_;

    std::shared_ptr<rclcpp::Node> parent_;
    CameraConfig config_;

    PoseEstimator* pose_estimator_{nullptr};

    // 图像缓存
    std::deque<std::vector<uint8_t>> rgbs_;
    std::deque<std::vector<uint16_t>> deps_;
    std::deque<uint64_t> rgbs_ts_, deps_ts_;
    std::mutex rgb_mutex_, dep_mutex_;

    // SDK对象
    BerxelHawkDeviceInfo device_info_;
    BerxelHawkDevice* device_;

    // ROS发布器
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr color_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_pub_;
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr direct_color_pub_;
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr direct_depth_pub_;
    rclcpp::Publisher<camera_msgs::msg::CameraParameters>::SharedPtr param_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
    // 参数发布定时器
    rclcpp::TimerBase::SharedPtr param_timer_;
    size_t last_sub_count_;

    // 流配置
    int stream_type_;
    std::mutex frame_mutex_;

    // SDK callback only copies the newest frame.  Conversion and ROS publish
    // run independently so a slow codec/subscriber cannot block acquisition.
    std::atomic<bool> frame_workers_running_{false};
    std::thread color_worker_;
    std::thread depth_worker_;
    std::mutex color_pending_mutex_;
    std::mutex depth_pending_mutex_;
    std::condition_variable color_pending_cv_;
    std::condition_variable depth_pending_cv_;
    FrameCopy pending_color_;
    FrameCopy pending_depth_;
    bool color_pending_{false};
    bool depth_pending_{false};

    // 时间同步
    rclcpp::Time last_sync_time_;
    inline static int max_sync_gap_s_ = 5;

    PointCloudOptions cloud_op_;

    inline static BerxelHawkContext* context_ = nullptr;
};

#endif
