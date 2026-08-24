// file: include/advanced_pose_estimator.hpp
#ifndef ADVANCED_POSE_ESTIMATOR_HPP
#define ADVANCED_POSE_ESTIMATOR_HPP

#include <rclcpp/rclcpp.hpp>
#include <deque>
#include <mutex>
#include <atomic>
#include <map>
#include <string>
#include <optional>
#include <thread>
#include <array>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <yaml-cpp/yaml.h>
#include <functional>
#include "axis_transform.h"
#include "rediswrapper.h"
#include "config.h"
#include "camera_msgs/msg/camera_pose.hpp"

// ==================== 数据结构定义 ====================

// body关节状态
struct BodyState
{
    std::array<double, 3> joint_angles;  // [θ1, θ2, θ3] 弧度
    uint64_t ts_ns;

    BodyState() : joint_angles{0, 0, 0}, ts_ns(0) {}
    BodyState(double a1, double a2, double a3, uint64_t ts)
        : joint_angles{a1, a2, a3}, ts_ns(ts) {}
};

// chassis位姿（二维刚体变换）
struct SE2Pose
{
    double x;          // X坐标 (米)
    double y;          // Y坐标 (米)
    double theta;      // 朝向角 (弧度，绕Z轴)
    uint64_t ts_ns;  // 时间戳 (纳秒)

    SE2Pose() : x(0), y(0), theta(0), ts_ns(0) {}
    SE2Pose(double x_, double y_, double theta_, uint64_t ts)
        : x(x_), y(y_), theta(theta_), ts_ns(ts) {}

    // SE(2)逆变换
    SE2Pose inverse() const;
    // 复合变换
    SE2Pose operator*(const SE2Pose& other) const;
    // 规范化角度到 [-π, π)
    void normalizeAngle();
};

// 位姿请求
struct PoseRequest
{
    bool send_chassis;
    uint64_t ts_ns;
    std::string cam_name;

    PoseRequest(uint64_t ts, const std::string& cam, bool send=false)
        : send_chassis(send), ts_ns(ts), cam_name(cam) {}
};

// ==================== Body插值器 ====================

class BodyInterpolator
{
public:    
    explicit BodyInterpolator(const size_t max_history=1000)
        : max_history_(max_history) {}

    // 添加关节状态
    void addBodyState(const BodyState& state);    
    // Catmull-Rom样条插值
    std::optional<std::array<double, 3>> catmullRomInterpolate(
        uint64_t ts_ns) const;
    // 线性插值（备用）
    std::optional<std::array<double, 3>> linearInterpolate(
        uint64_t ts_ns) const;
    // 预测函数
    std::optional<std::array<double, 3>> predictExtrapolate(
        uint64_t future_ts_ns) const;
    // 获取最新状态
    std::optional<BodyState> getLatestState() const;
    // 清理旧数据
    void cleanup(uint64_t current_time_ns, 
        uint64_t keep_duration_ns = 2'000'000'000ULL);
    size_t size() const;
    // 前向运动学（3自由度平面机械臂）
    static std::array<double, 3> forwardKinematics(
        const std::array<double, 3>& joint_angles,
        const double calf=0.38, const double thigh=0.35);

private:
    // 一维Catmull-Rom样条
    static double catmullRom1D(double p0, double p1,
        double p2, double p3, double t);
    // 规范化角度到 [-π, π]
    static double normalizeAngle(double angle);

private:
    std::deque<BodyState> history_;
    mutable std::mutex mutex_;
    size_t max_history_;
};

// ==================== Chassis插值器 ====================

class SE2Interpolator
{
public:
    SE2Interpolator(size_t max_history=1000) : max_history_(max_history) {}

    // 添加位姿到历史
    void addPose(const SE2Pose& pose);
    // SE(2)刚体插值
    std::optional<SE2Pose> rigidInterpolate(uint64_t ts_ns) const;
    // 获取历史大小
    size_t size() const;
    // 获取最新位姿
    std::optional<SE2Pose> getLatestPose() const;
    // 清理旧数据
    void cleanup(uint64_t current_time_ns,
        uint64_t keep_duration_ns = 2'000'000'000ULL);

private:
    // 预测功能
    std::optional<SE2Pose> predictExtrapolate(
        uint64_t future_ts_ns) const;
    // 使用基于旋转和平移分离的简化方法
    static std::optional<SE2Pose> interpolateSimple(const SE2Pose& pose1,
        const SE2Pose& pose2, double alpha);
    // SE(2)刚体插值核心算法
    static std::optional<SE2Pose> interpolateRigid(const SE2Pose& pose1,
        const SE2Pose& pose2, double alpha);
    // 在切空间进行线性插值
    static SE2Pose interpolateInTangentSpace(const SE2Pose& transform,
        double alpha);

    std::deque<SE2Pose> history_;
    mutable std::mutex mutex_;
    size_t max_history_;
};

// ==================== 位姿估计器 ====================

class PoseEstimator
{
public:
    explicit PoseEstimator(rclcpp::Node::SharedPtr node);
    ~PoseEstimator();
    void setMode(bool use_body, bool use_chassis);
    bool setAxisTransform(std::map<std::string, Pose3D> &calib_map,
        bool fixed_head, double calf, double thigh);
    bool setRedis(RedisInfo& body_redis, RedisInfo& chassis_redis);
    void setTimestampOffset(int64_t body_offset_ms, int64_t chassis_offset_ms);

    // 启动估计器
    void start();
    // 停止估计器
    void stop();
    // 请求相机位姿（异步）
    void requestCameraPose(uint64_t ts_ns, const std::string& cam_name,
        bool send_chassis=false);

    // 获取统计信息
    struct Statistics
    {
        uint64_t total_requests{0};
        uint64_t processed_requests{0};
        uint64_t failed_requests{0};
        double avg_processing_time_ms{0};
        double max_processing_time_ms{0};
        size_t chassis_history_size{0};
        size_t body_history_size{0};
    };

    Statistics getStatistics() const;

private:
    // 主处理线程
    void poseThreadMain();
    // 发布相机位姿
    void publishCameraPose(const PoseRequest& request,
        const Pose3D& cam_pose, const Pose3D& base_pose,
        const std::array<double, 3> &body_angles); 
    // 处理请求队列
    void processRequestQueue();
    void processSingleRequest(const PoseRequest& request);
    // 处理延迟请求
    void processDelayedRequests();
    // 检查数据是否可用
    bool isDataAvailableForTimestamp(uint64_t ts_ns);
    // 更新统计信息
    void updateStatistics(uint64_t processing_time_ns, bool success);
    // 获取底盘位姿
    std::optional<BodyState> getBodyState();
    // 获取底盘位姿
    std::optional<SE2Pose> getChassisPose();

private:
    rclcpp::Node::SharedPtr node_;

    // 线程控制
    std::atomic<bool> running_;
    std::thread pose_thread_;

    // 插值器
    BodyInterpolator body_interpolator_;
    SE2Interpolator  chassis_interpolator_;

    // 请求队列
    std::deque<PoseRequest> request_queue_;
    std::mutex queue_mutex_;

    // 发布器
    std::map<std::string,
        rclcpp::Publisher<camera_msgs::msg::CameraPose>::SharedPtr> publishers_, chassis_publishers_;

    // Redis客户端
    std::shared_ptr<RedisWrapper> body_redis_, chassis_redis_;

    // 标定器
    std::unique_ptr<AxisTransform> atf_;
    double calf_;
    double thigh_;
    std::map<std::string, AxisLevel> cam_map_;

    // 延迟请求结构
    struct DelayedPoseRequest
    {
        PoseRequest request;
        uint64_t arrival_time_ns;
        int retry_count;

        DelayedPoseRequest(const PoseRequest& req, uint64_t arrival)
            : request(req), arrival_time_ns(arrival), retry_count(0) {}
    };
    // 延迟队列
    std::deque<DelayedPoseRequest> delayed_requests_;

    // 统计信息
    mutable std::mutex stats_mutex_;
    Statistics stats_;

    // 时间偏移
    int64_t chassis_offset_ms_, body_offset_ms_;
    bool body_not_sync_, chassis_not_sync_;

    // use body, chassis data
    bool use_body_, use_chassis_;
    bool use_time_sync_;
};

#endif // ADVANCED_POSE_ESTIMATOR_HPP
