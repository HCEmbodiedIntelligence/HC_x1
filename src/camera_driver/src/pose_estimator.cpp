#include "pose_estimator.h"
#include "camera_msgs/msg/camera_pose.hpp"
#include <iostream>
#include <fstream>
#include <chrono>
#include <algorithm>
#include <cmath>
#include <iomanip>
#include "config.h"

using namespace std::chrono_literals;

// ==================== 构造函数与析构函数 ====================
PoseEstimator::PoseEstimator(rclcpp::Node::SharedPtr node)
    : node_(node), running_(false),
    body_not_sync_(false), chassis_not_sync_(false)
{
    g_logger->info("Pose Estimator initializing...");
    cam_map_["eye"]    = AxisLevel::EYE_AXIS;
    cam_map_["chest"]  = AxisLevel::CHEST_AXIS;
    cam_map_["Bfront"] = AxisLevel::BFRONT_AXIS;
    cam_map_["Bback"]  = AxisLevel::BBACK_AXIS;
    cam_map_["Bleft"]  = AxisLevel::BLEFT_AXIS;
    cam_map_["Bright"] = AxisLevel::BRIGHT_AXIS;
    use_time_sync_ = true;
}

PoseEstimator::~PoseEstimator()
{
    stop();
}

void PoseEstimator::setMode(bool use_body, bool use_chassis)
{
    use_body_    = use_body;
    use_chassis_ = use_chassis;
    g_logger->warn("{} using body data, {} using chassis data!",
        use_body_ ? "Enable" : "Disable",
        use_chassis_ ? "Enable" : "Disable");
}

bool PoseEstimator::setAxisTransform(std::map<std::string, Pose3D> &calib_map, 
    bool fixed_head, double calf, double thigh)
{
    g_logger->info("Initializing Axis Transform ...");
    calf_  = calf;
    thigh_ = thigh;
    // 加载标定参数
    try
    {
        atf_ = std::make_unique<AxisTransform>(calib_map, fixed_head, calf, thigh);
    }
    catch (const std::runtime_error& e)
    {
        g_logger->error("Failed to initialize axis transformer: {}!", e.what());
        return false;
    } 
    return true;
}

bool PoseEstimator::setRedis(RedisInfo& body_info, RedisInfo& chassis_info)
{
    g_logger->info("Trying to create redis client...");
    try
    {
        if(use_body_)
            body_redis_ = std::make_shared<RedisWrapper>(
                body_info.host, body_info.port, body_info.pw);
        if(use_chassis_)
        {
            if(use_body_ && body_info.host == chassis_info.host)
                chassis_redis_ = body_redis_;
            else
                chassis_redis_ = std::make_shared<RedisWrapper>(chassis_info.host,
                chassis_info.port, chassis_info.pw);
        }
    }
    catch (const std::runtime_error& e)
    {
        g_logger->error("Failed to connect to Redis: {}", e.what());
        return false;
    }
    catch (const std::exception& e)
    {
        g_logger->error("Failed to initinalize Redis: {}", e.what());
        return false;
    }
    return true;
}

void PoseEstimator::setTimestampOffset(int64_t body_offset_ms, int64_t chassis_offset_ms)
{
    body_offset_ms_    = body_offset_ms;
    chassis_offset_ms_ = chassis_offset_ms;
    g_logger->info("Set timestamp offset: body {} ms, chassis {} ms",
        body_offset_ms_, chassis_offset_ms_);
}

// ====================== 公共接口 ======================
void PoseEstimator::start()
{
    if (running_.load()) return;    
    running_.store(true);
    pose_thread_ = std::thread(&PoseEstimator::poseThreadMain, this);
    g_logger->info("Pose Estimator thread started.");
}

void PoseEstimator::stop()
{
    if(!running_.load()) return;
    running_.store(false);
    g_logger->info("Stopping Pose Estimator...");
    if (pose_thread_.joinable()) 
        pose_thread_.join();
}

void PoseEstimator::requestCameraPose(uint64_t ts_ns, const std::string& cam_name,
    bool send_chassis)
{
    std::lock_guard<std::mutex> lock(queue_mutex_);
    request_queue_.emplace_back(PoseRequest(ts_ns, cam_name, send_chassis));
    {
        std::lock_guard<std::mutex> lock(stats_mutex_);
        stats_.total_requests++;
    }
    if(request_queue_.size() > 1000) 
        request_queue_.pop_front();    
}

// ==================== 位姿线程主函数 ====================
void PoseEstimator::poseThreadMain()
{
    g_logger->info("Initializing Pose Estimator thread ...");

    auto last_redis   = std::chrono::steady_clock::now();
    auto last_cleanup = last_redis;
    auto last_stats   = last_redis;

    while (running_.load())
    {
        auto start = std::chrono::steady_clock::now();
        // 1. 从Redis获取最新的机器人状态（100Hz）
        if (std::chrono::duration<double>(start - last_redis).count() >= 0.01)
        {
            if(use_body_)
            {
                auto body_joints = getBodyState();
                if (body_joints)
                {
                    body_interpolator_.addBodyState(body_joints.value());
                    g_logger->debug("{} New body angles: ankle={:.1f}, knee={:.1f}, waist={:.1f}",
                        tsNsToStr((*body_joints).ts_ns),
                        (*body_joints).joint_angles[0],
                        (*body_joints).joint_angles[1],
                        (*body_joints).joint_angles[2]);
                }
            }
            if(use_chassis_)
            {
                auto chassis_pose = getChassisPose();
                if (chassis_pose)
                {
                    chassis_interpolator_.addPose(chassis_pose.value());
                    g_logger->debug("{} New chassis pose: x={:.3f}, y={:.3f}, theta={:.1f}", 
                        tsNsToStr((*chassis_pose).ts_ns),
                        (*chassis_pose).x, (*chassis_pose).y, (*chassis_pose).theta * 180.0/M_PI);
                }
            }
            last_redis = start;
        }
        auto update_t = std::chrono::steady_clock::now();

        // 2. 处理位姿请求
        processRequestQueue();
        auto process_queue_t = std::chrono::steady_clock::now();    

        // 3. 定期清理旧数据（1Hz）
        if (std::chrono::duration<double>(start - last_cleanup).count() >= 1.0)
        {
            uint64_t current_time_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::system_clock::now().time_since_epoch()).count();
            if(use_body_)    body_interpolator_.cleanup(current_time_ns);
            if(use_chassis_) chassis_interpolator_.cleanup(current_time_ns);
            last_cleanup = start;
        }

        // 4. 打印统计信息（0.05Hz）
        if (std::chrono::duration<double>(start - last_stats).count() >= 30.0) 
        {
            auto stats = getStatistics();
            if (stats.processed_requests > 0) 
            {
                double success_rate = static_cast<double>(stats.processed_requests) * 
                    100.0 / stats.total_requests;
                g_logger->info("=== Pose Estimator Statistics ===");
                g_logger->info("Requests: {} total, {} processed, {} failed.", 
                    stats.total_requests, stats.processed_requests, stats.failed_requests);
                g_logger->info("Success rate: {:.1f}%.", success_rate);
                g_logger->info("Avg processing time: {:.1f} ms", stats.avg_processing_time_ms);
                g_logger->info("Max processing time: {:.1f} ms", stats.max_processing_time_ms);
                g_logger->info("History sizes: chassis={}, body={}", stats.chassis_history_size,
                    stats.body_history_size);
                g_logger->info("=================================");
            }
            last_stats = start;
        }

        // 5. 控制循环频率（200Hz）
        auto cycle_time = std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::steady_clock::now() - start).count();
        int64_t sleep_time_us = 5000 - cycle_time; // 5ms = 5000us
        if (sleep_time_us > 0)
            std::this_thread::sleep_for(std::chrono::microseconds(sleep_time_us));
        else if (sleep_time_us < -10'000)
        {
            auto update_time = std::chrono::duration_cast<std::chrono::microseconds>(
                update_t - start).count();
            auto process_time = std::chrono::duration_cast<std::chrono::microseconds>(
                process_queue_t - update_t).count();
            auto clean_time = std::chrono::duration_cast<std::chrono::microseconds>(
                std::chrono::steady_clock::now() - process_queue_t).count();
            g_logger->info("Cycle total: {:.1f} ms, update: {:.1f} ms, "
                "process: {:.1f} ms, clean: {:.1f} ms",
                cycle_time/1000.0, update_time/1000.0, 
                process_time/1000.0, clean_time/1000.0);
        }
    }

    g_logger->info("Pose Estimator thread stopped.");
}

// ==================== Redis数据获取 ====================
// 获取腰部位姿
std::optional<BodyState> PoseEstimator::getBodyState()
{
    try 
    {
        // 1. 从Redis获取位姿JSON
        static bool redis_warn = false;
        auto pose_json_opt = body_redis_->GetJson("waist_status");
        if (!pose_json_opt)
        {
            if(!redis_warn)
            {
                redis_warn = true;
                g_logger->error("No body state data found in Redis!");
            }
            return std::nullopt;
        }
        else if(redis_warn)
        {
            redis_warn = false;
            g_logger->warn("Body state data restored in Redis!");
        }
        const auto& pose_json = *pose_json_opt;

        // 2. 更新超时检测
        int64_t ts_ns = pose_json["timestamp"].get<int64_t>();
        static bool timeout_warn = false, align_warn = false;
        static int64_t last_ts_ns = 0;
        int64_t timeout_diff_ns = ts_ns - last_ts_ns;
        if(timeout_diff_ns > 100e6)
        {
            if(!timeout_warn)
            {
                timeout_warn = true;
                g_logger->error("Body state update timeout: {:.1f} ms!",
                    timeout_diff_ns / 1e6);
            }
        }
        else
        {
            if(timeout_warn)
            {
                timeout_warn = false;
                g_logger->warn("Body state update restored! TS: {}!",
                    tsNsToStr(ts_ns));
            }
            // 时间对齐检测
            auto now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::system_clock::now().time_since_epoch()).count();
            int64_t align_diff_ns = std::llabs(static_cast<int64_t>(now_ns) - ts_ns);
            if(align_diff_ns > 100e6)
            {
                body_not_sync_ = true;
                if(!align_warn)
                {
                    align_warn = true;
                    g_logger->error("Body state timestamp is too far from current time! "
                        "Now: {}, Bosy: {}",
                        tsNsToStr(now_ns), tsNsToStr(ts_ns));
                }
                return std::nullopt;
            }
            else
            {
                body_not_sync_ = false;
                if(align_warn)
                {
                    align_warn = false;
                    g_logger->warn("Body state timestamp is now in sync with current time!");
                }
            }
        }
        if (ts_ns <= last_ts_ns) return std::nullopt;
        last_ts_ns = ts_ns;

        // 3. 提取最新状态
        BodyState state;
        state.ts_ns = static_cast<uint64_t>(ts_ns + body_offset_ms_ * 1'000'000);

        state.joint_angles[0] = -pose_json["joint"][0].get<double>() * M_PI / 180.0;
        state.joint_angles[1] =  pose_json["joint"][1].get<double>() * M_PI / 180.0;
        state.joint_angles[2] =  pose_json["joint"][2].get<double>() * M_PI / 180.0;

        return state;
    } 
    catch (const nlohmann::json::exception& e) 
    {
        g_logger->error("JSON parse error for body state: {}!", e.what());
        return std::nullopt;
    } 
    catch (const std::exception& e) 
    {
        g_logger->error("Error getting body state: {}!", e.what());
        return std::nullopt;
    }
}

// 获取底盘位姿
std::optional<SE2Pose> PoseEstimator::getChassisPose()
{
    try 
    {
        auto start = std::chrono::steady_clock::now();
        // 1. 从Redis获取位姿JSON
        static bool redis_warn = false;
        auto pose_json_opt = chassis_redis_->GetJson("agvState");
        auto end_get = std::chrono::steady_clock::now();
        if (!pose_json_opt) 
        {
            if(!redis_warn)
            {
                redis_warn = true;
                g_logger->error("No chassis pose data found in Redis!");
            }
            return std::nullopt;
        }
        else if(redis_warn)
        {
            redis_warn = false;
            g_logger->warn("Chassis pose data restored in Redis!");
        }
        const auto& pose_json = *pose_json_opt;

        // 2. 更新超时检测
        int64_t ts_ms = pose_json["position"]["timeStamp"].get<int64_t>();
        static bool timeout_warn = false, align_warn = false;
        static int64_t last_ts_ms = 0;
        int64_t timeout_diff_ms = ts_ms - last_ts_ms;
        if(timeout_diff_ms > 100)
        {
            if(!timeout_warn)
            {
                timeout_warn = true;
                g_logger->error("Chassis pose update timeout: {} ms!", timeout_diff_ms);
            }
        }
        else
        {
            if(timeout_warn)
            {
                timeout_warn = false;
                g_logger->warn("Chassis pose update restored! TS: {}!", 
                    tsNsToStr(ts_ms * 1'000'000));
            }
            // 时间对齐检测
            auto now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::system_clock::now().time_since_epoch()).count();
            int64_t align_diff_ms = std::llabs(static_cast<int64_t>(now_ms) - ts_ms);
            if(align_diff_ms > 100)
            {
                chassis_not_sync_ = true;
                if(!align_warn)
                {
                    align_warn = true;
                    g_logger->warn("Chassis pose timestamp is too far from current time! "
                        "Now: {}, Chassis: {}",
                        tsNsToStr(now_ms * 1'000'000), tsNsToStr(ts_ms * 1'000'000));
                }
                return std::nullopt;
            }
            else
            {
                chassis_not_sync_ = false;
                if(align_warn)
                {
                    align_warn = false;
                    g_logger->warn("Chassis pose timestamp is now in sync with current time!");
                }
            }
        }
        auto end_out = std::chrono::steady_clock::now();
        if (ts_ms <= last_ts_ms) return std::nullopt;
        last_ts_ms = ts_ms;

        // 3. 提取底盘位姿
        SE2Pose pose;  
        pose.ts_ns = static_cast<uint64_t>(ts_ms + chassis_offset_ms_)* 1'000'000ULL;
        // 提取位置 (x, y)
        pose.x = pose_json["position"]["x"].get<double>();
        pose.y = pose_json["position"]["y"].get<double>();
        // 提取角度 (angle)
        pose.theta = pose_json["position"]["angle"].get<double>();

        auto end = std::chrono::steady_clock::now();
        auto all_time = std::chrono::duration_cast<std::chrono::microseconds>(
            end - start).count();
        auto get_time = std::chrono::duration_cast<std::chrono::microseconds>(
            end_get - start).count();
        auto out_time = std::chrono::duration_cast<std::chrono::microseconds>(
            end_out - end_get).count();
        auto assign_time = std::chrono::duration_cast<std::chrono::microseconds>(
            end - end_out).count();
        g_logger->debug("getChassisPose x={:.3f}, y={:.3f}, theta={:.1f}. "
            "total: {:.1f} ms, get: {:.1f} ms, timeout: {:.1f} ms, assign: {:.1f} ms",
            pose.x, pose.y, pose.theta,
            all_time/1000.0, get_time/1000.0, out_time/1000.0,
            assign_time/1000.0);

        return pose;
    } 
    catch (const nlohmann::json::exception& e) 
    {
        g_logger->error("JSON parse error for chassis pose: {}!", e.what());
        return std::nullopt;
    } 
    catch (const std::exception& e) 
    {
        g_logger->error("Error getting chassis pose: {}!", e.what());
        return std::nullopt;
    }
}

// ===================== 处理请求队列 ====================
void PoseEstimator::processRequestQueue()
{
    if((use_body_ && body_not_sync_) || (use_chassis_ && chassis_not_sync_))
    {
        std::lock_guard<std::mutex> lock1(queue_mutex_);        
        std::lock_guard<std::mutex> lock2(stats_mutex_);
        stats_.total_requests -= request_queue_.size();
        request_queue_.clear();
        return;
    }

    // 1. 批量获取请求
    std::vector<PoseRequest> requests;
    {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        if (!request_queue_.empty())
        {
            // 一次处理最多100个请求
            size_t count = std::min(request_queue_.size(), static_cast<size_t>(100));
            requests.reserve(count);

            for (size_t i = 0; i < count; i++) 
            {
                requests.push_back(request_queue_.front());
                request_queue_.pop_front();
            }
        }
    }

    // 2. 处理新请求
    for (const auto& req : requests)
    {
        if (isDataAvailableForTimestamp(req.ts_ns))
        {
            // 立即处理
            processSingleRequest(req);
        } 
        else
        {
            // 放入延迟队列
            auto now_ns = static_cast<int64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::system_clock::now().time_since_epoch()).count());
            delayed_requests_.emplace_back(req, now_ns);
        }
    }

    // 3. 处理延迟队列
    processDelayedRequests();
}

void PoseEstimator::processSingleRequest(const PoseRequest& request)
{ 
    try 
    {
        auto processing_start = std::chrono::steady_clock::now(); 
        // 计算相机位姿
        Pose3D camera_pose;
        std::optional<std::array<double, 3>> body_angles = std::nullopt;
        if(use_body_)
        {
            if(use_time_sync_)
                body_angles = body_interpolator_.catmullRomInterpolate(request.ts_ns);
            else
            {
                auto body = body_interpolator_.getLatestState();
                if(body) body_angles = body->joint_angles;
            }
            if (!body_angles)
            {
                g_logger->error("[{}] {} No body state available for timestamp: {}!",
                    request.cam_name, currentTimeStr(), tsNsToStr(request.ts_ns));
                updateStatistics(0, false);
                return;
            }
            atf_->setHeadVar(0);
            atf_->setWaistVar(body_angles.value()[0], body_angles.value()[1],
                body_angles.value()[2]);
            camera_pose = atf_->getRelativePose(cam_map_[request.cam_name],
                AxisLevel::CHASSIS_AXIS);
        }
        else camera_pose = Pose3D(0.1, 0.1, 0.1, 0, 0, 0);
        g_logger->debug("[{}] {} Estimate body state: ({:.1f}, {:.1f}, {:.1f})",
            request.cam_name, tsNsToStr(request.ts_ns),
            body_angles.value()[0] * 180 / M_PI, body_angles.value()[1] * 180 / M_PI,
            body_angles.value()[2] * 180 / M_PI);

        // 获取底盘位姿
        Pose3D chassis_pose;
        std::optional<SE2Pose> chassis = std::nullopt;
        if(use_chassis_)
        {
            if(use_time_sync_)
                chassis = chassis_interpolator_.rigidInterpolate(request.ts_ns);
            else
                chassis = chassis_interpolator_.getLatestPose();
            if (!chassis)
            {
                g_logger->error("[{}] {} No base pose available for timestamp: {}!",
                    request.cam_name, currentTimeStr(), tsNsToStr(request.ts_ns));
                updateStatistics(0, false);
                return;
            }
            chassis_pose = Pose3D(chassis.value().x, chassis.value().y, 0,
                chassis.value().theta, 0.0, 0.0);
        }
        else chassis_pose = Pose3D(0.2, 0.2, 0.2, 0, 0, 0);
        g_logger->debug("[{}] {} Estimate chassis pose: ({:.4f}, {:.4f}, {:.1f})",
            request.cam_name, tsNsToStr(request.ts_ns),
            chassis.value().x, chassis.value().y, chassis.value().theta * 180 / M_PI);

        publishCameraPose(request, camera_pose, chassis_pose, body_angles.value());
        auto processing_time = std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now() - processing_start).count();
        updateStatistics(processing_time, true);
    } 
    catch (const std::exception& e)
    {
        g_logger->error("Exception while processing request: {}!", e.what());
        updateStatistics(0, false);
    }
}

// 处理延迟请求
void PoseEstimator::processDelayedRequests()
{    
    if (delayed_requests_.empty()) return;

    auto now_ns = static_cast<int64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count());
    static uint64_t MAX_WAIT_NS       = 60'000'000;  // 60ms
    static uint64_t RETRY_INTERVAL_NS = 5'000'000;   //  5ms

    auto it = delayed_requests_.begin();
    while (it != delayed_requests_.end())
    {
        DelayedPoseRequest& dreq = *it;
        // 检查是否超时
        bool timeout = (now_ns - dreq.arrival_time_ns) >= MAX_WAIT_NS;

        if (timeout)
        {
            // 超时：强制处理
            g_logger->warn("[{}] Request timeout, forcing action: waitted {:.1f} ms, "
                "retried {} times. Request: {}, latest body: {}, chassis: {}",
                dreq.request.cam_name, (now_ns - dreq.arrival_time_ns) / 1e6, dreq.retry_count,
                tsNsToStr(dreq.request.ts_ns),
                tsNsToStr(body_interpolator_.getLatestState()->ts_ns),
                tsNsToStr(chassis_interpolator_.getLatestPose()->ts_ns));
            processSingleRequest(dreq.request);
            it = delayed_requests_.erase(it);
            continue;
        }

        // 检查是否到了重试时间        
        if (now_ns >= dreq.arrival_time_ns + dreq.retry_count * RETRY_INTERVAL_NS)
        {
            // 检查数据是否可用
            if (isDataAvailableForTimestamp(dreq.request.ts_ns))
            {
                // 数据可用，处理请求
                // g_logger->debug("[{}] Process delayed request: waitted {:.1f} ms.",
                //     dreq.request.cam_name,
                //     (now_ns - dreq.arrival_time_ns) / 1e6);
                processSingleRequest(dreq.request);
                it = delayed_requests_.erase(it);
            } 
            else
            {
                // g_logger->debug("[{}] Delayed request timeout again: waitted {:.1f} ms. "
                //     "Request time: {}, latest body: {}, latest chassis: {}.",
                //     dreq.request.cam_name, (now_ns - dreq.arrival_time_ns) / 1e6,
                //     tsNsToStr(dreq.request.ts_ns),
                //     tsNsToStr(body_interpolator_.getLatestState()->ts_ns),
                //     tsNsToStr(chassis_interpolator_.getLatestPose()->ts_ns));
                // 数据不可用，增加重试计数，继续等待
                dreq.retry_count++;
                ++it;
            }
        } 
        else
        {
            // g_logger->debug("[{}] Delayed request not processed, request time: {}, "
            //     "latest body: {}, latest chassis: {}.",
            //     dreq.request.cam_name,
            //     tsNsToStr(dreq.request.ts_ns),
            //     tsNsToStr(body_interpolator_.getLatestState()->ts_ns),
            //     tsNsToStr(chassis_interpolator_.getLatestPose()->ts_ns)
            // )
            ++it;
        }
    }
}

// 检查数据是否可用
bool PoseEstimator::isDataAvailableForTimestamp(uint64_t ts_ns)
{
    bool body_available = true, chassis_available = true;
    if(use_body_)
    {
        auto latest_body = body_interpolator_.getLatestState();
        if(!latest_body || latest_body->ts_ns < ts_ns)
            body_available = false;
    }
    if(use_chassis_)
    {
        auto latest_chassis = chassis_interpolator_.getLatestPose();
        if(!latest_chassis || latest_chassis->ts_ns < ts_ns)
            chassis_available = false;
    }
    return body_available && chassis_available;
}

// ====================== 发布位姿 ======================
void PoseEstimator::publishCameraPose(const PoseRequest& request, 
    const Pose3D& cam_pose, const Pose3D& chassis_pose,
    const std::array<double, 3> &body_angles)
{
    // 1. 初始化发布器
    if (publishers_.find(request.cam_name) == publishers_.end())
    {
        std::string topic = "/cameras/" + request.cam_name + "/pose";
        auto pose_qos = rclcpp::QoS(rclcpp::KeepLast(3)).reliable();
        publishers_[request.cam_name] = node_->create_publisher<camera_msgs::msg::CameraPose>(
            topic, pose_qos);
        g_logger->info("[{}] Created pose publisher topic: {}.", request.cam_name, topic);
    }
    if (request.send_chassis &&
        chassis_publishers_.find(request.cam_name) == chassis_publishers_.end())
    {
        std::string topic = "/cameras/" + request.cam_name + "/pose_chassis";
        auto pose_qos = rclcpp::QoS(rclcpp::KeepLast(3)).reliable();
        chassis_publishers_[request.cam_name] =
            node_->create_publisher<camera_msgs::msg::CameraPose>(topic, pose_qos);
        g_logger->info("[{}] Created chassis pose publisher topic: {}.",
            request.cam_name, topic);
    }

    // 2. 设置时间戳
    camera_msgs::msg::CameraPose msg;
    msg.timestamp.sec = static_cast<int32_t>(request.ts_ns / 1'000'000'000);
    msg.timestamp.nanosec = static_cast<uint32_t>(request.ts_ns % 1'000'000'000);

    // 3. 设置相机平移 (cam_translation[3])
    Eigen::Vector3d cam_trans = cam_pose.translation();
    msg.cam_translation[0] = static_cast<float>(cam_trans.x());
    msg.cam_translation[1] = static_cast<float>(cam_trans.y());
    msg.cam_translation[2] = static_cast<float>(cam_trans.z());

    // 4. 设置相机欧拉角 (cam_euler_angles[3])
    // 从旋转矩阵或四元数转换为欧拉角（弧度）
    Eigen::Vector3d cam_euler = cam_pose.eulerAngles(); // ZYX顺序：yaw, pitch, roll
    msg.cam_euler_angles[0] = static_cast<float>(cam_euler[0]);  // yaw (绕Z轴)
    msg.cam_euler_angles[1] = static_cast<float>(cam_euler[1]);  // pitch (绕Y轴)
    msg.cam_euler_angles[2] = static_cast<float>(cam_euler[2]);  // roll (绕X轴)

    // 5. 设置底盘平移 (base_translation[2])
    Eigen::Vector3d base_trans = chassis_pose.translation();
    msg.base_translation[0] = static_cast<float>(base_trans.x());
    msg.base_translation[1] = static_cast<float>(base_trans.y());

    // 6. 设置底盘朝向角 (base_theta)
    Eigen::Vector3d base_euler = chassis_pose.eulerAngles(); // ZYX顺序：yaw, pitch, roll
    msg.base_theta = static_cast<float>(base_euler[0]);

    // 7. 设置躯干旋转角 (body_angles)
    msg.body_angles[0] = static_cast<float>(body_angles[0]);
    msg.body_angles[1] = static_cast<float>(body_angles[1]);
    msg.body_angles[2] = static_cast<float>(body_angles[2]);

    // 8. 设置躯干状态 (body_states)
    std::array<double, 3> body_states = BodyInterpolator::forwardKinematics(body_angles);
    msg.body_states[0] = static_cast<float>(body_states[0]);
    msg.body_states[1] = static_cast<float>(body_states[1]);
    msg.body_states[2] = static_cast<float>(body_states[2]);

    // 7. 发布消息
    publishers_[request.cam_name]->publish(msg);
    if(request.send_chassis)
        chassis_publishers_[request.cam_name]->publish(std::move(msg));

    g_logger->debug("[{}] Publish pose: \n"
        "  camera  pose: ({:.4f}, {:.4f}, {:.4f}) ({:.1f}, {:.1f}, {:.1f})\n"
        "  chassis pose: ({:.4f}, {:.4f}, {:.1f})\n"
        "  body  angles: ({:.1f}, {:.1f}, {:.1f})",
        request.cam_name, tsNsToStr(request.ts_ns),
        cam_trans.x(), cam_trans.y(), cam_trans.z(),
        cam_euler[0] * 180.0 / M_PI, cam_euler[1] * 180.0 / M_PI, cam_euler[2] * 180.0 / M_PI,
        base_trans.x(), base_trans.y(), base_euler[0] * 180.0 / M_PI,
        body_angles[0] * 180.0 / M_PI, body_angles[1] * 180.0 / M_PI,
        body_angles[2] * 180.0 / M_PI);
}

// ====================== 统计信息 ======================
void PoseEstimator::updateStatistics(uint64_t processing_time_ns, bool success)
{
    std::lock_guard<std::mutex> lock(stats_mutex_);

    if (success) 
    {
        stats_.processed_requests++;        
        double processing_time_ms = processing_time_ns / 1e6;

        // 更新平均处理时间（指数移动平均）
        if (stats_.processed_requests == 1) 
            stats_.avg_processing_time_ms = processing_time_ms;
        else 
            stats_.avg_processing_time_ms = 
                0.9 * stats_.avg_processing_time_ms + 0.1 * processing_time_ms;

        // 更新最大处理时间
        stats_.max_processing_time_ms = 
            std::max(stats_.max_processing_time_ms, processing_time_ms);
    } 
    else stats_.failed_requests++;
}

PoseEstimator::Statistics PoseEstimator::getStatistics() const
{
    std::lock_guard<std::mutex> lock(stats_mutex_);
    Statistics result = stats_;
    result.chassis_history_size = chassis_interpolator_.size();
    result.body_history_size = body_interpolator_.size();
    return result;
}


// ==================== Body插值器 ====================
// 添加关节状态
void BodyInterpolator::addBodyState(const BodyState& state)
{
    // 保持时间顺序
    auto it = std::lower_bound(history_.begin(), history_.end(), state,
        [](const BodyState& a, const BodyState& b) {
            return a.ts_ns < b.ts_ns;
        });
    history_.insert(it, state);
    if (history_.size() > max_history_)
        history_.pop_front();
}

// Catmull-Rom样条插值
std::optional<std::array<double, 3>> BodyInterpolator::catmullRomInterpolate(uint64_t ts_ns) const
{
    return linearInterpolate(ts_ns);
    if (history_.size() < 4) return linearInterpolate(ts_ns);

    // 找到最近的4个点用于Catmull-Rom插值
    auto it = std::lower_bound(
        history_.begin(), history_.end(), ts_ns,
        [](const BodyState& state, uint64_t ts) {
            return state.ts_ns < ts;
        });

    if (it == history_.begin())
        it++;
    if (it == history_.end() || (it + 1) == history_.end())
        return linearInterpolate(ts_ns);

    // 获取4个控制点：p0, p1, p2, p3
    auto p1_it = it - 1;
    auto p2_it = it;
    auto p0_it = (p1_it == history_.begin()) ? p1_it : p1_it - 1;
    auto p3_it = (p2_it + 1 == history_.end()) ? p2_it : p2_it + 1;

    const BodyState& p0 = *p0_it;
    const BodyState& p1 = *p1_it;
    const BodyState& p2 = *p2_it;
    const BodyState& p3 = *p3_it;
    // 计算归一化时间
    double t = static_cast<double>(ts_ns - p1.ts_ns) /
                static_cast<double>(p2.ts_ns - p1.ts_ns);
    // 对每个关节进行Catmull-Rom插值
    std::array<double, 3> interpolated_angles;
    for (int i = 0; i < 3; i++)
    {
        interpolated_angles[i] = catmullRom1D(
            p0.joint_angles[i], p1.joint_angles[i],
            p2.joint_angles[i], p3.joint_angles[i], t);
        // 处理角度环绕
        interpolated_angles[i] = normalizeAngle(interpolated_angles[i]);
    }

    return interpolated_angles;
}

// 线性插值（备用）
std::optional<std::array<double, 3>> BodyInterpolator::linearInterpolate(uint64_t ts_ns) const
{
    if (history_.size() < 2)
    {
        g_logger->warn("Not enough body history to interpolate!");
        return history_.empty() ? std::nullopt :
            std::make_optional(history_.back().joint_angles);
    }
    auto it_upper = std::lower_bound(
        history_.begin(), history_.end(), ts_ns,
        [](const BodyState& state, uint64_t ts) {
            return state.ts_ns < ts;
        });

    if (it_upper == history_.begin()) return it_upper->joint_angles;
    if (it_upper == history_.end())
    {
        g_logger->warn("Out of history records of body state: {} ms!",
            (ts_ns - history_.back().ts_ns) / 1000000);
        return predictExtrapolate(ts_ns);
    }

    const BodyState& state1 = *(it_upper - 1);
    const BodyState& state2 = *it_upper;
    double alpha = static_cast<double>(ts_ns - state1.ts_ns) /
                    static_cast<double>(state2.ts_ns - state1.ts_ns);

    std::array<double, 3> interpolated_angles;
    for (int i = 0; i < 3; i++)
    {
        double delta = state2.joint_angles[i] - state1.joint_angles[i];
        // 处理角度环绕
        if (delta > M_PI) delta -= 2 * M_PI;
        if (delta < -M_PI) delta += 2 * M_PI;
        interpolated_angles[i] = state1.joint_angles[i] + alpha * delta;
        interpolated_angles[i] = normalizeAngle(interpolated_angles[i]);
    }

    return interpolated_angles;
}

// 预测函数
std::optional<std::array<double, 3>> BodyInterpolator::predictExtrapolate(
    uint64_t future_ts_ns) const
{
    // 获取最近的两个状态点
    const auto& state_n2 = history_[history_.size() - 2];
    const auto& state_n1 = history_[history_.size() - 1];

    // 计算时间差
    double dt_hist_ns = static_cast<double>(state_n1.ts_ns - state_n2.ts_ns);
    double dt_future_ns = static_cast<double>(future_ts_ns - state_n1.ts_ns);

    // 限制预测时间（例如不超过100ms）
    if (dt_future_ns > 100e6)
    {
        g_logger->error("Out of history records of body state! {:.1f} ms",
            dt_future_ns / 1e6);
        return std::nullopt;
    }

    // 计算预测比例, 避免除以零
    if (dt_hist_ns < 1e-6)
    {
        g_logger->error("Last two records of body history too short! {} ns",
            dt_hist_ns);
        return std::nullopt;
    }
    double alpha = dt_future_ns / dt_hist_ns;

    // 对每个关节角度进行线性外推
    std::array<double, 3> predicted_angles;
    for (int i = 0; i < 3; i++)
    {
        // 计算角度变化
        double delta_angle = state_n1.joint_angles[i] - state_n2.joint_angles[i];
        // 处理角度环绕（角度差值应在 [-π, π] 范围内）
        if (delta_angle > M_PI) delta_angle -= 2 * M_PI;
        if (delta_angle < -M_PI) delta_angle += 2 * M_PI;
        // 线性外推
        predicted_angles[i] = state_n1.joint_angles[i] + alpha * delta_angle;
        // 规范化角度
        predicted_angles[i] = normalizeAngle(predicted_angles[i]);
    }

    return predicted_angles;
}

// 获取最新状态
std::optional<BodyState> BodyInterpolator::getLatestState() const
{
    if (history_.empty()) return std::nullopt;
    return history_.back();  // 返回最新状态
}

// 清理旧数据
void BodyInterpolator::cleanup(uint64_t current_time_ns, uint64_t keep_duration_ns)
{
    uint64_t cutoff_time = current_time_ns - keep_duration_ns;
    while (!history_.empty() && history_.front().ts_ns < cutoff_time)
        history_.pop_front();
}

size_t BodyInterpolator::size() const
{
    return history_.size();
}

// 一维Catmull-Rom样条
double BodyInterpolator::catmullRom1D(double p0, double p1, double p2, double p3, double t)
{
    // 计算切线
    double m1 = (p2 - p0) * 0.5;
    double m2 = (p3 - p1) * 0.5;
    // 三次多项式系数
    double t2 = t * t;
    double t3 = t2 * t;
    return (2*t3 - 3*t2 + 1) * p1 +
            (t3 - 2*t2 + t) * m1 +
            (-2*t3 + 3*t2) * p2 +
            (t3 - t2) * m2;
}

// 前向运动学（3自由度平面机械臂），弧度
std::array<double, 3> BodyInterpolator::forwardKinematics(
    const std::array<double, 3>& joint_angles,
    const double calf, const double thigh)
{
    Pose3D pose1(0, 0,     0, 0, joint_angles[0], 0);
    Pose3D pose2(0, 0,  calf, 0, joint_angles[1], 0);
    Pose3D pose3(0, 0, thigh, 0, joint_angles[2], 0);
    Pose3D pose = pose1 * pose2 * pose3;
    return std::array<double, 3>{pose.translation().x(), pose.translation().z(),
        pose.eulerAngles().y()};
}

// 规范化角度到 [-π, π]
double BodyInterpolator::normalizeAngle(double angle)
{
    while (angle > M_PI) angle -= 2 * M_PI;
    while (angle < -M_PI) angle += 2 * M_PI;
    return angle;
}


// ==================== SE(2)插值器 ====================
// SE(2)逆变换
SE2Pose SE2Pose::inverse() const
{
    SE2Pose inv;
    inv.theta = -theta;
    double cos_theta = cos(theta);
    double sin_theta = sin(theta);
    inv.x = -x * cos_theta - y * sin_theta;
    inv.y = x * sin_theta - y * cos_theta;
    return inv;
}

// 复合变换
SE2Pose SE2Pose::operator*(const SE2Pose& other) const
{
    SE2Pose result;
    result.theta = theta + other.theta;
    result.x = x + other.x * cos(theta) - other.y * sin(theta);
    result.y = y + other.x * sin(theta) + other.y * cos(theta);
    return result;
}

// 规范化角度到 [-π, π)
void SE2Pose::normalizeAngle()
{
    while (theta > M_PI) theta -= 2 * M_PI;
    while (theta < -M_PI) theta += 2 * M_PI;
}

// 添加位姿到历史
void SE2Interpolator::addPose(const SE2Pose& pose)
{
    // 按时间顺序插入
    auto it = std::lower_bound(history_.begin(), history_.end(), pose,
        [](const SE2Pose& a, const SE2Pose& b) {
            return a.ts_ns < b.ts_ns;
        });
    history_.insert(it, pose);
    // 保持历史大小
    if (history_.size() > max_history_)
        history_.pop_front();
}

// SE(2)刚体插值
std::optional<SE2Pose> SE2Interpolator::rigidInterpolate(uint64_t ts_ns) const
{ 
    static bool less_warn = false;
    if (history_.size() < 2)
    {
        if(!less_warn)
        {
            less_warn = true;
            g_logger->warn("Chassis records history size({}) is less than 2!",
                history_.size());
        }
        if(history_.empty()) return std::nullopt;
        return history_.back();
    }
    else if(less_warn)
    {
        less_warn = false;
        g_logger->warn("Chassis records history size({}) restored!", history_.size());
    }

    // 查找边界点
    auto it_upper = std::lower_bound(
        history_.begin(), history_.end(), ts_ns,
        [](const SE2Pose& pose, uint64_t ts) {
            return pose.ts_ns < ts;
        });

    // 边界情况处理
    if (it_upper == history_.begin())
        return *it_upper;
    if (it_upper == history_.end())
        return predictExtrapolate(ts_ns);

    const SE2Pose& pose1 = *(it_upper - 1);
    const SE2Pose& pose2 = *it_upper;

    // 计算插值比例
    double alpha = static_cast<double>(ts_ns - pose1.ts_ns) /
                    static_cast<double>(pose2.ts_ns - pose1.ts_ns);

    // auto start1 = std::chrono::high_resolution_clock::now();
    auto mid1 = interpolateRigid(pose1, pose2, alpha);
    // auto end1 = std::chrono::high_resolution_clock::now();
    // auto duration1 = std::chrono::duration_cast<std::chrono::microseconds>(end1 - start1);
    // printf("最新点: (%.3f, %.3f, %.3f) @ %lu\n",
    //     history_.back().x, history_.back().y, history_.back().theta, history_.back().ts_ns);
    // printf("pose1: (%.3f, %.3f, %.3f) @ %lu\n",
    //     pose1.x, pose1.y, pose1.theta, pose1.ts_ns);
    // printf("pose2: (%.3f, %.3f, %.3f) @ %lu\n",
    //     pose2.x, pose2.y, pose2.theta, pose2.ts_ns);
    // printf("刚体插值方法: 耗时 %ld μs\n", duration1.count());
    // if (mid1)
    //     printf("刚体插值中点: (%.3f, %.3f, %.3f) @ %lu\n",
    //            mid1->x, mid1->y, mid1->theta, mid1->ts_ns);

    // 方法2：使用简化插值
    // auto start2 = std::chrono::high_resolution_clock::now();
    // auto mid2 = interpolateSimple(pose1, pose2, alpha);
    // auto end2 = std::chrono::high_resolution_clock::now();
    // auto duration2 = std::chrono::duration_cast<std::chrono::microseconds>(end2 - start2);    
    // printf("简化插值方法: 耗时 %ld μs\n", duration2.count());
    // if (mid2)
    //     printf("简化插值中点: (%.3f, %.3f, %.3f) @ %lu\n",
    //            mid2->x, mid2->y, mid2->theta, mid2->ts_ns);

    return mid1;
}

// 预测功能
std::optional<SE2Pose> SE2Interpolator::predictExtrapolate(uint64_t future_ts_ns) const
{
    if (history_.size() < 2)
        return history_.empty() ? std::nullopt : std::optional<SE2Pose>(history_.back());

    // 获取最近的两个位姿
    const auto& pose_n2 = history_[history_.size() - 2];
    const auto& pose_n1 = history_[history_.size() - 1];

    // 计算时间差和位姿变化率
    double dt_hist = static_cast<double>(pose_n1.ts_ns - pose_n2.ts_ns);
    double dt_future = static_cast<double>(future_ts_ns - pose_n1.ts_ns);

    // 限制预测时间（例如不超过120ms）
    if (dt_future > 120e6)
    {
        g_logger->error("Prediction of chassis pose: predict time is newer "
            "than history records {:.1f} ms!", dt_future / 1e6);
        return std::nullopt;  // 100ms
    }

    // 线性外推（可根据需要改为二阶外推）
    double alpha = dt_future / dt_hist;
    // 使用简化插值方法进行外推
    return interpolateSimple(pose_n2, pose_n1, 1.0 + alpha);
}

// 获取历史大小
size_t SE2Interpolator::size() const
{
    return history_.size();
}

// 获取最新位姿
std::optional<SE2Pose> SE2Interpolator::getLatestPose() const
{
    if (history_.empty()) return std::nullopt;    
    return history_.back();  // 返回最新位姿
}

// 清理旧数据
void SE2Interpolator::cleanup(uint64_t current_time_ns, uint64_t keep_duration_ns)
{ 
    uint64_t cutoff_time = current_time_ns - keep_duration_ns;
    while (!history_.empty() && history_.front().ts_ns < cutoff_time)
        history_.pop_front();
}

// SE(2)刚体插值核心算法
std::optional<SE2Pose> SE2Interpolator::interpolateRigid(const SE2Pose& pose1, 
    const SE2Pose& pose2, double alpha)
{ 
    if (alpha < 0.0 || alpha > 1.0)
    {
        g_logger->error("Chassis pose: invalid alpha value={}!", alpha);
        return std::nullopt;
    }

    // 1. 计算相对变换：pose1^{-1} * pose2
    SE2Pose relative = pose1.inverse() * pose2;
    relative.normalizeAngle();

    // 2. 在切空间进行插值
    SE2Pose interpolated_relative = interpolateInTangentSpace(relative, alpha);

    // 3. 应用插值后的相对变换：pose1 * interpolated_relative
    SE2Pose result = pose1 * interpolated_relative;
    result.normalizeAngle();
    result.ts_ns = static_cast<uint64_t>(
        pose1.ts_ns + alpha * (pose2.ts_ns - pose1.ts_ns)
    );

    return result;
}

// 在切空间进行线性插值
SE2Pose SE2Interpolator::interpolateInTangentSpace(const SE2Pose& transform, double alpha)
{
    double omega = transform.theta;
    // 处理小角度情况
    if (fabs(omega) < 1e-8)
    {
        // 无旋转时，直接线性插值平移
        return SE2Pose(transform.x * alpha, 
                        transform.y * alpha, 
                        0.0, 
                        0);
    }

    // 使用精确的SE(2)插值公式
    // 计算J矩阵及其逆
    double sin_omega = sin(omega);
    double cos_omega = cos(omega);

    // J矩阵（将切空间速度映射到位移）
    double J00 = sin_omega / omega;
    double J01 = (1 - cos_omega) / omega;
    double J10 = -J01;
    double J11 = J00;

    // J的逆矩阵
    double det = J00 * J11 - J01 * J10;
    if (fabs(det) < 1e-12)
    {
        // 避免数值问题，回退到简单插值
        return SE2Pose(transform.x * alpha, 
                        transform.y * alpha, 
                        omega * alpha, 
                        0);
    }

    double invJ00 = J11 / det;
    double invJ01 = -J01 / det;
    double invJ10 = -J10 / det;
    double invJ11 = J00 / det;

    // 计算切空间速度v = J_inv * t
    double vx = invJ00 * transform.x + invJ01 * transform.y;
    double vy = invJ10 * transform.x + invJ11 * transform.y;

    // 插值
    double omega_interp = omega * alpha;
    double vx_interp = vx * alpha;
    double vy_interp = vy * alpha;

    // 使用J矩阵映射回位移
    double sin_omega_interp = sin(omega_interp);
    double cos_omega_interp = cos(omega_interp);

    if (fabs(omega_interp) < 1e-8)
    {
        // 小角度近似
        return SE2Pose(vx_interp, vy_interp, omega_interp, 0);
    }

    double J00_interp = sin_omega_interp / omega_interp;
    double J01_interp = (1 - cos_omega_interp) / omega_interp;
    double J10_interp = -J01_interp;
    double J11_interp = J00_interp;

    double x_interp = J00_interp * vx_interp + J01_interp * vy_interp;
    double y_interp = J10_interp * vx_interp + J11_interp * vy_interp;

    return SE2Pose(x_interp, y_interp, omega_interp, 0);
}

// 使用基于旋转和平移分离的简化方法
std::optional<SE2Pose> SE2Interpolator::interpolateSimple(const SE2Pose& pose1,
    const SE2Pose& pose2, double alpha)
{
    if (alpha < 0.0)
    {
        g_logger->error("Unable to interpolate chassis pose: alpha = {}(< 0.0)!");
        return std::nullopt;
    }

    // 角度差值（处理角度环绕）
    double delta_theta = pose2.theta - pose1.theta;
    if (delta_theta > M_PI) delta_theta -= 2 * M_PI;
    if (delta_theta < -M_PI) delta_theta += 2 * M_PI;

    // 插值角度
    double theta_interp = pose1.theta + alpha * delta_theta;

    // 在pose1的局部坐标系中计算位移
    double cos_theta1 = cos(pose1.theta);
    double sin_theta1 = sin(pose1.theta);
    double dx = pose2.x - pose1.x;
    double dy = pose2.y - pose1.y;

    // 旋转到pose1的局部坐标系
    double local_dx = dx * cos_theta1 + dy * sin_theta1;
    double local_dy = -dx * sin_theta1 + dy * cos_theta1;

    // 插值局部位移
    double local_dx_interp = alpha * local_dx;
    double local_dy_interp = alpha * local_dy;

    // 旋转回世界坐标系
    double cos_theta_interp = cos(theta_interp);
    double sin_theta_interp = sin(theta_interp);

    SE2Pose result;
    result.x = pose1.x + local_dx_interp * cos_theta_interp 
                - local_dy_interp * sin_theta_interp;
    result.y = pose1.y + local_dx_interp * sin_theta_interp 
                + local_dy_interp * cos_theta_interp;
    result.theta = theta_interp;
    result.normalizeAngle();
    result.ts_ns = static_cast<uint64_t>(
        pose1.ts_ns + alpha * (pose2.ts_ns - pose1.ts_ns)
    );

    return result;
}
