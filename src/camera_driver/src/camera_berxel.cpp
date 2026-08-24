#include "camera_berxel.h"
#include "pose_estimator.h"
#include <memory>
#include <chrono>

using namespace std::chrono_literals;

CameraBerxel::CameraBerxel(std::shared_ptr<rclcpp::Node> parent, 
    const CameraConfig& config, PoseEstimator *pose_estimator)
    : param_ready_(false), parent_(parent), config_(config),
    pose_estimator_(pose_estimator), last_sub_count_(0)
{
    // 设置流类型
    stream_type_ = BERXEL_HAWK_COLOR_STREAM | BERXEL_HAWK_DEPTH_STREAM;

    if(config_.pointcloud_type > 0)
    {
        cloud_op_.ordered   = false;
        cloud_op_.max_depth = 8.0f;
        cloud_op_.min_depth = 0.3f;
        cloud_op_.depth_scale = 1.0f / 1000.0f;
    }
}

CameraBerxel::~CameraBerxel()
{
    stop();
}

bool CameraBerxel::initialize()
{
    g_logger->info("[{}] Initializing...", config_.name);
    if(!context_) context_ = BerxelHawkContext::getBerxelContext();
    if(!context_) 
    {
        g_logger->error("[{}] Get berxel context failed!", config_.name);
        return false;
    }
    context_->setDeviceStateCallback(onDeviceStatusChange, this);
    return start();
}

bool CameraBerxel::start()
{
    g_logger->info("[{}] Starting device...", config_.name);
	BerxelHawkDeviceInfo* p_device_info = nullptr;
	uint32_t device_count = 0;
	context_->getDeviceList(&p_device_info, &device_count);
	if((device_count <= 0) || (nullptr == p_device_info))
	{
        g_logger->error("[{}] Get no connected berxel device!", config_.name);
		return false;
	}
	memset(&device_info_, 0x00, sizeof(device_info_));
    for (int i = 0; i < device_count; i++)
        if (strcmp(p_device_info[i].deviceAddress, config_.id.c_str()) == 0)
        {
            device_info_ = p_device_info[i];
            break;
        }
	if (device_info_.vendorId == 0x00)
	{
        g_logger->error("[{}] Get current device info error!", config_.name);
		return false;
	}

    g_logger->info("[{}] Will open device: {}.", config_.name, config_.id);
	device_ = context_->openDevice(device_info_);
	if(!device_)
	{
        g_logger->error("[{}] Open berxel device failed!", config_.name);
		return false;
	}

    last_sync_time_ = parent_->now();
	device_->setSystemClock();
	device_->setRegistrationEnable(true);
	device_->setTemperatureCompensationEnable(false);
    device_->enableColorAutoExposure();
    // device_->setColorExposureGain(config_.expos_us, config_.expos_gain);
    // g_logger->info("[{}] Set exposure time to {} us, gain: {}.", 
    //     config_.name, config_.expos_us, config_.expos_gain);

	createPublishers();
	getParameter();

    if(config_.width == 640)
        device_->setStreamFlagMode(BERXEL_HAWK_MIX_STREAM_FLAG_MODE);
    else
        device_->setStreamFlagMode(BERXEL_HAWK_MIX_HD_STREAM_FLAG_MODE);
    BerxelHawkStreamFrameMode colorFrameMode, depthFrameMode;
    device_->getCurrentFrameMode(BERXEL_HAWK_COLOR_STREAM, &colorFrameMode);
    device_->getCurrentFrameMode(BERXEL_HAWK_DEPTH_STREAM, &depthFrameMode);
    if(config_.width == 640)
    {
        colorFrameMode.resolutionX = config_.width;
        colorFrameMode.resolutionY = config_.height;
    }
    device_->setFrameMode(BERXEL_HAWK_COLOR_STREAM, &colorFrameMode);
    device_->setFrameMode(BERXEL_HAWK_DEPTH_STREAM, &depthFrameMode);
    g_logger->info("[{}] Set resolution to {}x{}",
        config_.name, config_.width, config_.height);

    int ret = device_->startStreams(stream_type_, onNewFrameCallback, this);
	if(ret != 0)
	{
        g_logger->error("[{}] Open berxel stream failed!", config_.name);
		return false;
	}

    g_logger->info("[{}] Starting berxel stream...", config_.name);
	return true;
}

void CameraBerxel::stop()
{
    if (!device_) return;
    try 
    {
        if(device_)
        {
            device_->stopStreams(stream_type_);
            g_logger->info("[{}] Stopping berxel stream...", config_.name);
        }
        if(context_)
        {
            context_->closeDevice(device_);
            device_ = nullptr;
            g_logger->info("[{}] Closing berxel device...", config_.name);
        }
    } 
    catch (const std::exception& e)
    {
        g_logger->error("[{}] Stop error: {}!", config_.name, e.what());
    }
}

void CameraBerxel::destroy()
{
    if(context_)
    {
        g_logger->info("Destroying berxel context...");
        BerxelHawkContext::destroyBerxelContext(context_);
        context_ = nullptr;
    }
}    

void CameraBerxel::processNewFrame(BerxelHawkStreamType stream_type, 
    BerxelHawkFrame* p_frame)
{
    try
    {
        switch (stream_type)
        {
            case BERXEL_HAWK_COLOR_STREAM:
                processColorFrame(p_frame);
                break;
            case BERXEL_HAWK_DEPTH_STREAM:
                processDepthFrame(p_frame);
                break;
            case berxel::BERXEL_HAWK_IR_STREAM:
            case berxel::BERXEL_HAWK_LIGHT_IR_STREAM:
                processIrFrame(p_frame);
                break;
            default:
                break;
        }
    }
    catch (const std::exception& e)
    {
        g_logger->error("[{}]: Process new frame error: {}!", config_.name, e.what());
    }
    // 释放帧
    if (device_) device_->releaseFrame(p_frame);
}

void CameraBerxel::processColorFrame(BerxelHawkFrame* p_frame)
{
    // 创建彩色图像消息
    sensor_msgs::msg::Image msg;
    uint64_t ts = p_frame->getTimeStamp();
    int width   = p_frame->getWidth();
    int height  = p_frame->getHeight();

    msg.header.stamp = rclcpp::Time(ts * 1000);
    msg.header.frame_id = config_.frame_id;
    msg.height = height;
    msg.width  = width;
    msg.encoding = sensor_msgs::image_encodings::BGR8;
    msg.is_bigendian = false;
    msg.step = width * 3;

    // size_t last_digit =  static_cast<uint64_t>(msg.header.stamp.nanosec / 1e6) % 10;
    // if (last_digit == 1 || last_digit == 4 || last_digit == 8)
    // {
    //     rclcpp::Time current_time = parent_->now(); 
    //     int64_t current_ns = current_time.nanoseconds();
    //     int64_t frame_ns = msg.header.stamp.nanosec;
    //     int64_t time_diff_ns = current_ns - frame_ns;   
    //     double time_diff_ms = static_cast<double>(time_diff_ns) / 1e6;
    //     g_logger->debug("[{}] Color {}, diff {:.1f} ms.", 
    //         config_.name, rosTimeToStr(msg.header.stamp), time_diff_ms);
    // }

    // 复制数据
    size_t data_size = p_frame->getDataSize();
    std::vector<uint8_t> rgb(width * height * 3);
    memcpy(rgb.data(), p_frame->getData(), data_size);
    bgr2rgb(rgb);
    msg.data.resize(data_size);
    memcpy(msg.data.data(), rgb.data(), rgb.size());

    // 发布图像
    if(color_pub_) color_pub_->publish(std::move(msg));

    if(config_.pointcloud_type == 2)
    {
        {
            std::lock_guard<std::mutex> lock(rgb_mutex_);   
            rgbs_.push_back(rgb);
            rgbs_ts_.push_back(ts);
            if (rgbs_.size() > 3) 
            {
                rgbs_.pop_front();
                rgbs_ts_.pop_front();
            }
        }
        tryPublishRgbCloud();
    }
}

void CameraBerxel::processDepthFrame(BerxelHawkFrame* p_frame)
{
    // 创建深度图像消息
    sensor_msgs::msg::Image msg;
    uint64_t ts = p_frame->getTimeStamp();
    int width   = p_frame->getWidth();
    int height  = p_frame->getHeight();

    msg.header.stamp = rclcpp::Time(ts * 1000);
    msg.header.frame_id = config_.frame_id;
    msg.height = height;
    msg.width  = width;
    msg.encoding = sensor_msgs::image_encodings::TYPE_16UC1;
    msg.is_bigendian = false;
    msg.step = width * 2;

    // size_t last_digit =  static_cast<uint64_t>(msg.header.stamp.nanosec / 1e6) % 10;
    // if (last_digit == 1 || last_digit == 4 || last_digit == 8)
    // {
    //     rclcpp::Time current_time = parent_->now(); 
    //     int64_t current_ns = current_time.nanoseconds();
    //     int64_t frame_ns = msg.header.stamp.nanosec;
    //     int64_t time_diff_ns = current_ns - frame_ns;  
    //     double time_diff_ms = static_cast<double>(time_diff_ns) / 1e6;
    //     g_logger->debug("[{}] Depth {}, diff {:.1f} ms.",
    //         config_.name, rosTimeToStr(msg.header.stamp), time_diff_ms);
    // }

    // 分配内存并转换数据
    size_t data_size = p_frame->getDataSize();
    std::vector<uint16_t> depth(width * height);
    convertDepthToCv16UC((uint16_t*)p_frame->getData(),
        (uint16_t*)depth.data(), width, height, p_frame->getPixelType());
    msg.data.resize(data_size);
    memcpy(msg.data.data(), depth.data(), depth.size() * sizeof(uint16_t));

    // 发布深度图像
    if(depth_pub_)
    {
        depth_pub_->publish(std::move(msg));
        uint64_t timestamp_ns =
            static_cast<uint64_t>(msg.header.stamp.sec) * 1'000'000'000ULL +
            msg.header.stamp.nanosec;
        pose_estimator_->requestCameraPose(timestamp_ns, config_.name);
    }

    if(config_.pointcloud_type > 0 && param_ready_)
    {
        if(config_.pointcloud_type == 1)
        {
            sensor_msgs::msg::PointCloud2 cloud_msg;
            cloud_msg.header.stamp = rclcpp::Time(ts * 1000);
            cloud_msg.header.frame_id = config_.frame_id;
            cloud_op_.enable_color = false;

            convertDepthToPointCloud(depth.data(), nullptr, width, height,
                rgb_intrinsics_, cloud_op_, cloud_msg);
            cloud_pub_->publish(std::move(cloud_msg));
        }
        else if(config_.pointcloud_type == 2)
        { 
            {
                std::lock_guard<std::mutex> lock(dep_mutex_);   
                deps_.push_back(depth);
                deps_ts_.push_back(ts);
                if (deps_.size() > 3) 
                {
                    deps_.pop_front();
                    deps_ts_.pop_front();
                }
            }
            tryPublishRgbCloud();
        }
    }
}

void CameraBerxel::processIrFrame(BerxelHawkFrame* p_frame)
{
    // 如果需要红外图像，可以在这里实现
}

void CameraBerxel::convertDepthToCv16UC(uint16_t* p_depth, uint16_t* data, 
    int width, int height, BerxelHawkPixelType pixel_type)
{
    // 根据像素类型进行转换
    if (pixel_type == BERXEL_HAWK_PIXEL_TYPE_DEP_16BIT_13I_3D)
    {
        // 13位深度，右移3位
        for (int i = 0; i < width * height; ++i)
            data[i] = p_depth[i] >> 3;
    }
    else
    {
        // 12位深度，右移4位
        for (int i = 0; i < width * height; ++i)
            data[i] = p_depth[i] >> 4;
    }
}

// 静态回调函数
void __attribute__((__stdcall__)) CameraBerxel::onNewFrameCallback(
    BerxelHawkStreamType stream_type, BerxelHawkFrame* p_frame, void* user_data)
{
    CameraBerxel* berxel = static_cast<CameraBerxel*>(user_data);
	if (berxel)
        berxel->processNewFrame(stream_type, p_frame);
}

void __attribute__((__stdcall__)) CameraBerxel::onDeviceStatusChange(
    const char* address, const char* serial_number,
    berxel::BerxelHawkDeviceStatus device_state, void* user_data)
{
    CameraBerxel* berxel = static_cast<CameraBerxel*>(user_data);
    if(berxel)
        berxel->processDeviceStatusChange(address, serial_number, device_state);
}

int32_t CameraBerxel::processDeviceStatusChange(const char* address,
    const char* serial_number, berxel::BerxelHawkDeviceStatus device_state)
{
	switch(device_state)
	{
    case berxel::BERXEL_HAWK_DEVICE_DISCONNECT:
		{
            g_logger->warn("Device {}({}) disconnected!", address, serial_number);
			if (strcmp(device_info_.deviceAddress, address) == 0)
				stop();
		}
		break;
	case berxel::BERXEL_HAWK_DEVICE_CONNECT:
		{
            g_logger->warn("Device {}({}) connected!", address, serial_number);
			if (strcmp(device_info_.deviceAddress, address) == 0)
				start();
		}
		break;
	default:
		break;
	}
	return 0;
}

bool CameraBerxel::getParameter()
{
    uint32_t factor = 1;
    if (config_.width == 640) factor = 2;
    if(config_.use_param_from_file)
    {
        g_logger->info("[{}] Using parameters from file.", config_.name);
        param_msg_.rgb_fx = config_.param.rgb_intrinsics.fx / factor;
        param_msg_.rgb_fy = config_.param.rgb_intrinsics.fy / factor;
        param_msg_.rgb_cx = config_.param.rgb_intrinsics.cx / factor;
        param_msg_.rgb_cy = config_.param.rgb_intrinsics.cy / factor;
        memcpy(param_msg_.rgb_distortion_coeffs.data(), config_.param.rgb_intrinsics.coeffs, sizeof(float)*5);

        param_msg_.dep_fx = config_.param.dep_intrinsics.fx / factor;
        param_msg_.dep_fy = config_.param.dep_intrinsics.fy / factor;
        param_msg_.dep_cx = config_.param.dep_intrinsics.cx / factor;
        param_msg_.dep_cy = config_.param.dep_intrinsics.cy / factor;
        memcpy(param_msg_.dep_distortion_coeffs.data(), config_.param.dep_intrinsics.coeffs, sizeof(float)*5);

        memcpy(param_msg_.rotation.data(), config_.param.extrinsics.rotation, sizeof(float)*9);
        memcpy(param_msg_.translation.data(), config_.param.extrinsics.translation, sizeof(float)*3);
    }
    else
    {
        g_logger->info("[{}] Using online parameters.", config_.name);
        BerxelHawkDeviceIntrinsicParams params;
        memset((uint8_t*)&params, 0x00, sizeof(BerxelHawkDeviceIntrinsicParams));
        if(device_->getDeviceIntriscParams(&params) != 0)
        {
            g_logger->error("[{}]: Failed to get intrinsic parameters online!", 
                config_.name);
            return false;
        }
        BerxelHawkCameraIntrinsic color_params, depth_params;
        memcpy((uint8_t*)&color_params, (uint8_t*)&params.colorIntrinsicParams, sizeof(BerxelHawkCameraIntrinsic));
        memcpy((uint8_t*)&depth_params, (uint8_t*)&params.liteIrIntrinsicParams, sizeof(BerxelHawkCameraIntrinsic));
        //旋转矩阵
        // std::copy(params.rotateIntrinsicParams, params.rotateIntrinsicParams + 9, msg.rotation.data());
        memcpy(param_msg_.rotation.data(), params.rotateIntrinsicParams, sizeof(float)*9);
        // 平移矩阵
        // std::copy(params.translationIntrinsicParams, params.translationIntrinsicParams + 3, msg.translation.data());
        memcpy(param_msg_.translation.data(), params.translationIntrinsicParams, sizeof(float)*3);

        param_msg_.rgb_fx = color_params.fxParam / factor;
        param_msg_.rgb_fy = color_params.fyParam / factor;
        param_msg_.rgb_cx = color_params.cxParam / factor;
        param_msg_.rgb_cy = color_params.cyParam / factor;
        param_msg_.rgb_distortion_coeffs[0] = color_params.k1Param;
        param_msg_.rgb_distortion_coeffs[1] = color_params.k2Param;
        param_msg_.rgb_distortion_coeffs[2] = color_params.p1Param;
        param_msg_.rgb_distortion_coeffs[3] = color_params.p2Param;
        param_msg_.rgb_distortion_coeffs[4] = color_params.k3Param;

        param_msg_.dep_fx = depth_params.fxParam / factor;
        param_msg_.dep_fy = depth_params.fyParam / factor;
        param_msg_.dep_cx = depth_params.cxParam / factor;
        param_msg_.dep_cy = depth_params.cyParam / factor;
        param_msg_.dep_distortion_coeffs[0] = depth_params.k1Param;
        param_msg_.dep_distortion_coeffs[1] = depth_params.k2Param;
        param_msg_.dep_distortion_coeffs[2] = depth_params.p1Param;
        param_msg_.dep_distortion_coeffs[3] = depth_params.p2Param;
        param_msg_.dep_distortion_coeffs[4] = depth_params.k3Param;
    }

    rgb_intrinsics_.fx = param_msg_.rgb_fx;
    rgb_intrinsics_.fy = param_msg_.rgb_fy;
    rgb_intrinsics_.cx = param_msg_.rgb_cx;
    rgb_intrinsics_.cy = param_msg_.rgb_cy;
    param_ready_ = true;

    if(param_pub_) param_pub_->publish(param_msg_);

    g_logger->info("[{}] Get and publish camera parameters succeed.", config_.name);
    g_logger->info("[{}] color intrinsics: fx: {:.1f}, fy: {:.1f}, "
        "cx: {:.1f}, cy: {:.1f}",
        config_.name, param_msg_.rgb_fx, param_msg_.rgb_fy,
        param_msg_.rgb_cx, param_msg_.rgb_cy);
    g_logger->info("[{}] color distortion: k1: {:.5f}, k2: {:.5f}, k3: {:.5f}, "
        "p1: {:.5f}, p2: {:.5f}",
        config_.name, param_msg_.rgb_distortion_coeffs[0],
        param_msg_.rgb_distortion_coeffs[1], param_msg_.rgb_distortion_coeffs[4],
        param_msg_.rgb_distortion_coeffs[2], param_msg_.rgb_distortion_coeffs[3]);
    g_logger->info("[{}] depth intrinsics: fx: {:.1f}, fy: {:.1f}, "
        "cx: {:.1f}, cy: {:.1f}",
        config_.name, param_msg_.dep_fx, param_msg_.dep_fy,
        param_msg_.dep_cx, param_msg_.dep_cy);
    g_logger->info("[{}] depth distortion: k1: {:.5f}, k2: {:.5f}, k3: {:.5f}, "
        "p1: {:.5f}, p2: {:.5f}",
        config_.name, param_msg_.dep_distortion_coeffs[0],
        param_msg_.dep_distortion_coeffs[1], param_msg_.dep_distortion_coeffs[4],
        param_msg_.dep_distortion_coeffs[2], param_msg_.dep_distortion_coeffs[3]);
    g_logger->info("[{}] translation: x: {:.3f}, y: {:.3f}, z: {:.3f}",
        config_.name, param_msg_.translation[0],
        param_msg_.translation[1], param_msg_.translation[2]);
    g_logger->info("[{}] rotation: [\n"
        "\t{:.3f}, {:.3f}, {:.3f}\n"
        "\t{:.3f}, {:.3f}, {:.3f}\n"
        "\t{:.3f}, {:.3f}, {:.3f}]\n",
        config_.name,
        param_msg_.rotation[0], param_msg_.rotation[1], param_msg_.rotation[2],
        param_msg_.rotation[3], param_msg_.rotation[4], param_msg_.rotation[5],
        param_msg_.rotation[6], param_msg_.rotation[7], param_msg_.rotation[8]);

    return true;
}

void CameraBerxel::tryPublishRgbCloud()
{ 
    std::lock_guard<std::mutex> lock_rgb(rgb_mutex_);  
    std::lock_guard<std::mutex> lock_dep(dep_mutex_);  
    if(rgbs_ts_.empty() || deps_ts_.empty()) return;

    sensor_msgs::msg::PointCloud2 cloud_msg;
    cloud_msg.header.frame_id = config_.frame_id;

    auto latest_rgb = rgbs_ts_.back();
    auto latest_dep = deps_ts_.back();

    if(isRgbDepPair(deps_.back(), rgbs_.back(), latest_dep, latest_rgb, 
        cloud_msg))
    {
        rgbs_.clear(); rgbs_ts_.clear();
        deps_.clear(); deps_ts_.clear();
        return;
    }

    if(latest_rgb > latest_dep)
    {
        for (int i = rgbs_ts_.size() - 2; i >= 0; --i)
            if(isRgbDepPair(deps_.back(), rgbs_[i], latest_dep, rgbs_ts_[i], 
                cloud_msg))
            {
                removeFrames(true, i + 1);
                return;
            }
    }
    else
    {
        for (int i = deps_ts_.size() - 2; i >= 0; --i)
            if(isRgbDepPair(deps_[i], rgbs_.back(), deps_ts_[i], latest_rgb, 
                cloud_msg))
            {
                removeFrames(false, i + 1);
                return;
            }
    }
}

bool CameraBerxel::isRgbDepPair(
    const std::vector<uint16_t> &dep, const std::vector<uint8_t> &rgb,
    const uint64_t dep_ts, const uint64_t rgb_ts, 
    sensor_msgs::msg::PointCloud2 &cloud_msg)
{
    auto diff = dep_ts > rgb_ts ? (dep_ts - rgb_ts) : (rgb_ts - dep_ts);
    if(diff < 1000)
    {
        cloud_op_.enable_color = true;
        cloud_msg.header.stamp = rclcpp::Time(dep_ts * 1000);
        convertDepthToPointCloud(dep.data(), rgb.data(), 
            config_.width, config_.height, 
            rgb_intrinsics_, cloud_op_, cloud_msg);
        if(cloud_pub_) cloud_pub_->publish(std::move(cloud_msg));
        return true;
    }
    return false;
}

void CameraBerxel::removeFrames(bool rgb, size_t num)
{
    if(rgb)
    {
        // 删除彩色图中时间戳小于等于已发布彩色图时间戳的所有帧
        while (!rgbs_.empty() && !rgbs_ts_.empty() && num--)
        {
            rgbs_.pop_front();
            rgbs_ts_.pop_front();
        }
        deps_.clear();
        deps_ts_.clear();
    }
    else
    {
        // 删除深度图中时间戳小于等于已发布深度图时间戳的所有帧
        while (!deps_.empty() && !deps_ts_.empty() && num--)
        {
            deps_.pop_front();
            deps_ts_.pop_front();
        }
        rgbs_.clear();
        rgbs_ts_.clear();
    }
}

void CameraBerxel::createPublishers()
{
    rclcpp::QoS qos = rclcpp::SensorDataQoS();
    qos.keep_last(10);  // 保持最后10条消息
    if (config_.qos == "best_effort")
    {  
        qos.best_effort();
        g_logger->info("[{}] Using best_effort QoS.", config_.name);
    } 
    else
    {
        qos.reliable();
        g_logger->info("[{}] Using reliable QoS.", config_.name);
    }
    // 创建彩色图像发布器
    if(color_pub_ == nullptr)
    {
        color_pub_ = parent_->create_publisher<sensor_msgs::msg::Image>(
            config_.color_topic, qos);
        g_logger->info("[{}] Create color topic: {}.",
            config_.name, config_.color_topic);
    }

    // 创建深度图像发布器
    if(depth_pub_ == nullptr)
    {
        depth_pub_ = parent_->create_publisher<sensor_msgs::msg::Image>(
            config_.depth_topic, qos);
        g_logger->info("[{}] Create depth topic: {}.",
            config_.name, config_.depth_topic);
    }

    // 创建相机参数发布器
    auto param_qos = rclcpp::QoS(rclcpp::KeepLast(1))
        .transient_local().reliable();
    param_pub_ = parent_->create_publisher<camera_msgs::msg::CameraParameters>(
        config_.param_topic, param_qos);

    // 定期发布参数
    param_timer_ = parent_->create_wall_timer(
        std::chrono::seconds(2),  // 每2秒发布一次
        [this]()
        {
            size_t sub_count = param_pub_->get_subscription_count();
            if (sub_count > 0 && sub_count != last_sub_count_ && param_ready_)
            {
                g_logger->info("[{}] New subscriber! Re-publishing parameters...",
                    config_.name);
                // 简单直接：重新发布
                param_pub_->publish(param_msg_);
            }
            last_sub_count_ = sub_count;

            rclcpp::Time current_time = parent_->now();
            if(current_time.seconds() - last_sync_time_.seconds() > max_sync_gap_s_)
            {
                device_->setSystemClock();
                last_sync_time_ = current_time;
            }
        });

    if(config_.pointcloud_type > 0 && cloud_pub_ == nullptr)
    {
        rclcpp::QoS cloud_qos = rclcpp::SensorDataQoS();
        cloud_qos.keep_last(10);  
        if (config_.qos == "best_effort") cloud_qos.best_effort();
        else cloud_qos.reliable();
        if(config_.pointcloud_type == 1)
            cloud_pub_ = parent_->create_publisher<sensor_msgs::msg::PointCloud2>(
                config_.cloud_topic, cloud_qos);
        else if(config_.pointcloud_type == 2)
            cloud_pub_ = parent_->create_publisher<sensor_msgs::msg::PointCloud2>(
                config_.rgb_cloud_topic, cloud_qos);

        g_logger->info("[{}] Created {} pointcloud publisher.", config_.name,
                       config_.pointcloud_type == 1 ? "normal" : "rgb");
    }

    g_logger->info("[{}] Created all publishers.", config_.name);
}
