#include "camera_driver.h"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <iostream>
#include <fstream>
#include <cstdlib>
#include <nlohmann/json.hpp>
#include "camera_berxel.h"
#include "camera_angstrong.h"

using namespace std::chrono_literals;
using json = nlohmann::json;

namespace {

std::string camera_config_dir()
{
    if (const char* configured = std::getenv("HC_X1_CAMERA_CONFIG_DIR"))
    {
        std::string path(configured);
        if (!path.empty()) return path.back() == '/' ? path : path + "/";
    }
    const char* home = std::getenv("HOME");
    return std::string(home == nullptr ? "." : home) +
        "/Programs/AGIInterface/Config/";
}

}  // namespace

CameraDriver::CameraDriver()
    : Node("camera_driver"), running_(false)
{
    version_ = "2.0.0_260323";
    camera_names_ = {"eye", "chest", "Aleft", "Aright", 
        "Bfront", "Bback", "Bleft", "Bright"};
    g_logger->info("Initializing camera driver node...");
}

CameraDriver::~CameraDriver()
{
    stop();
}

bool CameraDriver::initialize()
{
    if (running_.load()) return true;

    // 加载参数
    loadBasicInfo();
    loadCameraConfigs();
    if(!loadCameraParameters()) return false;

    if(debug_mode_)
    {
        auto& cl = CustomLogger::instance();
        cl.setEnvironment(debug_mode_);
        g_logger->warn("Debug mode enabled.");
    }
    else
        g_logger->warn("Debug mode disabled.");

    // 位姿估计器
    pose_estimator_ = std::make_unique<PoseEstimator>(shared_from_this());
    pose_estimator_->setMode(use_body_, use_chassis_);
    if(use_body_ || use_chassis_)
    {
        if(!loadCalibrations()) return false;
        if(!pose_estimator_->setAxisTransform(calib_map_, fixed_head_,
            calf_length_, thigh_length_)) return false;
        if(!pose_estimator_->setRedis(body_info_, chassis_info_))
            return false;
        pose_estimator_->setTimestampOffset(body_offset_ms_, chassis_offset_ms_);
    }
    pose_estimator_->start();

    // 相机管理器
    std::vector<CameraConfig> as_configs;
    for (auto& [name, config] : configs_)
    {
        if(config.type == "berxel")
        {
            auto berxel = std::make_unique<CameraBerxel>(shared_from_this(), config,
                pose_estimator_.get());
            if(berxel->initialize())
            {
                berxels_[name] = std::move(berxel);
                g_logger->info("[{}] Initialize berxel camera succeed.", name);
            }
            else
                g_logger->error("[{}] Initialize berxel camera failed.", name);
        }
        else
            as_configs.push_back(config);
    }

    if(!as_configs.empty())
    {
        if (!std::ifstream(angstrong_file_).good())
        {
            g_logger->error("Angstrong config file '{}' not found!", angstrong_file_);
            return false;
        }
        angstrongs_ = std::make_unique<CameraAngstrong>(shared_from_this(),
            as_configs, angstrong_file_, pose_estimator_.get());
        if (angstrongs_->initialize())
            g_logger->info("Initialize angstrong cameras succeed.");
        else
            g_logger->error("Initialize angstrong cameras failed!");
    }

    running_.store(true);
    g_logger->info("Initialized pose estimator and all cameras.");
    return true;
}

void CameraDriver::loadBasicInfo()
{
    std::string config_file(camera_config_dir() + "basic_info.yaml");
    g_logger->info("Loading basic info from '{}'...", config_file);
    if (!std::ifstream(config_file).good()) 
    {
        g_logger->error("Config file '{}' not found!", config_file);
        setupDefaultBasicInfo();
        return;
    }

    try 
    {
        YAML::Node config = YAML::LoadFile(config_file);
        if(!config["CameraDriver"] || !config["PoseEstimator"] || !config["RedisInfo"])
        {
            g_logger->error("No 'CameraDriver', 'PoseEstimator' or 'RedisInfo' "
                "section in '{}'!", config_file);
            setupDefaultBasicInfo();
            return;
        }

        YAML::Node node = config["CameraDriver"];
        debug_mode_ = node["debug_mode"].as<bool>();

        node = config["RedisInfo"];
        body_info_.host    = node["body_addr"].as<std::string>();
        body_info_.pw      = node["body_pw"].as<std::string>();
        body_info_.port    = node["body_port"].as<int>();
        chassis_info_.host = node["chassis_addr"].as<std::string>();
        chassis_info_.pw   = node["chassis_pw"].as<std::string>();
        chassis_info_.port = node["chassis_port"].as<int>();

        node = config["PoseEstimator"];
        use_body_          = node["use_body"].as<bool>();
        use_chassis_       = node["use_chassis"].as<bool>();
        body_offset_ms_    = node["body_offset"].as<int64_t>();
        chassis_offset_ms_ = node["chassis_offset"].as<int64_t>();

        g_logger->info("Loaded basic info succeed from '{}'!", config_file);      
    } 
    catch (const YAML::Exception& e) 
    {
        g_logger->error("Load file '{}' error: {}!", config_file, e.what());
        setupDefaultBasicInfo();
    } 
    catch (const std::exception& e) 
    {
        g_logger->error("Faild to load file '{}': {}!", config_file, e.what());
        setupDefaultBasicInfo();
    }
}

void CameraDriver::loadCameraConfigs()
{
    std::string config_path(camera_config_dir());
    std::string config_file = config_path + "camera_config.yaml";
    g_logger->info("Loading camera config from '{}'...", config_file);
    if (!std::ifstream(config_file).good())
    {
        g_logger->error("Config file '{}' not found!", config_file);
        setupDefaultCameraConfigs();
        return;
    }
    angstrong_file_ = config_path + "hp60c.json";

    // 加载YAML配置文件
    try
    {
        YAML::Node config = YAML::LoadFile(config_file);
        if(!config["cameras"])
        {
            g_logger->error("No 'cameras' section in config file!");
            setupDefaultCameraConfigs();
            return;
        }
        for(auto &name : camera_names_)
        {
            if(!config["cameras"][name])
            {
                g_logger->error("[{}] Camera not found in config '{}', skipping!",
                    name, config_file);
                continue;
            }

            YAML::Node node = config["cameras"][name];
            if(getYamlValue(node, "enabled", false))
            {
                CameraConfig cfg; 
                cfg.name   = name;
                cfg.id     = node["id"].as<std::string>();
                cfg.type   = node["type"].as<std::string>();
                cfg.qos    = node["qos"].as<std::string>();
                cfg.width  = node["width"].as<int>();
                cfg.height = node["height"].as<int>();
                cfg.frame_id    = name;
                cfg.color_topic = "/cameras/" + name + "/color";
                cfg.depth_topic = "/cameras/" + name + "/depth";
                cfg.param_topic = "/cameras/" + name + "/param";
                cfg.cloud_topic = "/cameras/" + name + "/cloud";
                cfg.rgb_cloud_topic = "/cameras/" + name + "/rgb_cloud";
                cfg.chassis_depth_topic = cfg.depth_topic + "_chassis";
                cfg.chassis_param_topic = cfg.param_topic + "_chassis";
                cfg.pointcloud_type = node["pointcloud_type"].as<int>();
                cfg.expos_us   = node["expos_us"].as<uint32_t>();
                cfg.expos_gain = node["expos_gain"].as<uint32_t>();
                cfg.use_param_from_file = node["param_in_file"].as<bool>();
                cfg.send_chassis_depth  = node["send_chassis_depth"].as<bool>();
                g_logger->info("[{}] Point cloud type: {}, expos: {} us, gain: {}, "
                    "param_in_file: {}, send_chassis_depth: {}.",
                    name, cfg.pointcloud_type, cfg.expos_us, cfg.expos_gain,
                    cfg.use_param_from_file ? "true" : "false",
                    cfg.send_chassis_depth ? "true" : "false");

                configs_[name] = cfg;
            }
        }

        g_logger->info("Loaded camera configurations succeed from '{}'.", config_file);
        g_logger->info("Loaded total {} camera configurations.", configs_.size());
    } 
    catch (const YAML::Exception& e)
    {
        g_logger->error("Load file '{}' error: {}!", config_file, e.what());
        setupDefaultCameraConfigs();
    } 
    catch (const std::exception& e)
    {
        g_logger->error("Faild to load file '{}': {}!", config_file, e.what());
        setupDefaultCameraConfigs();
    }
}

bool CameraDriver::loadCameraParameters()
{
    std::string config_path(camera_config_dir());
    g_logger->info("Loading camera parameters from '{}'...", config_path);
    bool succeed = true;
    try 
    {
        for(auto& [name, config] : configs_)
        {
            if(config.use_param_from_file == false) continue;

            std::string param_file = config_path + "camera_" + name + ".json";
            g_logger->info("[{}] Loading camera parameters from '{}'...", name, param_file);
            std::ifstream file(param_file);
            if (!file.good()) 
            {
                succeed = false;
                g_logger->error("[{}] Camera parameters file '{}' not found!", name, param_file);
                continue;
            }
            // 1. 读取 JSON 文件
            json data = json::parse(file);

            // 2. 解析外参：旋转矩阵
            std::vector<float> rotation_vec = data["Camera_Rotation"];
            if (rotation_vec.size() != 9)
            {
                succeed = false;
                g_logger->error("[{}] Rotation matrix should have 9 elements!", name);
                continue;
            }
            for (uint i = 0; i < 9; ++i) 
                config.param.extrinsics.rotation[i] = rotation_vec[i];

            // 3. 解析外参：平移向量
            std::vector<float> translation_vec = data["Camera_Translation"];
            if (translation_vec.size() != 3) 
            {
                succeed = false;
                g_logger->error("[{}] Translation vector should have 3 elements!", name);
                continue;
            }
            for (uint i = 0; i < 3; ++i) 
                config.param.extrinsics.translation[i] = translation_vec[i];

            // 4. 解析 RGB 内参
            json& rgb_intrinsic = data["RGB_Sensor_Intrinsic"];
            config.param.rgb_intrinsics.fx = rgb_intrinsic["fx"];
            config.param.rgb_intrinsics.fy = rgb_intrinsic["fy"];
            config.param.rgb_intrinsics.cx = rgb_intrinsic["cx"];
            config.param.rgb_intrinsics.cy = rgb_intrinsic["cy"];
            
            // 5. 解析 RGB 畸变系数
            json& rgb_dist = data["RGB_Sensor_Distortion_Coefficient"];
            config.param.rgb_intrinsics.coeffs[0] = rgb_dist["k1"];
            config.param.rgb_intrinsics.coeffs[1] = rgb_dist["k2"];
            config.param.rgb_intrinsics.coeffs[2] = rgb_dist["p1"];
            config.param.rgb_intrinsics.coeffs[3] = rgb_dist["p2"];
            config.param.rgb_intrinsics.coeffs[4] = rgb_dist["k3"];
            
            // 6. 解析 TOF/Depth 内参
            json& tof_intrinsic = data["TOF_Sensor_Intrinsic"];
            config.param.dep_intrinsics.fx = tof_intrinsic["fx"];
            config.param.dep_intrinsics.fy = tof_intrinsic["fy"];
            config.param.dep_intrinsics.cx = tof_intrinsic["cx"];
            config.param.dep_intrinsics.cy = tof_intrinsic["cy"];
            
            // 7. 解析 TOF/Depth 畸变系数
            json& tof_dist = data["TOF_Sensor_Distortion_Coefficient"];
            config.param.dep_intrinsics.coeffs[0] = tof_dist["k1"];
            config.param.dep_intrinsics.coeffs[1] = tof_dist["k2"];
            config.param.dep_intrinsics.coeffs[2] = tof_dist["p1"];
            config.param.dep_intrinsics.coeffs[3] = tof_dist["p2"];
            config.param.dep_intrinsics.coeffs[4] = tof_dist["k3"];

            g_logger->info("[{}] Parameters loaded succeed from '{}'.", name, param_file);
        }
    } 
    catch (const std::exception& e) 
    {
        g_logger->error("Load camera parameters error: {}!", e.what());
        return false;
    }
    return succeed;
}

bool CameraDriver::loadCalibrations()
{
    std::string config_file(camera_config_dir() + "calibration.yaml");
    g_logger->info("Loading calibration from '{}'...", config_file);
    if (!std::ifstream(config_file).good())
    {
        g_logger->error("Config file '{}' not found!", config_file);
        return false;
    }

    // 加载YAML配置文件
    try
    {
        YAML::Node data = YAML::LoadFile(config_file);
        YAML::Node node = data["RobotBody"];
        fixed_head_ = node["fixed_head"].as<bool>();
        fold_       = node["fold"].as<bool>();
        g_logger->info("Robot version: {}.", fold_ ? "FOLDER" : "UP-RIGHT");
        if (fold_)
        {
            calf_length_  = node["calf_length"].as<float>();
            thigh_length_ = node["thigh_length"].as<float>();
        }

        // 辅助函数：从YAML节点读取位姿数据
        auto readPose = [&](const std::string& type) -> Pose3D
        {
            YAML::Node calibration_data = data[type];
            if (!calibration_data) 
            {
                g_logger->error("Calibration type '{}' does not exist!", type);
                return Pose3D();
            }

            return Pose3D(
                calibration_data["translation"]["x"].as<double>(),
                calibration_data["translation"]["y"].as<double>(),
                calibration_data["translation"]["z"].as<double>(),
                calibration_data["euler_degree"]["yaw"].as<double>()   * M_PI / 180.0,
                calibration_data["euler_degree"]["pitch"].as<double>() * M_PI / 180.0,
                calibration_data["euler_degree"]["roll"].as<double>()  * M_PI / 180.0
            );
        };

        // 遍历所有顶层节点
        for (YAML::const_iterator it = data.begin(); it != data.end(); ++it)
        {
            std::string name = it->first.as<std::string>();
            const YAML::Node& node = it->second;
            if (node["translation"] && node["euler_degree"])
                calib_map_[name] = readPose(name);
        }
        g_logger->info("Loaded calibration configurations succeed from '{}'.", config_file);
    }
    catch (const YAML::Exception& e)
    {
        g_logger->error("Load file '{}' error: {}!", config_file, e.what());
        return false;
    }
    catch (const std::exception& e)
    {
        g_logger->error("Faild to load file '{}': {}!", config_file, e.what());
        return false;
    }
    return true;
}

void CameraDriver::setupDefaultBasicInfo()
{
    g_logger->warn("Setting up default basic info...");
    debug_mode_        = false;

    body_info_.host    = "127.0.0.1";
    body_info_.pw      = "";
    body_info_.port    = 6379;
    chassis_info_.host = "192.168.0.3";
    chassis_info_.pw   = "";
    chassis_info_.port = 6379;

    use_body_          = true;
    use_chassis_       = true;
    body_offset_ms_    = 0;
    chassis_offset_ms_ = 0;
}

void CameraDriver::setupDefaultCameraConfigs() 
{
    // 当配置文件加载失败时使用的默认配置
    g_logger->warn("Setting up default camera configurations...");
    configs_.clear();
    for(auto &name : camera_names_)
    {
        CameraConfig cfg;
        cfg.name     = name;
        cfg.frame_id = name;
        cfg.width    = 640;
        cfg.qos      = "best_effort";
        if(name == "eye")         cfg.id = "192.168.2.10";
        else if(name == "chest")  cfg.id = "192.168.2.11";
        else if(name == "Bfront") cfg.id = "2.3.2";
        else if(name == "Bback")  cfg.id = "2.3.1";
        else if(name == "Bleft")  cfg.id = "2.3.5";
        else if(name == "Bright") cfg.id = "2.4.4";
        if(name == "eye" || name == "chest")
        {
            cfg.type   = "berxel";
            cfg.height = 400;
            cfg.send_chassis_depth = false;
        }
        else if(name == "Bfront" || name == "Bback" || name == "Bleft" || name == "Bright")
        {
            cfg.type   = "angstrong";
            cfg.height = 480;
            cfg.send_chassis_depth = true;
        }
        cfg.color_topic = "/cameras/" + name + "/color";
        cfg.depth_topic = "/cameras/" + name + "/depth";
        cfg.param_topic = "/cameras/" + name + "/param";
        cfg.cloud_topic = "/cameras/" + name + "/cloud";
        cfg.rgb_cloud_topic = "/cameras/" + name + "/rgb_cloud";
        cfg.chassis_depth_topic = cfg.depth_topic + "_chassis";
        cfg.chassis_param_topic = cfg.param_topic + "_chassis";
        cfg.pointcloud_type = 0;
        cfg.expos_us   = 0;
        cfg.expos_gain = 0;
        cfg.use_param_from_file = true;

        configs_[name] = cfg;
    }
}

void CameraDriver::saveBasicInfo()
{
    std::string config_file(camera_config_dir() + "basic_info.yaml");
    YAML::Node config;
    // 尝试加载现有文件
    try
    {
        config = YAML::LoadFile(config_file);
    }
    catch (const YAML::BadFile& e)
    {
        // 文件不存在，创建一个新的
        g_logger->error("Creating new YAML file: '{}'!", config_file);
    }

    config["Other"]["camera_driver_version"] = version_;

    std::ofstream fout(config_file);
    fout << config;
    fout.close();
    g_logger->info("Saved basic info to: '{}'.", config_file);
}

void CameraDriver::stop()
{
    if(!running_.load()) return;
    running_.store(false);
    saveBasicInfo();
    g_logger->info("Stopping pose estimator and all cameras ...");

    pose_estimator_->stop();

    // 停止博升相机
    for (auto& [name, berxel] : berxels_) 
    {
        berxel->stop();
        g_logger->info("[{}] Camera has stopped.", name);
    }
    berxels_.clear();
    CameraBerxel::destroy();

    // 停止安思疆相机
    if (angstrongs_) 
    {
        angstrongs_->stop();
        g_logger->info("Angstrong cameras have stopped.");
    }

    g_logger->info("Pose estimator and all cameras have stopped.");
}
