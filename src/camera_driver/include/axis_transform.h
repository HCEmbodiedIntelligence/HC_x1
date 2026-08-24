#ifndef AXIS_TRANSFORM_H
#define AXIS_TRANSFORM_H

#include <vector>
#include <map>
#include <cmath>
#include <iostream>
#include "pose3d.h"

// 定义坐标系枚举
enum class AxisLevel : int 
{
    NONE_AXIS      = -20,
    MAP_AXIS       = -10,
    CHASSIS_AXIS   = 0,
    BFRONT_AXIS    = 10,
    CHEST_AXIS     = 20,
    BBACK_AXIS     = 21,
    BLEFT_AXIS     = 22,
    BRIGHT_AXIS    = 23,
    EYE_AXIS       = 30,
    LEFT_ARM_AXIS  = 31,
    RIGHT_ARM_AXIS = 32
};

class AxisTransform 
{
public:
    AxisTransform(std::map<std::string, Pose3D> &calib_map, 
        bool fixed_head, double calf, double thigh);
    void getCalfThigh(double &calf, double &thigh)
    {
        calf = calf_length_; thigh = thigh_length_;
    }
    void getRedis(std::string &body_addr, std::string &body_pw, int &body_port,
        std::string &chassis_addr, std::string &chassis_pw, int &chassis_port)
    {
        body_addr = body_addr_;   body_pw_ = body_pw_;
        body_port = body_port_;   chassis_addr = chassis_addr_;
        chassis_pw = chassis_pw_; chassis_port = chassis_port_;
    }
    // 获取两个坐标系之间的相对位姿
    Pose3D getRelativePose(AxisLevel axis_from, AxisLevel axis_to) const;
    
    // 设置前相机到车辆的标定参数
    void setBFront2BaseCalib(const Pose3D& pose_calib) 
    {
        setAxisRelativePose(AxisLevel::BFRONT_AXIS, pose_calib);
    }    
    // 设置后相机到前相机的标定参数
    void setBBack2BFrontCalib(const Pose3D& pose_calib) 
    {
        setAxisRelativePose(AxisLevel::BBACK_AXIS, pose_calib);
    }    
    // 设置左相机到前相机的标定参数
    void setBLeft2BFrontCalib(const Pose3D& pose_calib) 
    {
        setAxisRelativePose(AxisLevel::BLEFT_AXIS, pose_calib);
    }    
    // 设置右相机到前相机的标定参数
    void setBRight2BFrontCalib(const Pose3D& pose_calib) 
    {
        setAxisRelativePose(AxisLevel::BRIGHT_AXIS, pose_calib);
    }
    
    // 设置身体相机相关的标定参数
    void setChestWaistBaseCalib(
        const Pose3D& Bfront2waist_lift, 
        const Pose3D& chest2waist_pitch) 
    {
        waist_lift2Bfront_ = Bfront2waist_lift.inverse();
        chest2waist_pitch_ = chest2waist_pitch;
        Bfront2waist_lift.print("Bfront2waist_lift");
        waist_lift2Bfront_.print("waist_lift2Bfront");
        chest2waist_pitch_.print("chest2waist_pitch");
    }
        
    // 设置身体相机到左臂基座的标定参数
    void setChest2ArmLeftCalib(const Pose3D& chest2left_arm) 
    {
        setAxisRelativePose(AxisLevel::LEFT_ARM_AXIS, chest2left_arm.inverse());
    }
    // 设置身体相机到右臂基座的标定参数
    void setChest2ArmRightCalib(const Pose3D& chest2right_arm) 
    {
        setAxisRelativePose(AxisLevel::RIGHT_ARM_AXIS, chest2right_arm.inverse());
    }
    
    // 设置头部相机相关的标定参数
    void setEyeNeckChestCalib(
        const Pose3D& chest2neck, 
        const Pose3D& eye2head) 
    {
        neck2chest_ = chest2neck.inverse();
        eye2head_ = eye2head;
    }
    
    // 设置车辆底盘位姿
    void setChassisPose(double x, double y, double yaw_deg) 
    {
        Pose3D vehicle_pose(x, y, 0, yaw_deg, 0, 0);
        setAxisRelativePose(AxisLevel::CHASSIS_AXIS, vehicle_pose);
    }
    // 设置头部变量（俯仰），弧度
    void setHeadVar(double pitch_rad) 
    {
        Pose3D head2neck(0, 0, 0, 0, pitch_rad, 0);
        Pose3D eye2chest = neck2chest_ * head2neck * eye2head_;        
        setAxisRelativePose(AxisLevel::EYE_AXIS, eye2chest);
    }
    
    // 设置腰部变量（升降和俯仰），弧度
    void setWaistVar(double lift_height, double pitch_rad);
    // 设置折叠版腰部变量，弧度
    void setWaistVar(double pitch1, double pitch2, double pitch3);
    
    // 获取坐标系索引列表
    // std::vector<AxisLevel> getAxisLevelIndexList() const {
    //     return axis_level_index_list_;
    // }

private:
    // 从YAML文件初始化数据
    void initData();
    // 获取指定坐标系的相对位姿
    Pose3D getAxisRelativePose(AxisLevel axis_level) const;
    // 获取从高等级坐标系到低等级坐标系的位姿变换
    Pose3D getRelativePoseHighToLow(AxisLevel axis_level_high, 
        AxisLevel axis_level_low) const;    
    // 设置指定坐标系的相对位姿
    void setAxisRelativePose(AxisLevel axis_level, const Pose3D& pose)
    {
        pose_map_[axis_level] = pose;
    }
    
    std::map<std::string, Pose3D> calib_map_;
    std::map<AxisLevel, Pose3D> pose_map_;
    Pose3D waist_lift2Bfront_;
    Pose3D chest2waist_pitch_;
    Pose3D neck2chest_;
    Pose3D eye2head_;
    
    bool fold_, fixed_head_;
    double calf_length_;
    double thigh_length_;

    int body_port_, chassis_port_;
    std::string body_addr_, body_pw_;
    std::string chassis_addr_, chassis_pw_;
    
    // std::vector<AxisLevel> axis_level_index_list_;
};

#endif