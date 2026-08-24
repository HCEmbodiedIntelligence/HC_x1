#include "axis_transform.h"

AxisTransform::AxisTransform(std::map<std::string, Pose3D> &calib_map, 
    bool fixed_head, double calf, double thigh) : 
    calib_map_(calib_map), fixed_head_(fixed_head),
    calf_length_(calf), thigh_length_(thigh) 
{
    // 初始化所有坐标系位姿为0
    pose_map_[AxisLevel::CHASSIS_AXIS]   = Pose3D(0, 0, 0, 0, 0, 0);
    pose_map_[AxisLevel::BFRONT_AXIS]    = Pose3D(0, 0, 0, 0, 0, 0);
    pose_map_[AxisLevel::BBACK_AXIS]     = Pose3D(0, 0, 0, 0, 0, 0);
    pose_map_[AxisLevel::BLEFT_AXIS]     = Pose3D(0, 0, 0, 0, 0, 0);
    pose_map_[AxisLevel::BRIGHT_AXIS]    = Pose3D(0, 0, 0, 0, 0, 0);
    pose_map_[AxisLevel::CHEST_AXIS]     = Pose3D(0, 0, 0, 0, 0, 0);
    pose_map_[AxisLevel::LEFT_ARM_AXIS]  = Pose3D(0, 0, 0, 0, 0, 0);
    pose_map_[AxisLevel::RIGHT_ARM_AXIS] = Pose3D(0, 0, 0, 0, 0, 0);
    pose_map_[AxisLevel::EYE_AXIS]       = Pose3D(0, 0, 0, 0, 0, 0);
    
    initData();
}

// 从YAML文件初始化数据
void AxisTransform::initData() 
{
    // 设置各个标定参数
    setBFront2BaseCalib(calib_map_["BaseFront2Base"]);
    setBBack2BFrontCalib(calib_map_["BaseBack2BaseFront"]);
    setBLeft2BFrontCalib(calib_map_["BaseLeft2BaseFront"]);
    setBRight2BFrontCalib(calib_map_["BaseRight2BaseFront"]);
    
    // 设置多级坐标变换参数
    setChestWaistBaseCalib(
        calib_map_["BaseFront2WaistLift"],
        calib_map_["Chest2WaistPitch"]
    );
    
    // 设置手臂基座坐标参数
    setChest2ArmLeftCalib(calib_map_["Chest2ArmLeft"]);
    setChest2ArmRightCalib(calib_map_["Chest2ArmRight"]);
    
    // 设置头部相关坐标参数
    setEyeNeckChestCalib(
        calib_map_["Chest2Neck"],
        calib_map_["Eye2Head"]
    );
    
    // 设置初始角度
    if (fixed_head_) setHeadVar(0);
}

// 获取两个坐标系之间的相对位姿
Pose3D AxisTransform::getRelativePose(AxisLevel axis_from, AxisLevel axis_to) const 
{
    if (axis_from == axis_to) 
        return Pose3D();  // 返回单位矩阵
    
    int from_group = static_cast<int>(axis_from) / 10;
    int to_group = static_cast<int>(axis_to) / 10;
    
    if (from_group == to_group) 
    {
        Pose3D pose_from = getAxisRelativePose(axis_from);
        Pose3D pose_to = getAxisRelativePose(axis_to);
        return pose_to.inverse() * pose_from;
    }
    else if (axis_from > axis_to) 
        return getRelativePoseHighToLow(axis_from, axis_to);
    else // axis_from < axis_to
        return getRelativePoseHighToLow(axis_to, axis_from).inverse();
}

// 获取指定坐标系的相对位姿
Pose3D AxisTransform::getAxisRelativePose(AxisLevel axis_level) const 
{
    auto it = pose_map_.find(axis_level);
    if (it != pose_map_.end()) return it->second;
    return Pose3D();  // 返回单位矩阵
}

// 获取从高等级坐标系到低等级坐标系的位姿变换
Pose3D AxisTransform::getRelativePoseHighToLow(AxisLevel axis_level_high, 
    AxisLevel axis_level_low) const 
{
    Pose3D result = getAxisRelativePose(axis_level_high);
    
    int high_group = static_cast<int>(axis_level_high) / 10 * 10;
    int low_group = static_cast<int>(axis_level_low) / 10 * 10;
    
    AxisLevel next_level = static_cast<AxisLevel>(high_group - 10);
    while (next_level >= static_cast<AxisLevel>(low_group)) 
    {
        result = getAxisRelativePose(next_level) * result;
        next_level = static_cast<AxisLevel>(static_cast<int>(next_level) - 10);
    }
    
    Pose3D pose_to = getAxisRelativePose(axis_level_low);
    return pose_to.inverse() * result;
}

// 设置立柱版腰部变量，弧度
void AxisTransform::setWaistVar(double lift_height, double pitch_rad) 
{    
    Pose3D waist_pitch2lift = Pose3D(0, 0, lift_height, 0, pitch_rad, 0);
    Pose3D chest2Bfront = waist_lift2Bfront_
                        * waist_pitch2lift
                        * chest2waist_pitch_;
    
    setAxisRelativePose(AxisLevel::CHEST_AXIS, chest2Bfront);
}

// 设置折叠版腰部变量，弧度
void AxisTransform::setWaistVar(double pitch1, double pitch2, double pitch3) 
{    
    Pose3D pose1(0, 0, 0, 0, pitch1, 0);
    Pose3D pose2(0, 0, calf_length_, 0, pitch2, 0);
    Pose3D pose3(0, 0, thigh_length_, 0, pitch3, 0);
    Pose3D waist_pitch2lift = pose1 * pose2 * pose3;
    // pose1.print("pose1");
    // pose2.print("pose2");
    // pose3.print("pose3");
    // waist_pitch2lift.print("waist_pitch2lift");
    Pose3D chest2Bfront = waist_lift2Bfront_
                        * waist_pitch2lift
                        * chest2waist_pitch_;
    // waist_lift2Bfront_.print("waist_lift2Bfront");
    // chest2waist_pitch_.print("chest2waist_pitch");
    // chest2Bfront.print("chest2Bfront");
    
    setAxisRelativePose(AxisLevel::CHEST_AXIS, chest2Bfront);
}


