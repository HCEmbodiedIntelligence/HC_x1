#include "pose3d.h"
#include <sstream>
#include <iostream>

// 构造函数
Pose3D::Pose3D() : translation_(Eigen::Vector3d::Zero()), quaterniond_(Eigen::Quaterniond::Identity()) {}

// 构造函数 - 从位置和欧拉角，弧度
Pose3D::Pose3D(double x, double y, double z, 
    double yaw, double pitch, double roll)
{
    translation_ = Eigen::Vector3d(x, y, z);
    setEulerAngles(yaw, pitch, roll);
}

Pose3D::Pose3D(const Eigen::Quaterniond& quaterniond, const Eigen::Vector3d& translation)
    : translation_(translation), quaterniond_(quaterniond.normalized()) {}

Pose3D::Pose3D(const Eigen::Matrix3d& rotation_matrix, const Eigen::Vector3d& translation)
    : translation_(translation), quaterniond_(Eigen::Quaterniond(rotation_matrix)) {}

Pose3D::Pose3D(const Eigen::Matrix4d& transform_matrix)
{
    quaterniond_ = Eigen::Quaterniond(transform_matrix.block<3,3>(0,0)).normalized();
    translation_ = transform_matrix.block<3,1>(0,3);
}

// 设置函数
void Pose3D::setTranslation(const Eigen::Vector3d& translation)
{
    translation_ = translation;
}

void Pose3D::setQuaterniond(const Eigen::Quaterniond& quaterniond)
{
    quaterniond_ = quaterniond.normalized();
}

void Pose3D::setRotationMatrix(const Eigen::Matrix3d& rotation_matrix)
{
    quaterniond_ = Eigen::Quaterniond(rotation_matrix).normalized();
}

void Pose3D::setEulerAngles(double yaw, double pitch, double roll)
{
    quaterniond_ = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
                Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
                Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());
}

void Pose3D::setTransformMatrix(const Eigen::Matrix4d& transform_matrix)
{
    quaterniond_ = Eigen::Quaterniond(transform_matrix.block<3,3>(0,0)).normalized();
    translation_ = transform_matrix.block<3,1>(0,3);
}

// 获取函数
Eigen::Vector3d Pose3D::translation() const
{
    return translation_;
}

Eigen::Quaterniond Pose3D::quaternion() const
{
    return quaterniond_;
}

Eigen::Matrix3d Pose3D::rotationMatrix() const
{
    return quaterniond_.toRotationMatrix();
}

Eigen::Vector3d Pose3D::eulerAngles() const
{
    return normalizeEulerAngles(); // Z-Y-X顺序
}

Eigen::Matrix4d Pose3D::matrix() const
{
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T.block<3,3>(0,0) = rotationMatrix();
    T.block<3,1>(0,3) = translation_;
    return T;
}

// 位姿操作
Pose3D Pose3D::operator*(const Pose3D& other) const
{
    return Pose3D(
                quaterniond_ * other.quaterniond_,
                quaterniond_ * other.translation_ + translation_
    );
}

Pose3D Pose3D::inverse() const
{
    Eigen::Quaterniond inv_rot = quaterniond_.conjugate(); // 等同于inverse()
    return Pose3D(inv_rot, inv_rot * (-translation_));
//    Eigen::Quaterniond inv_rot = quaterniond_.inverse();
//    return Pose3D(inv_rot, inv_rot * (-translation_));
}

Eigen::Vector3d Pose3D::transformPoint(const Eigen::Vector3d& point) const
{
    return quaterniond_ * point + translation_;
}

Eigen::Vector3d Pose3D::transformVector(const Eigen::Vector3d& vector) const
{
    return quaterniond_ * vector;
}

// 实用函数
Eigen::Vector3d Pose3D::normalizeEulerAngles() const
{
    Eigen::Vector3d euler = quaterniond_.matrix().eulerAngles(2, 1, 0);
    double yaw   = euler[0];
    double pitch = euler[1];
    double roll  = euler[2];
    // 如果 pitch 不在 [-π/2, π/2]，则进行等价变换
    if (pitch > M_PI/2)
    {
        pitch = M_PI - pitch;
        yaw += M_PI;
        roll += M_PI;
    }
    else if (pitch < -M_PI/2)
    {
        pitch = -M_PI - pitch;
        yaw += M_PI;
        roll += M_PI;
    }

    // 对角度做模 2π 处理
    yaw = fmod(yaw + M_PI, 2 * M_PI);
    yaw = (yaw <= 0) ? yaw + M_PI : yaw - M_PI;

    roll = fmod(roll + M_PI, 2 * M_PI);
    roll = (roll <= 0) ? roll + M_PI : roll - M_PI;

    return Eigen::Vector3d(yaw, pitch, roll);
}

bool Pose3D::isApprox(const Pose3D& other, double translation_tol, double rotation_tol) const
{
    return translation_.isApprox(other.translation_, translation_tol) &&
           quaterniond_.isApprox(other.quaterniond_, rotation_tol);
}

void Pose3D::normalizeRotation()
{
    quaterniond_.normalize();
}

void Pose3D::print(const std::string& name) const
{
    std::stringstream ss;
    if (!name.empty()) ss << "[" + name + "] ";

    ss << "Translation: " << translation_.transpose() << ", ";

    Eigen::Vector3d euler = eulerAngles();
    ss << "Euler angles(deg): "
       << euler.transpose() * 180/M_PI << " \n";
    // ss << "Rotation matrix:\n" << rotationMatrix() << "\n";
    // std::cout << ss.str();

    // ss << "Quaternion (w,x,y,z): "
    //    << quaterniond_.w() << ", "
    //    << quaterniond_.x() << ", "
    //    << quaterniond_.y() << ", "
    //    << quaterniond_.z() << "\n";

    // ss << "Rotation matrix:\n" << rotationMatrix() << "\n";
    // ss << "Transformation matrix:\n" << matrix() << "\n";
    // return ss.str();
}

// 静态函数
Pose3D Pose3D::identity()
{
    return Pose3D();
}

Pose3D Pose3D::fromTranslation(const Eigen::Vector3d& translation)
{
    return Pose3D(Eigen::Quaterniond::Identity(), translation);
}

Pose3D Pose3D::fromRotation(const Eigen::Quaterniond& rotation)
{
    return Pose3D(rotation, Eigen::Vector3d::Zero());
}

Pose3D Pose3D::fromEulerAngles(double yaw, double pitch, double roll)
{
    Pose3D pose;
    pose.setEulerAngles(yaw, pitch, roll);
    return pose;
}

bool Pose3D::isIdentity()
{
    if(std::abs(translation_.x()) < 0.01 && std::abs(translation_.y()) < 0.01
            && std::abs(translation_.z()) < 0.01)
        return true;
    return false;
}
