#ifndef POS3D_H
#define POS3D_H

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <vector>

class Pose3D 
{
public:
    // 构造函数
    Pose3D();  
    // 构造函数 - 从位置和欧拉角，弧度
    Pose3D(double x, double y, double z, 
        double yaw, double pitch, double roll);
    Pose3D(const Eigen::Quaterniond& quaterniond, const Eigen::Vector3d& translation);
    Pose3D(const Eigen::Matrix3d& rotation_matrix, const Eigen::Vector3d& translation);
    explicit Pose3D(const Eigen::Matrix4d& transform_matrix);

    // 设置函数
    void setTranslation(const Eigen::Vector3d& translation);
    void setQuaterniond(const Eigen::Quaterniond& quaterniond);
    void setRotationMatrix(const Eigen::Matrix3d& rotation_matrix);
    void setEulerAngles(double yaw, double pitch, double roll);
    void setTransformMatrix(const Eigen::Matrix4d& transform_matrix);

    // 获取函数
    Eigen::Vector3d translation() const;
    Eigen::Quaterniond quaternion() const;
    Eigen::Matrix3d rotationMatrix() const;
    Eigen::Vector3d eulerAngles() const; // Z-Y-X顺序 (yaw-pitch-roll)
    Eigen::Matrix4d matrix() const;

    // 位姿操作
    Pose3D operator*(const Pose3D& other) const;
    Pose3D inverse() const;
    Eigen::Vector3d transformPoint(const Eigen::Vector3d& point) const;
    Eigen::Vector3d transformVector(const Eigen::Vector3d& vector) const;

    // 实用函数
    Eigen::Vector3d normalizeEulerAngles() const;
    bool isApprox(const Pose3D& other, double translation_tol = 1e-6, double rotation_tol = 1e-6) const;
    void normalizeRotation();
    void print(const std::string& name = "") const;

    // 静态函数
    static Pose3D identity();
    static Pose3D fromTranslation(const Eigen::Vector3d& translation);
    static Pose3D fromRotation(const Eigen::Quaterniond& rotation);
    static Pose3D fromEulerAngles(double yaw, double pitch, double roll);

    bool isIdentity();

private:
    Eigen::Vector3d translation_;
    Eigen::Quaterniond quaterniond_;
};

#endif