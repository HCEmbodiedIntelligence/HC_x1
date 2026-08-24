# CameraParameters.msg

# RGB相机内参
float32 rgb_fx
float32 rgb_fy
float32 rgb_cx
float32 rgb_cy
float32[5] rgb_distortion_coeffs  # k1, k2, p1, p2, k3

# 深度相机内参
float32 dep_fx
float32 dep_fy
float32 dep_cx
float32 dep_cy
float32[5] dep_distortion_coeffs

# 外参：旋转矩阵（行优先）和平移向量
# [R11, R12, R13, R21, R22, R23, R31, R32, R33]
float32[9] rotation
float32[3] translation
