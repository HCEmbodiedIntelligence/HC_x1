# CameraPose.msg

builtin_interfaces/Time timestamp

# Camera 6DoF pose
float32[3] cam_translation
float32[3] cam_euler_angles

# Base 2D pose (chassis pose)
float32[2] base_translation
float32 base_theta

# Body 3DoF pose
float32[3] body_angles
# Body 2DoF pose
float32[3] body_states