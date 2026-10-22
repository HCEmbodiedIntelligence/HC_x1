"""hc_tj 上半身启动：Marvin 双臂 + 腰部，同一个节点。

腰部走 Redis servoJ，双臂走 Marvin SDK，但反馈必须合成同一条
/io_teleop/joint_states（17 关节）。TeleXperience 的左右臂 IK 分别以
R_arm_0_Link/L_arm_0_Link 为基座；腰部仍是第三条受控运动链，缺腰部关节就不会下发 joint_cmd。

arms:=false 用于双臂未接线时单独调腰，此时只发布腰部关节。
arms_control:=false 保留双臂反馈，但禁止双臂控制、模式切换和回零。
left_arm_control:=false / right_arm_control:=false 可单独将一侧机械臂设为只读。
waist:=false 用于仅启动双臂，此时不连接腰部 Redis。
waist_control:=false 保留腰部状态上报，但禁止腰部命令和回零。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    arms_arg = DeclareLaunchArgument(
        'arms',
        default_value='true',
        description='是否连接 Marvin 双臂；false 为仅腰部调试模式',
    )
    arms_control_arg = DeclareLaunchArgument(
        'arms_control',
        default_value='true',
        description='是否允许下发双臂命令；false 为双臂状态只读模式',
    )
    left_arm_control_arg = DeclareLaunchArgument(
        'left_arm_control',
        default_value='true',
        description='是否允许下发左臂命令、切换模式和回零',
    )
    right_arm_control_arg = DeclareLaunchArgument(
        'right_arm_control',
        default_value='true',
        description='是否允许下发右臂命令、切换模式和回零',
    )
    waist_arg = DeclareLaunchArgument(
        'waist',
        default_value='true',
        description='是否启用腰部；false 为仅双臂模式',
    )
    waist_control_arg = DeclareLaunchArgument(
        'waist_control',
        default_value='true',
        description='是否允许下发腰部命令；false 为腰部状态只读模式',
    )

    bridge_node = Node(
        package='hc_tj_marvin_bridge_cpp',
        executable='hc_tj_marvin_bridge_node',
        name='hc_tj_marvin_bridge',
        output='screen',
        parameters=[
            {
                'connect_arms': ParameterValue(
                    LaunchConfiguration('arms'), value_type=bool
                ),
                'enable_arms_control': ParameterValue(
                    LaunchConfiguration('arms_control'), value_type=bool
                ),
                'enable_left_arm_control': ParameterValue(
                    LaunchConfiguration('left_arm_control'), value_type=bool
                ),
                'enable_right_arm_control': ParameterValue(
                    LaunchConfiguration('right_arm_control'), value_type=bool
                ),
                'enable_waist': ParameterValue(
                    LaunchConfiguration('waist'), value_type=bool
                ),
                'enable_waist_control': ParameterValue(
                    LaunchConfiguration('waist_control'), value_type=bool
                ),
            }
        ],
    )

    return LaunchDescription([
        arms_arg,
        arms_control_arg,
        left_arm_control_arg,
        right_arm_control_arg,
        waist_arg,
        waist_control_arg,
        bridge_node,
    ])
