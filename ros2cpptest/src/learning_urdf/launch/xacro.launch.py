# 加载机器人模型的一种更简单的方法是使用 urdf_launch 包自动加载 xacro/urdf。
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    ld = LaunchDescription()

    ld.add_action(IncludeLaunchDescription(
        PathJoinSubstitution([FindPackageShare('urdf_launch'), 'launch', 'display.launch.py']),
        launch_arguments={
            'urdf_package': 'turtlebot3_description',
            'urdf_package_path': PathJoinSubstitution(['urdf', 'turtlebot3_burger.urdf'])}.items()
    ))
    return ld