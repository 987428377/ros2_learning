#这些 import 语句引入一些 Python 启动模块
from launch import LaunchDescription
from launch_ros.actions import Node
#接下来，启动描述本身开始：
def generate_launch_description():
    return LaunchDescription([
        #启动描述中的前两个操作启动两个turtlesim窗口：
        Node(
            package='turtlesim',
            namespace='turtlesim1',
            executable='turtlesim_node',
            name='sim'
        ),
        Node(
            package='turtlesim',
            namespace='turtlesim2',
            executable='turtlesim_node',
            name='sim'
        ),
        #最后一个操作启动带有重映射的模拟节点：
        Node(
            package='turtlesim',
            executable='mimic',
            name='mimic',
            remappings=[
                ('/input/pose', '/turtlesim1/turtle1/pose'),
                ('/output/cmd_vel', '/turtlesim2/turtle1/cmd_vel'),
            ]
        )
    ])