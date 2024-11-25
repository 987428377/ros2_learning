from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression


def generate_launch_description():
    turtlesim_ns = LaunchConfiguration('turtlesim_ns')
    use_provided_red = LaunchConfiguration('use_provided_red')
    new_background_r = LaunchConfiguration('new_background_r')

    turtlesim_ns_launch_arg = DeclareLaunchArgument(
        'turtlesim_ns',
        default_value='turtlesim1'
    )
    use_provided_red_launch_arg = DeclareLaunchArgument(
        'use_provided_red',
        default_value='False'
    )
    new_background_r_launch_arg = DeclareLaunchArgument(
        'new_background_r',
        default_value='200'
    )
    
#定义了命名空间设置为turtlesim_ns LaunchConfiguration 替换的turtlesim_node 节点。
    turtlesim_node = Node(
        package='turtlesim',
        namespace=turtlesim_ns,
        executable='turtlesim_node',
        name='sim'
    )
#然后，使用相应的 cmd 参数定义名为 spawn_turtle 的 ExecuteProcess 操作。
#该命令调用turtlesim 节点的spawn 服务。
#此外，LaunchConfiguration 替换用于获取turtlesim_ns 启动参数的值以构造命令字符串。
    spawn_turtle = ExecuteProcess(
        cmd=[[
            'ros2 service call ',
            turtlesim_ns,
            '/spawn ',
            'turtlesim/srv/Spawn ',
            '"{x: 2, y: 2, theta: 0.2}"'
        ]],
        shell=True
    )
#相同的方法用于更改turtlesim背景的红色参数的change_background_r操作。
    change_background_r = ExecuteProcess(
        cmd=[[
            'ros2 param set ',
            turtlesim_ns,
            '/sim background_r ',
            '120'
        ]],
        shell=True
    )
#相同的方法用于更改turtlesim 的change_background_r_condition操作。
#ExecuteProcess 是一个启动动作，用于在启动文件中执行外部进程（例如命令行指令）。
#condition 是一个条件参数，决定了这个动作是否应该被执行。
    #IfCondition 用于定义一个条件表达式，当条件为真时，才会执行这个动作。
    #PythonExpression 用于在启动文件中评估一个Python表达式。
    change_background_r_conditioned = ExecuteProcess(
        condition=IfCondition(
            PythonExpression([
                new_background_r,
                ' == 200',
                ' and ',
                use_provided_red
            ])
        ),
        #如果 new_background_r 等于200，并且 use_provided_red 为真，那么执行以下命令：
        cmd=[[
            'ros2 param set ',
            turtlesim_ns,
            '/sim background_r ',
            new_background_r
        ]],
        #shell=True 表示该命令将在shell中执行，这使得你可以使用shell的功能，如命令替换、环境变量等。
        shell=True
    )

    return LaunchDescription([
        turtlesim_ns_launch_arg,     #启动参数
        use_provided_red_launch_arg, #启动参数
        new_background_r_launch_arg, #启动参数
        turtlesim_node,              #启动节点
        spawn_turtle,                #启动终端
        change_background_r,         #启动条件终端
        #启动定时器动作，它用于在启动2秒后执行 change_background_r_conditioned 动作。
        TimerAction(
            period=2.0,
            actions=[change_background_r_conditioned],
        )
    ])