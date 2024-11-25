from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, ExecuteProcess,
                            LogInfo, RegisterEventHandler, TimerAction)
from launch.conditions import IfCondition
from launch.event_handlers import (OnExecutionComplete, OnProcessExit,
                                OnProcessIO, OnProcessStart, OnShutdown)
from launch.events import Shutdown
from launch.substitutions import (EnvironmentVariable, FindExecutable,
                                LaunchConfiguration, LocalSubstitution,
                                PythonExpression)


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

    turtlesim_node = Node(
        package='turtlesim',
        namespace=turtlesim_ns,
        executable='turtlesim_node',
        name='sim'
    )
    spawn_turtle = ExecuteProcess(
        cmd=[[
            FindExecutable(name='ros2'),
            ' service call ',
            turtlesim_ns,
            '/spawn ',
            'turtlesim/srv/Spawn ',
            '"{x: 2, y: 2, theta: 0.2}"'
        ]],
        shell=True
    )
    change_background_r = ExecuteProcess(
        cmd=[[
            FindExecutable(name='ros2'),
            ' param set ',
            turtlesim_ns,
            '/sim background_r ',
            '120'
        ]],
        shell=True
    )
    change_background_r_conditioned = ExecuteProcess(
        condition=IfCondition(
            PythonExpression([
                new_background_r,
                ' == 200',
                ' and ',
                use_provided_red
            ])
        ),
        cmd=[[
            FindExecutable(name='ros2'),
            ' param set ',
            turtlesim_ns,
            '/sim background_r ',
            new_background_r
        ]],
        shell=True
    )

    return LaunchDescription([
        turtlesim_ns_launch_arg,
        use_provided_red_launch_arg,
        new_background_r_launch_arg,
        turtlesim_node,
        #OnProcessStart 事件处理程序用于注册在turtlesim 节点启动时执行的回调函数。
        #当turtlesim节点启动时，它会向控制台记录一条消息并执行spawn_turtle操作。
        RegisterEventHandler(
            OnProcessStart(
                target_action=turtlesim_node,
                on_start=[
                    LogInfo(msg='Turtlesim started, spawning turtle'),
                    spawn_turtle
                ]
            )
        ),
        #OnProcessIO 事件处理程序用于注册当spawn_turtle 操作写入其标准输出时执行的回调函数。
        #它记录生成请求的结果。
        RegisterEventHandler(
            OnProcessIO(
                #target_action=spawn_turtle: 目标动作是 spawn_turtle，即当 spawn_turtle 产生标准输出时触发。
                target_action=spawn_turtle,
                # 当 spawn_turtle 操作写入标准输出时，记录输出信息。
                on_stdout=lambda event: LogInfo(
                    msg='Spawn request says "{}"'.format(
                        event.text.decode().strip())  #event.text.decode().strip(): 获取并解码标准输出文本，去除首尾空格。
                )
            )
        ),
        #OnExecutionComplete 事件处理程序用于注册在spawn_turtle 操作完成时执行的回调函数。
        #它会向控制台记录一条消息，并在生成操作完成时执行change_background_r 和change_background_r_conditioned 操作。
        RegisterEventHandler(
            OnExecutionComplete(
                target_action=spawn_turtle,
                on_completion=[
                    LogInfo(msg='Spawn finished'),
                    change_background_r,
                    TimerAction(
                        period=2.0,
                        actions=[change_background_r_conditioned],
                    )
                ]
            )
        ),
        #OnProcessExit 事件处理程序用于注册当turtlesim 节点退出时执行的回调函数。
        #它将一条消息记录到控制台并执行 EmitEvent 操作，以便在turtlesim 节点退出时发出 Shutdown 事件。
        #意味着当turtlesim窗口关闭时，启动进程将关闭。
        RegisterEventHandler(
            OnProcessExit(
                target_action=turtlesim_node,
                on_exit=[
                    LogInfo(msg=(EnvironmentVariable(name='USER'),
                            ' closed the turtlesim window')),
                    EmitEvent(event=Shutdown(
                        reason='Window closed'))
                ]
            )
        ),
        #最后，OnShutdown 事件处理程序用于注册一个回调函数，该函数在启动文件被要求关闭时执行。
        #它会向控制台记录一条消息，说明为什么要求关闭启动文件。它会记录带有关闭原因的消息，例如关闭turtlesim窗口或用户发出的ctrl-c信号。
        RegisterEventHandler(
            OnShutdown(
                on_shutdown=[LogInfo(
                    msg=['Launch was asked to shutdown: ',
                        LocalSubstitution('event.reason')]
                )]
            )
        ),
    ])