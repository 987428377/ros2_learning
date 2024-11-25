import os
#get_package_share_directory: 这是 ament_index_python 库中的函数，用于获取指定 ROS 2 包的共享目录路径。它返回一个字符串，表示包的路径。
from ament_index_python.packages import get_package_share_directory 
#LaunchDescription: 是 ROS 2 启动系统中用来描述启动文件的类，它包含了所有的启动命令和动作。
from launch import LaunchDescription
#DeclareLaunchArgument: 允许在启动文件中声明参数，这些参数可以从命令行传递给启动文件。
#SetEnvironmentVariable: 用来设置环境变量，通常用于调整运行时环境的设置。
from launch.actions import DeclareLaunchArgument,SetEnvironmentVariable
#LaunchConfiguration: 启动时的配置项（即启动时传递的参数）。用来获取启动文件中的参数值。
from launch.substitutions import LaunchConfiguration
#Node: 专门用于定义和启动 ROS 2 节点。。
from launch_ros.actions import Node


def generate_launch_description():  
    pkg_dir = get_package_share_directory('change_yaml_pkg')

    namespace = LaunchConfiguration('namespace')                      #namespace: 用于定义该节点的命名空间。
    use_namespace = LaunchConfiguration('use_namespace')              #use_namespace: 用于指定是否使用命名空间。
    use_sim_time = LaunchConfiguration('use_sim_time')                #use_sim_time: 用于指定是否使用仿真时间。
    config_file = LaunchConfiguration('config_file')                  #config_file: 用于指定配置文件的路径。

    # etEnvironmentVariable 设置了一个环境变量：RCUTILS_LOGGING_BUFFERED_STREAM。
    # 设置为 1 后，它会启用 ROS 2 的日志缓冲，这意味着输出的日志信息会被缓存在内存中，直到被刷新，这样可以减少输出频率，避免日志信息被频繁打印出来。
    stdout_linebuf_envvar = SetEnvironmentVariable(
		'RCUTILS_LOGGING_BUFFERED_STREAM', '1')

    # DeclareLaunchArgument: 这部分代码使用 DeclareLaunchArgument 来声明启动文件中的参数，声明后这些参数会暴露给用户，并可以通过命令行来传递。
    declare_namespace_cmd = DeclareLaunchArgument(
		'namespace',
		default_value='change_yaml',
		description='Top-level namespace')
    
    declare_use_namespace_cmd = DeclareLaunchArgument(
		'use_namespace',
		default_value='false',
		description='Whether to apply a namespace to the navigation stack')
    
    declare_use_sim_time_cmd = DeclareLaunchArgument(
		'use_sim_time',
		default_value='true',
		description='Use simulation (Gazebo) clock if true')
    
    declare_config_file_cmd = DeclareLaunchArgument(
		'config_file', default_value=os.path.join(pkg_dir, 'config', 'change.yaml'),
		description='Full path to yaml config file to load')
    
    node_cmd = Node(
        package='change_yaml_pkg',
        executable='change_yaml_pkg_node',
        name='change_yaml_pkg_node',
        # namespace=namespace,
        output='screen',
		   ,
        parameters=[{'test': 'ABS',
                     'use_sim_time': use_sim_time
                     'config_path': LaunchConfiguration('config_file'),
                     },
            config_file,
                    ],                                #parameters: 用来加载和设置节点参数。参数值可以直接指定，也可以通过配置文件传递。
        emulate_tty=True,
        # arguments=['--ros-args', '--param', 'global_map_path:="ABCD"'],
        #arguments=['--ros-args', '--params-file', config_file],  #arguments:  主要用于启动时传递 ROS 2 特定的命令行参数（例如 --ros-args, --params-file）。这可以用于动态地覆盖启动文件中的参数或传递额外的启动选项。
    )


    
    return LaunchDescription([
        declare_namespace_cmd,
        declare_use_namespace_cmd,
        declare_use_sim_time_cmd,
        declare_config_file_cmd,
        stdout_linebuf_envvar,
        node_cmd,
    ])