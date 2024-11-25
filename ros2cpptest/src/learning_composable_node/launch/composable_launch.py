
# from launch import LaunchDescription
# from launch_ros.actions import Node


# def generate_launch_description():
#     ld = LaunchDescription()

#     ld.add_action(Node(
#         package='learning_composable_node',
#         executable='vincent_driver',
#         output='screen'
#     ))
#     return ld

#LaunchDescription：启动系统中定义和描述一个启动文件的类，它用来创建一个启动描述对象。
from launch import LaunchDescription  
#ComposableNodeContainer：用于创建一个容器，该容器可以包含多个可组合节点。这个容器会在同一个进程中启动这些节点，以提高通信效率。
from launch_ros.actions import  ComposableNodeContainer
#ComposableNode：这是一个描述可组合节点的类。它定义了一个组件节点的相关属性，比如包名、插件名称等。
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    ld = LaunchDescription()
    ld.add_action(ComposableNodeContainer(
        name='a_buncha_nodes', #name='a_buncha_nodes'：指定容器的名称。这个名称可以用于在日志或调试时识别容器。
        namespace='',
        package='rclcpp_components',#指定用于运行容器的 ROS 2 包。rclcpp_components 包含了运行组件容器所需的执行文件。
        executable='component_container', # 如果您需要多线程，请不要将可执行文件设置为 component_container，而是将其设置为 component_container_mt
        output='screen', #指定容器的输出方式为屏幕，这意味着容器的日志信息将显示在终端中。
        composable_node_descriptions=[ #一个列表，包含要在容器中运行的可组合节点的描述。在这里，你可以定义一个或多个 ComposableNode 对象，每个对象描述一个要在容器中运行的组件节点。
            ComposableNode(
                package='learning_composable_node', #指定包含节点的 ROS 2 包名称。这里是 learning_composable_node 包，其中定义了组件节点。
                plugin='palomino::VincentDriver', #指定节点的插件名称。这是组件类的全名，包括包名和类名（palomino::VincentDriver）。这个名称应该与 C++ 代码中的插件注册名匹配。
                name='vincent_driver', #指定节点的名称。在容器中启动时，这个名称将被用来识别节点。
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
        ],
        remappings=[] #用于指定任何需要的主题（topic）重映射规则。如果你没有主题重映射需求，可以保持为空。
    ))
    return ld
