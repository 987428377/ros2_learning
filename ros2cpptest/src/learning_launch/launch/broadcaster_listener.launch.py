from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
   return LaunchDescription([
       #我们声明了 target_frame 启动参数，默认值为turtle1。
       #默认值意味着启动文件可以接收参数以转发到其节点，或者在未提供参数的情况下，它将将默认值传递到其节点。
      DeclareLaunchArgument(
         'target_frame', default_value='turtle1',
         description='Target frame name.'
      ),
       #之后，我们在启动期间使用不同的名称和参数两次使用turtle_tf2_broadcaster节点。
       #这允许我们复制相同的节点而不会发生冲突
      Node(
         package='turtle_tf2_py',
         executable='turtle_tf2_broadcaster',
         name='broadcaster1',
         parameters=[
            {'turtlename': 'turtle1'}
         ]
      ),
      Node(
         package='turtle_tf2_py',
         executable='turtle_tf2_broadcaster',
         name='broadcaster2',
         parameters=[
            {'turtlename': 'turtle2'}
         ]
      ),
       #们还启动一个turtle_tf2_listener节点并设置我们在上面声明和获取的target_frame参数。
      Node(
         package='turtle_tf2_py',
         executable='turtle_tf2_listener',
         name='listener',
         parameters=[
            {'target_frame': LaunchConfiguration('target_frame')}
         ]
      ),
   ])