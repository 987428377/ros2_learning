from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Twist

import rclpy
from rclpy.node import Node

from turtlesim.msg import Pose
from turtlesim.srv import Spawn


class PointPublisher(Node):

    def __init__(self):
        super().__init__('turtle_tf2_message_broadcaster')

        # Create a client to spawn a turtle
        self.spawner = self.create_client(Spawn, 'spawn')
        # Boolean values to store the information
        # if the service for spawning turtle is available
        self.turtle_spawning_service_ready = False
        # if the turtle was successfully spawned
        self.turtle_spawned = False
        # if the topics of turtle3 can be subscribed
        self.turtle_pose_cansubscribe = False

        self.timer = self.create_timer(1.0, self.on_timer)

#首先，在on_timer回调函数中，我们通过异步调用turtlesim的Spawn服务来生成turtle3，并在海龟生成服务准备就绪时将其位置初始化为(4,2,0)。
    def on_timer(self):
        if self.turtle_spawning_service_ready:
            if self.turtle_spawned:
                self.turtle_pose_cansubscribe = True
            else:
                if self.result.done():
                    self.get_logger().info(
                        f'Successfully spawned {self.result.result().name}')
                    self.turtle_spawned = True
                else:
                    self.get_logger().info('Spawn is not finished')
        else:
            if self.spawner.service_is_ready():
                # Initialize request with turtle name and coordinates
                # Note that x, y and theta are defined as floats in turtlesim/srv/Spawn
                request = Spawn.Request()
                request.name = 'turtle3'
                request.x = 4.0
                request.y = 2.0
                request.theta = 0.0
                # Call request
                self.result = self.spawner.call_async(request)
                self.turtle_spawning_service_ready = True
            else:
                # Check if the service is ready
                self.get_logger().info('Service is not ready')
#随后，节点发布主题turtle3/cmd_vel、主题turtle3/turtle_point_stamped，并订阅主题turtle3/pose，并对每条传入消息运行回调函数handle_turtle_pose。
        if self.turtle_pose_cansubscribe:
            self.vel_pub = self.create_publisher(Twist, 'turtle3/cmd_vel', 10)
            self.sub = self.create_subscription(Pose, 'turtle3/pose', self.handle_turtle_pose, 10)
            self.pub = self.create_publisher(PointStamped, 'turtle3/turtle_point_stamped', 10)
#最后，在回调函数handle_turtle_pose中，我们初始化turtle3的Twist消息并发布它们，这将使turtle3沿着圆周运动。
#然后我们用传入的 Pose 消息填充turtle3 的 PointStamped 消息并发布它们。
    def handle_turtle_pose(self, msg):
        vel_msg = Twist()
        vel_msg.linear.x = 1.0
        vel_msg.angular.z = 1.0
        self.vel_pub.publish(vel_msg)

        ps = PointStamped()
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.header.frame_id = 'world'
        ps.point.x = msg.x
        ps.point.y = msg.y
        ps.point.z = 0.0
        self.pub.publish(ps)


def main():
    rclpy.init()
    node = PointPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()