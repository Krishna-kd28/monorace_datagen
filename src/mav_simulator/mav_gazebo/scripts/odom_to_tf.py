#!/usr/bin/env python3
"""
Publishes:
  - Static TF: world -> odom (identity).
  - Dynamic TF: odom -> child_frame from nav_msgs/Odometry (subscribes to 'odom').

Node namespace: if the node is launched with a namespace (e.g. namespace='X3'),
the odom frame becomes namespaced (e.g. 'X3/odom') so the tree is
world -> X3/odom -> X3/base_link. Topic is then /X3/odom via remapping or namespace.
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster


class OdomToTf(Node):
    def __init__(self):
        super().__init__('odom_to_tf')
        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)

        # Use node namespace for frame ids (e.g. /X3 -> odom frame "X3/odom")
        ns = self.get_namespace().strip('/')
        self.odom_frame_id = f"{ns}/odom" if ns else 'odom'
        self.world_frame_id = 'world'

        # Static transform: world -> odom (identity)
        world_to_odom = TransformStamped()
        world_to_odom.header.stamp = self.get_clock().now().to_msg()
        world_to_odom.header.frame_id = self.world_frame_id
        world_to_odom.child_frame_id = self.odom_frame_id
        world_to_odom.transform.translation.x = 0.0
        world_to_odom.transform.translation.y = 0.0
        world_to_odom.transform.translation.z = 0.0
        world_to_odom.transform.rotation.x = 0.0
        world_to_odom.transform.rotation.y = 0.0
        world_to_odom.transform.rotation.z = 0.0
        world_to_odom.transform.rotation.w = 1.0
        self.static_tf_broadcaster.sendTransform(world_to_odom)

        self.sub = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10
        )

    def odom_callback(self, msg):
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = self.odom_frame_id  # match static TF (namespaced if node has namespace)
        t.child_frame_id = msg.child_frame_id
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation = msg.pose.pose.orientation
        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = OdomToTf()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
