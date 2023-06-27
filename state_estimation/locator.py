import rclpy
import numpy as np
from rclpy.node import Node

from driving_swarm_messages.msg import Range
from geometry_msgs.msg import PointStamped
from copy import deepcopy


class LocatorNode(Node):

    x_estimate = np.zeros(3)

    def __init__(self):
        super().__init__('locator_node')
        self.anchor_ranges = []
        self.create_subscription(Range, 'range', self.range_cb, 10)
        self.position_pub = self.create_publisher(PointStamped, 'position', 10)
        self.initialized = False
        self.create_timer(1.0, self.timer_cb)
        self.get_logger().info('locator node started')
        
    def range_cb(self, msg):
        self.anchor_ranges.append(msg)
        self.anchor_ranges = self.anchor_ranges[-10:]
        if not self.initialized:
            self.initialized = True
            self.get_logger().info('first range received')

    def timer_cb(self):
        if not self.initialized:
            return
        msg = PointStamped()
        msg.point.x, msg.point.y, msg.point.z = self.calculate_position()
        msg.header.frame_id = 'world'
        self.position_pub.publish(msg)
    
    def calculate_position(self):
        if not len(self.anchor_ranges):
            return self.x_estimate
        
        # YOUR CODE GOES HERE:
        x_estimate,anchor_ranges = deepcopy(self.x_estimate),deepcopy(self.anchor_ranges)
        anchors = {"x"+str(elem.anchor.x)+"y"+str(elem.anchor.y)+"z"+str(elem.anchor.z):elem for elem in anchor_ranges}
        anchors = anchors.values()
        for _ in range(10):
            a = np.array([np.array([elem.anchor.x,elem.anchor.y,elem.anchor.z]) for elem in anchors])
            m = np.array([elem.range for elem in anchors])

            gradR = -np.array([(x_estimate-ai)/np.linalg.norm(x_estimate-ai) for ai in a])
            R = m-np.sqrt(np.sum((a - x_estimate)**2, axis=1))
            x_estimate = x_estimate - np.linalg.pinv(gradR) @ R
            
        self.x_estimate = x_estimate

        return x_estimate


def main(args=None):
    rclpy.init(args=args)

    node = LocatorNode()

    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
