import rclpy
import numpy as np
from rclpy.node import Node
from copy import deepcopy
from geometry_msgs.msg import Twist, PoseStamped, PointStamped
from sensor_msgs.msg import LaserScan
import math 

# LIM in accordance to https://emanual.robotis.com/docs/en/platform/turtlebot3/features/
ROT_LIM = 1.42 #50% of max speed
VEL_LIM = 0.15 #75% of max speed
DIST2FUT = 0.45 #tripple VEL_LIM


class VelocityController(Node):
    
    def __init__(self):
        super().__init__('velocity_controller')
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.forward_distance = 0
        self.left_distance = 0
        self.right_distance = 0
        self.backward_distance = 0
        self.turning_rounds = 0
        self.rot = 0.0
        self.turns = 0
        self.prev_goal = None
        self.goal = None
        self.position = None
        self.prev_position = None
        self.create_subscription(LaserScan, 'scan', self.laser_cb, rclpy.qos.qos_profile_sensor_data)
        self.create_subscription(PoseStamped, 'nav/goal', self.goal_cb, 10)
        self.create_subscription(PointStamped, 'position', self.position_cb, 10)
        self.create_timer(0.1, self.timer_cb)
        self.get_logger().info('controller node started')
        
    def timer_cb(self):
        msg = Twist()

        if self.goal is None or self.position is None:
            return
        
        if self.prev_goal is None and self.goal is not None:
            self.prev_goal = self.goal

        x_diffrence = self.goal[0] - self.position[0]
        y_diffrence = self.goal[1] - self.position[1]

        x = 0.0
        z = 0.0

        #go to right x 
        if abs(x_diffrence) > 0.06 and self.turns%2 == 0:
            x_diffrence = x_diffrence if x_diffrence < 0.3 else 0.3
            x_diffrence = x_diffrence if x_diffrence > -0.3 else -0.3
            x = x_diffrence
        else:
            #first rotation
            if self.turns%2 == 0 and self.turning_rounds == 0:
                self.rot = -1.57
                self.turning_rounds = 9
            else:
                #go to right y
                if abs(y_diffrence) > 0.06:
                    y_diffrence = y_diffrence if y_diffrence < 0.3 else 0.3
                    y_diffrence = y_diffrence if y_diffrence > -0.3 else -0.3
                    x = -y_diffrence
                else:
                    #second rotation
                    if self.turning_rounds == 0:
                        self.rot = 1.57
                        self.turning_rounds = 9

        if self.prev_goal != self.goal:
            if self.turns%2!=0 and self.turning_rounds == 0:
                self.rot = 1.57
                self.turning_rounds = 9
            else:
                self.prev_goal = self.goal

        if self.turning_rounds > 0:
            z = self.rot
            x = 0.0
            self.turning_rounds -= 1
            if self.turning_rounds == 0:
                self.turns += 1
        
        msg.linear.x = x
        msg.angular.z = z
        self.publisher.publish(msg)


    
    def goal_cb(self, msg):
        goal = msg.pose.position.x, msg.pose.position.y
        if self.goal != goal:
            self.get_logger().info(f'received a new goal: (x={goal[0]}, y={goal[1]})')
            self.goal = goal
    
    def laser_cb(self, msg):
        self.forward_distance = np.min(np.concatenate([np.array(msg.ranges[:22]),np.array(msg.ranges[-22:])]))
        self.left_distance = np.min(np.array(msg.ranges[45:136]))
        self.right_distance = np.min(np.array(msg.ranges[225:315]))
        self.backward_distance = np.min(np.array(msg.ranges[158:203]))

        
    def position_cb(self, msg):
        if self.position is not None: self.prev_position = deepcopy(self.position)
        self.position = msg.point.x, msg.point.y



def main(args=None):
    rclpy.init(args=args)

    node = VelocityController()

    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
