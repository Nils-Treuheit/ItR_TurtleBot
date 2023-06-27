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
        self.current_rot = 0
        self.goal_rot = None
        self.back_up = False
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
        x = None
        rot = 0
        dis_front = self.forward_distance - 0.3
        dis_back = self.backward_distance - 0.3

        if self.goal is not None and self.position is not None and self.prev_position is not None:
            goal_pos = np.array([self.goal[0],self.goal[1]])
            current_pos = np.array([(self.position[0]+self.prev_position[0])*0.5,(self.position[1]+self.prev_position[1])*0.5])
            future_pos = np.array([current_pos[0] + DIST2FUT * math.cos(self.current_rot),
                                   current_pos[1] + DIST2FUT * math.sin(self.current_rot)])

            future_angle = math.atan2(future_pos[1]-current_pos[1],future_pos[0]-current_pos[0])
            goal_angle = math.atan2(goal_pos[1]-current_pos[1],goal_pos[0]-current_pos[0])

            rot = max(min((goal_angle - future_angle), ROT_LIM), -ROT_LIM)
            x = np.sqrt(np.sum((future_pos-current_pos)**2))

        if x == None: x = dis_front
        x = min(x,dis_front) if x > 0 else max(x,dis_back)
        if x == 0.0: 
            if self.right_distance > 0.15:
                self.goal_rot = self.current_rot+np.pi*0.5
                self.back_up = False
            elif self.left_distance > 0.15:
                self.goal_rot = self.current_rot-np.pi*0.5
                self.back_up = False
            else: self.back_up = True

        if self.back_up: 
            x = dis_back
            self.back_up = False
            rot = 0
        elif self.goal_rot is not None and self.goal_rot!=self.current_rot:
            rot = self.goal_rot - self.current_rot 
            x = 0.0
        elif self.goal_rot is not None and self.goal_rot==self.current_rot:
            self.goal_rot = None
            
        x = min(max(x,-VEL_LIM),VEL_LIM)
        rot = min(max(rot,-ROT_LIM),ROT_LIM)
        self.current_rot += rot

        msg.linear.x = x
        if rot!=0: msg.angular.z = rot
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
