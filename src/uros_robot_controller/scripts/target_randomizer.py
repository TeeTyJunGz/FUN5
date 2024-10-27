#!/usr/bin/python3
import rclpy
import random

from rclpy.node import Node
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped
from rcl_interfaces.msg import SetParametersResult
from robotic_interfaces.srv import RandomTarget

class TargetRandomizer(Node):
    def __init__(self):
        super().__init__('target_randomizer')
        
        self.create_service(RandomTarget, 'rand_target', self.rand_target_callback)
        self.target_pub = self.create_publisher(PoseStamped, 'target', 10)
        
        self.declare_parameter('frequency', 100.0)
        self.declare_parameter('L_Base_F1', 0.2)
        self.declare_parameter('L_F2_F3', 0.25)
        self.declare_parameter('L_F3_Fe', 0.28)

        self.frequency = self.get_parameter('frequency').get_parameter_value().double_value
        self.Z_offset = self.get_parameter('L_Base_F1').get_parameter_value().double_value
        self.L1 = self.get_parameter('L_F2_F3').get_parameter_value().double_value
        self.L2 = self.get_parameter('L_F3_Fe').get_parameter_value().double_value
        
        self.msg = PoseStamped()
        self.target = []

        self.add_on_set_parameters_callback(self.set_param_callback)
    
    def set_param_callback(self, params):
        for param in params:
            if param.name == 'frequency':
                self.get_logger().info(f'Updated frequency: {param.value}')
                self.frequency = param.value
            elif param.name == 'L_Base_F1':
                self.get_logger().info(f'Updated Z_offset: {param.value}')
                self.Z_offset = param.value
            elif param.name == 'L_F2_F3':
                self.get_logger().info(f'Updated L1: {param.value}')
                self.L1 = param.value
            elif param.name == 'L_F3_Fe':
                self.get_logger().info(f'Updated L2: {param.value}')
                self.L2 = param.value
            else:
                self.get_logger().warn(f'Unknown parameter: {param.name}')
                # Return failure result for unknown parameters
                return SetParametersResult(successful=False, reason=f'Unknown parameter: {param.name}')
        # If all parameters are known, return success
        return SetParametersResult(successful=True)
    
    def rand_target_callback(self, request: RandomTarget, response: RandomTarget):
        srv = request.data
        if srv:
            
            r_max = self.L1 + self.L2
            r_min = abs(self.L1 - self.L2)
            
            while True:
                x = random.uniform(-r_max, r_max)
                y = random.uniform(-r_max, r_max)
                z = random.uniform(-r_max, r_max)
                
                distance_squared = x**2 + y**2 + (z - self.Z_offset)**2
                
                if r_min**2 <= distance_squared <= r_max**2 and distance_squared != 0.0:
                    
                    self.target = [x, y, z]
                    
                    self.msg.header = Header()
                    self.msg.header.stamp = self.get_clock().now().to_msg()
                    
                    self.msg.header.frame_id = 'link_0'
                    self.msg.pose.position.x = x
                    self.msg.pose.position.y = y
                    self.msg.pose.position.z = z
                    
                    response.x_target = x
                    response.y_target = y
                    response.z_target = z
                    response.success = True
                    
                    self.target_pub.publish(self.msg)
                    break
        else:
            response.success = False
            
        return response
            
def main(args=None):
    rclpy.init(args=args)
    node = TargetRandomizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
