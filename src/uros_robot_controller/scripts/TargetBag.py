#!/usr/bin/python3
import numpy as np
import rclpy
import yaml

from rclpy.node import Node
from std_msgs.msg import Header
from std_srvs.srv import Trigger
from geometry_msgs.msg import PoseStamped
from robotic_interfaces.srv import RandomTarget, Keyboard

class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')
        
        self.declare_parameter('file_name', 'Path.yaml')
        self.file_name = self.get_parameter('file_name').get_parameter_value().string_value
        
        self.create_service(Trigger, "SavePath", self.savepath_callback)
        self.create_service(RandomTarget, 'rand_target', self.rand_target_callback)
        
        self.keyboard_call = self.create_client(Keyboard, "/Mode")
        
        self.target_pub = self.create_publisher(PoseStamped, 'target', 10)
        
        self.create_subscription(PoseStamped, 'end_effector', self.Eff_callback, 10)

        self.eff_pose = [0.0, 0.0, 0.0]
        self.msg = PoseStamped()
        self.key = Keyboard.Request()
        
        self.file_path = 'Saved_Paths/' + self.file_name
        
        self.createYAML()
        
    def createYAML(self):
        empty_data = {}
        with open(self.file_path, 'w') as file:
            yaml.dump(empty_data, file)
            
    def loadYAML(self):
        with open(self.file_path, 'r') as file:
            data = yaml.safe_load(file) or {}
            return data
        
    def writeYAML(self, value):
        with open(self.file_path, 'w') as file:
            yaml.dump(value, file)
        
    def savepath_callback(self, request: Trigger, response: Trigger):
        self.get_logger().info(f"Target at: {self.eff_pose}")
        data = self.loadYAML()
        
        if not isinstance(data, list):
            data = []
        
        data.append({
            'x': self.eff_pose[0],
            'y': self.eff_pose[1],
            'z': self.eff_pose[2]
        })
        
        self.writeYAML(data)
        return response
    
    def rand_target_callback(self, request: RandomTarget, response: RandomTarget):
        srv = request.data
        if srv:
            data = self.loadYAML()
            
            if not isinstance(data, list) or not data:
                x = 0.1
                y = 0.2
                z = 0.0
                self.key.mode = "TOB"
                self.keyboard_call.call_async(self.key)
            
            else:
                x = data[0]['x']
                y = data[0]['y']
                z = data[0]['z']
                data.pop(0)
            
            self.writeYAML(data)
                                
            self.msg.header = Header()
            self.msg.header.stamp = self.get_clock().now().to_msg()
            
            self.msg.header.frame_id = 'link_0'
            self.msg.pose.position.x = x
            self.msg.pose.position.y = y
            self.msg.pose.position.z = z
            
            self.target_pub.publish(self.msg)
            
            response.x_target = x
            response.y_target = y
            response.z_target = z
            response.success = True
        return response
    
    def Eff_callback(self, msg: PoseStamped):
        self.eff_pose[0] = msg.pose.position.x
        self.eff_pose[1] = msg.pose.position.y
        self.eff_pose[2] = msg.pose.position.z

    # def timer_callback(self):
            
    #     self.joint_state_msg.header.stamp = self.get_clock().now().to_msg()

    #     self.joint_state_msg.velocity = [self.q_velocities[0], self.q_velocities[1], self.q_velocities[2]]
        
    #     for i in range(len(self.joint_state_msg.name)):
                
    #         self.joint_state_msg.position[i] += self.joint_state_msg.velocity[i] * 1/self.frequency
    #         self.joint_state_msg.position[i] %= self.position_limits[i]
            
    #     self.joint_pub.publish(self.joint_state_msg)

            
def main(args=None):
    rclpy.init(args=args)
    node = RobotController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
