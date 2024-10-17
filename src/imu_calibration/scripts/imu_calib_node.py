#!/usr/bin/python3
import os
import yaml
import rclpy
import numpy as np

from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.node import Node
from scipy.constants import g
from sensor_msgs.msg import Imu

class IMUNode(Node):
    def __init__(self):
        super().__init__('imu_calib_node')
        
        self.declare_parameter('file_name', 'imu_calibration.yaml')
        self.declare_parameter('frequency', 100.0)
        
        self.file_name = self.get_parameter('file_name').get_parameter_value().string_value
        self.frequency = self.get_parameter('frequency').get_parameter_value().double_value

        qosProfile = QoSProfile(
            reliability = ReliabilityPolicy.BEST_EFFORT,
            depth = 10
        )
        self.create_subscription(Imu, 'mpu6050_publisher', self.imu_callback, qosProfile)

        self.imu_n = 0
        self.imu_n_max = 10000
        self.accl_list = []
        self.gyro_list = []
        
        self.file_path = 'imu_config/' + self.file_name
        
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
  
    def save_calibration(self, mean, covr, name: str):
        value = self.loadYAML()
        
        mean_list = mean.tolist()
        covr_list = covr.tolist()
        
        value[f'{name} offset'] = mean_list
        value[f'{name} covariance'] = covr_list
        
        self.writeYAML(value)
        
    def imu_callback(self, msg: Imu):
        if self.imu_n < self.imu_n_max:
            
            self.imu_n += 1
            
            self.accl_list.append([
                msg.linear_acceleration.x,
                msg.linear_acceleration.y,
                msg.linear_acceleration.z - g
            ])
            
            self.gyro_list.append([
                msg.angular_velocity.x,
                msg.angular_velocity.y,
                msg.angular_velocity.z
            ])
            print('Collect Data: ' + str(self.imu_n))
        else:
            accl_array = np.array(self.accl_list)
            accl_covr = np.absolute(np.cov(accl_array.T)) 
            accl_offset = np.mean(accl_array, 0)
            
            gyro_array = np.array(self.gyro_list)
            gyro_covr = np.absolute(np.cov(gyro_array.T))
            gyro_offset = np.mean(gyro_array, 0)

            self.save_calibration(accl_offset, accl_covr, 'accl')
            self.save_calibration(gyro_offset, gyro_covr, 'gyro')
            print('========================')
            print('Finished Collecting Data')
            print('========================')
            print(accl_offset)
            print(accl_covr)
            print('========================')
            print(gyro_offset)
            print(gyro_covr)
            print('Exiting Program...')
            exit()
            
def main(args=None):
    rclpy.init(args=args)
    node = IMUNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
