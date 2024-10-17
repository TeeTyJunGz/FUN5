#!/usr/bin/python3
import os
import yaml
import rclpy
import numpy as np

from rclpy.node import Node
from imu_interfaces.srv import ImuCalibration
from rcl_interfaces.msg import SetParametersResult
from ament_index_python import get_package_share_directory

class ImuBridgeNode(Node):
    def __init__(self):
        super().__init__('imu_bridge_node')
        self.declare_parameter('file_name', 'imu_calibration.yaml')
        
        self.calib_client = self.create_client(ImuCalibration, 'mpu6050_calibration')
        
        while not self.calib_client.wait_for_service(timeout_sec=1.0):
            print("waiting.. for service")
            
        self.file_path = 'imu_config/' + self.file_name
        
        self.YAML_value = self.service_request_created()
        self.service_requester(self.YAML_value)

    def loadYAML(self):
        with open(self.file_path, 'r') as file:
            data = yaml.safe_load(file) or {}
            return data
        
    def service_request_created(self):
        value = self.loadYAML()
        
        request = ImuCalibration.Request()

        request.imu_calib.linear_acceleration_covariance = [
            item for sublist in value['accl covariance'] for item in sublist
        ]

        request.imu_calib.linear_acceleration.x = value['accl offset'][0]
        request.imu_calib.linear_acceleration.y = value['accl offset'][1]
        request.imu_calib.linear_acceleration.z = value['accl offset'][2]

        request.imu_calib.angular_velocity_covariance = [
            item for sublist in value['gyro covariance'] for item in sublist
        ]

        request.imu_calib.angular_velocity.x = value['gyro offset'][0]
        request.imu_calib.angular_velocity.y = value['gyro offset'][1]
        request.imu_calib.angular_velocity.z = value['gyro offset'][2]
        
        return request

    def service_requester(self, request):
        while True:
            future = self.calib_client.call_async(request)
            rclpy.spin_until_future_complete(self, future)

            result = ImuCalibration.Response()
            result = future.result()
            print(result.success)

            if result.success == True:
                exit()
                
def main(args=None):
    rclpy.init(args=args)
    node = ImuBridgeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()