#!/usr/bin/python3
import rclpy
import numpy as np
import roboticstoolbox as rtb

from math import pi
from tf2_ros import TransformListener, Buffer
from rclpy.node import Node
from spatialmath import SE3
from std_msgs.msg import Bool, Header
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from rcl_interfaces.msg import SetParametersResult
from robotic_interfaces.srv import RandomTarget, StateScheduler


#OLD MDH
L1 = 0.200
L2 = 0.120
L3 = 0.100
L4 = 0.250
L5 = 0.280

robot = rtb.DHRobot(
    [
        rtb.RevoluteMDH(alpha = 0.0     ,a = 0.0      ,d = L1     ,offset = 0.0),
        rtb.RevoluteMDH(alpha = -pi/2   ,a = 0.0      ,d = -L2    ,offset = -pi/2),
        rtb.RevoluteMDH(alpha = 0.0     ,a = L4       ,d = L3     ,offset = 0.0),
    ],tool = SE3([
    [0, 0, 1, L5],
    [1, 0, 0, 0],
    [0, 1, 0, 0],
    [0, 0, 0, 1]]),
    name = "3DOF_Robot"
)
# robot = rtb.DHRobot(
#         [   
#         rtb.RevoluteMDH(a=0.0,       alpha=0.0,     offset=0.0,     d=0.2   ),
#         rtb.RevoluteMDH(a=0.0,       alpha=pi/2,    offset=0.0,     d=0.02  ),
#         rtb.RevoluteMDH(a=0.25,      alpha=0.0,     offset=0.0,     d=0.0  )
#         ], tool=SE3.Tx(0.28),
#         name="3DOF_Robot"
#     )

class Kinematics(Node):
    def __init__(self):
        super().__init__('kinematics_calculator')
        
        self.create_service(StateScheduler, "state_sch", self.call_state_callback)
        self.call_random = self.create_client(RandomTarget, "rand_target")

        self.create_subscription(PoseStamped, 'IPK_target', self.target_callback, 10)
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)

        self.kinematics_Ready_State = self.create_publisher(Bool, 'kinematics_Ready_State', 10)
        self.end_effector = self.create_publisher(PoseStamped, 'end_effector', 10)
        self.q_pub = self.create_publisher(JointState, "q_velocities", 10)
        
        self.declare_parameter('singularity_thres', 0.01)
        self.declare_parameter('frequency', 100.0)
        self.declare_parameter('Kp', 1.0)

        self.singularity_thres = self.get_parameter('singularity_thres').get_parameter_value().double_value
        self.frequency = self.get_parameter('frequency').get_parameter_value().double_value
        self.Kp = self.get_parameter('Kp').get_parameter_value().double_value
        
        self.eff_msg = PoseStamped()
        self.eff_msg.header = Header()
        self.eff_msg.header.frame_id = 'link_0'
                
        self.cmd_vel= np.array([0.0, 0.0, 0.0])
        self.target = np.array([0.0, 0.0, 0.0])
        self.target_rc = False
        self.q = [0.0, 0.0, 0.0]
        
        self.q_velocities = JointState()
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_eff = np.array([0.0, 0.0, 0.0])
        self.tf_position = [0.0, 0.0, 0.0]
        self.target_frame = "end_effector"
        self.source_frame = "link_0"
        
        self.state_srv = "Initial"

        self.create_timer(1/self.frequency, self.timer_callback)
        self.add_on_set_parameters_callback(self.set_param_callback)
    
    def set_param_callback(self, params):
        for param in params:
            if param.name == 'singularity_thres':
                self.get_logger().info(f'Updated singularity_threshold: {param.value}')
                self.singularity_thres = param.value
            elif param.name == 'frequency':
                self.get_logger().info(f'Updated frequency: {param.value}')
                self.frequency = param.value
            elif param.name == 'Kp':
                self.get_logger().info(f'Updated Kp: {param.value}')
                self.Kp = param.value
            else:
                self.get_logger().warn(f'Unknown parameter: {param.name}')
                # Return failure result for unknown parameters
                return SetParametersResult(successful=False, reason=f'Unknown parameter: {param.name}')
        # If all parameters are known, return success
        return SetParametersResult(successful=True)
    
    def call_random_function(self, boolean):
        rand = RandomTarget.Request()
        rand.data = boolean
            
        future = self.call_random.call_async(rand)
        
        future.add_done_callback(self.handle_random_target_response)
        
    def handle_random_target_response(self, future):
        try:
            response = future.result()
            if response:
                x = response.x_target
                y = response.y_target
                z = response.z_target
                self.target = np.array([x, y, z])
                self.target_rc = True

            else:
                self.get_logger().error("Received an empty response from the service.")
        except Exception as e:
            self.get_logger().error(f"Service call failed with error: {str(e)}")
            
    def call_state_callback(self, request: StateScheduler, response: StateScheduler):
        
        srv = request.state
        if srv == "Auto":
            self.call_random_function(True)
            self.state_srv = srv
            
            response.success = True
        
        elif srv == "Teleop Based" or srv == "Teleop End Effector":
            self.state_srv = srv
            self.target_rc = True

            response.success = True
        elif srv == "IPK":
            self.state_srv = srv
            self.target_rc = True
            
            response.success = True
        else:
            self.target_rc = False
            response.success = False
            
        return response
    
    def target_callback(self, msg: PoseStamped):
        x = msg.pose.position.x
        y = msg.pose.position.y
        z = msg.pose.position.z

        self.target = np.array([x, y, z])
        self.target_rc = True
        
    def cmd_vel_callback(self, msg: Twist):
        self.cmd_vel = np.array([msg.linear.x, msg.linear.y, msg.linear.z])
        
    def timer_callback(self):
        msg = Bool()
        self.eff_msg.header.stamp = self.get_clock().now().to_msg()
        
        if self.target_rc:
            current_pose = robot.fkine(self.q).t[:3]
            # current_pose = self.tf_position

            error = self.target - current_pose


            if self.state_srv == "Auto" or self.state_srv == "IPK":
                v_end_effector = self.Kp * error
                
            elif self.state_srv == "Teleop Based":
                v_end_effector = self.cmd_vel
                
            elif self.state_srv == "Teleop End Effector":
                T_e = robot.fkine(self.q)
                R_e = T_e.R
                v_end_effector = self.cmd_vel @ R_e
                        
            J_full = robot.jacob0(self.q)
            J_translational = J_full[:3, :3]  # 3x3 matrix
            
            q_dot = np.linalg.pinv(J_translational).dot(v_end_effector)
                        
            self.q += q_dot * 1/self.frequency
            self.q_velocities.velocity = [q_dot[0], q_dot[1], q_dot[2]]
            
            manipulability = np.linalg.det(J_full @ J_full.T)
            
            if manipulability > self.singularity_thres:
                self.q_velocities.velocity = [0.0, 0.0, 0.0]
                self.get_logger().info(f"Near a singularity")
            
            
            if np.linalg.norm(error) < 1e-3 and (self.state_srv != "Teleop Based" or self.state_srv != "Teleop End Effector"):
                self.q_velocities.velocity = [0.0, 0.0, 0.0]
                self.target_rc = False
                
                msg.data = True

                self.get_logger().info(f"Target pose reached! At x: {current_pose[0]}, y: {current_pose[1]}, z: {current_pose[2]}")
                
            self.eff_msg.pose.position.x, self.eff_msg.pose.position.y, self.eff_msg.pose.position.z = current_pose
            self.q_pub.publish(self.q_velocities)
            
        self.kinematics_Ready_State.publish(msg)
        self.end_effector.publish(self.eff_msg)
        
    def tf_echo(self):
        try:
            now = rclpy.time.Time()
            transform = self.tf_buffer.lookup_transform(
                "link_0",   
                "end_effector",   
                now)
            position = transform.transform.translation

            self.tf_position[0] = position.x
            self.tf_position[1] = position.y
            self.tf_position[2] = position.z
        
        except Exception as e:
            self.get_logger().error(f"Failed to get transform: {e}")
            
def main(args=None):
    rclpy.init(args=args)
    node = Kinematics()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
