#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import Float64MultiArray
from roboticstoolbox import DHRobot, RevoluteMDH
from math import pi
from spatialmath import SE3
import numpy as np

class SimpleRobot(DHRobot):
    def __init__(self):
        deg = pi / 180

        L1 = RevoluteMDH(a=0.0, d=0.2, alpha=0.0, qlim=[-180 * deg, 180 * deg])
        L2 = RevoluteMDH(a=0.0, d=-0.06, alpha=-pi / 2, qlim=[-180 * deg, 65 * deg], offset=-pi/2)
        L3 = RevoluteMDH(a=0.25, d=0.04, alpha=0.0, qlim=[-180 * deg, 180 * deg])

        L = [L1, L2, L3]

        super().__init__(L, name="SimpleRobot", manufacturer="FIBO")

class DummyNode(Node):
    def __init__(self):
        super().__init__('dummy_node')
        self.eff_pub = self.create_publisher(PoseStamped, "eff", 10)
        self.velo_pub = self.create_publisher(Float64MultiArray, "velocity_controllers/commands", 10)
        
        # Subscribing to /cmd_vel
        self.create_subscription(Twist, "/dofdof/cmd_vel", self.cmd_callback, 10)
        
        # Setting up robot parameters
        self.dt = 0.01
        self.create_timer(self.dt, self.sim_loop)
        self.q = [0.0, 0.5, 1.4]
        self.cmd_vel = [0.0, 0.0, 0.0]
        self.name = ["joint_1", "joint_2", "joint_3"]
        self.my_robot = SimpleRobot()
        
        tool = SE3(0.28, 0.0, 0.0) * SE3.Rx(pi/2) * SE3.Ry(pi/2)
        self.my_robot.tool = tool

    def cmd_callback(self, msg: Twist):
        # Update cmd_vel based on incoming Twist message
        self.cmd_vel[0] = msg.linear.x
        self.cmd_vel[1] = msg.linear.y
        self.cmd_vel[2] = msg.linear.z
        
        print(self.cmd_vel)

    def send_vel(self, qd):
        msg = Float64MultiArray()
        msg.data = [qd[0], qd[1], qd[2]]
        self.velo_pub.publish(msg)

    def sim_loop(self):
        # Jacobian and inverse kinematics to calculate qd from cmd_vel
        J = self.my_robot.jacob0(self.q)
        j_reduce = J[:3, :3]
        qd = np.linalg.inv(j_reduce) @ np.array(self.cmd_vel)
        
        # Send the calculated joint velocities
        self.send_vel([qd[0], qd[1], qd[2]])
        
        # Publish end-effector position
        self.T0e = self.my_robot.fkine(self.q)
        eff_msg = PoseStamped()
        eff_msg.header.frame_id = "link_0"
        eff_msg.header.stamp = self.get_clock().now().to_msg()
        eff_msg.pose.position.x = self.T0e.t[0]
        eff_msg.pose.position.y = self.T0e.t[1]
        eff_msg.pose.position.z = self.T0e.t[2]
        self.eff_pub.publish(eff_msg)

def main(args=None):
    rclpy.init(args=args)
    node = DummyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()