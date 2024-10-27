#!/usr/bin/env python3

"""
This program is free software: you can redistribute it and/or modify it 
under the terms of the GNU General Public License as published by the Free Software Foundation, 
either version 3 of the License, or (at your option) any later version.

This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; 
without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. 
See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with this program. 
If not, see <https://www.gnu.org/licenses/>.

created by Thanacha Choopojcharoen at CoXsys Robotics (2022)
"""

from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro    
import os
    
def generate_launch_description():
    
    pkg = get_package_share_directory('robotics_model_3dof')
    rviz_path = os.path.join(pkg,'config','eff.target.display.rviz')
    rviz1 = Node(
        package='rviz2',
        namespace = 'robot1',
        executable='rviz2',
        name='rviz',
        arguments=['-d', rviz_path],
        output='screen')
    
    rviz2 = Node(
        package='rviz2',
        namespace = 'robot2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', rviz_path],
        output='screen')
        
    path_description = os.path.join(pkg,'robot','visual','TeeTyBot.xacro')
    # path_description = os.path.join(pkg,'robot','visual','my-robot.xacro')
    robot_desc_xml = xacro.process_file(path_description).toxml()
    #robot_desc_xml = xacro.process_file(path_description,mappings={'robot_name': namespace}).toxml()
    
    parameters = [{'robot_description':robot_desc_xml}]
    #parameters.append({'frame_prefix':namespace+'/'})
    
    robot_state_publisher1 = Node(package='robot_state_publisher',
                                  namespace = 'robot1',
                                  executable='robot_state_publisher',
                                  output='screen',
                                  parameters=parameters
    )

    robot_state_publisher2 = Node(package='robot_state_publisher',
                                  namespace = 'robot2',
                                  executable='robot_state_publisher',
                                  output='screen',
                                  parameters=parameters
    )
    
    launch_description = LaunchDescription()
    
    launch_description.add_action(rviz1)
    launch_description.add_action(rviz2)
    launch_description.add_action(robot_state_publisher1)
    launch_description.add_action(robot_state_publisher2)
    
    Kinematics_Kp = LaunchConfiguration('Kinematics_Kp')
    Kinematics = DeclareLaunchArgument(
        'Kinematics_Kp',
        default_value = '1.0'
    )
    
    launch_description.add_action(Kinematics)
    
    
    package_name = 'robotics_model_3dof'
    executable_name = ['target_randomizer', 'kinematics', 'robot_controller', 'robot_scheduler']
    joint_name = ['joint_1', 'joint_2', 'joint_3']
    namespace = ['robot1', 'robot2']
    for j in range(len(namespace)):
        for i in range(len(executable_name)):
            if executable_name[i] == 'target_randomizer':
                node = Node(
                package = package_name,
                namespace = namespace[j],
                executable = executable_name[i] + '.py',
                name = executable_name[i],
                parameters=[
                { 'frequency': 100.0 },
                { 'L_Base_F1': 0.2 },
                { 'L_F2_F3': 0.25 },
                { 'L_F3_Fe': 0.28 }
                ]
            )
            elif executable_name[i] == 'kinematics':
                node = Node(
                package = package_name,
                namespace = namespace[j],
                executable = executable_name[i] + '.py',
                name = executable_name[i],
                parameters=[
                {'singularity_thres': 0.01},
                { 'frequency': 100.0 },
                { 'Kp': Kinematics_Kp }
                ]
            )
            elif executable_name[i] == 'robot_controller':
                node = Node(
                package = package_name,
                namespace = namespace[j],
                executable = executable_name[i] + '.py',
                name = executable_name[i],
                parameters=[
                { 'frequency': 100.0 },
                {'joint_name': joint_name}
                ]
            )
            elif executable_name[i] == 'robot_scheduler':
                node = Node(
                package = package_name,
                namespace = namespace[j],
                executable = executable_name[i] + '.py',
                name = executable_name[i],
                parameters=[
                { 'frequency': 100.0 },
                { 'L_Base_F1': 0.2 },
                { 'L_F2_F3': 0.25 },
                { 'L_F3_Fe': 0.28 }
                ]
            )
            else:
                node = Node(
                package = package_name,
                namespace = '',
                executable = executable_name[i] + '.py',
                name = executable_name[i],
            )
            launch_description.add_action(node)
    
    return launch_description