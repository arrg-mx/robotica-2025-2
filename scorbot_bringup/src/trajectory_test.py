#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Duration
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from math import cos, sin, acos, asin, atan2, sqrt, pi

goal_workspace_pose = [0.3841, 0.2099, 0.2, 0, 1.5, 0.5]

class TrajectoryTest(Node):

    def __init__(self, joint_positions):
        super().__init__('trajectory_test')
        topic_name= "/scorbot_trajectory_controller/joint_trajectory"
        self.joint_positions = joint_positions
        self.trajectory_publisher = self.create_publisher(JointTrajectory, topic_name,10)
        self.timer = self.create_timer(1, self.timer_callback)
        self.joints = ['joint_1', 'joint_2', 'joint_3', 'joint_4']
        self.goal_positions = self.joint_positions
        self.get_logger().info('Controller is running and publishing to topic: {}'.format(topic_name))

    def timer_callback(self):
        trajectory_msg = JointTrajectory()
        trajectory_msg.joint_names = self.joints
        point = JointTrajectoryPoint()
        point.positions = self.goal_positions
        point.time_from_start = Duration(sec=2)
        trajectory_msg.points.append(point)
        self.trajectory_publisher.publish(trajectory_msg)
        

def dot_product(A, B):
    return A[0]*B[0]+A[1]*B[1]+A[2]*B[2]
        
def norm(A):
    return sqrt(dot_product(A, A))
     	
def transpose(A):
    return [[A[0][0],A[1][0],A[2][0]],
    	    [A[0][1],A[1][1],A[2][1]],
     	    [A[0][2],A[1][2],A[2][2]]]
     	
def rotation_matrix(psi,theta,phi):
    return [[cos(phi)*cos(theta), -sin(phi)*cos(psi) + cos(phi)*sin(theta)*sin(psi), sin(phi)*sin(psi) + cos(phi)*sin(theta)*cos(psi)],
	    [sin(phi)*cos(theta), cos(phi)*cos(psi) + sin(phi)*sin(theta)*sin(psi), -cos(phi)*sin(psi) + sin(phi)*sin(theta)*cos(psi)],
	    [-sin(theta), cos(theta)*sin(psi), cos(theta)*cos(psi)]]

    	
def matrix_product(A,B):
    if(type(B[0]) == list):
        return [[A[0][0]*B[0][0] + A[0][1]*B[1][0] + A[0][2]*B[2][0], A[0][0]*B[0][1] + A[0][1]*B[1][1] + A[0][2]*B[2][1], A[0][0]*B[0][2] + A[0][1]*B[1][2] + A[0][2]*B[2][2]],
     		[A[1][0]*B[0][0] + A[1][1]*B[1][0] + A[1][2]*B[2][0], A[1][0]*B[0][1] + A[1][1]*B[1][1] + A[1][2]*B[2][1], A[1][0]*B[0][2] + A[1][1]*B[1][2] + A[1][2]*B[2][2]],
     		[A[2][0]*B[0][0] + A[2][1]*B[1][0] + A[2][2]*B[2][0], A[2][0]*B[0][1] + A[2][1]*B[1][1] + A[2][2]*B[2][1], A[2][0]*B[0][2] + A[2][1]*B[1][2] + A[2][2]*B[2][2]]]
    else:
        return [A[0][0]*B[0]+A[0][1]*B[1]+A[0][2]*B[2],A[1][0]*B[0]+A[1][1]*B[1]+A[1][2]*B[2],A[2][0]*B[0]+A[2][1]*B[1]+A[2][2]*B[2]]
        
def vector_difference(a, b):
    return [a[0]-b[0], a[1]-b[1], a[2]-b[2]]
    
def scale_vector(c,a):
    return [c*a[0], c*a[1], c*a[2]]
     
def get_joint_positions(goal_pose):
    # Pose inverse kinematic function
    l = [0.362,0.22,0.22,0.1475]
    goal_position = [goal_pose[0], goal_pose[1], goal_pose[2]]
	
    z_1 = [0, 0, 1]
    z_3 = [0, 0, 1]
    R_0_p = rotation_matrix(goal_pose[3], goal_pose[4], goal_pose[5])
    z_p_0 = [R_0_p[0][2], R_0_p[1][2], R_0_p[2][2]]
    p_1_0 = [0, 0, l[0]]
    p_p_4 = [0,0,l[2]]

    joint_1 = atan2(goal_position[1],goal_position[0])

    R_joint_1 = rotation_matrix(0, 0, joint_1)
			       
    p_4_0 = vector_difference(goal_position, scale_vector(l[3], z_p_0))
    epsilon = atan2(sqrt(pow(p_4_0[0], 2) + pow(p_4_0[1], 2)), p_4_0[2] - l[0])
    
    joint_3 = acos((pow(p_4_0[0], 2) + pow(p_4_0[1], 2) + pow(p_4_0[2] - l[0], 2) - pow(l[1], 2) - pow(l[2], 2))/(2*l[1]*l[2]))
    
    alpha = atan2(l[2]*sin(joint_3), l[1]+l[2]*cos(joint_3))
    
    joint_2 = epsilon - alpha

    R_0_3 = rotation_matrix(0, joint_2 + joint_3, joint_1)
    R_3_4 = matrix_product(transpose(R_0_3), R_0_p)

    joint_4 = acos(R_3_4[0][0])
    
    return [joint_1, joint_2, joint_3, joint_4]

def main(args=None):
    rclpy.init(args=args)
    trajectory_publisher_node = TrajectoryTest(get_joint_positions(goal_workspace_pose))
    rclpy.spin(trajectory_publisher_node)
    trajectory_publisher_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()




