#!/urs/bin/env python3 

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

import time
from math import atan2, acos, asin, cos, sin, sqrt


class ScaraTrayectory(Node):
    def __init__(self):
        super().__init__("scara_trayectory_node")
        self.lambda_ = 0
        topic_name = "/scara_trajectory_controller/joint_trajectory"
        self.trayectory_publisher_ = self.create_publisher(JointTrajectory, 
                                                           topic_name, 10)
        self.joints_ = ['link_1_joint', 'link_2_joint', 'link_3_joint']
        self.timer_ = self.create_timer(0.5, self.timer_callback)
        self.get_logger().info('Robot ejecutando trayectoria')

    def timer_callback(self):     
        trajectory_msg = JointTrajectory()
        trajectory_msg.joint_names = self.joints_
        point = JointTrajectoryPoint()
        x_in =0.6
        x_fin = 0.6
        y_in = 0.6
        theta_P_in = 0
        y_fin = -0.6
        theta_P_fin = 0

        solution = ink_sol(self.lambda_, x_in, y_in, theta_P_in, x_fin, y_fin,theta_P_fin)

        point.positions = solution
        point.time_from_start = Duration(sec=1)

        trajectory_msg.points.append(point)
        self.trayectory_publisher_.publish(trajectory_msg)
        self.get_logger().info('Ejecucion de trayectoria {}'.format(solution))
        time.sleep(2)

        self.lambda_ += 1


def ink_sol(param,x_in, y_in, theta_P_in, x_fin, y_fin, theta_P_fin):
    if param <= 10:
        L_3 = 0.3
        L_1 = 0.5
        L_2 = 0.5

        x_P = x_in + (param)/10 *(x_fin - x_in)
        y_P = y_in + (param)/10 *(y_fin - y_in)
        theta_P = theta_P_in + (param)/10 * (theta_P_fin - theta_P_in)
        
        x_3 = x_P - L_3*cos(theta_P)
        y_3 = y_P - L_3*sin(theta_P)

        link_2_joint = acos((pow(x_3, 2) + pow(y_3, 2) - pow(L_1, 2) - pow(L_2, 2))/(2*L_1*L_2))

        beta = atan2(y_3, x_3)
        psi = acos((pow(x_3, 2) + pow(y_3, 2) + pow(L_1, 2) - pow(L_2, 2))/(2*L_1*sqrt(pow(x_3, 2) + pow(y_3, 2))))

        link_1_joint = beta - psi

        link_3_joint = theta_P - link_1_joint - link_2_joint

        return [link_1_joint, link_2_joint, link_3_joint]

    elif param > 10:
        link_1_joint = 0
        link_2_joint = 0
        link_3_joint = 0
        return [link_1_joint, link_2_joint, link_3_joint]




def main(args=None):
    rclpy.init(args=args)
    node = ScaraTrayectory()
    rclpy.spin(node)
    rclpy.shutdown()



if __name__ == "__main__":
    main()