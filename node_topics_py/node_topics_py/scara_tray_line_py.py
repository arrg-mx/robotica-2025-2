#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

import time
from math import cos, sin, acos, asin, atan2, sqrt

class ScaraTrayLineNode(Node):
    def __init__(self):
        super().__init__("scara_tray_line_node")
        topic_name = "/scara_trajectory_controller/joint_trajectory"
        self.joints_ = ['link_1_joint', 'link_2_joint', 'link_3_joint']
        self.lamda_ = 0
        self.Tiempo_ejec_ = 10

        self.scara_tray_pub_ = self.create_publisher(
            JointTrajectory, topic_name, 10)
        self.tray_timer_ = self.create_timer(
            1, self.trayectory_cbck)
        self.get_logger().info(
            'Scara activo, trayectoria linea recta')

    def trayectory_cbck(self):
        trayectory_msg = JointTrajectory()
        trayectory_msg.joint_names = self.joints_
        point = JointTrajectoryPoint()

        if self.lamda_ <= self.Tiempo_ejec_:
            x_1 = 0.1
            y_1 = 0.6
            theta_1 = 0
            x_2 = 0.3
            y_2 = -0.6
            theta_2 = 1.57
            solucion = invk_sol(self.lamda_, x_1, y_1,theta_1, x_2, y_2,theta_2)
            point.positions = solucion
            point.time_from_start = Duration(sec=1)
            trayectory_msg.points.append(point)
            self.scara_tray_pub_.publish(trayectory_msg)
            self.get_logger().info("Postura actual {}".format(solucion))
            time.sleep(2)
            self.lamda_ += 1
        elif self.lamda_ > 10:
            link_1_joint = 0
            link_2_joint = 0
            link_3_joint = 0
            return [link_1_joint, link_2_joint, link_3_joint]


def invk_sol(param,x_in, y_in, theta_in, x_fin, y_fin, theta_fin):
    Tiempo_ejec_ = 10
    L_1 = 0.5
    L_2 = 0.5
    L_3 = 0.3

    x_P = x_in + (param/Tiempo_ejec_)*(x_fin - x_in)
    y_P = y_in + (param/Tiempo_ejec_)*(y_fin - y_in)
    theta_P = theta_in + (param/Tiempo_ejec_)*(theta_fin - theta_in)

    x_3 = x_P - L_3*cos(theta_P)
    y_3 = y_P - L_3*sin(theta_P)

    theta_2 = acos((pow(x_3, 2)+pow(y_3,2)-pow(L_1, 2)-pow(L_2, 2))/(2*L_1*L_2))
    
    beta = atan2(y_3, x_3)

    psi = acos((pow(x_3, 2)+pow(y_3,2)+pow(L_1, 2)-pow(L_2, 2))/(2*L_1*sqrt(pow(x_3, 2)+pow(y_3,2))))

    theta_1 = beta - psi

    theta_3 = theta_P -theta_1 -theta_2

    return [theta_1, theta_2, theta_3]




def main(args=None):
    rclpy.init(args=args)
    node = ScaraTrayLineNode()
    rclpy.spin(node)
    rclpy.shutdown()



if __name__ == "__main__":
    main()