from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import Imu
from transforms3d.euler import euler2mat
import rospy
import numpy as np


               
class process:
    def __init__(self):
        rospy.init_node('rotatedDVL', anonymous=True)

        self.dvl_pub = rospy.Publisher('/auv/devices/dvl/velrot', TwistStamped, queue_size=10)
        self.dvl_sub = rospy.Subscriber("/auv/devices/dvl/velocity",TwistStamped, self.DVLcallback)
        self.imu_sub = rospy.Subscriber("/auv/devices/vectornav", Imu, self.IMUcallback)
        self.imu_acc_data   = {"ax": 0, "ay": 0, "az": 0}
        self.imu_ori_data   = {"yaw": 0, "pitch": 0, "roll": 0}  # store one line of IMU data for ekf predict

    def DVLcallback(self,vel):
        yaw = self.imu_ori_data['yaw']
        pitch = self.imu_ori_data['pitch']
        roll = self.imu_ori_data['roll']
        rot_Matrix = euler2mat(ai=yaw, aj=pitch , ak=roll, axes='szyx')
        R_yaw = np.array([
        [np.cos(yaw), -np.sin(yaw), 0],
        [np.sin(yaw),  np.cos(yaw), 0],
        [0,            0,           1]
        ])
        vel_body = np.array([vel.twist.linear.x,vel.twist.linear.y,vel.twist.linear.z])
        vel_world = R_yaw @ vel_body

        msg = TwistStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "base_link"  # or "dvl_link", "odom", etc.

        msg.twist.linear.x = vel_world[0]
        msg.twist.linear.y = vel_world[1]
        msg.twist.linear.z = vel_world[2]

        msg.twist.angular.x = 0.0
        msg.twist.angular.y = 0.0
        msg.twist.angular.z = 0.0

        self.dvl_pub.publish(msg)

    def IMUcallback(self,msg):
                # (self.imu_data["ax"], self.imu_data["ay"], self.imu_data["az"]) = quat2euler(orientation_list)
        self.imu_acc_data["ax"] = msg.linear_acceleration.x
        self.imu_acc_data["ay"] = msg.linear_acceleration.y
        self.imu_acc_data["az"] = msg.linear_acceleration.z

        self.imu_ori_data['roll'] = msg.orientation.x
        self.imu_ori_data['pitch'] = (msg.orientation.y + 180) % 360
        self.imu_ori_data['yaw'] = msg.orientation.z

if __name__=="__main__":
    object = process()
    rospy.spin()