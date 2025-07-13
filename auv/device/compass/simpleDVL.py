#!/usr/bin/env python3
import rospy
import numpy as np
from geometry_msgs.msg import TwistStamped, PoseStamped
from sensor_msgs.msg import Imu
from transforms3d.euler import euler2mat
import threading


class SimpleIntegrator:
    def __init__(self):
        rospy.init_node("simple_dvl_integrator")
        self.pose_pub = rospy.Publisher("/auv/state/pose", PoseStamped, queue_size=10)

        self.lock = threading.Lock()
        self.last_time = None
        self.position = np.zeros((3, 1))
        self.velocity_body = np.zeros((3, 1))
        self.imu_acc_data   = {"ax": 0, "ay": 0, "az": 0}
        self.imu_ori_data   = {"yaw": 0, "pitch": 0, "roll": 0}  # store one line of IMU data for ekf predict

        rospy.Subscriber("/auv/devices/dvl/velocity", TwistStamped, self.dvl_callback)
        rospy.Subscriber("/auv/devices/vectornav", Imu, self.imu_callback)

        self.rate = rospy.Rate(50)
        self.run()

    def imu_callback(self,msg):
        # (self.imu_data["ax"], self.imu_data["ay"], self.imu_data["az"]) = quat2euler(orientation_list)
        self.imu_acc_data["ax"] = msg.linear_acceleration.x
        self.imu_acc_data["ay"] = msg.linear_acceleration.y
        self.imu_acc_data["az"] = msg.linear_acceleration.z

        self.imu_ori_data['roll'] = msg.orientation.x
        self.imu_ori_data['pitch'] = (msg.orientation.y + 180) % 360
        self.imu_ori_data['yaw'] = msg.orientation.z

    def dvl_callback(self, msg):
        with self.lock:
            self.velocity_body = np.array([
                [msg.twist.linear.x],
                [msg.twist.linear.y],
                [msg.twist.linear.z]
            ])

    def run(self):
        while not rospy.is_shutdown():
            with self.lock:
                current_time = rospy.Time.now()
                if self.last_time is None:
                    self.last_time = current_time
                    self.rate.sleep()
                    continue

                dt = (current_time - self.last_time).to_sec()
                self.last_time = current_time

                yaw = np.deg2rad(self.imu_ori_data['yaw'])
                pitch = np.deg2rad(self.imu_ori_data['pitch'])
                roll = np.deg2rad(self.imu_ori_data['roll'])

            rot_matrix = euler2mat(ai=yaw, aj=pitch, ak=roll, axes='szyx')  # Body-to-world rotation
            # Convert DVL velocity to world frame
            velocity_world = rot_matrix @ self.velocity_body

            # Integrate position: p += v * dt
            self.position += velocity_world * dt

            # Publish as PoseStamped
            pose_msg = PoseStamped()
            pose_msg.header.stamp = current_time
            pose_msg.header.frame_id = "base_link"

            pose_msg.pose.position.x = self.position[0, 0]
            pose_msg.pose.position.y = self.position[1, 0]
            pose_msg.pose.position.z = self.position[2, 0]

            # Orientation (not critical for position-only integration)
            pose_msg.pose.orientation.w = 1.0  # Identity quaternion

            self.pose_pub.publish(pose_msg)

            self.rate.sleep()


if __name__ == "__main__":
    try:
        SimpleIntegrator()
    except rospy.ROSInterruptException:
        pass
