#!/usr/bin/env python3

"""
6‑state EKF for an AUV:
    x = [x, y, z, vx, vy, vz]ᵀ  (world frame, ENU)
Fuses:
    • IMU           – orientation & linear accel
    • DVL velocity  – body-frame velocity → rotated to world frame
    • Barometer     – depth (z)
"""

import numpy as np
from statistics import mean
from transforms3d.euler import euler2mat

class EKF6State:
    def __init__(self, dt):
        self.dt = dt

        # State vector: [x, y, z, vx, vy, vz]
        self.x = np.zeros((6, 1))

        # State covariance
        self.P = np.eye(6) * 0.1

        # Process noise
        self.Q = np.diag([0.01]*3 + [0.3]*3)

        # Measurement noise (DVL)
        self.R_dvl = np.eye(3) * 0.05

        # Measurement noise (barometer)
        self.R_baro = np.array([[0.05]])

    def predict(self):
        F = np.eye(6)
        F[0, 3] = self.dt
        F[1, 4] = self.dt
        F[2, 5] = self.dt

        self.x = F @ self.x
        self.P = F @ self.P @ F.T + self.Q

    def update_dvl(self, z):
        H = np.zeros((3, 6))
        H[0, 3] = 1
        H[1, 4] = 1
        H[2, 5] = 1

        y = z - H @ self.x
        S = H @ self.P @ H.T + self.R_dvl
        K = self.P @ H.T @ np.linalg.inv(S)

        self.x += K @ y
        self.P = (np.eye(6) - K @ H) @ self.P

    def update_depth(self, depth):
        H = np.zeros((1, 6))
        H[0, 2] = 1  # z position

        z = np.array([[depth]])
        y = z - H @ self.x
        S = H @ self.P @ H.T + self.R_baro
        K = self.P @ H.T @ np.linalg.inv(S)

        self.x += K @ y
        self.P = (np.eye(6) - K @ H) @ self.P


# ROS node wrapper
import rospy
import time
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TwistStamped, PoseStamped
from mavros_msgs.msg import Mavlink

class EKFNode:
    def __init__(self):
        rospy.init_node("ekf_6d_node")
        self.dt = 1.0 / 50.0  # 50 Hz
        self.ekf = EKF6State(self.dt)

        self.dvl_velocity = np.zeros((3, 1))
        self.depth = None
        self.depth_calib = 0
        self.calibrated = False

        self.imu_ori_data = {"yaw": 0, "pitch": 0, "roll": 0}

        rospy.Subscriber("/auv/devices/vectornav", Imu, self.imu_callback)
        rospy.Subscriber("/auv/devices/dvl/velocity", TwistStamped, self.dvl_callback)
        rospy.Subscriber("/mavlink/from", Mavlink, self.barometer_callback)

        self.pub = rospy.Publisher("/auv/state/pose", PoseStamped, queue_size=10)

        self.calibrate_depth()
        rospy.Timer(rospy.Duration(self.dt), self.ekf_step)

    def imu_callback(self, msg):
        # Note: orientation.x/y/z hold roll, pitch, yaw respectively (not quaternion)
        self.imu_ori_data['roll'] = msg.orientation.x
        self.imu_ori_data['pitch'] = (msg.orientation.y + 180) % 360
        self.imu_ori_data['yaw'] = msg.orientation.z

    def dvl_callback(self, msg):
        # Body to world-frame using RPY rotation matrix
        yaw = np.deg2rad(self.imu_ori_data['yaw'])
        pitch = np.deg2rad(self.imu_ori_data['pitch'])
        roll = np.deg2rad(self.imu_ori_data['roll'])

        rot_matrix = euler2mat(ai=yaw, aj=pitch, ak=roll, axes='szyx')

        self.dvl_velocity = rot_matrix @ np.array([
            [msg.twist.linear.x],
            [msg.twist.linear.y],
            [msg.twist.linear.z]
        ])

    def barometer_callback(self, msg):
        try:
            if msg.msgid == 143:
                from struct import pack, unpack
                p = pack("QQ", *msg.payload64)
                _, press_abs, _, _ = unpack("Iffhxx", p)
                self.depth = (press_abs / (997.0474 * 9.80665 * 0.01)) - self.depth_calib
        except:
            pass

    def calibrate_depth(self, sample_time=3):
        rospy.loginfo("Starting Depth Calibration...")
        samples = []

        while self.depth is None:
            rospy.sleep(0.1)

        prev = self.depth
        start_time = time.time()

        while time.time() - start_time < sample_time:
            if self.depth == prev:
                continue
            samples.append(self.depth)
            prev = self.depth

        self.depth_calib = mean(samples)
        self.calibrated = True
        rospy.loginfo(f"Depth calibration finished. Surface = {self.depth_calib:.2f} m")

    def ekf_step(self, event):
        self.ekf.predict()
        self.ekf.update_dvl(self.dvl_velocity)
        # if self.depth is not None and self.calibrated:
        #     self.ekf.update_depth(self.depth)
        self.publish_pose()

    def publish_pose(self):
        pose_msg = PoseStamped()
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.header.frame_id = "base_link"

        pose_msg.pose.position.x = self.ekf.x[0, 0]
        pose_msg.pose.position.y = self.ekf.x[1, 0]
        pose_msg.pose.position.z = self.ekf.x[2, 0]
        pose_msg.pose.orientation.w = 1  # dummy orientation

        self.pub.publish(pose_msg)

if __name__ == "__main__":
    rospy.sleep(2)
    rospy.loginfo("Running the simple EKF node")
    EKFNode()
    rospy.spin()