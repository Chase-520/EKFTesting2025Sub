import numpy as np
from filterpy.kalman import ExtendedKalmanFilter
from filterpy.common import Q_discrete_white_noise
from auv.utils import deviceHelper
from auv.device.dvl import DVL
import time
import threading
import rospy
from transforms3d.euler import quat2euler, euler2mat
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import PoseStamped


class SensorFuse:
    def __init__(self):
        # This is the ekf node that takes DVL and imu data and give estimation of velocity
        # Initialize node
        rospy.init_node('ekfNode', anonymous=True)
        self.pub = rospy.Publisher('/auv/state/pose', PoseStamped, queue_size=10)
        self.rate = rospy.Rate(10)  # 10 Hz
        

        # Create subscriber for imu and dvl
        self.imu_sub = rospy.Subscriber("auv/devices/vectornav", Imu, self.imu_callback)
        self.imu_data = {"ax": 0, "ay": 0, "az": 0}  # store one line of IMU data for ekf predict
        self.imu_ori_data   = {"yaw": 0, "pitch": 0, "roll": 0}  # store one line of IMU data for ekf predict
        self.imu_array = None # used for passing into the ekf

        self.dvl_sub = rospy.Subscriber("auv/devices/dvl/velocity", TwistStamped, self.dvl_callback)
        self.dvl_data = {"vx": 0, "vy": 0, "vz": 0}
        self.dvl_array = None # used for passing into the ekf
        # initialize filter, dvl, imu, dt, and last_time
        # initialize dt before creating the filter
        self.dt = 1.0 / 40  # IMU time step (40 Hz)
        self.ekf = self.create_filter()
        self.imu = {"ax": 0, "ay": 0, "az": 0}
        # tracks the cumulative position
        self.position = np.array([0.0, 0.0, 0.0])
        self.last_time = time.time()

    def imu_callback(self,msg):
        self.imu_data["ax"] = msg.linear_acceleration.x
        self.imu_data["ay"] = msg.linear_acceleration.y
        self.imu_data["az"] = msg.linear_acceleration.z
        
        self.imu_ori_data['roll'] = msg.orientation.x
        self.imu_ori_data['pitch'] = msg.orientation.y
        self.imu_ori_data['yaw'] = msg.orientation.z
        # Store body-frame acceleration
        accel_body = np.array([msg.linear_acceleration.x,
                            msg.linear_acceleration.y,
                            msg.linear_acceleration.z])
        # Get rotation matrix from IMU YPR
        rot_matrix = euler2mat(ai=self.imu_ori_data['yaw'], aj=self.imu_ori_data['pitch'], ak=self.imu_ori_data['roll'], axes='szyx')  # Body-to-world rotation

        # Rotate acceleration to world frame
        accel_world = rot_matrix @ accel_body
        
        self.imu_array = accel_world.reshape(-1, 1)  # Store as column vector
        
        # update state
        self.update_state()

    def dvl_callback(self,msg):
        try:
            self.dvl_data["vx"] = msg.twist.linear.x
            self.dvl_data["vy"] = msg.twist.linear.y
            self.dvl_data["vz"] = msg.twist.linear.z
            
            # Get rotation matrix from IMU quaternion
            rot_matrix = euler2mat(ai=self.imu_ori_data['yaw'], aj=self.imu_ori_data['pitch'], ak=self.imu_ori_data['roll'], axes='szyx')  # Body-to-world rotation

            # Convert DVL velocities to numpy array and rotate
            v_body = np.array([self.dvl_data["vx"],
                            self.dvl_data["vy"],
                            self.dvl_data["vz"]])
            v_global = rot_matrix @ v_body  # (3,3) @ (3,) -> (3,)

            # Store rotated velocity for EKF update
            self.dvl_array = v_global.reshape(-1, 1)  # Convert to column vector        # update filter
            self.update_filter()
        except Exception as e:
            rospy.logerr(f"DVL callback error: {str(e)}")

    def update_state(self):
        # Calculate time delta
        current_time = time.time()
        dt = current_time - self.last_time
        self.last_time = current_time

        # Update the state transition matrix F with the new dt
        self.ekf.F = self.FJacobian_at(self.ekf.x, dt)

        # Update the state with IMU data
        self.ekf.x[3:] = self.imu_array

        # Predict the next state
        self.ekf.predict()

        # Integrate position and publish it
        self.position += dt * np.array(self.ekf.x[0:3])
        self.publish()

    def update_filter(self):
        # Update the filter with the latest DVL measurements
        self.ekf.update(self.dvl_array, self.HJacobian_at, self.hx)   

    def publish(self):
        pose_msg = PoseStamped()
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.header.frame_id = "base_link"  # or "odom", "base_link", etc.

        pose_msg.pose.position.x = self.position[0]
        pose_msg.pose.position.y = self.position[1]
        pose_msg.pose.position.z = self.position[2]
        
        pose_msg.pose.orientation.w = 0.0
        pose_msg.pose.orientation.x = self.imu_ori_data['roll']
        pose_msg.pose.orientation.y = self.imu_ori_data['pitch']
        pose_msg.pose.orientation.z = self.imu_ori_data['yaw']

        self.pub.publish(pose_msg)
    
    @staticmethod
    def f(x, dt):
        # Non-linear state transition matrix
        # predicts the next state based on the current state and time step
        x_new = np.copy(x)
        x_new[0] += x[3] * dt  # vel_x update
        x_new[1] += x[4] * dt  # vel_y update
        x_new[2] += x[5] * dt  # vel_z update
        return x_new

    @staticmethod
    def FJacobian_at(x, dt):
        # Linearizes f(x)
        return np.array([
            [1., 0., 0., dt, 0., 0.],  # dvl_vel_x depends on vx and ax
            [0., 1., 0., 0., dt, 0.],  # dvl_vel_y depends on vy and ay
            [0., 0., 1., 0., 0., dt],  # dvl_vel_z depends on vz and az
            [0., 0., 0., 1., 0., 0.],  # imu_accel_x depends on ax
            [0., 0., 0., 0., 1., 0.],  # imu_accel_y depends on ay
            [0., 0., 0., 0., 0., 1.]   # imu_accel_z depends on az
        ])

    @staticmethod
    def hx(x):
        # non-linear measurement matrix
        return np.array([x[0], x[1], x[2]])  # extracting velocity from the state vector

    @staticmethod
    def HJacobian_at(x):
        # Define the Jacobian matrix for the measurement function
        H = np.zeros((3, 6))
        H[0, 0] = 1
        H[1, 1] = 1
        H[2, 2] = 1
        return H

    # ------------- Extended Kalman Filter ----------------
    def create_filter(self) -> ExtendedKalmanFilter:
        # and 3 measurements (vel_x, vel_y, vel_z)
        ekf = ExtendedKalmanFilter(dim_x=6, dim_z=3, dim_u=0)

        # Init everything to 0
        # These are the sensor measurements
        ekf.x = np.array([0., 0., 0., 0., 0., 0.])

        # Create non-linear state transition matrix
        # Each row corresponds to a measurement from sensors
        # Each column indicates the dependence of the measurement on a sensor measurement
        # ie. Since Vx depends on vx dvl and ax imu we put 1's in the vx dvl and ax imu cols
        # 1's for acceleration are placeholder for dt which you'll see in the predict class
        ekf.f = self.f
        ekf.F = self.FJacobian_at(ekf.x, self.dt)

        # Create the non-linear measurement matrix
        # Each row corresponds to a predicted val so vx vy vz
        # Each col corresponds to the sensor vals
        # Convert the predicted state estimate into predicted sensor values so just pulling out the vel values
        ekf.hx = self.hx
        ekf.H = self.HJacobian_at(ekf.x)

        # Covariance matrix (P): initial uncertainty in the state
        # Covariance matrix uncertainty will change over time to be more certain
        ekf.P = np.eye(6) * 1000  # High uncertainty at the start

        # Create the process noise covariance matrix
        # model noise so that the predict model can account for it (can be tuned thru trial and error)
        # Large Q means trusting actual sensor observations more than predicted measurements
        ekf.Q = np.eye(6)

        # Create the measurement noise matrix
        # uncertainty in the predicted vals (can be tuned)
        ekf.R = np.array([
            [0.1, 0., 0.],  # vel_x
            [0., 0.1, 0.],  # vel_y
            [0., 0., 0.1]   # vel_z
        ])


        return ekf


if __name__=="__main__":
    ekf = SensorFuse()
    time.sleep(2)
    rospy.loginfo("ekf node running")
    rospy.spin()