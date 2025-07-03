import rospy
import pandas as pd
import time
from auv.utils.SimEKF import SensorSimulator
from sensor_msgs.msg import Imu

class fakeIMU:
    def __init__(self):
        rospy.init_node('fakeIMU', anonymous=True)
        self.pub = rospy.Publisher('/auv/device/imu', Imu, queue_size=10)
        self.rate = rospy.Rate(40) # 40 Hz

        self.simulator = SensorSimulator(
            side_length=5.0,
            imu_rate=100,  # 100 Hz
            dvl_rate=10    # 10 Hz
        )
        self.imu_data, _ = self.simulator.generate_square_path()
        self.imu_df = pd.DataFrame(self.imu_data).sort_values('Time')

    def FakePublish(self):
        while not rospy.is_shutdown():
            for _, data in self.imu_df.iterrows():
                imu_msg = Imu()
                imu_msg.header.stamp = rospy.Time.now()
                imu_msg.header.frame_id = "base_link"

                # Angular velocity (rad/s)
                imu_msg.angular_velocity.x = 0
                imu_msg.angular_velocity.y = 0
                imu_msg.angular_velocity.z = 0

                # Linear acceleration (m/s^2)
                imu_msg.linear_acceleration.x = data['linear_acceleration.x']
                imu_msg.linear_acceleration.y = data['linear_acceleration.y']
                imu_msg.linear_acceleration.z = data['linear_acceleration.z']

                # Orientation (optional; in quaternion)
                imu_msg.orientation.x = 0.0
                imu_msg.orientation.y = 0.0
                imu_msg.orientation.z = 0.0
                imu_msg.orientation.w = 0.0

                # Covariances (optional; set to -1 or proper values)
                imu_msg.orientation_covariance[0] = -1  # if orientation is unknown

                self.pub.publish(imu_msg)
                self.rate.sleep()

if __name__=="__main__":
    node = fakeIMU()
    time.sleep(2)
    print("[INFO] up and running")
    node.FakePublish()