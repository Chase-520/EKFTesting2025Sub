import rospy
import pandas as pd
import time
import threading
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import Imu
from auv.utils.SimEKF import SensorSimulator


class fakeIntegrated:
    def __init__(self):
        rospy.init_node('fakeIntegrated', anonymous=True)

        self.imu_pub = rospy.Publisher('/auv/device/imu', Imu, queue_size=10)
        self.dvl_pub = rospy.Publisher('/auv/device/dvl', TwistStamped, queue_size=10)

        self.imu_rate = rospy.Rate(40)  # 40 Hz
        self.dvl_rate = rospy.Rate(10)  # 10 Hz

        self.running = True  # for thread control

        self.simulator = SensorSimulator(
            side_length=5.0,
            imu_rate=40,
            dvl_rate=10
        )

        self.imu_data, self.dvl_data = self.simulator.generate_square_path()
        self.imu_df = pd.DataFrame(self.imu_data).sort_values('Time')
        self.dvl_df = pd.DataFrame(self.dvl_data).sort_values('time')

    def imuThread(self):
        while self.running and not rospy.is_shutdown():
            for _, data in self.imu_df.iterrows():
                if not self.running or rospy.is_shutdown():
                    break

                imu_msg = Imu()
                imu_msg.header.stamp = rospy.Time.now()
                imu_msg.header.frame_id = "base_link"

                imu_msg.angular_velocity.x = 0
                imu_msg.angular_velocity.y = 0
                imu_msg.angular_velocity.z = 0

                imu_msg.linear_acceleration.x = data['linear_acceleration.x']
                imu_msg.linear_acceleration.y = data['linear_acceleration.y']
                imu_msg.linear_acceleration.z = data['linear_acceleration.z']

                imu_msg.orientation.x = 0.0
                imu_msg.orientation.y = 0.0
                imu_msg.orientation.z = 0.0
                imu_msg.orientation.w = 0.0
                imu_msg.orientation_covariance[0] = -1

                self.imu_pub.publish(imu_msg)
                self.imu_rate.sleep()

    def dvlThread(self):
        while self.running and not rospy.is_shutdown():
            for _, data in self.dvl_df.iterrows():
                if not self.running or rospy.is_shutdown():
                    break

                msg = TwistStamped()
                msg.header.stamp = rospy.Time.now()
                msg.header.frame_id = "base_link"

                msg.twist.linear.x = data['vx']
                msg.twist.linear.y = data['vy']
                msg.twist.linear.z = data['vz']

                msg.twist.angular.x = 0.0
                msg.twist.angular.y = 0.0
                msg.twist.angular.z = 0.0

                self.dvl_pub.publish(msg)
                self.dvl_rate.sleep()

    def FakePublish(self):
        imu_thread = threading.Thread(target=self.imuThread)
        dvl_thread = threading.Thread(target=self.dvlThread)

        imu_thread.start()
        dvl_thread.start()

        rospy.loginfo("[INFO] fakeIntegrated is publishing IMU and DVL data")

        try:
            rospy.spin()
        except KeyboardInterrupt:
            rospy.loginfo("Shutdown requested")

        self.running = False
        imu_thread.join()
        dvl_thread.join()
        rospy.loginfo("fakeIntegrated shut down cleanly")


if __name__ == "__main__":
    node = fakeIntegrated()
    time.sleep(1)
    rospy.loginfo("[INFO] Fake node initialized")
    node.FakePublish()
