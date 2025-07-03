import rospy
import pandas as pd
import time
from auv.utils.SimEKF import SensorSimulator
from sensor_msgs.msg import Imu

class fakeIMU:
    def __init__(self):
        rospy.init_node('fakeIMU', anonymous=True)
        self.pub = rospy.Publisher('/auv/device/imu', Imu, queue_size=10)
        self.rate = rospy.Rate(40)  # 40 Hz

        self.simulator = SensorSimulator(
            side_length=5.0,
            imu_rate=100,
            dvl_rate=10
        )
        self.imu_data, _ = self.simulator.generate_square_path()
        self.imu_df = pd.DataFrame(self.imu_data).sort_values('Time')

    def FakePublish(self):
        try:
            while not rospy.is_shutdown():
                for _, data in self.imu_df.iterrows():
                    if rospy.is_shutdown():
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

                    self.pub.publish(imu_msg)
                    self.rate.sleep()
        except rospy.ROSInterruptException:
            rospy.loginfo("ROS node shutdown requested.")
        except KeyboardInterrupt:
            rospy.loginfo("KeyboardInterrupt received. Exiting gracefully.")
        finally:
            rospy.signal_shutdown("Shutting down fakeIMU node")

if __name__ == "__main__":
    node = fakeIMU()
    time.sleep(2)
    rospy.loginfo("[INFO] fakeIMU node up and running")
    node.FakePublish()
