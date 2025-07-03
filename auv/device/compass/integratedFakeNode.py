import rospy
import pandas as pd
import time
import threading
from geometry_msgs.msg import TwistStamped  # <- fixed import
from auv.utils.SimEKF import SensorSimulator
from sensor_msgs.msg import Imu

class fakeIntegrated:
    def __init__(self):
        rospy.init_node('fakeIntegrated', anonymous=True)
        self.imu_pub = rospy.Publisher('/auv/device/imu', Imu, queue_size=10)
        self.dvl_pub = rospy.Publisher('/auv/device/dvl', TwistStamped, queue_size=10)

        self.imu_rate = rospy.Rate(40)  # 40 Hz
        self.dvl_rate = rospy.Rate(10)  # 10 Hz

        self.simulator = SensorSimulator(
            side_length=5.0,
            imu_rate=40,
            dvl_rate=10
        )
        
        self.imu_data, self.dvl_data = self.simulator.generate_square_path()
        
        self.imu_df = pd.DataFrame(self.imu_data).sort_values('Time')
        self.dvl_df = pd.DataFrame(self.dvl_data).sort_values('time')

    def imuThread(self):
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

                    self.imu_pub.publish(imu_msg)
                    self.rate.sleep()
        except rospy.ROSInterruptException:
            rospy.loginfo("ROS node shutdown requested.")
        except KeyboardInterrupt:
            rospy.loginfo("KeyboardInterrupt received. Exiting gracefully.")
        finally:
            rospy.signal_shutdown("Shutting down fakeIMU node")

    def dvlThread(self):
        try:
            while not rospy.is_shutdown():
                for _, data in self.dvl_df.iterrows():
                    if rospy.is_shutdown():
                        break

                    msg = TwistStamped()
                    msg.header.stamp = rospy.Time.now()
                    msg.header.frame_id = "base_link"  # or "dvl_link", "odom", etc.

                    msg.twist.linear.x = data['vx']
                    msg.twist.linear.y = data['vy']
                    msg.twist.linear.z = data['vz']

                    msg.twist.angular.x = 0.0
                    msg.twist.angular.y = 0.0
                    msg.twist.angular.z = 0.0

                    self.dvl_pub.publish(msg)
                    self.rate.sleep()
        except rospy.ROSInterruptException:
            rospy.loginfo("ROS shutdown requested.")
        except KeyboardInterrupt:
            rospy.loginfo("KeyboardInterrupt received. Exiting gracefully.")
        finally:
            rospy.signal_shutdown("Shutting down fakeDVL node")
    
    def FakePublish(self):
        imuthread = threading.Thread(target=self.imuThread, daemon=True)
        dvlthread = threading.Thread(target=self.dvlThread, daemon=True)

        time.sleep(2)

        rospy.loginfo("[INFO] fake information publishing")

        imuthread.start()
        dvlthread.start()

if __name__=="__main__":
    node = fakeIntegrated()
    time.sleep(2)
    rospy.loginfo("[INFO] Fake node initialized")
    node.FakePublish()
