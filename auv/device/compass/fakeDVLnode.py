import rospy
import pandas as pd
import time
from geometry_msgs.msg import TwistStamped  # <- fixed import
from auv.utils.SimEKF import SensorSimulator


class fakeDVL:
    def __init__(self):
        rospy.init_node('fakeDVL', anonymous=True)
        self.pub = rospy.Publisher('/auv/device/dvl', TwistStamped, queue_size=10)
        self.rate = rospy.Rate(10)  # 10 Hz

        self.simulator = SensorSimulator(
            side_length=5.0,
            imu_rate=100,  # 100 Hz
            dvl_rate=10    # 10 Hz
        )
        _, self.dvl_data = self.simulator.generate_square_path()
        self.dvl_df = pd.DataFrame(self.dvl_data).sort_values('time')

    def FakePublish(self):
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

                    self.pub.publish(msg)
                    self.rate.sleep()
        except rospy.ROSInterruptException:
            rospy.loginfo("ROS shutdown requested.")
        except KeyboardInterrupt:
            rospy.loginfo("KeyboardInterrupt received. Exiting gracefully.")
        finally:
            rospy.signal_shutdown("Shutting down fakeDVL node")

if __name__ == "__main__":
    node = fakeDVL()
    time.sleep(2)
    rospy.loginfo("[INFO] fakeDVL node up and running")
    node.FakePublish()
