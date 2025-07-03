import rospy
import pandas as pd
import time
from geometry_msgs import TwistStamped
from auv.utils.SimEKF import SensorSimulator


class fakeDVL:
    def __init__(self):
        # fake DVL core publishing fake dvl data repeatedly
        rospy.init_node('fakeDVL', anonymous=True)
        self.pub = rospy.Publisher('/auv/device/dvl', TwistStamped, queue_size=10)
        self.rate = rospy.Rate(10) # 10 Hz

        self.simulator = SensorSimulator(
            side_length=5.0,
            imu_rate=100,  # 100 Hz
            dvl_rate=10    # 10 Hz
        )
        _, self.dvl_data = self.simulator.generate_square_path()
        self.dvl_df = pd.DataFrame(self.dvl_data).sort_values('time')


    def FakePublish(self):
        while not rospy.is_shutdown():
            for _, data in self.dvl_df.iterrows():
                msg = TwistStamped()
                msg.header.stamp = rospy.Time.now()
                msg.header.frame_id = "base_link"  # or "dvl_link", "odom", etc.

                # Example velocity values
                msg.twist.linear.x = data['vx']
                msg.twist.linear.y = data['vy']
                msg.twist.linear.z = data['vz']

                msg.twist.angular.x = 0.0
                msg.twist.angular.y = 0.0
                msg.twist.angular.z = 0.0

                self.pub.publish(msg)

                self.rate.sleep()
    
if __name__=="__main__":
    node = fakeDVL()

    time.sleep(2)
    print("[INFO] up and running")
    node.FakePublish()