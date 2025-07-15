#!/usr/bin/env python3
# simple_ekf_node.py
"""
6‑state EKF for an AUV:
    x = [x, y, z, vx, vy, vz]ᵀ  (world frame, ENU)
Fuses:
    • IMU           – orientation & linear accel
    • DVL velocity  – body-frame velocity → rotated to world frame
    • Barometer     – depth (z)
"""

import numpy as np
import rospy, time
from statistics import mean

from geometry_msgs.msg import TwistStamped, PoseStamped
from sensor_msgs.msg import Imu
from mavros_msgs.msg import Mavlink
from tf.transformations import quaternion_matrix

GRAV = 9.80665            # m s⁻²  (positive down if ENU)

# ══════════════════════════════════════════════════════════════════════════════
class EKF6State:
    """Pure EKF math (no ROS)."""
    def __init__(self,
                 q_proc=np.diag([0.01]*3 + [0.3]*3),
                 r_dvl =np.eye(3)*0.05,
                 r_baro=np.array([[0.05]])):
        self.x = np.zeros((6,1))     # state
        self.P = np.eye(6)*0.1       # covariance

        self.Qc = q_proc             # continuous‑time process noise
        self.R_dvl  = r_dvl
        self.R_baro = r_baro

    # -------------------------------------------------------------------------
    def predict(self, dt, accel_n=np.zeros((3,1))):
        """Constant‑acceleration model driven by world‑frame accel."""
        F = np.eye(6)
        F[0,3] = F[1,4] = F[2,5] = dt

        B = np.zeros((6,3))
        B[0:3,:] = 0.5*dt*dt*np.eye(3)
        B[3:6,:] = dt*np.eye(3)

        self.x = F @ self.x + B @ accel_n
        self.P = F @ self.P @ F.T + self.Qc*dt

    # -------------------------------------------------------------------------
    def update_dvl(self, z):
        """z = 3×1 world‑frame velocity measurement"""
        if not np.isfinite(z).all():          # NaN guard
            return
        H = np.zeros((3,6)); H[:,3:6] = np.eye(3)
        S = H @ self.P @ H.T + self.R_dvl
        K = self.P @ H.T @ np.linalg.solve(S, np.eye(3))
        y = z - H @ self.x
        self.x += K @ y
        self.P  = (np.eye(6) - K @ H) @ self.P

    # -------------------------------------------------------------------------
    def update_depth(self, depth):
        if not np.isfinite(depth):
            return
        H = np.zeros((1,6)); H[0,2] = 1.0
        S = H @ self.P @ H.T + self.R_baro
        K = self.P @ H.T / S          # scalar divide
        y = np.array([[depth]]) - H @ self.x
        self.x += K @ y
        self.P  = (np.eye(6) - K @ H) @ self.P

# ══════════════════════════════════════════════════════════════════════════════
class SimpleEKFNode:
    def __init__(self):
        rospy.init_node("simple_ekf_node")

        # ── parameters ────────────────────────────────────────────────────────
        self.topic_imu   = rospy.get_param("~topic_imu",  "/auv/devices/vectornav/cor")
        self.topic_dvl   = rospy.get_param("~topic_dvl",  "/auv/devices/dvl/velocity")
        self.topic_baro  = rospy.get_param("~topic_baro", "/mavlink/from")
        self.pub_pose_t  = rospy.get_param("~pub_pose",   "/auv/state/pose")
        self.pub_twist_t = rospy.get_param("~pub_twist",  "/auv/state/velocity")
        self.loop_hz     = rospy.get_param("~rate", 50.0)

        # noise params (all in SI)
        q_pos   = rospy.get_param("~q_pos",   0.01)
        q_vel   = rospy.get_param("~q_vel",   0.3)
        r_dvl   = rospy.get_param("~r_dvl",   0.05)
        r_baro  = rospy.get_param("~r_baro",  0.05)

        Qc = np.diag([q_pos]*3 + [q_vel]*3)
        self.ekf = EKF6State(Qc,
                             np.eye(3)*r_dvl,
                             np.array([[r_baro]]))

        # ── state for callbacks ───────────────────────────────────────────────
        self.q_body_to_world = np.array([0,0,0,1], dtype=float)  # (x,y,z,w)
        self.last_accel_b    = np.zeros((3,1))
        self.last_dvl_world  = np.zeros((3,1))
        self.last_dvl_stamp  = rospy.Time(0)

        self.depth       = None
        self.depth_zero  = 0.0
        self.depth_ready = False

        # ── ROS I/O ───────────────────────────────────────────────────────────
        self.sub_imu  = rospy.Subscriber(self.topic_imu,  Imu,          self.cb_imu,  queue_size=20)
        self.sub_dvl  = rospy.Subscriber(self.topic_dvl,  TwistStamped, self.cb_dvl,  queue_size=10)
        self.sub_baro = rospy.Subscriber(self.topic_baro, Mavlink,      self.cb_baro, queue_size=10)

        self.pub_pose  = rospy.Publisher(self.pub_pose_t,  PoseStamped,  queue_size=10)
        self.pub_twist = rospy.Publisher(self.pub_twist_t, TwistStamped, queue_size=10)

        # start timers
        self.last_time = rospy.Time.now()
        rospy.Timer(rospy.Duration(1.0/self.loop_hz), self.timer_step)
        rospy.Timer(rospy.Duration(0.1), self._depth_zero_calib, oneshot=True)

    # ───────────────────────────── callbacks ─────────────────────────────────
    def cb_imu(self, msg: Imu):
        # store quaternion (geometry_msgs uses (x,y,z,w))
        self.q_body_to_world = np.array([msg.orientation.x,
                                         msg.orientation.y,
                                         msg.orientation.z,
                                         msg.orientation.w], dtype=float)
        # body-frame accel
        self.last_accel_b = np.array([[msg.linear_acceleration.x],
                                      [msg.linear_acceleration.y],
                                      [msg.linear_acceleration.z]])

    def cb_dvl(self, msg: TwistStamped):
        # rotate body‑frame velocity to world frame
        R = quaternion_matrix(self.q_body_to_world)[:3,:3]
        body_v = np.array([[msg.twist.linear.x],
                           [msg.twist.linear.y],
                           [msg.twist.linear.z]])
        self.last_dvl_world = R @ body_v
        self.last_dvl_stamp = msg.header.stamp

    def cb_baro(self, msg: Mavlink):
        if msg.msgid != 143:   # SCALED_PRESSURE
            return
        from struct import pack, unpack
        p = pack("QQ", *msg.payload64)
        _, press_abs, _, _ = unpack("Iffhxx", p)
        self.depth = press_abs/(997.0474*GRAV*0.01) - self.depth_zero

    # ───────────────────────────── main loop ────────────────────────────────
    def timer_step(self, _evt):
        now = rospy.Time.now()
        dt  = (now - self.last_time).to_sec()
        self.last_time = now
        if not 0.0 < dt < 1.0:
            dt = 1.0/self.loop_hz

        # ---- prediction -----------------------------------------------------
        # rotate accel to world frame & subtract gravity
        R = quaternion_matrix(self.q_body_to_world)[:3,:3]
        accel_n = R @ self.last_accel_b
        accel_n[2] += GRAV      # ENU: gravity is negative Z

        self.ekf.predict(dt, accel_n)

        # ---- DVL update (only if fresh) -------------------------------------
        if (now - self.last_dvl_stamp) < rospy.Duration(3.0/self.loop_hz):
            self.ekf.update_dvl(self.last_dvl_world)

        # ---- depth update ----------------------------------------------------
        if self.depth_ready and self.depth is not None:
            self.ekf.update_depth(self.depth)

        # ---- publish ---------------------------------------------------------
        self.publish(now)

    # ───────────────────────────── publish helpers ───────────────────────────
    def publish(self, stamp):
        pose = PoseStamped()
        pose.header.stamp = stamp
        pose.header.frame_id = "base_link"
        pose.pose.orientation.w = 1.0
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = self.ekf.x[0:3,0]
        self.pub_pose.publish(pose)

        twist = TwistStamped()
        twist.header.stamp = stamp
        twist.header.frame_id = "base_llink"
        twist.twist.linear.x, twist.twist.linear.y, twist.twist.linear.z = self.ekf.x[3:6,0]
        self.pub_twist.publish(twist)

    # ───────────────────────── depth zeroing once ────────────────────────────
    def _depth_zero_calib(self, _):
        rospy.loginfo("Depth zeroing…")
        samples, t0 = [], time.time()
        while len(samples) < 30 and not rospy.is_shutdown():
            if self.depth is not None:
                samples.append(self.depth)
            rospy.sleep(0.05)
            if time.time() - t0 > 5.0:
                break
        if samples:
            self.depth_zero  = mean(samples)
            self.depth_ready = True
            rospy.loginfo(f"Depth zero set to {self.depth_zero:.3f} m")
        else:
            rospy.logwarn("Depth zeroing failed")

# ══════════════════════════════════════════════════════════════════════════════
if __name__ == "__main__":
    rospy.loginfo("Launching SimpleEKF node")
    SimpleEKFNode()
    rospy.spin()