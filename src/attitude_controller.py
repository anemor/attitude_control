import rospy

import numpy as np
import math                                             # accel and orientation calculations
from scipy.spatial.transform import Rotation

from nav_msgs.msg import RollPitchYawrateThrust         # input to rotors_control package
from trajectory_msgs.msg import MultiDOFJointTrajectory # reference trajectory (pos + yaw)
from nav_msgs.msg import Odometry                       # sensor data

from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3



K_POS = np.array([1.0, 1.0, 1.0]) #p,i,d
K_VEL = np.array([1.0, 1.0, 1.0])

class AttitudeController():
    def __init__(self) -> None:
        # initialize node
        rospy.init_node("attitude_controller_node")
        node_rate = rospy.get_param("~node_rate")
        self.dt = 1.0 / node_rate
        self.rate = rospy.Rate(node_rate)

        # initialize variables
        

        # publishers
        self.rates_thrust_pub = rospy.Publisher('rmf_obelix/command/roll_pitch_yawrate_thrust', RollPitchYawrateThrust, queue_size=1)

        # subscribers
        self.ref_waypoints_sub = rospy.Subscriber('rmf_obelix/command/trajectory', MultiDOFJointTrajectory, self._ref_waypoints_cb, queue_size=1)
        self.pose_sub = rospy.Subscriber('rmf_obelix/odometry_sensor1/odometry', Odometry, self._pose_cb)
    
    # CALLBACKS
    def _ref_wayppints_cb(self, msg : MultiDOFJointTrajectory) -> None:
        # receive 10 waypoints at a time, start at nr0
        self.waypoints = msg.points
        self.next_wp_n = 0

    def _pose_cb(self, msg : Odometry) -> None:
        self.pos = msg.pose.pose.position
        self.vel = msg.twist.twist.linear
        self.att = msg.pose.pose.orientation
        # timestamp ???        

        #if self.next_wp < len(wps):

    # Calculations from paper 
    def _calculate_acceleration_ref(pos : Point, pos_ref : Point, vel : Vector3) ->  np.ndarray:
        p = np.array([pos.x, pos.y, pos.z])
        p_ref = np.array([pos_ref.x, pos_ref.y, pos_ref.z])
        v = np.array([vel.x, vel.y, vel.z])

        a_fb = -K_POS*(p - p_ref) + -K_VEL*v
        a_ref = a_fb + np.array([0.0, 0.0, 9.81])
        
        return a_ref

    def _calculate_orientation_ref(yaw : float, a_ref : np.ndarray) -> np.ndarray:
        x_c = np.array([math.cos(yaw), math.sin(yaw), 0.0])
        y_c = np.array([-math.sin(yaw), math.cos(yaw), 0.0])

        z_B_ref = a_ref / np.linalg.norm(a_ref)
        x_B_ref = np.cross(y_c, z_B_ref) / np.linalg.norm(np.cross(y_c, z_B_ref))
        y_B_ref = np.cross(z_B_ref, x_B_ref)
        R_ref = np.array([x_B_ref, y_B_ref, z_B_ref])

        # Rotate as quaternion
        r = Rotation.from_matrix(R_ref)
        q_ref = r.as_quat()

        return q_ref #[x,y,z,0] rad


    def publish_thrust(self) -> None:

        while not rospy.is_shutdown():
            
            
            # update reference

            # reference acceleration -> reference orientation
            self.pos_ref = self.waypoints[self.next_wp_n].transforms[0].translation if self.next_wp_n < len(self.waypoints) else self.pos
            self.acc_ref = self._calculate_acceleration_ref(self.pos, self.pos_ref, self.vel) 
            self.att_q_ref = self._calculate_orientation_ref(self.att.z, self.acc_ref)

            # PID calculate collective thrust
            roll_err = self.att_q_ref[0] - self.att.x
            pitch_err = self.att_q_ref[1] - self.att.y
            yaw_rate_err = self.att_q_ref[2] - self.att.z # TODO currently using yaw not rate
        

            # publish to self.rates_thrust_pub
            sim_input_msg = RollPitchYawrateThrust()
            sim_input_msg.header.stamp = rospy.Time.now()
            sim_input_msg.roll = roll_sim_input
            sim_input_msg.pitch = pitch_sim_input
            sim_input_msg.yaw_rate = yaw_rate_sim_input
            sim_input_msg.thrust.z = a_ref[2]*0.5

            self.rates_thrust_pub.publish(sim_input_msg)

            self.rate.sleep()
        

if __name__ == '__main__':
    node = AttitudeController()
    rospy.spin()