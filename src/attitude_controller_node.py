#!/usr/bin/env python3

import rospy

import numpy as np
import math                                             # accel and orientation calculations
from scipy.spatial.transform import Rotation

from simple_pid import PID                              # using this PID class because making my own would look just like this

# from tf.transformations import quaternion_from_matrix, euler_from_quaternion, euler_from_matrix, quaternion_multiply

from mav_msgs.msg import RollPitchYawrateThrust         # input to rotors_control package
from trajectory_msgs.msg import MultiDOFJointTrajectory # reference trajectory (pos + yaw), from 7.2
from nav_msgs.msg import Odometry                       # sensor data

from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3


    
class AttitudeController():
    def __init__(self) -> None:
        # initialize node
        rospy.init_node("attitude_controller_node")
        node_rate = rospy.get_param("~node_rate")
        self.dt = 1.0 / node_rate
        self.rate = rospy.Rate(node_rate)

        # just using the same PID for everything since i don't care about specifics rn
        self.roll_pid = PID(1.0, 0.1, 0.1, setpoint=0) 
        self.pitch_pid = PID(1.0, 0.1, 0.1, setpoint=0) 
        self.yaw_rate_pid = PID(1.0, 0.1, 0.1, setpoint=0) 

        # initialize variables
        self.next_wp_n = 0
        self.pos = Point()
        self.vel = Vector3()
        self.att_q = Point()
        self.waypoints = [] #MultiDOFJointTrajectory()

        # publishers
        self.rates_thrust_pub = rospy.Publisher('rmf_obelix/command/roll_pitch_yawrate_thrust', RollPitchYawrateThrust, queue_size=1)

        # subscribers
        self.ref_waypoints_sub = rospy.Subscriber('rmf_obelix/command/trajectory', MultiDOFJointTrajectory, self._ref_waypoints_cb, queue_size=10)
        # self.pose_sub = rospy.Subscriber('rmf_obelix/ground_truth/pose', Pose, self._pose_cb)
        self.odometry_sub = rospy.Subscriber('rmf_obelix/odometry_sensor1/odometry', Odometry, self._odometry_cb)

    # CALLBACKS
    def _ref_waypoints_cb(self, msg : MultiDOFJointTrajectory) -> None:
        # receive 10 waypoints at a time, start at nr0
        self.waypoints = msg.points
        self.next_wp_n = 0
        rospy.loginfo(f"--> Received WAYPOINTS {self.waypoints}") # Never happens, should have made another node to publish these

    def _odometry_cb(self, msg : Odometry) -> None:
        self.pos = msg.pose.pose.position #msg.position
        self.vel = msg.twist.twist.linear
        self.att_q = msg.pose.pose.orientation #msg.orientation
        rospy.loginfo(f"--> Received position \n{self.pos} \nvelocity \n{self.vel}, \nattitude {self.att_q}")
 
        self.generate_control()
        # timestamp ??? 

        #if self.next_wp < len(wps):

    # Calculations from paper 
    def _get_acc_ref(self, pos : Point, pos_ref : Point, vel : Vector3) ->  np.ndarray:
        p = np.array([pos.x, pos.y, pos.z])
        p_ref = np.array([pos_ref.x, pos_ref.y, pos_ref.z])
        v = np.array([vel.x, vel.y, vel.z])

        # just random values
        K_POS = np.array([1.0, 1.0, 1.0]) 
        K_VEL = np.array([1.0, 1.0, 1.0])

        a_fb = -K_POS*(p - p_ref) + -K_VEL*v
        a_ref = a_fb + np.array([0.0, 0.0, 9.81])
        
        return a_ref

    def _get_att_ref(self, yaw : float, a_ref : np.ndarray) -> np.ndarray:
        
        x_c = np.array([math.cos(yaw), math.sin(yaw), 0.0])
        y_c = np.array([-math.sin(yaw), math.cos(yaw), 0.0])

        z_B_ref = a_ref / np.linalg.norm(a_ref)
        x_B_ref = np.cross(y_c, z_B_ref) / np.linalg.norm(np.cross(y_c, z_B_ref))
        y_B_ref = np.cross(z_B_ref, x_B_ref)
        R_ref = np.array([x_B_ref, y_B_ref, z_B_ref])

        # att_ref = euler_from_matrix(R_ref)
        att_q = Rotation.from_matrix(R_ref)
        att_ref = att_q.as_quat()

        return att_ref #[x,y,z,0] rad

    def _publish_control_msg(self, roll_control, pitch_control, yaw_rate_control, thrust_control) -> None:
        # publish to self.rates_thrust_pub
        control_msg = RollPitchYawrateThrust()
        control_msg.roll = roll_control
        control_msg.pitch = pitch_control
        control_msg.yaw_rate = yaw_rate_control
        control_msg.thrust.z = thrust_control
        control_msg.header.stamp = rospy.Time.now()
        rospy.loginfo(f"<-- Publishing control message \n{control_msg}")

        self.rates_thrust_pub.publish(control_msg)


    def generate_control(self) -> None:
        # rospy.loginfo("Starting generate_control")
        # while not rospy.is_shutdown():
        # update reference
        # rospy.loginfo("generate control loop")
        pos_ref = self.waypoints[self.next_wp_n].transforms[0].translation if self.next_wp_n < len(self.waypoints) else self.pos
        acc_ref = self._get_acc_ref(self.pos, pos_ref, self.vel) 
        [x_ref, y_ref, z_ref, w_ref] = self._get_att_ref(self.att_q.z, acc_ref)

        # roll, pitch, yaw = euler_from_quaternion(self.att_q)

        # PID calculate collective thrust
        roll_err = x_ref - self.att_q.x
        pitch_err = y_ref - self.att_q.y
        yaw_rate_err = z_ref - self.att_q.z
    
        # PID here o/
        roll_control = self.roll_pid(roll_err)
        pitch_control = self.pitch_pid(pitch_err)
        yaw_rate_control = self.yaw_rate_pid(yaw_rate_err)

        self._publish_control_msg(roll_control, pitch_control, yaw_rate_control, acc_ref[2]*0.5)

        # self.rate.sleep()
        

if __name__ == '__main__':
    rospy.loginfo("TTK22 Starting attitude_controller_node")
    node = AttitudeController()
    
    rospy.spin()
    # node.generate_control()
