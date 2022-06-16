#!/usr/bin/python3.8
import rospy
import rosparam
import rospkg

import swift
import roboticstoolbox as rtb
import spatialgeometry as sg
import spatialmath as sm
import qpsolvers as qp
import numpy as np
import math
import controller
from scipy.spatial.transform import Rotation as R

from sensor_msgs.msg import JointState
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from control_msgs.msg import JointJog, GripperCommandGoal, GripperCommandAction
from ds4_driver.msg import Status

# import tf
# from fetch_driver_msgs.msg import GripperState


class FetchTeleop:
    def __init__(self):
        self.SIMULATED = True 

        # Robotics toolbox
        self.fetch = rtb.models.Fetch()

        if self.SIMULATED:
            self.env = swift.Swift()
            self.env.launch(realtime=False)

            self.ee_goal = sg.Axes(0.1)
            self.env.add(self.ee_goal)
            self.b_goal = sg.Axes(0.1)
            self.env.add(self.b_goal)

            self.waypoint_0 = sg.Axes(0.1)
            self.env.add(self.waypoint_0)
            self.waypoint_f = sg.Axes(0.1)
            self.env.add(self.waypoint_f)            

            self.fetch.q = self.fetch.qr
            self.env.add(self.fetch)

            self.env.step()


        # ROS parameters
        self.bTe_handover = np.array(rospy.get_param("/fetch_teleop/bTe_handover"))   
        self.bRe_handover = np.array(rospy.get_param("/fetch_teleop/bRe_handover"))      
        self.wTb_waypoints = np.array(rospy.get_param("/fetch_teleop/wTb_waypoints"))
        self.ee_speed = rospy.get_param("/fetch_teleop/ee_speed")
        self.base_lin_speed = rospy.get_param("/fetch_teleop/base_lin_speed")
        self.base_ang_speed = rospy.get_param("/fetch_teleop/base_ang_speed")
        self.torso_max = rospy.get_param("/fetch_teleop/torso_max")


        # Waypoint information
        # self.bTe_handover = np.array([0.9, 0.0, 0.8])
        self.bTe_tucked = np.array([0.3, 0, 0.6])
        self.bRe_tucked = np.array([0., np.pi / 2, 0.])

        self.bTe_handover_list = np.array([[0.75,  0.30, 0.7],
                                           [0.75,  0.00, 0.7],
                                           [0.75, -0.30, 0.7]])

        self.bRe_handover_list = np.array([[0.00, -0.50, 0.00],
                                           [0.00,  0.00, 0.00],
                                           [0.00,  0.50, 0.00]])   

        self.handover_mode = 0 # 0: Left, 1: Middle, 2: Right                                        

        # self.wTb_waypoints = np.array([np.array([0.0, 0.0]),
        #                       np.array([5.0, 0.0]),
        #                       np.array([5.0, 5.0]),
        #                       np.array([0.0, 5.0])])

        self.wTb_cumulative_lengths = [0.0]
        for i in range(len(self.wTb_waypoints) - 1):
            l = np.linalg.norm(self.wTb_waypoints[i + 1] - self.wTb_waypoints[i])
            self.wTb_cumulative_lengths += [self.wTb_cumulative_lengths[i] + l]

        self.b_ratio = 0.0 # 0: Start, 1: End
        self.ee_ratio = 0.0 # 0: Tucked, 1: Handovver
        self.segment = 0
        self.start_ee_edit = False


        # ROS publishers and subscribers
        # self.tf_broadcaster = tf.TransformBroadcaster()
        # self.tf_listener  = tf.TransformListener()
        self.jointpos_sub = rospy.Subscriber("joint_states", JointState, self.jointpos_callback)

        # self.jointvel_pub = rospy.Publisher("/arm_controller/joint_velocity/command", JointJog, queue_size=1)
        # self.basevel_pub  = rospy.Publisher("/base_controller/command", Twist, queue_size=1)
        self.teleop_sub = rospy.Subscriber("cmd_vel", Twist, self.teleop_callback)
        self.ds4_sub = rospy.Subscriber("status", Status, self.ds4_callback)

        self.main_timer = rospy.Timer(rospy.Duration(0.01), self.main_callback)


    def jointpos_callback(self, data): 
        # Check if message is a gripper state or manipulator state
        if len(data.name) > 2:
            # Read joint positions
            self.fetch.q[:2] = 0
            self.fetch.q[2:] = np.array([data.position[6],
                                         data.position[7],
                                         data.position[8],
                                         data.position[9],
                                         data.position[10],
                                         data.position[11],
                                         data.position[12]])


    def main_callback(self, event):
        # if not self.SIMULATED:
        #     # Read base position
        #     try:
        #         # Read base frame coordinates
        #         (trans, rot) = self.tf_listener.lookupTransform('map', 'base_link', rospy.Time(0))

        #         T = sm.SE3()
        #         T.A[:3, :3] = sm.UnitQuaternion(rot[3], rot[:3]).R
        #         T.A[:3, 3] = trans
        #         self.fetch._base = T
        #     except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
        #         print(e)
        #         pass


        # Calculate waypoints given control ratios
        bRep = sm.SE3.Rz(self.bRe_handover[1] * self.ee_ratio + self.bRe_tucked[1] * (1 - self.ee_ratio)) * sm.SE3.Ry(self.bRe_handover[2] * self.ee_ratio)
        bTep = sm.SE3(self.bTe_tucked * (1 - self.ee_ratio) + self.bTe_handover * self.ee_ratio) * bRep
        
        self.ee_goal._base = (self.fetch.base * bTep).A

        wTb = controller.interpolate_path(self.b_ratio, self.wTb_cumulative_lengths, self.wTb_waypoints)
        self.b_goal.base = sm.SE2(wTb).SE3()


        # Solve for arm and base velocities to target waypoints
        arm_qd = controller.arm(self.fetch, bTep.A)
        self.fetch.qd[3:] = arm_qd[3:]
        self.b_ratio, self.segment = controller.closest_point(self.fetch, self.wTb_cumulative_lengths, self.wTb_waypoints)

        if self.SIMULATED:
            self.env.step(0.01)

            base_new = self.fetch.fkine(self.fetch._q, end=self.fetch.links[2], fast=True)
            self.fetch._base.A[:] = base_new
            self.fetch.q[:2] = 0

            self.waypoint_0.base = sm.SE2(self.wTb_waypoints[0]).SE3()
            self.waypoint_f.base = sm.SE2(self.wTb_waypoints[-1]).SE3()


        # if not self.SIMULATED:
        #     # Publish base and joint velocities
        #     v_base = Twist()
        #     self.moving_avg_base = np.r_[self.moving_avg_base[1:], np.array([qd[:2]])]
        #     v_base.angular.z = np.average(self.moving_avg_base, axis=0)[0]
        #     v_base.linear.x = np.average(self.moving_avg_base, axis=0)[1]
        #     self.basevel_pub.publish(v_base)

        #     v_joint = JointJog()
        #     self.moving_avg_qd = np.r_[self.moving_avg_qd[1:], np.array([qd[2:]])]
        #     v_joint.velocities = np.average(self.moving_avg_qd, axis=0)
        #     self.jointvel_pub.publish(v_joint)

        #     v_head = JointJog()
        #     self.moving_avg_camera = np.r_[self.moving_avg_camera[1:], np.array([qd_cam[-2:]])]
        #     v_head.velocities = np.average(self.moving_avg_camera, axis=0)        
        #     self.headvel_pub.publish(v_head)        


    def teleop_callback(self, msg):
        TELEOP_HZ = 30
        EE_SPEED = 0.25

        self.ee_ratio += msg.angular.z / (TELEOP_HZ / EE_SPEED)
        self.ee_ratio = min(max(0.0, self.ee_ratio), 1.0)

        base_qd = controller.base(self.fetch, msg.linear.x, self.b_ratio, self.wTb_cumulative_lengths, self.wTb_waypoints)
        self.fetch.qd[0:2] = base_qd


    def ds4_callback(self, msg):
        TELEOP_HZ = 100
        EE_SPEED = 0.25
        rate = EE_SPEED / TELEOP_HZ

        # L1: Activate control
        #   Default: Base control
        #       R-stick: Forward/backward
        #       L-stick: Rotate
        #   R1: Arm control
        #       R-stick: Extend/retract
        # L2: Edit position mode
        #   Default: Base waypoint edit
        #   R1: Arm waypoint edit

        if msg.button_l1: # Activate control
            if msg.button_r1: # Arm control
                if msg.button_l2: # Edit ee final waypoint
                    if not self.start_ee_edit:
                        r_pose = sm.SE3(self.fetch.fkine(self.fetch.q, include_base=False, fast=True))
                        self.bTe_handover = r_pose.t
                        self.bRe_handover = controller.rotationToRPY(r_pose.R)
                        self.start_ee_edit = True
                    self.ee_ratio = 1.0

                    self.bTe_handover += controller.edit_ee_waypoint(msg, rate)
                    self.bRe_handover += controller.edit_ee_angle_waypoint(msg, rate)
                else: # Control arm
                    if msg.button_r3:
                        self.ee_speed = abs(msg.axis_right_y)
                    if self.ee_ratio > 0.9 and self.handover_mode != 1:
                        self.bRe_tucked[1] = np.sign(self.bRe_handover[1]) * np.pi/2
                    self.ee_ratio += np.sign(msg.axis_right_y) * self.ee_speed * rate
                    self.ee_ratio = min(max(0.0, self.ee_ratio), 1.0)
                    self.bRe_tucked = controller.edit_ee_tucked(self.bRe_tucked, msg, rate)
                    self.start_ee_edit = False
            else: # Base control
                if msg.button_l2: # Edit base waypoint
                    self.fetch.qd[0:2] = np.array([
                        max(min(msg.axis_left_x, self.fetch.qdlim[1]), -self.fetch.qdlim[1]),
                        max(min(msg.axis_right_y, self.fetch.qdlim[0]), -self.fetch.qdlim[0])
                    ])

                    self.wTb_waypoints = controller.edit_base_waypoint(self.fetch, self.wTb_waypoints, self.b_ratio)
                else:
                    if msg.button_r3:
                        self.base_lin_speed = abs(msg.axis_right_y)
                    if msg.button_l3:
                        self.base_ang_speed = abs(msg.axis_left_x)
                    v = np.sign(msg.axis_right_y) * self.base_lin_speed
                    omega = np.sign(msg.axis_left_x) * self.base_ang_speed             
                    base_qd = controller.base(self.fetch, v, omega, self.b_ratio, self.wTb_cumulative_lengths, self.wTb_waypoints, self.segment)
                    self.fetch.qd[0:2] = base_qd

        # Torso control
        if msg.button_square:
            self.handover_mode = 0
            self.bTe_handover = self.bTe_handover_list[0]
            self.bRe_handover = self.bRe_handover_list[0]
        elif msg.button_triangle:
            self.handover_mode = 1
            self.bTe_handover = self.bTe_handover_list[1]
            self.bRe_handover = self.bRe_handover_list[1]  
        elif msg.button_circle:
            self.handover_mode = 2
            self.bTe_handover = self.bTe_handover_list[2]
            self.bRe_handover = self.bRe_handover_list[2]                   

        if msg.button_options:
            self.save_params()


    def save_params(self):
        # Update old ROS params
        rospy.set_param("/fetch_teleop/bTe_handover", self.bTe_handover.tolist())
        rospy.set_param("/fetch_teleop/wTb_waypoints", self.wTb_waypoints.tolist())
        rospy.set_param("/fetch_teleop/ee_speed", float(self.ee_speed))
        rospy.set_param("/fetch_teleop/base_lin_speed", float(self.base_lin_speed))
        rospy.set_param("/fetch_teleop/base_ang_speed", float(self.base_ang_speed))
        rospy.set_param("/fetch_teleop/torso_max", float(self.torso_max))

        # Overwrite old ROS params file
        rospack = rospkg.RosPack()
        path = rospack.get_path("fetch_teleop")
        rosparam.dump_params(path + "/cfg/preferences.yaml", "/fetch_teleop", verbose=True)


if __name__ == '__main__':
    try:
        rospy.init_node("fetch_teleop", anonymous=True)
        fetch_teleop = FetchTeleop()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass