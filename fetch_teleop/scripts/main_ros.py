#!/usr/bin/python3
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

from sensor_msgs.msg import JointState, LaserScan
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Twist, Point, PointStamped
from control_msgs.msg import JointJog, GripperCommandGoal, GripperCommandAction
from visualization_msgs.msg import Marker
from ds4_driver.msg import Status

import tf


class FetchTeleop:
    def __init__(self):
        # Robotics toolbox
        self.fetch = rtb.models.Fetch()
        self.fetch_cam = rtb.models.FetchCamera()
        self.fetch.base
        self.fetch_cam.base

        # ROS parameters
        self.bTe_handover_list = np.array(rospy.get_param("/fetch_teleop/bTe_handover_list"))
        self.bRe_handover_list = np.array(rospy.get_param("/fetch_teleop/bRe_handover_list"))   
        self.wTb_waypoints = np.array(rospy.get_param("/fetch_teleop/wTb_waypoints"))
        self.ee_speed = rospy.get_param("/fetch_teleop/ee_speed")
        self.base_lin_speed = rospy.get_param("/fetch_teleop/base_lin_speed")
        self.base_ang_speed = rospy.get_param("/fetch_teleop/base_ang_speed")

        self.ds4_name = rospy.get_param("/ds4_name")


        # Waypoint information
        self.bTe_tucked = np.array([0.3, 0, 0.9])
        self.bRe_tucked = np.array([0., -np.pi / 2, 0.])

        self.handover_mode = 1 # 0: Left, 1: Middle, 2: Right
        self.bTe_handover = self.bTe_handover_list[self.handover_mode]
        self.bRe_handover = self.bRe_handover_list[self.handover_mode]

        self.wTb_cumulative_lengths = [0.0]
        for i in range(len(self.wTb_waypoints) - 1):
            l = np.linalg.norm(self.wTb_waypoints[i + 1] - self.wTb_waypoints[i])
            self.wTb_cumulative_lengths += [self.wTb_cumulative_lengths[i] + l]

        self.b_ratio = 0.0 # 0: Start, 1: End
        self.ee_ratio = 0.0 # 0: Tucked, 1: Handovver
        self.segment = 0

        # Base safety stopping feature
        self.l1 = False
        self.detected_object = False
        self.min_distance_from_person = 0.5

        self.RANGE_THRESHOLD_UPPER = 0.6
        self.RANGE_THRESHOLD_LOWER = 0.01         


        # ROS publishers and subscribers
        self.jointvel_pub = rospy.Publisher("/arm_controller/joint_velocity/command", JointJog, queue_size=1)
        self.headvel_pub = rospy.Publisher("/head_controller/joint_velocity/command", JointJog, queue_size=1)
        self.basevel_pub  = rospy.Publisher("/base_controller/command", Twist, queue_size=1)
        self.viz_pub  = rospy.Publisher("/visualization_marker", Marker, queue_size=1)

        self.tf_broadcaster = tf.TransformBroadcaster()
        self.tf_listener  = tf.TransformListener()
        self.jointpos_sub = rospy.Subscriber("joint_states", JointState, self.jointpos_callback)
        self.laser_sub = rospy.Subscriber("base_scan", LaserScan, self.laser_scan_callback, queue_size = 1)
        self.teleop_sub = rospy.Subscriber("/fetch_teleop/cmd_vel", Twist, self.teleop_callback)
        self.ds4_sub = rospy.Subscriber("status", Status, self.ds4_callback)
        self.cp_sub = rospy.Subscriber("clicked_point", PointStamped, self.cp_callback)

        self.main_timer = rospy.Timer(rospy.Duration(0.01), self.main_callback)
        self.viz_timer = rospy.Timer(rospy.Duration(1.0), self.viz_callback)


    def jointpos_callback(self, data): 
        # Check if message is a gripper state or manipulator state
        if len(data.name) > 2:
            # Read joint positions
            self.fetch.q[:2] = 0
            self.fetch.q[2:] = np.array([data.position[2],
                                         data.position[6],
                                         data.position[7],
                                         data.position[8],
                                         data.position[9],
                                         data.position[10],
                                         data.position[11],
                                         data.position[12]])

            self.fetch_cam.q[:2] = 0
            self.fetch_cam.q[2:] = np.array([data.position[2],
                                             data.position[4],
                                             data.position[5]])                                         



    def main_callback(self, event):
        # Read base position
        try:
            # Read base frame coordinates
            (trans, rot) = self.tf_listener.lookupTransform('map', 'base_link', rospy.Time(0))

            T = sm.SE3()
            T.A[:3, :3] = sm.UnitQuaternion(rot[3], rot[:3]).R
            T.A[:3, 3] = trans
            self.fetch._base = T
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            # print(e)
            pass

        # Calculate waypoints given control ratios
        bRep = sm.SE3.Rz(self.bRe_handover[1] * self.ee_ratio + self.bRe_tucked[1] * (1 - self.ee_ratio)) * sm.SE3.Ry(self.bRe_handover[2] * self.ee_ratio)
        bTep = sm.SE3(self.bTe_tucked * (1 - self.ee_ratio) + self.bTe_handover * self.ee_ratio) * bRep

        self.tf_broadcaster.sendTransform(tuple(bTep.t),
                                          tuple(np.append(sm.UnitQuaternion(bTep.R).A[1:], sm.UnitQuaternion(bTep.R).A[0])),
                                          rospy.Time.now(),
                                          "ee_goal",
                                          "base_link")

        self.tf_broadcaster.sendTransform(tuple(self.bTe_handover),
                                          tf.transformations.quaternion_from_euler(self.bRe_handover[0], self.bRe_handover[2], self.bRe_handover[1]),
                                          rospy.Time.now(),
                                          "handover_goal",
                                          "base_link")        

        # Solve for arm and base velocities to target waypoints
        arm_qd = controller.arm(self.fetch, bTep.A)
        self.fetch.qd[3:] = arm_qd[3:]
        self.b_ratio, self.segment = controller.closest_point(self.fetch, self.wTb_cumulative_lengths, self.wTb_waypoints)

        camera_qd = controller.camera(self.fetch, self.fetch_cam, self.ee_ratio > 0.5)

        v_head = JointJog()
        v_head.velocities = camera_qd  
        self.headvel_pub.publish(v_head)            

        # Publish arm velocity
        if not self.detected_object:
            if self.l1:
                v_joint = JointJog()
                v_joint.velocities = self.fetch.qd[2:]
                self.jointvel_pub.publish(v_joint)        
        else:
            rospy.logwarn_throttle(1.0, "Obstacle or human detected in front of robot. Stopping robot.")
            self.stop_robot()


    def teleop_callback(self, msg):
        # (Legacy) For keyboard control
        TELEOP_HZ = 30
        EE_SPEED = 0.25

        self.ee_ratio += msg.angular.z / (TELEOP_HZ / EE_SPEED)
        self.ee_ratio = min(max(0.0, self.ee_ratio), 1.0)
        
        base_qd = controller.base(self.fetch, msg.linear.x, self.b_ratio, self.wTb_cumulative_lengths, self.wTb_waypoints)

        # Publish base velocity
        if not self.detected_object:
            v_base = Twist()
            v_base.angular.z = base_qd[0]
            v_base.linear.x = base_qd[1]
            self.basevel_pub.publish(v_base)
        else:
            rospy.logwarn_throttle(1.0, "Obstacle or human detected in front of robot. Stopping robot.")
            self.stop_robot()


    def ds4_callback(self, msg):

        # Make sure DS4 header is as expected to avoid conflicts between automatic and manual control
        if msg.header.frame_id != self.ds4_name:
            return

        # Define rate of DS4 messages
        TELEOP_HZ = 100
        EE_SPEED = 0.25
        rate = EE_SPEED / TELEOP_HZ

        # L1: Activate control
        #   Default: Base control
        #       R-stick: Forward/backward
        #       L-stick: Rotate
        #   R1: Arm control
        #       R-stick: Extend/retract
        # L1 + L2: Edit position mode
        #   Default: Base waypoint edit
        #   R1: Arm waypoint edit        

        # L1 trigger used to activate robot control
        self.l1 = msg.button_l1
        if not msg.button_l1:
            # Handover quality
            if msg.button_circle:
                rospy.loginfo_throttle(1.0, "Registed GOOD handover.")
            elif msg.button_cross:
                rospy.loginfo_throttle(1.0, "Registed BAD handover.")

            # Handover type
            if msg.button_l2:
                rospy.loginfo_throttle(1.0, "Registed ROBOT TO HUMAN handover.")
            elif msg.button_r2:
                rospy.loginfo_throttle(1.0, "Registed HUMAN TO ROBOT handover.")                

            self.stop_robot()

            # Share button to reset params to those currently in yaml file
            if msg.button_share:
                self.reset_params()

            # Option button to save params to yaml file
            if msg.button_options:
                self.save_params()                 
            
            return

        if msg.button_r1: 
            # Arm control
            if msg.button_l2: 
                # Edit ee final waypoint
                self.bTe_handover += controller.edit_ee_waypoint(msg, rate)
                self.bTe_handover[2] = max(0.90, self.bTe_handover[2]) # Limit z-axis
                self.bRe_handover += controller.edit_ee_angle_waypoint(msg, rate)

                # Save edited goal to relevant slot
                self.bTe_handover_list[self.handover_mode] = self.bTe_handover
                self.bRe_handover_list[self.handover_mode] = self.bRe_handover
            else: 
                # Control arm
                if msg.button_r3:
                    # Edit speed of end-effector
                    self.ee_speed = abs(msg.axis_right_y)
                if self.ee_ratio > 0.99 and self.handover_mode != 1:
                    # If changing from left to right handover, change tucked orientation (and vice versa)
                    self.bRe_tucked[1] = np.sign(self.bRe_handover[1]) * np.pi/2
                if self.ee_ratio < 0.01:
                    # If tucked, allow for rotation of tuck to change from left to right handover (and vice versa)
                    self.bTe_tucked, self.bRe_tucked = controller.edit_ee_tucked(self.bTe_tucked, self.bRe_tucked, msg, rate)  

                self.ee_ratio += np.sign(msg.axis_right_y) * self.ee_speed * rate
                self.ee_ratio = min(max(0.0, self.ee_ratio), 1.0)

            base_qd = np.array([0., 0.])
        else: 
            # Base control
            if msg.button_l2: 
                # Edit base waypoint
                base_qd = np.array([
                    max(min(msg.axis_left_x, self.fetch.qdlim[1]), -self.fetch.qdlim[1]),
                    max(min(msg.axis_right_y, self.fetch.qdlim[0]), -self.fetch.qdlim[0])
                ])

                self.wTb_waypoints = controller.edit_base_waypoint(self.fetch, self.wTb_waypoints, self.b_ratio)
            else:
                # Control base
                if msg.button_r3:
                    # Edit base angular speed
                    self.base_lin_speed = abs(msg.axis_right_y)
                if msg.button_l3:
                    # Edit base linear speed
                    self.base_ang_speed = abs(msg.axis_left_x)
                v = np.sign(msg.axis_right_y) * self.base_lin_speed
                omega = np.sign(msg.axis_left_x) * self.base_ang_speed
                base_qd = controller.base(self.fetch, v, omega, self.b_ratio, self.wTb_cumulative_lengths, self.wTb_waypoints, self.segment)


        # Arm discrete positions
        if self.ee_ratio < 0.01:
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


        # Publish base velocity       
        if not self.detected_object:
            if self.ee_ratio < 0.1:
                v_base = Twist()
                v_base.angular.z = base_qd[0]
                v_base.linear.x = base_qd[1]
                self.basevel_pub.publish(v_base)
        else:
            rospy.logwarn_throttle(1.0, "Obstacle or human detected in front of robot. Stopping robot.")
            self.stop_robot()       

    
    def viz_callback(self, event):
        # Draw base path to RViz
        marker = Marker()

        marker.header.frame_id = "/map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "path"

        # set shape, Arrow: 0; Cube: 1 ; Sphere: 2 ; Cylinder: 3 ; Line strip 4
        marker.type = 4
        marker.id = 0

        # Set the scale of the marker
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1

        # Set the color
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        # Set the pose of the marker
        for wTb_waypoint in self.wTb_waypoints:
            point = Point()
            point.x = wTb_waypoint[0]
            point.y = wTb_waypoint[1]

            marker.points += [point]

        self.viz_pub.publish(marker)


    def stop_robot(self):
        # Stop robot by sending zero velocities to base and arm
        v_base = Twist()
        self.basevel_pub.publish(v_base)         

        v_joint = JointJog()
        v_joint.velocities = np.zeros(8)
        self.jointvel_pub.publish(v_joint)           


    def laser_scan_callback(self, data):
        ranges = data.ranges
        minimum = 0.6 # if something detected at less than 0.6m in front
        detected_object_temp = False
        max_range_indices = len(ranges)
        # choosing only lased scan results in front of robot for roughly 1/5 of FOV
        range_val_lower = int(2*max_range_indices/5)
        range_vals_upper = int(max_range_indices*3/5)
        # Checking if any value less than 0.6 and saving minimum distance
        for range_val in ranges[range_val_lower: range_vals_upper]:
            if np.isnan(range_val) or np.isinf(range_val):
                continue
            if range_val < self.RANGE_THRESHOLD_UPPER and range_val > self.RANGE_THRESHOLD_LOWER and detected_object_temp == False:
                detected_object_temp = True
            if range_val < minimum:
                minimum = range_val
        self.min_distance_from_person = minimum
        # self.detected_object = detected_object_temp    
        self.detected_object = False   

    
    def cp_callback(self, msg):
        point = np.array([msg.point.x, msg.point.y])
        self.wTb_waypoints = controller.edit_base_waypoint_cp(point, self.wTb_waypoints)

    def save_params(self):
        # Update old ROS params
        rospy.set_param("/fetch_teleop/bTe_handover_list", self.bTe_handover_list.tolist())
        rospy.set_param("/fetch_teleop/bRe_handover_list", self.bRe_handover_list.tolist())
        rospy.set_param("/fetch_teleop/wTb_waypoints", self.wTb_waypoints.tolist())
        rospy.set_param("/fetch_teleop/ee_speed", float(self.ee_speed))
        rospy.set_param("/fetch_teleop/base_lin_speed", float(self.base_lin_speed))
        rospy.set_param("/fetch_teleop/base_ang_speed", float(self.base_ang_speed))

        # Overwrite old ROS params file
        rospack = rospkg.RosPack()
        path = rospack.get_path("fetch_teleop")
        rosparam.dump_params(path + "/cfg/preferences.yaml", "/fetch_teleop", verbose=True)

    def reset_params(self):
        # Re-write ROS params with those currently in cfg file
        rospy.loginfo("Reset parameters")
        self.bTe_handover_list = np.array(rospy.get_param("/fetch_teleop/bTe_handover_list"))
        self.bRe_handover_list = np.array(rospy.get_param("/fetch_teleop/bRe_handover_list"))   
        self.wTb_waypoints = np.array(rospy.get_param("/fetch_teleop/wTb_waypoints"))
        self.ee_speed = rospy.get_param("/fetch_teleop/ee_speed")
        self.base_lin_speed = rospy.get_param("/fetch_teleop/base_lin_speed")
        self.base_ang_speed = rospy.get_param("/fetch_teleop/base_ang_speed")        


if __name__ == '__main__':
    try:
        rospy.init_node("fetch_teleop", anonymous=True)
        fetch_teleop = FetchTeleop()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass