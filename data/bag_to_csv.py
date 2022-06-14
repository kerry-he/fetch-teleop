import os
import csv
import numpy as np

import rosbag
import rospy

import tf
import tf2_ros
import geometry_msgs.msg

BODYPARTS = [
    "nose",
    "left_eye_inner",
    "left_eye",
    "left_eye_outer",
    "right_eye_inner",
    "right_eye",
    "right_eye_outer",
    "left_ear",
    "right_ear",
    "mouth_left",
    "mouth_right",
    "left_shoulder",
    "right_shoulder",
    "left_elbow",
    "right_elbow",
    "left_wrist",
    "right_wrist",
    "left_pinky",
    "right_pinky",
    "left_index",
    "right_index",
    "left_thumb",
    "right_thumb",
    "left_hip",
    "right_hip"
]

def ang_diff(a, b):
    normDeg = abs(a-b)
    while normDeg > 2*np.pi:
        normDeg -= 2*np.pi
    absDiffDeg = min(2*np.pi-normDeg, normDeg)
    return absDiffDeg


class BagToCSV:
    def __init__(self, name):
        self.name = name
        self.directory = "./"
        self.br = tf.TransformBroadcaster()
        self.static_br = tf2_ros.StaticTransformBroadcaster()
        self.listener = tf.TransformListener()

        self.dt = 0.1
        self.t0 = None
        self.i = 0

        self.episode = 0
        self.theta_to_user = -3.0
        self.episode_hysteresis = False

        self.base_lin = 0.0
        self.base_ang = 0.0
        self.ee_lin = 0.0
        
        self.bodyparts = [0.0] * len(BODYPARTS) * 4

        self.t_base = [0, 0, 0]
        self.R_base = [0, 0, 0, 1]
        self.t_gripper = [0, 0, 0]
        self.R_gripper = [0, 0, 0, 1]
        self.t_handover_goal = [0, 0, 0]
        self.R_handover_goal = [0, 0, 0, 1]

        self.handover_quality = "N/A"
        self.handover_type = "N/A"          


    def run(self):
        with open(self.name + ".csv", "w", newline='') as csvfile:
            csv_writer = csv.writer(csvfile, delimiter=',',
                                    quotechar='|', quoting=csv.QUOTE_MINIMAL)

            title = ["episode", "time (s)", "handover quality", "handover type"]
            title += ["base (linear)", "base (angular)", "arm (linear)"]
            title += ["base (x)", "base (y)", "base (theta)"]
            title += ["gripper (x)", "gripper (y)", "gripper (z), gripper (qx)", "gripper (qy)", "gripper (qz)", "gripper (qw)"]
            title += ["handover_goal (x)", "handover_goal (y)", "handover_goal (z), handover_goal (qx)", "handover_goal (qy)", "handover_goal (qz)", "handover_goal (qw)"]
            for bodypart in BODYPARTS:
                title += [bodypart + " (x)",
                          bodypart + " (y)",
                          bodypart + " (z)",
                          bodypart + " (confidence)" 
                ]
            csv_writer.writerow(title)

            for filename in os.scandir(self.directory):
                if filename.is_file():
                    if os.path.splitext(filename)[-1].lower() == ".bag":
                        if filename.name[0:len(self.name)] == self.name:

                            total_messages = rosbag.Bag(filename.name).get_message_count()
                            current_message = 0                            

                            for topic, msg, t in rosbag.Bag(filename.name).read_messages():
                                t_obj = t
                                t = t.to_sec()
                                if self.t0 is None:
                                    self.t0 = t

                                if t - self.t0 >= self.dt * self.i:
                                    csv_writer.writerow(
                                        [self.episode, self.dt * self.i, self.handover_quality, self.handover_type] 
                                         + self.write_csv_line()
                                    )
                                    self.i += 1

                                if topic == "/status":
                                    self.read_ds4(msg)
                                elif topic == "/tf":
                                    self.read_tf(msg, t_obj)
                                elif topic == "/tf_static":
                                    self.read_static_tf(msg, t_obj)
                                elif topic == "/body_pose":
                                    self.read_pose(msg)
                                elif topic == "/good_data":
                                    self.read_data_quality(msg)

                                self.update_episode()

                                current_message += 1

                                if current_message % 10000 == 0:
                                    print("Finished converting: ", current_message, "/", total_messages)

                            print("Finished converting: ", current_message, "/", total_messages)                                    

    def read_ds4(self, msg):
        # Base movement
        if msg.button_l1 and not msg.button_r1:
            self.base_lin = msg.axis_right_y
            self.base_ang = msg.axis_left_x
        else:
            self.base_lin = 0.0
            self.base_ang = 0.0

        # Arm movement
        if msg.button_l1 and msg.button_r1:
            self.ee_lin = msg.axis_right_y
        else:
            self.ee_lin = 0.0

    def read_tf(self, msg, t):
        for transform in msg.transforms:
            trans = [
                transform.transform.translation.x,
                transform.transform.translation.y,
                transform.transform.translation.z
            ]
            quat= [
                transform.transform.rotation.x,
                transform.transform.rotation.y,
                transform.transform.rotation.z,
                transform.transform.rotation.w
            ]            
            self.br.sendTransform(trans,
                                  quat,
                                  t,
                                  transform.child_frame_id,
                                  transform.header.frame_id)

    def read_static_tf(self, msg, t):
        for transform in msg.transforms:
            static_transformStamped = transform
            self.static_br.sendTransform(static_transformStamped)


    def read_pose(self, msg):
        self.bodyparts = []
        for i in range(len(BODYPARTS)):
            self.bodyparts += [
                msg.array[i].x,
                msg.array[i].y,
                msg.array[i].z,
                msg.array[i].confidence
            ]

    def read_data_quality(self, msg):
        if msg.data:
            self.handover_quality = "GOOD"
        else:
            self.handover_quality = "BAD"

    def update_episode(self):
        robot_theta = tf.transformations.euler_from_quaternion(self.R_base, axes='sxyz')[-1]

        if self.episode_hysteresis:
            if ang_diff(robot_theta, self.theta_to_user) < 0.1:
                self.episode += 1
                self.episode_hysteresis = False
                self.handover_quality = "N/A"
                self.handover_type = "N/A"
        else:
            self.episode_hysteresis = ang_diff(robot_theta, self.theta_to_user) > np.pi / 2


    def write_csv_line(self):
        # Controller inputs
        row = [
            self.base_lin,
            self.base_ang,
            self.ee_lin
        ]

        # TF frames
        try:
            (trans,rot) = self.listener.lookupTransform("map", "base_link", rospy.Time(0))
            self.t_base = trans
            self.R_base = rot
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass

        try:
            (trans,rot) = self.listener.lookupTransform("base_link", "gripper_link", rospy.Time(0))
            self.t_gripper = trans
            self.R_gripper = rot            
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass

        try:
            (trans,rot) = self.listener.lookupTransform("base_link", "handover_goal", rospy.Time(0))
            self.t_handover_goal = trans
            self.R_handover_goal = rot            
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass

        row += self.t_base[0:2] + [tf.transformations.euler_from_quaternion(self.R_base, axes='sxyz')[-1]]
        row += self.t_gripper + self.R_gripper
        row += self.t_handover_goal + self.R_handover_goal

        # Pose estimation
        row += self.bodyparts

        return row


if __name__ == "__main__":
    name = input("Enter prefix of bag to convert to CSV: ")

    rospy.init_node('test_node')
    
    bag_to_csv = BagToCSV(name)
    bag_to_csv.run()