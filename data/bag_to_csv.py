import rosbag
import os
import csv
import numpy as np


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


class BagToCSV:
    def __init__(self, name):
        self.name = name
        self.directory = "./"

        self.dt = 0.1
        self.t0 = None
        self.i = 0

        self.base_lin = 0.0
        self.base_ang = 0.0
        self.ee_lin = 0.0
        
        self.bodyparts = [0.0] * len(BODYPARTS) * 4


    def run(self):
        with open(self.name + ".csv", "w", newline='') as csvfile:
            csv_writer = csv.writer(csvfile, delimiter=',',
                                    quotechar='|', quoting=csv.QUOTE_MINIMAL)

            title = ["time (s)", "base (linear)", "base (angular)", "arm (linear)"]
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
                                t = t.to_sec()
                                if self.t0 is None:
                                    self.t0 = t

                                if t - self.t0 >= self.dt * self.i:
                                    csv_writer.writerow([self.dt * self.i] + self.write_csv_line())
                                    self.i += 1

                                if topic == "/status":
                                    self.read_ds4(msg)
                                # elif topic == "/tf":
                                #     for transforms in msg.transforms:
                                #         if transforms.header.frame_id == "odom" or transforms.header.frame_id == "map":
                                #             print(msg)
                                elif topic == "/body_pose":
                                    self.read_pose(msg)

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

    def read_tf(self, msg):
        pass

    def read_pose(self, msg):
        self.bodyparts = []
        for i in range(len(BODYPARTS)):
            self.bodyparts += [
                msg.array[i].x,
                msg.array[i].y,
                msg.array[i].z,
                msg.array[i].confidence
            ]


    def write_csv_line(self):
        row = [
            self.base_lin,
            self.base_ang,
            self.ee_lin
        ]

        row += self.bodyparts

        return row


if __name__ == "__main__":
    name = input("Enter prefix of bag to convert to CSV: ")

    bag_to_csv = BagToCSV(name)
    bag_to_csv.run()