import rosbag
import os

directory = "./"
name = input("Enter prefix of bags to combine: ")

with rosbag.Bag(name + "_combined.bag", "w") as outbag:

    for filename in os.scandir(directory):
        if filename.is_file():
            if os.path.splitext(filename)[-1].lower() == ".bag":
                if filename.name[0:len(name)] == name and filename.name != (name + "_combined.bag"):
                    print("Merging: ", filename.name)

                    total_messages = rosbag.Bag(filename.name).get_message_count()
                    current_message = 0

                    for topic, msg, t in rosbag.Bag(filename.name).read_messages():
                        outbag.write(topic, msg, t)
                        current_message += 1

                        if current_message % 10000 == 0:
                            print("Finished merging: ", current_message, "/", total_messages)

                    print("Finished merging: ", current_message, "/", total_messages)