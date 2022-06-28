# Fetch Teleoperation

### Launch mapping (Fetch)

    roslaunch fetch_navigation fetch_nav.launch map_file:=/home/hrigroup/lounge.yaml


### MoveIt! to initial configuration (Fetch)

    source kerryaswell_ws/devel/setup.bash
    roslaunch move_pose move_arm.launch


### Launch control and recording (Fetch)

    holistart
    source kerry_ws/devel/setup.bash
    roslaunch fetch_teleop main.launch id:=**\<ID\>**
    holistop


### Launch global camera on local machine (Local)

    source fetch_ws/devel/setup.bash
    roslaunch pose_estimation main.launch id:=**\<ID\>**


### Mount/Unmount HDD  (Fetch)

    sudo mount /dev/sdb1 /media/hdd
    sudo umount /media/hdd


### Visualize (Local)

If you want to visualise the Fetch model in RViz from ROS bag data

    sudo apt-get install ros-melodic-fetch-description
    rosparam set robot_description -t /opt/ros/melodic/share/fetch_description/robots/fetch.urdf


### Replay data (Fetch)

Data that can be replayed is of the same form of the CSV produced from `bag_to_csv.py` (see `../data/`). For example, you can choose data from a single episode an make a CSV using that data to replay robot motions from that episode.

    source kerry_ws/devel/setup.bash
    roslaunch fetch_teleop replay.launch repeat:=true

When prompted (it might be hard to see the prompt under all the other console text), input the prefix of the file you want to play the data from. 

You can press the **L1** button to toggle on and off pause.


# Controls

## Navigate Mode
Navigate Mode is activated by holding down the **L1** button. When released, or when an obstacle is detected in front of the Fetch's LiDAR, the robot will stop completely. 

### Base Control
By default, Navigate Mode will be in Base Control Mode. To move the robot along the predefined trajectory, use the **Right Joystick** by pushing forwards and backwards. To rotate the base, use the **Left Joystick** by pushing left and right.

The base will move and rotate at a set velocity. To change this velocity, hold down either the right joystick (**R3**) or left joystick (**L3**) while moving the base to vary the linear or angular velocity respectively. The new set velocities are changed to the latest velocity set while the joysticks were pressed.

### Arm Control
To activate Arm Control Mode, hold down the **R1** button. To move the robot towards the predefined waypoint, use the **Right Joystick** by pushing forwards and backwards. 

There are 3 discrete arm waypoints that have been predefined. Use **Square** to change to the left-handed waypoint, **Triangle** to change to the straight-on waypoint, and **Circle** to change to the right-handed waypoint. Note that these can only be changed between when the arm is tucked.

The arm can be rotated around to position itself for a left-handed or right-handed handover. To do this, use the **D-Pad Left/Right** buttons. 

The arm will move at a set velocity. To change this velocity, hold down the right joystick (**R3**) while moving the arm to vary the velocity. The new set velocity is changed to the latest velocity set while the joystick was pressed.

## Edit Mode

To activate Edit Mode, hold down the **L2** button while also holding down the **L1** button. To save all parameters to the .yaml file, press the **Options** button. 

### Base parameters
While in Edit Mode, the Base Control will allow complete freedom over the control of the Fetch's base.  Linear motion is controlled using the **Right Joystick**, while angular motion is controlled using the **Left Joystick**. The position the base moves to while in Edit Mode defines the new terminal endpoint of the predefined trajectory. This change is reflected symmetrically on the other end of the trajectory.

The base can also be changed by publishing a point in RViz. The closest waypoint to the published point will change to the published point's position.

### Arm parameters
While in Edit Mode, the Arm Control will instead modify the goal position of the end-effector. Linear motion in the $xy$-plane is controlled using the **Right Joystick**, linear motion in the $z$-axis is controlled using the **D-Pad Up/Down** buttons, and pitch and yaw of the end-effector is controlled using the **Left Joystick**. The edited goal position will be saved to the left/straight/right handover mode the robot is currently on.

### General
Parameters are saved in a `.yaml` file, and loaded up when the script is run. To save parameters which have been changed, press the **Option** button. To reset all settings to those currently in the `.yaml` file, press the **Share** button.


## Other
To indicate whether a handover is a good or bad sample, press the **Circle** button to indicate a good handover, or **Cross** to indicate a bad handover.

To indicate what type of handover the robot is performing, press the **L2** trigger to indicate a robot-to-human handover, or **R2** to indicate a human-to-robot handover.

