## ROS 2 Messages for Controlling HSR

[](https://github.com/RoboCupatHomeSim/ros2-competition-msgs/wiki/RosMessage-HSR#ros-2-messages-for-controlling-hsr)

An operated robot is Toyota HSR.  
[https://github.com/ToyotaResearchInstitute/hsr\_description](https://github.com/ToyotaResearchInstitute/hsr_description)  
[https://github.com/ToyotaResearchInstitute/hsr\_meshes](https://github.com/ToyotaResearchInstitute/hsr_meshes)

Operate the robot by subscribing and publishing ROS 2 topics in the table below.

| No | Topic Name | Direction | Message Type | Description |
| --- | --- | --- | --- | --- |
| 1 | /hsrb/command\_ve  
locity | ROS ⇒ | geometry\_msgs/  
Twist | To move Base (base\_footprint) of Robot |
| 2 | /hsrb/head\_trajector  
y\_controller/command | ROS ⇒ | trajectory\_msgs/  
JointTrajectory | To move Head  
(head\_pan\_joint, head\_tilt\_joint) |
| 3 | /hsrb/arm\_trajectory  
\_controller/command | ROS ⇒ | trajectory\_msgs/  
JointTrajectory | To move Arm  
(arm\_lift\_joint, arm\_flex\_joint, arm\_roll\_joint,  
wrist\_flex\_joint, wrist\_roll\_joint) |
| 4 | /hsrb/gripper\_controller/command | ROS ⇒ | trajectory\_msgs/  
JointTrajectory | To move Gripper  
(hand\_l\_proximal\_joint, hand\_r\_proximal\_joint) |
| 5 | /hsrb/joint\_states | SIGVerse ⇒ | sensor\_msgs/  
JointState | State of joints |
| 6 | /hsrb/base\_scan | SIGVerse ⇒ | sensor\_msgs/  
LaserScan | Value of Laser range sensor |
| 7 | /hsrb/head\_rgbd\_sens  
or/rgb/image\_raw | SIGVerse ⇒ | sensor\_msgs/  
Image | Image of Xtion rgb camera |
| 8 | /hsrb/head\_rgbd\_sens  
or/rgb/camera\_info | SIGVerse ⇒ | sensor\_msgs/  
CameraInfo | CameraInfo of Xtion rgb camera |
| 9 | /hsrb/head\_rgbd\_sens  
or/depth\_registered/  
image\_raw | SIGVerse ⇒ | sensor\_msgs/  
Image | Image of Xtion depth camera |
| 10 | /hsrb/head\_rgbd\_sens  
or/depth\_registered/  
camera\_info | SIGVerse ⇒ | sensor\_msgs/  
CameraInfo | CameraInfo of Xtion depth camera |
| 11 | /hsrb/head\_center\_ca  
mera/image\_raw | SIGVerse ⇒ | sensor\_msgs/  
Image | Image of the head camera |
| 12 | /hsrb/head\_center\_ca  
mera/camera\_info | SIGVerse ⇒ | sensor\_msgs/  
CameraInfo | CameraInfo of the head camera |
| 13 | /hsrb/hand\_camera/im  
age\_raw | SIGVerse ⇒ | sensor\_msgs/  
Image | Image of the hand camera |
| 14 | /hsrb/hand\_camera/ca  
mera\_info | SIGVerse ⇒ | sensor\_msgs/  
CameraInfo | CameraInfo of the hand camera |
| 15 | /hsrb/head\_l\_stereo\_ca  
mera/image\_rect\_color | SIGVerse ⇒ | sensor\_msgs/  
Image | Image of the left stereo camera |
| 16 | /hsrb/head\_l\_stereo\_ca  
mera/camera\_info | SIGVerse ⇒ | sensor\_msgs/  
CameraInfo | CameraInfo of the left stereo camera |
| 17 | /hsrb/head\_r\_stereo\_ca  
mera/image\_rect\_color | SIGVerse ⇒ | sensor\_msgs/  
Image | Image of the right stereo camera |
| 18 | /hsrb/head\_r\_stereo\_ca  
mera/camera\_info | SIGVerse ⇒ | sensor\_msgs/  
CameraInfo | CameraInfo of the right stereo camera |

**Note:** SIGVerse ⇒: SIGVerse to User's ROS Node, ROS ⇒: User's ROS Node to SIGVerse

Furthermore, the TF information of robots is distributed at a regular interval.