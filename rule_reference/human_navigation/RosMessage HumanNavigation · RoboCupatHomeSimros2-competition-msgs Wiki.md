## ROS 2 Messages for Human Navigation

[](https://github.com/RoboCupatHomeSim/ros2-competition-msgs/wiki/RosMessage-HumanNavigation#ros-2-messages-for-human-navigation)

## Control HSR

[](https://github.com/RoboCupatHomeSim/ros2-competition-msgs/wiki/RosMessage-HumanNavigation#control-hsr)

For ROS 2 messages used to control the robot, please refer to the page below.  
[RosMessage-HSR](https://github.com/RoboCupatHomeSim/ros2-competition-msgs/wiki/RosMessage-HSR)

## 1\. Messages from/to the moderator

[](https://github.com/RoboCupatHomeSim/ros2-competition-msgs/wiki/RosMessage-HumanNavigation#1-messages-fromto-the-moderator)

Basic competition messages are sent and received using the topics below.

| No | Topic Name | Direction | Message Type | Description |
| --- | --- | --- | --- | --- |
| 1 | /human\_navigation/  
message/to\_robot | SIGVerse ⇒ | HumanNaviMsg | Event message from  
Robot to Avatar |
| 2 | /human\_navigation/  
message/to\_moderator | ROS ⇒ | HumanNaviMsg | Event message from  
Avatar to Robot |

**Note:** SIGVerse ⇒: SIGVerse to User's ROS Node, ROS ⇒: User's ROS Node to SIGVerse

```
[human_navigation_msgs/msg/HumanNaviMsg]
string message
string detail
```

For details of event messages please see the following table.

| No | Event | Direction | message | detail |
| --- | --- | --- | --- | --- |
| 1 | Send Are\_you\_ready? | Moderator ⇒ | Are\_you\_ready? | (blank) |
| 2 | Send I\_am\_ready | Robot ⇒ | I\_am\_ready | (blank) |
| 3 | Request of guidance message | Moderator ⇒ | Guidance\_request | (blank) |
| 4 | End of session | Moderator ⇒ | Task\_finished | (blank) |
| 5 | Go to next session | Moderator ⇒ | Go\_to\_next\_session | (blank) |
| 6 | Send Give\_up | Robot ⇒ | Give\_up | (blank) |
| 7 | Give up | Moderator ⇒ | Task\_failed | Give\_up |
| 8 | Time is up | Moderator ⇒ | Task\_failed | Time\_is\_up |
| 9 | Succeeded the task | Moderator ⇒ | Task\_succeeded | (blank) |
| 10 | All tasks finished | Moderator ⇒ | Mission\_complete | (blank) |
| 11 | Request of avatar status | Robot ⇒ | Get\_avatar\_status | (blank) |
| 12 | Request of object status | Robot ⇒ | Get\_object\_status | (blank) |
| 13 | Request of speech state | Robot ⇒ | Get\_speech\_state | (blank) |
| 14 | TTS is running | Moderator ⇒ | Speech\_state | Is\_speaking |
| 15 | TTS is not running | Moderator ⇒ | Speech\_state | Is\_not\_speaking |
| 16 | TTS started an utterance | Moderator ⇒ | Speech\_result | Started |
| 17 | TTS finished an utterance | Moderator ⇒ | Speech\_result | Finished |

**Note:** Moderator ⇒: Moderator to Robot, Robot ⇒: Robot to Moderator

## 2\. Task Information

[](https://github.com/RoboCupatHomeSim/ros2-competition-msgs/wiki/RosMessage-HumanNavigation#2-task-information)

The task information is sent from the moderator. The environment, objects to manipulate, and the destination to place the target object are included in the task information.

| No | Topic Name | Direction | Message Type | Description |
| --- | --- | --- | --- | --- |
| 1 | /human\_navigation/  
message/task\_info | SIGVerse ⇒ | HumanNaviTaskInfo | Information needed to perform the task |

**Note:** SIGVerse ⇒: SIGVerse to User's ROS Node, ROS ⇒: User's ROS Node to SIGVerse

```
[human_navigation_msgs/msg/HumanNaviTaskInfo]
string environment_id
HumanNaviObjectInfo target_object
HumanNaviDestination destination
HumanNaviObjectInfo[] non_target_objects
HumanNaviObjectInfo[] furniture
```

| No | Name | Description |
| --- | --- | --- |
| 1 | environment\_id | Room layout ID |
| 2 | target\_object | Label, position, and orientation of the target object |
| 3 | destination | Anchor position, orientation, and size of designated area to place the target object |
| 4 | non\_target\_objects | Label, position, and orientation of objects excepting the target object |
| 5 | furniture | Label, position, and orientation of furniture objects |

```
[human_navigation_msgs/msg/HumanNaviObjectInfo]
string name
geometry_msgs/Point position
geometry_msgs/Quaternion orientation
```

| No | Name | Description |
| --- | --- | --- |
| 1 | name | Label name of the object |
| 2 | position | Position of the object in the ROS coordinate system |
| 3 | orientation | orientation of the object in the ROS coordinate system |

```
[human_navigation_msgs/msg/HumanNaviDestination]
geometry_msgs/Point position
geometry_msgs/Quaternion orientation
geometry_msgs/Point size
```

| No | Name | Description |
| --- | --- | --- |
| 1 | position | Anchor position of the destination in the ROS coordinate system |
| 2 | orientation | Orientation of the destination in the ROS coordinate system |
| 3 | size | Area size of the destination |

## 3\. Guidance Message

[](https://github.com/RoboCupatHomeSim/ros2-competition-msgs/wiki/RosMessage-HumanNavigation#3-guidance-message)

The natural language instructions are provided audibly and visually to the test subject by sending the guidance message. The number of characters used in the instruction should not exceed 400.  
Although the size of characters in message board effect in Unity is automatically adjusted according to the number of characters, please be careful the size of a text area in the message board effect if you send long sentences with a lot of large characters (i.e., capital letters).  
The robot can give instructions at any time.

| No | Topic Name | Direction | Message Type | Description |
| --- | --- | --- | --- | --- |
| 1 | /human\_navigation/  
message/guidance\_message | Robot ⇒ | HumanNaviGuidanceMsg | Instruction statement and display target |

**Note:** SIGVerse ⇒: SIGVerse to User's ROS Node, ROS ⇒: User's ROS Node to SIGVerse

```
[human_navigation_msgs/msg/HumanNaviGuidanceMsg]
string message
string display_type
string source_language
string target_language
```

| No | Name | Description |
| --- | --- | --- |
| 1 | message | Instruction statement by natural language |
| 2 | display\_type | Target message boards to show the instruction statement for the test subject  
**"All"** : instruction is shown above the robot and in the HMD of the test subject  
**"RobotOnly"** : instruction is shown only above the robot  
**"AvatarOnly"** : instruction is shown only in the HMD  
**"None"** : instructions are provided to the test subject only verbally |
| 3 | source\_language | **Blank("")**.  
Set the ISO-639-1 language code (e.g. "en") if you want to translate, or the blank if you do not.  
PC setup and Internet connection are required for translation. |
| 4 | target\_language | **Blank("")**.  
Set the ISO-639-1 language code (e.g. "ja") if you want to translate, or the blank if you do not.  
PC setup and Internet connection are required for translation. |

## 4\. Avatar Status

[](https://github.com/RoboCupatHomeSim/ros2-competition-msgs/wiki/RosMessage-HumanNavigation#4-avatar-status)

The robot can receive the current status of the avatar by sending "Get\_avatar\_status" message to the moderator.

| No | Topic Name | Direction | Message Type | Description |
| --- | --- | --- | --- | --- |
| 1 | /human\_navigation/  
message/avatar\_status | SIGVerse ⇒ | HumanNaviAvatarStatus | Avatar status |

**Note:** SIGVerse ⇒: SIGVerse to User's ROS Node, ROS ⇒: User's ROS Node to SIGVerse

```
[human_navigation_msgs/msg/HumanNaviAvatarStatus]
geometry_msgs/Pose head
geometry_msgs/Pose body
geometry_msgs/Pose left_hand
geometry_msgs/Pose right_hand
string object_in_left_hand
string object_in_right_hand
bool is_target_object_in_left_hand
bool is_target_object_in_right_hand
```

| No | Name | Description |
| --- | --- | --- |
| 1 | head | Position and orientation of Avatar's head |
| 2 | body | Position and orientation of Avatar's chest (EthanSpine2) |
| 3 | left\_hand | Position and orientation of Avatar's left hand |
| 4 | right\_hand | Position and orientation of Avatar's right hand |
| 5 | object\_in\_left\_hand | Label of the object in the left hand |
| 6 | object\_in\_right\_hand | Label of the object in the right hand |
| 7 | is\_target\_object\_in\_left\_hand | Whether the object in left hand is the target object |
| 8 | is\_target\_object\_in\_right\_hand | Whether the object in right hand is the target object |

## 5\. Object Status

[](https://github.com/RoboCupatHomeSim/ros2-competition-msgs/wiki/RosMessage-HumanNavigation#5-object-status)

The robot can receive the current status of the objects by sending "Get\_object\_status" message to the moderator.

| No | Topic Name | Direction | Message Type | Description |
| --- | --- | --- | --- | --- |
| 1 | /human\_navigation/  
message/object\_status | SIGVerse ⇒ | HumanNaviObjectStatus | Object status |

**Note:** SIGVerse ⇒: SIGVerse to User's ROS Node, ROS ⇒: User's ROS Node to SIGVerse

```
[human_navigation_msgs/msg/HumanNaviObjectStatus]
HumanNaviObjectInfo target_object
HumanNaviObjectInfo[] non_target_objects
```

| No | Name | Description |
| --- | --- | --- |
| 1 | target\_object | Label, position, and orientation of the target object |
| 2 | non\_target\_objects | Label, position, and orientation of objects excepting the target object |