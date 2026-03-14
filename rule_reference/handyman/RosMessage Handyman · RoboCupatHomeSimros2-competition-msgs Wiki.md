## ROS 2 Messages for Handyman

[](https://github.com/RoboCupatHomeSim/ros2-competition-msgs/wiki/RosMessage-Handyman#ros-2-messages-for-handyman)

## Control HSR

[](https://github.com/RoboCupatHomeSim/ros2-competition-msgs/wiki/RosMessage-Handyman#control-hsr)

For ROS 2 messages used to control the robot, please refer to the page below.  
[RosMessage-HSR](https://github.com/RoboCupatHomeSim/ros2-competition-msgs/wiki/RosMessage-HSR)

## Messages from/to the moderator

[](https://github.com/RoboCupatHomeSim/ros2-competition-msgs/wiki/RosMessage-Handyman#messages-fromto-the-moderator)

Basic competition messages are sent and received using the topics below.

| No | Topic Name | Direction | Message Type | Description |
| --- | --- | --- | --- | --- |
| 1 | /handyman/message/  
to\_robot | SIGVerse ⇒ | HandymanMsg | Event message from  
Robot to Avatar |
| 2 | /handyman/message/  
to\_moderator | ROS ⇒ | HandymanMsg | Event message from  
Avatar to Robot |

**Note:** SIGVerse ⇒: SIGVerse to User's ROS Node, ROS ⇒: User's ROS Node to SIGVerse

```
[handyman_msgs/msg/HandymanMsg]
string message
string detail
```

For details of event messages please see the following table.

| No | Event | Direction | message | detail |
| --- | --- | --- | --- | --- |
| 1 | Send Are\_you\_ready? | Avatar ⇒ | Are\_you\_ready? | (blank) |
| 2 | Send Environment info | Avatar ⇒ | Environment | Environment name  
(e.g.: Environment\_01)  
or  
"unknown" |
| 3 | Send I\_am\_ready | Robot ⇒ | I\_am\_ready | (blank) |
| 4 | Send a task message | Avatar ⇒ | Instruction | task message(e.g.: Go to the XXXX, grasp the YYYY and bring it here.) |
| 5 | Send Room\_reached | Robot ⇒ | Room\_reached | (blank) |
| 6 | Failed Room\_reached | Avatar ⇒ | Task\_failed | Room\_reached |
| 7 | Send Does\_not\_exist | Robot ⇒ | Does\_not\_exist | (blank) |
| 8 | Failed Does\_not\_exist | Avatar ⇒ | Task\_failed | The target exist |
| 9 | Send a corrected task message | Avatar ⇒ | Corrected\_instruction | task message(e.g.: Go to the XXXX, grasp the ZZZZ and bring it here.) |
| 10 | Send Object\_grasped | Robot ⇒ | Object\_grasped | (blank) |
| 11 | Failed Object\_grasped | Avatar ⇒ | Task\_failed | Object\_grasped |
| 12 | Failed Object\_grasped  
(the robot sent the message  
when the target  
doesn't exist) | Avatar ⇒ | Task\_failed | The target doesn't exist |
| 13 | Send Task\_finished | Robot ⇒ | Task\_finished | (blank) |
| 14 | Failed Task\_finished | Avatar ⇒ | Task\_failed | Task\_finished |
| 15 | Succeeded the task | Avatar ⇒ | Task\_succeeded | (blank) |
| 16 | All tasks finished | Avatar ⇒ | Mission\_complete | (blank) |
| 17 | Send Give\_up | Robot ⇒ | Give\_up | (blank) |
| 18 | Give up | Avatar ⇒ | Task\_failed | Give\_up |
| 19 | Time is up | Avatar ⇒ | Task\_failed | Time\_is\_up |

**Note:** Avatar ⇒: Human avatar to Robot, Robot ⇒: Robot to Human avatar