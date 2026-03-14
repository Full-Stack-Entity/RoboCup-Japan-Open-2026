## ROS 2 Messages for Interactive Cleanup

[](https://github.com/RoboCupatHomeSim/ros2-competition-msgs/wiki/RosMessage-InteractiveCleanup#ros-2-messages-for-interactive-cleanup)

## Control HSR

[](https://github.com/RoboCupatHomeSim/ros2-competition-msgs/wiki/RosMessage-InteractiveCleanup#control-hsr)

For ROS 2 messages used to control the robot, please refer to the page below.  
[RosMessage-HSR](https://github.com/RoboCupatHomeSim/ros2-competition-msgs/wiki/RosMessage-HSR)

## Messages from/to the moderator

[](https://github.com/RoboCupatHomeSim/ros2-competition-msgs/wiki/RosMessage-InteractiveCleanup#messages-fromto-the-moderator)

Basic competition messages are sent and received using the topics below.

| No | Topic Name | Direction | Message Type | Description |
| --- | --- | --- | --- | --- |
| 1 | /interactive\_cleanup/  
message/to\_robot | SIGVerse ⇒ | InteractiveCleanupMsg | Event message  
from Robot  
to Avatar |
| 2 | /interactive\_cleanup/  
message/to\_moderator | ROS ⇒ | InteractiveCleanupMsg | Event message  
from Avatar  
to Robot |

**Note:** SIGVerse ⇒: SIGVerse to User's ROS Node, ROS ⇒: User's ROS Node to SIGVerse

```
[interactive_cleanup_msgs/msg/InteractiveCleanupMsg]
string message
string detail
```

For details of event messages please see the following table.

| No | Event | Direction | message | detail |
| --- | --- | --- | --- | --- |
| 1 | Send Are\_you\_ready? | Avatar ⇒ | Are\_you\_ready? | (blank) |
| 2 | Send I\_am\_ready | Robot ⇒ | I\_am\_ready | (blank) |
| 3 | Send Pick\_it\_up! | Avatar ⇒ | Pick\_it\_up! | (blank) |
| 4 | Send Clean\_up! | Avatar ⇒ | Clean\_up! | (blank) |
| 5 | Send Is\_this\_correct? | Robot ⇒ | Is\_this\_correct? | (blank) |
| 6 | Send Point\_it\_again | Robot ⇒ | Point\_it\_again | (blank) |
| 7 | Send Yes or No | Avatar ⇒ | Yes / No | (blank) |
| 8 | Send Object\_grasped | Robot ⇒ | Object\_grasped | (blank) |
| 9 | Failed Object\_grasped | Avatar ⇒ | Task\_failed | Object\_grasped |
| 10 | Send Task\_finished | Robot ⇒ | Task\_finished | (blank) |
| 11 | Failed Task\_finished | Avatar ⇒ | Task\_failed | Task\_finished |
| 12 | Succeeded the task | Avatar ⇒ | Task\_succeeded | (blank) |
| 12 | All tasks finished | Avatar ⇒ | Mission\_complete | (blank) |
| 13 | Send Give\_up | Robot ⇒ | Give\_up | (blank) |
| 14 | Give up | Avatar ⇒ | Task\_failed | Give\_up |
| 15 | Time is up | Avatar ⇒ | Task\_failed | Time\_is\_up |

**Note:** Avatar ⇒: Human avatar to Robot, Robot ⇒: Robot to Human avatar