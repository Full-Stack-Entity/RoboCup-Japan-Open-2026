The system overview of the Interactive Cleanup is described below.  
下面描述了交互式清理的系统概览。

## System Configuration  系统配置

[](https://github.com/RoboCupatHomeSim/interactive-cleanup-unity2/wiki/SystemOverview#system-configuration)

The system configuration for this competitive challenge is outlined below.  
下面概述了此竞赛挑战的系统配置。

![System](https://github.com/RoboCupatHomeSim/interactive-cleanup-unity2/wiki/images/interactive-cleanup-system.jpg)

The Windows computer runs the Interactive Cleanup program. The Interactive Cleanup program has been created in Unity.  
Windows 计算机运行 Interactive Cleanup 程序。Interactive Cleanup 程序是在 Unity 中创建的。  
The Ubuntu computer runs the rosbridge server, the SIGVerse rosbridge server, and the robot controller created by the competition participants.  
Ubuntu 计算机运行 rosbridge 服务器、SIGVerse rosbridge 服务器和由竞赛参与者创建的机器人控制器。

The Interactive Cleanup program and robot controller communicate through the basic rosbridge server, but communication with a large amount of data (sensor data, etc.) is transmitted through the SIGVerse rosbridge server.  
Interactive Cleanup 程序和机器人控制器通过基础 rosbridge 服务器进行通信，但大量数据（传感器数据等）的通信通过 SIGVerse rosbridge 服务器传输。

In the Interactive cleanup program, robots move in accordance with the instructions from the robot controller when the human avatar issues the Cleanup command to the robot.  
在 Interactive Cleanup 程序中，当人类虚拟形象向机器人发出清理命令时，机器人根据机器人控制器的指令移动。  
The instructions for the Cleanup command are determined based on the operations of the human avatar and the messages sent by the human avatar.  
清理命令的指令是根据人类化身的操作和人类化身发送的消息确定的。

The robot controller sends ROS messages such as Twist and JointTrajectory to the Interactive Cleanup program to move the robot in the Interactive Cleanup program.  
机器人控制器向交互式清理程序发送 Twist 和 JointTrajectory 等 ROS 消息，以在交互式清理程序中移动机器人。

The Interactive Cleanup program distributes JointState, TF, sensor information, and other ROS messages at a regular interval to the Robot Controller.  
交互式清理程序以固定间隔向机器人控制器分发 JointState、TF、传感器信息和其他 ROS 消息。

## Flow of the competition  竞赛流程

[](https://github.com/RoboCupatHomeSim/interactive-cleanup-unity2/wiki/SystemOverview#flow-of-the-competition)

The flow of the competitive challenge is outlined below.  
竞争挑战的流程概述如下。

1.  Launch the robot controller, the SIGVerse rosbridge, etc on Ubuntu side.  
    在 Ubuntu 端启动机器人控制器、SIGVerse rosbridge 等。
2.  Launch the Interactive Cleanup program on Windows side.  
    在 Windows 端启动 Interactive Cleanup 程序。
3.  Initialize the position and direction of the robot and object to grasp.  
    初始化机器人和待抓取物体的位置和方向。
4.  The avatar sends the **"Are\_you\_ready?"** message to the robot.  
    虚拟形象向机器人发送"Are\_you\_ready?"消息。
5.  The robot sends the **"I\_am\_ready"** message to the avatar.  
    机器人向化身发送"I\_am\_ready"消息。
6.  The avatar sends the Cleanup command.  
    化身发送清理命令。
    1.  The avatar moves to point to the object for Cleanup.  
        化身移动到指向要清理的物体的位置。
    2.  The **"Pick\_it\_up!"** message is sent to the robot.  
        "Pick\_it\_up!"消息被发送到机器人。
    3.  The avatar moves to indicate the location of the object for Cleanup.  
        化身移动以指示需要清理的物体位置。
    4.  The **"Clean\_up!"** message is sent to the robot.  
        向机器人发送"Clean\_up!"消息。
    5.  The avatar moves back to initial position.  
        化身移回初始位置。
    6.  The robot can request a re-pointing optionally.  
        机器人可以选择请求重新指向。  
        If the robot sends the **"Point\_it\_again"** message, this step is repeated. (re-pointing)  
        如果机器人发送"Point\_it\_again"消息，则重复此步骤。（重新指向）
7.  The robot moves to grasp the object for cleanup.  
    机器人移动到要清理的物体处进行抓取。
    1.  The robot closes the gripper to clasp the object to grasp.  
        机器人关闭夹爪以夹住物体。
    2.  Confirmation of whether the object is correct can be performed optionally.  
        可以选择性地确认物体是否正确。  
        The robot will send the **"Is\_this\_correct?"** message after grasping the object, and then the avatar responds either **"Yes/No"**. However, a deduction is taken for each confirmation.  
        机器人抓取物体后会发送"Is\_this\_correct?"消息，然后虚拟形象回应"Yes/No"。但是，每次确认都会扣分。
8.  The robot sends the **"Object\_grasped"** message to the avatar.  
    机器人向虚拟形象发送"Object\_grasped"消息。
9.  The robot moves and releases the object for cleanup.  
    机器人移动并释放物体进行清理。
10.  The robot sends the **"Task\_finished"** message to the avatar.  
    机器人向虚拟形象发送"Task\_finished"消息。
11.  The avatar checks the status of Cleanup and assigns points. The task ends.  
    化身检查清理状态并分配分数。任务结束。

-   If the task ends (successfully or unsuccessfully):  
    如果任务结束（成功或失败）：  
    The avatar sends a **"Task\_succeeded"** (successful task) or **"Task\_failed"** (unsuccessful task) message to the robot to start the next task when participants still have attempts left.  
    当参与者还有剩余尝试次数时，化身向机器人发送"Task\_succeeded"（任务成功）或"Task\_failed"（任务失败）消息以启动下一个任务。  
    The avatar sends the **"Mission\_complete"** message to the robot to end the competitive challenge when participants have no attempts left.  
    当参与者没有剩余尝试次数时，化身向机器人发送"Mission\_complete"消息以结束竞争挑战。
-   If the time limit has passed:  
    如果时间限制已过期：  
    The avatar sends the **"Task\_failed"** message to the robot to indicate the task was unsuccessful.  
    化身向机器人发送"Task\_failed"消息，表示任务失败。
-   If users would like to withdraw from the task:  
    如果用户想要退出任务：  
    Users press the Give Up button on the screen to have the avatar send the **"Task\_failed"** message to the robot and indicate the task was unsuccessful.  
    用户按下屏幕上的放弃按钮，让化身向机器人发送"Task\_failed"消息，表示任务失败。
-   Results of Cleanup:  清理结果：  
    The Cleanup will be deemed a success if the object for clean-up is placed on the table or disposed of in the garbage box pointed to by the avatar.  
    如果待清理物体被放置在桌子上或被丢弃到化身指向的垃圾箱中，则清理任务视为成功。
-   The robot can send a **"Give\_up"** message if it is impossible to achieve the task. In that case, the task is aborted, the **"Task\_failed"** message is sent, and the system proceeds to the next session.  
    如果机器人无法完成任务，可以发送"Give\_up"消息。在这种情况下，任务被中止，系统发送"Task\_failed"消息，并进行下一个会话。