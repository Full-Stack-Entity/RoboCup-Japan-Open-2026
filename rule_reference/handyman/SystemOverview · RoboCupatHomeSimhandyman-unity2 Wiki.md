The system overview of the Handyman is described below.

## System Configuration

[](https://github.com/RoboCupatHomeSim/handyman-unity2/wiki/SystemOverview#system-configuration)

The system configuration for this competitive challenge is outlined below.

![System](https://github.com/RoboCupatHomeSim/handyman-unity2/wiki/images/handyman-system.jpg)

The Windows computer runs the Handyman app. The Handyman app has been created in Unity.  
The Ubuntu computer runs the rosbridge server, the SIGVerse rosbridge server, and the robot controller created by the competition participants.

The Handyman app and robot controller communicate through the basic rosbridge server, but communication with a large amount of data (sensor data, etc.) is transmitted through the SIGVerse rosbridge server.

In the Handyman app, robots move in accordance with the instructions from the robot controller when the human avatar issues commands to the robot. The robot controller sends ROS messages such as Twist and JointTrajectory to the Handyman app to move the robot in the Handyman app.

The Handyman app distributes JointState, TF, sensor information, and other ROS messages at a regular interval to the Robot Controller.

## Flow of the competition

[](https://github.com/RoboCupatHomeSim/handyman-unity2/wiki/SystemOverview#flow-of-the-competition)

The flow of the competition is outlined below.

1.  Launch the robot controller, the SIGVerse rosbridge, etc. on the Ubuntu side.
2.  Launch the Handyman app on Windows side.
3.  Initialize the position and direction of the robot and object to grasp.
4.  The avatar sends the **"Are\_you\_ready?"** message to the robot, and the **"Environment"** message is sent at the same time.
5.  The robot sends the **"I\_am\_ready"** message to the avatar.
6.  The avatar sends **"Instruction"** message to the robot.  
    (e.g.: Go to the XXXX, grasp the YYYY and bring it here.)
7.  The robot moves to the room for instruction.
8.  The robot sends the **"Room\_reached"** message to the avatar.
9.  The avatar checks the first statement.  
    If successful, points are awarded and the challenge moves on to the next task.  
    If unsuccessful, the task ends.
10.  The robot looks for the object.  
    If the robot can find the target object, the robot should progress to grasp the object.  
    If the robot cannot find the target object and judges that the object does not exist, the robot should send the **"Does\_not\_exist"** message to the avatar.
    1.  If the indication is correct, a score is added and the avatar sends **"Corrected\_instruction"** message to the robot.  
        The robot then should look for the new object.
    2.  If the indication is not correct, the task will end.
11.  The robot grasps the object.
12.  The robot sends the **"Object\_grasped"** message to the avatar.
13.  The avatar checks whether the grasped object is correct.  
    If it is successful, a score is added.  
    If it is unsuccessful, the task ends and the session moves on to the next one.
14.  The robot carries out the instruction after grasping.
15.  The robot sends the **"Task\_finished"** message to the avatar.
16.  The avatar checks the achievement condition of the target behavior.  
    If it is successful, a score is added and the session goes to the next.

-   If the task ends (successfully or unsuccessfully): The avatar sends a **"Task\_succeeded"** (successful task) or **"Task\_failed"** (unsuccessful task) message to the robot to start the next task when participants still have attempts left.  
    The avatar sends the **"Mission\_complete"** message to the robot to end the competitive challenge when participants have no attempts left.
-   If the time limit has passed:  
    The avatar sends the **"Task\_failed"** message to the robot to indicate the task was unsuccessful.
-   The robot can send **"Give\_up"** message if it is impossible to achieve the task. In that case, the task is aborted, the **"Task\_failed"** message is sent, and the session moves on to the next one.

## Notes

[](https://github.com/RoboCupatHomeSim/handyman-unity2/wiki/SystemOverview#notes)

## Destination of Grasped Objects

[](https://github.com/RoboCupatHomeSim/handyman-unity2/wiki/SystemOverview#destination-of-grasped-objects)

Basically please put the grasped object on a desk or the like.  
However, if the destination is a person, move the arm so that the center of the grasped object is within the sphere of the following image.

![destinationSphere](https://github.com/RoboCupatHomeSim/handyman-unity2/wiki/images/destination-sphere-of-human-avatar.jpg)