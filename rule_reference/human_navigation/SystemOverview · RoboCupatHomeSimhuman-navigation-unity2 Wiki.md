An overview of the Human Navigation task is provided below.

## System Configuration

[](https://github.com/RoboCupatHomeSim/human-navigation-unity2/wiki/SystemOverview#system-configuration)

The Windows computer runs the Human Navigation app. The Human Navigation app has been created in Unity. The Ubuntu computer runs the rosbridge server, the SIGVerse rosbridge server, and the robot controller created by the competition participants.

The Human Navigation app and robot controller communicate via the rosbridge server, but high-bandwidth data (e.g., sensor data) is transmitted via the SIGVerse rosbridge server.

![SIGVerse with VR Device](https://github.com/RoboCupatHomeSim/human-navigation-unity2/wiki/images/sigverse_architecture_wiki.png)

## Flow

[](https://github.com/RoboCupatHomeSim/human-navigation-unity2/wiki/SystemOverview#flow)

The flow of the competitive challenge is outlined below.

1.  A team member launches the ROSBridge server, SIGVerse ROSBridge server, and robot controller.
2.  The operator (Competition committee) launches human\_navi.exe
3.  The test subject puts on the Meta Quest 2.
4.  The operator presses the session start button.
5.  The app loads the environment for the competitive challenge to initialize the position and direction of the avatar.
6.  Moderator sends the "Are\_you\_ready?" message to the robot controller.
7.  The robot controller sends the "I\_am\_ready" message to the moderator.
8.  The moderator sends the _TaskInfo_ to the robot. The _TaskInfo_ includes the target object and destination.
9.  The robot controller generates a _guidance\_message_ (a spoken instruction) and sends it to the test subject. The robot controller can send a _guidance\_message_ at any time.
10.  The test subject follows the spoken instructions to pick up the target object and place it at the designated location (destination). The test subject can request a guidance message by pressing the X button. After the test subject requested a new guidance message, the "Guidance\_request" message is sent to the robot controller.
    -   If the target object is grasped, points will be awarded.
    -   If the wrong object is grasped, a penalty will be given.
    -   If the target object is placed in/on the destination, points will be awarded.
    -   If the number of instructions exceeds 15, a penalty will be given.
11.  Each session ends when one of the following events occurs.
    -   The target object is placed in/on the destination: The Moderator sends the "Task\_succeeded" message to the robot controller.
    -   "Give\_up" is sent from the robot controller: The robot can send "Give\_up" message if it is impossible to achieve the task. In that case, the session is aborted and the Moderator sends the "Task\_failed" message to the robot controller.
    -   Time is up: The Moderator sends the "Task\_failed" message to the robot controller.
12.  Session ends.
    -   If no sessions are left, the moderator sends "Mission\_complete" message to the robot controller.
    -   If any sessions are left:
        1.  The test subject moves to the next team area and puts on the Meta Quest 2.
        2.  The operator presses the Go\_to\_next\_session button. The Moderator sends "Go\_to\_next\_session" message to the robot controller.
        3.  Return to step 4.