> [!NOTE]
> This repository is for ROS 2.


# Interactive Cleanup Project

This is a Unity project for the Interactive Cleanup task.

In this task, the robot grasps an object located at a place pointed to by a human avatar, and then delivers it to another place the avatar points to.  
This task focuses on recognizing human actions.

See also [wiki page](https://github.com/RoboCupatHomeSim/interactive-cleanup-unity2/wiki).

## Prerequisites

For the Unity version and other requirements, please refer to the following.  
https://github.com/RoboCupatHomeSim/overview/blob/master/Environment.md#windows-pc

Please prepare a common unitypackage in advance.  
For details on how to create the common unitypackage, please see the introduction in the following repository.  
https://github.com/RoboCupatHomeSim/common-unity2

Please prepare Text-to-Speech DLLs.  
For details, please see the following repository.  
https://github.com/RoboCupatHomeSim/console-simple-tts

## How to Build

### Import the Common Unitypackage

1. Open this project with Unity.
1. Click [**Ignore**] in the [Enter Safe Mode?] window.
1. Click [Assets]-[Import Package]-[Custom Package...].
1. Select a common unitypackage (e.g. robocup-common.unitypackage) and open the file.
1. Click [Import] button.
1. Click [Assets]-[**Reimport All**].
1. Click [**Reimport**] button.
1. Please confirm that no error occurred in Console window.

### Import the Text-to-Speech DLLs.

Copy **ConsoleSimpleTTS.exe** and **Interop.SpeechLib.dll** to the "TTS" folder next to the SIGVerseConfig folder.

## Modify Configuration

1. Open this project with Unity.
1. Click [SIGVerse]-[SIGVerse Settings].  
SIGVerse window will be opened.
1. Enter the Ubuntu PC's IP address in "Rosbridge IP".
1. Close SIGVerse window.

### Build
1. Create a "Build" folder under this project folder.
1. Open this project with Unity.
1. Click [File]-[Build Profiles].
1. Click [Build].
1. Select the "Build" folder.  
The build will start.
1. Copy the "TTS" folder into the "Build" folder.

## How to Run

Please start the ROS side application beforehand.  
For the ROS side, please see the following repository.  
https://github.com/RoboCupatHomeSim/ros2-competition-msgs

### Execute the Executable file
1. Double Click the "InteractiveCleanup.exe" in the "Build" folder.

### Execute on Unity Editor
1. Click [SIGVerse]-[Set Default GameView Size].
2. Double click "Assets/Competition/InteractiveCleanup/InteractiveCleanup(.unity)" in Project window.
3. Click the Play button at the top of the Unity editor.

## Task Creation
A VR device is required to generate the tasks.  
Please see [wiki page](https://github.com/RoboCupatHomeSim/interactive-cleanup-unity2/wiki) for more details.  

