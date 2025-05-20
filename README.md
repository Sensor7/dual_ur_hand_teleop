# dual ur hand teleoperation

This repository implemented dual pandas robot teleoperated by human hand movements and gestures. You can use RGB camera to control the robot.

- **Thumb up:** activate servo and start teleoperation

- **Thumb down:** stop servo and teleoperation

- **Close Fist:** close the two finger gripper

- **Open palm:** open the two finger gripper

And this repository is mainly built from [FrankaTeleop](https://github.com/gjcliff/FrankaTeleop), which is an open source repository already achieve nice depth camera hand gesture teleoperation. But this package is built on ros2 iron, and there is no so many suports about moveit servo in ros2 humble. So I adapted above repository to dual franka panda arm first and added the python wrapper to control robot.

And the UR3e robot description is from official Universal Robot Description, I adapt it to dual robot and add prefix. And build the moveit2 config using moveit2 setup assistant

And because ros2 humble is not yet integrated with moveit python interface, so I use [arm_api2](https://github.com/CroboticSolutions/arm_api2) as wrapper to control robot arm and wrote a python implementation of the arm api2 for easier usage.

# Install

1. install ros2 humble

2. install moveit2 and ros2 control

``` bash
sudo apt install ros-humble-moveit*
sudo apt install ros-humble-tf-transformations
```

3. install mediapipe, torch and torch vision
```bash
pip install mediapipe
pip install torch==2.5.1 torchvision==0.20.1 torchaudio==2.5.1 --index-url https://download.pytorch.org/whl/cu121
pip install transformers
pip install --upgrade transforms3d
```
**if you don't have cuda gpu, remember to deactivate the DL monocular camera depth estimation module, otherwise it will run very slow**

4. RGB Camera and usb_camera install
```bash
sudo apt install ros-humble-usb-cam
```

5. Install robotiq description and its driver
```bash
sudo apt install ros-humble-robotiq-description
```

# build and run

```bash
colcon build 
source install/setup.bash
```

Remember to source again in following steps

## start robot and moveit and also arm api2

```bash
ros2 launch moveit2_ur3e_config demo_my.launch.py
# In other three terminals
ros2 launch arm_api2 moveit2_iface.launch.py robot_name:=ur_left
ros2 launch arm_api2 moveit2_iface.launch.py robot_name:=ur_right
ros2 launch ur3e_robot_description display.launch.py # Use for viewing robot and also the camera image and marker instructions
```

## start camera and also handcv
```bash
ros2 launch handcv camera.launch.py
```

## start teleoperation
```bash
ros2 run cv_franka_bridge cv_franka_bridge
