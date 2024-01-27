# Project Eva
Project "*EVA – Enhanced Visitor Assistant*"  is to design, develop, and manufacture a
sophisticated humanoid robot capable of serving both as an Autonomous Professor and a Visitor Assistant
experiences through a combination of advanced AI technology and automated control systems

## 1 Start
### 1.1 Download

    cd $HOME
    mkdir project/src
    cd project/src
    git clone https://github.com/The-Kriz/project_eva

### 1.2 Build

    colcon build --symlink-install
    source install/setup.bash
    
### 1.2 Run

#### 1.2.1 Run Robot State Publisher
    ros2 launch project_eva rsp.launch.py use_sim_time:=true use_omni_wheel:=false use_drive:=true use_eva:=true


#### 1.2.2 Run Robot Simulation
    ros2 launch project_eva launch_sim.launch.py use_sim_time:=true use_omni_wheel:=true use_drive:=true use_eva:=true

## 2 URDF

### 2.1 Drive
![Drive_RVIZ](https://github.com/The-Kriz/project_eva/assets/90817926/9f931c61-958a-44ea-8d56-4918f714136c)
![Drive_Gazebo](https://github.com/The-Kriz/project_eva/assets/90817926/77d0dbb7-fc24-44be-a96e-c7d7c1400215)

### 2.2 Arm
![Arm_RVIZ](https://github.com/The-Kriz/project_eva/assets/90817926/d6fc030a-5922-4023-88fe-fe54f42359e4)

### 2.3 Fingers
![Fingers_RVIZ](https://github.com/The-Kriz/project_eva/assets/90817926/1fa3b454-476e-470e-9692-c7e8a863c41d)

### 2.4 EVA
![EVA_RVIZ](https://github.com/The-Kriz/project_eva/assets/90817926/73d8d449-0a14-4c71-b97e-e2c0f381ddf0)


## 3 POSE
![Namaste_RVIZ](https://github.com/The-Kriz/project_eva/assets/90817926/a8ccb727-971d-4e16-8bf1-e1d1d1865f27)

![Namaste_Gazebo](https://github.com/The-Kriz/project_eva/assets/90817926/49b6eb77-ddec-40f3-b5f3-dc92ccfeb3f8)




