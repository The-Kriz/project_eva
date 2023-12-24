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

## 2 Rviz

### 2.1 Drive
![rviz](https://github.com/The-Kriz/project_eva/assets/90817926/217f6b19-c6b4-49b9-a392-6fd262fec37f)

### 2.2 Arm
![Arm](https://github.com/The-Kriz/project_eva/assets/90817926/d6fc030a-5922-4023-88fe-fe54f42359e4)

### 2.3 Fingers
![Screenshot from 2023-12-24 17-36-03](https://github.com/The-Kriz/project_eva/assets/90817926/1fa3b454-476e-470e-9692-c7e8a863c41d)

### 2.4 EVA
![Screenshot from 2023-12-17 13-25-06](https://github.com/The-Kriz/project_eva/assets/90817926/17624510-85cd-474a-9a9d-8b34483601e0)

