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
    ros2 launch project_eva rsp.launch.py
    
#### 1.2.2 Run Robot Simulation
    ros2 launch project_eva launch_sim.launch.py 


## 2 Rviz

### 2.1 Drive
![rviz](https://github.com/The-Kriz/project_eva/assets/90817926/217f6b19-c6b4-49b9-a392-6fd262fec37f)
