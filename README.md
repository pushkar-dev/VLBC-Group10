# VLBC Project Group10
Gazebo and ROS simulation of Mars Rover, Reach marker and fetch 

# Setup
1. Install ros-noetic by running- 

    ```sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' & curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add - ```
    
    ```sudo apt install ros-noetic-desktop-full```

2. Install gazebo ```curl -sSL http://get.gazebosim.org | sh ```
3. Install available ros-controllers ```sudo apt install ros-*controller*```
4. cd in the folder and run ```cmake .```
5. To start the simulation and the spawn the rover in the gazebo world ```roslaunch rover rover.launch```
6. 

## Group 10
1. Pushkar Patel B20121
2. Om Kshatriya B20209

    Instructor Dr. Radhe Shyam Sharma
    TA: Mr. Yogesh Namdeo Dasgaonkar

# Objectives 
- [x] Create environment
- [x] Simulate rover movement and Actions
- [ ] Design IBVS Control for rover
- [ ] Simulation of autonomous rover
 

## refrences
1. Inital setup script and environment is taken from [[1]](https://github.com/advaitp/rover)