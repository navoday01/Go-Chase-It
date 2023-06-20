# Go Chase It!



![Alt text](assets/RobotSimulation.gif)
:--:
*Simulation of Robot for chasing white ball*

## Directory Tree 

```
.Go Chase It!                               # Go Chase It Project
├── assets                                  # simulation media
│   ├── RobotSimulation.gif
│   └── RobotSimulation.mp4
├── ball_chaser                             # ball_chaser package
│   ├── CMakeLists.txt                      # compiler instructions
│   ├── include
│   │   └── ball_chaser                  
│   ├── launch                              # launch folder for launch files
│   │   └── ball_chaser.launch
│   ├── package.xml                         # package info
│   ├── src                                 # source folder for C++ scripts
│   │   ├── drive_bot.cpp
│   │   └── process_image.cpp
│   └── srv                                 # service folder for ROS services                   
│       └── DriveToTarget.srv
├── my_ball                                 # white ball model files
│   ├── model.config
│   └── model.sdf
├── my_robot                                # my_robot package
│   ├── CMakeLists.txt                      # compiler instructions
│   ├── launch                              # launch folder for launch files  
│   │   ├── robot_description.launch
│   │   └── world.launch
│   ├── meshes                              # meshes folder for sensors
│   │   ├── bases
│   │   │   └── burger_base.stl
│   │   ├── hokuyo.dae
│   │   ├── RpiCamera.stl
│   │   ├── sensors
│   │   │   ├── astra.dae
│   │   │   ├── astra.jpg
│   │   │   ├── lds.stl
│   │   │   ├── r200.dae
│   │   │   └── r200.jpg
│   │   └── wheels
│   │       ├── left_tire.stl
│   │       └── right_tire.stl
│   ├── package.xml                           # package info
│   ├── urdf                                  # urdf folder for xarco files
│   │   ├── common_properties.xacro
│   │   ├── my_robot.gazebo
│   │   └── turtlebot3_burger.urdf.xacro
│   └── worlds                                # world folder for world files
│       ├── empty.world
│       └── office.world
└── Readme.md                                 # project info

16 directories, 30 files
```
## Compiling and Running the Executable

To run the above project, clone the repository in catkin_ws/src/ 

```
$ cd /home/{name_of_your_workspace}/catkin_ws/src/
$ git clone https://github.com/navoday01/Go-Chase-It
```

Now build and source the project in catkin_ws
```
$ cd /home/{name_of_your_workspace}/catkin_ws/
$ catkin_make
$ source devel/setup.bash
```

Launch my_robot package
```
$ roslaunch my_robot world.launch
```

Launch ball_chaser package in another terminal
```
$ cd /home/{name_of_your_workspace}/catkin_ws/
$ source devel/setup.bash
$ roslaunch ball_chaser ball_chaser.launch
```

To visualize the camera image run following command in another terminal 
```
$ cd /home/{name_of_your_workspace}/catkin_ws/
$ source devel/setup.bash
$ rosrun rqt_image_view rqt_image_view
```




