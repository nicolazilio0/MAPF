# MAPF
Multi-Agent-Path-Finding algorithms implementation


This repository contains the code for the Robot Planning and its Applicaiton project.
The solutions provided are for tge Coordinated Evactuation and Target Rescue problem.


In order to make this project work you have to do the following: 

Optional: if you want to test the code on a clean environment, you may need to use the docker image we are providing at: https://drive.google.com/file/d/1RmApGyQUg42RH8ruZXtcSOcfCSUhp3EA/view?usp=drive_link

In the docker image, using tmux (that is already installed) you can run 
```
xhost +
sudo docker run --rm -it --name ros2 --network host --env="DISPLAY=$DISPLAY" -v /tmp/.X11-unix:/tmp/.X11-unix:rw -v PATH_TO_CODE_FOLDER:/root/ros2_ws/src pla10/ros2_humble:amd64 /bin/bash
```
to load and run the docker image. The PATH_TO_CODE_FOLDER should be changed according to the directory where the code is.

then build the ros2 packages with 
```
colcon build
source install/setup.bash
```
## COORDINATED EVACUATION

In order to launch the coordinated evacuation you want to launch on a termninal 
```
ros2 launch projects evacuation.launch.py
```
This command will launch the simulation, and in addition it will launch the env_map GUI and the orchestrator node that will run in background
After the simulation is launched then you can decide which one of the solutions to launch
On an other terminal you will run the command:
# VORONOI APPROACH
To run the Voronoi Planner
```
run path_planning voronoi_planner 
```


# RRT* APPROACH
To run the RRT* planner
```
ros2 run path_planning  rrt_star_planner
```

## TARGET RESCUE
In order to launch this problem do as before but this time you want to launch on the first terminal

```
ros2 launch projects victims.launch.py
```
That will launch the simulaiton, the env_map GUI and the orchestrator in background

Then on an other terminal you will launch the planner with 

```
ros2 run path_planning victims_planner
```


For every problem encountered while trying to launch the projects we are available for any clarifications




