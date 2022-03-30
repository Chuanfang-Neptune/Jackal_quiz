# Task Description
- Make Jackal go from point A to point B.
- The start is the robot but the end point needs to be an input of your code (e.g. read from a yaml file).
- The trajectory cannot be a straight line or a circular arch (you can get creative here!). (use of an already existing planner or create one yourself)

# Solution Explanation

- Use the MPC to plan a trajectory to optimize the time spent to reach the target given constraints of acceleration and velocity.
- Open-loop trajectory following: follow the planned trajectory from MPC by commanding the motor with control inputs at a given frequency (Control output is a function of time, as according to solution). This method works well at lower speed (vx < 0.2m/s, vw < 0.4 rad/s) but significnatly loses accuracy at higher speed, as the motor could not perfectly reproduce the theoretical control outputs.
- Close-loop trajecotry following: follow the planned trajectory from MPC by following the waypoints and disregarding the time. (Control output is a function of next waypoint on the trajectory). This method takes localization data from the sensor (filtered odometry) or ground truth in Gazebo (assuming that there is an accurate camera looking down on the robot). 2 PID controllers on angular and linear velocities are combined to best follow the waypoints with a curve. The parameters of PID are initialized with N-Z method and edited according to experience. Upon reaching the waypoint, the waypoint is removed from trajectory and next waypoint is poped.

# Code structure

    ├── Jackal_quiz  # Class for jackal_quiz robot
    │   ├── __init__  # initialization of robot states variables 
    │   ├── odometryCb # callback localisation with sensors
    │   ├── gazebo_model_Cb  # callback localisation with gazebo ground truth
    │   ├── l2_tar_distance  # static function for computing l2 error
    │   ├── reg_ang  # static function for computing angle error 
    │   ├── pid_control  # static function for pid control
    │   ├── traj_following # close loop trajectory following
    │   ├── open_loop_traj_following # open loop trajectory following
    ├── quat2euler  # math util for conversion quat2euler
    ├── solveProblem # function for planning trajectory to the target
    ├── cli # arguments parser
    └── __main__ # main program logic

# How to use

```sh
python3 trajectory_solver.py --yaml=target.yml --open_loop=False --gazebo_camera=False
```

| **Arguments**   | **Optinal** | **Type** | **Explanation** | 
| -- | -- | -- | -- |
| yaml | No | String | yml config file name to store target position, dynamic constraints and pid parameters |
| open_loop | Yes | bool | whether to use open-loop control (only in low speed) |
| gazebo_camera | Yes | bool | whether to use data from gazebo as localization input in close-loop control |
# Video

https://youtu.be/StZbzxR1Lu0

# Guide for reproducing results from video

## Create virtual environment on your local machine

```sh
virtualenv -p /usr/bin/<YOUR_PYTHON_DISTRIO> codetestvenv
source codetestvenv/bin/activate
pip3 install pyyaml numpy matplotlib rospkgs casadi 
```

## Bring up the simulation environment
Build the two repositories at:
  - https://github.com/jackal/jackal
  - https://github.com/jackal/jackal_simulator
```sh
roslaunch jackal_gazebo empty_world.launch
```

## Edit the target position in target.yaml

    tar:
        target_pose_x: 1.0 #change this
        target_pose_y: 4.0 #change this

Note:
- A lower speed is needed if using open-loop control. (comment the upper part and uncomment lower part)
- Setting a target that is too far away from the robot would result in a long solving time for the MPC path planner. Good examples of targets are [1.0,4.0], [2.0,4.0], [3.0,4.0], [4.0,4.0]

## Open loop control

```sh
python3 trajectory_solver.py --yaml=openloop.yml --open_loop==True
```

## Close loop control with odometry localization

```sh
python3 trajectory_solver.py --yaml=closeloop_odo.yml --open_loop==False
```

## Close loop control with gazebo ground truth localization

```sh
python3 trajectory_solver.py --yaml=closeloop_gazebo1.yml --gazebo_camera=True --open_loop==False
python3 trajectory_solver.py --yaml=closeloop_gazebo2.yml --gazebo_camera=True --open_loop==False
```

Note that filtered odometry is taking IMU data. If you want to restart experiment, a restart of gazebo is necessary to reset filtered position to the origin.
