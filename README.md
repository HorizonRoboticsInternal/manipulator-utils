# manipulator-utils


This repo contains several modules for the WX250s arm such as
- Kinematics module
- Trajectory generators
- Sim2Real diagnostic tool

### Dependencies

The kinematics module is coded in C++ and has Python bindings through pybind. 

Download the `Eigen` development package and `pybind11`.
```bash
sudo apt install libeigen3-dev
pip install pybind11
```

### How to Run

All diagnostic-related code can be found in the `diagnostic_tool` directory. This directory also contains the `kinematics_cpp` pybind module.

#### Kinematics Module
First, build the kinematics module by carrying out the following:
```bash
cd diagnostic_tool
mkdir build && cd build && cmake ..
make -j4
```
To test if the module has built successfully, you can run some provided unit tests.
```bash
cd kinematics_cpp          # from diagnostic_tool
python test_kinematics.py  # tests FK and IK functions
```

#### Trajectory Generation and Sim / Hardware Recording
Afterward, you can take a look at how some simple Cartesian trajectories can be created in `generate_joint_trajectories.py`.
Recording joint angles when carrying out trajectories can be done in both MuJoCo and hardware in `record_sim.py` and `record_hardware.py`, respectively.
A simple plotter for the hard-coded triangle trajectory can be seen in `sim2real_diff_plotter.py`.

#### ROS2 Dependency for Hardware Recording

If you wish to use `record_hardware.py`, currently, the code relies on ROS2 packages provided by Interbotix. A major TODO is to remove this dependency.
For now though, if you would like to use the code in the current state, you will have to download ROS2 using the instructions [here](https://docs.trossenrobotics.com/interbotix_xsarms_docs/ros_interface/ros2/software_setup.html). Those instructions will download a ROS2 package `interbotix_ws`, but you can disregard that as we provide one (***with some modified source code***) in this repo.

To then run `record_hardware.py`, first run the following in a separate terminal and then `python record_hardware.py`.
```bash
cd interbotix_ws   # from home directory
colcon build       # only need to build once
ros2 launch interbotix_xsarm_control xsarm_control.launch.py robot_model:=wx250s
```

### Troubleshooting
If your system cannot find `Eigen`, you may need to create symlinks if your `Eigen` package is actually named `eigen3/Eigen`.
Simply locate your `Eigen` installation location, `locate eigen`, and then create the necessary symlinks.
For example, if `Eigen` is found in `/usr/include`, then you would do the following:
```bash
cd /usr/include
sudo ln -sf eigen3/Eigen Eigen
```