# manipulator-utils


This repo contains several modules for the WX250s arm such as
- Interbotix ROS2 packages
- Kinematics module
- Trajectory generators
- Sim2Real diagnostic tool

All ROS2-related packages are incorporated as submodules. Therefore, be sure to clone the repo with the `--recurse-submodules` flag.

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
Results can be observed for the hard-coded triangle trajectory in `sim2real_diff_plotter.ipynb`.

#### ROS2 Dependency for Hardware Recording

If you wish to use `record_hardware.py`, currently, the code requires ROS2 `humble`.
You can download ROS2 quickly using the instructions [here](https://docs.trossenrobotics.com/interbotix_xsarms_docs/ros_interface/ros2/software_setup.html). Those instructions will also download all Interbotix ROS2 packages into `interbotix_ws`, but you can disregard that as we provide the packages (***some with modified source code***) as submodules.

To then run `record_hardware.py`, first run the following in a separate terminal 
```bash
cd interbotix_ws   # from home directory
colcon build       # only need to build once
ros2 launch interbotix_xsarm_control xsarm_control.launch.py robot_model:=wx250s
```
and then `python record_hardware.py`.
You can also try this code with a simulated version of the arm by instead running
```bash
ros2 launch interbotix_xsarm_control xsarm_control.launch.py robot_model:=wx250s use_sim:=true
```

### Troubleshooting
If your system cannot find `Eigen`, you may need to create symlinks if your `Eigen` package is actually named `eigen3/Eigen`.
Simply locate your `Eigen` installation location, `locate eigen`, and then create the necessary symlinks.
For example, if `Eigen` is found in `/usr/include`, then you would do the following:
```bash
cd /usr/include
sudo ln -sf eigen3/Eigen Eigen
```