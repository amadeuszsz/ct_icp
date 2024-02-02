![CT_ICP_LOGO](./doc/CT_ICP_About.png)

![](https://github.com/pierdell/gifs/blob/master/ct_icp_main.png)

This repository implements the SLAM **CT-ICP** (see  [our article](https://arxiv.org/abs/2109.12979)), a lightweight,
precise and versatile pure LiDAR odometry.

The code can be run with **ROS**, but also as an independent library, or using scripts we provide.

It is integrated with the python project **[pyLiDAR-SLAM](https://github.com/Kitware/pyLiDAR-SLAM)** which gives access
to more datasets.
**pyLiDAR-SLAM** requires the installation of the python binding for **CT-ICP** (see below).

![](https://github.com/pierdell/gifs/blob/master/ct_icp_NCLT.GIF)
![](https://github.com/pierdell/gifs/blob/master/ct_icp_UrbanLoco.GIF)

## NEWS:

##### [27/07/2022] New release, with increased ROS support

> We introduce a new release, with significant changes in the code. We do not guarantee the results of the article in
> this branch, (though this branch should globally our SLAM). To replicate the results from the dataset, see the
> release `ICRA-2022`
>

# Installation

### Requirements

> Compiler: GCC >= 7.5, clang >= 8.01
>
> cmake >= 3.14

##### Tested On:

| OS           | COMPILER    |
| ------------ | ----------- |
| Ubuntu 22.04 | GCC >= 11.4 |

### Step 0: Clone the directory

```bash
git clone https://github.com/amadeuszsz/ct_icp.git
cd ct_icp
```

### Step 1: Superbuild

> CT-ICP uses **Kitware**'s [**
Superbuild**](https://gitlab.kitware.com/keu-computervision/MappingResearchKEU/Superbuild) to build the external
> dependencies.
>
> You can either install the external dependencies, or use the script below to install all dependencies:

```bash
mkdir .cmake-build-superbuild && cd .cmake-build-superbuild     #< Creates the cmake folder
cmake ../superbuild                                             #< (1) Configure step 
cmake --build . --config Release                                #< Build step (Downloads and install the dependencies), add -DWITH_VIZ3D=ON to install with the GUI
```
> /!\ If you want to build the visualization do not forget to add `-DWITH_VIZ3D=ON`


> If everything worked, a directory `install` should have been created with at its root a `superbuild_import.cmake`
> file.

### Step 2: Build and install CT-ICP library

```bash
# Inside the main directory
mkdir cmake-build-release && cd  cmake-build-release                  #< Create the build directory
cmake .. -DCMAKE_BUILD_TYPE=Release                                   #< (2) Configure with the desired options (specify arguments with -D<arg_name>=<arg_value>), add -DWITH_VIZ3D=ON to install with the GUI
cmake --build . --target install --config Release --parallel 12       #< Build and Install the project
```

> /!\ If you want to build the visualization do not forget to add `-DWITH_VIZ3D=ON`


> If everything worked fine, a `CT_ICP` subdirectory should appear in your **Superbuild Directory**.
> You can use the config files located at `<SUPERBUILD_INSTALL_DIR>/CT_ICP/lib/cmake` to load the libraries in a cmake
> project, or use ROS or the specified executables.

### Step 3: ROS 2

To build the ROS 2 wrapping for **CT-ICP**, first build and install the CT-ICP library (see *Steps 1 and 2*).

> Make a symbolic link of the directory `ct_icp_odometry` of this project to the `src` directory
> of your ROS 2 workspace.

```bash
cd <path-to-your-ros2-workspace>/src                              #< Move to the ROS 2 Workspace's src directory
ln -s <path-to-ct_icp-git-project>/ros/ct_icp_odometry ct_icp_odometry        #< Make a symbolic link to the ROS2 workspace folder
cd ..                                                               #< Move back to the root of the ROS 2 workspace
colcon build --symlink-install --packages-up-to ct_icp_odometry --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=On -DSUPERBUILD_INSTALL_DIR=<path-to-superbuild-install-dir>
```

> If the installation is successful, and after sourcing the workspace's devel directory, you should be able to launch
> the ROS Nodes installed.

> The wrapping defines the following nodes:

- `ct_icp_odometry_node`: The main odometry node running `ct_icp`'s odometry.

```
ros2 launch ct_icp_odometry ct_icp_odometry.launch.py input_cloud_topic:=/your/pointcloud/topic
```
> You can change input cloud topic in Input/Cloud rviz panel to see the actual input.

> See launch file for more details on the parameters.

[//]: # (### Visualization [experimental])

[//]: # ()

[//]: # (> As a debugging/visualization tool we use a home-made/experimental lightweight OpenGL-based pointcloud visualizer **[viz3d]&#40;https://github.com/pierdell/viz3d&#41;** designed for our SLAM use case.)

[//]: # (> )

[//]: # (> To activate pass the argument `-DWITH_VIZ3D=ON` to the configure steps of the `Superbuild &#40;1&#41;`, `CT_ICP &#40;2&#41;` )

# Install the Datasets

### CT-ICP Datasets from the article

The Datasets are publicly available at:
https://cloud.mines-paristech.fr/index.php/s/UwgVFtiTOmrgKp5

The folder is protected by the following password : **npm3d**

Each dataset is a .zip archive containing the PLY scan file with the relative timestamps for each point in the frame,
and if available, the ground truth poses.

To install each dataset, simply download and extract the archives on disk. The datasets are redistributions of existing
and copyrighted datasets, we only offer a convenient repackaging of these datasets.

The dataset available are the following:

**Under Creative Commons Attribution-NonCommercial-ShareAlike LICENCE**

- *KITTI* (see [eval_odometry.php](http://www.cvlibs.net/datasets/kitti/eval_odometry.php)):
    - The most popular benchmark for odometry evaluation.
    - The sensor is a Velodyne HDL-64
    - The frames are motion-compensated (no relative-timestamps) and the Continuous-Time aspect of CT-ICP will not work
      on this dataset.
    - Contains 21 sequences for ~40k frames (11 with ground truth)
- *KITTI_raw* (see [eval_odometry.php](http://www.cvlibs.net/datasets/kitti/eval_odometry.php)): :
    - The same dataset as *KITTI* without the motion-compensation, thus with meaningful timestamps.
    - The raw data for sequence `03` is not available
- *KITTI_360* (see [KITTI-360](http://www.cvlibs.net/datasets/kitti-360/)):
    - The successor of *KITTI*, contains longer sequences with timestamped point clouds.
    - The sensor is also a Velodyne HDL-64

**Permissive LICENSE**

- *NCLT*: (see [nclt](http://robots.engin.umich.edu/nclt/))
    - Velodyne HDL-32 mounted on a segway
    - 27 long sequences (up to in the campus of MICHIGAN university over a long
    - Challenging motions (abrupt orientation changes)
    - **NOTE**: For this dataset, directly download the *Velodyne* links (
      e.g. [2012-01-08_vel.tar](http://robots.engin.umich.edu/nclt/velodyne_data/2012-01-08_vel.tar.gz)). Our code
      directly reads the *velodyne_hits.bin* file.
- *KITTI-CARLA*: (see and cite [KITTI-CARLA](https://arxiv.org/abs/2109.00892)):
    - 7 sequences of 5000 frames generated using the [CARLA](https://carla.readthedocs.io/en/0.9.10/) simulator
    - Imitates the KITTI sensor configuration (64 channel rotating LiDAR)
    - Simulated motion with very abrupt rotations
- *ParisLuco* (published with our work **CT-ICP**, cf below to cite us):
    - A single sequence taken around the Luxembourg Garden
    - HDL-32, with numerous dynamic objects

### Download ROSBAGS to run the SLAM with ROS

TODO: Provide ROS 2 bag files.

## Running the SLAM

### OPTION I -- Using the scripts (on the *ct-icp* datasets)

If the installation of CT-ICP went fine, there should be an executable located
at `<CT_ICP_INSTALL_DIR>/bin/run_odometry`.
This executable can be run with a config file with the command:

```./run_odometry -c <path-to-config-file>```

See `./config/odometry/driving_config.yaml` for an example of the format of the config file to expect.

If `CT-ICP` was installed with `viz3d`, the SLAM should run along a GUI, otherwise, the trajectory and metrics will be
saved to disk regularly.

### OPTION II -- Using the SLAM as a library

After the installation, you can also use `CT_ICP` and `SlamCore` libraries, located in `<CT_ICP_INSTALL_DIR>/lib`,
for instance with a cmake project with the cmake config files for the libraries located
at `<CT_ICP_INSTALL_DIR>/lib/cmake`.

See for example `command/cmd_run_odometry.cpp` and `command/odometry_runner.h` for an example of use.

#### Custom Datasets

Some datasets are defined in the library (with expected layout for the Data, see `dataset.h, dataset.cpp`), but you can
extend
`ct_icp::ADatasetSequence` to define your own custom datasets.

### OPTION III -- Using ROS 2

After completing the ROS 2 installation, play rosbag and use the launch file defined in `ros/your_workspace/ct_icp_odometry/launch`.

## Citation

If you use our work in your research project, please consider citing:

```
@misc{dellenbach2021cticp,
  title={CT-ICP: Real-time Elastic LiDAR Odometry with Loop Closure},
  author={Pierre Dellenbach and Jean-Emmanuel Deschaud and Bastien Jacquet and François Goulette},
  year={2021},
  eprint={2109.12979},
  archivePrefix={arXiv},
  primaryClass={cs.RO}
}
```

## TODO(s)

- [x] Write ROS packaging v.0.1
- [x] Update the Readme.md
- [x] Add integration / performance tests on synthetic data
- [ ] Improve the ROS packaging to be more robust in real time to more datasets
- [ ] Fix the binding (which is now broken)
- [ ] Add tests/automatic build to the Github CI

- [ ] Add a wiki (documentation on the code)
