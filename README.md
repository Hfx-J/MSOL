# MSOL
Multi-Sector Overlap Loss: A Universal Framework for 6DoF Global Localization Across Heterogeneous LiDARs.


### Installation

#### Prerequisites

Install required dependencies:

```bash
sudo apt-get install nlohmann-json3-dev
sudo apt-get install libeigen3-dev
sudo apt install libgtsam-dev libgtsam-unstable-dev
```

> **Note:** Please install ceres-solver (version > 2.1.0) by following the guide at [ceres Installation](http://ceres-solver.org/installation.html).

#### Setup & Usage

```bash
# Clone the repository
git clone https://github.com/Hfx-J/MSOL.git

# Navigate to project directory
cd MSOL 

# Build the project
catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3

# Setup environment
source ./devel/setup.bash

# Launch the application
roslaunch msol heliplace_v2.launch
```
### DataSet
We provide a example dataset in [BaiDuDrive](https://pan.baidu.com/s/1WVywzNPgzUt4VT0o5HGHhQ?pwd=f5i5).
### Citation

If you find our work useful in your research, please consider citing:

```
@ARTICLE{11122650,
  author={Gao, Wang and Huang, Feixuan and Liu, Hong and Pan, Shuguo and Zhao, Heng},
  journal={IEEE Robotics and Automation Letters}, 
  title={Multi-Sector Overlap Loss: A Universal Framework for One-Shot 6DoF Global Localization Across Heterogeneous LiDARs}, 
  year={2025},
  volume={10},
  number={10},
  pages={9862-9869},
  keywords={Laser radar;Location awareness;Point cloud compression;Semantics;Accuracy;Pose estimation;Windows;Data mining;Computational modeling;Training;Localization;SLAM;heterogeneous LiDAR;cylindrical coordinate;overlap loss strategy},
  doi={10.1109/LRA.2025.3597891}}

```

### Contact

For any questions or issues regarding MSOL, please open an issue on GitHub.

