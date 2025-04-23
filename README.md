<div align="center">
    <h1>AF-RLIO</h1>
    <a href=https://youtu.be/yYzz15vRkDI>YouTube</a>
    <br />
    <br />

AF-RLIO: Adaptive Fusion of Radar-LiDAR-Inertial Information for Robust Odometry in Challenging Environments ([ICRA 2025](https://2025.ieee-icra.org/), accepted).  

  <br />

Chenglong Qian, Yang Xu, Xiufang Shi, Jiming Chen, Liang Li

</div>

![image](https://github.com/QCL0920/AF-RLIO/blob/main/config/doc/image.png)

## Prerequisites

1.  Ubuntu 20.04, ROS1
2. Dependencies

- Eigen3
- PCL
- GTSAM
- OpenCV4

## Installation

```sh
  mkdir af_rlio/src
  cd af_rlio/src
  git clone https://github.com/QCL0920/AF-RLIO.git
  cd ..
  catkin_make
  source devel/setup.bash
  
```

## Run
```sh
source devel/setup.bash 
roslaunch af_rlio mapping_ouster64.launch
rosbag play YOUR_DOWNLOADED.bag
```
## Acknowledgements
Thanks for the excellent open-source projects that we rely on:  [FAST-LIO2](https://github.com/hku-mars/FAST_LIO).