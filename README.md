[![Project from BMW](https://github.com/chrisHuxi/Trajectory_Predictor/blob/master/readme_images/Absolut.jpeg)](https://absolut-projekt.de/)

[![Project from TUD](https://img.shields.io/badge/TU%20dresden-Computer%20Science-blue)](https://tu-dresden.de/ing/informatik)


# Trajectory Predictor
A python implementation of multi-model estimation algorithm, based on paper: [research project: trajectory predication technology for autonomous driving vehicle](https://github.com/chrisHuxi/Trajectory_Predictor/blob/master/readme_images/trajectory_predication_final.pdf)

Mainly refer to paper:
  * [A comparative study of multiple-model algorithms for maneuvering target tracking](http://citeseerx.ist.psu.edu/viewdoc/download?doi=10.1.1.61.9763&rep=rep1&type=pdf)
  * [Joint Monocular 3D Vehicle Detection and Tracking](https://github.com/ucbdrive/3d-vehicle-tracking)
  * [A Baseline for 3D Multi-Object Tracking](https://github.com/xinshuoweng/AB3DMOT)

## Intro:
In autonomous driving systems, the prediction of the other vehicles' behavior in the next few seconds based on their current state, including position, velocity, and yaw angle, has a critical influence on ego-vehicle's decision on the behavior at the next moment. 

Therefore, this project aims to implement a **trajectory predictor**, based on lidar and camera sensor. The result shows, **the AMM algorithm** based on the motion model can predict the future trajectory of the object within a certain range


## Outline:
  * We used [**the KITTI object tracking testset**]() as dataset. 
  
  * We used the **camera and lidar** as sensors, to get the position information of vehicle.
  
  * We used the [**Mono3DT(for camera)**](https://github.com/ucbdrive/3d-vehicle-tracking) and [**AB3DMOT(for lidar)**](A Baseline for 3D Multi-Object Tracking) as detection and tracking model. A comparison of tracking results shows as following (left: lidar vs right: camera):

<div align=center> <img src="https://github.com/chrisHuxi/Trajectory_Predictor/blob/master/readme_images/comparison.gif" alt="drawing" width="500"/> </div>
  
  * We used [**the AMM algorithm based on the motion model**](http://citeseerx.ist.psu.edu/viewdoc/download?doi=10.1.1.61.9763&rep=rep1&type=pdf) as prediction algorithm
  
  * Comparison of predicted result and ground truth at frame 0 (top left), frame 5 (top right), frame 10 (bottom left) and frame 20 (bottom right). The contour presents the probability distribution :
  
<div align=center> <img src="https://github.com/chrisHuxi/Trajectory_Predictor/blob/master/readme_images/frame_predict_10-30_obj_846.jpg" alt="drawing" width="500"/> </div>
  
 


