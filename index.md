---
layout: default
---

<center>


<a href="https://github.com/zhluk/Active-SLAM-in-the-Wild" class="btn">Code</a>
<!-- <a href="https://arxiv.org/abs/2112.03221" class="btn">Paper</a> -->

</center>

* * *
<center>
<iframe width="560" height="315" src="https://www.youtube.com/embed/C4t_Rp5XkfE" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>
</center>
## Abstract

This paper presents a Right-Invariant EKF-based Active Simultaneous Localization and Mapping (SLAM) framework for a robot to explore a 2D environment with the goal of achieving low SLAM uncertainty while maximizing area coverage. Two motion planning methods are proposed to solve the active SLAM problem: a covariance-based greedy approach and a novel Quadratic Programming (QP)-based approach. The greedy model is a one-step look-ahead method, which moves in the direction of the lowest uncertainty in the robot’s pose and explores the map through a finite state machine. The QP-based approach drives the robot towards dynamically set goal points. Both methods are implemented on the well-known 2D Victoria Park dataset and localization is performed in relation to the landmarks (trees). Tuning hyper-parameters in the greedy-based approach allows it to make satisfactory area coverage at a high computational cost while the QP motion planning model sacrifices the area coverage for lower runtime. Our method does not make definitive improvements on current methods but features promising results.

## Overview
 We start from an initial state and get a continuous stream of landmark data from the LiDAR sensor. This sensor data is fed into the active SLAM algorithm which first estimates robot pose and then uses the motion planning algorithm to actively estimate the next control inputs. For this project we have used RIEKF as the SLAM method and two different methods for motion planning, a covariance based greedy method and a Quadratic programming  based approach
<center>
<img src="assets/images/overall.PNG" alt="Pipeline" width="1000"/>
<p><em>Overall block diagram showing our approach.</span>.</em></p>
</center>

## Dataset
We have used the Victoria Park dataset to test our hypothesis on the suggest Active SLAM strategy in the paper. The Victoria Park dataset was captured using a car mounted Laser scanner. The dataset is available [here](http://www-personal.acfr.usyd.edu.au/nebot/victoria_park.htm)
## Outputs on Covariance-based greedy approach
Following are the results for the Active SLAM results on RIEKF with Covaraince based method for Motion Planning. 
<center>
<img src="assets/images/iekf_covariance.gif" alt="iekf_covariance" width="1000"/>
<p><em>Results for Covariance-based greedy approach</span>.</em></p>
</center>


## Outputs on QP-based approach
Following are the results for the Active SLAM results on RIEKF with QP-based method for Motion Planning. 
<center>
<img src="assets/images/iekf_qp.gif" alt="iekf_QP" width="1000"/>
<p><em>Results for QP-based approach</span>.</em></p>
</center>

## Conclusion and Future work
This paper proposes a novel method applying RIEKF based active SLAM to a real world 2D dataset, and the use of QP as motion planning algorithm. It presents the methods for RIEKF, motion planning algorithms, and goal point selection, the simulation environment setup, the experiments, and the results. The results for the QP-based method, while showing less area coverage, yields less errors and more efficient navigation, with a more practical path for a real robot to follow.
The future work for our project involves extending the experiments to other datasets to further confirm our findings. Since EKF-based methods are known to be prone to error accumulation, we suggest exploring non-EKF based active SLAM approaches with the same dataset and experiment setup. Furthermore the method could be extended to 3D datasets. Finally, alternative motion planning approaches such as model predictive control (MPC), sequential QP (SQP), multiple steps look-ahead, and collision-free motion planning can be explored. 
