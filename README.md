# Active-SLAM-in-the-Wild

This repository implements an Active SLAM framework on the Victoria Park dataset with the goal of localizing and tracking landmarks (in this case, the landmarks are trees) while maximizing area coverage. This is a repository that has been modified from the paper "Invariant EKF based 2D Active SLAM with Exploration Task"

Two methods are implemented to perform the Active SLAM framework: 
    1. The paper's original method with tuned weights in the objective function 
    2. A Quadratic Program (QP) based motion planning method 

Link to original paper: https://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=9561951
Link to paper's repo: https://github.com/MengyaXu/RIEKF-based-active-SLAM

Instruction on how to download and run the code:

    cd ~/path_to_your_folder
    git clone https://github.com/zhluk/Active-SLAM-in-the-Wild.git

Stay on the main branch to run the modified original implementation 

To run the QP-based implementation:

    git checkout QP

In the MATLAB command line, the following commands will run either implementation:

    >> getNoise
    >> map
    >> active_IEKF
