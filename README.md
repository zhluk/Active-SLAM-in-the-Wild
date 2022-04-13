# Active-SLAM-in-the-Wild

This repository implements an Active SLAM framework on the Victoria Park dataset with the goal of localizing and tracking landmarks (in this case, the landmarks are trees) while maximizing area coverage. This is a repository that has been modified from the paper "Invariant EKF based 2D Active SLAM with Exploration Task"

Two methods are implemented to perform the Active SLAM framework: 
    1. The paper's original method with tuned weights in the objective function 
    2. A Quadratic Program (QP) based motion planning method 

Link to original paper: https://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=9561951
Link to paper's repo: https://github.com/MengyaXu/RIEKF-based-active-SLAM

Instruction on how to download and run the code:

cd <your-folder>
git clone https://github.com/zhluk/Active-SLAM-in-the-Wild.git

Use the main branch code to run the modified original implementation
    
In the MATLAB command line, run this command to generate a map in 2D plane:
    
```
>> map
```

Run this command to generate motion and measurement noise:

```    
>> getNoise
```
    
Run this command to perfrom active slam:

```    
>> active_IEKF
```

Use the QP brach codes to run the Quadratic Program based motion planning method
