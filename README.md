# Path Reconstruction

This project focuses on path reconstruction and was undertaken as part of the Signals and Systems class in the second semester of RT3. It's implemented using MATLAB.

The questions and their corresponding answers required for this project are detailed in the following document: [Project Questions and Answers](https://docs.google.com/document/d/1WdYQ80hJ3NSnHRhVYLsBKBDqU2Zg9DnZ7z0518nPxSA/edit?usp=sharing)

The project comprises several MATLAB scripts:
- **mobile_trajectory.m**: Generates the original path.
- **EKF_tracking.m**: Reconstructs the path using the Extended Kalman Filter.
- **EKF_tracking_4points.m**: Reconstructs the path using measurements from four stations instead of three.
- **EKF_tracking_q7.m**: Adjusts the metric used for reconstruction to accommodate a clock offset \( t_0 \) in the stations.
