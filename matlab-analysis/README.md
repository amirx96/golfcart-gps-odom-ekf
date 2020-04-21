# Extended Kalman Filter for Fusing IMU, GPS and Bearing data
This package contains an EKF implementation for fusing information coming from an IMU (50 Hz) and GPS (5 Hz). The IMU provides the body acceleration and the body rotation rates. The GPS unit provides Latitude, Longitude, Altitude and absolute Orientation in NED cooridnate frame.

![alt text](figures/schematic.jpg "Schematic")


## Setup
MATLAB is required. 

## Running the code
Open the file named [stateEstimation3D.m](https://github.com/amirx96/golfcart-gps-odom-ekf/blob/master/matlab-analysis/stateEstimation3D.m) and hit Run. 
