# Extended Kalman Filter for Fusing IMU, GPS and Bearing data
This package contains an EKF implementation for fusing information coming from an IMU (50 Hz) and GPS (5 Hz). The IMU provides the body acceleration and the body rotation rates. The GPS unit provides Latitude, Longitude, Altitude and absolute Orientation in NED cooridnate frame. The estimator estimates the robot's 3D pose and linear velocities. Hence, the state size is 9.

![alt text](figures/schematic.jpg "Schematic")


## Setup
MATLAB is required. 

## Prediction
We use the IMU to predict and propagate the 3D pose of the robot. The IMU measurements are fed into a [motion model](https://github.com/amirx96/golfcart-gps-odom-ekf/blob/master/matlab-analysis/motion_model.m) for prediction of both the state and it's uncertainty. We assume all the uncertainty in the IMU's measuments are encoded in a process noise covariance matrix Q.
![alt text](figures/predict.png "Predict")
## Update
We correct the IMU prediction using GPS measurements and Orientation measurements. Latitudes and Longitudes are first converted to Northings and Eastings. Ideally, a magnetometer is used for determining the bearing but the INS we use provides the orientation in Frame W. So, we use that. The portion of the code where it is done is provided [here](https://github.com/amirx96/golfcart-gps-odom-ekf/blob/fd9d50f91709be9c295aee27ec22ead0fa7f0f59/matlab-analysis/stateEstimation3D.m#L106-L117)
![alt text](figures/schematic.jpg "Schematic")
![alt text](figures/update.png "Update")

## Results
Please click on the plots to get an elarged view.

### Dataset1
![alt text](figures/XYPlot.jpg "Schematic")

![alt text](figures/XYZRPYvsT.jpg "Schematic")

![alt text](figures/Vxyz.jpg "Schematic")

### Dataset2
![alt text](figures/XYPlot2.jpg "Schematic")

![alt text](figures/XYZRPYvsT2.jpg "Schematic")

![alt text](figures/Vxyz2.jpg "Schematic")

### Dataset3
![alt text](figures/XYPlot3.jpg "Schematic")

![alt text](figures/XYZRPYvsT3.jpg "Schematic")

![alt text](figures/Vxyz3.jpg "Schematic")


## Running the code
Open the file named [stateEstimation3D.m](https://github.com/amirx96/golfcart-gps-odom-ekf/blob/master/matlab-analysis/stateEstimation3D.m) and hit Run. 
