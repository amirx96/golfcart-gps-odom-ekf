# golfcart-gps-odom-ekf

## Intro

This repository is the workspace for the Spring 2020 final projects in two classes: (1) MEEN 689 (Robotic Perception),and (2) STAT 654 (Statistical Computations) - taught by Dr. Srikanth Saripalli and Dr. David Jones, respectively.


## Teammates

### MEEN 689
- Amir Darwesh (MEEN), M.S. Candidate
- Jacob Hartzer (MEEN), M.S. Candidate
- Subodh Mishra (MEEN), Ph.D Candidate
- Keith Sponsler (MEEN), Ph.D Candidate

### STAT 654
- Amir Darwesh (MEEN), M.S. Candidate
- Jacob Hartzer (MEEN), M.S. Candidate
- Keith Sponsler (MEEN), Ph.D Candidate


## Code Structure
```bash
├── docker-setup.sh
├── Notebooks
│   └── GPS & Pacmod Analysis
│       ├── ekf_analysis.ipynb
│       ├── GPS_INS_Uncertainty.ipynb
│       ├── IMU Analysis-1.ipynb
│       ├── IMU Analysis-2.ipynb
│       ├── IMU Analysis-3.ipynb
│       ├── csv
│       ├── bag_data
│       │   ├── 1.bag
│       │   ├── 2.bag
│       │   ├── 3.bag
│       │   ├── data_notes.txt
│       │   └── ekf_2.bag
├── 3rd_party
│   ├── rviz_satellite
│   └── vn300-e6_msgs_only
├── golfcart_ekf
│   ├── launch
│   │   └── estimation.launch
│   ├── nodes
│   │   ├── full_ekf
│   │   ├── ins_to_odom
│   │   ├── ins_to_path
│   │   ├── odom_covariance_marker
│   │   ├── odom_to_path
│   │   └── vectornav_ekf
├── matlab-analysis

```


### Datasets

Unfortunately, due to the COVID-19 Pandemic, we we're only able to collect three datasets (these were collected around the end of Feburary, before the university shutdown. The datasets were collected of driving for 10 to 15 minutes in Downtown Bryan Texas. 

As such, our analysis is limited by the type of data we collected. The data is stored in two ways, ROSbag formats, or CSV generated files from the ROS bags. They can be located in   `Notebooks\GPS & Pacmod Analysis\csv`  or `Notebooks\GPS & Pacmod Analysis\bag_data`. 

### Data Analysis 
Data Analysis for the IMU, GPS, INS, and PacMOD can be found under `Notebooks\
GPS & Pacmod Analysis`. Data analysis files are Jupyter Notebooks (.ipynb) files, which process the csv data and perform some statistical analysis.

## Realtime Code

### 2D
For 2D state estimation, the code can be found under `golfcart_ekf`. 
#### Requirements
- Ubuntu 16.04 & ROS-kinetic
OR
- Windows/Ubuntu with Docker & Visual Studio Code

#### Setup

##### Ubuntu 16.04

Install: 

```bash 
git clone https://github.com/amirx96/golfcart-gps-odom-ekf
rosdep install -y -r --from-path golfcart-gps-odom-ekf
cd golfcart-gps-odom-ekf && catkin_make

```
Launching:

```bash
roslaunch golfcart_ekf estimation.launch
roslaunch rviz_satellite demo.launch (visualization only)
```

##### Windows / Other Ubuntu Versions
Pre-reqs:
    Linux: Docker CE/EE 18.06+ and Docker Compose 1.21+. The snap version in Ubuntu 18.04 won’t do! Trust me, I’ve tried.
    Windows: Docker Desktop 2.0+
    
Launch Visual Studio Code in the folder, and it should ask you if you'd like to switch to opening the directory into a dev container (docker container) 
![vscode prompt](https://amirdarwesh.com/assets/img/2019-09-13/reopen_container.png)

## 3D

See matlab-analysis folder for readme.
