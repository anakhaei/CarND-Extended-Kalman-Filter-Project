# Extended Kalman Filter Project
The goal of this project is to implement a EKF algorithm tto estimate the state of a moving object of interest with noisy lidar and radar measurements. The algorithm reads sensor telemetry from Laser and Radar and has to fuse the data to estimate the position and velocity of the moving abject. The RMSE values of the state estimation has to be less than [.11, .11, 0.52, 0.52] 

## Dependencies
I created a docker image that includes all the dependencies. The script will create an image called carnd-term2. To run the container:

`./run_term2_docker.sh`

It will mount the current directory into the container as /src. That means you could keep all the source code outside of the container.

If you don't want to install this image, then follow [udacity's project repo](https://github.com/udacity/CarND-MPC-Project) to set up the environment.

## Build & Run
1. Clone this repo.
2- `./run_term2_docker.sh`
3- `cd src`
4. Make a build directory: `mkdir build && cd build`
5. Compile: `cmake .. && make`

Also, This project involves the Term 2 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases)

## Implementation
To do the sensor fusion, my algorithm follows the general processing flow as taught in the lessons. It handles the first measurements appropriately based on the sensor type. Then, at each iteration, it predict the next state and then update it. The algorithm canhandle radar and lidar measurements:

### State representation:
     * X
     * Y
     * V_x
     * V_y

### Initialization:
     * Initializing the state `ekf_.x_` with the first measurement.
     * Createing the covariance matrix.
     * Convert radar from polar to cartesian if the first sensor is radar
     
### Prediction:
     * Updating the state transition matrix F according to the new elapsed time.
     * Updateing the process noise covariance matrix.

### Update:
     * Useing the sensor type to perform the update step.
     * Updating the state and covariance matrices in `Update` or `UpdateEKF` function:
       * Update Function: Updates the state by using standard Kalman Filter equations
       * UpdateEKF Function: Updates the state by using Extended Kalman Filter equations. Main difference is that Hj was used to linierize H.
       
### Performance:
Based on this implementation, I got the following RMSE for data set provided for this project: [0.09, 0.08, 0.45, 0.43]
       
## Discussion
My algorithm follows the general processing flow as taught in the lessons and I was able to get the required performance. The only issue that I encountered during the implementation was regarding the the time stamp for the first measrement. Since I didn't set the `previous_timestamp_` during the initialization, I had a huge `P_` and RMSE at the end of the second iteration. After correcting the bug, the algorithm worked as expected.  


