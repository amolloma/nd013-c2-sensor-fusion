# Writeup: Track 3D-Objects Over Time

### 1. Write a short recap of the four tracking steps and what you implemented there (filter, track management, association, camera fusion). Which results did you achieve? Which part of the project was most difficult for you to complete, and why?

#### Step 1: Extended Kalman Filter
Included programming the EKF predict function and the update function. The EKF model is assumed to be a constant velocity model in 3D environment. The state vector is (6, 1) in dimension defining the state position (x, y, z) and state velocities in the (vx, vy, vz) in the x, y and z direction respectively.

The EKF predict function involves definition of the following attributes and function:
    1. State transition matrix or System Matrix F(x), as this is a linear model. The state transition matrix as (6, 6).
    2. Process noise is assumed to have zero mean and a covariance of Q, which accounts for the random change in velocity/ acceleration in our model. The process noise variable is defined in the parameter file and is 3 m/s**2, which assume normal driving condition and no emergency braking is expected
    3. State Covariance P, which accounts for uncertainity in our predicted state.

The EKF update function involves definition of the following attributes and functions:
    1. Gamma, the residual between the measurement z and the state prediction. The deterministic function hx from sensor measurements is used to detemine gamma.
    2. Associated gamma covariance S, which includes the measurement noise variance R.
    3. Kalman gain, which weighs the predicted state in comparision to the measurement.
    4. Measurement state (x+) and covariance update (P+).

#### Step 2: Track Management
    1. Included track state (x) and covariance (P) initialization based on the unassigned sensor measurement followed by conversion of the states from sensor coordinated to vehicle coordinates. Initialization fo the covariance P includes initializing the covariance in the track position using the measurement noise variance R and the rotation matrix. Covariance Matrix P also includes initialization of large uncertainity for velocity in the x, y and z direction as our model assumes constant velocity and cant predict velocity initially.
    2. Track management includes updating the track status from initialized, tentative to confirmed based on track score thresholds. Track scores are increased from 0 to 1, 0 being initialized and 1 being confirmed based on if the initialized has consecutive measurement updates. Track scores decrease if the track is in the field of view of the sensor but has no associated measurement to update it. 
    3. Tracks are deleted if the associated track score falls below deletion threshold or the uncertainity associated with the track in the x and y direction increase above the max covariance P.

#### Step 3: Association
    1. Includes creating the association matrix A, that associates each measurement as measured by the sensor and stored in the measurement list to each of the active tracks. 
    2. Measurements are associated with a track in the association matrix if the Mahalanobis distance between the measurement and the track lies within the gating or inverse cumulative function chi square of the track.

#### Step 4: Sensor Fusion (Adding Camera measurements)
    1. Implementation of nonlinear camera measurement function hx, which converts the vehicles coordinates to sensor coordinate and projects them on the 2D image coordinates.
    2. Updating the measurement class to include initialization of the camera measurement z and associated noise covariance R. Since the camera measures in 2D, the measurement dimension are (2, 1).The noise covariance is diagonal matrix with (2, 2) dimension, we assume that the measurement in x direction doesnt affect the measurement in y and vice versa.

#### what part was most difficult? 
For me the track management seemed to be the most difficult part as, I was not deleting initialized tracks and had a cluterred screen with multiple ghost tracks. It was fixed after using track states as a condition in defining different heuristics for confirmed tracks and tentative or initialized tracks.

Results associated with each step are attached below.

### 2. Do you see any benefits in camera-lidar fusion compared to lidar-only tracking (in theory and in your concrete results)?
There are benefits to camera-lidar fusion as, camera with its rich data feed and high resolution helps correct the lidars false positives as detected in the lidar BEV. 
##### Example :
    Faulty detection by lidar is not update and tracked.

    ![](/img/final/tracking027.png)

### 3. Which challenges will a sensor fusion system face in real-life scenarios? Did you see any of these challenges in the project?
Track management based on fov seems to be the most challenging scenario, as with multiple sensors having different fov of views oriented in various directions can lead to complex track state and track score update algorithm. 

In the project making sure that, track score is not decreased if the track is not in the cameras fov is still not perfect.

### 4. Can you think of ways to improve your tracking results in the future?
Using advanced data association techinques like the Joint Probabilistic Data Association (JPDA) or implementing process noise and measurement noise with non zero mean and tuned covariances are possible ways to improve tracking results.

# Task 1 Results and Plots

#### Plot showing 3D image detection and vehicle coordinate detection

![](/img/final/Task_1_plot.png)

#### Extended Kalman Filter RMSE (Evaluation Metric)

![](/img/final/Task_1_RMSE.png)

# Task 2 Result Plot

![](/img/final/Task_2_RMSE.png)

# Task 3 Result Plot without sensor fusion (lidar only)

![](/img/final/Task_3_RMSE.png)

# Task 4 Result Plot with sensor fusion (lidar and camera)

![](/img/final/Task_4_RMSE.png)

