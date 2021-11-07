#results

px4_vicon.pdf is the result for onboard estimator and the vicon system

EKF_px4_vicon.pdf is the result for our EKF, onboard estimator and the vicon system.

CF_px4_vicon.pdf is the result for complimentary filter, onboard estimator and the vicon system.


#code

state_estimation.py is the main script for EKF

complimentary_filter is the main script for complimentary filter

#math

Formulas are documented in ENAE788M_assignment2.pdf

# time alignment
In EKF, since we want to use the covariance matrix of a and w, we use the magnetic sensor as the 
reference and enforce alignment on IMU signal by selecting cloest sample in time

In complimentary filter, we use imu data as reference and use interpolation for alignment