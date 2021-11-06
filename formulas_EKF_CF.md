Extended Kalman Filter for fusing IMU and magnetometer

State vector:  $x=[p_x, p_y, p_z]$ represents position of the robots w.r.t. the map frame. $q=[q_x, q_y, q_z, q_w]$ represents the rotation of the robot w.r.t. the map frame
$$
\left[  p_x, p_y,  p_z, q_x, q_y, q_z, q_w \right]
$$
Observation vector $z=[a_1, a_2, a_3, m]$. $a_1$ denotes the acceleration reading from the first IMU, $a_2$ from the second, and $a_3$ the third. $m$ denotes the reading from the magnetometer 



$\alpha$





