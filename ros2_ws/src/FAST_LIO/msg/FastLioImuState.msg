# LiDAR-corrected odometry
nav_msgs/Odometry odometry

# Estimated IMU biases (IMU/body frame)
geometry_msgs/Vector3 gyro_bias
geometry_msgs/Vector3 accel_bias

# Estimated gravity (world frame)
geometry_msgs/Vector3 gravity

# FAST-LIO accelerometer normalization factor
float64 accel_scale
