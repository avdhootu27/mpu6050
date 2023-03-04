# mpu6050-jetson-xavier-nx

This package is specifically designed to get orientation, angular velocity and linear acceleration data in the form of "sensor_msgs/Imu" from mpu6050 imu sensor on Nvidia's Jetson Xavier NX board with ROS.

### The parameters assumed in the code :
* Gravity = 9.81m/s2
* Samples to calculate gyro offset = 3000
* Samples to calculate covariances = 1000
* Acceleration sesnitivity factor = 16384.0
* Gyroscope sensitivity factor = 131.0 (+/- 250 degree/sec)
* ROS distribution = Noetic

### Instructions to use:
* Conenct mpu6050 sensor by i2c communication with jetson xavier. Make sure you connect it on i2c bus no. 8
* Update the pakcages.
```bash
sudo apt update
```
* Install i2c-dev library on jetson xavier.
```bash
sudo apt install libi2c-dev
```
* Launch mpu6050.launch file
* Keep MPU stationary until you see a ROS log 'Publishing on topic mpu6050/data' in the terminal
* Always keep MPU in the XY plane (parallel to ground)
