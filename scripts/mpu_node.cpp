#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <cmath>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/UInt32.h>
#include <std_msgs/Float64.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/Quaternion.h>
#include <vector>

#define MPU6050_ADDR 0x68
#define GRAVITY 9.81
#define SAMPLES_FOR_OFFSET 3000
#define SAMPLES_FOR_VARIANCE 1000
#define ACC_SENSITIVITY_FACTOR 16384.0
#define GYRO_SENSITIVITY_FACTOR 131.0
#define GYRO_COEF 0.98f
#define ACC_COEF 0.02f

/*		MPU6050
				  +X
		-------------------------
		|						|
		|						|
	+Y	|						|	+Z (UP)
		|		  CHIP			|
		|					LED |
		|	      Pins			|
		-------------------------

		SDA: Pin 3 (I2C Bus no. 8)
		SCL: Pin 5
		VCC: Pin 1 (3.3V)
		GND: Pin 9

*/

std::vector<std::vector<double>> dataForOffset;
std::vector<float> variance(10, 0.0);
std::vector<double> data_for_offset;
std::vector<float> mean;
int16_t rawXacc,rawYacc,rawZacc,rawXomg,rawYomg,rawZomg;
float accX=0,accY=0,accZ=0,omgX=0,omgY=0,omgZ=0,angX=0,angY=0,angZ=0;
float angAccX,angAccY,accx,accy,accz,omgx,omgy,omgz,qx=0,qy=0,qz=0,qw=0;
float gyroXOffset,gyroYOffset,gyroZOffset,dt;

int main(int argc, char **argv) {

	ros::init(argc, argv, "mpu6050");
	ros::NodeHandle n;

	ros::Publisher imu_pub = n.advertise<sensor_msgs::Imu>("mpu6050/data", 10);

	ros::Rate loop_rate(100);
	ROS_INFO("MPU node started");

	//	---------- Connecting with MPU ---------- 
	int file;
    const char *bus = "/dev/i2c-8";
    if((file = open(bus, O_RDWR)) < 0) {
        ROS_ERROR("Failed to open i2c bus");
        return 1;
    }
    if(ioctl(file, I2C_SLAVE, MPU6050_ADDR) < 0) {
        ROS_ERROR("Failed to connect to MPU6050");
        return 1;
    }

    char config[2] = {0};
    config[0] = 0x6B; // PWR_MGMT_1 register
    config[1] = 0x00; // Wake up device
    // ---------- Connection done, MPU ready to send data ---------- 

    tf2::Quaternion q;
    geometry_msgs::Quaternion q_msg;
    sensor_msgs::Imu imu_data;
    int count = 0;
    
    // ---------- Calculating offsets ----------
    ROS_INFO("Calculating offsets...");
    for(int i=0;i<SAMPLES_FOR_OFFSET;i++){
    	write(file, config, 2);
		char reg[1] = {0x43}; // GYRO_XOUT_H register
		write(file, reg, 1);
		char data[6] = {0};
		if(read(file, data, 6) != 6) {
		    ROS_ERROR("Error: Input/output error\n");
		    return 1;
		}

		// Convert data to omega values
		rawXomg = (data[0] << 8) | data[1];
		rawYomg = (data[2] << 8) | data[3];
		rawZomg = (data[4] << 8) | data[5];

		omgX += rawXomg / GYRO_SENSITIVITY_FACTOR;
		omgY += rawYomg / GYRO_SENSITIVITY_FACTOR;
		omgZ += rawZomg / GYRO_SENSITIVITY_FACTOR;
    }
    gyroXOffset = omgX / SAMPLES_FOR_OFFSET;
    gyroYOffset = omgY / SAMPLES_FOR_OFFSET;
    gyroZOffset = omgZ / SAMPLES_FOR_OFFSET;
    omgX = 0;
    omgY = 0;
    omgZ = 0;
    // ---------- Offsets Calculated ----------

    double start = ros::Time::now().toSec() * 1000, end;

    // ---------- Calculating variances ----------
    ROS_INFO("Calculating variances...");
    for(int i=0;i<SAMPLES_FOR_VARIANCE;i++){
    	write(file, config, 2);
		char reg[1] = {0x3B}; // ACCEL_XOUT_H register
		write(file, reg, 1);
		char data[14] = {0};
		if(read(file, data, 14) != 14) {
		    ROS_ERROR("Error: Input/output error\n");
		    return 1;
		}

		rawXacc = (data[0] << 8) | data[1];
		rawYacc = (data[2] << 8) | data[3];
		rawZacc = (data[4] << 8) | data[5];

		accx = (rawXacc / ACC_SENSITIVITY_FACTOR * GRAVITY);
		accy = (rawYacc / ACC_SENSITIVITY_FACTOR * GRAVITY);
		accz = (rawZacc / ACC_SENSITIVITY_FACTOR * GRAVITY);
		data_for_offset.push_back(accx);
		data_for_offset.push_back(accy);
		data_for_offset.push_back(accz);
		accX += accx;
		accY += accy;
		accZ += accz;

		// Constant gravitational acceleration is there so...
		angAccX = atan2(accy, accz +abs(accx)) * 360 / 2.0 / M_PI;
		angAccY = atan2(accx, accz +abs(accy)) * 360 / -2.0 / M_PI;

		// Convert data to omega values
		rawXomg = (data[8] << 8) | data[9];
		rawYomg = (data[10] << 8) | data[11];
		rawZomg = (data[12] << 8) | data[13];

		omgx = (rawXomg / GYRO_SENSITIVITY_FACTOR) - gyroXOffset;
		omgy = (rawYomg / GYRO_SENSITIVITY_FACTOR) - gyroYOffset;
		omgz = (rawZomg / GYRO_SENSITIVITY_FACTOR) - gyroZOffset;
		data_for_offset.push_back(omgx * (M_PI / 180));
		data_for_offset.push_back(omgy * (M_PI / 180));
		data_for_offset.push_back(omgz * (M_PI / 180));
		omgX += omgx;
		omgY += omgy;
		omgZ += omgz;

		// Calculating angle in degrees
		end = ros::Time::now().toSec() * 1000;
		dt = (end - start) / 1000;
		// angX += omgx * dt;
		// angY += omgy * dt;
		angX = (GYRO_COEF * (angX + omgx * dt)) + (ACC_COEF * angAccX);
		angY = (GYRO_COEF * (angY + omgy * dt)) + (ACC_COEF * angAccY);
		angZ += omgx * dt;

		// Degree -> radians -> quaternians
		q.setRPY(angX * (M_PI / 180), angY * (M_PI / 180), angZ * (M_PI / 180));
		tf2::convert(q, q_msg);
		qx += q_msg.x;
		qy += q_msg.y;
		qz += q_msg.z;
		qw += q_msg.w;
		data_for_offset.push_back(q_msg.x);
		data_for_offset.push_back(q_msg.y);
		data_for_offset.push_back(q_msg.z);
		data_for_offset.push_back(q_msg.w);
		dataForOffset.push_back(data_for_offset);
		start = end;
    }

    // Calculate mean
    mean.push_back(accX / SAMPLES_FOR_VARIANCE);
    mean.push_back(accY / SAMPLES_FOR_VARIANCE);
    mean.push_back(accZ / SAMPLES_FOR_VARIANCE);
    mean.push_back(omgX / SAMPLES_FOR_VARIANCE);
    mean.push_back(omgY / SAMPLES_FOR_VARIANCE);
    mean.push_back(omgZ / SAMPLES_FOR_VARIANCE);
    mean.push_back(qx / SAMPLES_FOR_VARIANCE);
    mean.push_back(qy / SAMPLES_FOR_VARIANCE);
    mean.push_back(qz / SAMPLES_FOR_VARIANCE);
    mean.push_back(qw / SAMPLES_FOR_VARIANCE);

    // Calculate variance
    for(int i=0; i<SAMPLES_FOR_VARIANCE;i++){
    	for(int j=0;j<10;j++){
    		variance[j] += pow(dataForOffset[i][j] - mean[j], 2);
    	}
    }
    for(int j=0;j<10;j++){
		variance[j] /= (SAMPLES_FOR_VARIANCE-1);
	}
	data_for_offset.clear();
	dataForOffset.clear();
	mean.clear();

    imu_data.orientation_covariance[0] = variance[6];
    imu_data.orientation_covariance[1] = 0;
    imu_data.orientation_covariance[2] = 0;
    imu_data.orientation_covariance[3] = variance[7];
    imu_data.orientation_covariance[4] = 0;
    imu_data.orientation_covariance[5] = variance[8];
    imu_data.orientation_covariance[6] = variance[9];
    imu_data.orientation_covariance[7] = 0;
    imu_data.orientation_covariance[8] = 0;

    imu_data.angular_velocity_covariance[0] = variance[3];
    imu_data.angular_velocity_covariance[1] = 0;
    imu_data.angular_velocity_covariance[2] = 0;
    imu_data.angular_velocity_covariance[3] = 0;
    imu_data.angular_velocity_covariance[4] = variance[4];
    imu_data.angular_velocity_covariance[5] = 0;
    imu_data.angular_velocity_covariance[6] = 0;
    imu_data.angular_velocity_covariance[7] = 0;
    imu_data.angular_velocity_covariance[8] = variance[5];

    imu_data.linear_acceleration_covariance[0] = variance[0];
    imu_data.linear_acceleration_covariance[1] = 0;
    imu_data.linear_acceleration_covariance[2] = 0;
    imu_data.linear_acceleration_covariance[3] = 0;
    imu_data.linear_acceleration_covariance[4] = variance[1];
    imu_data.linear_acceleration_covariance[5] = 0;
    imu_data.linear_acceleration_covariance[6] = 0;
    imu_data.linear_acceleration_covariance[7] = 0;
    imu_data.linear_acceleration_covariance[8] = variance[2];
    // ---------- Variances Calculated ----------

    ROS_INFO("Publishing on topic /mpu6050/data");
    start = ros::Time::now().toSec() * 1000;
	while (ros::ok()){

		write(file, config, 2);
		char reg[1] = {0x3B}; // ACCEL_XOUT_H register
		write(file, reg, 1);
		char data[14] = {0};
		if(read(file, data, 14) != 14) {
		    ROS_ERROR("Error: Input/output error\n");
		    return 1;
		}

		// Convert data to acceleration values
		rawXacc = (data[0] << 8) | data[1];
		rawYacc = (data[2] << 8) | data[3];
		rawZacc = (data[4] << 8) | data[5];

		accX = rawXacc / ACC_SENSITIVITY_FACTOR * GRAVITY;
		accY = rawYacc / ACC_SENSITIVITY_FACTOR * GRAVITY;
		accZ = rawZacc / ACC_SENSITIVITY_FACTOR * GRAVITY;

		// Constant gravitational acceleration is there so...
		angAccX = atan2(accY, accZ +abs(accX)) * 360 / 2.0 / M_PI;
		angAccY = atan2(accX, accZ +abs(accY)) * 360 / -2.0 / M_PI;

		// Convert data to omega values
		rawXomg = (data[8] << 8) | data[9];
		rawYomg = (data[10] << 8) | data[11];
		rawZomg = (data[12] << 8) | data[13];

		omgX = (rawXomg / GYRO_SENSITIVITY_FACTOR) - gyroXOffset;
		omgY = (rawYomg / GYRO_SENSITIVITY_FACTOR) - gyroYOffset;
		omgZ = (rawZomg / GYRO_SENSITIVITY_FACTOR) - gyroZOffset;

		// Calculating angle in degrees
		end = ros::Time::now().toSec() * 1000;
		dt = (end - start) / 1000;
		// angX += omgX * dt;
		// angY += omgY * dt;
		angX = (GYRO_COEF * (angX + omgX * dt)) + (ACC_COEF * angAccX);
		angY = (GYRO_COEF * (angY + omgY * dt)) + (ACC_COEF * angAccY);
		angZ += omgZ * dt;

		// std::cout<<"Acceleration: X="<<accX<<", Y="<<accY<<", Z="<<accZ<<std::endl;
		// std::cout<<"Gyro:         X="<<omgX<<", Y="<<omgY<<", Z="<<omgZ<<std::endl;
		// std::cout<<"Angle:        X="<<angX<<", Y="<<angY<<", Z="<<angZ<<std::endl;
		// std::cout<<std::endl;

		// Degree -> radians -> quaternians);
		q.setRPY(angX * (M_PI / 180), angY * (M_PI / 180), angZ * (M_PI / 180));
		tf2::convert(q, q_msg);

		// Create Imu message and publish
		imu_data.header.seq = count;
		imu_data.header.stamp = ros::Time::now();
		imu_data.header.frame_id = "mpu6050_frame";

		imu_data.orientation = q_msg;
		
		imu_data.linear_acceleration.x = accX;
		imu_data.linear_acceleration.y = accY;
		imu_data.linear_acceleration.z = accZ;
		
		imu_data.angular_velocity.x = omgX * (M_PI / 180);
		imu_data.angular_velocity.y = omgY * (M_PI / 180);
		imu_data.angular_velocity.z = omgZ * (M_PI / 180);

		imu_pub.publish(imu_data);

		count++;
		start = end;
		ros::spinOnce();
		loop_rate.sleep();
	}    

    close(file);
    return 0;
}
