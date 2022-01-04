/*
 * mainpp.cpp
 *
 *  Created on: Jan 12, 2021
 *      Author: Fatih
 */

#include "mainpp.h"

#include "MPU6050.h"

#include <ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Imu.h>

extern UART_HandleTypeDef huart3; // uart3 interrupt

ros::NodeHandle nh;

sensor_msgs::Imu imu;
ros::Publisher pub_imu("imu", &imu);

double gyro[3];
double acc[3];
double quat[4];

#define PI 3.14159265359

char read[5];
int a;
uint8_t g[3];
float gyro_X,gyro_Y,gyro_Z,temp;

//I2cdev MPU6050
MPU6050 mpu;
uint16_t packetSize;
uint16_t fifoCount;
uint8_t fifoBuffer[64];
Quaternion q;
VectorFloat gravity;
VectorInt16 rawAcc;
VectorInt16 linearAcc;
float ypr[3];

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
  nh.getHardware()->flush();
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
  nh.getHardware()->reset_rbuf();
}

void setup() {
	HAL_Delay(2000);

	//I2cdev MPU6050
	mpu.initialize();
	mpu.dmpInitialize();
	mpu.setXGyroOffset(82);
	mpu.setYGyroOffset(-23);
	mpu.setZGyroOffset(-25);

	mpu.setXAccelOffset(686);
	mpu.setYAccelOffset(-3251);
	mpu.setZAccelOffset(1029);
	mpu.setDMPEnabled(true);
	packetSize = mpu.dmpGetFIFOPacketSize();
	fifoCount = mpu.getFIFOCount();

	// initializing publisher node for message type imu
	nh.initNode();
	nh.advertise(pub_imu);
}

void loop() {
//-------------------------------------------reading dmp--------------------------------------------------------------
	while (fifoCount < packetSize) {

		fifoCount = mpu.getFIFOCount();
	}

	if (fifoCount >= 1024) {

	  mpu.resetFIFO();
	  //Serial.println(F("FIFO overflow!"));
	}

	else{

		if (fifoCount % packetSize != 0) {

			mpu.resetFIFO();
			fifoCount = mpu.getFIFOCount();
		}

		else {

		  while (fifoCount >= packetSize) {

			  mpu.getFIFOBytes(fifoBuffer, packetSize);
			  fifoCount -= packetSize;
		  }

		  mpu.dmpGetQuaternion(&q,fifoBuffer);
		  mpu.dmpGetGravity(&gravity,&q);
		  mpu.dmpGetAccel(&rawAcc, fifoBuffer);
		  mpu.dmpGetLinearAccel(&linearAcc, &rawAcc, &gravity);
		  mpu.dmpGetYawPitchRoll(ypr,&q,&gravity);
		}
	}
//--------------------------------------------------------------------------------------------------------------------
//------------------------------------processing values to message packages-------------------------------------------
	imu.angular_velocity.x = gravity.x; //gyro[0];
    imu.angular_velocity.y = gravity.y; //gyro[1];
    imu.angular_velocity.z = gravity.z; //gyro[2];

    imu.linear_acceleration.x = linearAcc.x; //acc[0];
    imu.linear_acceleration.y = linearAcc.y; //acc[1];
    imu.linear_acceleration.z = linearAcc.z; //acc[2];

    imu.orientation.x = q.x; //quat[0];
    imu.orientation.y = q.y; //quat[1];
    imu.orientation.z = q.z; //quat[2];
    imu.orientation.w = q.w; //quat[3];
//--------------------------------------------------------------------------------------------------------------------
//------------------------------------publishing imu------------------------------------------------------------------
	pub_imu.publish(&imu);
	nh.spinOnce();

	HAL_Delay(50);
}


