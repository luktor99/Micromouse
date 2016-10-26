/*
 * motion.h
 *
 *  Created on: Jul 2, 2016
 *      Author: luktor99
 */

#ifndef MOTION_H_
#define MOTION_H_

#include <stdint.h>

// Robot dimensions
const float wheel_radius=0.0189006424;
const float wheel_dist=0.0717; // distance between the wheels // 0.0700455036
const float vel_coeff=0.00178499583; // 2pi/encoder_ticks_per_revolution = 2pi/3520
//const float dt=0.001; // sampling time

// Speed & Acc
//const float def_vmax=10.0;
const float def_acc=0.001; // 0.025 m/s^2

// PID settings
const float def_kP=350.0; //350.0;
const float def_kI=12.0; //12.0;
const float def_kD=0.0; //-2000.0;
const float iValMax=2000.0; // max absolute I term sum value

// Motor speed feedback moving average length
const uint8_t speed_avg_count = 20;

// Gyro bias average sample count
const uint16_t gyro_avg_count = 1000; // 1s

// Motion control:
class MotionCtrl {
public:
	void init(void);
	void tick(void);
	void setkP(float kP);
	void setkI(float kI);
	void setkD(float kD);
	void enable(void);
	void disable(void);
	void setVelLin(float v);
	void setVelRot(float w);
	void resetLocalisation();

//private: // all variables are public for debugging
	// Motor control and encoder variables:
	float velL, velR, velLin, velRot, acc;
	int8_t dirL, dirR;
	int32_t encL, encR;
	int16_t dencL, dencR;
	uint8_t enabled;

	// PID variables:
	float errL, errR, errLlast, errRlast, sumL, sumR;
	float pwmL, pwmR;
	float kP, kI, kD;
	float dvelRot, dvelLin;
	int8_t avgLdata[speed_avg_count], avgRdata[speed_avg_count];
	float avgL, avgR;
	uint32_t avg_counter;

	// Localization variables:
	float posX, posY, heading, dHeading;

	// Acc & gyro measurements
	int16_t ax, ay, az;
	int16_t gx, gy, gz;
	// gyro bias average storage
	int16_t gz_samples[gyro_avg_count];
	float gz_bias;
	uint16_t gz_counter;

	void setL(int32_t pwm);
	void setR(int32_t pwm);
	uint16_t clamp(uint32_t val);
};

#endif /* MOTION_H_ */
