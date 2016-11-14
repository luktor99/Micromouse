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
const float def_acc=0.001; // in m/s^2, 0.001 is fine

// PID settings
const float def_kP=350.0; //350.0;
const float def_kI=12.0; //12.0;
const float def_kD=0.0; //-2000.0;
const float iValMax=2000.0; // max absolute I term sum value

// Motor speed feedback moving average length
const uint8_t speed_avg_count = 20;

// Gyro bias average sample count
const uint16_t gyro_avg_count = 1000; // 1s

// Localisation correction coefficients
const float coeff_heading = 0.001; // 0.002
const float coeff_pos = 0.003; // 0.005

// Wall detection threshold for front and side wall sensors
const uint8_t wall_thresh_front = 187; // 18.7mm
const uint8_t wall_thresh_side = 125; // 12.5mm
const uint8_t wall_thresh_diag = 125; // 12.5mm

// distance between sensors (left and right wall sensors)
const float d_LR = 65.0; // mm
// distance from sensor to center axis (left and right wall sensors)
const float r_LR = 35.0; // mm
// distance between sensors (front and rear wall sensors)
const float d_FR = 60.0; // mm
// distance from sensor to center axis (front and rear wall sensors)
const float r_FR = 38.75; // mm

/* localisation correction calibration data:
[UP]Ang: -0.031592
[UP]Dist: 0.087272
[DOWN]Ang: -0.073596
[DOWN]Dist: 0.086655
[LEFT]Ang: 0.164456
[LEFT]Dist: 0.093283
[RIGHT]Ang: -0.139795
[RIGHT]Dist: 0.083677
 */
const float ang_calib[4]={-0.031592, -0.073596, 0.164456, -0.139795};
const float dist_calib[4]={0.087272-0.085, 0.086655-0.085, 0.093283-0.085, 0.083677-0.085}; // TODO: change 0.09 to a smaller value that includes wall's thicknesss


// possible global orientations of the robot
enum orientations_global {UP=0, DOWN, LEFT, RIGHT};
// and the local ones for sensor data
enum orientations_local {FRONT=0, REAR}; // LEFT and RIGHT are the same as above, so not included

// Motion control:
class MotionCtrl {
public:
	void init(void);
	void tick(void);
	void calib(void);
	void setkP(float kP);
	void setkI(float kI);
	void setkD(float kD);
	void enable(void);
	void disable(void);
	void setVelLin(float v);
	void setVelRot(float w);
	void resetLocalisation(void);

//private: // everything public for debugging
	void correctLocalisation(float *, float *, float *);
	float calcAng(uint8_t, uint8_t, float);
	float calcDist(uint8_t, uint8_t, float, float);
	void checkWall(float, float, uint8_t, float, float, float *, float *, float *, float *, uint8_t *, uint8_t *, uint8_t *);

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
	int8_t cellX, cellY;
	float cellXcenter, cellYcenter;
	uint8_t orientation;

	// Walls detection
	bool wallState[4];

	// Acc & gyro measurements
	int16_t ax, ay, az;
	int16_t gx, gy, gz;
	// gyro bias average storage
	int16_t gz_samples[gyro_avg_count];
	float gz_bias;
	uint16_t gz_counter;

	void setL(int32_t pwm);
	void setR(int32_t pwm);
	uint16_t clampPWM(uint32_t val);
};

#endif /* MOTION_H_ */
