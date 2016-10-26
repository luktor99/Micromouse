/*
 * motors.cpp
 *
 *  Created on: Jul 2, 2016
 *      Author: luktor99
 */
#include <common.h>
#include <hardware.h>
#include <stdlib.h>
#include <math.h>
#include <MPU6050.h>

void MotionCtrl::init(void) {
	// initialize PWM generation
	HAL_TIM_Base_Start(&htim1);
	HAL_TIMEx_OCN_Start(&htim1, TIM_CHANNEL_2); // MOT2 - right
	HAL_TIMEx_OCN_Start(&htim1, TIM_CHANNEL_3); // MOT1 - left

	// Initialize variables
	encL=0;
	encR=0;
	velL=0.0;
	velR=0.0;
	dirL=0;
	dirR=0;
	velLin=0.0;
	velRot=0.0;
	acc=def_acc*0.001;
	enabled=0;

	// Center encoders' values
	__HAL_TIM_SET_COUNTER(&htim2, 32768);
	__HAL_TIM_SET_COUNTER(&htim3, 32768);

	// Initialize the PID controllers
	setkP(def_kP);
	setkD(def_kD);
	setkI(def_kI);
	pwmL=0.0;
	pwmR=0.0;
	errLlast=0.0;
	errRlast=0.0;
	sumL=0.0;
	sumR=0.0;

	// Zero-out localisation
	posX=0.0;
	posY=0.0;
	heading=0.0;
	dHeading=0.0;

	// Init the speed average storage iterator variable
	avg_counter=0;

	// TODO: load previous gyro bias from flash memory (or BKP domain?)
	gz_bias=0;
	gz_counter=0;
}

void MotionCtrl::enable(void) {
	enabled=1;
}

void MotionCtrl::disable(void) {
	taskENTER_CRITICAL();
	enabled=0;
	velL=0.0;
	velR=0.0;
	velLin=0.0;
	velRot=0.0;
	taskEXIT_CRITICAL();
}

void MotionCtrl::resetLocalisation() {
	taskENTER_CRITICAL();
	posX=0.0;
	posY=0.0;
	heading=0.0;
	taskEXIT_CRITICAL();
}

void MotionCtrl::setkP(float skP) {
	kP=skP;
}

void MotionCtrl::setkI(float skI) {
	kI=skI;
}

void MotionCtrl::setkD(float skD) {
	kD=skD;
}

void MotionCtrl::tick(void) {
	// Update encoder readings
	taskENTER_CRITICAL();
	uint16_t encLnow=__HAL_TIM_GET_COUNTER(&htim2);
	__HAL_TIM_SET_COUNTER(&htim2, 32768);
	uint16_t encRnow=__HAL_TIM_GET_COUNTER(&htim3);
	__HAL_TIM_SET_COUNTER(&htim3, 32768);
	taskEXIT_CRITICAL();

	// Calculate encoders increments
	dencL=32768-encLnow;
	dencR=encRnow-32768;
	// Update the global sums
	encL+=dencL;
	encR+=dencR;

	// Calculate moving average of the speed feedback
	avgLdata[avg_counter%speed_avg_count]=dencL;
	avgRdata[avg_counter%speed_avg_count]=dencR; // TODO: remove %avg_count later
	avgL=0.0;
	avgR=0.0;
	for(uint8_t i=0; i<speed_avg_count; i++) {
		avgL+=avgLdata[i];
		avgR+=avgRdata[i];
	}
	avgL/=speed_avg_count;
	avgR/=speed_avg_count;

	// Increment the avg counter
	//avg_counter=(avg_counter+1)%avg_count;
	avg_counter++;

	// Retrieve the latest gyroscope data
	mpu6050.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
	// Store the reading to allow avg calculations
	gz_samples[gz_counter]=gz;
	gz_counter=(gz_counter+1)%gyro_avg_count;
	// Calculate heading increment from gyroscope reading
	float dGyro = ((float)gz-gz_bias)/16.4/180.0*M_PI*0.001;

	if(enabled) {
		// Update the PID controller
		taskENTER_CRITICAL(); // Critical section protects velL and velR from getting changed during calculations
		float velLnow=velL;
		float velRnow=velR;
		taskEXIT_CRITICAL();
		errL = velLnow - avgL;
		errR = velRnow - avgR;
		// Calculate feed-forward loop variables
		float velLff = velLnow*280.0;
		float velRff = velRnow*280.0;

		sumL+=errL;
		sumR+=errR;

		// I term anti wind-up
		if(sumL>iValMax)
			sumL=iValMax;
		else if(sumL<-iValMax)
			sumL=-iValMax;
		if(sumR>iValMax)
			sumR=iValMax;
		else if(sumR<-iValMax)
			sumR=-iValMax;

		// Calculate motor outputs as PID regulator output + feed-forward values - gyro feedback
		pwmL = kP*errL + kI*sumL + kD*(errL-errLlast) + velLff - dGyro*10000.0;
		pwmR = kP*errR + kI*sumR + kD*(errR-errRlast) + velRff + dGyro*10000.0; // TODO: provide a better PID disable (motors off) condition when vel=0
		// Direct regulator outputs to motors
		setL((int32_t)pwmL*840.0/adc_lipo[0]);
		setR((int32_t)pwmR*840.0/adc_lipo[0]);

		// Store current error for D term calculation in next time step
		errLlast=errL;
		errRlast=errR;
	} else {
		setL(0);
		setR(0);
	}

	// Localization update
	// linear velocities of the wheels (based on odometry)
	float dvLinL=dencL*vel_coeff*wheel_radius;
	float dvLinR=dencR*vel_coeff*wheel_radius;
	// linear and rotational velocity of the robot (based on odometry)
	dvelRot=(dvLinR-dvLinL)/wheel_dist;
	dvelLin=(dvLinR+dvLinL)/2.0;
	// update position and heading variables
	float dX=dvelLin*cos(heading);
	float dY=dvelLin*sin(heading);

	// read current heading to make it's later update thread safe
	float headingtemp=heading, dHeadingtemp;

	// gyrodometry:
	if(fabs(dGyro - dvelRot) > 0.001) {
		// gyro
		headingtemp+=dGyro;
		dHeadingtemp=dGyro;
		// LED3 ON
		LED3_GPIO_Port->BSRR = (uint32_t)LED3_Pin;
	} else {
		// odometry
		headingtemp+=dvelRot;
		dHeadingtemp=dvelRot;
		// LED3 OFF
		LED3_GPIO_Port->BSRR = (uint32_t)LED3_Pin << 16U;
	}

	// normalize updating heading to -pi..pi range
	if(headingtemp>M_PI)
		headingtemp-=2.0*M_PI;
	else if(headingtemp<-M_PI)
		headingtemp+=2.0*M_PI;

	// Update the localisation data all at once
	taskENTER_CRITICAL(); // protects posX, posY and heading from getting updated independently
	posX+=dX;
	posY+=dY;
	heading=headingtemp;
	dHeading=dHeadingtemp;
	taskEXIT_CRITICAL();

	// TODO: add range sensors feedback to correct heading when a wall is visible
	// TODO: correct posX and posY using range sensors with known walls
}

void MotionCtrl::setL(int32_t pwm) {
	int32_t pwmabs=abs(pwm);

	// Set new direction if it's not the same as the current one
	int8_t dir=pwm/pwmabs; // determine the direction
	if(dir!=dirL) {
		if(dir==1) {
			// forward direction
			MOT1_A_GPIO_Port->BSRR = (uint32_t)MOT1_A_Pin << 16U;
			MOT1_B_GPIO_Port->BSRR = (uint32_t)MOT1_B_Pin;
		} else if(dir==-1) {
			// reverse direction
			MOT1_A_GPIO_Port->BSRR = (uint32_t)MOT1_A_Pin;
			MOT1_B_GPIO_Port->BSRR = (uint32_t)MOT1_B_Pin << 16U;
		} else {
			// brake
			MOT1_A_GPIO_Port->BSRR = (uint32_t)MOT1_A_Pin << 16U;
			MOT1_B_GPIO_Port->BSRR = (uint32_t)MOT1_B_Pin << 16U;
		}
		// Remember the direction that's set
		dirL=dir;
	}

	// Set the new PWM output
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, clamp(pwmabs));
}

void MotionCtrl::setR(int32_t pwm) {
	int32_t pwmabs=abs(pwm);

	// Set new direction if it's not the same as the current one
	int8_t dir=pwm/pwmabs; // determine the direction
	if(dir!=dirR) {
		if(dir==1) {
			// forward direction
			MOT2_A_GPIO_Port->BSRR = (uint32_t)MOT2_A_Pin << 16U;
			MOT2_B_GPIO_Port->BSRR = (uint32_t)MOT2_B_Pin;
		} else if(dir==-1) {
			// reverse direction
			MOT2_A_GPIO_Port->BSRR = (uint32_t)MOT2_A_Pin;
			MOT2_B_GPIO_Port->BSRR = (uint32_t)MOT2_B_Pin << 16U;
		} else {
			// brake
			MOT2_A_GPIO_Port->BSRR = (uint32_t)MOT2_A_Pin << 16U;
			MOT2_B_GPIO_Port->BSRR = (uint32_t)MOT2_B_Pin << 16U;
		}
		// Remember the direction that's set
		dirR=dir;
	}

	// Set pwm
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, clamp(pwmabs));
}

uint16_t MotionCtrl::clamp(uint32_t val) {
	return (val>9999)?9999:val;
}

void MotionCtrl::setVelLin(float v) {
	float velLinSet=v*0.001;
	// acceleration limitation
	float velLdiff=velLinSet-velLin;
	if(velLdiff>=acc)
		velLin+=acc;
	else if(velLdiff<=-acc)
		velLin-=acc;
	else
		velLin=velLinSet;

	// update wheels' rotational velocities
	float velLnew=(velLin-0.5*wheel_dist*velRot)/vel_coeff/wheel_radius;
	float velRnew=(velLin+0.5*wheel_dist*velRot)/vel_coeff/wheel_radius;
	taskENTER_CRITICAL(); // Enter critical section so velocity update is not interrupted by something else
	velL=velLnew;
	velR=velRnew;
	taskEXIT_CRITICAL();
}

void MotionCtrl::setVelRot(float w) {
	velRot=w*0.001;

	// update wheels' rotational velocities
	float velLnew=(velLin-0.5*wheel_dist*velRot)/vel_coeff/wheel_radius;
	float velRnew=(velLin+0.5*wheel_dist*velRot)/vel_coeff/wheel_radius;
	taskENTER_CRITICAL(); // Enter critical section so velocity update is not interrupted by something else
	velL=velLnew;
	velR=velRnew;
	taskEXIT_CRITICAL();
}


