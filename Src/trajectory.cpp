/*
 * trajectory.cpp
 *
 *  Created on: 27 paü 2016
 *      Author: luktor99
 */

#include <common.h>
#include <hardware.h>
#include <stdlib.h>
#include <math.h>
#include <motion.h>
#include <trajectory.h>

TrajectoryCtrl Trajectory;

void *malloc(size_t size);
void free(void *ptr);

TrajectoryCtrl::TrajectoryCtrl(void) {
	// initialize trajectory building variables
	lastCellX=0;
	lastCellY=0;
	lastOrientation=UP;
}

void TrajectoryCtrl::pushCurveSearchRun(uint16_t P1X, uint16_t P1Y, uint16_t D1X, uint16_t D1Y, uint16_t P2X, uint16_t P2Y, uint16_t D2X, uint16_t D2Y) {
	container.push(BezierCurve(P1X, P1Y, D1X, D1Y, P2X, P2Y, D2X, D2Y));
}

void TrajectoryCtrl::loadCurve() {
	// calculate the required factors
	vp0X=container.front().P1X*0.001;
	vp0Y=container.front().P1Y*0.001;
	vcX=3.0*(container.front().D1X-container.front().P1X)*0.001;
	vcY=3.0*(container.front().D1Y-container.front().P1Y)*0.001;
	vbX=3.0*(container.front().D2X-container.front().D1X)*0.001-vcX;
	vbY=3.0*(container.front().D2Y-container.front().D1Y)*0.001-vcY;
	vaX=container.front().P2X*0.001-vp0X-vbX-vcX;
	vaY=container.front().P2Y*0.001-vp0Y-vbY-vcY;

	// calculate curve's finish direction vector
	finish_x = ((vaX + vbX) + vcX) - ((vaX*0.9999 + vbX)*0.9999 + vcX)*0.9999;
	finish_y = ((vaY + vbY) + vcY) - ((vaY*0.9999 + vbY)*0.9999 + vcY)*0.9999;
	float length = sqrt(finish_x*finish_x + finish_y*finish_y);
	finish_x /= length;
	finish_y /= length;

	// TODO: load speed and type of the curve as well

	// remove the curve from the queue
	container.pop();
}

void TrajectoryCtrl::updateTarget(float t) {
	if(t<=1.0) {
		targetX = ((vaX*t + vbX)*t + vcX)*t + vp0X;
		targetY = ((vaY*t + vbY)*t + vcY)*t + vp0Y;
	} else {
		targetX = ((vaX + vbX) + vcX) + vp0X + finish_x*(t-1.0);
		targetX = ((vaY + vbY) + vcY) + vp0Y + finish_y*(t-1.0);
	}
}

float TrajectoryCtrl::stepDelta(float t) {
	const float ds=0.01; // 1 cm

	float f1=invFuncS(t);
	float f2=invFuncS(t+0.5*ds*f1);
	float f3=invFuncS(t+0.5*ds*f2);
	float f4=invFuncS(t+ds*f3);

	return ds*(f1+2.0*f2+2.0*f3+f4)/6.0;
}

float TrajectoryCtrl::invFuncS(float t) {
	float dx = (3.0*vaX*t + 2.0*vbX)*t + vcX;
	float dy = (3.0*vaY*t + 2.0*vbY)*t + vcY;

	float out = dx*dx + dy*dy;

	return 1.0/sqrt(out);

	//	// fast inverse square root
	//	long i;
	//	float x2;
	//	const float threehalfs = 1.5F;
	//
	//	x2 = out * 0.5F;
	//	i  = * ( long * ) &out;
	//	i  = 0x5f3759df - ( i >> 1 );
	//	out  = * ( float * ) &i;
	//	out  = out * ( threehalfs - ( x2 * out * out ) );   // 1st iteration
	//    out  = out * ( threehalfs - ( x2 * out * out ) );   // 2nd iteration
	//
	//    // return 1/S(t)
	//	return out;
}

void TrajectoryCtrl::addSearchMove(uint8_t move) {
	uint16_t P1X, P1Y, D1X, D1Y, P2X, P2Y, D2X, D2Y;

	if(lastOrientation == UP) {
		if(move==MS_FORWARD) {
			P1X = cellToPos(lastCellX);
			P1Y = cellToPos(lastCellY) + CELL_HALF;
			D1X = P1X;
			D1Y = P1Y + CELL_HALF;
			P2X = P1X;
			P2Y = P1Y + CELL_FULL;
			D2X = D1X;
			D2Y = D1Y;
			//lastOrientation = UP;
			lastCellY++;
		}
		else if(move==MS_RIGHT) {
			P1X = cellToPos(lastCellX);
			P1Y = cellToPos(lastCellY) + CELL_HALF;
			D1X = P1X;
			D1Y = P1Y + SCAN_IN;
			P2X = P1X + CELL_HALF;
			P2Y = P1Y + CELL_HALF;
			D2X = P2X - SCAN_OUT;
			D2Y = P2Y;
			lastOrientation = RIGHT;
			lastCellY++;
		}
		else if(move==MS_LEFT) {
			P1X = cellToPos(lastCellX);
			P1Y = cellToPos(lastCellY) + CELL_HALF;
			D1X = P1X;
			D1Y = P1Y + SCAN_IN;
			P2X = P1X - CELL_HALF;
			P2Y = P1Y + CELL_HALF;
			D2X = P2X + SCAN_OUT;
			D2Y = P2Y;
			lastOrientation = LEFT;
			lastCellY++;
		}
		else if(move==MS_BACK) {
			P1X = cellToPos(lastCellX);
			P1Y = cellToPos(lastCellY) + CELL_HALF;
			D1X = P1X;
			D1Y = P1Y - CELL_HALF;
			P2X = P1X;
			P2Y = P1Y - CELL_FULL;
			D2X = D1X;
			D2Y = D1Y;
			lastOrientation = DOWN;
		}
		else if(move==MS_BACKRIGHT) {
			P1X = cellToPos(lastCellX);
			P1Y = cellToPos(lastCellY) + CELL_HALF;
			D1X = P1X;
			D1Y = P1Y - SCAN_IN;
			P2X = P1X + CELL_HALF;
			P2Y = P1Y - CELL_HALF;
			D2X = P2X - SCAN_OUT;
			D2Y = P2Y;
			lastOrientation = RIGHT;
		}
		else if(move==MS_BACKLEFT) {
			P1X = cellToPos(lastCellX);
			P1Y = cellToPos(lastCellY) + CELL_HALF;
			D1X = P1X;
			D1Y = P1Y - SCAN_IN;
			P2X = P1X - CELL_HALF;
			P2Y = P1Y - CELL_HALF;
			D2X = P2X + SCAN_OUT;
			D2Y = P2Y;
			lastOrientation = LEFT;
		}

		Trajectory.pushCurveSearchRun(P1X, P1Y, D1X, D1Y, P2X, P2Y, D2X, D2Y);
	}
	else if(lastOrientation == RIGHT) {
		if(move==MS_FORWARD) {
			P1X = cellToPos(lastCellX) + CELL_HALF;
			P1Y = cellToPos(lastCellY);
			D1X = P1X + CELL_HALF;
			D1Y = P1Y;
			P2X = P1X + CELL_FULL;
			P2Y = P1Y;
			D2X = D1X;
			D2Y = D1Y;
			//lastOrientation = RIGHT;
			lastCellX++;
		}
		else if(move==MS_RIGHT) {
			P1X = cellToPos(lastCellX) + CELL_HALF;
			P1Y = cellToPos(lastCellY);
			D1X = P1X + SCAN_IN;
			D1Y = P1Y;
			P2X = P1X + CELL_HALF;
			P2Y = P1Y - CELL_HALF;
			D2X = P2X;
			D2Y = P2Y + SCAN_OUT;
			lastOrientation = DOWN;
			lastCellX++;
		}
		else if(move==MS_LEFT) {
			P1X = cellToPos(lastCellX) + CELL_HALF;
			P1Y = cellToPos(lastCellY);
			D1X = P1X + SCAN_IN;
			D1Y = P1Y;
			P2X = P1X + CELL_HALF;
			P2Y = P1Y + CELL_HALF;
			D2X = P2X;
			D2Y = P2Y - SCAN_OUT;
			lastOrientation = UP;
			lastCellX++;
		}
		else if(move==MS_BACK) {
			P1X = cellToPos(lastCellX) + CELL_HALF;
			P1Y = cellToPos(lastCellY);
			D1X = P1X - CELL_HALF;
			D1Y = P1Y;
			P2X = P1X - CELL_FULL;
			P2Y = P1Y;
			D2X = D1X;
			D2Y = D1Y;
			lastOrientation = LEFT;
		}
		else if(move==MS_BACKRIGHT) {
			P1X = cellToPos(lastCellX) + CELL_HALF;
			P1Y = cellToPos(lastCellY);
			D1X = P1X - SCAN_IN;
			D1Y = P1Y;
			P2X = P1X - CELL_HALF;
			P2Y = P1Y - CELL_HALF;
			D2X = P2X;
			D2Y = P2Y + SCAN_OUT;
			lastOrientation = DOWN;
		}
		else if(move==MS_BACKLEFT) {
			P1X = cellToPos(lastCellX) + CELL_HALF;
			P1Y = cellToPos(lastCellY);
			D1X = P1X - SCAN_IN;
			D1Y = P1Y;
			P2X = P1X - CELL_HALF;
			P2Y = P1Y + CELL_HALF;
			D2X = P2X;
			D2Y = P2Y - SCAN_OUT;
			lastOrientation = UP;
		}

		Trajectory.pushCurveSearchRun(P1X, P1Y, D1X, D1Y, P2X, P2Y, D2X, D2Y);
	}
	else if(lastOrientation == LEFT) {
		if(move==MS_FORWARD) {
			P1X = cellToPos(lastCellX) - CELL_HALF;
			P1Y = cellToPos(lastCellY);
			D1X = P1X - CELL_HALF;
			D1Y = P1Y;
			P2X = P1X - CELL_FULL;
			P2Y = P1Y;
			D2X = D1X;
			D2Y = D1Y;
			//lastOrientation = LEFT;
			lastCellX--;
		}
		else if(move==MS_RIGHT) {
			P1X = cellToPos(lastCellX) - CELL_HALF;
			P1Y = cellToPos(lastCellY);
			D1X = P1X - SCAN_IN;
			D1Y = P1Y;
			P2X = P1X - CELL_HALF;
			P2Y = P1Y + CELL_HALF;
			D2X = P2X;
			D2Y = P2Y - SCAN_OUT;
			lastOrientation = UP;
			lastCellX--;
		}
		else if(move==MS_LEFT) {
			P1X = cellToPos(lastCellX) - CELL_HALF;
			P1Y = cellToPos(lastCellY);
			D1X = P1X - SCAN_IN;
			D1Y = P1Y;
			P2X = P1X - CELL_HALF;
			P2Y = P1Y - CELL_HALF;
			D2X = P2X;
			D2Y = P2Y + SCAN_OUT;
			lastOrientation = DOWN;
			lastCellX--;
		}
		else if(move==MS_BACK) {
			P1X = cellToPos(lastCellX) - CELL_HALF;
			P1Y = cellToPos(lastCellY);
			D1X = P1X + CELL_HALF;
			D1Y = P1Y;
			P2X = P1X + CELL_FULL;
			P2Y = P1Y;
			D2X = D1X;
			D2Y = D1Y;
			lastOrientation = RIGHT;
		}
		else if(move==MS_BACKRIGHT) {
			P1X = cellToPos(lastCellX) - CELL_HALF;
			P1Y = cellToPos(lastCellY);
			D1X = P1X + SCAN_IN;
			D1Y = P1Y;
			P2X = P1X + CELL_HALF;
			P2Y = P1Y + CELL_HALF;
			D2X = P2X;
			D2Y = P2Y - SCAN_OUT;
			lastOrientation = UP;
		}
		else if(move==MS_BACKLEFT) {
			P1X = cellToPos(lastCellX) - CELL_HALF;
			P1Y = cellToPos(lastCellY);
			D1X = P1X + SCAN_IN;
			D1Y = P1Y;
			P2X = P1X + CELL_HALF;
			P2Y = P1Y - CELL_HALF;
			D2X = P2X;
			D2Y = P2Y + SCAN_OUT;
			lastOrientation = DOWN;
		}

		Trajectory.pushCurveSearchRun(P1X, P1Y, D1X, D1Y, P2X, P2Y, D2X, D2Y);
	}
	else if(lastOrientation == DOWN) {
		if(move==MS_FORWARD) {
			P1X = cellToPos(lastCellX);
			P1Y = cellToPos(lastCellY) - CELL_HALF;
			D1X = P1X;
			D1Y = P1Y - CELL_HALF;
			P2X = P1X;
			P2Y = P1Y - CELL_FULL;
			D2X = D1X;
			D2Y = D1Y;
			//lastOrientation = DOWN;
			lastCellY--;
		}
		else if(move==MS_RIGHT) {
			P1X = cellToPos(lastCellX);
			P1Y = cellToPos(lastCellY) - CELL_HALF;
			D1X = P1X;
			D1Y = P1Y - SCAN_IN;
			P2X = P1X - CELL_HALF;
			P2Y = P1Y - CELL_HALF;
			D2X = P2X + SCAN_OUT;
			D2Y = P2Y;
			lastOrientation = LEFT;
			lastCellY--;
		}
		else if(move==MS_LEFT) {
			P1X = cellToPos(lastCellX);
			P1Y = cellToPos(lastCellY) - CELL_HALF;
			D1X = P1X;
			D1Y = P1Y - SCAN_IN;
			P2X = P1X + CELL_HALF;
			P2Y = P1Y - CELL_HALF;
			D2X = P2X - SCAN_OUT;
			D2Y = P2Y;
			lastOrientation = RIGHT;
			lastCellY--;
		}
		else if(move==MS_BACK) {
			P1X = cellToPos(lastCellX);
			P1Y = cellToPos(lastCellY) - CELL_HALF;
			D1X = P1X;
			D1Y = P1Y + CELL_HALF;
			P2X = P1X;
			P2Y = P1Y + CELL_FULL;
			D2X = D1X;
			D2Y = D1Y;
			lastOrientation = UP;
		}
		else if(move==MS_BACKRIGHT) {
			P1X = cellToPos(lastCellX);
			P1Y = cellToPos(lastCellY) - CELL_HALF;
			D1X = P1X;
			D1Y = P1Y + SCAN_IN;
			P2X = P1X - CELL_HALF;
			P2Y = P1Y + CELL_HALF;
			D2X = P2X + SCAN_OUT;
			D2Y = P2Y;
			lastOrientation = LEFT;
		}
		else if(move==MS_BACKLEFT) {
			P1X = cellToPos(lastCellX);
			P1Y = cellToPos(lastCellY) - CELL_HALF;
			D1X = P1X;
			D1Y = P1Y + SCAN_IN;
			P2X = P1X + CELL_HALF;
			P2Y = P1Y + CELL_HALF;
			D2X = P2X - SCAN_OUT;
			D2Y = P2Y;
			lastOrientation = RIGHT;
		}

		Trajectory.pushCurveSearchRun(P1X, P1Y, D1X, D1Y, P2X, P2Y, D2X, D2Y);
	}
}

uint16_t TrajectoryCtrl::count() {
	return container.size();
}

uint16_t TrajectoryCtrl::cellToPos(uint16_t cell) {
	return cell*180+90;
}


void *malloc(size_t size) {
	return pvPortMalloc(size);
}

void free(void *ptr){
	vPortFree(ptr);
}
