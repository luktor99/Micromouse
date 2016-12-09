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
	lastCellX = 0;
	lastCellY = 0;
	lastOrientation = UP;

	// Initialize Bezier curve variables
	step = 0.0;
	must_stop = 0;
	signal_sent = 0;
	targetX = 0.0;
	targetY = 0.0;
}

void TrajectoryCtrl::pushCurveSearchRun(uint16_t P1X, uint16_t P1Y,
		uint16_t D1X, uint16_t D1Y, uint16_t P2X, uint16_t P2Y, uint16_t D2X,
		uint16_t D2Y) {
	container.push(BezierCurve(P1X, P1Y, D1X, D1Y, P2X, P2Y, D2X, D2Y));
}
void TrajectoryCtrl::pushCurveFastRun(uint16_t P1X, uint16_t P1Y, uint16_t D1X,
		uint16_t D1Y, uint16_t P2X, uint16_t P2Y, uint16_t D2X, uint16_t D2Y,
		uint8_t type, uint8_t speed1, uint8_t speed2) {
	container.push(
			BezierCurve(P1X, P1Y, D1X, D1Y, P2X, P2Y, D2X, D2Y, type, speed1,
					speed2));
}

void TrajectoryCtrl::loadCurve() {
	// calculate the required factors
	vp0X = container.front().P1X * 0.001;
	vp0Y = container.front().P1Y * 0.001;
	vcX = 3.0 * (container.front().D1X - container.front().P1X) * 0.001;
	vcY = 3.0 * (container.front().D1Y - container.front().P1Y) * 0.001;
	vbX = 3.0 * (container.front().D2X - container.front().D1X) * 0.001 - vcX;
	vbY = 3.0 * (container.front().D2Y - container.front().D1Y) * 0.001 - vcY;
	vaX = container.front().P2X * 0.001 - vp0X - vbX - vcX;
	vaY = container.front().P2Y * 0.001 - vp0Y - vbY - vcY;

	// calculate curve's finish direction vector
	finish_x = ((vaX + vbX) + vcX)
			- ((vaX * 0.9999 + vbX) * 0.9999 + vcX) * 0.9999;
	finish_y = ((vaY + vbY) + vcY)
			- ((vaY * 0.9999 + vbY) * 0.9999 + vcY) * 0.9999;
	float length = sqrt(finish_x * finish_x + finish_y * finish_y);
	finish_x /= length;
	finish_y /= length;

	// load speed and type of the curve as well
	//get parameters

	type = container.front().type;
	speed1 = container.front().speed1;
	speed2 = container.front().speed2;
	PfX = container.front().P2X;
	PfY = container.front().P2Y;

	// remove the curve from the queue
	container.pop();
}

void TrajectoryCtrl::updateTarget(float t) {
	if (t <= 1.0) {
		targetX = ((vaX * t + vbX) * t + vcX) * t + vp0X;
		targetY = ((vaY * t + vbY) * t + vcY) * t + vp0Y;
	} else {
		// Extrapolate the Bezier curve using its last direction vector (finish_x, finish_y)
		targetX = ((vaX + vbX) + vcX) + vp0X + finish_x * (t - 1.0);
		targetY = ((vaY + vbY) + vcY) + vp0Y + finish_y * (t - 1.0);
	}
}

float TrajectoryCtrl::stepDelta(float t) {
	const float ds = 0.01; // 1 cm

	float f1 = invFuncS(t);
	float f2 = invFuncS(t + 0.5 * ds * f1);
	float f3 = invFuncS(t + 0.5 * ds * f2);
	float f4 = invFuncS(t + ds * f3);

	return ds * (f1 + 2.0 * f2 + 2.0 * f3 + f4) / 6.0;
}

float TrajectoryCtrl::invFuncS(float t) {
	float dx = (3.0 * vaX * t + 2.0 * vbX) * t + vcX;
	float dy = (3.0 * vaY * t + 2.0 * vbY) * t + vcY;

	float out = dx * dx + dy * dy;

	return 1.0 / sqrt(out);

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

void TrajectoryCtrl::tick() {
	float dx = targetX - Motion.posX, dy = targetY - Motion.posY;
	float dist = sqrt(dx * dx + dy * dy); // distance to the target

	// Calculate target heading and the error
	float targetHeading = atan2(dy, dx);
	float errHeading = clampAngle(targetHeading - Motion.heading);

	//profiler

	float velLin = 0.0;
	//profiler

	// Current target captured, set the next one if available
	if (dist < dist_accuracy) {
		step += 0.01; // increment the step variable

		// check if we've just finished the current curve
		if (step >= 1.0) {
			// is there another one waiting in the queue?
			if (Trajectory.count() > 0) {
				// load the new curve
				Trajectory.loadCurve();
				// move to its first point
				step = 0.0;
				// reset signal_sent flag
				signal_sent = 0;
			} else {
				// Send a signal to start a cell scan
				if (!signal_sent)
					osSignalSet(MazeAlgorithmTaskHandle, SIGNAL_SCAN);
				signal_sent = 1;
				step = 1.0; // hold this position
			}
		}

		// Update the target position
		Trajectory.updateTarget(step);
	}

	// Check if turning in place is required
	if (Motion.velLin > 0.01 * 0.001 && fabs(errHeading) > M_PI / 2.0) {
		must_stop = 1;
	} else if (Motion.velLin < 0.01 * 0.001 && fabs(errHeading) < M_PI / 16.0) { // heading ok, start again
		must_stop = 0;
	}

	// Calculate and apply rotational velocity
	float velRot = errHeading * 10.0
			* (!must_stop || (must_stop && Motion.velLin < 0.01 * 0.001)); // if turning in place: first wait for the robot to stop, then rotate
	Motion.setVelRot(
			((velRot < velRotMax) ?
					((velRot > -velRotMax) ? (velRot) : -velRotMax) : velRotMax)
					* (dist > dist_accuracy));

	// Calculate and apply linear velocity

	if (type == CURVE_CONSTANT) {
		velLin = speed_curve_fastrun;

	} else if (type == CURVE_BRAKE) {
		distanceToTarget = sqrt(
				((float) PfX - 1000.0 * Motion.posX)
						* ((float) PfX - 1000.0 * Motion.posX)
						+ ((float) PfY - 1000.0 * Motion.posY)
								* ((float) PfY - 1000.0 * Motion.posY));
		if (speed2 > 3) {
			if (distanceToTarget >= 1.5* CELL_FULL)
				velLin = Motion.velLinMax;
			else
				velLin = speed_curve_fastrun;
		} else {
			if (distanceToTarget >= 0.5 * speed2 * CELL_FULL)
				velLin = Motion.velLinMax;
			else
				velLin = speed_curve_fastrun;
		}


	} else if (type == CURVE_SEARCHRUN) {
		velLin = Motion.velLinMax;
	} else {
		print("Curve type err\r\n");
		for (;;)
			;
	}

	velLin = velLin * (!must_stop); // stop if turning in place
	Motion.setVelLin(((velLin > velLinMin) ? velLin : velLinMin));
}

void TrajectoryCtrl::addFastMove(uint8_t move) {

	//debug printing
	if ((move >= MF_FORWARD) && (move <= MF_FORWARD + 13))
		print("FF %d\r\n ", move);
	else if (move == MF_LEFT)
		print("FL %d\r\n", move);
	else if (move == MF_RIGHT)
		print("FR %d \r\n", move);
	//points defining bezier curve
	uint16_t P1X, P1Y, D1X, D1Y, P2X, P2Y, D2X, D2Y;

	uint8_t type = CURVE_CONSTANT;
	uint8_t speed1 = 255;
	uint8_t speed2 = 255;

	if (lastOrientation == UP) {
		if ((move >= MF_FORWARD) && (move <= MF_FORWARD + 13)) {
			P1X = cellToPos(lastCellX);
			P1Y = cellToPos(lastCellY) + CELL_HALF;
			D1X = P1X;
			D1Y = P1Y + CELL_HALF;
			P2X = P1X;
			P2Y = P1Y + (move + 1 - MF_FORWARD) * CELL_FULL;
			D2X = D1X;
			D2Y = D1Y;
			//lastOrientation = UP;
			speed1 = 1;
			speed2 = (move + 1 - MF_FORWARD);
			type = CURVE_BRAKE;
			lastCellY += move + 1 - MF_FORWARD;
			//print("LastCellY %d ", lastCellY);
		}

		else if (move == MF_RIGHT) {
			P1X = cellToPos(lastCellX);
			P1Y = cellToPos(lastCellY) + CELL_HALF;
			D1X = P1X;
			D1Y = P1Y + SCAN_IN;
			P2X = P1X + CELL_HALF;
			P2Y = P1Y + CELL_HALF;
			D2X = P2X - SCAN_OUT;
			D2Y = P2Y;
			speed1 = 1;
			speed2 = 1;
			lastOrientation = RIGHT;
			lastCellY++;
		} else if (move == MF_LEFT) {
			P1X = cellToPos(lastCellX);
			P1Y = cellToPos(lastCellY) + CELL_HALF;
			D1X = P1X;
			D1Y = P1Y + SCAN_IN;
			P2X = P1X - CELL_HALF;
			P2Y = P1Y + CELL_HALF;
			D2X = P2X + SCAN_OUT;
			D2Y = P2Y;
			speed1 = 1;
			speed2 = 1;
			lastOrientation = LEFT;
			lastCellY++;
		} else if (move == M_START) {
			P1X = cellToPos(lastCellX);
			P1Y = cellToPos(lastCellY);
			D1X = P1X;
			D1Y = P1Y + CELL_QUARTER;
			P2X = P1X;
			P2Y = P1Y + CELL_HALF;
			D2X = D1X;
			D2Y = D1Y;
			//lastOrientation = UP;
		} else if (move == M_FINISH) {
			P1X = cellToPos(lastCellX);
			P1Y = cellToPos(lastCellY) + CELL_HALF;
			D1X = P1X;
			D1Y = P1Y + CELL_HALF;
			P2X = P1X;
			P2Y = P1Y + CELL_HALF;
			D2X = D1X;
			D2Y = D1Y;
			//lastOrientation = UP;
			lastCellY++;
		}

		Trajectory.pushCurveFastRun(P1X, P1Y, D1X, D1Y, P2X, P2Y, D2X, D2Y,
				type, speed1, speed2);
	}

	else if (lastOrientation == RIGHT) {
		if ((move >= MF_FORWARD) && (move <= MF_FORWARD + 13)) {

			P1X = cellToPos(lastCellX) + CELL_HALF;
			P1Y = cellToPos(lastCellY);

			D1X = P1X + CELL_HALF;
			D1Y = P1Y;

			P2X = P1X + (move + 1 - MF_FORWARD) * CELL_FULL;
			P2Y = P1Y;
			D2X = D1X;
			D2Y = D1Y;
			speed1 = 1;
			speed2 = (move + 1 - MF_FORWARD);
			type = CURVE_BRAKE;
			//lastOrientation = RIGHT;
			lastCellX += move + 1 - MF_FORWARD;

		} else if (move == MF_RIGHT) {
			P1X = cellToPos(lastCellX) + CELL_HALF;
			P1Y = cellToPos(lastCellY);
			D1X = P1X + SCAN_IN;
			D1Y = P1Y;
			P2X = P1X + CELL_HALF;
			P2Y = P1Y - CELL_HALF;
			D2X = P2X;
			D2Y = P2Y + SCAN_OUT;
			speed1 = 1;
			speed2 = 1;
			lastOrientation = DOWN;
			lastCellX++;
		} else if (move == MF_LEFT) {
			P1X = cellToPos(lastCellX) + CELL_HALF;
			P1Y = cellToPos(lastCellY);
			D1X = P1X + SCAN_IN;
			D1Y = P1Y;
			P2X = P1X + CELL_HALF;
			P2Y = P1Y + CELL_HALF;
			D2X = P2X;
			D2Y = P2Y - SCAN_OUT;
			speed1 = 1;
			speed2 = 1;
			lastOrientation = UP;
			lastCellX++;
		} else if (move == M_FINISH) {
			P1X = cellToPos(lastCellX) + CELL_HALF;
			P1Y = cellToPos(lastCellY);
			D1X = P1X + CELL_HALF;
			D1Y = P1Y;
			P2X = P1X + CELL_HALF;
			P2Y = P1Y;
			D2X = D1X;
			D2Y = D1Y;
			//lastOrientation = RIGHT;
			lastCellX++;
		}

		Trajectory.pushCurveFastRun(P1X, P1Y, D1X, D1Y, P2X, P2Y, D2X, D2Y,
				type, speed1, speed2);
	}

	else if (lastOrientation == LEFT) {
		if ((move >= MF_FORWARD) && (move <= MF_FORWARD + 13)) {

			P1X = cellToPos(lastCellX) - CELL_HALF;
			P1Y = cellToPos(lastCellY);
			D1X = P1X - CELL_HALF;
			D1Y = P1Y;
			P2X = P1X - (move + 1 - MF_FORWARD) * CELL_FULL;
			P2Y = P1Y;
			D2X = D1X;
			D2Y = D1Y;
			speed1 = 1;
			speed2 = (move + 1 - MF_FORWARD);
			type = CURVE_BRAKE;
			//lastOrientation = LEFT;
			lastCellX -= (move + 1 - MF_FORWARD);

		} else if (move == MF_RIGHT) {
			P1X = cellToPos(lastCellX) - CELL_HALF;
			P1Y = cellToPos(lastCellY);
			D1X = P1X - SCAN_IN;
			D1Y = P1Y;
			P2X = P1X - CELL_HALF;
			P2Y = P1Y + CELL_HALF;
			D2X = P2X;
			D2Y = P2Y - SCAN_OUT;
			speed1 = 1;
			speed2 = 1;
			lastOrientation = UP;
			lastCellX--;
		} else if (move == MF_LEFT) {
			P1X = cellToPos(lastCellX) - CELL_HALF;
			P1Y = cellToPos(lastCellY);
			D1X = P1X - SCAN_IN;
			D1Y = P1Y;
			P2X = P1X - CELL_HALF;
			P2Y = P1Y - CELL_HALF;
			D2X = P2X;
			D2Y = P2Y + SCAN_OUT;
			speed1 = 1;
			speed2 = 1;
			lastOrientation = DOWN;
			lastCellX--;
		}

		else if (move == M_FINISH) {
			P1X = cellToPos(lastCellX) - CELL_HALF;
			P1Y = cellToPos(lastCellY);
			D1X = P1X - CELL_HALF;
			D1Y = P1Y;
			P2X = P1X - CELL_HALF;
			P2Y = P1Y;
			D2X = D1X;
			D2Y = D1Y;
			//lastOrientation = LEFT;
			lastCellX--;
		}
		Trajectory.pushCurveFastRun(P1X, P1Y, D1X, D1Y, P2X, P2Y, D2X, D2Y,
				type, speed1, speed2);
	} else if (lastOrientation == DOWN) {
		if ((move >= MF_FORWARD) && (move <= MF_FORWARD + 13)) {
			P1X = cellToPos(lastCellX);
			P1Y = cellToPos(lastCellY) - CELL_HALF;
			D1X = P1X;
			D1Y = P1Y - CELL_HALF;
			P2X = P1X;
			P2Y = P1Y - (move + 1 - MF_FORWARD) * CELL_FULL;
			D2X = D1X;
			D2Y = D1Y;
			speed1 = 1;
			speed2 = (move + 1 - MF_FORWARD);
			type = CURVE_BRAKE;
			//lastOrientation = DOWN;
			lastCellY -= (move + 1 - MF_FORWARD);
		} else if (move == MF_RIGHT) {
			P1X = cellToPos(lastCellX);
			P1Y = cellToPos(lastCellY) - CELL_HALF;
			D1X = P1X;
			D1Y = P1Y - SCAN_IN;
			P2X = P1X - CELL_HALF;
			P2Y = P1Y - CELL_HALF;
			D2X = P2X + SCAN_OUT;
			D2Y = P2Y;
			speed1 = 1;
			speed2 = 1;
			lastOrientation = LEFT;
			lastCellY--;
		} else if (move == MF_LEFT) {
			P1X = cellToPos(lastCellX);
			P1Y = cellToPos(lastCellY) - CELL_HALF;
			D1X = P1X;
			D1Y = P1Y - SCAN_IN;
			P2X = P1X + CELL_HALF;
			P2Y = P1Y - CELL_HALF;
			D2X = P2X - SCAN_OUT;
			D2Y = P2Y;
			speed1 = 1;
			speed2 = 1;
			lastOrientation = RIGHT;
			lastCellY--;
		} else if (move == M_FINISH) {
			P1X = cellToPos(lastCellX);
			P1Y = cellToPos(lastCellY) - CELL_HALF;
			D1X = P1X;
			D1Y = P1Y - CELL_HALF;
			P2X = P1X;
			P2Y = P1Y - CELL_HALF;
			D2X = D1X;
			D2Y = D1Y;
			//lastOrientation = DOWN;
			lastCellY--;
		}

		Trajectory.pushCurveFastRun(P1X, P1Y, D1X, D1Y, P2X, P2Y, D2X, D2Y,
				type, speed1, speed2);
	}
}

void TrajectoryCtrl::addSearchMove(uint8_t move) {
	// debug printing:
	if (move == MS_FORWARD)
		print("F ");
	else if (move == MS_LEFT)
		print("L ");
	else if (move == MS_RIGHT)
		print("R ");
	else if (move == MS_BACK)
		print("B ");
	else if (move == MS_BACKLEFT)
		print("BL ");
	else if (move == MS_BACKRIGHT)
		print("BR ");

	uint16_t P1X, P1Y, D1X, D1Y, P2X, P2Y, D2X, D2Y;

	if (lastOrientation == UP) {
		if (move == M_FINISH) {
			P1X = cellToPos(lastCellX);
			P1Y = cellToPos(lastCellY) + CELL_HALF;
			D1X = P1X;
			D1Y = P1Y + CELL_HALF;
			P2X = P1X;
			P2Y = P1Y + CELL_HALF;
			D2X = D1X;
			D2Y = D1Y;
			//lastOrientation = UP;
			lastCellY++;
		}

		if (move == MS_FORWARD) {
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
		} else if (move == MS_RIGHT) {
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
		} else if (move == MS_LEFT) {
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
		} else if (move == MS_BACK) {
			P1X = cellToPos(lastCellX);
			P1Y = cellToPos(lastCellY) + CELL_HALF;
			D1X = P1X;
			D1Y = P1Y - CELL_HALF;
			P2X = P1X;
			P2Y = P1Y - CELL_FULL;
			D2X = D1X;
			D2Y = D1Y;
			lastOrientation = DOWN;
		} else if (move == MS_BACKRIGHT) {
			P1X = cellToPos(lastCellX);
			P1Y = cellToPos(lastCellY) + CELL_HALF;
			D1X = P1X;
			D1Y = P1Y - SCAN_IN;
			P2X = P1X + CELL_HALF;
			P2Y = P1Y - CELL_HALF;
			D2X = P2X - SCAN_OUT;
			D2Y = P2Y;
			lastOrientation = RIGHT;
		} else if (move == MS_BACKLEFT) {
			P1X = cellToPos(lastCellX);
			P1Y = cellToPos(lastCellY) + CELL_HALF;
			D1X = P1X;
			D1Y = P1Y - SCAN_IN;
			P2X = P1X - CELL_HALF;
			P2Y = P1Y - CELL_HALF;
			D2X = P2X + SCAN_OUT;
			D2Y = P2Y;
			lastOrientation = LEFT;
		} else if (move == M_START) {
			P1X = cellToPos(lastCellX);
			P1Y = cellToPos(lastCellY);
			D1X = P1X;
			D1Y = P1Y + CELL_QUARTER;
			P2X = P1X;
			P2Y = P1Y + CELL_HALF;
			D2X = D1X;
			D2Y = D1Y;
			//lastOrientation = UP;
		}

		Trajectory.pushCurveSearchRun(P1X, P1Y, D1X, D1Y, P2X, P2Y, D2X, D2Y);
	} else if (lastOrientation == RIGHT) {
		if (move == M_FINISH) {
			P1X = cellToPos(lastCellX) + CELL_HALF;
			P1Y = cellToPos(lastCellY);
			D1X = P1X + CELL_HALF;
			D1Y = P1Y;
			P2X = P1X + CELL_HALF;
			P2Y = P1Y;
			D2X = D1X;
			D2Y = D1Y;
			//lastOrientation = RIGHT;
			lastCellX++;
		}

		if (move == MS_FORWARD) {
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
		} else if (move == MS_RIGHT) {
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
		} else if (move == MS_LEFT) {
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
		} else if (move == MS_BACK) {
			P1X = cellToPos(lastCellX) + CELL_HALF;
			P1Y = cellToPos(lastCellY);
			D1X = P1X - CELL_HALF;
			D1Y = P1Y;
			P2X = P1X - CELL_FULL;
			P2Y = P1Y;
			D2X = D1X;
			D2Y = D1Y;
			lastOrientation = LEFT;
		} else if (move == MS_BACKRIGHT) {
			P1X = cellToPos(lastCellX) + CELL_HALF;
			P1Y = cellToPos(lastCellY);
			D1X = P1X - SCAN_IN;
			D1Y = P1Y;
			P2X = P1X - CELL_HALF;
			P2Y = P1Y - CELL_HALF;
			D2X = P2X;
			D2Y = P2Y + SCAN_OUT;
			lastOrientation = DOWN;
		} else if (move == MS_BACKLEFT) {
			P1X = cellToPos(lastCellX) + CELL_HALF;
			P1Y = cellToPos(lastCellY);
			D1X = P1X - SCAN_IN;
			D1Y = P1Y;
			P2X = P1X - CELL_HALF;
			P2Y = P1Y + CELL_HALF;
			D2X = P2X;
			D2Y = P2Y - SCAN_OUT;
			lastOrientation = UP;
		} else if (move == M_START) {
			P1X = cellToPos(lastCellX);
			P1Y = cellToPos(lastCellY);
			D1X = P1X + CELL_QUARTER;
			D1Y = P1Y;
			P2X = P1X + CELL_HALF;
			P2Y = P1Y;
			D2X = D1X;
			D2Y = D1Y;
			//lastOrientation = RIGHT;
		}

		Trajectory.pushCurveSearchRun(P1X, P1Y, D1X, D1Y, P2X, P2Y, D2X, D2Y);
	} else if (lastOrientation == LEFT) {
		if (move == M_FINISH) {
			P1X = cellToPos(lastCellX) - CELL_HALF;
			P1Y = cellToPos(lastCellY);
			D1X = P1X - CELL_HALF;
			D1Y = P1Y;
			P2X = P1X - CELL_HALF;
			P2Y = P1Y;
			D2X = D1X;
			D2Y = D1Y;
			//lastOrientation = LEFT;
			lastCellX--;
		}

		if (move == MS_FORWARD) {
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
		} else if (move == MS_RIGHT) {
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
		} else if (move == MS_LEFT) {
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
		} else if (move == MS_BACK) {
			P1X = cellToPos(lastCellX) - CELL_HALF;
			P1Y = cellToPos(lastCellY);
			D1X = P1X + CELL_HALF;
			D1Y = P1Y;
			P2X = P1X + CELL_FULL;
			P2Y = P1Y;
			D2X = D1X;
			D2Y = D1Y;
			lastOrientation = RIGHT;
		} else if (move == MS_BACKRIGHT) {
			P1X = cellToPos(lastCellX) - CELL_HALF;
			P1Y = cellToPos(lastCellY);
			D1X = P1X + SCAN_IN;
			D1Y = P1Y;
			P2X = P1X + CELL_HALF;
			P2Y = P1Y + CELL_HALF;
			D2X = P2X;
			D2Y = P2Y - SCAN_OUT;
			lastOrientation = UP;
		} else if (move == MS_BACKLEFT) {
			P1X = cellToPos(lastCellX) - CELL_HALF;
			P1Y = cellToPos(lastCellY);
			D1X = P1X + SCAN_IN;
			D1Y = P1Y;
			P2X = P1X + CELL_HALF;
			P2Y = P1Y - CELL_HALF;
			D2X = P2X;
			D2Y = P2Y + SCAN_OUT;
			lastOrientation = DOWN;
		} else if (move == M_START) {
			P1X = cellToPos(lastCellX);
			P1Y = cellToPos(lastCellY);
			D1X = P1X - CELL_QUARTER;
			D1Y = P1Y;
			P2X = P1X - CELL_HALF;
			P2Y = P1Y;
			D2X = D1X;
			D2Y = D1Y;
			//lastOrientation = LEFT;
		}

		Trajectory.pushCurveSearchRun(P1X, P1Y, D1X, D1Y, P2X, P2Y, D2X, D2Y);
	} else if (lastOrientation == DOWN) {

		if (move == M_FINISH) {
			P1X = cellToPos(lastCellX);
			P1Y = cellToPos(lastCellY) - CELL_HALF;
			D1X = P1X;
			D1Y = P1Y - CELL_HALF;
			P2X = P1X;
			P2Y = P1Y - CELL_HALF;
			D2X = D1X;
			D2Y = D1Y;
			//lastOrientation = DOWN;
			lastCellY--;
		}

		if (move == MS_FORWARD) {
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
		} else if (move == MS_RIGHT) {
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
		} else if (move == MS_LEFT) {
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
		} else if (move == MS_BACK) {
			P1X = cellToPos(lastCellX);
			P1Y = cellToPos(lastCellY) - CELL_HALF;
			D1X = P1X;
			D1Y = P1Y + CELL_HALF;
			P2X = P1X;
			P2Y = P1Y + CELL_FULL;
			D2X = D1X;
			D2Y = D1Y;
			lastOrientation = UP;
		} else if (move == MS_BACKRIGHT) {
			P1X = cellToPos(lastCellX);
			P1Y = cellToPos(lastCellY) - CELL_HALF;
			D1X = P1X;
			D1Y = P1Y + SCAN_IN;
			P2X = P1X - CELL_HALF;
			P2Y = P1Y + CELL_HALF;
			D2X = P2X + SCAN_OUT;
			D2Y = P2Y;
			lastOrientation = LEFT;
		} else if (move == MS_BACKLEFT) {
			P1X = cellToPos(lastCellX);
			P1Y = cellToPos(lastCellY) - CELL_HALF;
			D1X = P1X;
			D1Y = P1Y + SCAN_IN;
			P2X = P1X + CELL_HALF;
			P2Y = P1Y + CELL_HALF;
			D2X = P2X - SCAN_OUT;
			D2Y = P2Y;
			lastOrientation = RIGHT;
		} else if (move == M_START) {
			P1X = cellToPos(lastCellX);
			P1Y = cellToPos(lastCellY);
			D1X = P1X;
			D1Y = P1Y - CELL_QUARTER;
			P2X = P1X;
			P2Y = P1Y - CELL_HALF;
			D2X = D1X;
			D2Y = D1Y;
			//lastOrientation = DOWN;
		}

		Trajectory.pushCurveSearchRun(P1X, P1Y, D1X, D1Y, P2X, P2Y, D2X, D2Y);
	}
}

uint16_t TrajectoryCtrl::count() {
	return container.size();
}

uint16_t TrajectoryCtrl::cellToPos(uint16_t cell) {
	return cell * 180 + 90;
}

void TrajectoryCtrl::reset() {
	lastOrientation = UP;
	lastCellX = 0;
	lastCellY = 0;

	// Initialize Bezier curve variables
	step = 0.0;
	must_stop = 0;
	signal_sent = 0;
	targetX = 0.09;
	targetY = 0.09;
}

void TrajectoryCtrl::clear() {
	// Remove all elements from the queue
	while (container.size()) {
		container.pop();
	}
}

void *malloc(size_t size) {
	return pvPortMalloc(size);
}

void free(void *ptr) {
	vPortFree(ptr);
}

