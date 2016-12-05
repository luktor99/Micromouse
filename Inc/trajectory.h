/*
 * trajectory.h
 *
 *  Created on: 25 paü 2016
 *      Author: luktor99
 */

#ifndef TRAJECTORY_H_
#define TRAJECTORY_H_

#include <stdint.h>
#include <queue>

// Trajectory follow constants
const float velLinMin=0.0; // Min linear velocity (to prevent the robot from going backwards in certain conditions)
const float velRotMax=30.0; // Max rotational velocity
const float dist_accuracy=0.015; // accuracy of position following (20mm)

// shape coefficients of the bezier curve trajectories:
const uint16_t SCAN_IN = 45; // start of a turn (SEARCH RUN)
const uint16_t SCAN_OUT = 90; // end of a turn (SEARCH RUN)

// size of a cell
const uint16_t CELL_QUARTER = 45;
const uint16_t CELL_HALF = 90;
const uint16_t CELL_FULL = 180;

enum moves {
	MS_FORWARD, MS_LEFT, MS_RIGHT, MS_BACK, MS_BACKLEFT, MS_BACKRIGHT, // search run moves
	M_START, M_FINISH, // START gets from the border to the center of a cell, FINISH does the opposite
	MF_FORWARD=8, MF_LEFT=22, MF_RIGHT
};

enum CURVE_TYPES {
	CURVE_SEARCHRUN=0, // full speed for any path (useful only for search run trajectories)
	CURVE_CONSTANT, // constant speed (e.g. for fast run turns)
	CURVE_LINEAR, // speed scale is increased in a linear fashion (evenly from speed1 to speed2)
	CURVE_BRAKE, // speed is kept at speed1 level for as long as possible and then is decreased to speed2 (to be used before turning points)
	CURVE_SCAN=0x80 // (FLAG) call the floodfill algorithm at the end of the curve
};

struct BezierCurve {
	// Search run only constructor
	BezierCurve(uint16_t P1X, uint16_t P1Y, uint16_t D1X, uint16_t D1Y, uint16_t P2X, uint16_t P2Y, uint16_t D2X, uint16_t D2Y) : P1X(P1X), P1Y(P1Y), D1X(D1X), D1Y(D1Y), P2X(P2X), P2Y(P2Y), D2X(D2X), D2Y(D2Y), type(CURVE_SEARCHRUN), speed1(255), speed2(255) {}
	// Constant speed only constructor
	BezierCurve(uint16_t P1X, uint16_t P1Y, uint16_t D1X, uint16_t D1Y, uint16_t P2X, uint16_t P2Y, uint16_t D2X, uint16_t D2Y, uint8_t speed) : P1X(P1X), P1Y(P1Y), D1X(D1X), D1Y(D1Y), P2X(P2X), P2Y(P2Y), D2X(D2X), D2Y(D2Y), type(CURVE_CONSTANT), speed1(speed), speed2(speed) {}
	// Normal constructor
	BezierCurve(uint16_t P1X, uint16_t P1Y, uint16_t D1X, uint16_t D1Y, uint16_t P2X, uint16_t P2Y, uint16_t D2X, uint16_t D2Y, uint8_t type, uint8_t speed1, uint8_t speed2) : P1X(P1X), P1Y(P1Y), D1X(D1X), D1Y(D1Y), P2X(P2X), P2Y(P2Y), D2X(D2X), D2Y(D2Y), type(type), speed1(speed1), speed2(speed2) {}
	uint16_t P1X, P1Y; // first point;
	uint16_t D1X, D1Y; // direction point for the first point;
	uint16_t P2X, P2Y; // last point;
	uint16_t D2X, D2Y; // direction point for the last point;
	uint8_t type; // for one of the CURVE_TYPES
	uint8_t speed1; // speed scale at the first point
	uint8_t speed2; // speed scale at the last point


};

class TrajectoryCtrl {
public:
	TrajectoryCtrl();
	void tick();
	void pushCurveSearchRun(uint16_t P1X, uint16_t P1Y, uint16_t D1X, uint16_t D1Y, uint16_t P2X, uint16_t P2Y, uint16_t D2X, uint16_t D2Y);
	void pushCurveFastRun(uint16_t P1X, uint16_t P1Y, uint16_t D1X, uint16_t D1Y, uint16_t P2X, uint16_t P2Y, uint16_t D2X, uint16_t D2Y,uint8_t type,uint8_t speed1, uint8_t speed2);
	void loadCurve();
	void updateTarget(float);
	float stepDelta(float t);
	uint16_t cellToPos(uint16_t cell);
	uint16_t count();
	void clear();
	void reset();

	void addSearchMove(uint8_t move);
	void addFastMove(uint8_t move);
//private:
	float invFuncS(float t);

	std::queue<BezierCurve> container;

	// target position
	float targetX, targetY;

	// curve factors
	float vaX, vaY;
	float vbX, vbY;
	float vcX, vcY;
	float vp0X, vp0Y;
	uint8_t type, speed1, speed2;

	// position on the curve
	float step;
	// flags
	uint8_t must_stop, signal_sent;

	// temporary vars that allow building longer trajectories out of the available segments
	uint16_t lastCellX, lastCellY; // end cell of the last trajectory
	uint8_t lastOrientation; // orientation of the robot after arriving to the end cell

	float finish_x, finish_y;

	//profiler variables
	float velLinFwMax;
	int breakingStepsNmb;
	float distanceToTarget;
	uint16_t PfX, PfY;
};

extern TrajectoryCtrl Trajectory;

#endif /* TRAJECTORY_H_ */
