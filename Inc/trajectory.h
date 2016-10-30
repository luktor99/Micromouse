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

enum WAYPOINT_TYPES {WAYPOINT_POSITION, WAYPOINT_SCAN};

struct Waypoint {
	Waypoint(uint16_t x, uint16_t y) : x(x), y(y), type(WAYPOINT_POSITION), speed_scale(255) {}
	Waypoint(uint16_t x, uint16_t y, uint8_t type, uint8_t speed_scale) : x(x), y(y), type(type), speed_scale(speed_scale) {}
	uint16_t x, y;
	uint8_t type = WAYPOINT_POSITION;
	uint8_t speed_scale = 255;
};

class TrajectoryCtrl {
public:
	void pushPointNormal(uint16_t, uint16_t);
	void popPoint();
	Waypoint target();
	uint16_t count();
private:
	std::queue<Waypoint> container;
};

extern TrajectoryCtrl Trajectory;

#endif /* TRAJECTORY_H_ */
