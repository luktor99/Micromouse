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

void TrajectoryCtrl::pushPointNormal(uint16_t x, uint16_t y) {
	container.push(Waypoint(x, y));
}

void TrajectoryCtrl::popPoint() {
	container.pop();
}

Waypoint TrajectoryCtrl::target() {
	return container.front();
}

uint16_t TrajectoryCtrl::count() {
	return container.size();
}

void *malloc(size_t size) {
	return pvPortMalloc(size);
}

void free(void *ptr){
	vPortFree(ptr);
}
