/*
 * path_command.h
 *
 *  Created on: Aug 8, 2023
 *      Author: d.zakharov
 */

#define COUNT_POINT_PATH 100
#define BLOCK_COUNT 50

#include "stdbool.h"
#include "stdint.h"

#ifndef INC_PATH_COMMAND_H_
#define INC_PATH_COMMAND_H_

typedef struct __PathPoint{
	double speed;
	uint32_t j1;
	uint32_t j2;
	uint32_t j3;
	uint32_t j4;
	uint32_t j5;
	uint32_t j6;
} PathPoint;

void AddPathPoint(double speed, uint32_t j1, uint32_t j2, uint32_t j3, uint32_t j4, uint32_t j5, uint32_t j6);
void ClearPath(void);
int GetNextPoint(void);
int GetNextPointWithStep(void);
bool PathMoving(void);
void StartMovePath(void);
bool PathStep(void);

#endif /* INC_PATH_COMMAND_H_ */
