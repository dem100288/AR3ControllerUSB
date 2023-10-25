/*
 * path_command.c
 *
 *  Created on: Aug 8, 2023
 *      Author: d.zakharov
 */

#include "path_command.h"
#include "stdbool.h"
#include "stdint.h"
#include "cdc_message.h"
#include "arm.h"

PathPoint path[COUNT_POINT_PATH];

uint32_t current_point = 0;
uint32_t all_point = 0;
bool moving_path = false;

void AddPathPoint(double speed, uint32_t j1, uint32_t j2, uint32_t j3, uint32_t j4, uint32_t j5, uint32_t j6){
	uint32_t ins = all_point % COUNT_POINT_PATH;
	path[ins].speed = speed;
	path[ins].j1 = j1;
	path[ins].j2 = j2;
	path[ins].j3 = j3;
	path[ins].j4 = j4;
	path[ins].j5 = j5;
	path[ins].j6 = j6;
	all_point++;
}

void ClearPath(void){
	current_point = 0;
	all_point = 0;
}

int GetNextPoint(void){
	if (current_point < all_point){
		uint32_t p = current_point % COUNT_POINT_PATH;
		return p;
	}
	moving_path = false;
	return -1;
}

int GetNextPointWithStep(void){
	int f = GetNextPoint();
	if (f >= 0){
		if ((current_point % BLOCK_COUNT) == 0 && current_point != 0){
			BlockMoveEndMessage();
		}
		current_point++;
	}
	return f;
}

bool PathMoving(void){
	return moving_path;
}

bool PathStep(void){
	int p = GetNextPointWithStep();
	if (p >= 0){
		SetMaxSpeed(path[p].speed);
		SetTargetPointEqSpeed(path[p].j1, path[p].j2, path[p].j3, path[p].j4, path[p].j5, path[p].j6);
		return true;
	}
	else{
		return false;
	}
}

void StartMovePath(void){
	if (!moving_path && all_point > 0){
		moving_path = true;
		current_point = 0;
		PathStep();
	}
}
