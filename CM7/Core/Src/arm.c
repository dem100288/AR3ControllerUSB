/*
 * arm.c
 *
 *  Created on: Aug 4, 2023
 *      Author: d.zakharov
 */

#include "arm.h"
#include "main.h"
#include "math.h"
#include "cdc_message.h"
#include "path_command.h"

JointState ArmJoints[JOINT_COUNT];
double_t find_home_speed = 0.2;

double_t max_speed = 0.5;

void InitArm(void){
	InitJoint(&ArmJoints[0], 1, NOT_INVERT_PIN, J1D_GPIO_Port, J1D_Pin, J1P_GPIO_Port, J1P_Pin, JL1_GPIO_Port, JL1_Pin);
	InitJoint(&ArmJoints[1], 2, NOT_INVERT_PIN, J2D_GPIO_Port, J2D_Pin, J2P_GPIO_Port, J2P_Pin, JL2_GPIO_Port, JL2_Pin);
	InitJointInvDir(&ArmJoints[2], 3, NOT_INVERT_PIN, J3D_GPIO_Port, J3D_Pin, J3P_GPIO_Port, J3P_Pin, JL3_GPIO_Port, JL3_Pin, INVERT_PIN);
	InitJointInvDir(&ArmJoints[3], 4, NOT_INVERT_PIN, J4D_GPIO_Port, J4D_Pin, J4P_GPIO_Port, J4P_Pin, JL4_GPIO_Port, JL4_Pin, INVERT_PIN);
	InitJointInvDir(&ArmJoints[4], 5, NOT_INVERT_PIN, J5D_GPIO_Port, J5D_Pin, J5P_GPIO_Port, J5P_Pin, JL5_GPIO_Port, JL5_Pin, INVERT_PIN);
	InitJoint(&ArmJoints[5], 6, NOT_INVERT_PIN, J6D_GPIO_Port, J6D_Pin, J6P_GPIO_Port, J6P_Pin, JL6_GPIO_Port, JL6_Pin);
}

void FindHomeJoint(JointNums num){
	if (!ArmJoints[num].init){
		FindHomeForce(num);
	}
}

void FindHomeForce(JointNums num){
	InitProcStart(&ArmJoints[num], find_home_speed);
}

void FindHomeArm(void){
	for(int i = 0; i < JOINT_COUNT; i++){
		FindHomeJoint(i);
	}
}

void FindHomeArmForce(void){
	for(int i = 0; i < JOINT_COUNT; i++){
		FindHomeForce(i);
	}
}

void TickTimerChennel1(void){
	for(int i = 0; i < JOINT_COUNT; i++){
		TickStepperChanel1(&ArmJoints[i]);
	}
}

void TickTimerChennel2(void){
	bool prev_mooving = false;
	bool mooving = false;
	bool prev_init = false;
	bool init = false;
	for(int i = 0; i < JOINT_COUNT; i++){
		prev_mooving |= ArmJoints[i].mooving;
		prev_init |= ArmJoints[i].init_proc;
		TickStepperChanel2(&ArmJoints[i]);
		mooving |= ArmJoints[i].mooving;
		init |= ArmJoints[i].init_proc;
	}
	if (prev_mooving && !mooving && !prev_init){
		MoveEndMessage();
		if (PathMoving()){
			PathStep();
		}
	}
	if (prev_init && !init){
		FindedHome();
	}
}

void SetTargetPosition(JointNums num, int32_t position){
	SetTarget(&ArmJoints[num], position);
}

void SetTargetPointEqSpeed(int32_t j1, int32_t j2, int32_t j3, int32_t j4, int32_t j5, int32_t j6){
	uint32_t maxd = 0;

	uint32_t d1 = abs(j1 - ArmJoints[JOINT1].current_possition);
	if (d1 > maxd){
		maxd = d1;
	}
	uint32_t d2 = abs(j2 - ArmJoints[JOINT2].current_possition);
	if (d2 > maxd){
		maxd = d2;
	}
	uint32_t d3 = abs(j3 - ArmJoints[JOINT3].current_possition);
	if (d3 > maxd){
		maxd = d3;
	}
	uint32_t d4 = abs(j4 - ArmJoints[JOINT4].current_possition);
	if (d4 > maxd){
		maxd = d4;
	}
	uint32_t d5 = abs(j5 - ArmJoints[JOINT5].current_possition);
	if (d5 > maxd){
		maxd = d5;
	}
	uint32_t d6 = abs(j6 - ArmJoints[JOINT6].current_possition);
	if (d6 > maxd){
		maxd = d6;
	}

	if (maxd == 0) return;
	ArmJoints[JOINT1].stepper.speed_step = (double)max_speed * ((double)d1 / (double)maxd);
	ArmJoints[JOINT2].stepper.speed_step = (double)max_speed * ((double)d2 / (double)maxd);
	ArmJoints[JOINT3].stepper.speed_step = (double)max_speed * ((double)d3 / (double)maxd);
	ArmJoints[JOINT4].stepper.speed_step = (double)max_speed * ((double)d4 / (double)maxd);
	ArmJoints[JOINT5].stepper.speed_step = (double)max_speed * ((double)d5 / (double)maxd);
	ArmJoints[JOINT6].stepper.speed_step = (double)max_speed * ((double)d6 / (double)maxd);
	SetTarget(&ArmJoints[JOINT1], j1);
	SetTarget(&ArmJoints[JOINT2], j2);
	SetTarget(&ArmJoints[JOINT3], j3);
	SetTarget(&ArmJoints[JOINT4], j4);
	SetTarget(&ArmJoints[JOINT5], j5);
	SetTarget(&ArmJoints[JOINT6], j6);
}

void SetTargetPointMaxSpeed(int32_t j1, int32_t j2, int32_t j3, int32_t j4, int32_t j5, int32_t j6){
	ArmJoints[JOINT1].stepper.speed_step = max_speed;
	ArmJoints[JOINT2].stepper.speed_step = max_speed;
	ArmJoints[JOINT3].stepper.speed_step = max_speed;
	ArmJoints[JOINT4].stepper.speed_step = max_speed;
	ArmJoints[JOINT5].stepper.speed_step = max_speed;
	ArmJoints[JOINT6].stepper.speed_step = max_speed;
	SetTarget(&ArmJoints[JOINT1], j1);
	SetTarget(&ArmJoints[JOINT2], j2);
	SetTarget(&ArmJoints[JOINT3], j3);
	SetTarget(&ArmJoints[JOINT4], j4);
	SetTarget(&ArmJoints[JOINT5], j5);
	SetTarget(&ArmJoints[JOINT6], j6);
}

void SetBackToZeroJoint(JointNums num, int btz){
	ArmJoints[num].back_to_home = btz;
}

void SetFullStepsJoint(JointNums num, int fs){
	ArmJoints[num].full_steps = fs;
}

void SetFindHomeSpeed(double fhs){
	find_home_speed = fhs;
}

void SetMaxSpeed(double ms){
	max_speed = ms;
}

void ReportJointPosition(JointNums num){
	JointPosition(num, ArmJoints[num].current_possition);
}

void ReportJointState(JointNums num){
	JointStateMessage(num, ArmJoints[num].init, ArmJoints[num].back_to_home, ArmJoints[num].current_possition, ArmJoints[num].stepper.speed_step * CLOCK_STEP);
}

void ReportJointsState(void){
	for(int i = 0; i < JOINT_COUNT; i++){
		JointStateMessage(i, ArmJoints[i].init, ArmJoints[i].back_to_home, ArmJoints[i].current_possition, ArmJoints[i].stepper.speed_step * CLOCK_STEP);
	}

}
