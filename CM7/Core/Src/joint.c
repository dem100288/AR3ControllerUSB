/*
 * stepper.c
 *
 *  Created on: Aug 4, 2023
 *      Author: d.zakharov
 */

#include "math.h"
#include "joint.h"
#include "stm32h7xx_hal.h"
#include "stdlib.h"
#include "cdc_message.h"



void InitJointInvDirStep(JointState *joint,
		uint8_t num,
		InvertPinType invert_direction,
		GPIO_TypeDef *port_dir,
		uint16_t pin_dir,
		GPIO_TypeDef *port_step,
		uint16_t pin_step,
		GPIO_TypeDef *port_limit,
		uint16_t pin_limit,
		InvertPinType inv_dir,
		InvertPinType inv_step)
{
	joint->number = num;
	joint->current_possition = 0;
	joint->target_possition = 0;
	joint->invert_direction = invert_direction;
	joint->back_to_home = 500;
	joint->stepper.inc_step = 0;
	joint->stepper.speed_step = 0.05;
	joint->stepper.direction = 0;
	joint->stepper.port_dir = port_dir;
	joint->stepper.pin_dir = pin_dir;
	joint->stepper.port_step = port_step;
	joint->stepper.pin_step = pin_step;
	joint->stepper.port_limit = port_limit;
	joint->stepper.pin_limit = pin_limit;
	joint->stepper.invert_dir = inv_dir;
	joint->stepper.invert_step = inv_step;
}

void InitJointInvDir(JointState *joint,
		uint8_t num,
		InvertPinType invert_direction,
		GPIO_TypeDef *port_dir,
		uint16_t pin_dir,
		GPIO_TypeDef *port_step,
		uint16_t pin_step,
		GPIO_TypeDef *port_limit,
		uint16_t pin_limit,
		InvertPinType inv_dir)
{
	InitJointInvDirStep(joint, num, invert_direction, port_dir, pin_dir, port_step, pin_step, port_limit, pin_limit, inv_dir, NOT_INVERT_PIN);
}

void InitJoint(JointState *joint,
		uint8_t num,
		InvertPinType invert_direction,
		GPIO_TypeDef *port_dir,
		uint16_t pin_dir,
		GPIO_TypeDef *port_step,
		uint16_t pin_step,
		GPIO_TypeDef *port_limit,
		uint16_t pin_limit)
{
	InitJointInvDirStep(joint, num, invert_direction, port_dir, pin_dir, port_step, pin_step, port_limit, pin_limit, NOT_INVERT_PIN, NOT_INVERT_PIN);
}

void LeadingEdge(JointState *joint){
	if (joint->stepper.direction == 1){
		joint->current_possition++;
	}
	else{
		joint->current_possition--;
	}
	if (joint->stepper.invert_step == NOT_INVERT_PIN)
	{
		HAL_GPIO_WritePin(joint->stepper.port_step, joint->stepper.pin_step, 1);
	}
	else
	{
		HAL_GPIO_WritePin(joint->stepper.port_step, joint->stepper.pin_step, 0);
	}
	joint->stepper.impulse = true;
}

void TrailingEdge(JointState *joint){
	if (joint->stepper.invert_step == NOT_INVERT_PIN)
	{
		HAL_GPIO_WritePin(joint->stepper.port_step, joint->stepper.pin_step, 0);
	}
	else
	{
		HAL_GPIO_WritePin(joint->stepper.port_step, joint->stepper.pin_step, 1);
	}
	joint->stepper.impulse = false;
}

void SetDirectionForward(JointState *joint){
	if (joint->invert_direction == NOT_INVERT_PIN){
		joint->stepper.direction = 1;
	}
	else{
		joint->stepper.direction = 0;
	}
	if (joint->stepper.invert_dir == NOT_INVERT_PIN)
	{
		HAL_GPIO_WritePin(joint->stepper.port_dir, joint->stepper.pin_dir, 1);
	}
	else
	{
		HAL_GPIO_WritePin(joint->stepper.port_dir, joint->stepper.pin_dir, 0);
	}
}

void SetDirectionBackward(JointState *joint){
	if (joint->invert_direction == NOT_INVERT_PIN){
		joint->stepper.direction = 0;
	}
	else{
		joint->stepper.direction = 1;
	}
	if (joint->stepper.invert_dir == NOT_INVERT_PIN)
	{
		HAL_GPIO_WritePin(joint->stepper.port_dir, joint->stepper.pin_dir, 0);
	}
	else
	{
		HAL_GPIO_WritePin(joint->stepper.port_dir, joint->stepper.pin_dir, 1);
	}
}

void ResetPosition(JointState *joint){
	joint->current_possition = 0;
	joint->target_possition = 0;
}

uint32_t DistanceToTarget(JointState *joint){
	return abs(joint->target_possition - joint->current_possition);
}

bool NeedSteps(JointState *joint){
	return DistanceToTarget(joint) > 0;
}

void SetTarget(JointState *joint, int32_t position){
	joint->target_possition = position;
	if ((joint->target_possition - joint->current_possition) >= 0){
		SetDirectionForward(joint);
	}
	else{
		SetDirectionBackward(joint);
	}
	if (NeedSteps(joint)){
		joint->mooving = true;
	}
}

bool CheckLimiter(JointState *joint){
	return HAL_GPIO_ReadPin(joint->stepper.port_limit, joint->stepper.pin_limit) == GPIO_PIN_RESET ? true : false;
}

void InitProcStart(JointState *joint, double_t speed){
	joint->find_limiter = false;
	joint->init = false;
	joint->init_proc = true;
	joint->stepper.speed_step = speed;
	SetTarget(joint, INT32_MIN);
}

void StepFindHome(JointState *joint){
	if (!joint->find_limiter){
		if(CheckLimiter(joint)){
			joint->find_limiter = true;
			ResetPosition(joint);
			SetTarget(joint, joint->back_to_home);
		}
	}
	else{
		if (!NeedSteps(joint)){
			joint->init = true;
			joint->init_proc = false;
			ResetPosition(joint);
			JointFindedHome(joint->number - 1);
		}
	}
}

void TickStepperChanel1(JointState *joint){
	if (!joint->stepper.impulse){
		if (joint->mooving){
			joint->stepper.inc_step += joint->stepper.speed_step;
			if (joint->stepper.inc_step >= 1){
				LeadingEdge(joint);
				joint->stepper.inc_step -= 1;
			}
		}
	}
}

void TickStepperChanel2(JointState *joint){
	if (joint->stepper.impulse){
		TrailingEdge(joint);
		if (!NeedSteps(joint)){
			joint->mooving = false;
		}
	}
	if (joint->init_proc){
		StepFindHome(joint);
	}
}



