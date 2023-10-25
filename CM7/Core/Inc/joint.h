/*
 * stepper.h
 *
 *  Created on: Aug 4, 2023
 *      Author: d.zakharov
 */

#include "math.h"
#include "stdint.h"
#include "stm32h745xx.h"
#include "stdbool.h"

#ifndef INC_STEPPER_H_
#define INC_STEPPER_H_

typedef enum __InvertPinType{
	NOT_INVERT_PIN = 0U,
	INVERT_PIN = 1U
} InvertPinType;

typedef enum __JointNums{
	JOINT1 = 0U,
	JOINT2 = 1U,
	JOINT3 = 2U,
	JOINT4 = 3U,
	JOINT5 = 4U,
	JOINT6 = 5U
} JointNums;

typedef struct __StepperT{
	double_t inc_step;
	double_t speed_step;
	uint8_t direction;
	GPIO_TypeDef *port_dir;
	uint16_t pin_dir;
	GPIO_TypeDef *port_step;
	uint16_t pin_step;
	GPIO_TypeDef *port_limit;
	uint16_t pin_limit;
	InvertPinType invert_dir;
	InvertPinType invert_step;
	bool impulse;
} StepperT;

typedef struct __JointState{
	uint8_t number;
	int32_t current_possition;
	int32_t target_possition;
	int32_t back_to_home;
	int32_t full_steps;
	InvertPinType invert_direction;
	bool mooving;
	bool init;
	bool init_proc;
	bool find_limiter;
	StepperT stepper;
} JointState;

void InitJoint(JointState *joint,
		uint8_t num,
		InvertPinType invert_direction,
		GPIO_TypeDef *port_dir,
		uint16_t pin_dir,
		GPIO_TypeDef *port_step,
		uint16_t pin_step,
		GPIO_TypeDef *port_limit,
		uint16_t pin_limit);

void InitJointInvDir(JointState *joint,
		uint8_t num,
		InvertPinType invert_direction,
		GPIO_TypeDef *port_dir,
		uint16_t pin_dir,
		GPIO_TypeDef *port_step,
		uint16_t pin_step,
		GPIO_TypeDef *port_limit,
		uint16_t pin_limit,
		InvertPinType inv_dir);

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
		InvertPinType inv_step);

void InitProcStart(JointState *, double_t);
void TickStepperChanel1(JointState *);
void TickStepperChanel2(JointState *);
void SetTarget(JointState *, int32_t);
uint32_t DistanceToTarget(JointState *joint);

#endif /* INC_STEPPER_H_ */

