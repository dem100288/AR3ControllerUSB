/*
 * arm.h
 *
 *  Created on: Aug 4, 2023
 *      Author: d.zakharov
 */


#include "joint.h"

#define JOINT_COUNT 6
#define CLOCK_STEP 10000.0

#ifndef INC_ARM_H_
#define INC_ARM_H_

void InitArm(void);
void FindHomeJoint(JointNums num);
void FindHomeForce(JointNums num);
void FindHomeArm(void);
void FindHomeArmForce(void);
void TickTimerChennel1(void);
void TickTimerChennel2(void);
void SetTargetPosition(uint8_t, int32_t);
void SetTargetPointEqSpeed(int32_t j1, int32_t j2, int32_t j3, int32_t j4, int32_t j5, int32_t j6);
void SetTargetPointMaxSpeed(int32_t j1, int32_t j2, int32_t j3, int32_t j4, int32_t j5, int32_t j6);
void SetBackToZeroJoint(JointNums, int);
void SetFullStepsJoint(JointNums, int);
void SetFindHomeSpeed(double);
void SetMaxSpeed(double);

void ReportJointPosition(JointNums);
void ReportJointState(JointNums);
void ReportJointsState(void);

#endif /* INC_ARM_H_ */


