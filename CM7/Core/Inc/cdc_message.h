/*
 * cdc_message.h
 *
 *  Created on: Aug 7, 2023
 *      Author: d.zakharov
 */

#include "stdbool.h"
#include "stdint.h"
#include "joint.h"

#ifndef INC_CDC_MESSAGE_H_
#define INC_CDC_MESSAGE_H_

void FindedHome(void);
void JointFindedHome(uint8_t);
void JointPosition(JointNums, int32_t);
void JointStateMessage(JointNums joint, bool _init, uint32_t stb, int32_t pos, double_t speed);
void MoveEndMessage(void);
void BlockMoveEndMessage(void);
void PongMessage(void);
void SpeedJointMessage(JointNums joint, double_t speed);
void SendMessage(char *);

#endif /* INC_CDC_MESSAGE_H_ */


