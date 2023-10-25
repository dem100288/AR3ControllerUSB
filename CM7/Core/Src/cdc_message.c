/*
 * cdc_message.c
 *
 *  Created on: Aug 7, 2023
 *      Author: d.zakharov
 */

#include "stdint.h"
#include "string.h"
#include "usbd_cdc_if.h"
#include "stdlib.h"
#include "joint.h"
#include "stdbool.h"
#include "math.h"

void SendMessage(char *msg){
	//uint8_t res = USBD_BUSY;
	//while (res == USBD_BUSY){
	CDC_Transmit((uint8_t *)msg, strlen((char *)msg));
	//}
}

void FindedHome(void){
	char *msg = "fh_end\n";
	SendMessage(msg);
}

void JointFindedHome(JointNums joint){
	char msg[10];
	memset(&msg, '\0', 10);
	sprintf(&msg, "fh%i_end\n", joint + 1);
	SendMessage(&msg);
}

void JointPosition(JointNums joint, int32_t pos){
	char msg[15];
	memset(&msg, '\0', 15);
	sprintf(&msg, "j%i %i\n", joint + 1, pos);
	SendMessage(&msg);
}

void JointStateMessage(JointNums joint, bool _init, uint32_t stb, int32_t pos, double_t speed){
	char msg[100];
	memset(&msg, '\0', 100);
	char i = _init ? '1' : '0';
	sprintf(&msg, "st%i %c %i 0 %i %f\n", joint + 1, i, stb, pos, speed);
	SendMessage(&msg);
}

void MoveEndMessage(void){
	char *msg = "m_end\n";
	SendMessage(msg);
}

void BlockMoveEndMessage(void){
	char *msg = "b_end\n";
	SendMessage(msg);
}

void PongMessage(void){
	char *msg = "PP\n";
	SendMessage(msg);
}

void SpeedJointMessage(JointNums joint, double_t speed){
	char msg[30];
	memset(&msg, '\0', 30);
	sprintf(&msg, "speed%i %f\n", joint + 1, speed);
	SendMessage(&msg);
}
