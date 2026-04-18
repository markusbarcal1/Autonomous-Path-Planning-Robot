#ifndef MOTOR_H
#define MOTOR_H

#include <stdint.h>

void Motor_Init(void);

void Motor_LeftForward(void);
void Motor_LeftReverse(void);
void Motor_RightForward(void);
void Motor_RightReverse(void);

void Motor_StopAll(void);

void Motor_SetLeftDutyPercent(uint8_t percent);
void Motor_SetRightDutyPercent(uint8_t percent);

void Motor_SetBothDutyPercent(uint8_t leftPercent, uint8_t rightPercent);

#endif
