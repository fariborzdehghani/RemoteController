/*
 * Tools.h
 *
 *  Created on: Apr 7, 2023
 *      Author: fariborz
 */

#ifndef INC_TOOLS_H_
#define INC_TOOLS_H_

#include "main.h"

extern UART_HandleTypeDef huart1;

void LogIntToPC(uint8_t Data);
void LogStringToPC(char *Data);
char *int_to_binary_string(int num);

#endif /* INC_TOOLS_H_ */



