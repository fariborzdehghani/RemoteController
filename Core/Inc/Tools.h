/*
 * Tools.h
 *
 *  Created on: Apr 7, 2023
 *      Author: fariborz
 */

#ifndef INC_TOOLS_H_
#define INC_TOOLS_H_

#include "main.h"
#include <stdbool.h>

extern UART_HandleTypeDef huart1;

bool LogStringToPC(const char *data);
void Tools_ProcessLogQueue(void);
void Tools_UART_TxCpltCallback(UART_HandleTypeDef *huart);
uint32_t Tools_GetDroppedLogCount(void);

#endif /* INC_TOOLS_H_ */



