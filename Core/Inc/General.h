#ifndef GENERAL_H
#define GENERAL_H

#include "main.h"
#include <stdbool.h>

extern UART_HandleTypeDef huart1;

// Variable declarations
extern bool PacketReceived;
extern uint8_t rx_data[32];
extern uint8_t USART_ReceivedByte;
extern uint8_t USART_Buffer[32];
extern uint16_t USART_dataIndex;
extern bool PacketReadyToSend;

// Function declarations
void HandlePackage(char *dataStr);
void General_Init(void);
void General_Loop(void);

#endif