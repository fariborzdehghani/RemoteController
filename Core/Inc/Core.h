#ifndef CORE_H
#define CORE_H

#include "main.h"

#define DATA_PACKET_SIZE          64u
#define PACKET_TYPE_CONFIGURATION 1u
#define PACKET_TYPE_CONTROL       2u
#define PACKET_TYPE_HEARTBEAT     3u
#define CONTROL_COMMAND_DISARM    0u
#define CONTROL_COMMAND_ARM       1u
#define CONTROL_COMMAND_TAKEOFF   2u

/* Set to 0 (or override with -DHEARTBEAT_FAILSAFE_ENABLED=0) to disable
 * the operator-silence DISARM transmission. */
#ifndef HEARTBEAT_FAILSAFE_ENABLED
#define HEARTBEAT_FAILSAFE_ENABLED 1u
#endif

#if (HEARTBEAT_FAILSAFE_ENABLED != 0u) && \
    (HEARTBEAT_FAILSAFE_ENABLED != 1u)
#error "HEARTBEAT_FAILSAFE_ENABLED must be 0 or 1"
#endif

extern UART_HandleTypeDef huart1;
extern SPI_HandleTypeDef hspi1;

void General_Init(void);
void General_Loop(void);

#endif
