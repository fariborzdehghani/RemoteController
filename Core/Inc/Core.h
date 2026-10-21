#ifndef CORE_H
#define CORE_H

#include "main.h"
#include "FlightLogProtocol.h"

#define DATA_PACKET_SIZE          64u
#define PACKET_TYPE_CONFIGURATION 1u
#define PACKET_TYPE_CONTROL       2u
#define PACKET_TYPE_HEARTBEAT     3u
#define CONTROL_COMMAND_DISARM    0u
#define CONTROL_COMMAND_ARM       1u
#define CONTROL_COMMAND_TAKEOFF   2u
#define CONFIG_TAKEOFF_ALTITUDE_LSB_INDEX 13u
#define CONFIG_TAKEOFF_ALTITUDE_MSB_INDEX 14u
#define CONFIG_ARM_THROTTLE_INDEX          1u
#define CONFIG_MAX_THROTTLE_INDEX          3u
#define CONFIG_HOVER_THROTTLE_INDEX        43u
#define TAKEOFF_ALTITUDE_LEGACY_DEFAULT   0u
#define TAKEOFF_ALTITUDE_MIN_CM           20u
#define TAKEOFF_ALTITUDE_MAX_CM           300u
#define HOVER_THROTTLE_AUTO               0u

/* Set to 0 (or override with -DHEARTBEAT_FAILSAFE_ENABLED=0) to disable
 * the operator-silence DISARM transmission. */
#ifndef HEARTBEAT_FAILSAFE_ENABLED
#define HEARTBEAT_FAILSAFE_ENABLED 1u
#endif

_Static_assert(HEARTBEAT_FAILSAFE_ENABLED == 0u ||
               HEARTBEAT_FAILSAFE_ENABLED == 1u,
               "HEARTBEAT_FAILSAFE_ENABLED must be 0 or 1");

extern UART_HandleTypeDef huart1;
extern SPI_HandleTypeDef hspi1;

void General_Init(void);
void General_Loop(void);

#endif
