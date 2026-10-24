#include "Core.h"
#include "RadioConfig.h"
#include "CommandProtocol.h"
#include "Tools.h"
#include "sx127x.h"

#include <stdbool.h>
#include <string.h>

#define PACKET_QUEUE_DEPTH       2u
#define OPERATOR_TIMEOUT_MS      750u
#define RADIO_RETRY_INTERVAL_MS 1000u
#define LED_INTERVAL_MS         1000u
#define WATCHDOG_ACCESS_KEY     0x5555u
#define WATCHDOG_START_KEY      0xccccu
#define WATCHDOG_REFRESH_KEY    0xaaaau
#define WATCHDOG_PRESCALER_64   4u
#define WATCHDOG_RELOAD_VALUE   1249u

static uint8_t packet_queue[PACKET_QUEUE_DEPTH][DATA_PACKET_SIZE];
static uint8_t transmit_buffer[DATA_PACKET_SIZE];
static uint8_t received_byte;
static volatile uint8_t queue_head;
static volatile uint8_t queue_tail;
static volatile uint8_t queue_count;
static volatile uint8_t receive_index;
static volatile uint8_t uart_receive_paused;

static bool watchdog_started;
static bool radio_ready;
static bool operator_armed;
static uint32_t last_operator_packet_time;
static uint32_t last_radio_attempt_time;
static uint32_t led_timer;
static char pending_log_line[139];
static bool log_line_pending;

static bool packet_is_valid(const uint8_t *packet)
{
    if (!CommandPacket_IsValidShape(packet, DATA_PACKET_SIZE))
    {
        return false;
    }

    if (packet[0] == PACKET_TYPE_CONFIGURATION)
    {
        const uint16_t takeoff_altitude_cm =
            (uint16_t)((uint16_t)packet[CONFIG_TAKEOFF_ALTITUDE_LSB_INDEX] |
                       ((uint16_t)packet[CONFIG_TAKEOFF_ALTITUDE_MSB_INDEX] << 8));
        const uint8_t hover_throttle = packet[CONFIG_HOVER_THROTTLE_INDEX];
        const bool takeoff_altitude_valid =
            takeoff_altitude_cm == TAKEOFF_ALTITUDE_LEGACY_DEFAULT ||
            (takeoff_altitude_cm >= TAKEOFF_ALTITUDE_MIN_CM &&
             takeoff_altitude_cm <= TAKEOFF_ALTITUDE_MAX_CM);
        const bool hover_throttle_valid =
            hover_throttle == HOVER_THROTTLE_AUTO ||
            (hover_throttle >= packet[CONFIG_ARM_THROTTLE_INDEX] &&
             hover_throttle <= packet[CONFIG_MAX_THROTTLE_INDEX]);

        /*
         * Zero is accepted for compatibility with older desktop builds; the
         * flight controller interprets it as the legacy 50 cm target.
         */
        return takeoff_altitude_valid && hover_throttle_valid;
    }

    if (packet[0] == PACKET_TYPE_LOG_REQUEST)
    {
        /* Diagnostics are available before takeoff too, but never while the
         * operator has armed the aircraft. The aircraft repeats this check. */
        return !operator_armed && FlightLog_IsRequestValid(packet);
    }

    if (packet[0] == PACKET_TYPE_HEARTBEAT)
    {
        return true;
    }

    return packet[0] == PACKET_TYPE_CONTROL &&
           (packet[1] == CONTROL_COMMAND_DISARM ||
            packet[1] == CONTROL_COMMAND_ARM ||
            packet[1] == CONTROL_COMMAND_TAKEOFF) &&
           (packet[1] != CONTROL_COMMAND_DISARM ||
            packet[2] <= CONTROL_DISARM_REASON_LINK_TIMEOUT);
}

static bool configure_radio(void)
{
    if (!SX127X_Init(&hspi1))
    {
        return false;
    }

    SX127X_SetFrequency(915.0f);
    SX127X_SetTxPower(17, false);
    SX127X_SetSpreadingFactor(7u);
    SX127X_SetSignalBandwidth(RADIO_BANDWIDTH_HZ);
    SX127X_SetCodingRate4(5u);
    SX127X_SetPreambleLength(8u);
    SX127X_ClearIrqFlags();
    SX127X_SetModeRx();
    return SX127X_IsOperational();
}

static void start_watchdog(void)
{
    const uint32_t started_at = HAL_GetTick();

    IWDG->KR = WATCHDOG_START_KEY;
    IWDG->KR = WATCHDOG_ACCESS_KEY;
    IWDG->PR = WATCHDOG_PRESCALER_64;
    IWDG->RLR = WATCHDOG_RELOAD_VALUE;

    while (IWDG->SR != 0u && (HAL_GetTick() - started_at) < 50u)
    {
    }

    IWDG->KR = WATCHDOG_REFRESH_KEY;
    watchdog_started = true;
}

static void resume_uart_receive(void)
{
    if (HAL_UART_Receive_IT(&huart1, &received_byte, 1u) == HAL_OK)
    {
        uart_receive_paused = 0u;
    }
    else
    {
        uart_receive_paused = 1u;
    }
}

static bool dequeue_packet(uint8_t *destination)
{
    bool should_resume = false;
    uint32_t interrupt_state;

    interrupt_state = __get_PRIMASK();
    __disable_irq();

    if (queue_count == 0u)
    {
        if (interrupt_state == 0u)
        {
            __enable_irq();
        }
        return false;
    }

    memcpy(destination, packet_queue[queue_tail], DATA_PACKET_SIZE);
    queue_tail = (uint8_t)((queue_tail + 1u) % PACKET_QUEUE_DEPTH);
    queue_count--;

    if (uart_receive_paused != 0u)
    {
        should_resume = true;
    }

    if (interrupt_state == 0u)
    {
        __enable_irq();
    }

    if (should_resume)
    {
        resume_uart_receive();
    }

    return true;
}

static bool transmit_packet(const uint8_t *packet)
{
    if (!radio_ready || !SX127X_Send(packet, DATA_PACKET_SIZE))
    {
        radio_ready = false;
        last_radio_attempt_time = HAL_GetTick();
        (void)LogStringToPC("LoRa transmission failed");
        return false;
    }

    /* Listen immediately after TX; the aircraft's response follows shortly. */
    SX127X_SetModeRx();
    return true;
}

static void process_radio_response(void)
{
    if (!radio_ready) return;
    if (log_line_pending)
    {
        if (LogStringToPC(pending_log_line)) log_line_pending = false;
        return;
    }
    uint8_t packet[DATA_PACKET_SIZE], length;
    if (!SX127X_Receive(packet, sizeof(packet), &length) ||
        length != DATA_PACKET_SIZE || !FlightLog_IsResponseValid(packet)) return;
    static const char hex[] = "0123456789ABCDEF";
    memcpy(pending_log_line, "FlightLog=", 10u);
    for (unsigned i = 0; i < DATA_PACKET_SIZE; i++)
    {
        pending_log_line[10u + i * 2u] = hex[packet[i] >> 4];
        pending_log_line[11u + i * 2u] = hex[packet[i] & 15u];
    }
    pending_log_line[138] = '\0';
    log_line_pending = !LogStringToPC(pending_log_line);
}

static void process_packet(const uint8_t *packet)
{
    if (!packet_is_valid(packet))
    {
        (void)LogStringToPC("Rejected invalid packet");
        return;
    }

    if (HEARTBEAT_FAILSAFE_ENABLED)
        last_operator_packet_time = HAL_GetTick();

    /* Track the download interlock even in builds with heartbeat disabled. */
    if (packet[0] == PACKET_TYPE_CONTROL &&
        (packet[1] == CONTROL_COMMAND_ARM || packet[1] == CONTROL_COMMAND_TAKEOFF))
        operator_armed = true;

    const bool sent = transmit_packet(packet);
    if (sent && packet[0] == PACKET_TYPE_CONTROL && packet[1] == CONTROL_COMMAND_DISARM)
        operator_armed = false;
}

static void process_operator_timeout(void)
{
    static const uint8_t disarm_packet[DATA_PACKET_SIZE] = {
        PACKET_TYPE_CONTROL, CONTROL_COMMAND_DISARM, CONTROL_DISARM_REASON_LINK_TIMEOUT};

    if (!operator_armed || (HAL_GetTick() - last_operator_packet_time) < OPERATOR_TIMEOUT_MS)
    {
        return;
    }

    if (transmit_packet(disarm_packet))
    {
        operator_armed = false;
        (void)LogStringToPC("Operator link timed out; DISARM sent");
    }
}

void General_Init(void)
{
    if (HEARTBEAT_FAILSAFE_ENABLED)
    {
        last_operator_packet_time = HAL_GetTick();
    }

    last_radio_attempt_time = HAL_GetTick();
    resume_uart_receive();

    radio_ready = configure_radio();
    if (radio_ready)
    {
        (void)LogStringToPC("SX127X initialized successfully");
    }
    else
    {
        (void)LogStringToPC("SX127X initialization failed");
    }

    if (!HEARTBEAT_FAILSAFE_ENABLED)
    {
        (void)LogStringToPC("WARNING: heartbeat failsafe is disabled");
    }

    start_watchdog();
}

void General_Loop(void)
{
    const uint32_t now = HAL_GetTick();

    if (!radio_ready && (now - last_radio_attempt_time) >= RADIO_RETRY_INTERVAL_MS)
    {
        last_radio_attempt_time = now;
        radio_ready = configure_radio();
        if (radio_ready)
        {
            (void)LogStringToPC("SX127X recovered");
        }
    }

    if (dequeue_packet(transmit_buffer))
    {
        process_packet(transmit_buffer);
    }

    if (HEARTBEAT_FAILSAFE_ENABLED)
    {
        process_operator_timeout();
    }

    process_radio_response();
    Tools_ProcessLogQueue();

    if ((now - led_timer) >= LED_INTERVAL_MS)
    {
        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
        led_timer = now;
    }

    if (watchdog_started)
    {
        IWDG->KR = WATCHDOG_REFRESH_KEY;
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == NULL || huart->Instance != USART1)
    {
        return;
    }

    packet_queue[queue_head][receive_index] = received_byte;
    receive_index++;

    if (receive_index >= DATA_PACKET_SIZE)
    {
        receive_index = 0u;
        __DMB();
        queue_head = (uint8_t)((queue_head + 1u) % PACKET_QUEUE_DEPTH);
        queue_count++;

        if (queue_count >= PACKET_QUEUE_DEPTH)
        {
            uart_receive_paused = 1u;
            return;
        }
    }

    resume_uart_receive();
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    Tools_UART_TxCpltCallback(huart);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart != NULL && huart->Instance == USART1 &&
        queue_count < PACKET_QUEUE_DEPTH)
    {
        resume_uart_receive();
    }
}
