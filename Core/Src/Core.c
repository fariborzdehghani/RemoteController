#include "Core.h"
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
#if HEARTBEAT_FAILSAFE_ENABLED
static bool operator_armed;
static uint32_t last_operator_packet_time;
#endif
static uint32_t last_radio_attempt_time;
static uint32_t led_timer;

static bool packet_is_valid(const uint8_t *packet)
{
    if (packet == NULL)
    {
        return false;
    }

    if (packet[0] == PACKET_TYPE_CONFIGURATION ||
        packet[0] == PACKET_TYPE_HEARTBEAT)
    {
        return true;
    }

    return packet[0] == PACKET_TYPE_CONTROL &&
           (packet[1] == CONTROL_COMMAND_DISARM ||
            packet[1] == CONTROL_COMMAND_ARM ||
            packet[1] == CONTROL_COMMAND_TAKEOFF);
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
    SX127X_SetSignalBandwidth(125000u);
    SX127X_SetCodingRate4(5u);
    SX127X_SetPreambleLength(8u);
    SX127X_ClearIrqFlags();
    SX127X_SetModeStandby();
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

    return true;
}

static void process_packet(const uint8_t *packet)
{
#if HEARTBEAT_FAILSAFE_ENABLED
    bool sent;
#endif

    if (!packet_is_valid(packet))
    {
        (void)LogStringToPC("Rejected invalid control packet");
        return;
    }

#if HEARTBEAT_FAILSAFE_ENABLED
    last_operator_packet_time = HAL_GetTick();

    if (packet[0] == PACKET_TYPE_CONTROL &&
        (packet[1] == CONTROL_COMMAND_ARM ||
         packet[1] == CONTROL_COMMAND_TAKEOFF))
    {
        operator_armed = true;
    }

    sent = transmit_packet(packet);
    if (sent && packet[0] == PACKET_TYPE_CONTROL &&
        packet[1] == CONTROL_COMMAND_DISARM)
    {
        operator_armed = false;
    }
#else
    (void)transmit_packet(packet);
#endif
}

#if HEARTBEAT_FAILSAFE_ENABLED
static void process_operator_timeout(void)
{
    static const uint8_t disarm_packet[DATA_PACKET_SIZE] = {
        PACKET_TYPE_CONTROL, CONTROL_COMMAND_DISARM};

    if (!operator_armed ||
        (HAL_GetTick() - last_operator_packet_time) < OPERATOR_TIMEOUT_MS)
    {
        return;
    }

    if (transmit_packet(disarm_packet))
    {
        operator_armed = false;
        (void)LogStringToPC("Operator link timed out; DISARM sent");
    }
}
#endif

void General_Init(void)
{
#if HEARTBEAT_FAILSAFE_ENABLED
    last_operator_packet_time = HAL_GetTick();
#endif
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

#if !HEARTBEAT_FAILSAFE_ENABLED
    (void)LogStringToPC("WARNING: heartbeat failsafe is disabled");
#endif

    start_watchdog();
}

void General_Loop(void)
{
    const uint32_t now = HAL_GetTick();

    if (!radio_ready &&
        (now - last_radio_attempt_time) >= RADIO_RETRY_INTERVAL_MS)
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

#if HEARTBEAT_FAILSAFE_ENABLED
    process_operator_timeout();
#endif
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
