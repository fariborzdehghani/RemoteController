#include "Tools.h"

#include <string.h>

#define LOG_QUEUE_SIZE 512u

static uint8_t log_queue[LOG_QUEUE_SIZE];
static volatile uint16_t log_head;
static volatile uint16_t log_tail;
static volatile uint16_t active_length;
static volatile uint8_t transmit_busy;
static volatile uint32_t dropped_messages;

static uint16_t queue_used(void)
{
    if (log_head >= log_tail)
    {
        return (uint16_t)(log_head - log_tail);
    }

    return (uint16_t)(LOG_QUEUE_SIZE - log_tail + log_head);
}

static void start_next_transmission(void)
{
    uint16_t length;

    if (transmit_busy != 0u || log_head == log_tail)
    {
        return;
    }

    length = (log_head > log_tail)
                 ? (uint16_t)(log_head - log_tail)
                 : (uint16_t)(LOG_QUEUE_SIZE - log_tail);

    if (HAL_UART_Transmit_IT(&huart1, &log_queue[log_tail], length) == HAL_OK)
    {
        active_length = length;
        transmit_busy = 1u;
    }
}

bool LogStringToPC(const char *data)
{
    size_t length;
    uint32_t interrupt_state;
    uint16_t free_space;

    if (data == NULL)
    {
        return false;
    }

    length = strlen(data);
    if (length > (LOG_QUEUE_SIZE - 2u))
    {
        length = LOG_QUEUE_SIZE - 2u;
    }

    interrupt_state = __get_PRIMASK();
    __disable_irq();

    free_space = (uint16_t)(LOG_QUEUE_SIZE - 1u - queue_used());
    if ((length + 1u) > free_space)
    {
        dropped_messages++;
        if (interrupt_state == 0u)
        {
            __enable_irq();
        }
        return false;
    }

    for (size_t index = 0u; index < length; index++)
    {
        log_queue[log_head] = (uint8_t)data[index];
        log_head = (uint16_t)((log_head + 1u) % LOG_QUEUE_SIZE);
    }
    log_queue[log_head] = (uint8_t)'\n';
    log_head = (uint16_t)((log_head + 1u) % LOG_QUEUE_SIZE);

    start_next_transmission();

    if (interrupt_state == 0u)
    {
        __enable_irq();
    }

    return true;
}

void Tools_ProcessLogQueue(void)
{
    uint32_t interrupt_state = __get_PRIMASK();

    __disable_irq();
    start_next_transmission();
    if (interrupt_state == 0u)
    {
        __enable_irq();
    }
}

void Tools_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == NULL || huart->Instance != USART1 || transmit_busy == 0u)
    {
        return;
    }

    log_tail = (uint16_t)((log_tail + active_length) % LOG_QUEUE_SIZE);
    active_length = 0u;
    transmit_busy = 0u;
    start_next_transmission();
}

uint32_t Tools_GetDroppedLogCount(void)
{
    return dropped_messages;
}
