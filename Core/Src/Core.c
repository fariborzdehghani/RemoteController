#include "Core.h"
#include "nrf24l01p.h"
#include "Tools.h"
#include <string.h>

static uint16_t LED_Timer = 0;

bool PacketReceived = false;
bool PacketReadyToSend = false;

uint8_t rx_data[32];
uint8_t USART_ReceivedByte;
uint8_t USART_Buffer[32];
uint16_t USART_dataIndex = 0;

void General_Init(void) {
    HAL_UART_Receive_IT(&huart1, &USART_ReceivedByte, 1);

    nrf24l01p_init(2500, _250kbps);
    nrf24l01p_ptx_mode();

    uint8_t tx_address[5] = {0x11, 0x22, 0x33, 0x44, 0x55};

    nRF24L01_Set_Tx_Addr(tx_address);
    nRF24L01_Set_Rx_Addr(tx_address, NRF24L01P_REG_RX_ADDR_P0);
}

void General_Loop(void) {
    uint8_t status = 0;

    // Handle transmission
    if (PacketReadyToSend) {
        LogStringToPC("trying to send the Command...");
        nrf24l01p_tx_transmit(USART_Buffer);
        PacketReadyToSend = false;
        
        uint32_t startTime = HAL_GetTick();
        const uint32_t TIMEOUT_MS = 10; // 10ms timeout
        
        // Wait for transmission complete or timeout
        do {
            status = nrf24l01p_get_status();
            if (HAL_GetTick() - startTime > TIMEOUT_MS) {
                LogStringToPC("Message NOT sent: timeout!");
                break;
            }
        } while(!(status & (NRF24L01P_TX_DS | NRF24L01P_MAX_RT)));
        
        if (status & NRF24L01P_TX_DS) {
            LogStringToPC("Message sent successfully!");
        }
        if (status & NRF24L01P_MAX_RT) {
            LogStringToPC("Message NOT sent: max retries reached!");
        }
        
        nrf24l01p_tx_irq();
        nrf24l01p_flush_tx_fifo();
        memset(USART_Buffer, '\0', sizeof(USART_Buffer));
        USART_dataIndex = 0;
    }

    if (HAL_GetTick() - LED_Timer > 1000) {
        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
        LED_Timer = HAL_GetTick();
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART1) {
        if (USART_ReceivedByte == 255) {
            PacketReadyToSend = true;
        } else {
            USART_Buffer[USART_dataIndex++] = USART_ReceivedByte;
        }
        HAL_UART_Receive_IT(&huart1, &USART_ReceivedByte, 1);
    }
}

