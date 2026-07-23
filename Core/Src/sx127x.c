#include "sx127x.h"

#define SX127X_SPI_TIMEOUT_MS 100u
#define SX127X_TX_TIMEOUT_MS  750u
#define SX127X_LONG_RANGE_MODE 0x80u

static SPI_HandleTypeDef *spi;
static bool radio_io_ok;

uint8_t SX127X_ReadRegister(uint8_t reg)
{
    uint8_t address = reg & 0x7fu;
    uint8_t value = 0u;

    if (spi == NULL)
    {
        radio_io_ok = false;
        return 0u;
    }

    HAL_GPIO_WritePin(SX127X_NSS_PORT, SX127X_NSS_PIN, GPIO_PIN_RESET);
    if (HAL_SPI_Transmit(spi, &address, 1u, SX127X_SPI_TIMEOUT_MS) != HAL_OK ||
        HAL_SPI_Receive(spi, &value, 1u, SX127X_SPI_TIMEOUT_MS) != HAL_OK)
    {
        radio_io_ok = false;
    }
    HAL_GPIO_WritePin(SX127X_NSS_PORT, SX127X_NSS_PIN, GPIO_PIN_SET);

    return value;
}

void SX127X_WriteRegister(uint8_t reg, uint8_t data)
{
    uint8_t command[2] = {(uint8_t)(reg | 0x80u), data};

    if (spi == NULL)
    {
        radio_io_ok = false;
        return;
    }

    HAL_GPIO_WritePin(SX127X_NSS_PORT, SX127X_NSS_PIN, GPIO_PIN_RESET);
    if (HAL_SPI_Transmit(spi, command, sizeof(command),
                         SX127X_SPI_TIMEOUT_MS) != HAL_OK)
    {
        radio_io_ok = false;
    }
    HAL_GPIO_WritePin(SX127X_NSS_PORT, SX127X_NSS_PIN, GPIO_PIN_SET);
}

static void reset_radio(void)
{
    HAL_GPIO_WritePin(SX127X_RESET_PORT, SX127X_RESET_PIN, GPIO_PIN_RESET);
    HAL_Delay(10u);
    HAL_GPIO_WritePin(SX127X_RESET_PORT, SX127X_RESET_PIN, GPIO_PIN_SET);
    HAL_Delay(10u);
}

bool SX127X_Init(SPI_HandleTypeDef *spi_port)
{
    if (spi_port == NULL)
    {
        return false;
    }

    spi = spi_port;
    radio_io_ok = true;
    reset_radio();

    if (SX127X_ReadRegister(REG_VERSION) != SX127X_VERSION || !radio_io_ok)
    {
        return false;
    }

    SX127X_WriteRegister(REG_OP_MODE,
                         (uint8_t)(SX127X_LONG_RANGE_MODE | MODE_SLEEP));
    HAL_Delay(1u);
    SX127X_SetModeStandby();
    SX127X_WriteRegister(REG_FIFO_TX_BASE_ADDR, 0u);
    SX127X_WriteRegister(REG_FIFO_RX_BASE_ADDR, 0u);
    SX127X_WriteRegister(REG_LNA, 0x23u);
    SX127X_WriteRegister(REG_MODEM_CONFIG_3, 0x04u);
    SX127X_WriteRegister(REG_PA_RAMP, 0x08u);
    SX127X_ClearIrqFlags();

    return radio_io_ok;
}

bool SX127X_IsOperational(void)
{
    return spi != NULL && radio_io_ok;
}

void SX127X_SetFrequency(float freq)
{
    uint32_t frf = (uint32_t)((freq * 1000000.0f) / 61.03515625f);

    SX127X_WriteRegister(REG_FRF_MSB, (uint8_t)(frf >> 16));
    SX127X_WriteRegister(REG_FRF_MID, (uint8_t)(frf >> 8));
    SX127X_WriteRegister(REG_FRF_LSB, (uint8_t)frf);
}

void SX127X_SetTxPower(int8_t power, bool useRFO)
{
    if (useRFO)
    {
        if (power > 14)
        {
            power = 14;
        }
        if (power < -1)
        {
            power = -1;
        }
        SX127X_WriteRegister(REG_PA_CONFIG,
                             (uint8_t)(0x70u | (uint8_t)(power + 1)));
        return;
    }

    if (power > 20)
    {
        power = 20;
    }
    if (power < 2)
    {
        power = 2;
    }
    SX127X_WriteRegister(REG_PA_CONFIG,
                         (uint8_t)(PA_BOOST | (uint8_t)(power - 2)));
}

void SX127X_SetSpreadingFactor(uint8_t sf)
{
    if (sf < 6u)
    {
        sf = 6u;
    }
    else if (sf > 12u)
    {
        sf = 12u;
    }

    if (sf == 6u)
    {
        SX127X_WriteRegister(REG_DETECTION_OPTIMIZE, 0xc5u);
        SX127X_WriteRegister(REG_DETECTION_THRESHOLD, 0x0cu);
    }
    else
    {
        SX127X_WriteRegister(REG_DETECTION_OPTIMIZE, 0xc3u);
        SX127X_WriteRegister(REG_DETECTION_THRESHOLD, 0x0au);
    }

    SX127X_WriteRegister(
        REG_MODEM_CONFIG_2,
        (uint8_t)((SX127X_ReadRegister(REG_MODEM_CONFIG_2) & 0x0fu) |
                  (uint8_t)(sf << 4)));
}

void SX127X_SetSignalBandwidth(uint32_t bandwidth)
{
    uint8_t setting;

    if (bandwidth <= 7800u)
    {
        setting = 0u;
    }
    else if (bandwidth <= 10400u)
    {
        setting = 1u;
    }
    else if (bandwidth <= 15600u)
    {
        setting = 2u;
    }
    else if (bandwidth <= 20800u)
    {
        setting = 3u;
    }
    else if (bandwidth <= 31250u)
    {
        setting = 4u;
    }
    else if (bandwidth <= 41700u)
    {
        setting = 5u;
    }
    else if (bandwidth <= 62500u)
    {
        setting = 6u;
    }
    else if (bandwidth <= 125000u)
    {
        setting = 7u;
    }
    else if (bandwidth <= 250000u)
    {
        setting = 8u;
    }
    else
    {
        setting = 9u;
    }

    SX127X_WriteRegister(
        REG_MODEM_CONFIG_1,
        (uint8_t)((SX127X_ReadRegister(REG_MODEM_CONFIG_1) & 0x0fu) |
                  (uint8_t)(setting << 4)));
}

void SX127X_SetCodingRate4(uint8_t denominator)
{
    uint8_t setting;

    if (denominator < 5u)
    {
        denominator = 5u;
    }
    else if (denominator > 8u)
    {
        denominator = 8u;
    }

    setting = (uint8_t)(denominator - 4u);
    SX127X_WriteRegister(
        REG_MODEM_CONFIG_1,
        (uint8_t)((SX127X_ReadRegister(REG_MODEM_CONFIG_1) & 0xf1u) |
                  (uint8_t)(setting << 1)));
}

void SX127X_SetPreambleLength(uint16_t length)
{
    SX127X_WriteRegister(REG_PREAMBLE_MSB, (uint8_t)(length >> 8));
    SX127X_WriteRegister(REG_PREAMBLE_LSB, (uint8_t)length);
}

bool SX127X_Send(const uint8_t *buf, uint8_t len)
{
    uint8_t fifo_command = (uint8_t)(REG_FIFO | 0x80u);
    uint32_t started_at;

    if (spi == NULL || buf == NULL || len == 0u)
    {
        return false;
    }

    radio_io_ok = true;
    SX127X_SetModeStandby();
    SX127X_ClearIrqFlags();
    SX127X_WriteRegister(REG_FIFO_ADDR_PTR, 0u);

    HAL_GPIO_WritePin(SX127X_NSS_PORT, SX127X_NSS_PIN, GPIO_PIN_RESET);
    if (HAL_SPI_Transmit(spi, &fifo_command, 1u,
                         SX127X_SPI_TIMEOUT_MS) != HAL_OK ||
        HAL_SPI_Transmit(spi, (uint8_t *)buf, len,
                         SX127X_SPI_TIMEOUT_MS) != HAL_OK)
    {
        radio_io_ok = false;
    }
    HAL_GPIO_WritePin(SX127X_NSS_PORT, SX127X_NSS_PIN, GPIO_PIN_SET);

    if (!radio_io_ok)
    {
        SX127X_SetModeStandby();
        return false;
    }

    SX127X_WriteRegister(REG_PAYLOAD_LENGTH, len);
    SX127X_SetModeTx();
    started_at = HAL_GetTick();

    while ((SX127X_ReadRegister(REG_IRQ_FLAGS) & IRQ_TX_DONE_MASK) == 0u)
    {
        if (!radio_io_ok ||
            (HAL_GetTick() - started_at) >= SX127X_TX_TIMEOUT_MS)
        {
            SX127X_SetModeStandby();
            return false;
        }
        HAL_Delay(1u);
    }

    SX127X_WriteRegister(REG_IRQ_FLAGS, IRQ_TX_DONE_MASK);
    SX127X_SetModeStandby();
    return radio_io_ok;
}

bool SX127X_Receive(uint8_t *buf, uint8_t capacity, uint8_t *len)
{
    uint8_t flags;
    uint8_t packet_length;
    uint8_t fifo_command = REG_FIFO & 0x7fu;

    if (spi == NULL || buf == NULL || len == NULL || capacity == 0u)
    {
        return false;
    }

    *len = 0u;
    radio_io_ok = true;
    SX127X_SetModeRx();
    flags = SX127X_ReadRegister(REG_IRQ_FLAGS);

    if (!radio_io_ok || (flags & IRQ_RX_DONE_MASK) == 0u)
    {
        return false;
    }

    if ((flags & IRQ_PAYLOAD_CRC_ERROR_MASK) != 0u)
    {
        SX127X_WriteRegister(
            REG_IRQ_FLAGS,
            (uint8_t)(IRQ_RX_DONE_MASK | IRQ_PAYLOAD_CRC_ERROR_MASK));
        return false;
    }

    packet_length = SX127X_ReadRegister(REG_RX_NB_BYTES);
    if (!radio_io_ok || packet_length > capacity)
    {
        SX127X_WriteRegister(REG_IRQ_FLAGS, IRQ_RX_DONE_MASK);
        return false;
    }

    SX127X_WriteRegister(
        REG_FIFO_ADDR_PTR, SX127X_ReadRegister(REG_FIFO_RX_CURRENT_ADDR));

    HAL_GPIO_WritePin(SX127X_NSS_PORT, SX127X_NSS_PIN, GPIO_PIN_RESET);
    if (HAL_SPI_Transmit(spi, &fifo_command, 1u,
                         SX127X_SPI_TIMEOUT_MS) != HAL_OK ||
        HAL_SPI_Receive(spi, buf, packet_length,
                        SX127X_SPI_TIMEOUT_MS) != HAL_OK)
    {
        radio_io_ok = false;
    }
    HAL_GPIO_WritePin(SX127X_NSS_PORT, SX127X_NSS_PIN, GPIO_PIN_SET);

    SX127X_WriteRegister(REG_IRQ_FLAGS, IRQ_RX_DONE_MASK);
    if (!radio_io_ok)
    {
        return false;
    }

    *len = packet_length;
    return true;
}

void SX127X_Sleep(void)
{
    SX127X_WriteRegister(REG_OP_MODE,
                         (uint8_t)(SX127X_LONG_RANGE_MODE | MODE_SLEEP));
}

void SX127X_SetModeStandby(void)
{
    SX127X_WriteRegister(REG_OP_MODE,
                         (uint8_t)(SX127X_LONG_RANGE_MODE | MODE_STDBY));
}

void SX127X_SetModeRx(void)
{
    SX127X_WriteRegister(
        REG_OP_MODE, (uint8_t)(SX127X_LONG_RANGE_MODE | MODE_RX_CONTINUOUS));
}

void SX127X_SetModeTx(void)
{
    SX127X_WriteRegister(REG_OP_MODE,
                         (uint8_t)(SX127X_LONG_RANGE_MODE | MODE_TX));
}

uint8_t SX127X_GetIrqFlags(void)
{
    return SX127X_ReadRegister(REG_IRQ_FLAGS);
}

void SX127X_ClearIrqFlags(void)
{
    SX127X_WriteRegister(REG_IRQ_FLAGS, 0xffu);
}

int16_t SX127X_GetPacketRssi(void)
{
    return (int16_t)SX127X_ReadRegister(REG_PKT_RSSI_VALUE) -
           RSSI_OFFSET_HF_PORT;
}

float SX127X_GetPacketSnr(void)
{
    return (float)(int8_t)SX127X_ReadRegister(REG_PKT_SNR_VALUE) * 0.25f;
}
