#include "Tools.h"
#include "string.h"
// #include "stm32f1xx_hal_uart.h"

void LogIntToPC(uint8_t Data)
{
	HAL_UART_Transmit(&huart1, &Data, 1, 1000);
}

void LogStringToPC(char *Data)
{
	HAL_UART_Transmit(&huart1, Data, strlen(Data), 1000);
	HAL_UART_Transmit(&huart1, "\n", 1, 1000);
}

void LogInBinaryToPC(int Data)
{
	char *binValue;
	binValue = int_to_binary_string(Data);
	HAL_UART_Transmit(&huart1, binValue, 8, 1000);
	HAL_UART_Transmit(&huart1, "\n", 1, 1000);
}

char *int_to_binary_string(int num)
{
	int INT_BITS = 8;
	char *bin_str = malloc(INT_BITS + 1); // allocate space for binary string
	int i;
	for (i = 0; i < INT_BITS; i++)
	{
		bin_str[i] = (num & (1 << (INT_BITS - i - 1))) ? '1' : '0'; // get i-th bit
	}
	bin_str[i] = '\0'; // add null terminator
	return bin_str;
}
