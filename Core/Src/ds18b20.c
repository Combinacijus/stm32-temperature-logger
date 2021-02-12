#include "ds18b20.h"

//uint8_t DS18B20_Start(const DS18B20 const *DS18B20_Handle)
//{
//	uint8_t presence = 0;
//
//	Delay_Us(10);
////	Set_1W_Output();
////	Set_1W_Input();
//
//	return presence;
//}

//void Set_1W_Output(void)
//{
//	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
//	GPIO_InitStruct.Pin = DS18B20_Pin;
//	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT;
//	GPIO_InitStruct.Pull = GPIO_NOPULLUP;
//	HAL_GPIO_Init(DS18B20_GPIO_Port, &GPIO_InitStruct);
//}

/*
// Example of delay function using TIM1
int main()
{
	MX_TIM1_Init(); // Initialize to count every 1us
	HAL_TIM_Base_Start(&htim1);
}

void Delay_Us(uint16_t us)
{
	__HAL_TIM_SET_COUNTER(&htim1, 0);
	while (__HAL_TIM_GET_COUNTER(&htim1) < us)
		;
}
 */
