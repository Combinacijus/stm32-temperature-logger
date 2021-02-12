/*
 * ds18b20.h
 *
 *  Created on: Nov 13, 2020
 *      Author: Gintaras
 *
 *  Library for DS18B20 1-Wire temperature sensor
 *  Temperature range: -55C to +125C
 */

#ifndef INC_DS18B20_H_
#define INC_DS18B20_H_

#include <stdint.h>

// Commands
#define CMD_ROM_SEARCH 0xF0        // For ROM codes identification
#define CMD_ROM_READ 0x33          // Can be used with only 1 slave device on a bus
#define CMD_ROM_MATCH 0x55         // Follow by 64-bit ROM addresses specific device
#define CMD_ROM_SKIP 0xCC          // To address all devices on the bus simultaneously
#define CMD_ALARM_SEARCH 0xEC      // Identical to Search ROM except only slaves with a set alarm flag will respond
#define CMD_CONVERT_T 0x44         // Initiates a single temperature conversion
#define CMD_SCRATCHPAD_WRITE 0x4E  // Write 3 bytes of data to the DS18B20’s scratchpad
#define CMD_SCRATCHPAD_READ 0xBE   // Read the contents of the scratchpad
#define CMD_SCRATCHPAD_COPY 0x48   // Copies scratchpad bytes TH, TL and config to EEPROM
#define CMD_RECALL_E2 0xB8         // Recalls the alarm trigger values TH and TL and config from EEPROM to scratchpad
#define CMD_PWR_SUPPLY_READ 0xB4   // Determines if any slave device is using parasite power

//Variables
//typedef struct
//{
//	uint8_t GPIO_Port;
//	uint8_t GPIO_Pin;
//} DS18B20;
//
//// Functions
//extern void Delay_Us(uint16_t us);
//extern void Set_1W_Output(void);
//extern void Set_1W_Input(void);
//uint8_t DS18B20_Start(const DS18B20 const *DS18B20_Handle);

#endif /* INC_DS18B20_H_ */
