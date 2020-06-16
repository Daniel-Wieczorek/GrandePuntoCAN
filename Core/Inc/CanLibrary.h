/*
 * CanLibrary.h
 * DWI: Header file created for definition of all used CAN frames
 * v. 1.0
 *
 *  Created on: Apr 1, 2020
 *      Author: Daniel
 */

#ifndef INC_CANLIBRARY_H_
#define INC_CANLIBRARY_H_

#endif /* INC_CANLIBRARY_H_ */


/* CAN structure ------------------------------------------------------------*/

typedef struct
{
	uint32_t ID;
	uint8_t DLC;
	uint8_t CAN_Tx[8];
	uint8_t CAN_Rx[8];

} CAN_MessageTypeDef;

/* -------------------------------------------------------------------------*/

CAN_MessageTypeDef IPC_Ligths =
{
		0x2214000,							// ID
		6,									// DLC
		{0x00,0x00,0x00,0x00,0x00,0x00}, 	// TX frame
		{0} 								// RX frame initialization
};

CAN_MessageTypeDef IPC_SpeedOdometerInfo =
{
		0x4294000,
		4,
		{0x27,0x10,0x0F,0xF0},
		{0}
};

CAN_MessageTypeDef IPC_StatusBCM =
{
		0x6214000,
		8,
		{0x00,0x00,0x40,0x00,0x00,0x64,0x00,0x41},
		{0}
};

CAN_MessageTypeDef IPC_SeatBelts =
{
		0x621401A,
		8,
		{0x00,0x00,0x40,0x00,0x00,0x64,0x00,0x41},
		{0}
};

CAN_MessageTypeDef IPC_EngineInfo =
{
		0x4214001,
		8,
		{0x00,0x00,0x00,0x91,0x00,0xFF,0xAA,0x00},
		{0}
};

CAN_MessageTypeDef IPC_StatusB_BSM =
{
		0x4214006,
		8,
		{0x00,0x00,0x00,0x20,0x00,0xAA,0x00,0xAA},
		{0}
};

CAN_MessageTypeDef IPC_HeartBeat =
{
		0x7000000,
		2,
		{0x00,0x00},
		{0}
};

CAN_MessageTypeDef IPC_StatusB_EPS =
{
		0x4214002,
		2,
		{0x00,0x00},
		{0}
};

