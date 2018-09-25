/*****************************************************************************

file	evrs_bs_rssi.h

brief	This file contain definitions and prototypes about polling rssi data

proj	EVRS

date	0527pm 15 Aug 2018

author	Ziyi


*****************************************************************************/

#ifndef EVRS_BS_RSSI_H_
#define EVRS_BS_RSSI_H_

#include "bcomdef.h"
#include "gap.h"
#include "util.h"
#include "ble_user_config.h"

#include "evrs_bs_main.h"


/*
 * RSSI read data structure
 */
typedef struct {
	Clock_Struct *pClock; // pointer to clock struct
	uint16_t period;      // how often to read RSSI
	uint16_t connHandle;  // connection handle
} readRssi_t;

/*
 * external functions
 */
bStatus_t EBS_StartRssi(uint16_t connHandle, uint16_t period);
bStatus_t EBS_CancelRssi(uint16_t connHandle);
readRssi_t *EBS_RssiFind(uint16_t connHandle);




#endif /* EVRS_BS_RSSI_H_ */
