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
#include "evrs_bs_typedefs.h"
#include "util.h"
#include "ble_user_config.h"

#include "evrs_bs_main.h"



extern readRssi_t readRssi[MAX_NUM_BLE_CONNS];


extern bStatus_t EBS_StartRssi(uint16_t connHandle,
		uint16_t period);
extern bStatus_t EBS_CancelRssi(uint16_t connHandle);
extern readRssi_t *EBS_RssiFind(uint16_t connHandle);




#endif /* EVRS_BS_RSSI_H_ */
