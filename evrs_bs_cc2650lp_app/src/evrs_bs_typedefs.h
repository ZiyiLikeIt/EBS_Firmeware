/****************************************
 *
 * @filename 	evrs_bs_typedefs.h
 *
 * @project 	evrs_bs_cc2650lp_app
 *
 * @brief 		global typedefs and macros
 *
 * @date 		22 Aug. 2018
 *
 * @author		Ziyi@outlook.com.au
 *
 ****************************************/


#ifndef EVRS_BS_TYPEDEFS_H_
#define EVRS_BS_TYPEDEFS_H_

#include "Util.h"


// RSSI read data structure
typedef struct {
	Clock_Struct *pClock; // pointer to clock struct
	uint16_t period;      // how often to read RSSI
	uint16_t connHandle;  // connection handle
} readRssi_t;

// Simple BLE Central Task Events
#define EBS_START_DISCOVERY_EVT      	0x0001
#define EBS_PAIRING_STATE_EVT     		0x0002
// #define EBS_PASSCODE_NEEDED_EVT     	0x0004
#define EBS_RSSI_READ_EVT            	0x0008
#define EBS_KEY_CHANGE_EVT            	0x0010
#define EBS_STATE_CHANGE_EVT          	0x0020
#define EBS_CONNECTING_TIMEOUT_EVT	  	0x0040
#define EBS_STACK_MSG_EVT				0x0080


#endif /* EVRS_BS_TYPEDEFS_H_ */
