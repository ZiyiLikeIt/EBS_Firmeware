/*****************************************************************************
 * 
 * @filepath 	/evrs_bs_cc2650lp_app/src/ebs_etxlist_mgr.c
 * 
 * @project 	evrs_bs_cc2650lp_app
 * 
 * @brief 		TODO
 * 
 * @date 		16 Sep. 2018
 * 
 * @author		Ziyi@outlook.com.au
 *
 ****************************************************************************/
/*****************************************************************************
 * Includes
 */
#include "ebs_conn_mgr.h"
#include "ebs_predefined.h"
#include <gap.h>
#include <gatt.h>

/*****************************************************************************
 * Macros
 */
/** for list initial **/
#define DEFAULT_CONNHDL		0xFFFE
#define DEFAULT_SVCHDL		0x0000
#define DEFAULT_ADTYPE		0

/** advert data types **/
#define ETX_ADTYPE_DEST			0xAF
#define ETX_ADTYPE_DEVID		0xAE

/*****************************************************************************
 * Global variables
 */
// Discovered ETX List
EtxInfo_t connList[MAX_CONNS];

/*****************************************************************************
 * Local variables
 */
/** number of elements inside the connList **/
static uint8_t scanRes = 0;

/*****************************************************************************
 * Local prototypes
 */


/*****************************************************************************
 * Function definitions
 */

/** reset the list **/
void EBS_connMgr_resetList() {
	scanRes = 0;
	for (uint8_t i = 0; i < MAX_CONNS; i++) {
		connList[i].addrType = DEFAULT_ADTYPE;
		connList[i].connHdl = GAP_CONNHANDLE_INIT;
		connList[i].svcStartHdl = GATT_INVALID_HANDLE;
		connList[i].svcEndHdl = GATT_INVALID_HANDLE;
		connList[i].state = POLL_STATE_IDLE;
		connList[i].data = 0;
		memset(connList[i].addr, 0x00, B_ADDR_LEN);
		memset(connList[i].txDevID, 0x00, ETX_DEVID_LEN);
	}
}

/** find the service uuid in the ad data **/
bool EBS_connMgr_findSvcUuid(uint16_t uuid, uint8_t *pData, uint8_t dataLen) {
	uint8_t adLen;
	uint8_t adType;
	uint8_t *pEnd;

	pEnd = pData + dataLen - 1;

	// While end of data not reached
	while (pData < pEnd) {
		// Get length of next AD item
		adLen = *pData++;
		if (adLen > 0) {
			adType = *pData;

			// If AD type is for 16-bit service UUID
			if ((adType == GAP_ADTYPE_16BIT_MORE)
					|| (adType == GAP_ADTYPE_16BIT_COMPLETE)) {
				pData++;
				adLen--;

				// For each UUID in list
				while (adLen >= 2 && pData < pEnd) {
					// Check for match
					if ((pData[0] == LO_UINT16(uuid))
							&& (pData[1] == HI_UINT16(uuid))) {
						// Match found
						return TRUE;
					}
					// Go to next AD item
					pData += 2;
					adLen -= 2;
				}
				// Handle possible erroneous extra byte in UUID list
				if (adLen == 1) {
					pData++;
				}
			} else {
				// Go to next AD item
				pData += adLen;
			}
		}
	}
	// Match not found
	return FALSE;
}

/** check bsID of received data **/
bool EBS_connMgr_checkBSID(uint8_t BSID, uint8_t *pEvtData, uint8_t dataLen) {
	uint8_t adLen;
	uint8_t adType;
	uint8_t *pEnd;

	pEnd = pEvtData + dataLen - 1;

	// While end of data not reached
	while (pEvtData < pEnd) {
		// Get length of next data item
		adLen = *pEvtData++;
		if (adLen > 0) {
			adType = *pEvtData;
			//Display_print1(dispHdl, ii+9, 0, "0x%02x",adType);
			// If AD type is for local name
			if (adType == ETX_ADTYPE_DEST) {
				pEvtData++;
				// For base station identifier in the advert data
				return (*pEvtData == BSID);
			} else {
				// Go to next item
				pEvtData += adLen;
			}
		}
	}
	// No name found
	return FALSE;
}

/** Add a device to the etx discovery result list **/
void EBS_connMgr_addAddr(uint8_t *pAddr, uint8_t addrType) {
	uint8_t i;

	// If result count not at max
	if (scanRes < MAX_CONNS)
	{
		// Check if device is already in scan results
		for (i = 0; i < scanRes; i++)
			if (memcmp(pAddr, connList[i].addr, B_ADDR_LEN) == 0)
				return;

		// Add addr to scan result list
		memcpy(connList[scanRes].addr, pAddr, B_ADDR_LEN);
		connList[scanRes].addrType = addrType;

		// Increment scan result count
		scanRes++;
	}
}

/** add txDevID to the list **/
void EBS_connMgr_addDeviceID(EtxInfo_t* pETX, uint8_t *pEvtData, uint8_t dataLen) {
	uint8_t scanRspLen;
	uint8_t scanRspType;
	uint8_t *pEnd;

	pEnd = pEvtData + dataLen - 1;
	// While end of data not reached
	while (pEvtData < pEnd) {
		// Get length of next scan response item
		scanRspLen = *pEvtData++;
		if (scanRspLen > 0) {
			scanRspType = *pEvtData;

			// If scan response type is for local name
			if (scanRspType == ETX_ADTYPE_DEVID) {
				//Set name length in the device struct.
				pEvtData++;

				//Copy device id from the scan response data
				memcpy(pETX->txDevID, pEvtData, ETX_DEVID_LEN);
			}
		} else {
			// Go to next scan response item
			pEvtData += scanRspLen;
		}
	}
}

/** find element using connHdl **/
EtxInfo_t* EBS_connMgr_findByConnHdl(uint16_t tConnHdl) {
	uint8_t index = 0;
	EtxInfo_t *rtn = NULL;
	for (index = 0; index < scanRes; index++)
		if (connList[index].connHdl == tConnHdl)
			rtn = connList + index;
	return rtn;
}

/** find element using addr **/
EtxInfo_t* EBS_connMgr_findByAddr(uint8_t* pAddr) {
	uint8_t index = 0;
	EtxInfo_t *rtn = NULL;
	for (index = 0; index < scanRes; index++)
		if (memcmp(pAddr, connList[index].addr, B_ADDR_LEN) == 0)
			rtn = connList + index;
	return rtn;
}
