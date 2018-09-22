/*****************************************************************************
 * 
 * @filepath 	/evrs_bs_cc2650lp_app/src/ebs_etxlist_mgr.h
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
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#include <bcomdef.h>
#include "ebs_predefined.h"

/*****************************************************************************
 * Typedefs
 */
// Discovered ETX info
typedef struct EtxInfo_t{
	uint8_t addrType;	//!< Address Type: @ref ADDRTYPE_DEFINES
	uint8_t addr[B_ADDR_LEN];	//!< Device's Address
	uint8_t devID[ETX_DEVID_LEN];	// Tx Id
	uint16_t connHdl;	// connection handle
	uint16_t svcStartHdl;
	uint16_t svcEndHdl;
	uint8_t isBusy; // connection state
	uint8_t data;
} EtxInfo_t;


/*****************************************************************************
 * Global variables
 */
// Discovered ETX List
extern EtxInfo_t connList[MAX_CONNS];
extern uint8_t discRes;



/*****************************************************************************
 * Global functions
 */
void EBS_connMgr_resetList();
bool EBS_connMgr_findSvcUuid(uint16_t uuid, uint8_t *pData, uint8_t dataLen);
bool EBS_connMgr_checkBSID(uint8_t BSID, uint8_t *pEvtData, uint8_t dataLen);
void EBS_connMgr_addAddr(uint8_t *pAddr, uint8_t addrType);
void EBS_connMgr_addDeviceID(EtxInfo_t* pETX, uint8_t *pEvtData, uint8_t dataLen);
EtxInfo_t* EBS_connMgr_findByConnHdl(uint16_t tConnHdl);
EtxInfo_t* EBS_connMgr_findByAddr(uint8_t* pAddr);
uint8_t EBS_connMgr_checkAllPollState();

