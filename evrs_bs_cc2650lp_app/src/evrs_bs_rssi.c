/*****************************************************************************

file	evrs_bs_rssi.c

brief	This file contain operations about polling rssi data

proj	EVRS

date	0527pm 15 Aug 2018

author	Ziyi

*****************************************************************************/

#include "evrs_bs_rssi.h"
#include "linkdb.h"



static readRssi_t *EBS_RssiAlloc(uint16_t connHandle);
static void EBS_RssiFree(uint16_t connHandle);
void EBS_readRssiHandler(UArg a0);

/*********************************************************************
 * @fn      EBS_StartRssi
 *
 * @brief   Start periodic RSSI reads on a link.
 *
 * @param   connHandle - connection handle of link
 * @param   period - RSSI read period in ms
 *
 * @return  SUCCESS: Terminate started
 *          bleIncorrectMode: No link
 *          bleNoResources: No resources
 */
bStatus_t EBS_StartRssi(uint16_t connHandle,
		uint16_t period) {
	readRssi_t *pRssi;

	// Verify link is up
	if (!linkDB_Up(connHandle))
	{
		return bleIncorrectMode;
	}

	// If already allocated
	if ((pRssi = EBS_RssiFind(connHandle)) != NULL)
	{
		// Stop timer
		Util_stopClock(pRssi->pClock);
		pRssi->period = period;
	}

	// Allocate structure
	else if ((pRssi = EBS_RssiAlloc(connHandle)) != NULL)
	{
		pRssi->period = period;
	}
	// Allocate failed
	else
	{
		return bleNoResources;
	}
	// Start timer
	Util_restartClock(pRssi->pClock, period);
	return SUCCESS;
}

/*********************************************************************
 * @fn      EBS_CancelRssi
 *
 * @brief   Cancel periodic RSSI reads on a link.
 *
 * @param   connHandle - connection handle of link
 *
 * @return  SUCCESS: Operation successful
 *          bleIncorrectMode: No link
 */
bStatus_t EBS_CancelRssi(uint16_t connHandle) {
	readRssi_t *pRssi;
	if ((pRssi = EBS_RssiFind(connHandle)) != NULL)
	{
		// Stop timer
		Util_stopClock(pRssi->pClock);

		// Free RSSI structure
		EBS_RssiFree(connHandle);
		return SUCCESS;
	}
	// Not found
	return bleIncorrectMode;
}

/*********************************************************************
 * @fn      gapCentralRole_RssiAlloc
 *
 * @brief   Allocate an RSSI structure.
 *
 * @param   connHandle - Connection handle
 *
 * @return  pointer to structure or NULL if allocation failed.
 */
static readRssi_t *EBS_RssiAlloc(uint16_t connHandle) {
	uint8_t i;

	// Find free RSSI structure
	for (i = 0; i < MAX_NUM_BLE_CONNS; i++)
	{
		if (readRssi[i].connHandle == GAP_CONNHANDLE_ALL)
		{
			readRssi_t *pRssi = &readRssi[i];
			pRssi->pClock = (Clock_Struct *) ICall_malloc(sizeof(Clock_Struct));
			if (pRssi->pClock)
			{
				Util_constructClock(pRssi->pClock,
						EBS_readRssiHandler, 0, 0, false, i);
				pRssi->connHandle = connHandle;
				return pRssi;
			}
		}
	}
	// No free structure found
	return NULL;
}

/*********************************************************************
 * @fn      gapCentralRole_RssiFind
 *
 * @brief   Find an RSSI structure.
 *
 * @param   connHandle - Connection handle
 *
 * @return  pointer to structure or NULL if not found.
 */
readRssi_t *EBS_RssiFind(uint16_t connHandle) {
	uint8_t i;
	// Find free RSSI structure
	for (i = 0; i < MAX_NUM_BLE_CONNS; i++)
	{
		if (readRssi[i].connHandle == connHandle)
		{
			return &readRssi[i];
		}
	}
	// Not found
	return NULL;
}

/*********************************************************************
 * @fn      gapCentralRole_RssiFree
 *
 * @brief   Free an RSSI structure.
 *
 * @param   connHandle - Connection handle
 *
 * @return  none
 */
static void EBS_RssiFree(uint16_t connHandle) {
	uint8_t i;

	// Find RSSI structure
	for (i = 0; i < MAX_NUM_BLE_CONNS; i++)
	{
		if (readRssi[i].connHandle == connHandle)
		{
			readRssi_t *pRssi = &readRssi[i];
			if (pRssi->pClock)
			{
				Clock_destruct(pRssi->pClock);

				// Free clock struct
				ICall_free(pRssi->pClock);
				pRssi->pClock = NULL;
			}
			pRssi->connHandle = GAP_CONNHANDLE_ALL;
			break;
		}
	}
}

/*********************************************************************
 * @fn      EBS_readRssiHandler
 *
 * @brief   Read RSSI handler function
 *
 * @param   a0 - read RSSI index
 *
 * @return  none
 */
void EBS_readRssiHandler(UArg a0) {
	EBS_enqueueMsg(EBS_RSSI_READ_EVT, SUCCESS,
			(uint8_t *) &readRssi[a0]);
}
