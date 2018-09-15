/****************************************
 *
 * @filename 	evrs_bs_main.c
 *
 * @project 	evrs_bs_cc2650lp_app
 *
 * @brief 		main functionality of base station
 *
 * @date 		22 Aug. 2018
 *
 * @author		Ziyi@outlook.com.au
 *
 ****************************************/


/*********************************************************************
 * INCLUDES
 */
#include <string.h>

#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Queue.h>

#include "bcomdef.h"

#include "hci_tl.h"
#include "linkdb.h"
#include "gatt.h"
#include "gapgattserver.h"
#include "gattservapp.h"
#include "central.h"
#include "gapbondmgr.h"
//#include "simple_gatt_profile.h"

#include "osal_snv.h"
#include "icall_apimsg.h"

#include "evrs_bs_typedefs.h"
#include "util.h"
#include "board_key.h"
#include "board_led.h"
#include "board_display.h"
#include "evrs_bs_rssi.h"
//#include <ti/mw/display/Display.h>
#include "board.h"

#include "ble_user_config.h"

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */

// Maximum number of scan responses
#define MAX_SCAN_RES		20

// Scan duration in ms
#define DEFAULT_SCAN_DURATION                 10000

// Discovery mode (limited, general, all)
#define DEFAULT_DISCOVERY_MODE                DEVDISC_MODE_ALL

// TRUE to use active scan
#define DEFAULT_DISCOVERY_ACTIVE_SCAN         TRUE

// TRUE to use white list during discovery
#define DEFAULT_DISCOVERY_WHITE_LIST          FALSE

// TRUE to use high scan duty cycle when creating link
#define LINK_HIGH_DUTY_CYCLE          TRUE

// TRUE to use white list when creating link
#define LINK_WHITE_LIST               FALSE

// Initial minimum connection interval (units of 1.25 ms.)
#define INITIAL_MIN_CONN_INTERVAL 	      	  16

// Initial minimum connection interval (units of 1.25 ms.)
#define INITIAL_MAX_CONN_INTERVAL             400

// Initial slave latency
#define INITIAL_SLAVE_LATENCY 		      	  0

// Initial supervision timeout (units of 1.25 ms)
#define INITIAL_CONN_TIMEOUT          	      700

// Default RSSI polling period in ms
#define DEFAULT_RSSI_PERIOD                   1000

// Whether to enable automatic parameter update request when a connection is
// formed
#define DEFAULT_ENABLE_UPDATE_REQUEST         FALSE

// Minimum connection interval (units of 1.25ms) if automatic parameter update
// request is enabled
#define DEFAULT_UPDATE_MIN_CONN_INTERVAL      80

// Maximum connection interval (units of 1.25ms) if automatic parameter update
// request is enabled
#define DEFAULT_UPDATE_MAX_CONN_INTERVAL      160

// Slave latency to use if automatic parameter update request is enabled
#define DEFAULT_UPDATE_SLAVE_LATENCY          0

// Supervision timeout value (units of 10ms) if automatic parameter update
// request is enabled
#define DEFAULT_UPDATE_CONN_TIMEOUT           600

// Default service discovery timer delay in ms
#define SVC_DISCOVERY_DELAY           500

// TRUE to filter discovery results on desired service UUID
#define DEV_DISC_BY_SVC_UUID          TRUE

// Length of bd addr as a string
#define B_ADDR_STR_LEN                        15

// Task configuration
#define EBS_TASK_PRIORITY                     1

#ifndef EBS_TASK_STACK_SIZE
#define EBS_TASK_STACK_SIZE                   864
#endif

// GATT Params
// EVRS Profile Service UUID
#define EVRSPROFILE_SERV_UUID 			0xAFF0
#define EVRSPROFILE_SYSID_UUID         	0xAFF2
#define EVRSPROFILE_DEVID_UUID      	0xAFF4
#define EVRSPROFILE_CMD_UUID        	0xAFF8
#define EVRSPROFILE_DATA_UUID         	0xAFFE

#define ETX_ADTYPE_DEST				0xAF
#define ETX_ADTYPE_DEVID			0xAE
#define ETX_DEVID_LEN 				4
#define ETX_DEVID_PREFIX			0x95

// Application states
typedef enum {
	EBS_STATE_INIT,
	EBS_STATE_DISCOVERY,
	EBS_STATE_UPLOAD,
	EBS_STATE_POLLING
} EbsState_t;

// Discovery states
typedef enum {
	EBS_DISC_STATE_IDLE,                // Idle
	EBS_DISC_STATE_MTU,                 // Exchange ATT MTU size
	EBS_DISC_STATE_SVC,                 // Service discovery
	EBS_DISC_STATE_CHAR                 // Characteristic discovery
} EbsDiscState_t;

// Polling states
typedef enum {
	EBS_POLL_STATE_IDLE,
	EBS_POLL_STATE_CONNECT,
	EBS_POLL_STATE_READ,
	EBS_POLL_STATE_WRITE,
	EBS_POLL_STATE_TERMINATE
} EbsPollState_t;

typedef enum {
	EVRSPROFILE_SYSID,
	EVRSPROFILE_DEVID,
	EVRSPROFILE_CMD,
	EVRSPROFILE_DATA
} ProfileId_t;


/*********************************************************************
 * TYPEDEFS
 */

// App event passed from profiles.
typedef struct {
	appEvtHdr_t hdr; // event header
	uint8_t *pData;  // event data
} EbsEvt_t;


/**
 * Type of device discovery (Scan) to perform.
 */
typedef struct {
	//char localName[20];		//!< Device's Name
	uint8_t addrType;	//!< Address Type: @ref ADDRTYPE_DEFINES
	uint8_t addr[B_ADDR_LEN];	//!< Device's Address
	uint8_t txDevID[ETX_DEVID_LEN];	// Tx Id
} DevRecInfo_t;


typedef struct {
	uint8_t addrType;	//!< Address Type: @ref ADDRTYPE_DEFINES
	uint8_t addr[B_ADDR_LEN];	//!< Device's Address
	uint8_t txDevID[ETX_DEVID_LEN];	// Tx Id
	uint16_t connHdl;	// connection handle
	EbsPollState_t state; // connection state
} TargetInfo_t;

/*********************************************************************
 * GLOBAL VARIABLES
 */

// Display Interface
// Display_Handle dispHdl = NULL;

/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * LOCAL VARIABLES
 */

// Entity ID globally used to check for source and/or destination of messages
static ICall_EntityID selfEntity;

// Semaphore globally used to post events to the application thread
static ICall_Semaphore sem;

// Clock object used to signal timeout
static Clock_Struct startDiscClock;

// Clock object used to timeout connection
static Clock_Struct connectingClock;

// Queue object used for app messages
static Queue_Struct appMsg;
static Queue_Handle appMsgQueue;

// Task pending events
static uint16_t events = 0;

// Task configuration
Task_Struct ebsTask;
Char ebsTaskStack[EBS_TASK_STACK_SIZE];

// GAP GATT Attributes
static const uint8_t attDeviceName[GAP_DEVICE_NAME_LEN] = "EVRS BaseStation";

// Number of scan results and scan result index
static uint8_t scanRes;
static uint8_t scanIdx;

// Scan result list
static DevRecInfo_t discTxList[MAX_SCAN_RES];

// Scanning state
static bool scanningStarted = FALSE;

// Connection handle of current connection
//static uint16_t connHandleList[MAX_NUM_BLE_CONNS] = GAP_CONNHANDLE_INIT;

// Application state
static EbsState_t ebsState = EBS_STATE_INIT;
//static bleState_t state = BLE_STATE_IDLE;

// Discovery state
static EbsDiscState_t discState = EBS_DISC_STATE_IDLE;

// Discovered service start and end handle
static uint16_t svcStartHdl = 0;
static uint16_t svcEndHdl = 0;

// Discovered characteristic handle
static uint16_t charHdl[4] = {0};

// GATT read/write procedure state
static bool procedureInProgress = FALSE;

// Maximum PDU size (default = 27 octets)
static uint16_t maxPduSize;

// Array of RSSI read structures
readRssi_t readRssi[MAX_NUM_BLE_CONNS];

// counter for profile found
int profileCounter = 0;

// Base Station Identifier
uint8_t baseStationID = 0x02;

// Target Tx List
TargetInfo_t targetList[MAX_NUM_BLE_CONNS];
TargetInfo_t* pVacantSlot = targetList;
TargetInfo_t* pConnectingSlot = NULL;

Semaphore_Handle targetConnSem;

// test
int tcounter = 0;

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void EBS_init(void);
static void EBS_taskFxn(UArg a0, UArg a1);

static void EBS_processGATTMsg(gattMsgEvent_t *pMsg);
static void EBS_handleKeys(uint8_t shift, uint8_t keys);
static void EBS_processStackMsg(ICall_Hdr *pMsg);
static void EBS_processAppMsg(EbsEvt_t *pMsg);
static void EBS_processRoleEvent(gapCentralRoleEvent_t *pEvent);
static void EBS_processGATTDiscEvent(gattMsgEvent_t *pMsg);
static uint8_t EBS_writeCharbyHandle(uint16_t connHandle, ProfileId_t charHdlId,
		uint8_t* pData, uint8_t len);
static uint8_t EBS_readCharbyHandle(uint16_t connHandle, ProfileId_t charHdlId);
static void EBS_startDiscovery(void);
static bool EBS_findSvcUuid(uint16_t uuid, uint8_t *pData,
		uint8_t dataLen);
static void EBS_discoverDevices(void);
void EBS_timeoutConnecting(UArg arg0);
static bool EBS_checkBSId(uint8_t bsID, uint8_t *pEvtData, uint8_t dataLen);
static void EBS_addDeviceInfo(uint8_t *pAddr, uint8_t addrType);
// static bool EBS_findLocalName(uint8_t *pEvtData, uint8_t dataLen);
static void EBS_addDeviceID(uint8_t index, uint8_t *pEvtData,
		uint8_t dataLen);
static void EBS_processPairState(uint8_t pairState, uint8_t status);
//static void EBS_processPasscode(uint16_t connectionHandle,
//		uint8_t uiOutputs);

static void EBS_processCmdCompleteEvt(hciEvt_CmdComplete_t *pMsg);

static uint8_t EBS_eventCB(gapCentralRoleEvent_t *pEvent);
//static void EBS_passcodeCB(uint8_t *deviceAddr,
//		uint16_t connHandle, uint8_t uiInputs, uint8_t uiOutputs);
static void EBS_pairStateCB(uint16_t connHandle, uint8_t pairState,
		uint8_t status);

void EBS_startDiscHandler(UArg a0);
void EBS_keyChangeHandler(uint8_t keys);

static void EBS_updateEbsState(EbsState_t newState);
static void EBS_stateChange(EbsState_t newState);
static void EBS_updatePollState(uint8_t targetIndex, EbsPollState_t newState);

static void EBS_updateTargetList(uint8_t* txID);

static uint32_t EBS_parseDevID(uint8_t* devID);

/*********************************************************************
 * PROFILE CALLBACKS
 */

// GAP Role Callbacks
static gapCentralRoleCB_t EBS_roleCB = { EBS_eventCB // Event callback
		};

// Bond Manager Callbacks
static gapBondCBs_t EBS_bondCB = {
		NULL, // Passcode callback
		EBS_pairStateCB                  // Pairing state callback
		};

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      SimpleBLEPeripheral_createTask
 *
 * @brief   Task creation function for the Simple BLE Peripheral.
 *
 * @param   none
 *
 * @return  none
 */
void EBS_createTask(void) {
	Task_Params taskParams;

	// Configure task
	Task_Params_init(&taskParams);
	taskParams.stack = ebsTaskStack;
	taskParams.stackSize = EBS_TASK_STACK_SIZE;
	taskParams.priority = EBS_TASK_PRIORITY;

	Task_construct(&ebsTask, EBS_taskFxn, &taskParams, NULL);
}

/*********************************************************************
 * @fn      EBS_Init
 *
 * @brief   Initialization function for the Simple BLE Central App Task.
 *          This is called during initialization and should contain
 *          any application specific initialization (ie. hardware
 *          initialization/setup, table initialization, power up
 *          notification).
 *
 * @param   none
 *
 * @return  none
 */
static void EBS_init(void) {
	uint8_t i;

	// ******************************************************************
	// N0 STACK API CALLS CAN OCCUR BEFORE THIS CALL TO ICall_registerApp
	// ******************************************************************
	// Register the current thread as an ICall dispatcher application
	// so that the application can send and receive messages.
	ICall_registerApp(&selfEntity, &sem);

	// Create an RTOS queue for message from profile to be sent to app.
	appMsgQueue = Util_constructQueue(&appMsg);

	// Setup discovery delay as a one-shot timer
	Util_constructClock(&startDiscClock, EBS_startDiscHandler,
	SVC_DISCOVERY_DELAY, 0, false, 0);

	// Set initial connection parameter values
	GAP_SetParamValue(TGAP_CONN_EST_INT_MIN, INITIAL_MIN_CONN_INTERVAL);
	GAP_SetParamValue(TGAP_CONN_EST_INT_MAX, INITIAL_MAX_CONN_INTERVAL);
	GAP_SetParamValue(TGAP_CONN_EST_SUPERV_TIMEOUT, INITIAL_CONN_TIMEOUT);
	GAP_SetParamValue(TGAP_CONN_EST_LATENCY, INITIAL_SLAVE_LATENCY);

	// Construct clock for connecting timeout
	Util_constructClock(&connectingClock, EBS_timeoutConnecting,
	DEFAULT_SCAN_DURATION, 0, false, 0);

	Board_initKeys(EBS_keyChangeHandler);
	Board_initLEDs();
	Board_Display_Init();

	// Initialize internal data
	for (i = 0; i < MAX_NUM_BLE_CONNS; i++)
	{
		readRssi[i].connHandle = GAP_CONNHANDLE_ALL;
		readRssi[i].pClock = NULL;
	}

	// Setup Central Profile
	{
		uint8_t maxScanRes = MAX_SCAN_RES;

		GAPCentralRole_SetParameter(GAPCENTRALROLE_MAX_SCAN_RES,
				sizeof(uint8_t), &maxScanRes);
	}

	// Setup GAP
	GAP_SetParamValue(TGAP_GEN_DISC_SCAN, DEFAULT_SCAN_DURATION);
	GAP_SetParamValue(TGAP_LIM_DISC_SCAN, DEFAULT_SCAN_DURATION);
	GGS_SetParameter(GGS_DEVICE_NAME_ATT, GAP_DEVICE_NAME_LEN,
			(void *) attDeviceName);

	// Setup the GAP Bond Manager
	{
		uint32_t passkey = 0; // passkey "000000"
		uint8_t pairMode = GAPBOND_PAIRING_MODE_WAIT_FOR_REQ;
		uint8_t mitm = FALSE;
		uint8_t ioCap = GAPBOND_IO_CAP_NO_INPUT_NO_OUTPUT;
		uint8_t bonding = FALSE;

		GAPBondMgr_SetParameter(GAPBOND_DEFAULT_PASSCODE, sizeof(uint32_t),
				&passkey);
		GAPBondMgr_SetParameter(GAPBOND_PAIRING_MODE, sizeof(uint8_t),
				&pairMode);
		GAPBondMgr_SetParameter(GAPBOND_MITM_PROTECTION, sizeof(uint8_t),
				&mitm);
		GAPBondMgr_SetParameter(GAPBOND_IO_CAPABILITIES, sizeof(uint8_t),
				&ioCap);
		GAPBondMgr_SetParameter(GAPBOND_BONDING_ENABLED, sizeof(uint8_t),
				&bonding);
	}

	// Initialize GATT Client
	VOID GATT_InitClient();

	// Register to receive incoming ATT Indications/Notifications
	GATT_RegisterForInd(selfEntity);

	// Initialize GATT attributes
	GGS_AddService(GATT_ALL_SERVICES);         // GAP
	GATTServApp_AddService(GATT_ALL_SERVICES); // GATT attributes

	// Start the Device
	VOID GAPCentralRole_StartDevice(&EBS_roleCB);

	// Register with bond manager after starting device
	GAPBondMgr_Register(&EBS_bondCB);

	// Register with GAP for HCI/Host messages (for RSSI)
	GAP_RegisterForMsgs(selfEntity);

	// Register for GATT local events and ATT Responses pending for transmission
	GATT_RegisterForMsgs(selfEntity);

	targetConnSem = Semaphore_create(0, NULL, NULL);
	Board_ledControl(BOARD_LED_ID_G, BOARD_LED_STATE_FLASH, 300);
}

/*********************************************************************
 * @fn      EBS_taskFxn
 *
 * @brief   Application task entry point for the Simple BLE Central.
 *
 * @param   none
 *
 * @return  events not processed
 */
static void EBS_taskFxn(UArg a0, UArg a1) {
	// Initialize application
	EBS_init();

	// Application main loop
	for (;;)
	{
		// Waits for a signal to the semaphore associated with the calling thread.
		// Note that the semaphore associated with a thread is signaled when a
		// message is queued to the message receive queue of the thread or when
		// ICall_signal() function is called onto the semaphore.
		ICall_Errno errno = ICall_wait(ICALL_TIMEOUT_FOREVER);

		if (errno == ICALL_ERRNO_SUCCESS)
		{
			ICall_EntityID dest;
			ICall_ServiceEnum src;
			ICall_HciExtEvt *pMsg = NULL;

			if (ICall_fetchServiceMsg(&src, &dest,
					(void **) &pMsg) == ICALL_ERRNO_SUCCESS)
			{
				if ((src == ICALL_SERVICE_CLASS_BLE) && (dest == selfEntity))
				{
					// Process inter-task message
					EBS_processStackMsg((ICall_Hdr *) pMsg);
				}

				if (pMsg)
				{
					ICall_freeMsg(pMsg);
				}
			}
		}

		// If RTOS queue is not empty, process app message
		while (!Queue_empty(appMsgQueue))
		{
			EbsEvt_t *pMsg = (EbsEvt_t *) Util_dequeueMsg(appMsgQueue);
			if (pMsg)
			{
				// Process message
				EBS_processAppMsg(pMsg);

				// Free the space from the message
				ICall_free(pMsg);
			}
		}

		if (events & EBS_START_DISCOVERY_EVT)
		{
			events &= ~EBS_START_DISCOVERY_EVT;

			EBS_startDiscovery();
		}
	}
}

/*********************************************************************
 * @fn      EBS_processStackMsg
 *
 * @brief   Process an incoming task message.
 *
 * @param   pMsg - message to process
 *
 * @return  none
 */
static void EBS_processStackMsg(ICall_Hdr *pMsg) {
	switch (pMsg->event)
	{
		case GAP_MSG_EVENT:
			EBS_processRoleEvent((gapCentralRoleEvent_t *) pMsg);
			break;

		case GATT_MSG_EVENT:
			EBS_processGATTMsg((gattMsgEvent_t *) pMsg);
			break;

		case HCI_GAP_EVENT_EVENT:
		{
			// Process HCI message
			switch (pMsg->status)
			{
				case HCI_COMMAND_COMPLETE_EVENT_CODE:
					EBS_processCmdCompleteEvt(
							(hciEvt_CmdComplete_t *) pMsg);
					break;

				default:
					break;
			}
		}
			break;

		default:
			break;
	}
}

/*********************************************************************
 * @fn      EBS_processAppMsg
 *
 * @brief   Central application event processing function.
 *
 * @param   pMsg - pointer to event structure
 *
 * @return  none
 */
static void EBS_processAppMsg(EbsEvt_t *pMsg) {
	switch (pMsg->hdr.event)
	{
		case EBS_STACK_MSG_EVT:
			EBS_processStackMsg((ICall_Hdr *) pMsg->pData);

			// Free the stack message
			ICall_freeMsg(pMsg->pData);
			break;

		case EBS_STATE_CHANGE_EVT:
			EBS_stateChange((EbsState_t)pMsg->hdr.state);
			break;

		case EBS_KEY_CHANGE_EVT:
			EBS_handleKeys(0, pMsg->hdr.state);
			break;

		case EBS_RSSI_READ_EVT:
		{
			readRssi_t *pRssi = (readRssi_t *) pMsg->pData;

			// If link is up and RSSI reads active
			if (pRssi->connHandle != GAP_CONNHANDLE_ALL
					&& linkDB_Up(pRssi->connHandle))
			{
				// Restart timer
				Util_restartClock(pRssi->pClock, pRssi->period);

				// Read RSSI
				VOID HCI_ReadRssiCmd(pRssi->connHandle);
			}
		}
			break;

			// Pairing event
		case EBS_PAIRING_STATE_EVT:
		{
			EBS_processPairState(pMsg->hdr.state, *pMsg->pData);

			ICall_free(pMsg->pData);
			break;
		}

			/* Passcode event
		case EBS_PASSCODE_NEEDED_EVT:
		{
			EBS_processPasscode(connHandle, *pMsg->pData);

			ICall_free(pMsg->pData);
			break;
		}
		*/
			// Connecting to device timed out
		case EBS_CONNECTING_TIMEOUT_EVT:
		{
			GAPCentralRole_TerminateLink(pConnectingSlot->connHdl);
		}

		default:
			// Do nothing.
			break;
	}
}

/*********************************************************************
 * @fn      EBS_processRoleEvent
 *
 * @brief   Central role event processing function.
 *
 * @param   pEvent - pointer to event structure
 *
 * @return  none
 */
static void EBS_processRoleEvent(gapCentralRoleEvent_t *pEvent) {
	switch (pEvent->gap.opcode)
	{
		case GAP_DEVICE_INIT_DONE_EVENT:
		{
			maxPduSize = pEvent->initDone.dataPktLen;
			uout0("EVRS BS initialized");
			uout0(Util_convertBdAddr2Str(pEvent->initDone.devAddr));
			uout1("BS ID: 0x%02x", baseStationID);
		}
			break;

		case GAP_DEVICE_INFO_EVENT:
		{

			//Find tx device address by UUID
			if (EBS_findSvcUuid(EVRSPROFILE_SERV_UUID,
					pEvent->deviceInfo.pEvtData, pEvent->deviceInfo.dataLen) &&
				EBS_checkBSId(baseStationID,
					pEvent->deviceInfo.pEvtData, pEvent->deviceInfo.dataLen))
			{
				EBS_addDeviceInfo(pEvent->deviceInfo.addr,
						pEvent->deviceInfo.addrType);
			}

			// Check if the discovered device is already in scan results
			uint8_t index;
			for (index = 0; index < scanRes; index++)
			{
				if (memcmp(pEvent->deviceInfo.addr, discTxList[index].addr, B_ADDR_LEN)
						== 0)
				{
					//Update deviceInfo entry with the name
					EBS_addDeviceID(index,
							pEvent->deviceInfo.pEvtData,
							pEvent->deviceInfo.dataLen);
				}
			}
		}
			break;

		case GAP_DEVICE_DISCOVERY_EVENT:
		{
			// discovery complete
			scanningStarted = FALSE;
			// initialize scan index to first
			scanIdx = 0;
			uout1("%d Device(s) found", scanRes);
			EBS_updateEbsState(EBS_STATE_UPLOAD);
		}
			break;

		case GAP_LINK_ESTABLISHED_EVENT:
		{
			if (pEvent->gap.hdr.status == SUCCESS)
			{
				//Connect to selected device
				//state = BLE_STATE_CONNECTED;
				//connHandle = pEvent->linkCmpl.connectionHandle;
				procedureInProgress = TRUE;

				// If service discovery not performed initiate service discovery
				if (charHdl[0] == 0)
				{
					Util_startClock(&startDiscClock);
				}

				//Find device ID in discTxList struct
				uint8_t i;
				for (i = 0; i < scanRes; i++)
				{
					if (memcmp(pEvent->linkCmpl.devAddr, targetList[i].addr,
					B_ADDR_LEN) == NULL)
					{
						break;
					}
				}
				targetList[i].connHdl = pEvent->linkCmpl.connectionHandle;
				uout1("Tx ID 0x%08x Connected", EBS_parseDevID(targetList[i].txDevID));
				uout1("Tx Addr %s", Util_convertBdAddr2Str(pEvent->linkCmpl.devAddr));

			} else
			{
				//connHandle = GAP_CONNHANDLE_INIT;
				discState = EBS_DISC_STATE_IDLE;

				uout1("Connect Failed: 0x%02x",pEvent->gap.hdr.status);
			}
		}
			break;

		case GAP_LINK_TERMINATED_EVENT:
		{
			//state = BLE_STATE_IDLE;
			//connHandle = GAP_CONNHANDLE_INIT;
			discState = EBS_DISC_STATE_IDLE;
			memset(charHdl,0x00,4);
			profileCounter = 0;
			procedureInProgress = FALSE;
			EBS_updatePollState(1, EBS_POLL_STATE_IDLE);

			// Cancel RSSI reads
			EBS_CancelRssi(pEvent->linkTerminate.connectionHandle);

			//Clear screen and display disconnect reason
			uout1("Disconnected: 0x%02x", pEvent->linkTerminate.reason);
		}
			break;
		/*
		case GAP_LINK_PARAM_UPDATE_EVENT:
		{
			if (state == BLE_STATE_CONNECTED)
			{
				if (pEvent->linkUpdate.status == SUCCESS)
				{
					Display_print1(dispHdl, ROW_FOUR, 0, "ParUpd: %d ms",
							pEvent->linkUpdate.connInterval * 1.25);
				} else
				{
					Display_print1(dispHdl, ROW_FOUR, 0, "Param error: %d",
							pEvent->linkUpdate.status);
				}
			}
		}
			break;
		*/
		default:
			break;
	}
}


/*********************************************************************
 * @fn      EBS_processGATTMsg
 *
 * @brief   Process GATT messages and events.
 *
 * @return  none
 */
static void EBS_processGATTMsg(gattMsgEvent_t *pMsg) {
	if (ebsState == EBS_STATE_POLLING)
	{
		// See if GATT server was unable to transmit an ATT response
		if (pMsg->hdr.status == blePending)
		{
			// No HCI buffer was available. App can try to retransmit the response
			// on the next connection event. Drop it for now.
			uout1("ATT Rsp drped %d", pMsg->method);
		} else if ((pMsg->method == ATT_READ_RSP)
				|| ((pMsg->method == ATT_ERROR_RSP)
						&& (pMsg->msg.errorRsp.reqOpcode == ATT_READ_REQ)))
		{
			if (pMsg->method == ATT_ERROR_RSP)
			{
				uout1("Read Error 0x%02x", pMsg->msg.errorRsp.errCode);
			} else
			{
				// After a successful read, display the read value
				uout1("Read rsp: 0x%02x", pMsg->msg.readRsp.pValue[0]);
				EBS_updatePollState(0, EBS_POLL_STATE_WRITE);
			}

			procedureInProgress = FALSE;
		} else if ((pMsg->method == ATT_WRITE_RSP)
				|| ((pMsg->method == ATT_ERROR_RSP)
						&& (pMsg->msg.errorRsp.reqOpcode == ATT_WRITE_REQ)))
		{
			if (pMsg->method == ATT_ERROR_RSP)
			{
				uout1("Write Error 0x%02x", pMsg->msg.errorRsp.errCode);
			} else
			{
				// After a successful write, display the value that was written and
				// increment value
				uout0("Write done");
				EBS_updatePollState(0, EBS_POLL_STATE_TERMINATE);
			}

			procedureInProgress = FALSE;
		} else if (pMsg->method == ATT_FLOW_CTRL_VIOLATED_EVENT)
		{
			// ATT request-response or indication-confirmation flow control is
			// violated. All subsequent ATT requests or indications will be dropped.
			// The app is informed in case it wants to drop the connection.

			// Display the opcode of the message that caused the violation.
			uout1("FC Violated: %d", pMsg->msg.flowCtrlEvt.opcode);
		} else if (pMsg->method == ATT_MTU_UPDATED_EVENT)
		{
			// MTU size updated
			uout1("MTU Size: %d", pMsg->msg.mtuEvt.MTU);
		} else if (discState != EBS_DISC_STATE_IDLE)
		{
			EBS_processGATTDiscEvent(pMsg);
		}
	} // else - in case a GATT message came after a connection has dropped, ignore it.

	// Needed only for ATT Protocol messages
	GATT_bm_free(&pMsg->msg, pMsg->method);
}

/*********************************************************************
 * @fn      EBS_processCmdCompleteEvt
 *
 * @brief   Process an incoming OSAL HCI Command Complete Event.
 *
 * @param   pMsg - message to process
 *
 * @return  none
 */
static void EBS_processCmdCompleteEvt(hciEvt_CmdComplete_t *pMsg) {
	switch (pMsg->cmdOpcode)
	{
		case HCI_READ_RSSI:
		{
			if (ebsState == EBS_STATE_POLLING)
			{
				int8 rssi = (int8) pMsg->pReturnParam[3];
				uout1("RSSI -dB: %d", (uint32_t )(-rssi));
			}
		}
			break;

		default:
			break;
	}
}

/*********************************************************************
 * @fn      EBS_processPairState
 *
 * @brief   Process the new paring state.
 *
 * @return  none
 */
static void EBS_processPairState(uint8_t pairState, uint8_t status) {
	if (pairState == GAPBOND_PAIRING_STATE_STARTED)
	{
		uout0("Pairing started");
	} else if (pairState == GAPBOND_PAIRING_STATE_COMPLETE)
	{
		if (status == SUCCESS)
		{
			uout0("Pairing success");
		} else
		{
			uout1("Pairing fail: %d", status);
		}
	} else if (pairState == GAPBOND_PAIRING_STATE_BONDED)
	{
		if (status == SUCCESS)
		{
			uout0("Bonding success");
		}
	} else if (pairState == GAPBOND_PAIRING_STATE_BOND_SAVED)
	{
		if (status == SUCCESS)
		{
			uout0("Bond save succ");
		} else
		{
			uout1("Bond save fail: %d", status);
		}
	}
}

/*********************************************************************
 * @fn      EBS_processPasscode
 *
 * @brief   Process the Passcode request.
 *
 * @return  none
 **************************
static void EBS_processPasscode(uint16_t connectionHandle,
		uint8_t uiOutputs) {
	uint32_t passcode;

	// Create random passcode
	passcode = Util_GetTRNG();
	passcode %= 1000000;

	// Display passcode to user
	if (uiOutputs != 0)
	{
		Display_print0(dispHdl, ROW_FOUR, 0, "Passcode:");
		Display_print1(dispHdl, ROW_FIVE, 0, "%d", passcode);
	}
	// Send passcode response
	GAPBondMgr_PasscodeRsp(connectionHandle, SUCCESS, passcode);
}
*/
/*********************************************************************
 * @fn      EBS_startDiscovery
 *
 * @brief   Start service discovery.
 *
 * @return  none
 */
static void EBS_startDiscovery(void) {
	attExchangeMTUReq_t req;

	// Initialize cached handles
	svcStartHdl = svcEndHdl = 0;
	memset(charHdl, 0x00, 4);
	discState = EBS_DISC_STATE_SVC;

	// Discovery simple BLE service
	uint8_t uuid[ATT_BT_UUID_SIZE] = { LO_UINT16(EVRSPROFILE_SERV_UUID),
			HI_UINT16(EVRSPROFILE_SERV_UUID) };
	VOID GATT_DiscPrimaryServiceByUUID(pConnectingSlot->connHdl, uuid,
			ATT_BT_UUID_SIZE, selfEntity);

	// Discover GATT Server's Rx MTU size
	//req.clientRxMTU = maxPduSize - L2CAP_HDR_SIZE;

	// ATT MTU size should be set to the minimum of the Client Rx MTU
	// and Server Rx MTU values
	//VOID GATT_ExchangeMTU(connHandle, &req, selfEntity);
}

/*********************************************************************
 * @fn      EBS_processGATTDiscEvent
 *
 * @brief   Process GATT discovery event
 *
 * @return  none
 */
static void EBS_processGATTDiscEvent(gattMsgEvent_t *pMsg) {
	if (discState == EBS_DISC_STATE_SVC)
	{
		// Service found, store handles
		if (pMsg->method == ATT_FIND_BY_TYPE_VALUE_RSP
				&& pMsg->msg.findByTypeValueRsp.numInfo > 0)
		{
			svcStartHdl = ATT_ATTR_HANDLE(
					pMsg->msg.findByTypeValueRsp.pHandlesInfo, 0);
			svcEndHdl = ATT_GRP_END_HANDLE(
					pMsg->msg.findByTypeValueRsp.pHandlesInfo, 0);

		}

		// If procedure complete
		if (((pMsg->method == ATT_FIND_BY_TYPE_VALUE_RSP)
				&& (pMsg->hdr.status == bleProcedureComplete))
				|| (pMsg->method == ATT_ERROR_RSP))
		{
			if (svcStartHdl != 0)
			{
				// Discover characteristic
				VOID GATT_DiscAllChars(pConnectingSlot->connHdl, svcStartHdl, svcEndHdl,
					selfEntity);
				discState = EBS_DISC_STATE_CHAR;
			}
		}
	} else if (discState == EBS_DISC_STATE_CHAR)
	{
		// Characteristic found, store handle
		if ((pMsg->method == ATT_READ_BY_TYPE_RSP)
				&& (pMsg->msg.readByTypeRsp.numPairs > 0))
		{
			for (int counter = 0; counter < pMsg->msg.readByTypeRsp.numPairs; counter++)
			{
				switch(*(pMsg->msg.readByTypeRsp.pDataList + counter*7 + 5))
				{
					case LO_UINT16(EVRSPROFILE_SYSID_UUID):
						charHdl[EVRSPROFILE_SYSID] = BUILD_UINT16(
								*(pMsg->msg.readByTypeRsp.pDataList + counter*7 + 3),
								*(pMsg->msg.readByTypeRsp.pDataList + counter*7 + 4));
						profileCounter++;
						break;

					case LO_UINT16(EVRSPROFILE_DEVID_UUID):
						charHdl[EVRSPROFILE_DEVID] = BUILD_UINT16(
								*(pMsg->msg.readByTypeRsp.pDataList + counter*7 + 3),
								*(pMsg->msg.readByTypeRsp.pDataList + counter*7 + 4));
						profileCounter++;
						break;

					case LO_UINT16(EVRSPROFILE_CMD_UUID):
						charHdl[EVRSPROFILE_CMD] = BUILD_UINT16(
								*(pMsg->msg.readByTypeRsp.pDataList + counter*7 + 3),
								*(pMsg->msg.readByTypeRsp.pDataList + counter*7 + 4));
						profileCounter++;
						break;

					case LO_UINT16(EVRSPROFILE_DATA_UUID):
						charHdl[EVRSPROFILE_DATA] = BUILD_UINT16(
								*(pMsg->msg.readByTypeRsp.pDataList + counter*7 + 3),
								*(pMsg->msg.readByTypeRsp.pDataList + counter*7 + 4));
						profileCounter++;
						break;
				}
			}
		} else if ((pMsg->method == ATT_READ_BY_TYPE_RSP)
				&& (pMsg->hdr.status == bleProcedureComplete)
				|| (pMsg->method == ATT_ERROR_RSP))
		{
			uout1("%d Profile(s) Found ", profileCounter);
			procedureInProgress = FALSE;
			discState = EBS_DISC_STATE_IDLE;
			EBS_updatePollState(0, EBS_POLL_STATE_READ);
		}

	}
}

/*********************************************************************
 * @fn      EBS_findSvcUuid
 *
 * @brief   Find a given UUID in an advertiser's service UUID list.
 *
 * @return  TRUE if service UUID found
 */
static bool EBS_findSvcUuid(uint16_t uuid, uint8_t *pData,
		uint8_t dataLen) {
	uint8_t adLen;
	uint8_t adType;
	uint8_t *pEnd;

	pEnd = pData + dataLen - 1;

	// While end of data not reached
	while (pData < pEnd)
	{
		// Get length of next AD item
		adLen = *pData++;
		if (adLen > 0)
		{
			adType = *pData;

			// If AD type is for 16-bit service UUID
			if ((adType == GAP_ADTYPE_16BIT_MORE)
					|| (adType == GAP_ADTYPE_16BIT_COMPLETE))
			{
				pData++;
				adLen--;

				// For each UUID in list
				while (adLen >= 2 && pData < pEnd)
				{
					// Check for match
					if ((pData[0] == LO_UINT16(uuid))
							&& (pData[1] == HI_UINT16(uuid)))
					{
						// Match found
						return TRUE;
					}

					// Go to next AD item
					pData += 2;
					adLen -= 2;
				}

				// Handle possible erroneous extra byte in UUID list
				if (adLen == 1)
				{
					pData++;
				}

			} else
			{
				// Go to next AD item
				pData += adLen;
			}
		}
	}
	// Match not found
	return FALSE;
}

/*********************************************************************
 * @fn      EBS_discoverDevices
 *
 * @brief   Scan to discover devices.
 *
 * @return  none
 */
static void EBS_discoverDevices(void) {
	if (!scanningStarted)
	{
		scanningStarted = TRUE;

		//Clear old scan results
		scanRes = 0;
		memset(discTxList, NULL, sizeof(discTxList[0]) * MAX_SCAN_RES);

		uout0("Discovering...");
		GAPCentralRole_StartDiscovery(DEFAULT_DISCOVERY_MODE,
		DEFAULT_DISCOVERY_ACTIVE_SCAN,
		DEFAULT_DISCOVERY_WHITE_LIST);
	} else
	{
		GAPCentralRole_CancelDiscovery();
	}
}

/**********************************************************************
 * @fn      EBS_timeoutConnecting
 *
 * @brief   Post event if connecting is timed out.
 *
 * @return  none
 */
Void EBS_timeoutConnecting(UArg arg0) {
	if (pConnectingSlot != NULL)
	{
		EBS_enqueueMsg(EBS_CONNECTING_TIMEOUT_EVT, 0, NULL);
	}
}


static bool EBS_checkBSId(uint8_t bsID, uint8_t *pEvtData, uint8_t dataLen) {
	uint8_t adLen;
	uint8_t adType;
	uint8_t *pEnd;

	int ii = 0;

	pEnd = pEvtData + dataLen - 1;

	//Display_print5(dispHdl, 9, 0,"len %d, 0x%02x, 0x%02x, 0x%02x, 0x%02x",
	//		dataLen,pEvtData[8],pEvtData[9],pEvtData[10],pEvtData[11]);
	// While end of data not reached
	while (pEvtData < pEnd)
	{
		// Get length of next data item
		adLen = *pEvtData++;
		if (adLen > 0)
		{
			adType = *pEvtData;
			//Display_print1(dispHdl, ii+9, 0, "0x%02x",adType);
			// If AD type is for local name
			if (adType == ETX_ADTYPE_DEST)
			{
				pEvtData++;
				// For base station identifier in the advert data
				return (*pEvtData == bsID);
			} else
			{
				// Go to next item
				pEvtData += adLen;
				ii++;
			}
		}
	}
	// No name found
	return FALSE;
}


/*********************************************************************
 * @fn      EBS_addDeviceInfo
 *
 * @brief   Add a device to the device discovery result list
 *
 * @return  none
 */
static void EBS_addDeviceInfo(uint8_t *pAddr, uint8_t addrType) {
	uint8_t i;

	// If result count not at max
	if (scanRes < MAX_SCAN_RES)
	{
		// Check if device is already in scan results
		for (i = 0; i < scanRes; i++)
		{
			if (memcmp(pAddr, discTxList[i].addr, B_ADDR_LEN) == 0)
			{
				return;
			}
		}

		// Add addr to scan result list
		memcpy(discTxList[scanRes].addr, pAddr, B_ADDR_LEN);
		discTxList[scanRes].addrType = addrType;

		// Increment scan result count
		scanRes++;
	}
}

/*********************************************************************
 * @fn      EBS_findLocalName
 *
 * @brief   Check if pEvtData contains a device local name
 *
 * @return  TRUE if local name found
 *******************************************************
static bool EBS_findLocalName(uint8_t *pEvtData, uint8_t dataLen) {
	uint8_t adLen;
	uint8_t adType;
	uint8_t *pEnd;

	pEnd = pEvtData + dataLen - 1;

	// While end of data not reached
	while (pEvtData < pEnd)
	{
		// Get length of next data item
		adLen = *pEvtData++;
		if (adLen > 0)
		{
			adType = *pEvtData;

			// If AD type is for local name
			if ((adType == GAP_ADTYPE_LOCAL_NAME_SHORT)
					|| (adType == GAP_ADTYPE_LOCAL_NAME_COMPLETE))
			{
				pEvtData++;
				adLen--;
				// For each local name in list
				if (adLen >= 2 && pEvtData < pEnd)
				{
					return TRUE;
				}

				// Handle possible erroneous extra byte in advertisement data
				if (adLen == 1)
				{
					pEvtData++;
				}
			} else
			{
				// Go to next item
				pEvtData += adLen;
			}
		}
	}
	// No name found
	return FALSE;
}
*/
/*********************************************************************
 * @fn      EBS_addDeviceName
 *
 * @brief   Add a name to an existing device in the scan result list
 *
 * @return  none
 */
static void EBS_addDeviceID(uint8_t index, uint8_t *pEvtData,
		uint8_t dataLen) {
	uint8_t scanRspLen;
	uint8_t scanRspType;
	uint8_t *pEnd;

	pEnd = pEvtData + dataLen - 1;

	// While end of data not reached
	while (pEvtData < pEnd)
	{
		// Get length of next scan response item
		scanRspLen = *pEvtData++;
		if (scanRspLen > 0)
		{
			scanRspType = *pEvtData;

			// If scan response type is for local name
			if (scanRspType == ETX_ADTYPE_DEVID)
			{
				//Set name length in the device struct.
				pEvtData++;

				//Copy device id from the scan response data
				for (int j = 0; j < ETX_DEVID_LEN; j++)
					discTxList[index].txDevID[j] = *pEvtData++;
			}
		} else
		{
			// Go to next scan response item
			pEvtData += scanRspLen;
		}
	}
}

/*********************************************************************
 * @fn      EBS_eventCB
 *
 * @brief   Central event callback function.
 *
 * @param   pEvent - pointer to event structure
 *
 * @return  TRUE if safe to deallocate event message, FALSE otherwise.
 */
static uint8_t EBS_eventCB(gapCentralRoleEvent_t *pEvent) {
	// Forward the role event to the application
	if (EBS_enqueueMsg(EBS_STACK_MSG_EVT, SUCCESS, (uint8_t *) pEvent))
	{
		// App will process and free the event
		return FALSE;
	}

	// Caller should free the event
	return TRUE;
}


/*********************************************************************
 * @fn      EBS_pairStateCB
 *
 * @brief   Pairing state callback.
 *
 * @return  none
 */
static void EBS_pairStateCB(uint16_t connHandle, uint8_t pairState,
		uint8_t status) {
	uint8_t *pData;

	// Allocate space for the event data.
	if ((pData = ICall_malloc(sizeof(uint8_t))))
	{
		*pData = status;

		// Queue the event.
		EBS_enqueueMsg(EBS_PAIRING_STATE_EVT, pairState, pData);
	}
}

/*********************************************************************
 * @fn      EBS_passcodeCB
 *
 * @brief   Passcode callback.
 *
 * @return  none
 *********************************************************
static void EBS_passcodeCB(uint8_t *deviceAddr,
		uint16_t connHandle, uint8_t uiInputs, uint8_t uiOutputs) {
	uint8_t *pData;

	// Allocate space for the passcode event.
	if ((pData = ICall_malloc(sizeof(uint8_t))))
	{
		*pData = uiOutputs;

		// Enqueue the event.
		EBS_enqueueMsg(EBS_PASSCODE_NEEDED_EVT, 0, pData);
	}
}
*/
/*********************************************************************
 * @fn      EBS_startDiscHandler
 *
 * @brief   Clock handler function
 *
 * @param   a0 - ignored
 *
 * @return  none
 */
void EBS_startDiscHandler(UArg a0) {
	events |= EBS_START_DISCOVERY_EVT;

	// Wake up the application thread when it waits for clock event
	Semaphore_post(sem);
}

/*********************************************************************
 * @fn      EBS_keyChangeHandler
 *
 * @brief   Key event handler function
 *
 * @param   a0 - ignored
 *
 * @return  none
 */
void EBS_keyChangeHandler(uint8_t keys) {
	EBS_enqueueMsg(EBS_KEY_CHANGE_EVT, keys, NULL);
}

/*********************************************************************
 * @fn      EBS_enqueueMsg
 *
 * @brief   Creates a message and puts the message in RTOS queue.
 *
 * @param   event - message event.
 * @param   state - message state.
 * @param   pData - message data pointer.
 *
 * @return  TRUE or FALSE
 */
uint8_t EBS_enqueueMsg(uint8_t event, uint8_t status,
		uint8_t *pData) {
	EbsEvt_t *pMsg = ICall_malloc(sizeof(EbsEvt_t));

	// Create dynamic pointer to message.
	if (pMsg)
	{
		pMsg->hdr.event = event;
		pMsg->hdr.state = status;
		pMsg->pData = pData;

		// Enqueue the message.
		return Util_enqueueMsg(appMsgQueue, sem, (uint8_t *) pMsg);
	}
	return FALSE;
}

static uint32_t EBS_parseDevID(uint8_t* devID) {
	return BUILD_UINT32(devID[0], devID[1], devID[2], devID[3]);
}

static uint8_t EBS_writeCharbyHandle(uint16_t connHandle, ProfileId_t charHdlId,
		uint8_t* pData, uint8_t len) {
	if (len > 23)
		return FAILURE;
	// Do a write using char handle
	attWriteReq_t req;
	uint8_t status;
	req.pValue = GATT_bm_alloc(connHandle, ATT_WRITE_REQ, len, NULL);
	if (req.pValue != NULL)
	{
		req.handle = charHdl[charHdlId];
		req.len = len;
		//memcpy(req.pValue, pData, len);
		for (int i = 0; i < len; i++)
			req.pValue[i] = pData[i];
		req.sig = 0;
		req.cmd = 0;
		status = GATT_WriteCharValue(connHandle, &req, selfEntity);
		//Display_print2(dispHandle,ROW_SIX,0,"Write req sent [%d,0x%02x]", req.len, *(req.pValue));
		if (status != SUCCESS)
			GATT_bm_free((gattMsg_t *) &req, ATT_WRITE_REQ);
	} else
	{
		status = bleMemAllocError;
	}
	return status;
}

static uint8_t EBS_readCharbyHandle(uint16_t connHandle, ProfileId_t charHdlId) {
	// Do a read
	attReadReq_t req;
	uint8_t status;
	req.handle = charHdl[charHdlId];
	status = GATT_ReadCharValue(connHandle, &req, selfEntity);
	return status;
}


static void EBS_updateEbsState(EbsState_t newState) {
	ebsState = newState;
	EBS_enqueueMsg(EBS_STATE_CHANGE_EVT, newState, NULL);
}

static void EBS_stateChange(EbsState_t newState) {
	switch (newState) {
		case EBS_STATE_INIT:
			uout0("ebsState = EBS_STATE_INIT");

			break;

		case EBS_STATE_DISCOVERY:
			uout0("ebsState = EBS_STATE_DISCOVERY");
			EBS_discoverDevices();
			break;

		case EBS_STATE_UPLOAD:
			//TODO: send the discTxList to BC using UART
			uout0("ebsState = EBS_STATE_UPLOAD");

			break;

		case EBS_STATE_POLLING:
			uout0("ebsState = EBS_STATE_POLLING");
			Semaphore_post(targetConnSem); // enable target connect

			break;

		default:
			break;
	}
}


static void EBS_updatePollState(uint8_t targetIndex, EbsPollState_t newState) {
	if (ebsState != EBS_STATE_POLLING)
		return;
	targetList[targetIndex].state = newState;
	uint8_t rsp = 0xFF;
	switch (newState) {
		case EBS_POLL_STATE_IDLE:
			break;

		case EBS_POLL_STATE_CONNECT:
			// TODO: need a lookup process if using parallel connections
			Semaphore_pend(targetConnSem, -1); // waiting for a vacant conn slot
			// findNextVacantSlot(pVacantSlot) // TODO: find a vacant target connection slot
			pConnectingSlot = targetList + targetIndex;
			Util_startClock(&connectingClock);
			GAPCentralRole_EstablishLink(LINK_HIGH_DUTY_CYCLE, LINK_WHITE_LIST,
					targetList[targetIndex].addrType, targetList[targetIndex].addr);
			break;

		case EBS_POLL_STATE_READ:
			Semaphore_post(targetConnSem); // release the sem to allow next connect
			pConnectingSlot = NULL;
			EBS_readCharbyHandle(targetList[targetIndex].connHdl, EVRSPROFILE_DATA);
			// TODO: upload the data to EBC
			break;

		case EBS_POLL_STATE_WRITE: // finish read
			uout0("into write process");
			EBS_writeCharbyHandle(targetList[targetIndex].connHdl, EVRSPROFILE_DATA, &rsp, 1);

			break;

		case EBS_POLL_STATE_TERMINATE: // finish write
			GAPCentralRole_TerminateLink(targetList[targetIndex].connHdl);
			break;

		default:
			break;
	}
}

/*********************************************************************
 * @fn      EBS_handleKeys
 *
 * @brief   Handles all key events for this device.
 *
 * @param   shift - true if in shift/alt.
 * @param   keys - bit field for key events. Valid entries:
 *                 HAL_KEY_SW_2
 *                 HAL_KEY_SW_1
 *
 * @return  none
 */
static void EBS_handleKeys(uint8_t shift, uint8_t keys) {
	switch (ebsState) {
		case EBS_STATE_INIT:
			// TODO: pretend to receive a uart_ack
			if (keys & KEY_RIGHT)
				EBS_updateEbsState(EBS_STATE_DISCOVERY);
			break;

		//case EBS_STATE_DISCOVERY:
		case EBS_STATE_UPLOAD:
			// TODO: pretend to receive a uart_ack
			if (keys & KEY_LEFT) {
				EBS_updateTargetList(discTxList[0].txDevID);
				EBS_updateEbsState(EBS_STATE_POLLING);
			}
			break;

		case EBS_STATE_POLLING:
			if (keys & KEY_RIGHT) {
				EBS_updatePollState(0,EBS_POLL_STATE_CONNECT);
			} else if (keys & KEY_LEFT) {
				EBS_updateTargetList(discTxList[1].txDevID);
				EBS_updatePollState(0,EBS_POLL_STATE_CONNECT);
			}

	}
}


static void EBS_updateTargetList(uint8_t* txID) {
	uint8_t index;
	for (index = 0; index < scanRes; index++)
		if (memcmp(discTxList[index].txDevID, txID, ETX_DEVID_LEN) == NULL)
			break;
	memcpy(pVacantSlot->addr, discTxList[index].addr, B_ADDR_LEN);
	memcpy(pVacantSlot->txDevID, discTxList[index].txDevID, ETX_DEVID_LEN);
	pVacantSlot->addrType = discTxList[index].addrType;
	// TODO: need a targetList manager to find next vacant

}




/*	switch (state)
	{
		case BLE_STATE_IDLE:
			//Display_print0(dispHdl, ROW_STATE, 0, "BLE_STATE_IDLE");
			if (keys & KEY_RIGHT)
			{
				// Discover devices
				EBS_discoverDevices();
			}
			//If LEFT is pressed, nothing happens.
			break;

		case BLE_STATE_DISCOVERED:
			//Display_print0(dispHdl, ROW_STATE, 0, "BLE_STATE_DISCOVERED");
			if (keys & KEY_LEFT)
			{
				//Display Discovery Results
				if (!scanningStarted && scanRes > 0)
				{
					if (scanIdx >= scanRes)
					{
						Display_clearLines(dispHdl, ROW_TWO, ROW_SEVEN);
						Display_print0(dispHdl, ROW_SIX, 0,
								"<LEFT to browse");
						Display_print0(dispHdl, ROW_SEVEN, 0,
								">RIGHT to scan");

						state = BLE_STATE_BROWSING;
						scanIdx = 0;
					} else
					{
						Display_print1(dispHdl, ROW_ONE, 0, "Device %d",
								(scanIdx + 1));
						Display_print0(dispHdl, ROW_TWO, 0,
								Util_convertBdAddr2Str(discTxList[scanIdx].addr));
						Display_print1(dispHdl, ROW_THREE, 0, "Tx ID 0x%08x",
								EBS_parseDevID(discTxList[scanIdx].txDevID));
						Display_print0(dispHdl, ROW_SEVEN, 0,
								">RIGHT to connect");

						state = BLE_STATE_BROWSING;
						scanIdx++;
					}
				}
				return;
			} else if (keys & KEY_RIGHT)
			{
				//Start scanning
				EBS_discoverDevices();
			}
			break;

		case BLE_STATE_BROWSING:
			//Display_print0(dispHdl, ROW_STATE, 0, "BLE_STATE_BROWSING");
			if (keys & KEY_LEFT)
			{
				//Navigate through discovery results
				if (!scanningStarted && scanRes > 0)
				{
					if (scanIdx >= scanRes)
					{
						//Display the scan option
						Display_clearLines(dispHdl, ROW_ONE, ROW_SEVEN);
						Display_print1(dispHdl, ROW_ONE, 0,
								"Devices found %d", scanRes);
						Display_print0(dispHdl, ROW_SIX, 0,
								"<LEFT to browse");
						Display_print0(dispHdl, ROW_SEVEN, 0,
								">RIGHT to scan");

						state = BLE_STATE_BROWSING;
						scanIdx = 0;
					} else
					{
						//Display next device
						Display_print1(dispHdl, ROW_ONE, 0, "Device %d",
								(scanIdx + 1));
						Display_print0(dispHdl, ROW_TWO, 0,
								Util_convertBdAddr2Str(discTxList[scanIdx].addr));
						Display_print1(dispHdl, ROW_THREE, 0, "Tx ID 0x%08x",
								EBS_parseDevID(discTxList[scanIdx].txDevID));
						Display_print0(dispHdl, ROW_SEVEN, 0,
								">RIGHT to connect");

						state = BLE_STATE_BROWSING;
						scanIdx++;
					}
				}
			} else if (keys & KEY_RIGHT)
			{
				//Scan for devices if the scan option is displayed
				if (scanIdx == 0)
				{
					EBS_discoverDevices();
				}

				//Connect to displayed device
				else
				{
					uint8_t addrType;
					uint8_t *peerAddr;
					if (scanRes > 0 && state == BLE_STATE_BROWSING)
					{
						// connect to current device in scan result
						peerAddr = discTxList[scanIdx - 1].addr;
						addrType = discTxList[scanIdx - 1].addrType;

						state = BLE_STATE_CONNECTING;

						Util_startClock(&connectingClock);

						GAPCentralRole_EstablishLink(LINK_HIGH_DUTY_CYCLE,
						DEFAULT_LINK_WHITE_LIST, addrType, peerAddr);

						Display_clearLines(dispHdl, ROW_FOUR, ROW_SEVEN);
						Display_print0(dispHdl, ROW_TWO, 0,
								Util_convertBdAddr2Str(peerAddr));
						Display_print0(dispHdl, ROW_FOUR, 0, "Connecting");
					}
				}
			}
			break;

		case BLE_STATE_CONNECTING:
			//Display_print0(dispHdl, ROW_STATE, 0, "BLE_STATE_CONNECTING");
			//Nothing happens if buttons are pressed while the device is connecting.
			break;

		case BLE_STATE_CONNECTED:
			//Display_print0(dispHdl, ROW_STATE, 0, "BLE_STATE_CONNECTED");
			if (keys & KEY_LEFT) //Navigate though menu.
			{
				//Iterate through rows
				switch (selectedMenuItem)
				{
					case MENU_ITEM_CONN_PARAM_UPDATE:
						selectedMenuItem = MENU_ITEM_RSSI;
						if (EBS_RssiFind(connHandle) == NULL)
						{
							Display_print0(dispHdl, ROW_SEVEN, 0,
									">Start RSSI poll");
						} else
						{
							Display_print0(dispHdl, ROW_SEVEN, 0,
									">Stop RSSI poll");
						}
						break;

					case MENU_ITEM_RSSI:
						selectedMenuItem = MENU_ITEM_READ_WRITE;
						Display_print0(dispHdl, ROW_SEVEN, 0,
								">Read/write req");
						break;

					case MENU_ITEM_READ_WRITE:
						selectedMenuItem = MENU_ITEM_DISCONNECT;
						Display_print0(dispHdl, ROW_SEVEN, 0, ">Disconnect");
						break;

					case MENU_ITEM_DISCONNECT:
						selectedMenuItem = MENU_ITEM_CONN_PARAM_UPDATE;
						Display_print0(dispHdl, ROW_SEVEN, 0,
								">Param upd req");
						break;
				}
			}
			if (keys & KEY_RIGHT)
			{
				switch (selectedMenuItem)
				{
					case MENU_ITEM_CONN_PARAM_UPDATE:
						//Connection Parameter Update
						Display_print0(dispHdl, ROW_FOUR, 0,
								"Param upd req");
						switch (currentConnectionParameter)
						{
							case INITIAL_PARAMETERS:
								GAPCentralRole_UpdateLink(connHandle,
								DEFAULT_UPDATE_MIN_CONN_INTERVAL,
								DEFAULT_UPDATE_MAX_CONN_INTERVAL,
								DEFAULT_UPDATE_SLAVE_LATENCY,
								DEFAULT_UPDATE_CONN_TIMEOUT);
								currentConnectionParameter =
										DEFAULT_UPDATE_PARAMETERS;
								break;
							case DEFAULT_UPDATE_PARAMETERS:
								GAPCentralRole_UpdateLink(connHandle,
								INITIAL_MIN_CONN_INTERVAL,
								INITIAL_MAX_CONN_INTERVAL,
								INITIAL_SLAVE_LATENCY,
								INITIAL_CONN_TIMEOUT);
								currentConnectionParameter = INITIAL_PARAMETERS;
								break;
						}
						break;

					case MENU_ITEM_RSSI:
						// Start or cancel RSSI polling
						if (EBS_RssiFind(connHandle) == NULL)
						{
							Display_clearLine(dispHdl, ROW_FIVE);
							EBS_StartRssi(connHandle,
							DEFAULT_RSSI_PERIOD);
							Display_print0(dispHdl, ROW_SEVEN, 0,
									">Stop RSSI poll");
						} else
						{
							EBS_CancelRssi(connHandle);
							Display_print0(dispHdl, ROW_FIVE, 0,
									"RSSI Cancelled");
							if (selectedMenuItem == MENU_ITEM_RSSI)
							{
								Display_print0(dispHdl, ROW_SEVEN, 0,
										">Start RSSI poll");
							}
						}
						break;

					case MENU_ITEM_READ_WRITE:
						if (state == BLE_STATE_CONNECTED&&
						charHdl != 0 &&
						procedureInProgress == FALSE)
						{
							uint8_t status;
							// Do a read or write as long as no other read or write is in progress
							if (doWrite)
							{
								// Do a write
								attWriteReq_t req;
								req.pValue = GATT_bm_alloc(connHandle,
								ATT_WRITE_REQ, 1, NULL);
								if (req.pValue != NULL)
								{
									Display_print0(dispHdl, ROW_SIX, 0,
											"Write req sent");
									req.handle = charHdl[EVRSPROFILE_DATA];
									req.len = 1;
									req.pValue[0] = 0xaf;
									req.sig = 0;
									req.cmd = 0;
									status = GATT_WriteCharValue(connHandle,
											&req, selfEntity);
									//Display_print1(dispHdl, 9, 0, "0x%04x",req.handle);
									if (status != SUCCESS)
									{
										GATT_bm_free((gattMsg_t *) &req,
										ATT_WRITE_REQ);
									}
								} else
								{
									status = bleMemAllocError;
								}
							} else
							{
								// Do a read
								attReadReq_t req;
								req.handle = charHdl[EVRSPROFILE_DATA];
								status = GATT_ReadCharValue(connHandle, &req,
										selfEntity);
								Display_print0(dispHdl, ROW_SIX, 0,
										"Read req sent");
							}

							if (status == SUCCESS)
							{
								procedureInProgress = TRUE;
								doWrite = !doWrite;
							}
						}
						break;

					case MENU_ITEM_DISCONNECT:
						GAPCentralRole_TerminateLink(connHandle);
						state = BLE_STATE_DISCONNECTING;
						Display_clearLines(dispHdl, ROW_ONE, ROW_SEVEN);
						Display_print0(dispHdl, ROW_ONE, 0, "Disconnecting");
						break;
				}
			}
	}
	return;
}
*/
