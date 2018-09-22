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

#include "ebs_predefined.h"
#include "bcomdef.h"

#include "hci_tl.h"
#include "linkdb.h"
#include "gatt.h"
#include "gapgattserver.h"
#include "gattservapp.h"
#include "central.h"
#include "gapbondmgr.h"

#include "osal_snv.h"
#include "icall_apimsg.h"

#include "evrs_bs_main.h"
#include "util.h"
#include "board_led.h"
#include "board_display.h"

#include "ebs_conn_mgr.h"

#if defined (NPI_USE_UART) && defined (NPI_ENABLE)
#include "tl.h"
#endif //TL

#include "board.h"

#include "ble_user_config.h"

/*********************************************************************
 * TYPEDEFS
 */
// App event passed from profiles.
typedef struct EbsEvt_t {
	appEvtHdr_t hdr; // event header
	uint8_t *pData;  // event data
} EbsEvt_t;


/*********************************************************************
 * GLOBAL VARIABLES
 */



/*********************************************************************
 * EXTERNAL VARIABLES
 */
// Discovered ETX List
extern EtxInfo_t connList[MAX_CONNS];

#if defined (NPI_USE_UART) && defined (NPI_ENABLE)
//used to store data read from transport layer
extern uint8_t appRxBuf[APP_TL_BUFF_SIZE];
#endif //TL


/*********************************************************************
 * LOCAL VARIABLES
 */

// Entity ID globally used to check for source and/or destination of messages
static ICall_EntityID selfEntity;

// Semaphore globally used to post events to the application thread
static ICall_Semaphore sem;

// Clock object used to timeout connection
static Clock_Struct connectingClock;

// Queue object used for app messages
static Queue_Struct appMsg;
static Queue_Handle appMsgQueue;

// Task pending events
//static uint16_t events = 0;

// Task configuration
Task_Struct ebsTask;
Char ebsTaskStack[EBS_TASK_STACK_SIZE];

// GAP GATT Attributes
static const uint8_t attDeviceName[GAP_DEVICE_NAME_LEN] = "EVRS BaseStation";

// Number of scan results and scan result index
//static uint8_t scanRes;

// Scanning state
//static bool scanningStarted = FALSE;

// Connection handle of current connection
//static uint16_t connHandleList[MAX_NUM_BLE_CONNS] = GAP_CONNHANDLE_INIT;

// Application state
static EbsState_t ebsState = APP_STATE_INIT;


// Base Station Identifier
uint8_t BSID = 0x00;



// sem for connecting
Semaphore_Handle targetConnSem;



/*********************************************************************
 * @TAG Local Functions
 */
/** Task Functions **/
static void EBS_init(void);
static void EBS_taskFxn(UArg a0, UArg a1);

/** Internal message gen and routing **/
static void EBS_processStackMsg(ICall_Hdr *pMsg);
static void EBS_processAppMsg(EbsEvt_t *pMsg);

/** Callbacks **/
static uint8_t EBS_CB_GAPRoleStateChange(gapCentralRoleEvent_t *pEvent);
static void EBS_CB_connectingTimeout(UArg arg0);
static void EBS_CB_pairStateChange(uint16_t connHandle, uint8_t pairState,
        uint8_t status);
static void EBS_CBm_appStateChange(EbsState_t newState);

/** Event Process Service **/
static void EBS_EVT_GAPRoleChange(gapCentralRoleEvent_t *pEvent);
static void EBS_EVT_GATTMsgReceive(gattMsgEvent_t *pMsg);
static void EBS_EVT_pairStateChange(uint8_t pairState, uint8_t status);
static void EBS_EVT_appStateChange(EbsState_t newState);

/** App State Functions **/
static void EBS_Disc_processStart(void);
static void EBS_Poll_enquireStart(void);
static void EBS_Poll_enquireMsgProcess(gattMsgEvent_t *pMsg);

/** UMSG Functions **/
#if defined (NPI_USE_UART) && defined (NPI_ENABLE)
static void EBS_CB_umsgReceive(void);
static void EBS_EVT_umsgProcess(uint8_t len, uint8_t* pData)
#endif //TL

/*********************************************************************
 * @TAG register CALLBACKS
 */

// GAP Role Callbacks
static gapCentralRoleCB_t EBS_roleCB = {
		EBS_CB_GAPRoleStateChange // Event callback
		};

// Bond Manager Callbacks
static gapBondCBs_t EBS_bondCB = {
		NULL, // Passcode callback
		EBS_CB_pairStateChange	// Pairing state callback
		};

#if defined (NPI_USE_UART) && defined (NPI_ENABLE)
static TLCBs_t EBS_TLCBs = {
		EBS_CB_umsgReceive // parse data read from transport layer
		};
#endif

/*********************************************************************
 * @TAG Task Functions
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
	// ******************************************************************
	// N0 STACK API CALLS CAN OCCUR BEFORE THIS CALL TO ICall_registerApp
	// ******************************************************************
	// Register the current thread as an ICall dispatcher application
	// so that the application can send and receive messages.
	ICall_registerApp(&selfEntity, &sem);

	// Create an RTOS queue for message from profile to be sent to app.
	appMsgQueue = Util_constructQueue(&appMsg);

	// Set initial connection parameter values
	GAP_SetParamValue(TGAP_CONN_EST_INT_MIN, INITIAL_MIN_CONN_INTERVAL);
	GAP_SetParamValue(TGAP_CONN_EST_INT_MAX, INITIAL_MAX_CONN_INTERVAL);
	GAP_SetParamValue(TGAP_CONN_EST_SUPERV_TIMEOUT, INITIAL_CONN_TIMEOUT);
	GAP_SetParamValue(TGAP_CONN_EST_LATENCY, INITIAL_SLAVE_LATENCY);

	// Construct clock for connecting timeout
//	Util_constructClock(&connectingClock, EBS_timeoutConnecting,
//			DEFAULT_SCAN_DURATION, 0, false, 0);

	Board_initLEDs();
	Board_Display_Init();

	// Setup Central Profile
	{
		uint8_t maxScanRes = MAX_CONNS;

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

	Board_ledControl(BOARD_LED_ID_G, BOARD_LED_STATE_OFF, 0);
	Board_ledControl(BOARD_LED_ID_G, BOARD_LED_STATE_OFF, 0);
	//Board_ledControl(BOARD_LED_ID_G, BOARD_LED_STATE_FLASH, 300);

#if defined (NPI_USE_UART) && defined (NPI_ENABLE)
	//initialize and pass information to TL
	TLinit(&sem, &EBS_TLCBs, TRANSPORT_TX_DONE_EVT,
			TRANSPORT_RX_EVT, MRDY_EVT);
#endif //TL
}

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

#if defined (NPI_USE_UART) && defined (NPI_ENABLE)
		//TL handles driver events. this must be done first
		TL_handleISRevent();
#endif //TL

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
	}
}

/*********************************************************************
 * @TAG Internal message gen and routing
 */
/** Creates a message and puts the message in RTOS queue **/
uint8_t EBS_enqueueMsg(uint8_t event, uint8_t status, uint8_t *pData) {
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

/** ble stack event processing function **/
static void EBS_processStackMsg(ICall_Hdr *pMsg) {
	switch (pMsg->event)
	{
		case GAP_MSG_EVENT:
			EBS_EVT_GAPRoleChange((gapCentralRoleEvent_t *) pMsg);
			break;

		case GATT_MSG_EVENT:
			EBS_EVT_GATTMsgReceive((gattMsgEvent_t *) pMsg);
			break;

		default:
			break;
	}
}

/** Central application event processing function **/
static void EBS_processAppMsg(EbsEvt_t *pMsg) {
	switch (pMsg->hdr.event) {
		case EBS_GAP_STATE_CHG_EVT:
			EBS_processStackMsg((ICall_Hdr *) pMsg->pData);

			// Free the stack message
			ICall_freeMsg(pMsg->pData);
		break;

		case EBS_STATE_CHANGE_EVT:
			EBS_EVT_appStateChange((EbsState_t) pMsg->hdr.state);
		break;

			// Pairing event
		case EBS_PAIRING_STATE_EVT:
			EBS_EVT_pairStateChange(pMsg->hdr.state, *pMsg->pData);

			ICall_free(pMsg->pData);
		break;

			// Connecting to device timed out
		case EBS_CONNECTING_TIMEOUT_EVT:
			//GAPCentralRole_TerminateLink(pConnectingSlot->connHdl);
		break;

		default:
			// Do nothing.
		break;
	}
}


/*****************************************************************************
 * @TAG Callbacks
 */
/** GAP Role event **/
static uint8_t EBS_CB_GAPRoleStateChange(gapCentralRoleEvent_t *pEvent) {
    // Forward the role event to the application
    if (EBS_enqueueMsg(EBS_GAP_STATE_CHG_EVT, SUCCESS, (uint8_t *) pEvent))
        return FALSE; // App will process and free the event
    else
        return TRUE; // Caller should free the event

}

/** Post event if connecting is timed out **/
//static void EBS_CB_connectingTimeout(UArg arg0) {
//    if (pConnectingSlot != NULL)
//        EBS_enqueueMsg(EBS_CONNECTING_TIMEOUT_EVT, 0, NULL);
//}

/** pair state cb **/
static void EBS_CB_pairStateChange(uint16_t connHandle, uint8_t pairState,
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

/** app state change cb (manual) **/
static void EBS_CBm_appStateChange(EbsState_t newState) {
    EBS_enqueueMsg(EBS_STATE_CHANGE_EVT, newState, NULL);
}

/*****************************************************************************
 * @TAG Event process service
 */
/** GAP Central role event processing function **/
static void EBS_EVT_GAPRoleChange(gapCentralRoleEvent_t *pEvent) {
	switch (pEvent->gap.opcode)
	{
		case GAP_DEVICE_INIT_DONE_EVENT:
		{
			EBS_CBm_appStateChange(APP_STATE_INIT);
//			uout0("EVRS BS initialized");
//			uout0(Util_convertBdAddr2Str(pEvent->initDone.devAddr));
//			uout1("BS ID: 0x%02x", BSID);
//			Board_ledControl(BOARD_LED_ID_R, BOARD_LED_STATE_FLASH, 300);
		}
			break;

		case GAP_DEVICE_INFO_EVENT:
		{

			//Find tx device address by UUID
			if (EBS_connMgr_findSvcUuid(EVRSPROFILE_SERV_UUID,
					pEvent->deviceInfo.pEvtData, pEvent->deviceInfo.dataLen) &&
					EBS_connMgr_checkBSID(BSID,
					pEvent->deviceInfo.pEvtData, pEvent->deviceInfo.dataLen)) {
				EBS_connMgr_addAddr(pEvent->deviceInfo.addr,
						pEvent->deviceInfo.addrType);
			}

			// Check if the discovered device is already in scan results
			EtxInfo_t* pETX = EBS_connMgr_findByAddr(pEvent->deviceInfo.addr);
			if (pETX != NULL) {
					//Update deviceInfo entry with the name
					EBS_connMgr_addDeviceID(pETX,
							pEvent->deviceInfo.pEvtData,
							pEvent->deviceInfo.dataLen);
			}
		}
		break;

		case GAP_DEVICE_DISCOVERY_EVENT:
		{
			// discovery complete
//			scanningStarted = FALSE;
			// initialize scan index to first
//			scanIdx = 0;
//			uout1("%d Device(s) found", scanRes);
			EBS_CBm_appStateChange(APP_STATE_POLL);
		}
			break;

		case GAP_LINK_ESTABLISHED_EVENT:
		{
			if (pEvent->gap.hdr.status == SUCCESS)
			{
				//Connect to selected device
				//state = BLE_STATE_CONNECTED;
				//connHandle = pEvent->linkCmpl.connectionHandle;
				//procedureInProgress = TRUE;

				// If service discovery not performed initiate service discovery
//				if (charHdl[0] == 0)
//				{
//					Util_startClock(&startEnquireClock);
//				}

				//Find device ID in discTxList struct
//				uint8_t i;
//				for (i = 0; i < scanRes; i++)
//				{
//					if (memcmp(pEvent->linkCmpl.devAddr, targetList[i].addr,
//					B_ADDR_LEN) == NULL)
//					{
//						break;
//					}
//				}
				EtxInfo_t* pETX = EBS_connMgr_findByAddr(pEvent->linkCmpl.devAddr);
				if (pETX == NULL) {
					//GAPCentralRole_TerminateLink(pEvent->linkCmpl.connectionHandle);
					break;
				}
				pETX->connHdl = pEvent->linkCmpl.connectionHandle;
				uout1("Tx ID 0x%08x Connected", BUILD_UINT32(pETX->txDevID[0],
						pETX->txDevID[1],pETX->txDevID[2],pETX->txDevID[3]));
//				uout1("Tx Addr %s", Util_convertBdAddr2Str(pEvent->linkCmpl.devAddr));

			} else
			{
				//connHandle = GAP_CONNHANDLE_INIT;
				//discState = EBS_DISC_STATE_IDLE;

				uout1("Connect Failed: 0x%02x",pEvent->gap.hdr.status);
			}
		}
			break;

		case GAP_LINK_TERMINATED_EVENT:
		{
			//state = BLE_STATE_IDLE;
			//connHandle = GAP_CONNHANDLE_INIT;

			//Clear screen and display disconnect reason
			uout1("Disconnected: 0x%02x", pEvent->linkTerminate.reason);
		}
			break;

		default:
			break;
	}
}


/** Process GATT messages and events **/
static void EBS_EVT_GATTMsgReceive(gattMsgEvent_t *pMsg) {
	if (ebsState == APP_STATE_POLL) {
		EBS_Poll_enquireMsgProcess(pMsg);
	} else {
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
			}

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
			}

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
		}
	} // else - in case a GATT message came after a connection has dropped, ignore it.

	// Needed only for ATT Protocol messages
	GATT_bm_free(&pMsg->msg, pMsg->method);
}

/** Process the new paring state **/
static void EBS_EVT_pairStateChange(uint8_t pairState, uint8_t status) {
	if (pairState == GAPBOND_PAIRING_STATE_STARTED) {
		uout0("Pairing started");
	} else if (pairState == GAPBOND_PAIRING_STATE_COMPLETE) {
		if (status == SUCCESS) {
			uout0("Pairing success");
		} else {
			uout1("Pairing fail: %d", status);
		}
	} else if (pairState == GAPBOND_PAIRING_STATE_BONDED) {
		if (status == SUCCESS) {
			uout0("Bonding success");
		}
	} else if (pairState == GAPBOND_PAIRING_STATE_BOND_SAVED) {
		if (status == SUCCESS) {
			uout0("Bond save succ");
		} else {
			uout1("Bond save fail: %d", status);
		}
	}
}

/** app state change event **/
static void EBS_EVT_appStateChange(EbsState_t newState) {
    switch (newState) {
        case APP_STATE_INIT:
            uout0("ebsState = APP_STATE_INIT");

            break;

        case APP_STATE_IDLE:
            uout0("ebsState = APP_STATE_IDLE");
            break;

        case APP_STATE_DISC:
            //TODO: send the discTxList to BC using UART
            uout0("ebsState = APP_STATE_DISC");

            break;

        case APP_STATE_POLL:
            uout0("ebsState = APP_STATE_POLL");

            break;

        default:
            break;
    }
}

/*****************************************************************************
 * @TAG Functions for app states
 */
/** Start discovery process **/
static void EBS_Disc_processStart(void) {
	//Clear old scan results
	EBS_connMgr_resetList();

	uout0("Discovering...");
	GAPCentralRole_StartDiscovery(DEFAULT_DISCOVERY_MODE,
	DEFAULT_DISCOVERY_ACTIVE_SCAN,
	DEFAULT_DISCOVERY_WHITE_LIST);
}

/** Start enquire process **/
static void EBS_Poll_enquireStart(void) {

	uint8_t uuid[ATT_BT_UUID_SIZE] = {
			LO_UINT16(EVRSPROFILE_SERV_UUID),
			HI_UINT16(EVRSPROFILE_SERV_UUID) };
	// Start polling process
	for (uint8_t i = 0; i < MAX_NUM_BLE_CONNS; i++) {
		connList[i].state = POLL_STATE_BUSY;
		VOID GATT_DiscPrimaryServiceByUUID(connList[i].connHdl, uuid,
					ATT_BT_UUID_SIZE, selfEntity);
	}
}

/** Process messages when enquiring **/
static void EBS_Poll_enquireMsgProcess(gattMsgEvent_t *pMsg) {
	EtxInfo_t* pCurConn = EBS_connMgr_findByConnHdl(pMsg->connHandle);
	if (pCurConn->state == POLL_STATE_BUSY) {
		if (pMsg->method == ATT_FIND_BY_TYPE_VALUE_RSP) {
			// Service found, store handles
			if (pMsg->msg.findByTypeValueRsp.numInfo > 0) {
				pCurConn->svcStartHdl = ATT_ATTR_HANDLE(
						pMsg->msg.findByTypeValueRsp.pHandlesInfo, 0);
				pCurConn->svcEndHdl = ATT_GRP_END_HANDLE(
						pMsg->msg.findByTypeValueRsp.pHandlesInfo, 0);
			}
			// If procedure complete
			if ((pMsg->hdr.status == bleProcedureComplete)
					|| (pMsg->method == ATT_ERROR_RSP)) {
				if (pCurConn->svcStartHdl != 0) {
					// Read Data by UUID
					attAttrType_t dataUuid = {
							.len = ATT_BT_UUID_SIZE,
							.uuid[0] = LO_UINT16(EVRSPROFILE_DATA_UUID),
							.uuid[1] = HI_UINT16(EVRSPROFILE_DATA_UUID)
					};
					attReadByTypeReq_t req = {
							.startHandle = pCurConn->svcStartHdl,
							.endHandle = pCurConn->svcEndHdl,
							.type = dataUuid
					};
					VOID GATT_ReadUsingCharUUID(pCurConn->connHdl, &req, selfEntity);
				}
			}
		}
		// Read using uuid done, got response
		if (pMsg->method == ATT_READ_BY_TYPE_RSP) {
			// receive read response message
			if (pMsg->msg.readByTypeRsp.numPairs > 0) {
				pCurConn->data = pMsg->msg.readByTypeRsp.pDataList[2];
			}
			// If procedure complete
			if ((pMsg->hdr.status == bleProcedureComplete)
					|| (pMsg->method == ATT_ERROR_RSP)) {
				pCurConn->state = POLL_STATE_IDLE;
				#if defined (NPI_USE_UART) && defined (NPI_ENABLE)
				// TODO send umsg

				#endif
			}
		}
	}
}

#if defined (NPI_USE_UART) && defined (NPI_ENABLE)
/*****************************************************************************
 * UART communication
 */
/** Receive an UMSG **/
static void EBS_CB_umsgReceive(void) {
	while 1 {
		uint8_t len = TLgetRxBufLen();
		if (len < 2)
			break;
		uint8_t umsgLen = 0;
		uint8_t umsgData[UMSG_BUFF_SIZE] = {0};

		TLread(&umsgLen, 1); // read msg size
		if (umsgLen == 0 || umsgLen >= UMSG_BUFF_SIZE) // size verify
			break;
		TLread(umsgData, umsgLen-1); // read rest of msg

		uint8_t *pData; // enqueue the msg
		if (pData = ICall_malloc(umsgLen - 1))
			memcpy(pData, umsgData, umsgLen - 1);
		EBS_enqueueMsg(EBS_UMSG_RECV_EVT, umsgLen, pData);
	}
}

/** Processing the UMSG **/
static void EBS_EVT_umsgProcess(uint8_t len, uint8_t* pData) {
	uint8_t opcode = *pData;
	switch (opcode) {
		case EBC_UMSG_READY:
			break;

	}
}
#endif // NPI_USE_UART



