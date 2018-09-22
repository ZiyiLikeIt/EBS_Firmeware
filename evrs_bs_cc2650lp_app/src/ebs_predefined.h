/*****************************************************************************
 * 
 * @filepath 	/evrs_bs_cc2650lp_app/src/ebs_umsg.h
 * 
 * @project 	evrs_bs_cc2650lp_app
 * 
 * @brief 		TODO
 * 
 * @date 		22 Sep. 2018
 * 
 * @author		Ziyi@outlook.com.au
 *
 ****************************************************************************/
#ifndef EBSPREDEFINEd_H
#define EBSPREDEFINEd_H

/*****************************************************************************
 * EBS Tasks
 */
// Task configuration
#define EBS_TASK_PRIORITY                     1

#ifndef EBS_TASK_STACK_SIZE
#define EBS_TASK_STACK_SIZE                   864
#endif

/*****************************************************************************
 * EBS Event
 */
// Simple BLE Central Task Events
#define EBS_PAIRING_STATE_EVT     		0x0001
#define EBS_STATE_CHANGE_EVT          	0x0002
#define EBS_CONNECTING_TIMEOUT_EVT	  	0x0004
#define EBS_GAP_STATE_CHG_EVT			0x0008
#define EBS_UMSG_RECV_EVT				0x0010
#define EBS_CONN_DONE_EVT				0x0020

#if defined (NPI_USE_UART) && defined(NPI_ENABLE)
//events that TL will use to control the driver
#define MRDY_EVT       	      		0x0100
#define TRANSPORT_RX_EVT       		0x0200
#define TRANSPORT_TX_DONE_EVT  		0x0400
#endif //TL

/*****************************************************************************
 * BLE Constants
 */
// Max number of connections
#define MAX_CONNS		4

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

/*****************************************************************************
 * EBS Application
 */
// Application states
typedef enum EbsState_t{
	APP_STATE_INIT,
	APP_STATE_IDLE,
	APP_STATE_DISC,
	APP_STATE_POLL
} EbsState_t;

// Polling states
typedef enum EbsPollState_t{
	POLL_STATE_IDLE,
	POLL_STATE_BUSY,
	POLL_STATE_END
} EbsPollState_t;
/*****************************************************************************
 * GATT Profile
 */
// EVRS Profile Service UUID
#define EVRSPROFILE_SERV_UUID 			0xAFF0
#define EVRSPROFILE_CMD_UUID        	0xAFF2
#define EVRSPROFILE_DATA_UUID         	0xAFF4
// GATT profile ID
typedef enum ProfileId_t{
	EVRSPROFILE_CMD,
	EVRSPROFILE_DATA
} ProfileId_t;

/*****************************************************************************
 * ETX Device ID
 */
#define ETX_DEVID_LEN 			4
#define ETX_DEVID_PREFIX		0x95

/*****************************************************************************
 * UMSG
 */
#if defined (NPI_USE_UART) && defined (NPI_ENABLE)

#define UMSG_BUFF_SIZE   	16

#define EBC_UMSG_READY		0x01
#define EBS_UMSG_ACK		0x02
#define EBC_UMSG_ACK		0x04
#define EBC_UMSG_BSID		0x08
#define EBC_UMSG_POLLEN		0x10
#define EBC_UMSG_POLLDIS	0x20
#define EBS_UMSG_RSP		0x40
#define EB_UMSG_GEN			0x80

#define EBS_UMSG_ACK_LEN	4
#define EBS_UMSG_RSP_LEN	6

#endif //TL

#endif //EBSPREDEFINEd_H
