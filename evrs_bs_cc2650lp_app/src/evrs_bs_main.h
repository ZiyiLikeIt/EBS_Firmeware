/****************************************
 *
 * @filename 	evrs_bs_main.h
 *
 * @project 	evrs_bs_cc2650lp_app
 *
 * @brief 		external typedefs, definition and variables
 *
 * @date 		22 Aug. 2018
 *
 * @author		Ziyi@outlook.com.au
 *
 ****************************************/


#ifndef EVRSBSMAIN_H
#define EVRSBSMAIN_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */

/*********************************************************************
*  EXTERNAL VARIABLES
*/

/*********************************************************************
 * CONSTANTS
 */

/*********************************************************************
 * MACROS
 */


// Simple BLE Central Task Events
#define EBS_PAIRING_STATE_EVT     		0x0001
#define EBS_STATE_CHANGE_EVT          	0x0002
#define EBS_CONNECTING_TIMEOUT_EVT	  	0x0004
#define EBS_STACK_MSG_EVT				0x0008
#define EBS_UMSG_RECV_EVT				0x0010
#define EBS_CONN_DONE_EVT				0x0020

#if defined (NPI_USE_UART) && defined(NPI_ENABLE)
//events that TL will use to control the driver
#define MRDY_EVT       	      		0x0100
#define TRANSPORT_RX_EVT       		0x0200
#define TRANSPORT_TX_DONE_EVT  		0x0400
#endif //TL


// Max number of connections
#define MAX_NUM_BLE_CONNS		4

/*********************************************************************
 * FUNCTIONS
 */
/*
 * Task creation function for the Simple BLE Central.
 */
void EBS_createTask(void);
uint8_t EBS_enqueueMsg(uint8_t event, uint8_t status, uint8_t *pData);

/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* SIMPLEBLECENTRAL_H */
