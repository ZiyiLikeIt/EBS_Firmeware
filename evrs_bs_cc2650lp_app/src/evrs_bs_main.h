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
