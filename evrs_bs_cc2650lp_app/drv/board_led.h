/*****************************************************************************

file	board_led.h

brief	This file contains the CC2650 LaunchXL LED service definitions and 
		prototypes

proj	EVRS

date	1023pm 29 Jul 2018

author	Ziyi

*****************************************************************************/

#ifndef BOARD_LED_H
#define BOARD_LED_H

#ifdef __cplusplus
extern "C"
{
#endif

/*****************************************************************************
 * Typedefs 
 */

/** LED Identifications **/
typedef enum
{
    BOARD_LED_ID_R,
    BOARD_LED_ID_G
} boardLedId_t;


/** LED States */
typedef enum
{
    BOARD_LED_STATE_OFF,
    BOARD_LED_STATE_ON,
    BOARD_LED_STATE_FLASH
} boardLedState_t;


/*****************************************************************************
 * @fn      Board_initLEDs
 *
 * @brief   Initial the LED services.
 *
 * @param   void
 *
 * @return  none
 */
void Board_initLEDs(void);

/*****************************************************************************
 * @fn      Board_ledControl
 *
 * @brief   Change the lED state between off, on and flashing
 *
 * @param   boardLedId_t ledId to indicate the LED being control
 			boardLedState_t state to indicate the state turning to
 			uint32_t period was used if the state is flashing
 *
 * @return  none
 */
void Board_ledControl(boardLedId_t ledId, boardLedState_t state, uint32_t period);




#ifdef __cplusplus
}
#endif

#endif /* BOARD_LED_H */
