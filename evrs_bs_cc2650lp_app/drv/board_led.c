/*****************************************************************************

 file	board_led.c

 brief	This file contains the functions and params to implement CC2650
 LaunchXL LED service.

 proj	EVRS

 date	1056pm 29 Jul 2018

 author	Ziyi

 *****************************************************************************/

/*********************************************************************
 * Includes
 */

#include <ti/sysbios/knl/Clock.h>
#include <ti/drivers/PIN.h>

#include "Board.h"

#include "util.h"
#include "board_led.h"

/*********************************************************************
 * Typedefs
 */
#define BOARD_LED_FLASH_PERIOD 500

#define IDPARSER(x) ((x == BOARD_LED_ID_R)?(Board_RLED):((x == BOARD_LED_ID_G)?(Board_GLED):(PIN_UNASSIGNED)))

/*
 * Application LED pin configuration table:
 *   - All LEDs board LEDs are off.
 */
PIN_Config ledPinTable[] = {
	Board_RLED | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
	Board_GLED | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
	PIN_TERMINATE };

/*********************************************************************
 * Local Varibles
 */

/* flash control timer */
static Clock_Struct ledFlashClk[2];

/* LED pin state */
static PIN_State ledPinState;

/* LED Pin Handle */
static PIN_Handle ledPinHandle;

/* LED state table */
static boardLedState_t ledState[2];

/*********************************************************************
 * Local Functions
 */
//static uint32_t Board_ledIdParser(boardLedId_t id);
static void Board_ledFlashTimeoutCB(UArg ledId);

/*********************************************************************
 * Public Functions
 */
/*****************************************************************************
 * @fn      Board_initLEDs
 *
 * @brief   Initial the LED services.
 *
 * @param   void
 *
 * @return  none
 */
void Board_initLEDs(void) {
	/* Open LED PIN driver */
	ledPinHandle = PIN_open(&ledPinState, ledPinTable);

	/* Set default value */
	PIN_setOutputValue(ledPinHandle, Board_RLED, BOARD_LED_STATE_OFF);
	PIN_setOutputValue(ledPinHandle, Board_GLED, BOARD_LED_STATE_OFF);
	ledState[BOARD_LED_ID_R] = BOARD_LED_STATE_OFF;
	ledState[BOARD_LED_ID_R] = BOARD_LED_STATE_OFF;

	/* construct clock to control the flashing */
	Util_constructClock(&ledFlashClk[BOARD_LED_ID_R], Board_ledFlashTimeoutCB,
	BOARD_LED_FLASH_PERIOD, 0, false, BOARD_LED_ID_R);
	Util_constructClock(&ledFlashClk[BOARD_LED_ID_G], Board_ledFlashTimeoutCB,
	BOARD_LED_FLASH_PERIOD, 0, false, BOARD_LED_ID_G);
}

/*****************************************************************************
 * @fn      Board_ledControl
 *
 * @brief   Change the lED state between off, on and flashing
 *
 * @param   uint32_t index is to indicate the LED being control
 boardLedState_t to indicate the state turning to
 uint32_t period is to set the flashing period
 *
 * @return  none
 */
void Board_ledControl(boardLedId_t ledId, boardLedState_t state,
		uint32_t period) {
	switch (state)
	{
		case BOARD_LED_STATE_OFF:
			PIN_setOutputValue(ledPinHandle, IDPARSER(ledId), 0);
			if (ledState[ledId] == BOARD_LED_STATE_FLASH) Util_stopClock(
					&ledFlashClk[ledId]);
			break;

		case BOARD_LED_STATE_ON:
			PIN_setOutputValue(ledPinHandle, IDPARSER(ledId), 1);
			if (ledState[ledId] == BOARD_LED_STATE_FLASH) Util_stopClock(
					&ledFlashClk[ledId]);
			break;

		case BOARD_LED_STATE_FLASH:
			PIN_setOutputValue(ledPinHandle, IDPARSER(ledId), 0);
			Util_restartClock(&ledFlashClk[ledId], period);
			break;

		default:
			break;
	}
	ledState[ledId] = state;
}

/*
 static uint32_t Board_ledIdParser(boardLedId_t ledId)
 {
 	 switch(ledId)
 	 {
		 case BOARD_LED_ID_R:
		 return Board_RLED;

		 case BOARD_LED_ID_G:
		 return Board_GLED;

		 default:
		 return PIN_UNASSIGNED;
 	 }
 }
 */
static void Board_ledFlashTimeoutCB(UArg ledId) {
	PIN_setOutputValue(ledPinHandle, IDPARSER((boardLedId_t ) ledId),
			!PIN_getOutputValue(IDPARSER((boardLedId_t ) ledId)));

	Util_startClock(&ledFlashClk[ledId]);
}
