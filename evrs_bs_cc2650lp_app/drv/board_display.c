/****************************************
 * 
 * @filename 	board_display.c
 * 
 * @project 	EVRS_driver
 * 
 * @brief 		A customized uart display component
 * 
 * @date 		26 Aug. 2018
 * 
 * @author		Ziyi@outlook.com.au
 *
 ****************************************/

#include <board_display.h>
#include <ti/mw/display/Display.h>

#define NULL 0

// Display Interface
Display_Handle dispHandle = NULL;

void Board_Display_Init() {
	dispHandle = Display_open(Display_Type_UART, NULL);
	uout0("\fUART Display initialized");
}

void Board_Display_Print(uintptr_t fmt,
	uintptr_t a0, uintptr_t a1, uintptr_t a2, uintptr_t a3, uintptr_t a4) {
	
	Display_doPut5(dispHandle, 0, 0, fmt, a0, a1, a2, a3, a4);
}
