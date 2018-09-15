/****************************************
 * 
 * @filename 	board_display.h
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

#ifndef BOARDDISPLAY_H
#define BOARDDISPLAY_H

#include <stdint.h>

void Board_Display_Init();
void Board_Display_Print(uintptr_t fmt,	uintptr_t a0, uintptr_t a1, uintptr_t a2, uintptr_t a3, uintptr_t a4);

#  define uout0(fmt) \
    Board_Display_Print((uintptr_t)(fmt), 0, 0, 0, 0, 0)

#  define uout1(fmt, a0) \
    Board_Display_Print((uintptr_t)(fmt), (uintptr_t)(a0), 0, 0, 0, 0)

#  define uout2(fmt, a0, a1) \
    Board_Display_Print((uintptr_t)(fmt), (uintptr_t)(a0), (uintptr_t)(a1), 0, 0, 0)

#  define uout3(fmt, a0, a1, a2) \
    Board_Display_Print((uintptr_t)(fmt), (uintptr_t)(a0), (uintptr_t)(a1), (uintptr_t)(a2), 0, 0)

#  define uout4(fmt, a0, a1, a2, a3) \
    Board_Display_Print((uintptr_t)(fmt), (uintptr_t)(a0), (uintptr_t)(a1), (uintptr_t)(a2), (uintptr_t)(a3), 0)

#  define uout4(fmt, a0, a1, a2, a3, a4) \
    Board_Display_Print((uintptr_t)(fmt), (uintptr_t)(a0), (uintptr_t)(a1), (uintptr_t)(a2), (uintptr_t)(a3), (uintptr_t)(a4)

#endif
