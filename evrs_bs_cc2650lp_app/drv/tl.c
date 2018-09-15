/**************************************************************************************************
  Filename:       peripheral.c
  Revised:        $Date: 2015-01-23 11:01:40 -0800 (Fri, 23 Jan 2015) $
  Revision:       $Revision: 41985 $

  Description:    transport layer handling


  Copyright 2009 - 2015 Texas Instruments Incorporated. All rights reserved.

  IMPORTANT: Your use of this Software is limited to those specific rights
  granted under the terms of a software license agreement between the user
  who downloaded the software, his/her employer (which must be your employer)
  and Texas Instruments Incorporated (the "License").  You may not use this
  Software unless you agree to abide by the terms of the License. The License
  limits your use, and you acknowledge, that the Software may not be modified,
  copied or distributed unless embedded on a Texas Instruments microcontroller
  or used solely and exclusively in conjunction with a Texas Instruments radio
  frequency transceiver, which is integrated into your product.  Other than for
  the foregoing purpose, you may not use, reproduce, copy, prepare derivative
  works of, modify, distribute, perform, display or sell this Software and/or
  its documentation for any purpose.

  YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE
  PROVIDED “AS IS” WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
  INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY, TITLE,
  NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL
  TEXAS INSTRUMENTS OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER CONTRACT,
  NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER
  LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
  INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE
  OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT
  OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
  (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.

  Should you have any questions regarding your right to use this Software,
  contact Texas Instruments Incorporated at www.TI.com.
**************************************************************************************************/

/*********************************************************************
 * INCLUDES
 */
#include <ti/sysbios/knl/Semaphore.h>   
#include <ti/sysbios/family/arm/m3/Hwi.h>
#include <ti/sysbios/knl/Task.h>
   
#include <inc/npi_data.h>
#include <inc/npi_rxbuf.h>
#include <inc/npi_tl.h>

#include "tl.h"
   
/*********************************************************************
 * GLOBAL VARIABLES
 */

//event flags
uint16_t TX_DONE_ISR_EVENT_FLAG = 0;
uint16_t MRDY_ISR_EVENT_FLAG = 0;
uint16_t TRANSPORT_RX_ISR_EVENT_FLAG = 0;

/*********************************************************************
 * LOCAL VARIABLES
 */

//stores any TL-related events
static uint16_t TL_events = 0;

//event values passed in from app
static uint16_t transport_tx_done_event = 0;
static uint16_t transport_rx_event = 0;
static uint16_t mrdy_event = 0;

//pointer to application's semaphore
static ICall_Semaphore *appSem = NULL;
//pointer to app callback function
static TLCBs_t *TL_AppCBs = NULL;

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void transportTxDoneCallBack(int size);
static void transportRXCallBack(int size);
static void MRDYEventCB(int size);

/*********************************************************************
 *  FUNCTIONS DECLARATIONS
 */
// -----------------------------------------------------------------------------
//! \brief      Call back function for TX Done event from driver.
//!
//! \param[in]  size    Number of bytes transmitted.
//!
//! \return     void
// -----------------------------------------------------------------------------
static void transportTxDoneCallBack(int size)
{
  // Post the event to the task thread.
  TX_DONE_ISR_EVENT_FLAG = transport_tx_done_event;
  Semaphore_post(*appSem);
}

// -----------------------------------------------------------------------------
//! \brief      RX Callback provided to driver for RX Event (ie.Bytes
//!             received).
//!
//! \param[in]  size    Number of bytes received.
//!
//! \return     void
// -----------------------------------------------------------------------------
static void transportRXCallBack(int size)
{
  //move bytes from driver buffer to TL circular buffer
  NPIRxBuf_Read(size);
  // Post the event to the task thread
  TRANSPORT_RX_ISR_EVENT_FLAG = transport_rx_event;
  Semaphore_post(*appSem);
}

// -----------------------------------------------------------------------------
//! \brief      RX Callback provided to driver for MRDY Event
//!
//! \param[in]  size    N/A
//!
//! \return     void
// -----------------------------------------------------------------------------
static void MRDYEventCB(int size)
{
  //Post event to task thread
  MRDY_ISR_EVENT_FLAG = mrdy_event;
  Semaphore_post(*appSem);
}

/*********************************************************************
 * GLOBAL FUNCTIONS
 */
// -----------------------------------------------------------------------------
//! \brief      Perform any TL related processing of events
//!             posted from ISR's.
//!
//! \return     void
// -----------------------------------------------------------------------------
void TL_handleISRevent(void)
{
  // Capture the ISR events flags now within a critical section.  
  // We do this to avoid possible race conditions where the ISR is 
  // modifying the event mask while the task is read/writing it.
  UInt hwiKey = Hwi_disable(); 
  UInt taskKey = Task_disable();
  TL_events = TL_events | TX_DONE_ISR_EVENT_FLAG | 
    MRDY_ISR_EVENT_FLAG | TRANSPORT_RX_ISR_EVENT_FLAG;
  TX_DONE_ISR_EVENT_FLAG = 0;
  MRDY_ISR_EVENT_FLAG = 0;
  TRANSPORT_RX_ISR_EVENT_FLAG = 0;      
  Task_restore(taskKey);
  Hwi_restore(hwiKey);
  
  //handle driver events first to avoid race conditions
  //MRDY event
  if (TL_events & mrdy_event)
  {
    TL_events &= ~mrdy_event;
#ifdef POWER_SAVING
    //perform handshaking
    NPITL_handleMrdyEvent();
#endif //POWER_SAVING
  }
  
  // The Transport Layer has received some bytes
  if(TL_events & transport_rx_event)
  {
    //call application callback to parse data
    if ((TL_AppCBs) && (TL_AppCBs->pfnTLpacketparser))
    {
      TL_AppCBs->pfnTLpacketparser();
    }
    
    //continue reading if necessary
    if (NPIRxBuf_GetRxBufCount() == 0)
    {
      // No additional bytes to collect, clear the read flag.
      TL_events &= ~transport_rx_event;
    }
    else
    {
      // Additional bytes to collect, preserve the flag and repost
      // to the semaphore to read again
      Semaphore_post(*appSem);
    }
  }
  
  // The last transmission to the host has completed.
  if(TL_events & transport_tx_done_event)
  {
    // Current TX is done.
    TL_events &= ~transport_tx_done_event;
  }    
}

// -----------------------------------------------------------------------------
//! \brief      Initialize TL 
//!
//! \param[in]  pAppSem    pointer to semaphore used by the application for ICall
//! \param[in]  appCallbacks    pointer to app callback to parse received data
//! \param[in]  TX_DONE_EVENT   value of transmission done event
//! \param[in]  RX_EVENT   value of data received event
//! \param[in]  MRDY_EVENT   value of MRDY event
//!
//! \return     void
// -----------------------------------------------------------------------------
void TLinit(ICall_Semaphore *pAppSem, TLCBs_t *appCallbacks, uint16_t TX_DONE_EVENT,
            uint16_t RX_EVENT, uint16_t MRDY_EVENT)
{
  // Initialize Network Processor Interface (NPI) which in turn will init driver
  NPITL_initTL( &transportTxDoneCallBack,
               &transportRXCallBack,
               &MRDYEventCB );
  
  //store pointer to app's semaphore
  appSem = pAppSem;
  
  //store pointer to application's callback function
  TL_AppCBs = appCallbacks;
  
  //store event values passed from app so as not overlap app's events
  transport_tx_done_event = TX_DONE_EVENT;
  transport_rx_event = RX_EVENT;
  mrdy_event = MRDY_EVENT;
}

// -----------------------------------------------------------------------------
//! \brief      Wrapper to NPI write function
//!
//! \param[in]  buf    pointer to data to write
//! \param[in]  len    length of data to write
//!
//! \return     void
// -----------------------------------------------------------------------------
void TLwrite (uint8_t *buf, uint8_t len)
{
  NPITL_writeTL(buf, len);
}

// -----------------------------------------------------------------------------
//! \brief      Wrapper to NPI read function
//!
//! \param[in]  buf    pointer to place read data
//! \param[in]  len    length of data to read
//!
//! \return     void
// -----------------------------------------------------------------------------
void TLread (uint8_t *buf, uint8_t len)
{
  NPIRxBuf_ReadFromRxBuf(buf, len);
}

// -----------------------------------------------------------------------------
//! \brief      Wrapper to NPI GetRxBufLen function
//!
//! \return     uint16_t amount of bytes in the NPI circular buffer
// -----------------------------------------------------------------------------
uint16_t TLgetRxBufLen(void)
{
  return NPIRxBuf_GetRxBufCount();
}
