/**
  @headerfile:    peripheral.h
  $Date: 2014-05-22 14:49:51 -0700 (Thu, 22 May 2014) $
  $Revision: 38618 $
  @mainpage Transport Layer implementation
  Copyright (c) 2015, Texas Instruments
  All rights reserved.
  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions are met:
  1. Redistributions of source code must retain the above copyright
     notice, this list of conditions and the following disclaimer.
  2. Redistributions in binary form must reproduce the above copyright
     notice, this list of conditions and the following disclaimer in the
     documentation and/or other materials provided with the distribution.
  3. All advertising materials mentioning features or use of this software
     must display the following acknowledgement:
     This product includes software developed by the Texas Instruments.
  4. Neither the name of the Texas Instruments nor the
     names of its contributors may be used to endorse or promote products
     derived from this software without specific prior written permission.
  THIS SOFTWARE IS PROVIDED BY Texas Instruments ''AS IS'' AND ANY
  EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  DISCLAIMED. IN NO EVENT SHALL Texas Instruments BE LIABLE FOR ANY
  DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
  ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
#ifndef TL_H
#define TL_H

/*********************************************************************
 * Profile Callbacks
 */
// Callback when data needs to be parsed from TL
typedef void (*TLpacketparser_t)(void);
typedef struct
{
  TLpacketparser_t        pfnTLpacketparser;  // Called there is data to parse
} TLCBs_t;
/*********************************************************************
 * Function APIs
 */
extern void TL_handleISRevent(void);
extern void TLinit(ICall_Semaphore *pAppSem, TLCBs_t *appCallbacks,
                  uint16_t TX_DONE_EVENT, uint16_t RX_EVENT, uint16_t MRDY_EVENT
				  );
extern void TLwrite (uint8_t *buf, uint8_t len);
extern void TLread (uint8_t *buf, uint8_t len);
extern uint16_t TLgetRxBufLen (void);

#endif /* TL_H */
