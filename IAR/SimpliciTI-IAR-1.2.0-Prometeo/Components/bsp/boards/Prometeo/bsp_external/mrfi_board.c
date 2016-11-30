/**************************************************************************************************
  Filename:       sample.c
  Revised:        $Date: 2007-07-06 11:19:00 -0700 (Fri, 06 Jul 2007) $
  Revision:       $Revision: 13579 $

  Description:    Describe the purpose and contents of the file.


  Copyright 2004-2007 Texas Instruments Incorporated. All rights reserved.

  IMPORTANT: Your use of this Software is limited to those specific rights granted under
  the terms of a software license agreement between the user who downloaded the software,
  his/her employer (which must be your employer) and Texas Instruments Incorporated (the
  "License"). You may not use this Software unless you agree to abide by the terms of the
  License. The License limits your use, and you acknowledge, that the Software may not be
  modified, copied or distributed unless embedded on a Texas Instruments microcontroller
  or used solely and exclusively in conjunction with a Texas Instruments radio frequency
  transceiver, which is integrated into your product. Other than for the foregoing purpose,
  you may not use, reproduce, copy, prepare derivative works of, modify, distribute,
  perform, display or sell this Software and/or its documentation for any purpose.

  YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE PROVIDED “AS IS”
  WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY
  WARRANTY OF MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
  IN NO EVENT SHALL TEXAS INSTRUMENTS OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER CONTRACT,
  NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER LEGAL EQUITABLE
  THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES INCLUDING BUT NOT LIMITED TO ANY
  INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST
  DATA, COST OF PROCUREMENT OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY
  THIRD PARTIES (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.

  Should you have any questions regarding your right to use this Software,
  contact Texas Instruments Incorporated at www.TI.com. 
**************************************************************************************************/

/* ~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=
 *   MRFI (Minimal RF Interface)
 *   Board code file.
 *   Target : Texas Instruments EZ430-RF2500
 *            "MSP430 Wireless Development Tool"
 *   Radios : CC2500
 * ~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=
 */

/* ------------------------------------------------------------------------------------------------
 *                                           Includes
 * ------------------------------------------------------------------------------------------------
 */
#include "bsp.h"
#include "mrfi_defs.h"
#include "bsp_digio.h"  // XXX-albarc Funcionalidad de interrupciones externas


/**************************************************************************************************
 * @fn          BSP_GpioPort1Isr
 *
 * @brief       ISR de interrupciones externas del puerto 1.
 *
 * @param       -
 *
 * @return      -
 **************************************************************************************************
 */
// XXX-albarc PORTANDO A LA PLACA DE PROMETEO
//BSP_ISR_FUNCTION( BSP_GpioPort1Isr, PORT2_VECTOR )
BSP_ISR_FUNCTION( BSP_GpioPort1Isr, PORT1_VECTOR )
// XXX-albarc FIN
{
  /*
   *  This ISR is easily replaced.  The new ISR must simply
   *  include the following function call.
   */
  MRFI_GpioIsr();
  
  // XXX-albarc Nuevo código para otras interrupciones GPIO (definidas en bsp_digio)
  register uint8_t i;
  if (P1IFG)
  {
      for (i = 0; i < 8; i++)
      {
          register const uint8_t pinmask = 1 << i;
          if ((P1IFG & pinmask) && (P1IE & pinmask) && (port1_isr_tbl[i] != 0))
          {
              (*port1_isr_tbl[i])();
              P1IFG &= ~pinmask;
          }
      }
      __low_power_mode_off_on_exit();
  }
}

/**************************************************************************************************
 * @fn          BSP_GpioPort2Isr
 *
 * @brief       ISR de interrupciones externas del puerto 2.
 *
 * @param       -
 *
 * @return      -
 **************************************************************************************************
 */
BSP_ISR_FUNCTION( BSP_GpioPort2Isr, PORT2_VECTOR )
{
  register uint8_t i;
  if (P2IFG)
  {
      for (i = 0; i < 8; i++)
      {
          register const uint8_t pinmask = 1 << i;
          if ((P2IFG & pinmask) && (P2IE & pinmask) && (port2_isr_tbl[i] != 0))
          {
              (*port2_isr_tbl[i])();
              P2IFG &= ~pinmask;
          }
      }
      __low_power_mode_off_on_exit();
  }
}

/**************************************************************************************************
 *                                  Compile Time Integrity Checks
 **************************************************************************************************
 */
#include "mrfi_board_defs.h"

// XXX-albarc PORTANDO A LA PLACA DE PROMETEO
//#if ( MRFI_GDO0_INT_VECTOR != PORT2_VECTOR )
#if ( MRFI_GDO0_INT_VECTOR != PORT1_VECTOR )
// XXX-albarc FIN
#error "ERROR:  Mismatch with specified vector and actual ISR."
/*
 *  The most likely fix is to modify the vector in the above ISR.
 *  This compile time check needs updated too.
 */
#endif


/**************************************************************************************************
 */


