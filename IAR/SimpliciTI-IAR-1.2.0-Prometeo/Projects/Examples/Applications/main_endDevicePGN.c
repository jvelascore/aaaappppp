/*----------------------------------------------------------------------------
 *  Demo Application for SimpliciTI
 *
 *  L. Friedman
 *  Texas Instruments, Inc.
 *---------------------------------------------------------------------------- */

/**********************************************************************************************
  Copyright 2007-2008 Texas Instruments Incorporated. All rights reserved.

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

#include "bsp.h"
#include "mrfi.h"
#include "nwk_types.h"
#include "nwk_api.h"
#include "bsp_leds.h"
#include "bsp_buttons.h"
#include "nwk_pll.h"

#include "bsp_sensors.h"
#include "bsp_digio.h"

static void linkTo(void);

void toggleLED(uint8_t);

/* Defines by Lucanu*/

#define NUMBER_OF_SENSORS 3 //Number of sensors the node has
#define REST_BYTES 2  // command + num sensors

/*Function headers by Lucanu*/

void initNode(uint8_t *description);
void rf_processMessage(uint8_t code, uint8_t length);
void rf_createMessage(uint8_t code, uint8_t *data, uint8_t datalength);

static uint8_t description[NUMBER_OF_SENSORS + REST_BYTES];

/* Global variables by Lucanu*/

static uint8_t rf_txMessage[MAX_APP_PAYLOAD];


/* Callback handler */
static uint8_t sRxCallback(linkID_t);

static volatile uint8_t  sPeerFrameSem = 0;
static          linkID_t sLinkID1 = 0;  /*  Access Point Link ID*/
static volatile uint8_t  sSemaphore = 0;

#define SPIN_ABOUT_A_SECOND   NWK_DELAY(1000)
#define SPIN_ABOUT_A_QUARTER_SECOND   NWK_DELAY(250)

/* How many times to try a Tx and miss an acknowledge before doing a scan */
#define MISSES_IN_A_ROW  2

void main (void)
{
  BSP_Init();
  /*  Initialize node's description */
  initNode(description);

  /* If an on-the-fly device address is generated it must be done before the
   * call to SMPL_Init(). If the address is set here the ROM value will not
   * be used. If SMPL_Init() runs before this IOCTL is used the IOCTL call
   * will not take effect. One shot only. The IOCTL call below is conformal.
   */
#ifdef I_WANT_TO_CHANGE_DEFAULT_ROM_DEVICE_ADDRESS_PSEUDO_CODE
  {
    addr_t lAddr;

    createRandomAddress(&lAddr);
    SMPL_Ioctl(IOCTL_OBJ_ADDR, IOCTL_ACT_SET, &lAddr);
  }
#endif /* I_WANT_TO_CHANGE_DEFAULT_ROM_DEVICE_ADDRESS_PSEUDO_CODE */

  /* Keep trying to join (a side effect of successful initialization) until
   * successful. Toggle LEDS to indicate that joining has not occurred.
   */
  while (SMPL_SUCCESS != SMPL_Init(sRxCallback))
  {
    toggleLED(1);
    toggleLED(2);
    SPIN_ABOUT_A_SECOND; /* calls nwk_pllBackgrounder for us */
  }

  /* LEDs on solid to indicate successful join. */
  if (!BSP_LED2_IS_ON())
  {
    toggleLED(2);
  }
  if (!BSP_LED1_IS_ON())
  {
    toggleLED(1);
  }


  /* Unconditional link to AP which is listening due to successful join. */
  linkTo();

  while (1)
    FHSS_ACTIVE( nwk_pllBackgrounder( false ) );
}



static void linkTo()
{


  /* Keep trying to link... */
  while (SMPL_SUCCESS != SMPL_Link(&sLinkID1))
  {
    toggleLED(1);
    toggleLED(2);
    SPIN_ABOUT_A_SECOND; /* calls nwk_pllBackgrounder for us */
  }

  /* we're linked. turn off red LED. received messages will toggle the green LED. */
  if (BSP_LED2_IS_ON())
  {
    toggleLED(2);
  }
  if (!BSP_LED1_IS_ON())
  {
    toggleLED(1);
  }

  /* turn on RX. default is RX off. */
  SMPL_Ioctl( IOCTL_OBJ_RADIO, IOCTL_ACT_RADIO_RXON, 0);


  while (1)
  {

  if(sSemaphore>0){

    sSemaphore--;
    rf_processMessage(sSemaphore,1);
    sSemaphore = 0;

    }

    
  }
}


void toggleLED(uint8_t which)
{
  if (1 == which)
  {
    BSP_TOGGLE_LED1();
  }
  else if (2 == which)
  {
    BSP_TOGGLE_LED2();
  }
  return;
}


/* handle received messages */
static uint8_t sRxCallback(linkID_t port)
{
  uint8_t msg[MAX_APP_PAYLOAD], len;
  /* is the callback for the link ID we want to handle? */
  if (port == sLinkID1)
  {


    /* yes. go get the frame. we know this call will succeed. */
     if((SMPL_SUCCESS == SMPL_Receive(sLinkID1, msg, &len)) && len){
     
      /*  Process the received frame, which is only a 1-byte command...  */
      /*  Store this byte in the flag  */
      sSemaphore = msg[0]+1;    
      
    }

    /* Post to the semaphore to let application know so it processes 
     * and sends the reply
     */




    return 1;
  }
  return 0;
}
/*
Initialize the node
*/

void initNode(uint8_t *des){
  
  // Create a vector with the type of sensors the node has. In this case:
  des[0] = 0; // command code. This is a response to command code "0" so put this command code to identify it
  des[1] = NUMBER_OF_SENSORS; // Nº sensors
  des[2] = 1; // temp
  des[3] = 2; // humidity
  des[4] = 3; // wind
}

/*
Create a message to send over RF
*/

void rf_createMessage(uint8_t code, uint8_t *data, uint8_t datalength){
  int i = 0;
  rf_txMessage[0] = code;
  for (i = 0; i < datalength; i++){
  rf_txMessage[i+1] = data[i];
  }
  SMPL_Send(sLinkID1, rf_txMessage, datalength+1);

}

/*
Process the message received from the AP
*/

void rf_processMessage(uint8_t code, uint8_t len){
  int temp, humid;
  uint8_t data[2];

  switch(code){

  case 0x0: // get Descriptor
    SMPL_Send(sLinkID1, description, sizeof(description));
      /*  Toggle LED in the message */

    if (!BSP_LED2_IS_ON())
    {
      toggleLED(2);
    }

    break;
    
  case 0x1:
    temp = SHT75_medirTemperatura();
    data[0] = (temp>>8) & 0xff; // most significant part first
    data[1] = temp & 0xff;
    // send the temperature
    rf_createMessage(code, data, sizeof(data));
    break;
    
  case 0x2:
    humid = SHT75_medirHumedad();
    data[0] = (humid>>8) & 0xff; // most significant part first
    data[1] = humid & 0xff;
   // send the humidity
    rf_createMessage(code, data, sizeof(data));
    break;
    
  case 0x3:   
    // wSpeed = readWindSpeed();
    // send the wind speed
    //rf_createMessage(messageCode, &data, sizeof(data));
    break;
    
  case 0x4:
    //rf_createMessage(messageCode, &data, sizeof(data));
    break;

  case 0x0A:

    toggleLED(1);

    break;

  case 0x0B:
    
    toggleLED(2);

    break;

    
  default:

    break;
    
  }

}