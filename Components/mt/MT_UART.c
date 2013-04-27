/***************************************************************************************************
  Filename:       MT_UART.c
  Revised:        $Date: 2009-03-12 16:25:22 -0700 (Thu, 12 Mar 2009) $
  Revision:       $Revision: 19404 $

  Description:  This module handles anything dealing with the serial port.

  Copyright 2007 Texas Instruments Incorporated. All rights reserved.

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
  PROVIDED �AS IS?WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
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

***************************************************************************************************/

/***************************************************************************************************
 * INCLUDES
 ***************************************************************************************************/
#include "ZComDef.h"
#include "OSAL.h"
#include "hal_uart.h"
#include "MT.h"
#include "MT_UART.h"
#include "OSAL_Memory.h"

#include "GenericApp.h"
/***************************************************************************************************
 * MACROS
 ***************************************************************************************************/

/***************************************************************************************************
 * CONSTANTS
 ***************************************************************************************************/
/* State values for ZTool protocal */
#define SOP_STATE      0x00
#define CMD_STATE1     0x01
#define CMD_STATE2     0x02
#define LEN_STATE      0x03
#define DATA_STATE     0x04
#define FCS_STATE      0x05

/***************************************************************************************************
 *                                         GLOBAL VARIABLES
 ***************************************************************************************************/
/* Used to indentify the application ID for osal task */
byte App_TaskID;

/* ZTool protocal parameters */
uint8 state;
uint8  CMD_Token[2];
uint8  LEN_Token;
uint8  FSC_Token;
mtOSALSerialData_t  *pMsg;
uint8  tempDataLen;

#if defined (ZAPP_P1) || defined (ZAPP_P2)
uint16  MT_UartMaxZAppBufLen;
bool    MT_UartZAppRxStatus;
#endif


/***************************************************************************************************
 *                                          LOCAL FUNCTIONS
 ***************************************************************************************************/

/***************************************************************************************************
 * @fn      MT_UartInit
 *
 * @brief   Initialize MT with UART support
 *
 * @param   None
 *
 * @return  None
***************************************************************************************************/
void MT_UartInit ()
{
  halUARTCfg_t uartConfig;

  /* Initialize APP ID */
  App_TaskID = 0;

  /* UART Configuration */
  uartConfig.configured           = TRUE;
  uartConfig.baudRate             = MT_UART_DEFAULT_BAUDRATE;
  uartConfig.flowControl          = MT_UART_DEFAULT_OVERFLOW;
  uartConfig.flowControlThreshold = MT_UART_DEFAULT_THRESHOLD;
  uartConfig.rx.maxBufSize        = MT_UART_DEFAULT_MAX_RX_BUFF;
  uartConfig.tx.maxBufSize        = MT_UART_DEFAULT_MAX_TX_BUFF;
  uartConfig.idleTimeout          = MT_UART_DEFAULT_IDLE_TIMEOUT;
  uartConfig.intEnable            = TRUE;
#if defined (ZTOOL_P1) || defined (ZTOOL_P2)
  uartConfig.callBackFunc         = MT_UartProcessZToolData;
#elif defined (ZAPP_P1) || defined (ZAPP_P2)
  uartConfig.callBackFunc         = MT_UartProcessZAppData;
#else
  uartConfig.callBackFunc         = NULL;
#endif

  /* Start UART */
#if defined (MT_UART_DEFAULT_PORT)
  HalUARTOpen (MT_UART_DEFAULT_PORT, &uartConfig);
#else
  /* Silence IAR compiler warning */
  (void)uartConfig;
#endif

  /* Initialize for ZApp */
#if defined (ZAPP_P1) || defined (ZAPP_P2)
  /* Default max bytes that ZAPP can take */
  MT_UartMaxZAppBufLen  = 1;
  MT_UartZAppRxStatus   = MT_UART_ZAPP_RX_READY;
#endif

}

/***************************************************************************************************
 * @fn      MT_SerialRegisterTaskID
 *
 * @brief   This function registers the taskID of the application so it knows
 *          where to send the messages whent they come in.
 *
 * @param   void
 *
 * @return  void
 ***************************************************************************************************/
void MT_UartRegisterTaskID( byte taskID )
{
  App_TaskID = taskID;
}

/***************************************************************************************************
 * @fn      SPIMgr_CalcFCS
 *
 * @brief   Calculate the FCS of a message buffer by XOR'ing each byte.
 *          Remember to NOT include SOP and FCS fields, so start at the CMD field.
 *
 * @param   byte *msg_ptr - message pointer
 * @param   byte len - length (in bytes) of message
 *
 * @return  result byte
 ***************************************************************************************************/
byte MT_UartCalcFCS( uint8 *msg_ptr, uint8 len )
{
  byte x;
  byte xorResult;

  xorResult = 0;

  for ( x = 0; x < len; x++, msg_ptr++ )
    xorResult = xorResult ^ *msg_ptr;

  return ( xorResult );
}


/***************************************************************************************************
 * @fn      MT_UartProcessZToolData
 *
 * @brief   | SOP | Data Length  |   CMD   |   Data   |  FCS  |
 *          |  1  |     1        |    2    |  0-Len   |   1   |
 *
 *          Parses the data and determine either is SPI or just simply serial data
 *          then send the data to correct place (MT or APP)
 *
 * @param   port     - UART port
 *          event    - Event that causes the callback
 *
 *
 * @return  None
 ***************************************************************************************************/
#if defined START_ROUTER
extern uint8 reg_len;
extern uint8 len_check_flag;
extern uint8 GenericApp_TaskID;
#endif

void MT_UartProcessZToolData ( uint8 port, uint8 event )
{
  uint8  ch;
  
  (void)event;  // Intentionally unreferenced parameter

  while (Hal_UART_RxBufLen(port))
  {
    // while length of RX buffer is not none, read one byte and put into global variable TxBuffer
    HalUARTRead (port, &ch, 1);
    TxBuffer.buf[TxBuffer.size++] = ch;      
  }
  
#if defined START_ROUTER
  // if start device as a router, check length of buffer, once size bigger than 3, send the event to user task
  if ( len_check_flag )
  {
    if( TxBuffer.size >= (reg_len*2 + 5))
    {
        osal_set_event( GenericApp_TaskID, TX_MSG_EVENT );
    }
  }
  else
    osal_set_event( GenericApp_TaskID, TX_MSG_EVENT );
#endif
}

#if defined (ZAPP_P1) || defined (ZAPP_P2)
/***************************************************************************************************
 * @fn      MT_UartProcessZAppData
 *
 * @brief   | SOP | CMD  |   Data Length   | FSC  |
 *          |  1  |  2   |       1         |  1   |
 *
 *          Parses the data and determine either is SPI or just simply serial data
 *          then send the data to correct place (MT or APP)
 *
 * @param   port    - UART port
 *          event   - Event that causes the callback
 *
 *
 * @return  None
 ***************************************************************************************************/
void MT_UartProcessZAppData ( uint8 port, uint8 event )
{

  osal_event_hdr_t  *msg_ptr;
  uint16 length = 0;
  uint16 rxBufLen  = Hal_UART_RxBufLen(MT_UART_DEFAULT_PORT);

  /*
     If maxZAppBufferLength is 0 or larger than current length
     the entire length of the current buffer is returned.
  */
  if ((MT_UartMaxZAppBufLen != 0) && (MT_UartMaxZAppBufLen <= rxBufLen))
  {
    length = MT_UartMaxZAppBufLen;
  }
  else
  {
    length = rxBufLen;
  }

  /* Verify events */
  if (event == HAL_UART_TX_FULL)
  {
    // Do something when TX if full
    return;
  }

  if (event & ( HAL_UART_RX_FULL | HAL_UART_RX_ABOUT_FULL | HAL_UART_RX_TIMEOUT))
  {
    if ( App_TaskID )
    {
      /*
         If Application is ready to receive and there is something
         in the Rx buffer then send it up
      */
      if ((MT_UartZAppRxStatus == MT_UART_ZAPP_RX_READY ) && (length != 0))
      {
        /* Disable App flow control until it processes the current data */
         MT_UartAppFlowControl (MT_UART_ZAPP_RX_NOT_READY);

        /* 2 more bytes are added, 1 for CMD type, other for length */
        msg_ptr = (osal_event_hdr_t *)osal_msg_allocate( length + sizeof(osal_event_hdr_t) );
        if ( msg_ptr )
        {
          msg_ptr->event = SPI_INCOMING_ZAPP_DATA;
          msg_ptr->status = length;

          /* Read the data of Rx buffer */
          HalUARTRead( MT_UART_DEFAULT_PORT, (uint8 *)(msg_ptr + 1), length );

          /* Send the raw data to application...or where ever */
          osal_msg_send( App_TaskID, (uint8 *)msg_ptr );
        }
      }
    }
  }
}

/***************************************************************************************************
 * @fn      SPIMgr_ZAppBufferLengthRegister
 *
 * @brief
 *
 * @param   maxLen - Max Length that the application wants at a time
 *
 * @return  None
 *
 ***************************************************************************************************/
void MT_UartZAppBufferLengthRegister ( uint16 maxLen )
{
  /* If the maxLen is larger than the RX buff, something is not right */
  if (maxLen <= MT_UART_DEFAULT_MAX_RX_BUFF)
    MT_UartMaxZAppBufLen = maxLen;
  else
    MT_UartMaxZAppBufLen = 1; /* default is 1 byte */
}

/***************************************************************************************************
 * @fn      SPIMgr_AppFlowControl
 *
 * @brief
 *
 * @param   status - ready to send or not
 *
 * @return  None
 *
 ***************************************************************************************************/
void MT_UartAppFlowControl ( bool status )
{

  /* Make sure only update if needed */
  if (status != MT_UartZAppRxStatus )
  {
    MT_UartZAppRxStatus = status;
  }

  /* App is ready to read again, ProcessZAppData have to be triggered too */
  if (status == MT_UART_ZAPP_RX_READY)
  {
    MT_UartProcessZAppData (MT_UART_DEFAULT_PORT, HAL_UART_RX_TIMEOUT );
  }

}

#endif //ZAPP

/***************************************************************************************************
***************************************************************************************************/
