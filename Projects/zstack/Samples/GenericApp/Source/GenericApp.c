/******************************************************************************
  Filename:       GenericApp.c
  Revised:        $Date: 2012-03-07 01:04:58 -0800 (Wed, 07 Mar 2012) $
  Revision:       $Revision: 29656 $

  Description:    Generic Application (no Profile).


  Copyright 2004-2012 Texas Instruments Incorporated. All rights reserved.

  IMPORTANT: Your use of this Software is limited to those specific rights
  granted under the terms of a software license agreement between the user
  who downloaded the software, his/her employer (which must be your employer)
  and Texas Instruments Incorporated (the "License"). You may not use this
  Software unless you agree to abide by the terms of the License. The License
  limits your use, and you acknowledge, that the Software may not be modified,
  copied or distributed unless embedded on a Texas Instruments microcontroller
  or used solely and exclusively in conjunction with a Texas Instruments radio
  frequency transceiver, which is integrated into your product. Other than for
  the foregoing purpose, you may not use, reproduce, copy, prepare derivative
  works of, modify, distribute, perform, display or sell this Software and/or
  its documentation for any purpose.

  YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE
  PROVIDED 揂S IS?WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
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
******************************************************************************/

/*********************************************************************
  This application isn't intended to do anything useful, it is
  intended to be a simple example of an application's structure.

  This application sends "Hello World" to another "Generic"
  application every 5 seconds.  The application will also
  receives "Hello World" packets.

  The "Hello World" messages are sent/received as MSG type message.

  This applications doesn't have a profile, so it handles everything
  directly - itself.

  Key control:
    SW1:
    SW2:  initiates end device binding
    SW3:
    SW4:  initiates a match description request
*********************************************************************/

/*********************************************************************
 * INCLUDES
 */
#include "OSAL.h"
#include "AF.h"
#include "ZDApp.h"
#include "ZDObject.h"
#include "ZDProfile.h"

#include "GenericApp.h"
#include "DebugTrace.h"

#if !defined( WIN32 )
  #include "OnBoard.h"
#endif

/* HAL */
#include "hal_lcd.h"
#include "hal_led.h"
#include "hal_key.h"
#include "hal_uart.h"
#include "MT_UART.h"

#if defined ( MODBUS_SUPPLY)
  #include "modbus.h"
#endif
/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */

/*********************************************************************
 * TYPEDEFS
 */
#if defined ( START_ROUTER)
  // signal strength list
  typedef struct signalStrength
  {
    struct signalStrength *next;
    uint8 modbus_id;
    int8 rssi;
  } signalStrength_t;
#endif

/*********************************************************************
 * GLOBAL VARIABLES
 */
// This list should be filled with Application specific Cluster IDs.
const cId_t GenericApp_ClusterList[GENERICAPP_MAX_CLUSTERS] =
{
  GENERICAPP_CLUSTERID
};

const SimpleDescriptionFormat_t GenericApp_SimpleDesc =
{
  GENERICAPP_ENDPOINT,              //  int Endpoint;
  GENERICAPP_PROFID,                //  uint16 AppProfId[2];
  GENERICAPP_DEVICEID,              //  uint16 AppDeviceId[2];
  GENERICAPP_DEVICE_VERSION,        //  int   AppDevVer:4;
  GENERICAPP_FLAGS,                 //  int   AppFlags:4;
  GENERICAPP_MAX_CLUSTERS,          //  byte  AppNumInClusters;
  (cId_t *)GenericApp_ClusterList,  //  byte *pAppInClusterList;
  GENERICAPP_MAX_CLUSTERS,          //  byte  AppNumInClusters;
  (cId_t *)GenericApp_ClusterList   //  byte *pAppInClusterList;
};

// This is the Endpoint/Interface description.  It is defined here, but
// filled-in in GenericApp_Init().  Another way to go would be to fill
// in the structure here and make it a "const" (in code space).  The
// way it's defined in this sample app it is define in RAM.
endPointDesc_t GenericApp_epDesc;

Tx_data_t TxBuffer;

#if defined (START_ROUTER)
  uint8 reg_len;
  uint8 len_check_flag = 0;
  bool ack_exist = TRUE;
  // check tstat modbus id
  uint8 ask_modbus_id[8] = { 0xff, 0x03, 0x00, 0x06, 0x00, 0x01, 0x71, 0xd5};
  uint8 tstat_id = 0;
  signalStrength_t *pSignalStren;
  uint8 numSignalStren = 0;
#else
  #ifdef PANID_DISPLAY
    bool panId_send = FALSE;
    uint8 panId_send_hi, panId_send_lo;
    uint8 rssi_send_hi, rssi_send_lo;
    uint8 rev_hi, rev_lo;
    #define TEMCO_ZIGBEE_REV  9
  #endif
  uint16 update_addr;
  
#endif
/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */
byte GenericApp_TaskID;   // Task ID for internal task/event processing
                          // This variable will be received when
                          // GenericApp_Init() is called.
devStates_t GenericApp_NwkState;


byte GenericApp_TransID;  // This is the unique message ID (counter)

afAddrType_t GenericApp_DstAddr;

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void GenericApp_ProcessZDOMsgs( zdoIncomingMsg_t *inMsg );
static void GenericApp_MessageMSGCB( afIncomingMSGPacket_t *pckt );

afStatus_t zb_SendDataRequest( uint16 destination, uint16 commandId, uint8 len,
                          uint8 *pData, uint8 handle, uint8 txOptions, uint8 radius );

void InitCRC16(void);
void CRC16_Tstat(unsigned char ch);
void calcCRC16( uint8* data, uint8 len);

#if defined (START_ROUTER)
  static Status_t register_signalStrength( uint8 modbus_id, int8 rssi);
  static signalStrength_t *findSignalStrength( uint8 modbus_id);
  static void sendAllSignalStren( void);
#endif
/*********************************************************************
 * NETWORK LAYER CALLBACKS
 */

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      GenericApp_Init
 *
 * @brief   Initialization function for the Generic App Task.
 *          This is called during initialization and should contain
 *          any application specific initialization (ie. hardware
 *          initialization/setup, table initialization, power up
 *          notificaiton ... ).
 *
 * @param   task_id - the ID assigned by OSAL.  This ID should be
 *                    used to send messages and set timers.
 *
 * @return  none
 */
void GenericApp_Init( uint8 task_id )
{
  MT_UartInit();
  GenericApp_TaskID = task_id;
  GenericApp_NwkState = DEV_INIT;
  GenericApp_TransID = 0;

  // Device hardware initialization can be added here or in main() (Zmain.c).
  // If the hardware is application specific - add it here.
  // If the hardware is other parts of the device add it in main().

  GenericApp_DstAddr.addrMode = (afAddrMode_t)AddrNotPresent;
  GenericApp_DstAddr.endPoint = 0;
  GenericApp_DstAddr.addr.shortAddr = 0;

  // Fill out the endpoint description.
  GenericApp_epDesc.endPoint = GENERICAPP_ENDPOINT;
  GenericApp_epDesc.task_id = &GenericApp_TaskID;
  GenericApp_epDesc.simpleDesc
            = (SimpleDescriptionFormat_t *)&GenericApp_SimpleDesc;
  GenericApp_epDesc.latencyReq = noLatencyReqs;

  // Register the endpoint description with the AF
  afRegister( &GenericApp_epDesc );

  // Register for all key events - This app will handle all key events
  RegisterForKeys( GenericApp_TaskID );

  // Update the display
#if defined ( LCD_SUPPORTED )
  HalLcdWriteString( "GenericApp", HAL_LCD_LINE_1 );
#endif

  ZDO_RegisterForZDOMsg( GenericApp_TaskID, End_Device_Bind_rsp );
  ZDO_RegisterForZDOMsg( GenericApp_TaskID, Match_Desc_rsp );

#if defined START_ROUTER
  //P0SEL &= ~BV(6);
  //P0DIR |= BV(6);
  //P0_6 = 1;
  pSignalStren = (signalStrength_t*)NULL;
#else
  HAL_TURN_ON_LED1();
  osal_set_event(GenericApp_TaskID, TX_MSG_EVENT);    // 用轮询的方法，发送串口消息
#endif
}

/*********************************************************************
 * @fn      GenericApp_ProcessEvent
 *
 * @brief   Generic Application Task event processor.  This function
 *          is called to process all events for the task.  Events
 *          include timers, messages and any other user defined events.
 *
 * @param   task_id  - The OSAL assigned task ID.
 * @param   events - events to process.  This is a bit map and can
 *                   contain more than one event.
 *
 * @return  none
 */
uint16 GenericApp_ProcessEvent( uint8 task_id, uint16 events )
{
  afIncomingMSGPacket_t *MSGpkt;
  afDataConfirm_t *afDataConfirm;

  // Data Confirmation message fields
  byte sentEP;
  ZStatus_t sentStatus;
  byte sentTransID;       // This should match the value sent
  (void)task_id;  // Intentionally unreferenced parameter

  if ( events & SYS_EVENT_MSG )
  {
    MSGpkt = (afIncomingMSGPacket_t *)osal_msg_receive( GenericApp_TaskID );
    while ( MSGpkt )
    {
      switch ( MSGpkt->hdr.event )
      {
        case ZDO_CB_MSG:
          GenericApp_ProcessZDOMsgs( (zdoIncomingMsg_t *)MSGpkt );
          break;

        case AF_DATA_CONFIRM_CMD:
          // This message is received as a confirmation of a data packet sent.
          // The status is of ZStatus_t type [defined in ZComDef.h]
          // The message fields are defined in AF.h
          afDataConfirm = (afDataConfirm_t *)MSGpkt;
          sentEP = afDataConfirm->endpoint;
          sentStatus = afDataConfirm->hdr.status;
          sentTransID = afDataConfirm->transID;
          (void)sentEP;
          (void)sentTransID;

          // Action taken when confirmation is received.
          if ( sentStatus != ZSuccess )
          {
            // The data wasn't delivered -- Do something
          }
          break;

        case AF_INCOMING_MSG_CMD:
          GenericApp_MessageMSGCB( MSGpkt );
          break;

        case ZDO_STATE_CHANGE:
          GenericApp_NwkState = (devStates_t)(MSGpkt->hdr.status);
          if ( (GenericApp_NwkState == DEV_ZB_COORD)
              || (GenericApp_NwkState == DEV_ROUTER)
              || (GenericApp_NwkState == DEV_END_DEVICE) )
          {
#if defined (START_ROUTER)
            osal_set_event( GenericApp_TaskID, ACK_CHECK);
            osal_set_event( GenericApp_TaskID, ASK_MODBUS_ID);
#endif
          }
          break;

        default:
          break;
      }

      // Release the memory
      osal_msg_deallocate( (uint8 *)MSGpkt );

      // Next
      MSGpkt = (afIncomingMSGPacket_t *)osal_msg_receive( GenericApp_TaskID );
    }

    // return unprocessed events
    return (events ^ SYS_EVENT_MSG);
  }
  
#if defined ( START_ROUTER)
  if ( events & ACK_CHECK)
  {
    uint8 ack_byte = 0;
    if( ack_exist == TRUE)
    {
      zb_SendDataRequest( 0, ACK_CMD_ID, 1, &ack_byte, 
                               0, AF_ACK_REQUEST, 0);
    }
    else
    {
      restore_factory_setting();
    }
    ack_exist = FALSE;
    osal_start_timerEx( GenericApp_TaskID, ACK_CHECK, 10000);   // Every minute check ack, if no receive, restart to join a new network
    return ( events ^ ACK_CHECK);
  }
  // send the command to check tstat modbus id
  if( events & ASK_MODBUS_ID)
  {
    uint8 rssi_byte = 0;
    // to read register 6 of tstat
    if( tstat_id != 0)
    {
      zb_SendDataRequest( 0xffff, RSSI_REQ_ID, 1, &rssi_byte, 
                               0, AF_ACK_REQUEST, 0);
    }
    else
    {
      HalUARTWrite ( 0, ask_modbus_id, 8 );
      len_check_flag = 0;
    }
    // if not received, send again 2 seconds later
    osal_start_timerEx( GenericApp_TaskID, ASK_MODBUS_ID, 2000);
    return ( events ^ ASK_MODBUS_ID);
  }
#endif  // __defined ( START_ROUTER)
  
  if( events & TX_MSG_EVENT)
  {
#if defined START_ROUTER
    uint16 coord_addr = 0;
    if( TxBuffer.buf[0] == 0xff)        // 用T3000扫描时要回复
    {
      // check whether a tstat modbus id response
      if( (TxBuffer.buf[1] == 0x03) && (TxBuffer.buf[2] == 0x02))
      {
        // store the tstat id
        tstat_id = TxBuffer.buf[4];
      }
      // When the first byte is 0xff, it's a scan command of T3000 software to scan all the nodes.
      zb_SendDataRequest( coord_addr, GENERICAPP_CLUSTERID, TxBuffer.size, TxBuffer.buf, 
                               0, AF_ACK_REQUEST, 0);
      TxBuffer.size = 0;
    }
    else if( (TxBuffer.buf[1] == 0x23) && (TxBuffer.buf[2] == 0x23))
    {
      send_char_Uart(0x23, 0);
      send_char_Uart(numSignalStren, 0);
      sendAllSignalStren();
      TxBuffer.size = 0;
    }
    else
    {
      zb_SendDataRequest( coord_addr, GENERICAPP_CLUSTERID, TxBuffer.size, TxBuffer.buf, 
                               0, AF_ACK_REQUEST, 0);
      TxBuffer.size = 0;
    }
#else
    if( TxBuffer.size > 5)
    {
      // the length of scan message is 5
     if( (TxBuffer.buf[0] == 0xff) && ( TxBuffer.buf[1] == 0x19))
     {
       // send a broadcast message
       zb_SendDataRequest( 0xFFFF, GENERICAPP_CLUSTERID, 6, TxBuffer.buf, 
                             0, AF_ACK_REQUEST, 0);
       TxBuffer.size = 0;
     }
     else if( (TxBuffer.buf[1] == 3) || ( TxBuffer.buf[1] == 6))
     {
       // the second byte of read command is 0x03, and write command is 0x06
       if( TxBuffer.size > 7)
       {
    // 添加发送PANid
    // PanId register need to be writen
  #ifdef PANID_DISPLAY
          if( (TxBuffer.buf[0] != 0xff) && ( TxBuffer.buf[1] == 0x03) && (TxBuffer.buf[3] <= 24))
          {
            panId_send_lo = 250;
            panId_send_hi = 250;
            
            panId_send = TRUE;
            for( uint8 i = 0; i < 24; i++)
            {
              if( i == TxBuffer.buf[3])
              {
                panId_send_hi = 45 - 2*i;
                panId_send_lo = 46 - 2*i;
                rssi_send_hi = 47 - 2*i;
                rssi_send_lo = 48 - 2*i;
                rev_hi = 49 - 2*i;
                rev_lo = 50 - 2*i;
              }
            }
          }
  #endif  // __PANID_DISPLAY
  #if defined ( USB_DONGLE)
          if( (TxBuffer.buf[0] == 0xff) || ( TxBuffer.buf[0] == modbus_id))
            modbus_uart_data_process( TxBuffer.buf, TxBuffer.size);
          else
  #endif  // __defined ( USB_DONGLE)
          {
             // the length of read command is 8
             // send a broadcast message
             zb_SendDataRequest( 0xFFFF, GENERICAPP_CLUSTERID, TxBuffer.size, TxBuffer.buf, 
                                 0, AF_ACK_REQUEST, 0);
          }
          TxBuffer.size = 0;
       }
     }
     else if( (TxBuffer.buf[1] == 0x10) && ( TxBuffer.buf[4] == 0x00) && ( TxBuffer.buf[5] == 0x80))
     {
       if( TxBuffer.size > 5)
       {
         if( TxBuffer.size >= (TxBuffer.buf[5] + 9))
         {
           zb_SendDataRequest( update_addr, GENERICAPP_CLUSTERID, TxBuffer.size, TxBuffer.buf, 
                             0, AF_ACK_REQUEST, 0);
           TxBuffer.size = 0;
         }
       }
     }
     else
     {
       zb_SendDataRequest( 0xFFFF, GENERICAPP_CLUSTERID, TxBuffer.size, TxBuffer.buf, 
                             0, AF_ACK_REQUEST, 0);
       TxBuffer.size = 0;
     }
    }
   // every 25 ms to check if there are uart msg
   osal_start_timerEx( GenericApp_TaskID, TX_MSG_EVENT, 25);    // LJ 每50毫秒轮询一次检测BUFFER里是否有消息要发出
#endif  // __defined START_ROUTER
   return ( events ^ TX_MSG_EVENT);
  }

  // Discard unknown events
  return 0;
}

/*********************************************************************
 * Event Generation Functions
 */

/*********************************************************************
 * @fn          zb_SendDataRequest
 *
 * @brief       The function initiates transmission of data
 *              to a peer device
 * 
 * @return      none
 */
afStatus_t zb_SendDataRequest( uint16 destination, uint16 commandId, uint8 len,
                          uint8 *pData, uint8 handle, uint8 txOptions, uint8 radius )
{
  afStatus_t status;
  afAddrType_t dstAddr;

  txOptions |= AF_DISCV_ROUTE;

  // Set the destination address
  if (destination == INVALID_NODE_ADDR)
  {
    // Binding
    dstAddr.addrMode = afAddrNotPresent;
  }
  /*
  else if( destination == MAC_SHORT_ADDR_BROADCAST)
  {
    dstAddr.addr.shortAddr = destination;
    dstAddr.addrMode = afAddrBroadcast;
  }*/
  else
  {
    // Use short address
    dstAddr.addr.shortAddr = destination;
    dstAddr.addrMode = afAddr16Bit;
  }

  dstAddr.panId = 0;                                    // Not an inter-pan message.
  dstAddr.endPoint = GenericApp_epDesc.simpleDesc->EndPoint;  // Set the endpoint.

  // Send the message
  status = AF_DataRequest(&dstAddr, &GenericApp_epDesc, commandId, len,
                          pData, &handle, txOptions, radius);

  return status;
}
/*********************************************************************
 * @fn      GenericApp_ProcessZDOMsgs()
 *
 * @brief   Process response messages
 *
 * @param   none
 *
 * @return  none
 */
static void GenericApp_ProcessZDOMsgs( zdoIncomingMsg_t *inMsg )
{
  switch ( inMsg->clusterID )
  {
    case End_Device_Bind_rsp:
      if ( ZDO_ParseBindRsp( inMsg ) == ZSuccess )
      {
        // Light LED
        HalLedSet( HAL_LED_4, HAL_LED_MODE_ON );
      }
#if defined( BLINK_LEDS )
      else
      {
        // Flash LED to show failure
        HalLedSet ( HAL_LED_4, HAL_LED_MODE_FLASH );
      }
#endif
      break;

    case Match_Desc_rsp:
      {
        ZDO_ActiveEndpointRsp_t *pRsp = ZDO_ParseEPListRsp( inMsg );
        if ( pRsp )
        {
          if ( pRsp->status == ZSuccess && pRsp->cnt )
          {
            GenericApp_DstAddr.addrMode = (afAddrMode_t)Addr16Bit;
            GenericApp_DstAddr.addr.shortAddr = pRsp->nwkAddr;
            // Take the first endpoint, Can be changed to search through endpoints
            GenericApp_DstAddr.endPoint = pRsp->epList[0];

            // Light LED
            HalLedSet( HAL_LED_4, HAL_LED_MODE_ON );
          }
          osal_mem_free( pRsp );
        }
      }
      break;
  }
}



/*********************************************************************
 * LOCAL FUNCTIONS
 */

/*********************************************************************
 * @fn      GenericApp_MessageMSGCB
 *
 * @brief   Data message processor callback.  This function processes
 *          any incoming data - probably from other devices.  So, based
 *          on cluster ID, perform the intended action.
 *
 * @param   none
 *
 * @return  none
 */
// ************************测试RSSI
#ifdef PANID_DISPLAY
int8 modbus_rssi;
// uint8 modbus_correlation;
unsigned char CRClo;
unsigned char CRChi;
CONST unsigned char auchCRCHi[256] = {
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,//12
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40
} ;	

CONST unsigned char auchCRCLo[256] = {

0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06, 0x07, 0xC7, 0x05, 0xC5, 0xC4, 0x04,
0xCC, 0x0C, 0x0D, 0xCD, 0x0F, 0xCF, 0xCE, 0x0E, 0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09, 0x08, 0xC8,
0xD8, 0x18, 0x19, 0xD9, 0x1B, 0xDB, 0xDA, 0x1A, 0x1E, 0xDE, 0xDF, 0x1F, 0xDD, 0x1D, 0x1C, 0xDC,
0x14, 0xD4, 0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3, 0x11, 0xD1, 0xD0, 0x10,
0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3, 0xF2, 0x32, 0x36, 0xF6, 0xF7, 0x37, 0xF5, 0x35, 0x34, 0xF4,
0x3C, 0xFC, 0xFD, 0x3D, 0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A, 0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38,
0x28, 0xE8, 0xE9, 0x29, 0xEB, 0x2B, 0x2A, 0xEA, 0xEE, 0x2E, 0x2F, 0xEF, 0x2D, 0xED, 0xEC, 0x2C,
0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26, 0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0,
0xA0, 0x60, 0x61, 0xA1, 0x63, 0xA3, 0xA2, 0x62, 0x66, 0xA6, 0xA7, 0x67, 0xA5, 0x65, 0x64, 0xA4,
0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F, 0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB, 0x69, 0xA9, 0xA8, 0x68,
0x78, 0xB8, 0xB9, 0x79, 0xBB, 0x7B, 0x7A, 0xBA, 0xBE, 0x7E, 0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C,
0xB4, 0x74, 0x75, 0xB5, 0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71, 0x70, 0xB0,//12
0x50, 0x90, 0x91, 0x51, 0x93, 0x53, 0x52, 0x92, 0x96, 0x56, 0x57, 0x97, 0x55, 0x95, 0x94, 0x54,
0x9C, 0x5C, 0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E, 0x5A, 0x9A, 0x9B, 0x5B, 0x99, 0x59, 0x58, 0x98,
0x88, 0x48, 0x49, 0x89, 0x4B, 0x8B, 0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C,
0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42, 0x43, 0x83, 0x41, 0x81, 0x80, 0x40

} ;
void InitCRC16(void)
{
  CRClo = 0xFF;
  CRChi = 0xFF;
}

void CRC16_Tstat(unsigned char ch)
{
  unsigned char uIndex ;
  uIndex = CRChi ^ ch ; // calculate the CRC 
  CRChi = CRClo ^ auchCRCHi[uIndex] ;
  CRClo = auchCRCLo[uIndex] ;
}

void calcCRC16( uint8* data, uint8 len)
{
  uint8 *pData = osal_mem_alloc( len);
  if( pData)
    osal_memcpy( pData, data, len);
  
  pData[panId_send_hi] = HI_UINT16(_NIB.nwkPanId);
  pData[panId_send_lo] = LO_UINT16(_NIB.nwkPanId);
  pData[rssi_send_hi] = 255;
  pData[rssi_send_lo] = modbus_rssi;
  pData[rev_hi] = 0;
  pData[rev_lo] = TEMCO_ZIGBEE_REV;
  InitCRC16();
  for( uint8 i = 0; i<(len-2); i++)
  {
    CRC16_Tstat(pData[i]);
  }
  pData[len-2] = CRChi;
  pData[len-1] = CRClo;

  send_str_Uart( pData, len, 0);
  osal_mem_free( pData);
}
#endif
//*********************************************
static void GenericApp_MessageMSGCB( afIncomingMSGPacket_t *pkt )
{
#if defined (START_ROUTER)
  signalStrength_t *pInSignal;
//  uint8 change_network_mode[8] = { 0xff, 0x06, 0x00, 0x78, 0x00, 0x01, 0xdd, 0xcd};
#else
  uint8 ack_byte = 1;
#endif
  switch ( pkt->clusterId )
  {
    case GENERICAPP_CLUSTERID:
#if defined START_ROUTER
    if( (pkt->cmd.Data[0] != 0xff) && (pkt->cmd.Data[1] == 0x03)&& (pkt->cmd.Data[2]!= 0xee))
    {
      if( tstat_id == pkt->cmd.Data[0])
      {
        len_check_flag = 1;
        reg_len = pkt->cmd.Data[5];
      }
      else
      {
        len_check_flag = 0;
      }
    }
    HalUARTWrite ( 0, pkt->cmd.Data, pkt->cmd.DataLength );
#else
    update_addr = pkt->srcAddr.addr.shortAddr;
      // send the received msg to uart
    
  #if defined ( USB_DONGLE)
      HalUARTWrite ( 0, pkt->cmd.Data, pkt->cmd.DataLength );
  #else
      // 测试距离用**********************************
      modbus_rssi = pkt->rssi;
  //   modbus_correlation = pkt->correlation;
  
  
    #ifdef PANID_DISPLAY
          if( panId_send == TRUE)
          {
          //  TxBuffer.buf[panId_send_hi] = HI_UINT16(_NIB.nwkPanId);
          //  TxBuffer.buf[panId_send_lo] = LO_UINT16(_NIB.nwkPanId);
          //  TxBuffer.buf[panId_send_hi] = 
            
            
            panId_send = FALSE;
            
            calcCRC16( pkt->cmd.Data, pkt->cmd.DataLength);
          }
          else
    #endif // __PANID_DISPLAY
          send_str_Uart( pkt->cmd.Data, pkt->cmd.DataLength, 0);
      
      // ********************************************
      
      
  #endif  //__defined ( USB_DONGLE)
    
#endif  // __defined START_ROUTER
    break;
    case ACK_CMD_ID:
#if defined ( START_ROUTER)
      if( pkt->cmd.Data[0] == 1)
        ack_exist = TRUE;
#else
      if( pkt->cmd.Data[0] == 0)
        zb_SendDataRequest( pkt->srcAddr.addr.shortAddr, ACK_CMD_ID, 1, &ack_byte, 
                               0, AF_ACK_REQUEST, 0);

#endif  // __defined ( START_ROUTER)
      break;
    
#if defined ( START_ROUTER)
    case RSSI_REQ_ID:
      if( pkt->cmd.Data[0] == 0)
      {
        zb_SendDataRequest( pkt->srcAddr.addr.shortAddr, RSSI_RSP_ID, 1, &tstat_id, 
                               0, AF_ACK_REQUEST, 0);
      }
      break;
      
    case RSSI_RSP_ID:
      if( pkt->cmd.Data[0] != 0)
      {
        pInSignal = findSignalStrength( pkt->cmd.Data[0]);
        if( pInSignal != NULL)
        {
          pInSignal->rssi = pkt->rssi;
        }
        else
        {
          register_signalStrength( pkt->cmd.Data[0], pkt->rssi);
          numSignalStren++;
        }
      }
      break;
#endif //  __defined ( START_ROUTER)
  }
}
//*********************************************
#if defined (START_ROUTER)

static Status_t register_signalStrength( uint8 modbus_id, int8 rssi)
{
  signalStrength_t *pNewItem;
  signalStrength_t *pLoop;
  
  pNewItem = osal_mem_alloc( sizeof( signalStrength_t));
  if( pNewItem == NULL)
  {
    return (ZMemError);
  }
  
  pNewItem->next = (signalStrength_t *)NULL;
  pNewItem->modbus_id = modbus_id;
  pNewItem->rssi = rssi;
  
  if( pSignalStren == NULL)
  {
    pSignalStren = pNewItem;
  }
  else
  {
    pLoop = pSignalStren;
    while( pLoop->next != NULL)
    {
      pLoop = pLoop->next;
    }
    pLoop->next = pNewItem;
  }
  
  return SUCCESS;
}
//*********************************************
static signalStrength_t *findSignalStrength( uint8 modbus_id)
{
  signalStrength_t *pLoop = pSignalStren;
  
  while( pLoop != NULL)
  {
    if( modbus_id == pLoop->modbus_id)
    {
      return (pLoop);
    }
    pLoop = pLoop->next;
  }
  
  return ( (signalStrength_t*)NULL);
}
//*********************************************
static void sendAllSignalStren( void)
{
  signalStrength_t *pLoop = pSignalStren;
  while( pLoop != NULL)
  {
    send_char_Uart( pLoop->modbus_id, 0);
    send_char_Uart( pLoop->rssi, 0);
    pLoop = pLoop->next;
  }
}
#endif
/*********************************************************************
 */
