/******************************************************************************
 * INCLUDES
 */

#include "ZComDef.h"
#include "ZDApp.h"
#include "OSAL.h"
#include "OSAL_Tasks.h"

#include "AF.h"
#include "hal_uart.h"

#include "temcoAPI.h"
#include "OnBoard.h"
#include "modbus.h"
#include "OSAL_nv.h"
#include "hal_led.h"
#include "string.h"
/*********************************************************************
 * CONSTANTS
 */
#define TYPE_ASSERT_TIMEOUT     6

#define ACK_CHECK_TIMEOUT        10000     // 10000 means 10 secends
#define RSSI_REQ_TIMEOUT         10000

#define RSSI_NODE_LEAVE_NUM   2
#define ACK_TIMEOUT_NUM       2
/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */
const cId_t temcoClusterList[TEMCO_MAX_CLUSTERS] =
{
  TEMCO_CLUSTERID,
  
};

const SimpleDescriptionFormat_t temcoSimpleDesc =
{
  TEMCO_ENDPOINT,              //  int Endpoint;
  TEMCO_PROFID,                //  uint16 AppProfId[2];
  TEMCO_DEVICEID,              //  uint16 AppDeviceId[2];
  TEMCO_DEVICE_VERSION,        //  int   AppDevVer:4;
  TEMCO_FLAGS,                 //  int   AppFlags:4;
  TEMCO_MAX_CLUSTERS,          //  byte  AppNumInClusters;
  (cId_t *)temcoClusterList,  //  byte *pAppInClusterList;
  TEMCO_MAX_CLUSTERS,          //  byte  AppNumInClusters;
  (cId_t *)temcoClusterList   //  byte *pAppInClusterList;
};

endPointDesc_t temco_epDesc;
bool ack_exist = TRUE;
uint8 ack_count = 0;
uint8 ask_modbus_id[8] = { 0xff, 0x03, 0x00, 0x06, 0x00, 0x02, 0x31, 0xd4};//0x01, 0x71, 0xd5};
uint8 tstat_id = 0;
uint8 product_id = 0;
uint8 type_assert = 0;
signalStrength_t *pSignalStren;
uint8 numSignalStren = 0;
/*********************************************************************
 * LOCAL VARIABLES
 */
byte temcoAPP_TaskID;
byte temcoApp_TransID;
/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void temcoApp_MessageMSGCB(afIncomingMSGPacket_t *pkt);
static Status_t register_signalStrength( uint8 modbus_id, int8 rssi);
static signalStrength_t *findSignalStrength( uint8 modbus_id);
static uint8 checkNodeAlive( void);
static void deleteSignalStrength( uint8 modbus_id);
static void sendAllSignalStren( void);
static void restart_to_other_type(void);
/*********************************************************************
 * @fn      temcoApp_Init
 */
void temcoApp_Init(uint8 task_id)
{
  temcoAPP_TaskID = task_id;
  
  temco_epDesc.endPoint = TEMCO_ENDPOINT;
  temco_epDesc.task_id = &temcoAPP_TaskID;
  temco_epDesc.simpleDesc = (SimpleDescriptionFormat_t *)&temcoSimpleDesc;
  temco_epDesc.latencyReq = noLatencyReqs;
  
  // register the endpoint description with the AF
  afRegister( &temco_epDesc);
#if 0
  if( zgDeviceLogicalType == ZG_DEVICETYPE_COORDINATOR)
  {
     tstat_id = 254;        // NC's modbus id always is 9
  }
#endif
  if( zgDeviceLogicalType == ZG_DEVICETYPE_COORDINATOR)
  {
    osal_set_event( temcoAPP_TaskID, SEND_ZIGBEE_FLAG);
  }
  else
    osal_set_event( temcoAPP_TaskID, ASK_MODBUS_ID);
}

/*********************************************************************
 * @fn      temcoApp_ProcessEvent
 */
uint16 temcoApp_ProcessEvent(uint8 task_id, uint16 events)
{
  osal_event_hdr_t *pMsg;
  afIncomingMSGPacket_t *pMSGpkt;
  
  if( events & SYS_EVENT_MSG)
  {
    pMsg = (osal_event_hdr_t *) osal_msg_receive( task_id );
    while(pMsg)
    {
      switch( pMsg->event)
      {
        case AF_INCOMING_MSG_CMD:
          pMSGpkt = (afIncomingMSGPacket_t *)pMsg;
          temcoApp_MessageMSGCB(pMSGpkt);
          break;
          
        case ZDO_STATE_CHANGE:
          if (pMsg->status == DEV_END_DEVICE ||
              pMsg->status == DEV_ROUTER )
              //||pMsg->status == DEV_ZB_COORD )
          {
            osal_set_event( temcoAPP_TaskID, ACK_CHECK);
//            HalLedSet ( HAL_LED_1, HAL_LED_MODE_FLASH );
//            HalLedBlink ( HAL_LED_1, 0, 50, 500 );
          }
          break;
          
        default:
          
          break;
      }
      // Release the memory
      osal_msg_deallocate( (uint8 *) pMsg );
      
      // Next
      pMsg = (osal_event_hdr_t *) osal_msg_receive( task_id );
    }
    // Return unprocessed events
    return (events ^ SYS_EVENT_MSG);
  }
  
  if (events & ACK_CHECK)
  {
    uint8 ack_byte = 0;
    
    if( ack_count > ACK_TIMEOUT_NUM)
    {
      ack_count = 0;
      if( ack_exist == FALSE)
      {
        restore_factory_setting();
      }
    }
    if( ack_exist == TRUE)
    {
      zb_SendDataRequest( 0, ACK_CMD_CLUSTERID, 1, &ack_byte,
                           0, AF_ACK_REQUEST, 0);
      ack_exist = FALSE;
    }
    ack_count ++;
    osal_start_timerEx( temcoAPP_TaskID, ACK_CHECK, ACK_CHECK_TIMEOUT);   // Every minute check ack, if no receive, restart to join a new network
    return ( events ^ ACK_CHECK);
  }
  
  // Send the command to check TSTAT modbus id
  if( events & ASK_MODBUS_ID)
  {
    uint8 rssi_byte = 0;
    uint8 deleteId;

    if( type_assert >= TYPE_ASSERT_TIMEOUT)  // Decide which type to start up
    {
#if 0
      if( (product_id == 0) || (product_id == 100))   
      {
        if( zgDeviceLogicalType == ZG_DEVICETYPE_ROUTER)
        {
          zgDeviceLogicalType = ZG_DEVICETYPE_COORDINATOR;
          restart_to_other_type();
        }
      }
      else
      {
        if( zgDeviceLogicalType == ZG_DEVICETYPE_COORDINATOR)
        {
          zgDeviceLogicalType = ZG_DEVICETYPE_ROUTER;
          restart_to_other_type();
        }
      }
#endif
      // To read register6 of TSTAT
      if( tstat_id != 0)
      {
        //if( zgDeviceLogicalType != ZG_DEVICETYPE_COORDINATOR)
        // Send a command asking other nodes' modbus id, the response message carry rssi 
        zb_SendDataRequest( 0xffff, RSSI_REQ_CLUSTERID, 1, &rssi_byte,
                             0, AF_ACK_REQUEST, 0);
        
        deleteId = checkNodeAlive();   // check if there are any nodes not alive
        if( deleteId != 255)
          deleteSignalStrength(deleteId);        // delete the dead id
      }
    }
    else
    {
      type_assert ++;
      HalUARTWrite ( 0, ask_modbus_id, 8 );
    }
    
    // if not received, send again X seconds later
    osal_start_timerEx( temcoAPP_TaskID, ASK_MODBUS_ID, RSSI_REQ_TIMEOUT);
    return ( events ^ ASK_MODBUS_ID);
  }
  
  if( events & SEND_ZIGBEE_FLAG)
  {
    if( tstat_id == 0)
      HalUARTWrite( 0, "zigbee", strlen("zigbee"));
    else
      osal_stop_timerEx( temcoAPP_TaskID, SEND_ZIGBEE_FLAG);
    osal_start_timerEx( temcoAPP_TaskID, SEND_ZIGBEE_FLAG, 3000);
  }
  
  // Discard unknown events
  return 0;
}

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
  
  else if( destination == MAC_SHORT_ADDR_BROADCAST)
  {
    dstAddr.addr.shortAddr = destination;
    dstAddr.addrMode = afAddrBroadcast;
  }
  else
  {
    // Use short address
    dstAddr.addr.shortAddr = destination;
    dstAddr.addrMode = afAddr16Bit;
  }

  dstAddr.panId = 0;                                    // Not an inter-pan message.
  dstAddr.endPoint = temco_epDesc.simpleDesc->EndPoint;  // Set the endpoint.

  // Send the message
  status = AF_DataRequest(&dstAddr, &temco_epDesc, commandId, len,
                          pData, &handle, txOptions, radius);

  return status;
}


/*********************************************************************
 * @fn      temcoApp_MessageMSGCB
 */
static void temcoApp_MessageMSGCB(afIncomingMSGPacket_t *pkt)
{
  switch(pkt->clusterId)
  {
    case TEMCO_CLUSTERID:
      if( zgDeviceLogicalType == ZG_DEVICETYPE_ROUTER)  // Prepare to insert modbus message into TSTAT's
      {
        if( (pkt->cmd.Data[1] == MODBUS_SINGLE_READ) && (pkt->cmd.DataLength == 8))
        {
          firstAddr = BUILD_UINT16(pkt->cmd.Data[3],pkt->cmd.Data[2]);
          uint8 length = pkt->cmd.Data[5];
          
          if( (firstAddr<21)&&((firstAddr+length)>21))
          {
            modbusDataLength = firstAddr+length-21;
            modbusStartAddr = 21;
          }
          else if( (firstAddr>=21) && (firstAddr <100))
          {
            modbusStartAddr = firstAddr;
            if(length <= (101-firstAddr))
              modbusDataLength = length;
            else
              modbusDataLength = 101-firstAddr;
          }
          else
          {
            modbusStartAddr = 0;
            modbusDataLength = 0;
          }
        }
        else if( pkt->cmd.Data[1] == MODBUS_SINGLE_WRITE)
        {
          modbus_single_write(pkt->cmd.Data, pkt->cmd.DataLength);
        }
        else if( pkt->cmd.Data[1] == MODBUS_MULTI_WRITE)
        {
          modbus_multi_write(pkt->cmd.Data, pkt->cmd.DataLength);
        }
      }
      HalUARTWrite ( 0, pkt->cmd.Data, pkt->cmd.DataLength );
      break;
      
    case ACK_CMD_CLUSTERID:  // 7/16取消修改 此处改为定时发广播，收到任何节点消息，不论对方类型，都不需要重启
      if(zgDeviceLogicalType == ZG_DEVICETYPE_COORDINATOR)
      {
        uint8 ack_byte = 1;
        zb_SendDataRequest( pkt->srcAddr.addr.shortAddr, ACK_CMD_CLUSTERID, 1, &ack_byte, 
                               0, AF_ACK_REQUEST, 0);
      }
      else
      {
        if(pkt->cmd.Data[0] == 1)
          ack_exist = TRUE;
      }
      break;
      
    case RSSI_REQ_CLUSTERID:
      if( pkt->cmd.Data[0] == 0)
      {
        if( zgDeviceLogicalType == ZG_DEVICETYPE_COORDINATOR)
        {
          zb_SendDataRequest( pkt->srcAddr.addr.shortAddr, RSSI_RSP_CLUSTERID, 1, &tstat_id, 
                                 0, AF_ACK_REQUEST, 0);
        }
        else
        {
          if( tstat_id != 0)
            zb_SendDataRequest( pkt->srcAddr.addr.shortAddr, RSSI_RSP_CLUSTERID, 1, &tstat_id, 
                                   0, AF_ACK_REQUEST, 0);
        }
      }
      break;
    case RSSI_RSP_CLUSTERID:
      if( pkt->cmd.Data[0] != 0)
      {
        
        signalStrength_t *pInSignal;
        pInSignal = findSignalStrength( pkt->cmd.Data[0]);
        if( pInSignal != NULL)
        {
          pInSignal->rssi = pkt->rssi;
          pInSignal->leaveTime = 0;
        }
        else
        {
          register_signalStrength( pkt->cmd.Data[0], pkt->rssi);
          numSignalStren++;
        }
      }
      break;
    default:
      break;
  }
}

//*********************************************
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
  pNewItem->leaveTime = 0;
  
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
static void deleteSignalStrength( uint8 modbus_id)
{
  signalStrength_t *pLoop, *pNext;
  
  pLoop = pSignalStren;
  pNext = pSignalStren->next;
  
  if( pLoop != NULL)
  {
    if(pLoop->modbus_id == modbus_id)
    {
      numSignalStren--;
      if(pNext != NULL)
        pSignalStren = pNext;
      else
        pSignalStren = NULL;
      osal_mem_free( pLoop);
    }
    else
    {
      while( pNext != NULL)
      {
        if(pNext->modbus_id == modbus_id)
        {
          numSignalStren--;
          pLoop->next = pNext->next;
          osal_mem_free(pNext);
          return;
        }
        pLoop = pNext;
        pNext = pLoop->next;
      }
    }
  }
}
//*********************************************
static uint8 checkNodeAlive( void)
{
  signalStrength_t *pLoop = pSignalStren;
  uint8 modbus_id = 255;
  
  while(pLoop != NULL)
  {
    pLoop->leaveTime++;
    if(pLoop->leaveTime >= RSSI_NODE_LEAVE_NUM)
    {
      modbus_id = pSignalStren->modbus_id;
      return modbus_id;
    }
    pLoop = pLoop->next;
  }
  return modbus_id;
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
//*********************************************
static void restart_to_other_type(void)
{
  NLME_InitNV();
  NLME_SetDefaultNV();
  osal_nv_write( ZCD_NV_LOGICAL_TYPE, 0, sizeof(zgDeviceLogicalType), &zgDeviceLogicalType);
  SystemReset();
}