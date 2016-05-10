

/* ------------------------------------------------------------------------------------------------
 *                                          Includes
 * ------------------------------------------------------------------------------------------------
 */
#include "ZGlobals.h"
#include "OSAL.h"
#include "MT_UART.h"
#include "temcoDevice.h"
#include "modbus.h"
#include "temcoAPI.h"

/* ------------------------------------------------------------------------------------------------
 *                                           Local Functions
 * ------------------------------------------------------------------------------------------------
 */
static void temcoInit(void);
static void temcoUartCback(uint8 port, uint8 event);

/* ------------------------------------------------------------------------------------------------
 *                                           Local Variables
 * ------------------------------------------------------------------------------------------------
 */
uint8 stateTemco;

//uint8 *pMsgTemco = NULL;
uint8 pMsgTemco[MT_UART_DEFAULT_MAX_RX_BUFF];
uint8 head, cmd;
uint8 tempDataLenTemco;
uint8 cmdLen;
static bool isAlloc = FALSE;
static bool haveMsg = FALSE;
/**************************************************************************************************
 * CONSTANT
 */
#define HEAD_STATE    0x00
#define CMD_STATE     0x01
#define DATA_STATE    0x02
#define LEN_STATE     0x03
#define ID_HI_STATE   0x04
#define ID_LO_STATE   0x05
#define LEN_HI_STATE  0x06

/* ------------------------------------------------------------------------------------------------
 *                                           Global Variables
 * ------------------------------------------------------------------------------------------------
 */

/**************************************************************************************************
 * @fn          temcoTaskInit
 */
void temcoTaskInit(uint8 taskId)
{
  temcoTaskId = taskId;
  osal_set_event(taskId, ZNP_SECONDARY_INIT_EVENT);
  osal_set_event(temcoTaskId, RESET_STATE);
}

/**************************************************************************************************
 * @fn          temcoEventLoop
 */
uint16 temcoEventLoop( uint8 taskId, uint16 events)
{
  if (events & SYS_EVENT_MSG)
  {
    
    events ^= SYS_EVENT_MSG;
  }
  else if(events & ZNP_SECONDARY_INIT_EVENT)
  {
    temcoInit();
    events ^= ZNP_SECONDARY_INIT_EVENT;
  }
  else if( events & RESET_STATE)
  {
    if( haveMsg )
    {
      haveMsg = FALSE;
      if( stateTemco != HEAD_STATE)
      {
        stateTemco = HEAD_STATE;
      }
      if( isAlloc == TRUE)
      {
        isAlloc = FALSE;
     //   osal_mem_free( pMsgTemco);
      }
    }
    events ^= RESET_STATE;
    osal_start_timerEx( temcoTaskId, RESET_STATE, 100);
  }
  else
  {
    events = 0;
  }
  
  return events;
}

/***************************************************************************************************
 * @fn           temcoInit
 */
static void temcoInit(void)
{
  halUARTCfg_t uartConfig;

  uartConfig.configured           = TRUE;
  if(zgDeviceLogicalType == ZG_DEVICETYPE_COORDINATOR)
    uartConfig.baudRate = HAL_UART_BR_19200;
  else
    uartConfig.baudRate             = HAL_UART_BR_19200;
  uartConfig.flowControl          = FALSE;
  uartConfig.flowControlThreshold = HAL_UART_FLOW_THRESHOLD;
  uartConfig.rx.maxBufSize        = MT_UART_DEFAULT_MAX_RX_BUFF;
  uartConfig.tx.maxBufSize        = MT_UART_DEFAULT_MAX_TX_BUFF;
  uartConfig.idleTimeout          = HAL_UART_IDLE_TIMEOUT;
  uartConfig.intEnable            = TRUE;
  uartConfig.callBackFunc         = temcoUartCback;
  HalUARTOpen(HAL_UART_PORT, &uartConfig);
  MT_UartRegisterTaskID(temcoTaskId);
  
  stateTemco = HEAD_STATE;
  MT_Init();
}
uint8 modbusId_hi,modbusId_lo,len_hi,len_lo;
/**************************************************************************************************
 * @fn          temcoUartCback
 */

static void temcoUartCback(uint8 port, uint8 event)
{
  uint8 ch;
  uint8 bytesInRxBuffer;
  
  (void)event;  // Intentionally unreferenced parameter
  uint8 headLen;
  
  while (Hal_UART_RxBufLen(port))
  {
    HalUARTRead( port, &ch, 1);
    
    if(zgDeviceLogicalType == ZG_DEVICETYPE_COORDINATOR)
    {
      switch(stateTemco)
      {
        case HEAD_STATE:
          head = ch;
          stateTemco = CMD_STATE;
          haveMsg = TRUE;
        //  osal_start_timerEx( temcoTaskId, RESET_STATE, RESET_STATE_TIMEOUT);
          break;
          
        case CMD_STATE:
          cmd = ch;
          tempDataLenTemco = 0;
          if( cmd == MODBUS_MULTI_WRITE)
          {
            stateTemco = ID_HI_STATE;
          }
          else
          {
            if( (head == 0xff) && (cmd == MODBUS_SCAN_CMD))
              cmdLen = 4;
            else if( (cmd == MODBUS_SINGLE_READ ) || ( cmd == MODBUS_SINGLE_WRITE))
              cmdLen = 6;
            else
              cmdLen = 6;
          //  pMsgTemco = osal_mem_alloc(cmdLen + 2);
            isAlloc = TRUE;
          //  if(pMsgTemco)
            {
              pMsgTemco[0] = head;
              pMsgTemco[1] = cmd;
            }
            stateTemco = DATA_STATE;
          }
          break;
          
        case ID_HI_STATE:
          modbusId_hi = ch;
          stateTemco = ID_LO_STATE;
          break;
          
        case ID_LO_STATE:
          modbusId_lo = ch;
          stateTemco = LEN_HI_STATE;
          break;
          
        case LEN_HI_STATE:
          len_hi = ch;
          stateTemco = LEN_STATE;
          break;
          
        case LEN_STATE:
          len_lo = ch;
          cmdLen = len_lo*2+3;
        //  pMsgTemco = osal_mem_alloc(cmdLen+6);
          isAlloc = TRUE;
       //   if(pMsgTemco)
          {
            pMsgTemco[0] = head;
            pMsgTemco[1] = cmd;
            pMsgTemco[2] = modbusId_hi;
            pMsgTemco[3] = modbusId_lo;
            pMsgTemco[4] = len_hi;
            pMsgTemco[5] = len_lo;
          }
          stateTemco = DATA_STATE;
          break;
        
        case DATA_STATE:
          if(cmd == MODBUS_MULTI_WRITE)
            headLen = 6;
          else
            headLen = 2;
          pMsgTemco[headLen+tempDataLenTemco++] = ch;
          bytesInRxBuffer = Hal_UART_RxBufLen(port);
          if(bytesInRxBuffer <= cmdLen - tempDataLenTemco)
          {
            HalUARTRead(port, &pMsgTemco[headLen+tempDataLenTemco], bytesInRxBuffer);
            tempDataLenTemco += bytesInRxBuffer;
          }
          else
          {
            HalUARTRead(port, &pMsgTemco[headLen+tempDataLenTemco], cmdLen - tempDataLenTemco);
            tempDataLenTemco += (cmdLen - tempDataLenTemco);
          }
          
          if( tempDataLenTemco == cmdLen)
          {
            if( (head == 0xff) && (cmd == 0x03) && (cmdLen == 0x06))  // Use while asking TSTAT ID
            {
              // store the tstat id
              tstat_id = pMsgTemco[4];
              product_id = pMsgTemco[6];
            }
            else
              modbus_uart_data_process( pMsgTemco, cmdLen+headLen);
            stateTemco = HEAD_STATE;
       //     osal_mem_free(pMsgTemco);
            haveMsg = FALSE;
          }
          break;
          
        default:
          break;
      }
    }
    else  // Router type device
    {
      uint16 coord_addr = 0; // Coordinater address
      switch(stateTemco)
      {
        case HEAD_STATE:
          head = ch;
          stateTemco = CMD_STATE;
      //    haveMsg = TRUE;
     //     osal_start_timerEx( temcoTaskId, RESET_STATE, RESET_STATE_TIMEOUT);
          break;
          
        case CMD_STATE:
          cmd = ch;
          tempDataLenTemco = 0;
          if( head == modbus_id) // for itself
          {
            cmdLen = 6;
            stateTemco = DATA_STATE;
          }
          else
          {
            if( cmd == MODBUS_SINGLE_READ )
            {
              stateTemco = LEN_STATE;
            }
            else if( (cmd == MODBUS_SCAN_CMD) || (cmd == MODBUS_SINGLE_WRITE) || (cmd == MODBUS_MULTI_WRITE))
            {
              if( cmd == MODBUS_SCAN_CMD)
                cmdLen = 7;
              else if( (cmd == MODBUS_SINGLE_WRITE) || (cmd == MODBUS_MULTI_WRITE))
                cmdLen = 6;
           //   pMsgTemco = osal_mem_alloc(cmdLen + 2);
              isAlloc = TRUE;
           //   if(pMsgTemco)
              {
                pMsgTemco[0] = head;
                pMsgTemco[1] = cmd;
              }
              stateTemco = DATA_STATE;
            }
            else
              stateTemco = HEAD_STATE;
          }
          break;
          
        case DATA_STATE:
          if(cmd == MODBUS_SINGLE_READ)
            headLen = 3;
          else
            headLen = 2;
          pMsgTemco[headLen+tempDataLenTemco++] = ch;
          bytesInRxBuffer = Hal_UART_RxBufLen(port);
          if(bytesInRxBuffer <= cmdLen - tempDataLenTemco)
          {
            HalUARTRead(port, &pMsgTemco[headLen+tempDataLenTemco], bytesInRxBuffer);
            tempDataLenTemco += bytesInRxBuffer;
          }
          else
          {
            HalUARTRead(port, &pMsgTemco[headLen+tempDataLenTemco], cmdLen - tempDataLenTemco);
            tempDataLenTemco += (cmdLen - tempDataLenTemco);
          }
          
          if( tempDataLenTemco == cmdLen)
          {
            stateTemco = HEAD_STATE;
            if( (head == 0xff) && (cmd == 0x03) && (cmdLen == 0x06))  // Use while asking TSTAT ID
            {
              // store the tstat id
              tstat_id = pMsgTemco[4];
              product_id = pMsgTemco[6];
            }
            else
            {
              if( head == 23)
                modbus_uart_data_process(pMsgTemco, cmdLen+headLen);
              else
              {
                if(modbusDataLength>0)
                  modbus_insert_msg(pMsgTemco, cmdLen+headLen);
                else
                  zb_SendDataRequest( coord_addr, TEMCO_CLUSTERID, cmdLen+headLen, pMsgTemco,
                                    0, AF_ACK_REQUEST, 0);
              }
            }
            haveMsg = FALSE;
         //   osal_mem_free(pMsgTemco);
          }
          break;
          
        case LEN_STATE:
          cmdLen = ch +2;
       //   pMsgTemco = osal_mem_alloc(cmdLen + 3);
          isAlloc = TRUE;
       //   if(pMsgTemco)
          {
            pMsgTemco[0] = head;
            pMsgTemco[1] = cmd;
            pMsgTemco[2] = ch;
          }
          stateTemco = DATA_STATE;
          break;
          
        default:
          break;
      }
    }
  }
  
  
}