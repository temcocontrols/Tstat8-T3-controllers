#ifndef TEMCOAPP_H
#define TEMCOAPP_H

#ifdef __cplusplus
extern "C"
{
#endif
  
/*********************************************************************
 * INCLUDES
 */
#include "ZComDef.h"
#include "AF.h"
  
/*********************************************************************
 * TYPEDEFS
 */
typedef struct signalStrength
{
  struct signalStrength *next;
  uint8 modbus_id;
  int8 rssi;
  uint8 leaveTime;
} signalStrength_t;  
/*********************************************************************
 * CONSTANTS
 */
#define TEMCO_ENDPOINT           10
#define TEMCO_PROFID             0x0F10
#define TEMCO_DEVICEID           0x0001
#define TEMCO_DEVICE_VERSION     0
#define TEMCO_FLAGS              0  
  
#define TEMCO_MAX_CLUSTERS       1
#define TEMCO_CLUSTERID          2
#define ACK_CMD_CLUSTERID        3
#define RSSI_REQ_CLUSTERID       4
#define RSSI_RSP_CLUSTERID       5
 
#define ACK_CHECK                0x0004
#define ASK_MODBUS_ID            0x0008
  

  
/*********************************************************************
 * GLOBAL VARIABLES
 */  
extern uint8 tstat_id;
extern uint8 product_id;
extern uint8 numSignalStren;
extern signalStrength_t *pSignalStren;
/*********************************************************************
 * FUNCTIONS
 */  
extern void temcoApp_Init(uint8 task_id);
extern uint16 temcoApp_ProcessEvent(uint8 task_id, uint16 events);
extern afStatus_t zb_SendDataRequest( uint16 destination, uint16 commandId, uint8 len,
                          uint8 *pData, uint8 handle, uint8 txOptions, uint8 radius );
#endif