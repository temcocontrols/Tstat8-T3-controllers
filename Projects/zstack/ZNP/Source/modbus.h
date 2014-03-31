#ifndef MODBUS_H
#define MODBUS_H
/*********************************************************************
* INCLUDE
 */
#include "ZComDef.h"

/*********************************************************************
* CONTANTS
 */
#define CRC_NO  0
#define CRC_YES 1
#define ZERO    0

#define ZIG_SOFTWARE_VER        0x0a

#define MODBUS_SINGLE_READ      0x03
#define MODBUS_SINGLE_WRITE     0x06
#define MODBUS_MULTI_WRITE      0x10
#define MODBUS_SCAN_CMD         0x19

#define MODBUS_TSTAT_NUM            (MODBUS_SECURITY_KEY_END+1)
#define MODBUS_FIRST_TSTAT_ID       (MODBUS_TSTAT_NUM+1)
#define MODBUS_LAST_TSTAT_ID        (MODBUS_FIRST_TSTAT_ID+numSignalStren-1)
#define MODBUS_FIRST_SIG_STREN      (MODBUS_LAST_TSTAT_ID+1)
#define MODBUS_LAST_SIG_STREN       (MODBUS_FIRST_SIG_STREN+numSignalStren)
/*********************************************************************
* MACROS
 */


/*********************************************************************
 * GLOBAL VARIABLES
 */
extern uint8 CRClo, CRChi;
extern uint8 modbus_id;
extern uint8 modbusDataLength;
extern uint8 modbusStartAddr;
extern uint8 firstAddr;

#if defined ( RSSI_DISPLAY)
  extern uint8 rssi_send_hi, rssi_send_lo;
  extern bool panId_send ;
  extern int8 modbus_rssi;
  extern void calcCRC16( uint8* data, uint8 len);
#endif
/*********************************************************************
 * GLOBAL FUNCTIONS
 */
extern void modbus_uart_data_process( uint8 *data_buffer, uint8 len);
extern void initCRC16( void);
extern void CRC16_byte( uint8 ch);
extern void modbus_insert_msg( uint8 *pBuf, uint8 len);
extern void modbus_single_write( uint8 *pBuf, uint8 len);
extern void modbus_multi_write( uint8 *data_buffer, uint8 len);
/*********************************************************************
 * LOCAL VARIABLES
 */
enum {
/*  MODBUS_SERIALNUMBER_LOWORD  = 0,             
  MODBUS_SERIALNUMBER_HIWORD  = 2,
  MODBUS_FIRMWARE_VERSION_NUMBER_LO = 4,
  MODBUS_FIRMWARE_VERSION_NUMBER_HI,
  MODBUS_ADDRESS                = 6,
  MODBUS_HARDWARE_REV,
  
  MODBUS_SYS_HOUR,
  MODBUS_SYS_MINUTES,
  MODBUS_SYS_SECONDS,
  MODBUS_SYS_MONTH,
  MODBUS_SYS_DAY,
  MODBUS_SYS_YEAR, */
  
  MODBUS_PANID = 21,
  MODBUS_DEVICE_TYPE,
  MODBUS_CHANNEL_LIST_HI,
  MODBUS_CHANNEL_LIST_LO,
  MODBUS_SOFTWARE_REV,
  MODBUS_EXTENDED_ADDR_HI,
  MODBUS_EXTENDED_ADDR_LO = 33,
  MODBUS_FACTORY_RESTORE,
  MODBUS_SECURITY_KEY_START = 35,
  MODBUS_SECURITY_KEY_END = 50,
};

#endif // MODBUS_H