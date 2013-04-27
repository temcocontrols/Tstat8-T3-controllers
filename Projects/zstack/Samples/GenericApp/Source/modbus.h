#if defined ( MODBUS_SUPPLY)

#ifndef MODBUS_H
#define MODBUS_H
/*********************************************************************
* INCLUDE
 */

#include "GenericApp.h"
/*********************************************************************
* CONTANTS
 */
#define CRC_NO  0
#define CRC_YES 1

#define MODBUS_SINGLE_READ      0x03
#define MODBUS_SINGLE_WRITE     0x06
#define MODBUS_MULTI_WRITE      0x10
/*********************************************************************
 * GLOBAL VARIABLES
 */
extern uint8 CRClo, CRChi;
extern uint8 modbus_id;
/*********************************************************************
 * GLOBAL FUNCTIONS
 */
extern void modbus_uart_data_process( uint8 *data_buffer, uint8 len);
/*********************************************************************
 * LOCAL VARIABLES
 */
enum {
  MODBUS_SERIALNUMBER_LOWORD  = 0,             
  MODBUS_SERIALNUMBER_HIWORD  = 2,
  MODBUS_FIRMWARE_VERSION_NUMBER_LO = 4,
  MODBUS_FIRMWARE_VERSION_NUMBER_HI,
  MODBUS_ADDRESS                = 6,
  
  MODBUS_SYS_HOUR = 8,
  MODBUS_SYS_MINUTES,
  MODBUS_SYS_SECONDS,
  MODBUS_SYS_MONTH,
  MODBUS_SYS_DAY,
  MODBUS_SYS_YEAR
};

#endif // MODBUS_H

#endif // MODBUS_SUPPLY