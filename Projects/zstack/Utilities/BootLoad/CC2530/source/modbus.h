#if defined ( MODBUS_SUPPLY)

#ifndef MODBUS_H
#define MODBUS_H



/*********************************************************************
* CONTANTS
 */
#define CRC_NO  0
#define CRC_YES 1

#define MODBUS_SINGLE_READ      0x03
#define MODBUS_SINGLE_WRITE     0x06
#define MODBUS_MULTI_WRITE      0x10

/*********************************************************************
 * LOCAL VARIABLES
 */
enum {
  MODBUS_ADDRESS,
  MODBUS_VERSION_NUM,
  FLAS
};




#endif                    // MOBUS_H