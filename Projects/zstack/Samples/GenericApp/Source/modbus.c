#if defined ( MODBUS_SUPPLY)
/*********************************************************************
 * INCLUDE
 */
#include "modbus.h"
#include "hal_uart.h"
#include "OSAL_Clock.h"
/*********************************************************************
 * LOCAL VARIABLES
 */
uint8 ttt[6] = {11, 22, 33, 44, 55, 66};

uint8 const auchCRCHi[256] = {
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
0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40
} ;	
	/* Table of CRC values for high Corder byte */


uint8 const auchCRCLo[256] = {

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
0xB4, 0x74, 0x75, 0xB5, 0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71, 0x70, 0xB0,
0x50, 0x90, 0x91, 0x51, 0x93, 0x53, 0x52, 0x92, 0x96, 0x56, 0x57, 0x97, 0x55, 0x95, 0x94, 0x54,
0x9C, 0x5C, 0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E, 0x5A, 0x9A, 0x9B, 0x5B, 0x99, 0x59, 0x58, 0x98,
0x88, 0x48, 0x49, 0x89, 0x4B, 0x8B, 0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C,
0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42, 0x43, 0x83, 0x41, 0x81, 0x80, 0x40

} ;
/*********************************************************************
 * GLOBAL VARIABLES
 */
uint8 CRClo, CRChi;
uint8 modbus_id = 5;

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void modbus_send_byte( uint8 byte, uint8 crc);
static bool check_data( uint8 *buf_com, uint8 len);
static void modbus_process_msg( uint8 *data_buffer, uint8 len);
static void initCRC16( void);
static void CRC16_byte( uint8 ch);
/*********************************************************************
 * PUBLIC FUNCTIONS
 */
//void modbus_uart_data_process( uint8 *data_buffer, uint8 len);

/******************************************************************************
 * @fn      initCRC16
 * 
 * @brief   Initialize the 16 bit CRC variables
 * 
 * @param   none
 * 
 * @return  none
 */
static void initCRC16( void)
{
  CRClo = 0xff;
  CRChi = 0xff;
}

/******************************************************************************
 * @fn      CRC16_byte
 * 
 * @brief   Calculate the 16 bit CRC variables
 * 
 * @param   uint8 - the byte wanted to be calc
 * 
 * @return  none
 */
static void CRC16_byte( uint8 ch)
{
  uint8 uIndex;
  
  uIndex = CRChi ^ ch;
  CRChi = CRClo ^ auchCRCHi[uIndex];
  CRClo = auchCRCLo[uIndex];
}

/******************************************************************************
 * @fn      modbus_send_byte
 * 
 * @brief   Send one byte and calculate the crc byte
 * 
 * @param   uint8 - the byte wanted to be sent
 *          uint8 - the crc flag, need to calculate or not
 * 
 * @return  none
 */
static void modbus_send_byte( uint8 byte, uint8 crc)
{
  HalUARTWrite( 0, &byte, 1);
  if ( crc == CRC_NO)
    CRC16_byte(byte);
}

/******************************************************************************
 * @fn      check_data
 * 
 * @brief   Check message to confirm it's a modbus message
 * 
 * @param   uint8 - the pointer the buffer that to be checked
 *          uint8 - length
 * 
 * @return  bool
 */

static bool check_data( uint8 *buf_com, uint8 len)
{
  if( ( buf_com[1] != MODBUS_SINGLE_READ) && (buf_com[1] != MODBUS_SINGLE_WRITE) && ( buf_com[1] != MODBUS_MULTI_WRITE))
    return FALSE;
  
  return TRUE;
}

/******************************************************************************
 * @fn      modbus_setUtcTime
 * 
 * @brief   Set UTC Time
 * 
 * @param   uint8 - the pointer the buffer
 * 
 * @return  none
 */

static void modbus_setUtcTime( uint8 *pBuf)
{
  UTCTime utcSecs = 0;
  UTCTimeStruct utc;
  
  utc.hour = pBuf[1];
  utc.minutes = pBuf[3];
  utc.seconds = pBuf[5];
  utc.month = pBuf[7] - 1;
  utc.day = pBuf[9] - 1;
  utc.year = BUILD_UINT16( pBuf[11], pBuf[10]);
  
  if ((utc.hour < 24) && (utc.minutes < 60) && (utc.seconds < 60) &&
        (utc.month < 12) && (utc.day < 31) && (utc.year > 1999) && (utc.year < 2136))
  {
    if ((utc.month != 1) || (utc.day < (IsLeapYear( utc.year ) ? 29 : 28)))
    {
      utcSecs = osal_ConvertUTCSecs( &utc );
    }
  }
  
  if( utcSecs)
  {
    osal_setClock( utcSecs );
  }
}
/******************************************************************************
 * @fn      modbus_uart_data_process
 * 
 * @brief   Process the message from UART
 * 
 * @param   uint8 - the pointer the buffer that to be process
 *          uint8 - length
 * 
 * @return  none
 */
void modbus_uart_data_process( uint8 *data_buffer, uint8 len)
{
  if( TRUE == check_data( data_buffer, len))
  {
    initCRC16();
    modbus_process_msg( data_buffer, len);
  }
}
/******************************************************************************
 * @fn      modbus_process_msg
 * 
 * @brief   Process modbus message
 * 
 * @param   uint8 - the pointer the buffer
 *          uint8 - length 
 * 
 * @return  none
 */
static void modbus_process_msg( uint8 *data_buffer, uint8 len)
{
  uint8 num, tempByte;
  uint8 zero = 0;
  uint16 i;
  uint16 address;
  
  UTCTime utcSecs;
  UTCTimeStruct utcTime;

  utcSecs = osal_getClock();
  osal_ConvertUTCTime( &utcTime, utcSecs );
  
  address = BUILD_UINT16( data_buffer[3], data_buffer[2]);
  
  if(( data_buffer[0] == modbus_id) || ( data_buffer[0] == 255))
  {
    if(data_buffer[1] == MODBUS_SINGLE_WRITE)
    {
      HalUARTWrite( 0, data_buffer, len);
      if( address == MODBUS_ADDRESS)
      {
        modbus_id = data_buffer[5];
      }
    }
    else if( data_buffer[1] == MODBUS_MULTI_WRITE)
    {
      // write system UTC
      if( address == MODBUS_SYS_HOUR)
      {
        if( data_buffer[6] >= 12)
          modbus_setUtcTime( &data_buffer[7]);
      }
    }
    else if( data_buffer[1] == MODBUS_SINGLE_READ)
    {
      num = data_buffer[5];
      modbus_send_byte( data_buffer[0], CRC_NO);
      modbus_send_byte( data_buffer[1], CRC_NO);
      modbus_send_byte( num*2, CRC_NO);
        
      for( i = 0; i < num; i++)
      {
        if ( i + address <= MODBUS_SERIALNUMBER_LOWORD + 3)
        {
          modbus_send_byte( zero, CRC_NO);
          modbus_send_byte( ttt[ i + address - MODBUS_SERIALNUMBER_LOWORD], CRC_NO);
        }
        else if( i + address == MODBUS_FIRMWARE_VERSION_NUMBER_LO)
        {
          modbus_send_byte( zero, CRC_NO);
          modbus_send_byte( ttt[4], CRC_NO);
        }
        else if( i + address == MODBUS_FIRMWARE_VERSION_NUMBER_HI)
        {
          modbus_send_byte( zero, CRC_NO);
          modbus_send_byte( ttt[5], CRC_NO);
        }
        // System UTC Time
        else if( i + address == MODBUS_SYS_HOUR)
        {
          modbus_send_byte( zero, CRC_NO);
          modbus_send_byte( utcTime.hour, CRC_NO);
        }
        else if( i + address == MODBUS_SYS_MINUTES)
        {
          modbus_send_byte( zero, CRC_NO);
          modbus_send_byte( utcTime.minutes, CRC_NO);          
        }
        else if( i + address == MODBUS_SYS_SECONDS)
        {
          modbus_send_byte( zero, CRC_NO);
          modbus_send_byte( utcTime.seconds, CRC_NO);
        }
        else if( i + address == MODBUS_SYS_MONTH)
        {
          tempByte = utcTime.month + 1;
          modbus_send_byte( zero, CRC_NO);
          modbus_send_byte( tempByte, CRC_NO);
        }
        else if( i + address == MODBUS_SYS_DAY)
        {
          tempByte = utcTime.day + 1;
          modbus_send_byte( zero, CRC_NO);
          modbus_send_byte( tempByte, CRC_NO);
        }
        else if( i + address == MODBUS_SYS_YEAR)
        {
          tempByte = HI_UINT16( utcTime.year);
          modbus_send_byte( tempByte, CRC_NO);
          tempByte = LO_UINT16( utcTime.year);
          modbus_send_byte( tempByte, CRC_NO);
        }
        else
        {
          modbus_send_byte( 0, CRC_NO);
          modbus_send_byte( 1, CRC_NO);
        }
      }
      modbus_send_byte( CRChi, CRC_YES);
      modbus_send_byte( CRClo, CRC_YES);
    }
  }
}
#endif  // MODBUS_SUPPLY