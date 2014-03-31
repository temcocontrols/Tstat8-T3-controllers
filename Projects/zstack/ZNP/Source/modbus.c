/*********************************************************************
 * INCLUDE
 */
#include "modbus.h"
#include "hal_defs.h"
#include "hal_uart.h"
#include "AF.h"
#include "OnBoard.h"
#include "OSAL_Clock.h"
#include "OSAL_Nv.h"

#include "temcoAPI.h"
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
#if defined ( RFID_SUPPLY)
uint8 modbus_id = 254;
#else
uint8 modbus_id = 23;
#endif
uint8 zSoftwareRev = 10;
uint8 modbusDataLength = 0;
uint8 modbusStartAddr = 0;
uint8 firstAddr;

#if defined ( RSSI_DISPLAY)
  bool panId_send = FALSE;
  uint8 rssi_send_hi, rssi_send_lo;
  int8 modbus_rssi;
#endif
bool restroe_factory_setting = FALSE;

/*********************************************************************
 * LOCAL FUNCTIONS
 */
// static uint16 CRC16( uint8 *puchMsg , uint8 usDataLen);
static void modbus_send_byte( uint8 byte, uint8 crc);
static bool check_data( uint8 *buf_com, uint8 len);
static void modbus_process_msg( uint8 *data_buffer, uint8 len);

/*********************************************************************
 * PUBLIC FUNCTIONS
 */
void modbus_uart_data_process( uint8 *data_buffer, uint8 len);

extern byte temcoApp_TransID;
extern endPointDesc_t temco_epDesc;
extern uint8 zgPreconfigKeyInit( uint8 setDefault );
extern void ZDSecMgrInitNVKeyTables(uint8 setDefault);
/*********************************************************************
 * MAIN PROGRAM
 */

/******************************************************************************
 * @fn      initCRC16
 * 
 * @brief   Initialize the 16 bit CRC variables
 * 
 * @param   none
 * 
 * @return  none
 */
void initCRC16( void)
{
  CRClo = 0xff;
  CRChi = 0xff;
}
/*
static uint16 CRC16( uint8 *puchMsg , uint8 usDataLen)
{
  uint16 uchCRCHi = 0xff;
  uint8 uchCRCLo = 0xff;
  uint8 uIndex;
  
  while( usDataLen--)
  {
    uIndex = uchCRCHi ^ *puchMsg++ ; 
    uchCRCHi = uchCRCLo ^ auchCRCHi[uIndex] ;
    uchCRCLo = auchCRCLo[uIndex] ;
  }  
  return ( uchCRCHi << 8 | uchCRCLo);
}
*/

/******************************************************************************
 * @fn      CRC16_byte
 * 
 * @brief   Calculate the 16 bit CRC variables
 * 
 * @param   uint8 - the byte wanted to be calc
 * 
 * @return  none
 */
void CRC16_byte( uint8 ch)
{
  uint8 uIndex;
  
  uIndex = CRChi ^ ch;
  CRChi = CRClo ^ auchCRCHi[uIndex];
  CRClo = auchCRCLo[uIndex];
}

#if defined ( RSSI_DISPLAY)
void calcCRC16( uint8* data, uint8 len)
{
  uint8 *pData = osal_mem_alloc( len);
  if( pData)
    osal_memcpy( pData, data, len);

  pData[rssi_send_hi] = 255;
  pData[rssi_send_lo] = modbus_rssi;
  initCRC16();
  for( uint8 i = 0; i<(len-2); i++)
  {
    CRC16_byte(pData[i]);
  }
  pData[len-2] = CRChi;
  pData[len-1] = CRClo;

  HalUARTWrite( 0, pData, len);
  osal_mem_free( pData);
}
#endif
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
  afAddrType_t destAddr;
  destAddr.addrMode = afAddrBroadcast;
  destAddr.addr.shortAddr = 0xffff;                
  destAddr.endPoint = 10;

  if( TRUE == check_data( data_buffer, len))
  {
    initCRC16();
    modbus_process_msg( data_buffer, len);
  }
  else
  {
    AF_DataRequest( &destAddr, &temco_epDesc,
                     TEMCO_CLUSTERID,
                     len,
                     data_buffer,
                     &temcoApp_TransID,
                     AF_DISCV_ROUTE,
                     AF_DEFAULT_RADIUS );
  }
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
 * @fn      modbus_single_write
 */
void modbus_single_write( uint8 *data_buffer, uint8 len)
{
  uint8 address;
  
  address = BUILD_UINT16( data_buffer[3], data_buffer[2]);
  
  if( address == MODBUS_PANID)
  {
    zgConfigPANID = BUILD_UINT16( data_buffer[5], data_buffer[4]);
    osal_nv_write( ZCD_NV_PANID, 0, sizeof(zgConfigPANID), &zgConfigPANID);
  }
  else if( address == MODBUS_DEVICE_TYPE)
  {
    zgDeviceLogicalType = data_buffer[5];
    osal_nv_write( ZCD_NV_LOGICAL_TYPE, 0, sizeof(zgDeviceLogicalType), &zgDeviceLogicalType);
    restore_factory_setting();
  }
  else if( address == MODBUS_CHANNEL_LIST_HI)
  {
    zgDefaultChannelList = BUILD_UINT32( 0, 0, data_buffer[5], data_buffer[4]);
  }
  else if( address == MODBUS_CHANNEL_LIST_LO)
  {
    zgDefaultChannelList = BUILD_UINT32( data_buffer[5], data_buffer[4], 0, 0 );
    osal_nv_write( ZCD_NV_CHANLIST, 0, sizeof(zgDefaultChannelList), &zgDefaultChannelList);
  }
  else if( address == MODBUS_FACTORY_RESTORE)
  {
    if(data_buffer[5] == 1)
    {
      restore_factory_setting();
    }
  }
}
/******************************************************************************
 * @fn      modbus_multi_write
 */
void modbus_multi_write( uint8 *data_buffer, uint8 len)
{
  uint8 address;
  
  address = BUILD_UINT16( data_buffer[3], data_buffer[2]);
  
  if( address == MODBUS_SECURITY_KEY_START)
  {
    if(data_buffer[6]>=32)
    {
      for(uint8 i=0; i<16; i++)
      {
      //  osal_memcpy(defaultKey, data_buffer[7+i*2], 1);
        defaultKey[i] = data_buffer[7+i*2];
        zgPreconfigKeyInit( TRUE );

        // Initialize NV items for all Keys: NWK, APS, TCLK and Master
        ZDSecMgrInitNVKeyTables( TRUE );
      }
    }
  }
}
/******************************************************************************
 * @fn      modbus_insert_msg
 */
void modbus_insert_msg( uint8 *pBuf, uint8 len)
{
  uint8 index = 0;
  afAddrType_t destAddr;
  destAddr.addrMode = afAddr16Bit;
  destAddr.addr.shortAddr = 0;                
  destAddr.endPoint = 10;
  signalStrength_t* pSigStren;
    
  initCRC16();
//  uint8 *pInsertBuf = osal_mem_alloc( modbusDataLength*2);
  if( firstAddr < 21)
    index = 45 - 2*firstAddr;
  else
    //index = (modbusStartAddr-20)*2 + 1;
    index = 3;
  
  for (uint8 i=0; i<modbusDataLength; i++)
  {
    if( i+modbusStartAddr == MODBUS_PANID)
    {
      pBuf[index++] = HI_UINT16( _NIB.nwkPanId);
      pBuf[index++] = LO_UINT16( _NIB.nwkPanId);
    }
    else if( i + modbusStartAddr == MODBUS_DEVICE_TYPE)
    {
      pBuf[index++] = ZERO;
      pBuf[index++] = zgDeviceLogicalType;
    }
    else if( i + modbusStartAddr == MODBUS_CHANNEL_LIST_HI)
    {
      pBuf[index++] = BREAK_UINT32(zgDefaultChannelList, 3);
      pBuf[index++] = BREAK_UINT32(zgDefaultChannelList, 2);
    }
    else if( i + modbusStartAddr == MODBUS_CHANNEL_LIST_LO)
    {
      pBuf[index++] = BREAK_UINT32(zgDefaultChannelList, 1);
      pBuf[index++] = BREAK_UINT32(zgDefaultChannelList, 0);
    }
    else if( i + modbusStartAddr == MODBUS_SOFTWARE_REV)
    {
      pBuf[index++] = ZERO;
      pBuf[index++] = ZIG_SOFTWARE_VER;
    }
    else if( (i + modbusStartAddr >= MODBUS_EXTENDED_ADDR_HI) && (i + modbusStartAddr <= MODBUS_EXTENDED_ADDR_LO))
    {
      pBuf[index++] = ZERO;
      pBuf[index++] = aExtendedAddress[ i+modbusStartAddr-MODBUS_EXTENDED_ADDR_HI];
    }
    else if( (i + modbusStartAddr >= MODBUS_SECURITY_KEY_START) && (i + modbusStartAddr <= MODBUS_SECURITY_KEY_END))
    {
      pBuf[index++] = ZERO;
      pBuf[index++] = defaultKey[ i+modbusStartAddr-MODBUS_SECURITY_KEY_START];
    }
    else if( i + modbusStartAddr == MODBUS_TSTAT_NUM)
    {
      pBuf[index++] = ZERO;
      pBuf[index++] = numSignalStren;
    }
    else if( (i + modbusStartAddr >= MODBUS_FIRST_TSTAT_ID) && ( i+ modbusStartAddr <= MODBUS_LAST_TSTAT_ID))
    {
      if( i + modbusStartAddr == MODBUS_FIRST_TSTAT_ID)
      {
        pSigStren = pSignalStren;
      }
      else
      {
        pSigStren = pSignalStren;
        for( uint8 j=0; j<((i+modbusStartAddr)-MODBUS_FIRST_TSTAT_ID); j++)
        {
          if(pSigStren)
            pSigStren = pSigStren->next;
        }
      }
      pBuf[index++] = ZERO;
      if(pSigStren != NULL)
      {
        pBuf[index++] = pSigStren->modbus_id;
      }
      else
        pBuf[index++] = ZERO;
    }
    else if( (i + modbusStartAddr >= MODBUS_FIRST_SIG_STREN) && ( i+ modbusStartAddr <= MODBUS_LAST_SIG_STREN))
    {
      if( i + modbusStartAddr == MODBUS_FIRST_SIG_STREN)
      {
        pSigStren = pSignalStren;
      }
      else
      {
        pSigStren = pSignalStren;
        for( uint8 j=0; j<((i+modbusStartAddr)-MODBUS_FIRST_SIG_STREN); j++)
        {
          if(pSigStren)
            pSigStren = pSigStren->next;
        }
      }
      pBuf[index++] = ZERO;
      if(pSigStren != NULL)
      {
        pBuf[index++] = pSigStren->rssi;
      }
      else
        pBuf[index++] = ZERO;
    }
    else
    {
      pBuf[index++] = ZERO;
      pBuf[index++] = ZERO;
    }
  }
  
  for( uint8 i=0; i<len-2; i++)
    CRC16_byte(pBuf[i]);
  
  pBuf[len-2] = CRChi;
  pBuf[len-1] = CRClo;
  
  AF_DataRequest( &destAddr, &temco_epDesc,
                     TEMCO_CLUSTERID,
                     len,
                     pBuf,
                     &temcoApp_TransID,
                     AF_DISCV_ROUTE,
                     AF_DEFAULT_RADIUS );
  
  modbusStartAddr = 0;
  modbusDataLength = 0;
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
  signalStrength_t* pSigStren;
  
  afAddrType_t destAddr;
  destAddr.addrMode = afAddrBroadcast;
  destAddr.addr.shortAddr = 0xffff;                
  destAddr.endPoint = 10;
  
  address = BUILD_UINT16( data_buffer[3], data_buffer[2]);
  
  if( data_buffer[0] == modbus_id)
  {
    if(data_buffer[1] == MODBUS_SINGLE_WRITE)
    {
      HalUARTWrite( 0, data_buffer, len);
      if (address == MODBUS_DEVICE_TYPE)
      {
        zgDeviceLogicalType = data_buffer[5];
        osal_nv_write( ZCD_NV_LOGICAL_TYPE, 0, sizeof(zgDeviceLogicalType), &zgDeviceLogicalType);
        restore_factory_setting();
      }
      else if( address == MODBUS_FACTORY_RESTORE)
      {
        if( address == MODBUS_FACTORY_RESTORE)
        {
          if(data_buffer[5] == 1)
          {
            restore_factory_setting();
          }
        }
      }
    }
    else if(data_buffer[1] == MODBUS_MULTI_WRITE)
    {
      
    }
    else if( data_buffer[1] == MODBUS_SINGLE_READ)
    {
      num = data_buffer[5];
      modbus_send_byte( data_buffer[0], CRC_NO);
      modbus_send_byte( data_buffer[1], CRC_NO);
      modbus_send_byte( num*2, CRC_NO);
      
      for( i = 0; i < num; i++)
      {
        if( i + address == MODBUS_PANID)
        {
          tempByte = HI_UINT16( _NIB.nwkPanId);
          modbus_send_byte( tempByte, CRC_NO);
          tempByte = LO_UINT16( _NIB.nwkPanId);
          modbus_send_byte( tempByte, CRC_NO);
        }
        else if( i + address == MODBUS_DEVICE_TYPE)
        {
          modbus_send_byte( zero, CRC_NO);
          modbus_send_byte( zgDeviceLogicalType, CRC_NO);
        }
        else if( i + address == MODBUS_CHANNEL_LIST_HI)
        {
          tempByte = HI_UINT16(zgConfigPANID);
          modbus_send_byte( zero, CRC_NO);
          tempByte = LO_UINT16(zgConfigPANID);
          modbus_send_byte( zero, CRC_NO);
        }
        else if( i + address == MODBUS_CHANNEL_LIST_LO)
        {
          tempByte = HI_UINT16(zgConfigPANID);
          modbus_send_byte( zero, CRC_NO);
          tempByte = LO_UINT16(zgConfigPANID);
          modbus_send_byte( zero, CRC_NO);
        }
        else if( i + address == MODBUS_SOFTWARE_REV)
        {
          modbus_send_byte( zero, CRC_NO);
          modbus_send_byte( zSoftwareRev, CRC_NO);
        }
        else if( (i + address >= MODBUS_EXTENDED_ADDR_HI) && (i + address <= MODBUS_EXTENDED_ADDR_LO))
        {
          modbus_send_byte( zero, CRC_NO);
          modbus_send_byte( aExtendedAddress[ i+modbusStartAddr-MODBUS_EXTENDED_ADDR_HI], CRC_NO);
        }
        else if( (i + address >= MODBUS_SECURITY_KEY_START) && (i + address <= MODBUS_SECURITY_KEY_END))
        {
          modbus_send_byte( zero, CRC_NO);
          modbus_send_byte( defaultKey[ i+modbusStartAddr-MODBUS_SECURITY_KEY_START], CRC_NO);
        }
        else if( i + address == MODBUS_TSTAT_NUM)
        {
          modbus_send_byte( zero, CRC_NO);
          modbus_send_byte( numSignalStren, CRC_NO);
        }
        else if( (i + address >= MODBUS_FIRST_TSTAT_ID) && ( i+ address <= MODBUS_LAST_TSTAT_ID))
        {
          if( i + address == MODBUS_FIRST_TSTAT_ID)
          {
            pSigStren = pSignalStren;
          }
          else
          {
            pSigStren = pSignalStren;
            for( uint8 j=0; j<((i+address)-MODBUS_FIRST_TSTAT_ID); j++)
            {
              if(pSigStren)
                pSigStren = pSigStren->next;
            }
          }
          modbus_send_byte( zero, CRC_NO);
          if(pSigStren != NULL)
          {
            modbus_send_byte( pSigStren->modbus_id, CRC_NO);
          }
          else
            modbus_send_byte( zero, CRC_NO);
        }
        else if( (i + address >= MODBUS_FIRST_SIG_STREN) && ( i+ address <= MODBUS_LAST_SIG_STREN))
        {
          if( i + address == MODBUS_FIRST_SIG_STREN)
          {
            pSigStren = pSignalStren;
          }
          else
          {
            pSigStren = pSignalStren;
            for( uint8 j=0; j<((i+address)-MODBUS_FIRST_SIG_STREN); j++)
            {
              if(pSigStren)
                pSigStren = pSigStren->next;
            }
          }
          modbus_send_byte( zero, CRC_NO);
          if(pSigStren != NULL)
          {
            modbus_send_byte( pSigStren->rssi, CRC_NO);
          }
          else
            modbus_send_byte( zero, CRC_NO);
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

  else
  {
    AF_DataRequest( &destAddr, &temco_epDesc,
                     TEMCO_CLUSTERID,
                     len,
                     data_buffer,
                     &temcoApp_TransID,
                     AF_DISCV_ROUTE,
                     AF_DEFAULT_RADIUS );
  }
}


