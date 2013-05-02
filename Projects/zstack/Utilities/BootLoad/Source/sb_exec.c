/**************************************************************************************************
  Filename:       sb_exec.c
  Revised:        $Date: 2012-03-27 14:53:26 -0700 (Tue, 27 Mar 2012) $
  Revision:       $Revision: 29910 $

  Description:    Serial Bootloader Executive.

  Copyright 2009-2011 Texas Instruments Incorporated. All rights reserved.

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
**************************************************************************************************/

/* ------------------------------------------------------------------------------------------------
 *                                          Includes
 * ------------------------------------------------------------------------------------------------
 */

#include "hal_board_cfg.h"
#include "hal_flash.h"
#include "hal_types.h"
#include "sb_exec.h"
#include "sb_main.h"

/* ------------------------------------------------------------------------------------------------
 *                                          Constants
 * ------------------------------------------------------------------------------------------------
 */

#if !defined MT_SYS_OSAL_NV_READ_CERTIFICATE_DATA
#define MT_SYS_OSAL_NV_READ_CERTIFICATE_DATA  FALSE
#endif

/* ------------------------------------------------------------------------------------------------
 *                                       Local Variables
 * ------------------------------------------------------------------------------------------------
 */

static uint8 sbBuf[SB_BUF_SIZE], sbCmd2, sbIdx, sbLen, sbSte, CRClo, CRChi;
//static uint8 sbCmd1, sbFcs;
static uint8 const auchCRCHi[256] = {
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


static uint8 const auchCRCLo[256] = {

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
/* ------------------------------------------------------------------------------------------------
 *                                       Local Functions
 * ------------------------------------------------------------------------------------------------
 */
static void CRC16_byte( uint8 ch);
static void initCRC16( void);
static uint8 sbCmnd(void);
//static void sbResp(uint8 rsp, uint8 len);
//static uint16 calcCRC(void);
//static uint16 runPoly(uint16 crc, uint8 val);

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

/**************************************************************************************************
 * @fn          sbExec
 *
 * @brief       Boot Loader main executive processing.
 *
 * input parameters
 *
 * None.
 *
 * output parameters
 *
 * None.
 *
 * @return      TRUE if sbCmnd() returns TRUE, indicating that an SB_ENABLE_CMD succeeded;
 *              FALSE otherwise.
 **************************************************************************************************
 */
uint8 sbExec(void)
{
  uint8 ch, rtrn = FALSE;

  while (SB_RX(&ch))
  {
    sbBuf[sbSte + sbIdx] = ch;
    switch (sbSte)
    {
    case SB_SOF_STATE:
      if (SB_SOF == ch)
      {
        sbSte = SB_LEN_STATE;
      }
      break;

    case SB_LEN_STATE:
      // 改为modbus命令
      sbCmd2 = ch;
      // 改为读取modbus地址
      if( (sbCmd2 != 0x10) && (sbCmd2 != 0x06) && (sbCmd2 != 0x03))
      {
        sbSte = SB_SOF_STATE;
      }
      else
        sbSte = SB_CMD1_STATE;
//      sbFcs = 0;
//      sbSte = ((sbLen = ch) >= SB_BUF_SIZE) ? SB_SOF_STATE : SB_CMD1_STATE;
      break;

    case SB_CMD1_STATE:
#if 0
      sbCmd1 = ch;
#endif
      sbSte = SB_CMD2_STATE;
      break;

    case SB_CMD2_STATE:
#if 0
      sbCmd2 = ch;
      sbSte = (sbLen) ? SB_DATA_STATE : SB_FCS_STATE;
#endif
      sbSte = SB_MODBUS_LEN_1;
      break;

    case SB_MODBUS_LEN_1:
      sbSte = SB_MODBUS_LEN_2;
      break;
      
    case SB_MODBUS_LEN_2:
      if( sbCmd2 == 0x10)
      {
        sbLen = ch;
        sbSte = SB_UNKNOWN_BYTE;
      }
      else
      {
        sbLen = 0;
        sbSte = SB_FCS_STATE_1;
      }
      break;
      
    case SB_UNKNOWN_BYTE:
      sbSte = SB_DATA_STATE;
      break;
      
    case SB_DATA_STATE:
      if( sbLen == 0)
      {
        sbSte = SB_FCS_STATE_1;
//        SB_TX( "1", 1);
      }
      else if (++sbIdx >= sbLen)
      {
        sbSte = SB_FCS_STATE_1;
//        SB_TX( "2", 1);
      }
#if 0
      else
      {
        sbSte = SB_SOF_STATE;
        SB_TX( "3", 1);
      }
#endif
      break;
      
    case SB_FCS_STATE_1:
      if( sbCmd2 == 0x06)
      {
        sbBuf[6] = ch;
      }
      sbSte = SB_FCS_STATE_2;
      break;
      
    case SB_FCS_STATE_2:
      if( sbCmd2 == 0x06)
      {
        sbBuf[7] = ch;
      }
      //if ((sbFcs == ch) && (sbCmd1 == SB_RPC_SYS_BOOT))
      {
        rtrn = sbCmnd();
      }
      //else
      {
        // TODO - RemoTI did not have here or on bad length - adding could cause > 1 SB_INVALID_FCS
        //        for a single data packet which could put out of sync with PC for awhile or
        //        infinte, depending on PC-side?
        // sbResp(SB_INVALID_FCS, 1);
      }

      sbSte = sbIdx = 0;
      break;

    default:
      break;
    }
//    sbFcs ^= ch;
  }

  return rtrn;
}

/**************************************************************************************************
 * @fn          sbImgValid
 *
 * @brief       Check validity of the run-code image.
 *
 * input parameters
 *
 * None.
 *
 * output parameters
 *
 * None.
 *
 * @return      TRUE or FALSE for image valid.
 **************************************************************************************************
 */
#if 0
uint8 sbImgValid(void)
{
  uint16 crc[2];

  HalFlashRead(HAL_SB_CRC_ADDR / HAL_FLASH_PAGE_SIZE,
               HAL_SB_CRC_ADDR % HAL_FLASH_PAGE_SIZE,
               (uint8 *)crc, sizeof(crc));

  if ((crc[1] == 0x0000) || (crc[0] == 0x0000) || (crc[0] == 0xFFFF))
  {
    return FALSE;
  }
  else if ((crc[1] == 0xFFFF) && (crc[1] != crc[0]))
  {
    crc[0] = 0xFFFF;  // Don't write any zero a second time.
    crc[1] = calcCRC();
    HalFlashWrite((HAL_SB_CRC_ADDR / HAL_FLASH_WORD_SIZE), (uint8 *)crc, 1);
    HalFlashRead(  HAL_SB_CRC_ADDR / HAL_FLASH_PAGE_SIZE,
                   HAL_SB_CRC_ADDR % HAL_FLASH_PAGE_SIZE,
                   (uint8 *)crc, sizeof(crc));
  }

  return (crc[0] == crc[1]);
}
#endif
/**************************************************************************************************
 * @fn          sbCmnd
 *
 * @brief       Act on the SB command and received buffer.
 *
 * input parameters
 *
 * None.
 *
 * output parameters
 *
 * None.
 *
 * @return      TRUE to indicate that the SB_ENABLE_CMD command was successful; FALSE otherwise.
 **************************************************************************************************
 */
static uint8 sbCmnd(void)
{
//  uint16 tmp = BUILD_UINT16(sbBuf[SB_DATA_STATE], sbBuf[SB_DATA_STATE+1]) + SB_IMG_OSET; 
  // 改为符合MODBUS的协议，此处地址因分块需除以4才是芯片地址
  uint16 tmp = BUILD_UINT16( sbBuf[3], sbBuf[2])/4 + SB_IMG_OSET;  // 烧写地址
//  uint16 crc[2];
//  uint8 len = 1;
//  uint8 rsp = SB_SUCCESS;
  uint8 rtrn = FALSE;

  switch (sbCmd2)
  {
  case SB_HANDSHAKE_CMD:
    break;

    // 命令码改为符合modbus标准的多写命令
  case SB_WRITE_CMD:
//    SB_TX( "aa", 2);
    if ((tmp % SB_WPG_SIZE) == 0)
    {
      HalFlashErase(tmp / SB_WPG_SIZE);
    }
    // 去掉mobus消息头，才是烧写的数据
    HalFlashWrite(tmp, sbBuf+7, SB_RW_BUF_LEN / HAL_FLASH_WORD_SIZE);
//    SB_TX( sbBuf+7, 128);
    initCRC16();
    for( int i=0; i<6; i++)
    {
      CRC16_byte(sbBuf[i]);
    }
    CRC16_byte(sbBuf[sbLen+7]);
    CRC16_byte(sbBuf[sbLen+8]);
    sbBuf[6] = CRChi;
    sbBuf[7] = CRClo;
    SB_TX( sbBuf, 8);
    break;

  case SB_READ_CMD:
#if 0
#if !MT_SYS_OSAL_NV_READ_CERTIFICATE_DATA
    if ((tmp / (HAL_FLASH_PAGE_SIZE / 4)) >= HAL_NV_PAGE_BEG)
    {
      rsp = SB_FAILURE;
      break;
    }
#endif
    HalFlashRead(tmp / (HAL_FLASH_PAGE_SIZE / 4),
                 (tmp % (HAL_FLASH_PAGE_SIZE / 4)) << 2,
                 sbBuf + SB_DATA_STATE + 3, SB_RW_BUF_LEN);
    sbBuf[SB_DATA_STATE+2] = sbBuf[SB_DATA_STATE+1];
    sbBuf[SB_DATA_STATE+1] = sbBuf[SB_DATA_STATE];
    len = SB_RW_BUF_LEN + 3;
#endif
    break;

  case SB_ENABLE_CMD:
#if 0
    HalFlashRead(HAL_SB_CRC_ADDR / HAL_FLASH_PAGE_SIZE,
                 HAL_SB_CRC_ADDR % HAL_FLASH_PAGE_SIZE,
                 (uint8 *)crc, sizeof(crc));

    // Bootload master must have verified extra checks to be issuing the SB_ENABLE_CMD.
    //if ((crc[0] != crc[1]) && (crc[0] != 0xFFFF) && (crc[0] != 0x0000))
    if (crc[1] != crc[0])
    {
      crc[1] = crc[0];
      HalFlashWrite((HAL_SB_CRC_ADDR / HAL_FLASH_WORD_SIZE), (uint8 *)crc, 1);
      HalFlashRead(  HAL_SB_CRC_ADDR / HAL_FLASH_PAGE_SIZE,
                     HAL_SB_CRC_ADDR % HAL_FLASH_PAGE_SIZE,
                     (uint8 *)crc, sizeof(crc));
    }

    // Bootload master must have verified extra checks to be issuing the SB_ENABLE_CMD.
    //if ((crc[0] == crc[1]) && (crc[0] != 0xFFFF) && (crc[0] != 0x0000))
    if (crc[0] == crc[1])
    {
      rtrn = TRUE;
    }
    else
    {
      rsp = SB_VALIDATE_FAILED;
    }
#endif
    // 到达文件尾
    if( (sbBuf[5] == 0x01) && (sbBuf[3] == 0x10))
    {
      rtrn = TRUE;
    }
    SB_TX( sbBuf, 8);
    break;

  default:
    break;
  }

//  sbResp(rsp, len);
  return rtrn;
}

#if 0


/**************************************************************************************************
 * @fn          sbResp
 *
 * @brief       Make the SB response.
 *
 * input parameters
 *
 * @param       rsp - The byte code response to send.
 * @param       len - The data length of the response.
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 **************************************************************************************************
 */
static void sbResp(uint8 rsp, uint8 len)
{
  int8 idx;

  sbBuf[SB_CMD2_STATE] |= 0x80;
  sbBuf[SB_DATA_STATE] = rsp;
  sbBuf[SB_LEN_STATE] = len;
  rsp = len ^ SB_RPC_SYS_BOOT;
  len += SB_FCS_STATE-1;

  for (idx = SB_CMD2_STATE; idx < len; idx++)
  {
    rsp ^= sbBuf[idx];
  }
  sbBuf[idx++] = rsp;

  SB_TX(sbBuf, idx);
}

/**************************************************************************************************
 * @fn          calcCRC
 *
 * @brief       Run the CRC16 Polynomial calculation over the RC image.
 *
 * input parameters
 *
 * None.
 *
 * output parameters
 *
 * None.
 *
 * @return      The CRC16 calculated.
 **************************************************************************************************
 */
static uint16 calcCRC(void)
{
  uint32 addr;
  uint16 crc = 0;

  // Run the CRC calculation over the active body of code.
  for (addr = HAL_SB_IMG_ADDR; addr < HAL_SB_IMG_ADDR + HAL_SB_IMG_SIZE; addr++)
  {
    if (addr == HAL_SB_CRC_ADDR)
    {
      addr += 3;
    }
    else
    {
      uint8 buf;
      HalFlashRead(addr / HAL_FLASH_PAGE_SIZE, addr % HAL_FLASH_PAGE_SIZE, &buf, 1);
      crc = runPoly(crc, buf);
    }
  }

  // IAR note explains that poly must be run with value zero for each byte of crc.
  crc = runPoly(crc, 0);
  crc = runPoly(crc, 0);

  return crc;
}
/**************************************************************************************************
 * @fn          runPoly
 *
 * @brief       Run the CRC16 Polynomial calculation over the byte parameter.
 *
 * input parameters
 *
 * @param       crc - Running CRC calculated so far.
 * @param       val - Value on which to run the CRC16.
 *
 * output parameters
 *
 * None.
 *
 * @return      crc - Updated for the run.
 **************************************************************************************************
 */
static uint16 runPoly(uint16 crc, uint8 val)
{
  const uint16 poly = 0x1021;
  uint8 cnt;

  for (cnt = 0; cnt < 8; cnt++, val <<= 1)
  {
    uint8 msb = (crc & 0x8000) ? 1 : 0;

    crc <<= 1;
    if (val & 0x80)  crc |= 0x0001;
    if (msb)         crc ^= poly;
  }

  return crc;
}
#endif

/**************************************************************************************************
*/
