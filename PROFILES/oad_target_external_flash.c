/*******************************************************************************

 @file  oad_target_external_flash.c

 @brief This file contains the external flash target implementation of the
        OAD profile.

 Group: WCS, BTS
 Target Device: CC2650, CC2640, CC1350

 *******************************************************************************
 
 Copyright (c) 2014-2016, Texas Instruments Incorporated
 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions
 are met:

 *  Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.

 *  Redistributions in binary form must reproduce the above copyright
    notice, this list of conditions and the following disclaimer in the
    documentation and/or other materials provided with the distribution.

 *  Neither the name of Texas Instruments Incorporated nor the names of
    its contributors may be used to endorse or promote products derived
    from this software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 *******************************************************************************
 Release Name: ble_sdk_2_02_01_18
 Release Date: 2016-10-26 15:20:04
 ******************************************************************************/

/*******************************************************************************
 * INCLUDES
 */
#ifdef FEATURE_OAD
#include <string.h>
#include "hal_board.h"
#include "oad_target.h"
#include <ti/mw/extflash/ExtFlash.h>
#include "ext_flash_layout.h"

/*******************************************************************************
 * Constants and macros
 */
#define PROG_BUF_SIZE             16
#define PAGE_0                    0
#define PAGE_1                    1
#define PAGE_31                   31

#define APP_IMAGE_START           0x1000
#define BOOT_LOADER_START         0x1F000

#define MAX_BLOCKS                (EFL_SIZE_IMAGE_APP / OAD_BLOCK_SIZE)

// Dummy header.
#if defined (__IAR_SYSTEMS_ICC__)
#pragma location=".checksum"
// 4 bytes for CRC and CRC Shadow.
const uint8_t _chksum[4] = {
  0xFF, 0xFF, 0xFF, 0xFF
};
// 12 byte for image header immediately following CRC/CRC Shadow.
#pragma location="IMAGE_HEADER"
const uint8_t _imgHdr[12] = {
  0xFF, 0xFF, 0xFF, 0xFF,
  0xFF, 0xFF, 0xFF, 0xFF,
  0xFF, 0xFF, 0xFF, 0xFF
};
#elif defined __TI_COMPILER_VERSION__
#pragma DATA_SECTION(_imgHdr, ".imgHdr")
#pragma RETAIN(_imgHdr)
// For CCS, this is the first 16 bytes at a 0 byte offset from the start of the
// OAD Image.
const uint8_t _imgHdr[16] = {
  0xFF, 0xFF, 0xFF, 0xFF,
  0xFF, 0xFF, 0xFF, 0xFF,
  0xFF, 0xFF, 0xFF, 0xFF,
  0xFF, 0xFF, 0xFF, 0xFF
};
#endif

/*******************************************************************************
 * PRIVATE VARIABLES
 */
static bool isOpen = false;
static ExtImageInfo_t imgInfo;

/*******************************************************************************
 * PRIVATE FUNCTIONS
 */

/*******************************************************************************
 * FUNCTIONS
 */

/*******************************************************************************
 * @fn      OADTarget_open
 *
 * @brief   Open an OAD target for download.
 *
 * @param   none
 *
 * @return  TRUE if OAD target successfully opened
 */
uint8_t OADTarget_open(void)
{
    if (!isOpen)
    {
        isOpen = ExtFlash_open();
    }

    return isOpen ? TRUE : FALSE;
}


/*******************************************************************************
 * @fn      OADTarget_close
 *
 * @brief   Close an OAD target after a download has finished
 *
 * @param   none
 *
 * @return  none
 */
void OADTarget_close(void)
{
  if (isOpen)
  {
    isOpen = false;
    ExtFlash_close();
  }
}

/*******************************************************************************
 * @fn      OADTarget_hasExternalFlash
 *
 * @brief   Check if the target has external flash
 *
 * @param   none
 *
 * @return  always TRUE
 */
uint8_t OADTarget_hasExternalFlash(void)
{
  return true;
}

/*******************************************************************************
 * @fn      OADTarget_getCurrentImageHeader
 *
 * @brief   Get the header of the running image.
 *
 * @param   pHdr - pointer to store running image header.
 *
 * @return  TRUE if flash is successfully opened
 */
uint8_t OADTarget_getCurrentImageHeader(img_hdr_t *pHdr)
{
  uint8_t ret;

  ret = ExtFlash_open() ? TRUE : FALSE;

  if (ret)
  {
    // N.B: it is assumed that OADTarget_storeImageHeader() has been called
    // prior to this function.
    uint32_t metaDataAddr;
    ExtImageInfo_t tempHdr;

    if (imgInfo.imgType == EFL_OAD_IMG_TYPE_STACK
        || imgInfo.imgType == EFL_OAD_IMG_TYPE_NP)
    {
      metaDataAddr = EFL_IMAGE_INFO_ADDR_BLE;
    }
    else // Assume imgInfo.imgType == EFL_OAD_IMG_TYPE_APP
    {
      metaDataAddr = EFL_IMAGE_INFO_ADDR_APP;
    }

    ExtFlash_read(metaDataAddr, sizeof(ExtImageInfo_t), (uint8_t*)&tempHdr);
    ExtFlash_close();

    pHdr->len = tempHdr.len;

    // In case metadata does not exist, use 0x00
    pHdr->ver = tempHdr.ver == 0xFF ? 0x00 : tempHdr.ver;
    memcpy(pHdr->uid,tempHdr.uid,sizeof(tempHdr.uid));
    pHdr->res[0] = HI_UINT16(tempHdr.addr);
    pHdr->res[1] = LO_UINT16(tempHdr.addr);
    pHdr->res[2] = tempHdr.imgType;
    pHdr->res[3] = 0xFF;
  }

  return ret;
}

/*******************************************************************************
 * @fn      OADTarget_validateNewImage
 *
 * @brief   Determine if a new image should be downloaded or not based on
 *          target specific criteria.
 *
 * @param   pValue - pointer to new Image header information
 * @param   pCur - pointer to contents of current image header
 * @param   blkTot - total number of blocks comprising new image.
 *
 * @return  TRUE to begin OAD otherwise FALSE to reject the image.
 */
uint8_t OADTarget_validateNewImage(uint8_t *pValue, img_hdr_t *pCur,
                                   uint16_t blkTot)
{
  img_hdr_t *pNew;
  uint32_t addr;
  uint8_t imgType;
  uint8_t valid;

  pNew = (img_hdr_t *)pValue;
  addr = BUILD_UINT16(pNew->res[0],pNew->res[1]) * EFL_OAD_ADDR_RESOLUTION;
  imgType = pNew->res[2];
  valid = FALSE;

  // Check if number of blocks make sense
  if (blkTot > MAX_BLOCKS || blkTot == 0)
  {
    return FALSE;
  }

  //Note that network processor images can/will take up the entire internal
  //flash space
  if (imgType != EFL_OAD_IMG_TYPE_NP
      && (addr < APP_IMAGE_START || addr > BOOT_LOADER_START))
  {
    return FALSE;
  }

  // Image type 'APP' must start at 0x1000
  if (addr != APP_IMAGE_START && imgType == EFL_OAD_IMG_TYPE_APP)
  {
    return FALSE;
  }

  // Check if current header is invalid
  if (pCur->ver == 0xFFFF || pCur->ver == 0x0000)
  {
    // Accept the image.
	return TRUE;
  }

  // By default, accept an image if version is 0.
  if (pNew->ver == 0)
  {
    valid = TRUE;
  }

  // If not already validated, check if new image is a later version than the
  // current image.
  if (!valid)
  {
    valid =  pNew->ver > pCur->ver;
  }

  return valid;
}

/*******************************************************************************
 * @fn      OADTarget_storeImageHeader
 *
 * @brief   Store the image header of the new image
 *
 * @param   pValue - pointer to the new image header
 *
 * @return  none
 */
void OADTarget_storeImageHeader(uint8_t *pValue)
{
  img_hdr_t* pNew;

  pNew = (img_hdr_t*)(pValue +4);

  // Storage image header (written to external flash before reboot)
  imgInfo.crc[0] = BUILD_UINT16(pValue[0],pValue[1]);
  imgInfo.crc[1] = BUILD_UINT16(pValue[2],pValue[3]);
  imgInfo.addr = BUILD_UINT16(pNew->res[0],pNew->res[1]);
  imgInfo.ver = pNew->ver;
  imgInfo.len = pNew->len;
  memcpy(imgInfo.uid,pNew->uid,sizeof(imgInfo.uid));
  imgInfo.imgType = pNew->res[2];
}

/*******************************************************************************
 * @fn      OADTarget_imageAddress
 *
 * @brief   Get the address to store the new image
 *
 * @param   pValue - pointer to the new image header
 *
 * @param   pHdr - pointer to the current image header
 *
 * @return  address
 */
uint32_t OADTarget_imageAddress(uint8_t *pValue)
{
  img_hdr_t *pNew;
  uint32_t extAddr;

  pNew = (img_hdr_t *)pValue;

  switch (pNew->res[2])
  {
    // Application start at the beginning of external flash.
    case EFL_OAD_IMG_TYPE_APP:
      extAddr = EFL_ADDR_IMAGE_APP;
      break;

    // All other images are placed into the next available image slot.
    default:
      extAddr = EFL_ADDR_IMAGE_BLE;
      break;
  }

  return extAddr;
}

/*******************************************************************************
 * @fn      OADTarget_getCrc
 *
 * @brief   Get the CRC array from the image that is being downloaded
 *
 * @param   pCrc - pointer to the new image header
 *
 * @return  None
 */
void OADTarget_getCrc(uint16_t *pCrc)
{
  // Copy CRC information from header information.
  pCrc[0] = imgInfo.crc[0];
  pCrc[1] = imgInfo.crc[1];
}

/*******************************************************************************
 * @fn      OADTarget_setCrc
 *
 * @brief   Set the CRC shadow of the downloaded image.
 *
 * @param   pCrc - pointer to the new image header
 *
 * @return  Non
 */
void OADTarget_setCrc(uint16_t *pCrc)
{
  // Update shadow CRC to verify
  imgInfo.crc[1] = pCrc[1];
}

/*******************************************************************************
 * @fn      OADTarget_enableCache
 *
 * @brief   Prepares system for a write to flash, if necessary.
 *
 * @param   None.
 *
 * @return  None.
 */
void OADTarget_enableCache(void)
{
  // Do nothing.
}

/*******************************************************************************
 * @fn      OADTarget_disableCache
 *
 * @brief   Resumes system after a write to flash, if necessary.
 *
 * @param   None.
 *
 * @return  None.
 */
void OADTarget_disableCache(void)
{
  // Do nothing.
}

/*******************************************************************************
 * @fn      OADTarget_readFlash
 *
 * @brief   Read data from flash.
 *
 * @param   page   - page to read from in flash
 * @param   offset - offset into flash page to begin reading
 * @param   pBuf   - pointer to buffer into which data is read.
 * @param   len    - length of data to read in bytes.
 *
 * @return  None.
 */
void OADTarget_readFlash(uint8_t page, uint32_t offset, uint8_t *pBuf,
                         uint16_t len)
{
  ExtFlash_read(FLASH_ADDRESS(page,offset), len, pBuf);
}

/*******************************************************************************
 * @fn      OADTarget_writeFlash
 *
 * @brief   Write data to flash.
 *
 * @param   page   - page to write to in flash
 * @param   offset - offset into flash page to begin writing
 * @param   pBuf   - pointer to buffer of data to write
 * @param   len    - length of data to write in bytes
 *
 * @return  None.
 */
void OADTarget_writeFlash(uint8_t page, uint32_t offset, uint8_t *pBuf,
                          uint16_t len)
{
  ExtFlash_write(FLASH_ADDRESS(page,offset), len, pBuf);
}

/*********************************************************************
 * @fn      OADTarget_eraseFlash
 *
 * @brief   Erase selected flash page.
 *
 * @param   page - the page to erase.
 *
 * @return  None.
 */
void OADTarget_eraseFlash(uint8_t page)
{
  ExtFlash_erase(FLASH_ADDRESS(page,0), HAL_FLASH_PAGE_SIZE);
}

/*********************************************************************
 * @fn      OADTarget_systemReset
 *
 * @brief   Prepare system for a reset and trigger a reset to the boot
 *          image manager. If the image contains a new BIM, copy it to
 *          page '0' before resetting. This is a critical operation.
 *
 * @param   None.
 *
 * @return  None.
 */
void OADTarget_systemReset(void)
{
  // Reset to the bootloader.
  HAL_SYSTEM_RESET();
}

/*******************************************************************************
 * @fn      saveImageInfo
 *
 * @brief   Save image information in the meta-data area
 *
 * @return  none
 */
void saveImageInfo(void)
{
  uint32_t addr;

  if (imgInfo.imgType == EFL_OAD_IMG_TYPE_APP)
  {
    addr = EFL_IMAGE_INFO_ADDR_APP;
  }
  else
  {
    addr = EFL_IMAGE_INFO_ADDR_BLE;
  }

  // Erase old meta data.
  ExtFlash_erase(addr, HAL_FLASH_PAGE_SIZE);

  // Set status so that bootloader pull in the new image.
  imgInfo.status = 0xFF;

  // Write new meta data.
  ExtFlash_write(addr, sizeof(ExtImageInfo_t),
                (uint8_t*)&imgInfo);
}

/*******************************************************************************
 * @fn      getImageFlag
 *
 * @brief   Get the image type flag.
 *
 * @return  Image type or 0 if unknown.
 */
uint8_t getImageFlag(void)
{

  uint8_t flag = 0;

  if (imgInfo.imgType == EFL_OAD_IMG_TYPE_APP)
  {
    flag = OAD_IMG_APP_FLAG;
  }
  else if (imgInfo.imgType == EFL_OAD_IMG_TYPE_STACK)
  {
    flag = OAD_IMG_STACK_FLAG;
  }
  else if (imgInfo.imgType == EFL_OAD_IMG_TYPE_NP)
  {
    flag = OAD_IMG_NP_FLAG;
  }

  return flag;
}

#endif //FEATURE_OAD

