/******************************************************************************

 @file  oad.c

 @brief This file contains OAD Target implementation.

 Group: WCS, BTS
 Target Device: CC2650, CC2640, CC1350

 ******************************************************************************
 
 Copyright (c) 2012-2016, Texas Instruments Incorporated
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

 ******************************************************************************
 Release Name: ble_sdk_2_02_01_18
 Release Date: 2016-10-26 15:20:04
 *****************************************************************************/

/*********************************************************************
 * INCLUDES
 */
#include <string.h>
#include "bcomdef.h"
#include "linkdb.h"
#include "att.h"
#include "gatt.h"
#include "gatt_uuid.h"
#include "gattservapp.h"
#include "hal_flash.h"
#if (defined HAL_LCD) && (HAL_LCD == TRUE)
#include "hal_lcd.h"
#endif
#include <driverlib/rom.h>
#include <driverlib/vims.h>

#include "oad_target.h"
#include "oad_constants.h"
#include "oad.h"

/*********************************************************************
 * CONSTANTS
 */
#define ERROR_BLOCK     0xFFFF

#define OAD_SUCCESS     0
#define OAD_CRC_ERR     1
#define OAD_FLASH_ERR   2
#define OAD_BUFFER_OFL  3

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */

// OAD Service UUID
static const uint8_t oadServUUID[ATT_UUID_SIZE] =
{
  TI_BASE_UUID_128(OAD_SERVICE_UUID)
};

static const uint8_t oadCharUUID[OAD_CHAR_CNT][ATT_UUID_SIZE] =
{
 // OAD Image Identify UUID
 TI_BASE_UUID_128(OAD_IMG_IDENTIFY_UUID),

 // OAD Image Block Request/Response UUID
 TI_BASE_UUID_128(OAD_IMG_BLOCK_UUID),

 // OAD Image Count UUID
 TI_BASE_UUID_128(OAD_IMG_COUNT_UUID),

 // OAD Status UUID
 TI_BASE_UUID_128(OAD_IMG_STATUS_UUID)
};

/*********************************************************************
 * Profile Attributes - variables
 */

// OAD Service attribute
static const gattAttrType_t oadService = { ATT_UUID_SIZE, oadServUUID };

// Place holders for the GATT Server App to be able to lookup handles.
static uint8_t oadCharVals[OAD_CHAR_CNT] = {0, 0 , 1, OAD_SUCCESS};

// OAD Characteristic Properties
static uint8_t oadCharProps = GATT_PROP_WRITE_NO_RSP | GATT_PROP_WRITE
                              | GATT_PROP_NOTIFY;

static uint8_t oadCharCountProps = GATT_PROP_WRITE_NO_RSP | GATT_PROP_WRITE;
static uint8_t oadCharStatusProps = GATT_PROP_READ | GATT_PROP_NOTIFY;

// OAD Client Characteristic Configs
static gattCharCfg_t *oadImgIdentifyConfig;
static gattCharCfg_t *oadImgBlockConfig;
static gattCharCfg_t *oadStatusConfig;

// OAD Characteristic user descriptions
static const uint8_t oadImgIdentifyDesc[] = "Img Identify";
static const uint8_t oadImgBlockDesc[] = "Img Block";
static const uint8_t oadImgCountDesc[] = "Img Count";
static const uint8_t oadStatusDesc[] = "Img Status";

/*********************************************************************
 * Profile Attributes - Table
 */

static gattAttribute_t oadAttrTbl[] =
{
  // OAD Service
  {
    { ATT_BT_UUID_SIZE, primaryServiceUUID },
    GATT_PERMIT_READ,
    0,
    (uint8_t *)&oadService
  },

    // OAD Image Identify Characteristic Declaration
    {
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ,
      0,
      &oadCharProps
    },

      // OAD Image Identify Characteristic Value
      {
        { ATT_UUID_SIZE, oadCharUUID[0] },
        GATT_PERMIT_WRITE,
        0,
        oadCharVals
      },

      // Characteristic configuration
      {
        { ATT_BT_UUID_SIZE, clientCharCfgUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE,
        0,
        (uint8_t *)&oadImgIdentifyConfig
      },

      // OAD Image Identify User Description
      {
        { ATT_BT_UUID_SIZE, charUserDescUUID },
        GATT_PERMIT_READ,
        0,
        (uint8_t *)oadImgIdentifyDesc
      },

    // OAD Image Block Request/Response Characteristic Declaration
    {
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ,
      0,
      &oadCharProps
    },

      // OAD Image Block Request/Response Characteristic Value
      {
        { ATT_UUID_SIZE, oadCharUUID[OAD_IDX_IMG_BLOCK] },
        GATT_PERMIT_WRITE,
        0,
        oadCharVals + OAD_IDX_IMG_BLOCK,
      },

       // Characteristic configuration
      {
        { ATT_BT_UUID_SIZE, clientCharCfgUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE,
        0,
        (uint8_t *)&oadImgBlockConfig
      },

      // OAD Image Block Request/Response User Description
      {
        { ATT_BT_UUID_SIZE, charUserDescUUID },
        GATT_PERMIT_READ,
        0,
        (uint8_t *)oadImgBlockDesc
      },

    // OAD Image Count Characteristic Declaration
    {
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ,
      0,
      &oadCharCountProps
    },

      // OAD Image Count Characteristic Value
      {
        { ATT_UUID_SIZE, oadCharUUID[OAD_IDX_IMG_COUNT] },
        GATT_PERMIT_WRITE,
        0,
        oadCharVals + OAD_IDX_IMG_COUNT
      },

      // OAD Image Count User Description
      {
        { ATT_BT_UUID_SIZE, charUserDescUUID },
        GATT_PERMIT_READ,
        0,
        (uint8_t *)oadImgCountDesc
      },

    // OAD Status Characteristic Declaration
    {
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ,
      0,
      &oadCharStatusProps
    },

      // OAD Status Characteristic Value
      {
        { ATT_UUID_SIZE, oadCharUUID[OAD_IDX_IMG_STATUS] },
        GATT_PERMIT_READ,
        0,
        oadCharVals + OAD_IDX_IMG_STATUS
      },

      // Characteristic configuration
      {
        { ATT_BT_UUID_SIZE, clientCharCfgUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE,
        0,
        (uint8_t *)&oadStatusConfig
      },

      // OAD Status User Description
      {
        { ATT_BT_UUID_SIZE, charUserDescUUID },
        GATT_PERMIT_READ,
        0,
        (uint8_t *)oadStatusDesc
      }

};

/*********************************************************************
 * LOCAL VARIABLES
 */

static oadWriteCB_t oadTargetWriteCB = NULL;

static uint16_t oadBlkNum = 0;
static uint16_t oadBlkTot = 0xFFFF;
static uint32_t imageAddress;
static uint16_t imagePage;

#ifndef FEATURE_OAD_ONCHIP
// Used to keep track of images written.
static uint8_t flagRecord = 0;
#endif //FEATURE_OAD_ONCHIP

static uint8_t oad_imageIdLen = 0;

/*********************************************************************
 * LOCAL FUNCTIONS
 */

static bStatus_t oadReadAttrCB(uint16_t connHandle, gattAttribute_t *pAttr,
                               uint8_t *pValue, uint16_t *pLen, uint16_t offset,
                               uint16_t maxLen, uint8_t method);

static bStatus_t oadWriteAttrCB(uint16_t connHandle, gattAttribute_t *pAttr,
                                uint8_t *pValue, uint16_t len, uint16_t offset,
                                uint8_t method);

static void OAD_getNextBlockReq(uint16_t connHandle, uint16_t blkNum);
static void OAD_rejectImage(uint16_t connHandle, img_hdr_t *pImgHdr);
static void OAD_sendStatus(uint16_t connHandle, uint8_t status);

#if !defined FEATURE_OAD_ONCHIP
static uint8_t CheckImageDownloadCount(void);
static uint8_t checkDL(void);
static uint16_t crcCalcDL(void);
static uint16_t crc16(uint16_t crc, uint8_t val);
#endif  // !FEATURE_OAD_ONCHIP

/*********************************************************************
 * PROFILE CALLBACKS
 */

// Note: When an operation on a characteristic requires authorization and
// pfnAuthorizeAttrCB is not defined for that characteristic's service, the
// Stack will report a status of ATT_ERR_UNLIKELY to the client.  When an
// operation on a characteristic requires authorization the Stack will call
// pfnAuthorizeAttrCB to check a client's authorization prior to calling
// pfnReadAttrCB or pfnWriteAttrCB, so no checks for authorization need to be
// made within these functions.
CONST gattServiceCBs_t oadCBs =
{
  oadReadAttrCB,  // Read callback function pointer.
  oadWriteAttrCB, // Write callback function pointer.
  NULL            // Authorization callback function pointer.
};

/*********************************************************************
 * @fn      OAD_addService
 *
 * @brief   Initializes the OAD Service by registering GATT attributes
 *          with the GATT server. Only call this function once.
 *
 * @param   None.
 *
 * @return  The return value of GATTServApp_RegisterForMsg().
 */
bStatus_t OAD_addService(void)
{
  // Allocate Client Characteristic Configuration table.
  oadImgIdentifyConfig = (gattCharCfg_t *)ICall_malloc(sizeof(gattCharCfg_t) *
                                                       linkDBNumConns);
  if (oadImgIdentifyConfig == NULL)
  {
    return bleMemAllocError;
  }

  // Allocate Client Characteristic Configuration table.
  oadImgBlockConfig = (gattCharCfg_t *)ICall_malloc(sizeof(gattCharCfg_t) *
                                                    linkDBNumConns);

  if (oadImgBlockConfig == NULL)
  {
    // Free already allocated data.
    ICall_free(oadImgIdentifyConfig);

    return (bleMemAllocError);
  }

  // Allocate Client Characteristic Configuration table.
  oadStatusConfig = (gattCharCfg_t *)ICall_malloc(sizeof(gattCharCfg_t) *
                                                  linkDBNumConns);

  if (oadStatusConfig == NULL)
  {
    // Free already allocated data.
    ICall_free(oadImgIdentifyConfig);
    ICall_free(oadImgBlockConfig);

    return (bleMemAllocError);
  }

  // Initialize Client Characteristic Configuration attributes.
  GATTServApp_InitCharCfg(INVALID_CONNHANDLE, oadImgIdentifyConfig);
  GATTServApp_InitCharCfg(INVALID_CONNHANDLE, oadImgBlockConfig);
  GATTServApp_InitCharCfg(INVALID_CONNHANDLE, oadStatusConfig);

  return GATTServApp_RegisterService(oadAttrTbl, GATT_NUM_ATTRS(oadAttrTbl),
                                     GATT_MAX_ENCRYPT_KEY_SIZE, &oadCBs);
}

/*********************************************************************
 * @fn      OAD_register
 *
 * @brief   Register a callback function with the OAD Target Profile.
 *
 * @param   *pfnOadCBs - write callback function container.
 *
 * @return  None.
 */
void OAD_register(oadTargetCBs_t *pfnOadCBs)
{
  // Register a write callback function.
  oadTargetWriteCB = pfnOadCBs->pfnOadWrite;
}

/*********************************************************************
 * @fn      oadReadAttrCB
 *
 * @brief   Read an attribute.
 *
 * @param   connHandle - connection message was received on
 * @param   pAttr      - pointer to attribute
 * @param   pValue     - pointer to data to be read
 * @param   pLen       - length of data to be read
 * @param   offset     - offset of the first octet to be read
 * @param   maxLen     - maximum length of data to be read
 * @param   method     - type of read message
 *
 * @return  Reads not supported, returns ATT_ERR_INVALID_HANDLE
 */
static bStatus_t oadReadAttrCB(uint16_t connHandle, gattAttribute_t *pAttr,
                               uint8_t *pValue, uint16_t *pLen, uint16_t offset,
                               uint16_t maxLen, uint8_t method)
{
    bStatus_t status;

    // Make sure it's not a blob operation (no attributes in the profile are long)
    if (offset > 0)
    {
        return ATT_ERR_ATTR_NOT_LONG;
    }

    // 128-bit UUID
    if (!memcmp(pAttr->type.uuid, oadCharUUID[OAD_IDX_IMG_STATUS],
                ATT_UUID_SIZE))
    {
        *pLen = 1;
        pValue[0] = *pAttr->pValue;
        status = SUCCESS;
    }
    else
    {
        *pLen = 0;
        status = ATT_ERR_INVALID_HANDLE;
    }

    return status;
}

/*********************************************************************
 * @fn      oadWriteAttrCB
 *
 * @brief   Validate and Write attribute data
 *
 * @param   connHandle - connection message was received on
 * @param   pAttr      - pointer to attribute
 * @param   pValue     - pointer to data to be written
 * @param   len        - length of data
 * @param   offset     - offset of the first octet to be written
 * @param   method     - type of write message
 *
 * @return  SUCCESS, blePending or Failure
 */
static bStatus_t oadWriteAttrCB(uint16_t connHandle, gattAttribute_t *pAttr,
                                uint8_t *pValue, uint16_t len, uint16_t offset,
                                uint8_t method)
{
  bStatus_t status = SUCCESS;

  if (pAttr->type.len == ATT_BT_UUID_SIZE)
  {
    // 16-bit UUID
    uint16_t uuid = BUILD_UINT16(pAttr->type.uuid[0], pAttr->type.uuid[1]);
    if (uuid == GATT_CLIENT_CHAR_CFG_UUID)
    {
      // Process a CCC write request.
      status = GATTServApp_ProcessCCCWriteReq(connHandle, pAttr, pValue, len,
                                               offset, GATT_CLIENT_CFG_NOTIFY);
    }
    else
    {
      status = ATT_ERR_ATTR_NOT_FOUND; // Should never get here!
    }
  }
  else
  {
    // 128-bit UUID
    if (!memcmp(pAttr->type.uuid, oadCharUUID[OAD_IDX_IMG_IDENTIFY],
                ATT_UUID_SIZE))
    {
      /* OAD manager has sent new image information.
       * Read and decide whether to accept or reject an OAD
       * of this new application image.
       */

      // Notify Application
      if (oadTargetWriteCB != NULL)
      {
        oad_imageIdLen = len;
        (*oadTargetWriteCB)(OAD_WRITE_IDENTIFY_REQ, connHandle, pValue);
      }
    }
    else if (!memcmp(pAttr->type.uuid, oadCharUUID[OAD_IDX_IMG_BLOCK],
                     ATT_UUID_SIZE))
    {
      /* OAD is ongoing.
       * the OAD manager has sent a block from the new image.
       */

      // Notify the application.
      if (oadTargetWriteCB != NULL)
      {
        (*oadTargetWriteCB)(OAD_WRITE_BLOCK_REQ, connHandle, pValue);
      }
    }
    else if (!memcmp(pAttr->type.uuid, oadCharUUID[OAD_IDX_IMG_COUNT],
                     ATT_UUID_SIZE))
    {
#ifndef FEATURE_OAD_ONCHIP
      /*
       * This is a manipulation of the number of OAD Images to download before
       * issuing a reset.
       */
      if (len == sizeof(uint8_t) && *pValue != 0x00)
      {
        oadCharVals[OAD_IDX_IMG_COUNT] = *pValue;
      }
      else
      {
        status = ATT_ERR_INVALID_VALUE;
      }
#endif // !FEATURE_OAD_ONCHIP
    }
    else
    {
      status = ATT_ERR_ATTR_NOT_FOUND; // Should never get here!
    }
  }

  return status;
}

/*********************************************************************
 * @fn      OAD_imgIdentifyWrite
 *
 * @brief   Process the Image Identify Write.  Determined if the image
 *          header identified here should or should not be downloaded by
 *          this application.
 *
 * @param   connHandle - connection message was received on
 * @param   pValue     - pointer to image header data
 *
 * @return  none
 */
void OAD_imgIdentifyWrite(uint16_t connHandle, uint8_t *pValue)
{
  img_hdr_t ImgHdr;
  uint8_t hdrOffset = oad_imageIdLen == 8 ? 0 : 4;

  // Store the new image's header
  OADTarget_storeImageHeader(pValue);

  // Read out running image's header.
  OADTarget_getCurrentImageHeader(&ImgHdr);

  // Calculate block total of the new image.
  oadBlkTot = BUILD_UINT16(pValue[hdrOffset + 2], pValue[hdrOffset + 3]) /
              (OAD_BLOCK_SIZE / HAL_FLASH_WORD_SIZE);
  oadBlkNum = 0;
#ifndef FEATURE_OAD_ONCHIP
    flagRecord = 0;
#endif

  /* Requirements to begin OAD:
   * 1) LSB of image version cannot be the same, this would imply a code overlap
   *    between currently running image and new image.
   * 2) Total blocks of new image must not exceed maximum blocks supported, else
   *    the new image cannot fit.
   * 3) Block total must be greater than 0.
   * 4) Optional: Add additional criteria for initiating OAD here.
   */
  if (OADTarget_validateNewImage(pValue + hdrOffset, &ImgHdr, oadBlkTot))
  {
    // Determine where image will be stored.
    imageAddress = OADTarget_imageAddress(pValue+hdrOffset);
    imagePage = imageAddress / HAL_FLASH_PAGE_SIZE;

    // Open the target interface
    if (OADTarget_open())
    {
        uint8_t page;
        uint8_t lastPage = oadBlkTot / OAD_BLOCKS_PER_PAGE;

        // Set last page to end of OAD image address range.
        lastPage += imagePage;

        // Erase required pages
        for (page = imagePage; page <= lastPage; page++)
        {
            OADTarget_eraseFlash(page);
        }

        // Image accepted, request block 0.
        OAD_getNextBlockReq(connHandle, 0);
    }
    else
    {
        // Opening the flash has failed; report error
        OAD_sendStatus(connHandle, OAD_FLASH_ERR);
    }
  }
  else
  {
    // Image rejected, send header information of currently running image
    // to OAD manager.
    OAD_rejectImage(connHandle, &ImgHdr);
  }
}

/*********************************************************************
 * @fn      OAD_imgBlockWrite
 *
 * @brief   Process the Image Block Write.
 *
 * @param   connHandle - connection message was received on
 * @param   pValue - pointer to data to be written
 *
 * @return  none
 */
void OAD_imgBlockWrite(uint16_t connHandle, uint8_t *pValue)
{
  // N.B. This must be left volatile.
  volatile uint16_t blkNum = BUILD_UINT16(pValue[0], pValue[1]);

  // Check that this is the expected block number.
  if (oadBlkNum == blkNum)
  {
    // Write a 16 byte block to Flash.
    OADTarget_writeFlash(imagePage, (blkNum * OAD_BLOCK_SIZE), pValue+2,
                         OAD_BLOCK_SIZE);

    // Increment received block count.
    oadBlkNum++;
  }
  else
  {
    // Overflow, abort OAD
    oadBlkNum = 0;
#ifndef FEATURE_OAD_ONCHIP
    flagRecord = 0;
#endif
    // Close the target device
    OADTarget_close();

    // Send status
    OAD_sendStatus(connHandle, OAD_BUFFER_OFL);

    return;
  }

  // Check if the OAD Image is complete.
  if (oadBlkNum == oadBlkTot)
  {
#if FEATURE_OAD_ONCHIP
    // Handle CRC verification in BIM.
    OADTarget_systemReset();
#else // !FEATURE_OAD_ONCHIP
    // Run CRC check on new image.
    if (checkDL())
    {
      // Indicate a successful download and CRC just before rebooting
      OAD_sendStatus(connHandle, OAD_SUCCESS);

      // Store the flag of the downloaded image.
      flagRecord |= getImageFlag();

      // Store the image information.
      saveImageInfo();

      // Check if all expected images have been downloaded.
      if (CheckImageDownloadCount())
      {
        // If one image is a network processor image, inform the application now
        // so that it can take action on that image.
        // Note: this callback is not being sent from the context of an
        // interrupt. It is ok to take any action here.
        if (flagRecord & OAD_IMG_NP_FLAG)
        {
          (*oadTargetWriteCB)(OAD_IMAGE_COMPLETE, connHandle, NULL);
        }

        // If one image is an application or stack image, perform the reset
        // here.
        if (flagRecord & (OAD_IMG_APP_FLAG|OAD_IMG_STACK_FLAG))
        {
          OADTarget_systemReset();
        }

        flagRecord = 0;
      }
    }
    else
    {
      // CRC error
      OAD_sendStatus(connHandle, OAD_CRC_ERR);
    }
    flagRecord = 0;
#endif //FEATURE_OAD_ONCHIP

    OADTarget_close();
    oadBlkNum = 0;
  }
  else
  {
    // Request the next OAD Image block.
    OAD_getNextBlockReq(connHandle, oadBlkNum);
  }
}

/*********************************************************************
 * @fn      OAD_getNextBlockReq
 *
 * @brief   Process the Request for next image block.
 *
 * @param   connHandle - connection message was received on
 * @param   blkNum - block number to request from OAD Manager.
 *
 * @return  None
 */
static void OAD_getNextBlockReq(uint16_t connHandle, uint16_t blkNum)
{
  uint16_t value = GATTServApp_ReadCharCfg(connHandle, oadImgBlockConfig);

  // If notifications enabled
  if (value & GATT_CLIENT_CFG_NOTIFY)
  {
    attHandleValueNoti_t noti;

    noti.pValue = GATT_bm_alloc(connHandle, ATT_HANDLE_VALUE_NOTI, 2, NULL);

    if (noti.pValue != NULL)
    {
      gattAttribute_t *pAttr;

      pAttr= GATTServApp_FindAttr(oadAttrTbl, GATT_NUM_ATTRS(oadAttrTbl),
                                  oadCharVals+OAD_IDX_IMG_BLOCK);

      noti.handle = pAttr->handle;
      noti.len = 2;

      noti.pValue[0] = LO_UINT16(blkNum);
      noti.pValue[1] = HI_UINT16(blkNum);

      if (GATT_Notification(connHandle, &noti, FALSE) != SUCCESS)
      {
        GATT_bm_free((gattMsg_t *)&noti, ATT_HANDLE_VALUE_NOTI);
      }
    }
  }
}

/*********************************************************************
 * @fn      OAD_sendStatus
 *
 * @brief   Send status to the OAD Manager if an error occurs
 *
 * @param   connHandle - connection message was received on
 * @param   status - status to OAD Manager.
 *
 * @return  None
 */
static void OAD_sendStatus(uint16_t connHandle, uint8_t status)
{
  uint16_t value = GATTServApp_ReadCharCfg(connHandle, oadStatusConfig);

  // Store the value
  oadCharVals[OAD_IDX_IMG_STATUS] = status;

  // If notifications enabled
  if (value & GATT_CLIENT_CFG_NOTIFY)
  {
    attHandleValueNoti_t noti;

    noti.pValue = GATT_bm_alloc(connHandle, ATT_HANDLE_VALUE_NOTI, 1, NULL);

    if (noti.pValue != NULL)
    {
      gattAttribute_t *pAttr;

      pAttr= GATTServApp_FindAttr(oadAttrTbl, GATT_NUM_ATTRS(oadAttrTbl),
                                  oadCharVals+OAD_IDX_IMG_STATUS);

      noti.handle = pAttr->handle;
      noti.len = 1;

      noti.pValue[0] = status;

      if (GATT_Notification(connHandle, &noti, FALSE) != SUCCESS)
      {
        GATT_bm_free((gattMsg_t *)&noti, ATT_HANDLE_VALUE_NOTI);
      }
    }
  }
}

/*********************************************************************
 * @fn      OAD_rejectImage
 *
 * @brief   Reject the Image identified by the OAD manager, send back
 *          active image version, length and UID to manager.
 *
 * @param   connHandle - connection message was received on.
 * @param   pImgHdr    - pointer to the img_hdr_t data to send.
 *
 * @return  None.
 */
static void OAD_rejectImage(uint16_t connHandle, img_hdr_t *pImgHdr)
{
  uint16_t value = GATTServApp_ReadCharCfg(connHandle, oadImgIdentifyConfig);

  // If notifications enabled
  if (value & GATT_CLIENT_CFG_NOTIFY)
  {
    attHandleValueNoti_t noti;

    noti.pValue = GATT_bm_alloc(connHandle, ATT_HANDLE_VALUE_NOTI, 8, NULL);

    if (noti.pValue != NULL)
    {
      gattAttribute_t *pAttr;

      pAttr= GATTServApp_FindAttr(oadAttrTbl, GATT_NUM_ATTRS(oadAttrTbl),
                                  oadCharVals+OAD_IDX_IMG_IDENTIFY);

      noti.handle = pAttr->handle;
      noti.len = OAD_IMG_HDR_SIZE;

      noti.pValue[0] = LO_UINT16(pImgHdr->ver);
      noti.pValue[1] = HI_UINT16(pImgHdr->ver);

      noti.pValue[2] = LO_UINT16(pImgHdr->len);
      noti.pValue[3] = HI_UINT16(pImgHdr->len);

      (void)memcpy(noti.pValue+4, pImgHdr->uid, sizeof(pImgHdr->uid));

      if (GATT_Notification(connHandle, &noti, FALSE) != SUCCESS)
      {
        GATT_bm_free((gattMsg_t *)&noti, ATT_HANDLE_VALUE_NOTI);
      }
    }
  }

  // Close the OAD target if it is open
  OADTarget_close();
}


#if !defined FEATURE_OAD_ONCHIP

/*********************************************************************
 * @fn      crcCalcDL
 *
 * @brief   Run the CRC16 Polynomial calculation over the DL image.
 *
 * @param   None
 *
 * @return  The CRC16 calculated.
 */
static uint16_t crcCalcDL(void)
{
  uint16_t imageCRC = 0;
  uint8_t page;
  uint8_t lastPage = oadBlkTot / OAD_BLOCKS_PER_PAGE;

  // Remainder of bytes not divisible by the size of a flash page in bytes.
  uint16_t numRemBytes = (oadBlkTot - (lastPage * OAD_BLOCKS_PER_PAGE))
                         * OAD_BLOCK_SIZE;

  // Set last page to end of OAD image address range.
  lastPage += imagePage;

  // Read over downloaded pages
  for (page = imagePage; page <= lastPage; page++)
  {
    uint16_t offset;

    // Read over all flash words in a page, excluding the CRC section of the
    // first page and all bytes after remainder bytes on the last page.
    for (offset = (page == imagePage) ? HAL_FLASH_WORD_SIZE : 0;
         offset < HAL_FLASH_PAGE_SIZE &&
         (page < lastPage || offset < numRemBytes);
         offset += HAL_FLASH_WORD_SIZE)
    {
      uint8_t buf[HAL_FLASH_WORD_SIZE];
      uint8_t idx;

      // Read a word from flash.
      OADTarget_readFlash(page, offset, buf, HAL_FLASH_WORD_SIZE);

      // Calculate CRC of word, byte by byte.
      for (idx = 0; idx < HAL_FLASH_WORD_SIZE; idx++)
      {
        imageCRC = crc16(imageCRC, buf[idx]);
      }
    }
  }

  // IAR note explains that poly must be run with value zero for each byte of
  // the crc.
  imageCRC = crc16(imageCRC, 0);
  imageCRC = crc16(imageCRC, 0);

  // Return the CRC calculated over the image.
  return imageCRC;
}

/*********************************************************************
 * @fn      checkDL
 *
 * @brief   Check validity of the downloaded image.
 *
 * @param   None.
 *
 * @return  TRUE or FALSE for image valid.
 */
static uint8_t checkDL(void)
{
  uint16_t crc[2];

  OADTarget_getCrc(crc);

  if ((crc[0] == 0xFFFF) || (crc[0] == 0x0000))
  {
    return FALSE;
  }

  // Calculate CRC of downloaded image.
  crc[1] = crcCalcDL();

  if (crc[1] == crc[0])
  {
    // Set the CRC shadow as equivalent to the CRC.
    OADTarget_setCrc(crc);
  }

  return (crc[0] == crc[1]);
}

/*********************************************************************
 * @fn          crc16
 *
 * @brief       Run the CRC16 Polynomial calculation over the byte parameter.
 *
 * @param       crc - Running CRC calculated so far.
 * @param       val - Value on which to run the CRC16.
 *
 * @return      crc - Updated for the run.
 */
static uint16_t crc16(uint16_t crc, uint8_t val)
{
  const uint16_t poly = 0x1021;
  uint8_t cnt;

  for (cnt = 0; cnt < 8; cnt++, val <<= 1)
  {
    uint8_t msb = (crc & 0x8000) ? 1 : 0;

    crc <<= 1;

    if (val & 0x80)
    {
      crc |= 0x0001;
    }

    if (msb)
    {
      crc ^= poly;
    }
  }

  return crc;
}


/*********************************************************************
 * @fn          CheckImageDownloadCount
 *
 * @brief       Decrement the image download count.
 *
 * @param       Image Type of completed image.
 *
 * @return      returns TRUE if no more images are expected, FALSE otherwise.
 */
static uint8_t CheckImageDownloadCount(void)
{
  return !--oadCharVals[OAD_IDX_IMG_COUNT];
}
#endif // !FEATURE_OAD_ONCHIP

/*********************************************************************
*********************************************************************/
