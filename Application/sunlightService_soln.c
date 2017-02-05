/**********************************************************************************************
 * Filename:       sunlightService.c
 *
 * Description:    This file contains the implementation of the service.
 *
 * Copyright (c) 2015-2016, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *************************************************************************************************/


/*********************************************************************
 * INCLUDES
 */
#include <string.h>

#include "bcomdef.h"
#include "OSAL.h"
#include "linkdb.h"
#include "att.h"
#include "gatt.h"
#include "gatt_uuid.h"
#include "gattservapp.h"
#include "gapbondmgr.h"

#include "sunlightService.h"

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
* GLOBAL VARIABLES
*/

// sunlightService Service UUID
CONST uint8_t sunlightServiceUUID[ATT_BT_UUID_SIZE] =
{
  LO_UINT16(SUNLIGHTSERVICE_SERV_UUID), HI_UINT16(SUNLIGHTSERVICE_SERV_UUID)
};

// sunlightServiceChar1 UUID
CONST uint8_t sunlightService_SunlightServiceChar1UUID[ATT_UUID_SIZE] =
{
  TI_BASE_UUID_128(SUNLIGHTSERVICE_SUNLIGHTSERVICECHAR1_UUID)
};

// updatePeriod UUID
CONST uint8_t sunlightService_UpdatePeriodUUID[ATT_UUID_SIZE] =
{
  TI_BASE_UUID_128(SUNLIGHTSERVICE_UPDATEPERIOD_UUID)
};

/*********************************************************************
 * LOCAL VARIABLES
 */

static sunlightServiceCBs_t *pAppCBs = NULL;


/*********************************************************************
* Profile Attributes - variables
*/

// Service declaration
static CONST gattAttrType_t sunlightServiceDecl = { ATT_BT_UUID_SIZE, sunlightServiceUUID };

// Characteristic "SunlightServiceChar1" Properties (for declaration)
static uint8_t sunlightService_SunlightServiceChar1Props = GATT_PROP_READ | GATT_PROP_NOTIFY;

// Characteristic "SunlightServiceChar1" Value variable
static uint8_t sunlightService_SunlightServiceChar1Val[SUNLIGHTSERVICE_SUNLIGHTSERVICECHAR1_LEN] = {0};

// Characteristic "SunlightServiceChar1" CCCD
static gattCharCfg_t *sunlightService_SunlightServiceChar1Config;

// sunlightService Characteristic 1 User Description
static uint8 sunlightService_SunlightServiceChar1UserDesp[17] = "Characteristic 1";

// Characteristic "UpdatePeriod" Properties (for declaration)
static uint8_t sunlightService_UpdatePeriodProps = GATT_PROP_READ | GATT_PERMIT_WRITE;

// Characteristic "UpdatePeriod" Value variable
static uint8_t sunlightService_UpdatePeriodVal[SUNLIGHTSERVICE_UPDATEPERIOD_LEN] = {0};

/*********************************************************************
* Profile Attributes - Table
*/

static gattAttribute_t sunlightServiceAttrTbl[] =
{
  // sunlightService Service Declaration
  {
    { ATT_BT_UUID_SIZE, primaryServiceUUID },
    GATT_PERMIT_READ,
    0,
    (uint8_t *)&sunlightServiceDecl
  },

  // sunlightService Characteristic 1 Declaration
  {
    { ATT_BT_UUID_SIZE, characterUUID },
    GATT_PERMIT_READ,
    0,
    &sunlightService_SunlightServiceChar1Props
  },

    // sunlightService Characteristic Value 1
    {
      { ATT_UUID_SIZE, sunlightService_SunlightServiceChar1UUID },
	  GATT_PERMIT_READ | GATT_PERMIT_WRITE,
      0,
	  sunlightService_SunlightServiceChar1Val
    },

    // sunlightService Characteristic 1 configuration
    {
      { ATT_BT_UUID_SIZE, clientCharCfgUUID },
      GATT_PERMIT_READ | GATT_PERMIT_WRITE,
      0,
      (uint8 *)&sunlightService_SunlightServiceChar1Config
    },

    // Characteristic 4 User Description
    {
      { ATT_BT_UUID_SIZE, charUserDescUUID },
      GATT_PERMIT_READ,
      0,
	  sunlightService_SunlightServiceChar1UserDesp
    },

    // UpdatePeriod Characteristic Declaration
    {
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ,
      0,
      &sunlightService_UpdatePeriodProps
    },
      // UpdatePeriod Characteristic Value
      {
        { ATT_UUID_SIZE, sunlightService_UpdatePeriodUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE,
        0,
        sunlightService_UpdatePeriodVal
      },

};

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static bStatus_t sunlightService_ReadAttrCB( uint16 connHandle, gattAttribute_t *pAttr,
                                           uint8 *pValue, uint16 *pLen, uint16 offset,
                                           uint16 maxLen, uint8 method );
static bStatus_t sunlightService_WriteAttrCB( uint16 connHandle, gattAttribute_t *pAttr,
                                            uint8 *pValue, uint16 len, uint16 offset,
                                            uint8 method );

/*********************************************************************
 * PROFILE CALLBACKS
 */
// Simple Profile Service Callbacks
CONST gattServiceCBs_t sunlightServiceCBs =
{
  sunlightService_ReadAttrCB,  // Read callback function pointer
  sunlightService_WriteAttrCB, // Write callback function pointer
  NULL                       // Authorization callback function pointer
};

/*********************************************************************
* PUBLIC FUNCTIONS
*/

/*
 * SunlightService_AddService- Initializes the SunlightService service by registering
 *          GATT attributes with the GATT server.
 *
 */
bStatus_t SunlightService_AddService( void )
{
  uint8_t status;

  // Allocate Client Characteristic Configuration table
  sunlightService_SunlightServiceChar1Config = (gattCharCfg_t *)ICall_malloc( sizeof(gattCharCfg_t) *
                                                            linkDBNumConns );
  if ( sunlightService_SunlightServiceChar1Config  == NULL )
  {
    return ( bleMemAllocError );
  }

  // Initialize Client Characteristic Configuration attributes
  GATTServApp_InitCharCfg( INVALID_CONNHANDLE, sunlightService_SunlightServiceChar1Config  );

  // Register GATT attribute list and CBs with GATT Server App
  status = GATTServApp_RegisterService( sunlightServiceAttrTbl,
                                        GATT_NUM_ATTRS( sunlightServiceAttrTbl ),
                                        GATT_MAX_ENCRYPT_KEY_SIZE,
                                        &sunlightServiceCBs );

  return ( status );
}

/*
 * SunlightService_RegisterAppCBs - Registers the application callback function.
 *                    Only call this function once.
 *
 *    appCallbacks - pointer to application callbacks.
 */
bStatus_t SunlightService_RegisterAppCBs( sunlightServiceCBs_t *appCallbacks )
{
  if ( appCallbacks )
  {
    pAppCBs = appCallbacks;

    return ( SUCCESS );
  }
  else
  {
    return ( bleAlreadyInRequestedMode );
  }
}

/*
 * SunlightService_SetParameter - Set a SunlightService parameter.
 *
 *    param - Profile parameter ID
 *    len - length of data to right
 *    value - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate
 *          data type (example: data type of uint16 will be cast to
 *          uint16 pointer).
 */
bStatus_t SunlightService_SetParameter( uint8 param, uint8 len, void *value )
{
  bStatus_t ret = SUCCESS;
  switch ( param )
  {
	  case SUNLIGHTSERVICE_SUNLIGHTSERVICECHAR1:
		if ( len == SUNLIGHTSERVICE_SUNLIGHTSERVICECHAR1_LEN )
		{
		  memcpy(sunlightService_SunlightServiceChar1Val, value, len);

		  // Try to send notification.
		  GATTServApp_ProcessCharCfg( sunlightService_SunlightServiceChar1Config, (uint8_t *)&sunlightService_SunlightServiceChar1Val, FALSE,
									  sunlightServiceAttrTbl, GATT_NUM_ATTRS( sunlightServiceAttrTbl ),
									  INVALID_TASK_ID,  sunlightService_ReadAttrCB);
		}
		else
		{
		  ret = bleInvalidRange;
		}
		break;

	  case SUNLIGHTSERVICE_UPDATEPERIOD:
	    if ( len == SUNLIGHTSERVICE_UPDATEPERIOD_LEN )
	    {
	      memcpy(sunlightService_UpdatePeriodVal, value, len);
	    }
	    else
	    {
	      ret = bleInvalidRange;
	    }
	    break;

    default:
      ret = INVALIDPARAMETER;
      break;
  }
  return ret;
}


/*
 * SunlightService_GetParameter - Get a SunlightService parameter.
 *
 *    param - Profile parameter ID
 *    value - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate
 *          data type (example: data type of uint16 will be cast to
 *          uint16 pointer).
 */
bStatus_t SunlightService_GetParameter( uint8 param, void *value )
{
  bStatus_t ret = SUCCESS;
  switch ( param )
  {
  	  case SUNLIGHTSERVICE_SUNLIGHTSERVICECHAR1:
		memcpy(value, sunlightService_SunlightServiceChar1Val, SUNLIGHTSERVICE_SUNLIGHTSERVICECHAR1_LEN);
		break;

  	  case SUNLIGHTSERVICE_UPDATEPERIOD:
		memcpy(value, sunlightService_UpdatePeriodVal, SUNLIGHTSERVICE_UPDATEPERIOD_LEN);
		break;

    default:
      ret = INVALIDPARAMETER;
      break;
  }
  return ret;
}


/*********************************************************************
 * @fn          sunlightService_ReadAttrCB
 *
 * @brief       Read an attribute.
 *
 * @param       connHandle - connection message was received on
 * @param       pAttr - pointer to attribute
 * @param       pValue - pointer to data to be read
 * @param       pLen - length of data to be read
 * @param       offset - offset of the first octet to be read
 * @param       maxLen - maximum length of data to be read
 * @param       method - type of read message
 *
 * @return      SUCCESS, blePending or Failure
 */
static bStatus_t sunlightService_ReadAttrCB( uint16 connHandle, gattAttribute_t *pAttr,
                                       uint8 *pValue, uint16 *pLen, uint16 offset,
                                       uint16 maxLen, uint8 method )
{
  bStatus_t status = SUCCESS;

  // See if request is regarding the SunlightServiceChar1 Characteristic Value
if ( ! memcmp(pAttr->type.uuid, sunlightService_SunlightServiceChar1UUID, pAttr->type.len) )
  {
    if ( offset > SUNLIGHTSERVICE_SUNLIGHTSERVICECHAR1_LEN )  // Prevent malicious ATT ReadBlob offsets.
    {
      status = ATT_ERR_INVALID_OFFSET;
    }
    else
    {
      *pLen = MIN(maxLen, SUNLIGHTSERVICE_SUNLIGHTSERVICECHAR1_LEN - offset);  // Transmit as much as possible
      memcpy(pValue, pAttr->pValue + offset, *pLen);
    }
  }
// See if request is regarding the UpdatePeriod Characteristic Value
else if ( ! memcmp(pAttr->type.uuid, sunlightService_UpdatePeriodUUID, pAttr->type.len) )
{
  if ( offset > SUNLIGHTSERVICE_UPDATEPERIOD_LEN )  // Prevent malicious ATT ReadBlob offsets.
  {
    status = ATT_ERR_INVALID_OFFSET;
  }
  else
  {
    *pLen = MIN(maxLen, SUNLIGHTSERVICE_UPDATEPERIOD_LEN - offset);  // Transmit as much as possible
    memcpy(pValue, pAttr->pValue + offset, *pLen);
  }
}
  else
  {
    // If we get here, that means you've forgotten to add an if clause for a
    // characteristic value attribute in the attribute table that has READ permissions.
    *pLen = 0;
    status = ATT_ERR_ATTR_NOT_FOUND;
  }

  return status;
}


/*********************************************************************
 * @fn      sunlightService_WriteAttrCB
 *
 * @brief   Validate attribute data prior to a write operation
 *
 * @param   connHandle - connection message was received on
 * @param   pAttr - pointer to attribute
 * @param   pValue - pointer to data to be written
 * @param   len - length of data
 * @param   offset - offset of the first octet to be written
 * @param   method - type of write message
 *
 * @return  SUCCESS, blePending or Failure
 */
static bStatus_t sunlightService_WriteAttrCB( uint16 connHandle, gattAttribute_t *pAttr,
                                        uint8 *pValue, uint16 len, uint16 offset,
                                        uint8 method )
{
  bStatus_t status  = SUCCESS;
  uint8_t   paramID = 0xFF;

  // See if request is regarding a Client Characterisic Configuration
  if ( ! memcmp(pAttr->type.uuid, clientCharCfgUUID, pAttr->type.len) )
  {
    // Allow only notifications.
    status = GATTServApp_ProcessCCCWriteReq( connHandle, pAttr, pValue, len,
                                             offset, GATT_CLIENT_CFG_NOTIFY);
  }
  // See if request is regarding the SunlightServiceChar1 Characteristic Value
  else if ( ! memcmp(pAttr->type.uuid, sunlightService_SunlightServiceChar1UUID, pAttr->type.len) )
  {
    if ( offset + len > SUNLIGHTSERVICE_SUNLIGHTSERVICECHAR1_LEN )
    {
      status = ATT_ERR_INVALID_OFFSET;
    }
    else
    {
      // Copy pValue into the variable we point to from the attribute table.
      memcpy(pAttr->pValue + offset, pValue, len);

      // Only notify application if entire expected value is written
      if ( offset + len == SUNLIGHTSERVICE_SUNLIGHTSERVICECHAR1_LEN)
        paramID = SUNLIGHTSERVICE_SUNLIGHTSERVICECHAR1;
    }
  }
  // See if request is regarding the UpdatePeriod Characteristic Value
  else if ( ! memcmp(pAttr->type.uuid, sunlightService_UpdatePeriodUUID, pAttr->type.len) )
  {
    if ( offset + len > SUNLIGHTSERVICE_UPDATEPERIOD_LEN )
    {
      status = ATT_ERR_INVALID_OFFSET;
    }
    else
    {
      // Copy pValue into the variable we point to from the attribute table.
      memcpy(pAttr->pValue + offset, pValue, len);

      // Only notify application if entire expected value is written
      if ( offset + len == SUNLIGHTSERVICE_UPDATEPERIOD_LEN)
        paramID = SUNLIGHTSERVICE_UPDATEPERIOD;
    }
  }
  else
  {
    // If we get here, that means you've forgotten to add an if clause for a
    // characteristic value attribute in the attribute table that has WRITE permissions.
    status = ATT_ERR_ATTR_NOT_FOUND;
  }

  // Let the application know something changed (if it did) by using the
  // callback it registered earlier (if it did).
  if (paramID != 0xFF)
    if ( pAppCBs && pAppCBs->pfnChangeCb )
      pAppCBs->pfnChangeCb( paramID ); // Call app function from stack task context.


  return status;
}
