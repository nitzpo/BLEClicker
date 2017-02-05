/**********************************************************************************************
 * Filename:       sunlightService.h
 *
 * Description:    This file contains the sunlightService service definitions and
 *                 prototypes.
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


#ifndef _SUNLIGHTSERVICE_H_
#define _SUNLIGHTSERVICE_H_

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */

/*********************************************************************
 * CONSTANTS
 */

/*********************************************************************
* CONSTANTS
*/
// Service UUID
#define SUNLIGHTSERVICE_SERV_UUID 	0xBA55

//Characteristic defines
#define SUNLIGHTSERVICE_SUNLIGHTSERVICECHAR1      0
#define SUNLIGHTSERVICE_SUNLIGHTSERVICECHAR1_UUID 0x2BAD
#define SUNLIGHTSERVICE_SUNLIGHTSERVICECHAR1_LEN  4

//  Characteristic defines
#define SUNLIGHTSERVICE_UPDATEPERIOD      1
#define SUNLIGHTSERVICE_UPDATEPERIOD_UUID 0x3BAD
#define SUNLIGHTSERVICE_UPDATEPERIOD_LEN  2
/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * Profile Callbacks
 */

// Callback when a characteristic value has changed
typedef void (*sunlightServiceChange_t)( uint8 paramID );

typedef struct
{
  sunlightServiceChange_t        pfnChangeCb;  // Called when characteristic value changes
} sunlightServiceCBs_t;



/*********************************************************************
 * API FUNCTIONS
 */


/*
 * SunlightService_AddService- Initializes the SunlightService service by registering
 *          GATT attributes with the GATT server.
 *
 */
extern bStatus_t SunlightService_AddService( void );

/*
 * SunlightService_RegisterAppCBs - Registers the application callback function.
 *                    Only call this function once.
 *
 *    appCallbacks - pointer to application callbacks.
 */
extern bStatus_t SunlightService_RegisterAppCBs( sunlightServiceCBs_t *appCallbacks );

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
extern bStatus_t SunlightService_SetParameter( uint8 param, uint8 len, void *value );

/*
 * SunlightService_GetParameter - Get a SunlightService parameter.
 *
 *    param - Profile parameter ID
 *    value - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate
 *          data type (example: data type of uint16 will be cast to
 *          uint16 pointer).
 */
extern bStatus_t SunlightService_GetParameter( uint8 param, void *value );

/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* _SUNLIGHTSERVICE_H_ */
