/******************************************************************************

 @file  hiddev.c

 @brief This file contains the common HID Device profile for use with the
        CC6250 Bluetooth Low Energy Protocol Stack.

 Group: WCS, BTS
 Target Device: CC2650, CC2640, CC1350

 ******************************************************************************
 
 Copyright (c) 2011-2016, Texas Instruments Incorporated
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
#include <xdc/std.h>

#include <xdc/runtime/Error.h>
#include <xdc/runtime/System.h>

#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Queue.h>

#include <icall.h>

#include "gatt.h"
#include "hci.h"

#include "gapgattserver.h"
#include "gattservapp.h"
#include "gatt_uuid.h"
#include "gatt_profile_uuid.h"
#include "devinfoservice.h"
#include "battservice.h"
#include "scanparamservice.h"

#include "peripheral.h"
#include "gapbondmgr.h"

#include "osal_snv.h"
#include "icall_apimsg.h"

#include "util.h"

#include "hiddev.h"

/*********************************************************************
 * MACROS
 */

// Battery measurement period in ms.
#define DEFAULT_BATT_PERIOD                   15000

// TRUE to run scan parameters refresh notify test.
#define DEFAULT_SCAN_PARAM_NOTIFY_TEST        TRUE

// Advertising intervals (units of 625us, 160=100ms).
#define HID_INITIAL_ADV_INT_MIN               48
#define HID_INITIAL_ADV_INT_MAX               80
#define HID_HIGH_ADV_INT_MIN                  32
#define HID_HIGH_ADV_INT_MAX                  48
#define HID_LOW_ADV_INT_MIN                   1600
#define HID_LOW_ADV_INT_MAX                   1600

// Advertising timeouts in sec.
#define HID_INITIAL_ADV_TIMEOUT               60
#define HID_HIGH_ADV_TIMEOUT                  5
#define HID_LOW_ADV_TIMEOUT                   0

/*
 * Time in ms to delay after reconnection. This is in place so that various
 * OS's have a chance to receive and process HID reports after reconnection.
 */
#define HID_REPORT_READY_TIME                 1000

// HID Service Task Events.
#define HID_STATE_CHANGE_EVT                  0x0001
#define HID_BATT_PERIODIC_EVT                 0x0002
#define HID_IDLE_EVT                          0x0004
#define HID_SEND_REPORT_EVT                   0x0008
#define HID_BATT_SERVICE_EVT                  0x0010
#define HID_PASSCODE_EVT                      0x0020
#define HID_PAIR_STATE_EVT                    0x0040

#define reportQEmpty()                        (firstQIdx == lastQIdx)

#define HIDDEVICE_TASK_PRIORITY               2

#ifndef HIDDEVICE_TASK_STACK_SIZE
#define HIDDEVICE_TASK_STACK_SIZE             400
#endif

/*********************************************************************
 * CONSTANTS
 */

#define HID_DEV_DATA_LEN                      9

#ifdef HID_DEV_RPT_QUEUE_LEN
  #define HID_DEV_REPORT_Q_SIZE               (HID_DEV_RPT_QUEUE_LEN+1)
#else
  #define HID_DEV_REPORT_Q_SIZE               (10+1)
#endif

// HID Auto Sync White List configuration parameter. This parameter should be
// set to FALSE if the HID Host (i.e., the Master device) uses a Resolvable
// Private Address (RPA). It should be set to TRUE, otherwise.
#ifndef HID_AUTO_SYNC_WL
  #define HID_AUTO_SYNC_WL                    FALSE
#endif

/*********************************************************************
 * TYPEDEFS
 */

// Event passed from other profiles.
typedef struct
{
  appEvtHdr_t hdr; // Event header
  uint8_t *pData;  // Event data
} hidDevEvt_t;

typedef struct
{
  uint8_t  deviceAddr[B_ADDR_LEN];
  uint16_t connHandle;
  uint8_t  uiInputs;
  uint8_t  uiOutputs;
} hidDevPasscodeEvt_t;

typedef struct
{
 uint8_t id;
 uint8_t type;
 uint8_t len;
 uint8_t data[HID_DEV_DATA_LEN];
} hidDevReport_t;

/*********************************************************************
 * GLOBAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */

// Entity ID globally used to check for source and/or destination of messages.
ICall_EntityID selfEntity;

// Semaphore globally used to post events to the application thread.
static ICall_Semaphore sem;

// Clock instances for internal periodic events.
static Clock_Struct battPerClock;
static Clock_Struct idleTimeoutClock;

// Queue object used for app messages.
static Queue_Struct appMsg;
static Queue_Handle appMsgQueue;

// events flag for internal application events.
static uint16_t events;

// Task configuration.
Task_Struct hidDeviceTask;
Char hidDeviceTaskStack[HIDDEVICE_TASK_STACK_SIZE];

// GAP State
static gaprole_States_t hidDevGapState = GAPROLE_INIT;

// TRUE if connection is secure
static uint8_t hidDevConnSecure = FALSE;

// GAP connection handle
static uint16_t gapConnHandle;

// TRUE if pairing in progress
static uint8_t hidDevPairingStarted = FALSE;

// Status of last pairing
static uint8_t pairingStatus = SUCCESS;

// Pairing state
static uint8_t hidDevGapBondPairingState = HID_GAPBOND_PAIRING_STATE_NONE;

static hidRptMap_t *pHidDevRptTbl;

static uint8_t hidDevRptTblLen;

static hidDevCB_t *pHidDevCB;

static hidDevCfg_t *pHidDevCfg;

// Whether to change to the preferred connection parameters
static uint8_t updateConnParams = TRUE;

// Pending reports
static uint8_t firstQIdx = 0;
static uint8_t lastQIdx = 0;
static hidDevReport_t hidDevReportQ[HID_DEV_REPORT_Q_SIZE];

// Last report sent out
static hidDevReport_t lastReport = { 0 };

// State when HID reports are ready to be sent out
static volatile uint8_t hidDevReportReadyState = TRUE;

// Report ready delay clock
static Clock_Struct reportReadyClock;

/*********************************************************************
 * LOCAL FUNCTIONS
 */

// Task events and processing functions.
static void HidDev_init(void);
static void HidDev_taskFxn(UArg a0, UArg a1);
static void HidDev_processStackMsg(ICall_Hdr *pMsg);
static void HidDev_processAppMsg(hidDevEvt_t *pMsg);
static void HidDev_processGattMsg(gattMsgEvent_t *pMsg);
static void HidDev_disconnected(void);
static void HidDev_highAdvertising(void);
static void HidDev_lowAdvertising(void);
static void HidDev_initialAdvertising(void);
static uint8_t HidDev_bondCount(void);
static void HidDev_clockHandler(UArg arg);
static uint8_t HidDev_enqueueMsg(uint16_t event, uint8_t state,
                                 uint8_t *pData);

// HID reports.
static hidRptMap_t *HidDev_reportByHandle(uint16_t handle);
static hidRptMap_t *HidDev_reportById(uint8_t id, uint8_t type);
static hidRptMap_t *HidDev_reportByCccdHandle(uint16_t handle);
static void HidDev_enqueueReport(uint8_t id, uint8_t type, uint8_t len,
                                 uint8_t *pData);
static hidDevReport_t *HidDev_dequeueReport(void);
static void HidDev_sendReport(uint8_t id, uint8_t type, uint8_t len,
                              uint8_t *pData);
static uint8_t HidDev_sendNoti(uint16_t handle, uint8_t len, uint8_t *pData);
static uint8_t HidDev_isbufset(uint8_t *buf, uint8_t val, uint8_t len);

// Peripheral GAP role.
static void HidDev_stateChangeCB(gaprole_States_t newState);
static void HidDev_processStateChangeEvt(gaprole_States_t newState);

// Pair state.
static void HidDev_pairStateCB(uint16_t connHandle, uint8_t state,
                               uint8_t status);
static void HidDev_processPairStateEvt(uint8_t state, uint8_t status);

// Passcode.
static void HidDev_passcodeCB(uint8_t *deviceAddr, uint16_t connectionHandle,
                              uint8_t uiInputs, uint8_t uiOutputs);
static void HidDev_processPasscodeEvt(uint8_t *deviceAddr,
                                      uint16_t connectionHandle,
                                      uint8_t uiInputs, uint8_t uiOutputs);

// Battery events.
static void HidDev_batteryCB(uint8_t event);
static void HidDev_processBatteryEvt(uint8_t event);
static void HidDev_battPeriodicTask(void);

// Scan parameter events.
static void HidDev_scanParamCB(uint8_t event);

// Process reconnection delay
static void HidDev_reportReadyClockCB(UArg a0);

/*********************************************************************
 * PROFILE CALLBACKS
 */

// GAP Role Callbacks
static gapRolesCBs_t hidDev_PeripheralCBs =
{
  HidDev_stateChangeCB   // Profile State Change Callbacks
};

// Bond Manager Callbacks
static const gapBondCBs_t hidDevBondCB =
{
  (pfnPasscodeCB_t)HidDev_passcodeCB,
  HidDev_pairStateCB
};

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      HidDev_createTask
 *
 * @brief   Task creation function for the HID service.
 *
 * @param   none
 *
 * @return  none
 */
void HidDev_createTask(void)
{
  Task_Params taskParams;

  // Configure task.
  Task_Params_init(&taskParams);
  taskParams.stack = hidDeviceTaskStack;
  taskParams.stackSize = HIDDEVICE_TASK_STACK_SIZE;
  taskParams.priority = HIDDEVICE_TASK_PRIORITY;

  Task_construct(&hidDeviceTask, HidDev_taskFxn, &taskParams, NULL);
}

/*********************************************************************
 * @fn      HidDev_init
 *
 * @brief   Initialization function for the Hid Dev Task.
 *          This is called during initialization and should contain
 *          any application specific initialization (ie. hardware
 *          initialization/setup, table initialization, power up
 *          notification ...).
 *
 * @param   none
 *
 * @return  none
 */
static void HidDev_init(void)
{
  // Register the current thread as an ICall dispatcher application
  // so that the application can send and receive messages.
  ICall_registerApp(&selfEntity, &sem);

  // Create an RTOS queue for message from profile to be sent to app.
  appMsgQueue = Util_constructQueue(&appMsg);

  // Create one-shot clocks for internal periodic events.
  Util_constructClock(&battPerClock, HidDev_clockHandler,
                      DEFAULT_BATT_PERIOD, 0, false, HID_BATT_PERIODIC_EVT);

  // Setup the GAP Bond Manager.
  {
    uint8_t syncWL = HID_AUTO_SYNC_WL;

    // If a bond is created, the HID Device should write the address of the
    // HID Host in the HID Device controller's white list and set the HID
    // Device controller's advertising filter policy to 'process scan and
    // connection requests only from devices in the White List'.
    VOID GAPBondMgr_SetParameter(GAPBOND_AUTO_SYNC_WL, sizeof(uint8_t),
                                 &syncWL);
  }

  // Set up services.
  GGS_AddService(GATT_ALL_SERVICES);         // GAP
  GATTServApp_AddService(GATT_ALL_SERVICES); // GATT attributes

  DevInfo_AddService();
  Batt_AddService();
  ScanParam_AddService();

  // Register for Battery service callback.
  Batt_Register(HidDev_batteryCB);

  // Register for Scan Parameters service callback.
  ScanParam_Register(HidDev_scanParamCB);

  // Initialize report ready clock timer
  Util_constructClock(&reportReadyClock, HidDev_reportReadyClockCB,
                      HID_REPORT_READY_TIME, 0, false, NULL);
}

/*********************************************************************
 * @fn      HidDev_taskFxn
 *
 * @brief   Hid Dev Task event processor.  This function
 *          is called to process all events for the task.  Events
 *          include timers, messages and any other user defined events.
 *
 * @param   a0, a1 - not used
 *
 * @return  not used
 */
static void HidDev_taskFxn(UArg a0, UArg a1)
{
  // Initialize the application.
  HidDev_init();

  // Application main loop.
  for (;;)
  {
    // Waits for a signal to the semaphore associated with the calling thread.
    // Note that the semaphore associated with a thread is signaled when a
    // message is queued to the message receive queue of the thread or when
    // ICall_signal() function is called onto the semaphore.
    ICall_Errno errno = ICall_wait(ICALL_TIMEOUT_FOREVER);

    if (errno == ICALL_ERRNO_SUCCESS)
    {
      ICall_EntityID dest;
      ICall_ServiceEnum src;
      ICall_HciExtEvt *pMsg = NULL;

      if (ICall_fetchServiceMsg(&src, &dest,
                                (void **)&pMsg) == ICALL_ERRNO_SUCCESS)
      {
        if ((src == ICALL_SERVICE_CLASS_BLE) && (dest == selfEntity))
        {
          // Process inter-task message.
          HidDev_processStackMsg((ICall_Hdr *)pMsg);
        }

        if (pMsg)
        {
          ICall_freeMsg(pMsg);
        }
      }

      // If RTOS queue is not empty, process app message.
      while (!Queue_empty(appMsgQueue))
      {
        hidDevEvt_t *pMsg = (hidDevEvt_t *)Util_dequeueMsg(appMsgQueue);
        if (pMsg)
        {
          // Process message.
          HidDev_processAppMsg(pMsg);

          // Free the space from the message.
          ICall_free(pMsg);
        }
      }
    }

    // Idle timeout.
    if (events & HID_IDLE_EVT)
    {
      events &= ~HID_IDLE_EVT;

      if (hidDevGapState == GAPROLE_CONNECTED)
      {
        // If pairing in progress then restart timer.
        if (hidDevPairingStarted)
        {
          HidDev_StartIdleTimer();
        }
        // Else disconnect and don't allow reports to be sent
        else
        {
          hidDevReportReadyState = FALSE;
          GAPRole_TerminateConnection();
        }
      }
    }

    // Battery periodic event.
    if (events & HID_BATT_PERIODIC_EVT)
    {
      events &= ~HID_BATT_PERIODIC_EVT;

      HidDev_battPeriodicTask();
    }

    // Send HID report event.
    if (events & HID_SEND_REPORT_EVT)
    {
      events &= ~HID_SEND_REPORT_EVT;

      // If connection is secure
      if (hidDevConnSecure && hidDevReportReadyState)
      {
        hidDevReport_t *pReport = HidDev_dequeueReport();

        if (pReport != NULL)
        {
          // Send report.
          HidDev_sendReport(pReport->id, pReport->type, pReport->len,
                            pReport->data);
        }

        // If there is another report in the queue
        if (!reportQEmpty())
        {
          // Set another event.
          events |= HID_SEND_REPORT_EVT;

          // Post the profile's semaphore.
          Semaphore_post(sem);
        }
      }
    }
  }
}

/*********************************************************************
 * @fn      HidDev_StartDevice
 *
 * @brief   Start the GAP Role and Register the Bond Manager.
 *          This function is intended to be called from the application
 *          task after setting up the GAP Role and Bond Manager.
 *
 * @param   None.
 *
 * @return  None.
 */
void HidDev_StartDevice(void)
{
  // Start the Device.
  VOID GAPRole_StartDevice(&hidDev_PeripheralCBs);

  // Register with bond manager after starting device.
  GAPBondMgr_Register((gapBondCBs_t *)&hidDevBondCB);
}

/*********************************************************************
 * @fn      HidDev_Register
 *
 * @brief   Register a callback function with HID Dev.
 *
 * @param   pCfg         - Parameter configuration.
 * @param   pfnServiceCB - Callback function.
 *
 * @return  None.
 */
void HidDev_Register(hidDevCfg_t *pCfg, hidDevCB_t *pCBs)
{
  pHidDevCB = pCBs;
  pHidDevCfg = pCfg;

  // If configured and not zero, create the idle timeout clock.
  if ((pHidDevCfg != NULL) && (pHidDevCfg->idleTimeout != 0))
  {
    Util_constructClock(&idleTimeoutClock, HidDev_clockHandler,
                        pHidDevCfg->idleTimeout, 0, false, HID_IDLE_EVT);
  }
}

/*********************************************************************
 * @fn      HidDev_RegisterReports
 *
 * @brief   Register the report table with HID Dev.
 *
 * @param   numReports - Length of report table.
 * @param   pRpt       - Report table.
 *
 * @return  None.
 */
void HidDev_RegisterReports(uint8_t numReports, hidRptMap_t *pRpt)
{
  pHidDevRptTbl = pRpt;
  hidDevRptTblLen = numReports;
}

/*********************************************************************
 * @fn      HidDev_Report
 *
 * @brief   Send a HID report.
 *
 * @param   id    - HID report ID.
 * @param   type  - HID report type.
 * @param   len   - Length of report.
 * @param   pData - Report data.
 *
 * @return  None.
 */
void HidDev_Report(uint8_t id, uint8_t type, uint8_t len, uint8_t *pData)
{
  // Validate length of report
  if ( len > HID_DEV_DATA_LEN )
  {
    return;
  }

  // If connected
  if (hidDevGapState == GAPROLE_CONNECTED)
  {
    // If connection is secure
    if (hidDevConnSecure)
    {
      // Make sure there're no pending reports.
      if (reportQEmpty())
      {
        // Send report.
        HidDev_sendReport(id, type, len, pData);

        return;
      }
    }
  }
  // Else if not already advertising
  else if (hidDevGapState != GAPROLE_ADVERTISING)
  {
    HidDev_StartAdvertising();
  }

  // HidDev task will send report when secure connection is established.
  HidDev_enqueueReport(id, type, len, pData);
}

/*********************************************************************
 * @fn      HidDev_Close
 *
 * @brief   Close the connection or stop advertising.
 *
 * @param   None.
 *
 * @return  None.
 */
void HidDev_Close(void)
{
  uint8_t param;

  // If connected then disconnect.
  if (hidDevGapState == GAPROLE_CONNECTED)
  {
    GAPRole_TerminateConnection();
  }
  // Else stop advertising.
  else
  {
    param = FALSE;
    GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8_t), &param);
  }
}

/*********************************************************************
 * @fn      HidDev_SetParameter
 *
 * @brief   Set a HID Dev parameter.
 *
 * @param   param  - profile parameter ID
 * @param   len    - length of data to right
 * @param   pValue - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate
 *          data type (example: data type of uint16_t will be cast to
 *          uint16_t pointer).
 *
 * @return  bStatus_t
 */
bStatus_t HidDev_SetParameter(uint8_t param, uint8_t len, void *pValue)
{
  bStatus_t ret = SUCCESS;

  switch (param)
  {
    case HIDDEV_ERASE_ALLBONDS:
      if (len == 0)
      {
        hidRptMap_t *pRpt;

        // Get ATT handle for last report
        if ((pRpt = HidDev_reportById(lastReport.id, lastReport.type)) != NULL)
        {
          // See if the last report sent out wasn't a release key
          if (HidDev_isbufset(lastReport.data, 0x00, lastReport.len) == FALSE)
          {
            // Send a release report before disconnecting, otherwise
            // the last pressed key would get 'stuck' on the HID Host.
            memset(lastReport.data, 0x00, lastReport.len);

            // Send report notification
            VOID HidDev_sendNoti(pRpt->handle, lastReport.len, lastReport.data);
          }

          // Clear out last report
          memset(&lastReport, 0, sizeof(hidDevReport_t));
        }

        // Drop connection.
        if (hidDevGapState == GAPROLE_CONNECTED)
        {
          GAPRole_TerminateConnection();
        }

        // Flush report queue.
        firstQIdx = lastQIdx = 0;

        // Erase bonding info.
        GAPBondMgr_SetParameter(GAPBOND_ERASE_ALLBONDS, 0, NULL);
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

  return (ret);
}

/*********************************************************************
 * @fn      HidDev_GetParameter
 *
 * @brief   Get a HID Dev parameter.
 *
 * @param   param  - profile parameter ID
 * @param   pValue - pointer to data to get.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate
 *          data type (example: data type of uint16_t will be cast to
 *          uint16_t pointer).
 *
 * @return  bStatus_t
 */
bStatus_t HidDev_GetParameter(uint8_t param, void *pValue)
{
  bStatus_t ret = SUCCESS;

  switch (param)
  {
    case HIDDEV_GAPROLE_STATE:
      *((uint8_t*)pValue) = hidDevGapState;
      break;

    case HIDDEV_GAPBOND_STATE:
      *((uint8_t*)pValue) = hidDevGapBondPairingState;
      break;

    default:
      ret = INVALIDPARAMETER;
      break;
  }

  return (ret);
}

/*********************************************************************
 * @fn      HidDev_PasscodeRsp
 *
 * @brief   Respond to a passcode request.
 *
 * @param   status - SUCCESS if passcode is available, otherwise
 *                   see @ref SMP_PAIRING_FAILED_DEFINES.
 * @param   passcode - integer value containing the passcode.
 *
 * @return  none
 */
void HidDev_PasscodeRsp(uint8_t status, uint32_t passcode)
{
  // Send passcode response.
  GAPBondMgr_PasscodeRsp(gapConnHandle, status, passcode);
}

/*********************************************************************
 * @fn          HidDev_ReadAttrCB
 *
 * @brief       HID Dev attribute read callback.
 *
 * @param       connHandle - connection message was received on
 * @param       pAttr      - pointer to attribute
 * @param       pValue     - pointer to data to be read
 * @param       pLen       - length of data to be read
 * @param       offset     - offset of the first octet to be read
 * @param       maxLen     - maximum length of data to be read
 * @param       method     - type of read message
 *
 * @return      SUCCESS, blePending or Failure
 */
bStatus_t HidDev_ReadAttrCB(uint16_t connHandle, gattAttribute_t *pAttr,
                            uint8_t *pValue, uint16_t *pLen, uint16_t offset,
                            uint16_t maxLen, uint8_t method)
{
  bStatus_t   status = SUCCESS;
  hidRptMap_t *pRpt;

  uint16_t uuid = BUILD_UINT16(pAttr->type.uuid[0], pAttr->type.uuid[1]);

  // Only report map is long.
  if (offset > 0 && uuid != REPORT_MAP_UUID)
  {
    return (ATT_ERR_ATTR_NOT_LONG);
  }

  if (uuid == REPORT_UUID ||
      uuid == BOOT_KEY_INPUT_UUID ||
      uuid == BOOT_KEY_OUTPUT_UUID ||
      uuid == BOOT_MOUSE_INPUT_UUID)
  {
    // Find report ID in table.
    if ((pRpt = HidDev_reportByHandle(pAttr->handle)) != NULL)
    {
      // Execute report callback.
      status  = (*pHidDevCB->reportCB)(pRpt->id, pRpt->type, uuid,
                                        HID_DEV_OPER_READ, pLen, pValue);
    }
    else
    {
      *pLen = 0;
    }
  }
  else if (uuid == REPORT_MAP_UUID)
  {
    // If the value offset of the Read Blob Request is greater than the
    // length of the attribute value, an Error Response shall be sent with
    // the error code Invalid Offset.
    if (offset > hidReportMapLen)
    {
      status = ATT_ERR_INVALID_OFFSET;
    }
    else
    {
      // Determine read length.
      *pLen = MIN(maxLen, (hidReportMapLen - offset));

      // Copy data.
      memcpy(pValue, pAttr->pValue + offset, *pLen);
    }
  }
  else if (uuid == HID_INFORMATION_UUID)
  {
    *pLen = HID_INFORMATION_LEN;
    memcpy(pValue, pAttr->pValue, HID_INFORMATION_LEN);
  }
  else if (uuid == GATT_REPORT_REF_UUID)
  {
    *pLen = HID_REPORT_REF_LEN;
    memcpy(pValue, pAttr->pValue, HID_REPORT_REF_LEN);
  }
  else if (uuid == PROTOCOL_MODE_UUID)
  {
    *pLen = HID_PROTOCOL_MODE_LEN;
    pValue[0] = pAttr->pValue[0];
  }
  else if (uuid == GATT_EXT_REPORT_REF_UUID)
  {
    *pLen = HID_EXT_REPORT_REF_LEN;
    memcpy(pValue, pAttr->pValue, HID_EXT_REPORT_REF_LEN);
  }

  // Restart idle timer.
  if (status == SUCCESS)
  {
    HidDev_StartIdleTimer();
  }

  return (status);
}

/*********************************************************************
 * @fn      HidDev_WriteAttrCB
 *
 * @brief   HID Dev attribute write callback.
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
bStatus_t HidDev_WriteAttrCB(uint16_t connHandle, gattAttribute_t *pAttr,
                             uint8_t *pValue, uint16_t len, uint16_t offset,
                             uint8_t method)
{
  bStatus_t   status = SUCCESS;
  hidRptMap_t *pRpt;

  // Make sure it's not a blob operation (no attributes in the profile are long).
  if (offset > 0)
  {
    return (ATT_ERR_ATTR_NOT_LONG);
  }

  uint16_t uuid = BUILD_UINT16(pAttr->type.uuid[0], pAttr->type.uuid[1]);

  if (uuid == REPORT_UUID || uuid == BOOT_KEY_OUTPUT_UUID)
  {
    // Find report ID in table.
    if ((pRpt = HidDev_reportByHandle(pAttr->handle)) != NULL)
    {
      // Execute report callback.
      status  = (*pHidDevCB->reportCB)(pRpt->id, pRpt->type, uuid,
                                        HID_DEV_OPER_WRITE, &len, pValue);
    }
  }
  else if (uuid == HID_CTRL_PT_UUID)
  {
    // Validate length and value range.
    if (len == 1)
    {
      if (pValue[0] == HID_CMD_SUSPEND ||  pValue[0] == HID_CMD_EXIT_SUSPEND)
      {
        // Execute HID app event callback.
        (*pHidDevCB->evtCB)((pValue[0] == HID_CMD_SUSPEND) ?
                             HID_DEV_SUSPEND_EVT : HID_DEV_EXIT_SUSPEND_EVT);
      }
      else
      {
        status = ATT_ERR_INVALID_VALUE;
      }
    }
    else
    {
      status = ATT_ERR_INVALID_VALUE_SIZE;
    }
  }
  else if (uuid == GATT_CLIENT_CHAR_CFG_UUID)
  {
    status = GATTServApp_ProcessCCCWriteReq(connHandle, pAttr, pValue, len,
                                            offset, GATT_CLIENT_CFG_NOTIFY);
    if (status == SUCCESS)
    {
      uint16_t charCfg = BUILD_UINT16(pValue[0], pValue[1]);

      // Find report ID in table.
      if ((pRpt = HidDev_reportByCccdHandle(pAttr->handle)) != NULL)
      {
        // Execute report callback.
        (*pHidDevCB->reportCB)(pRpt->id, pRpt->type, uuid,
                               (charCfg == GATT_CLIENT_CFG_NOTIFY) ?
                               HID_DEV_OPER_ENABLE : HID_DEV_OPER_DISABLE,
                               &len, pValue);
      }
    }
  }
  else if (uuid == PROTOCOL_MODE_UUID)
  {
    if (len == HID_PROTOCOL_MODE_LEN)
    {
      if (pValue[0] == HID_PROTOCOL_MODE_BOOT ||
          pValue[0] == HID_PROTOCOL_MODE_REPORT)
      {
        pAttr->pValue[0] = pValue[0];

        // Execute HID app event callback.
        (*pHidDevCB->evtCB)((pValue[0] == HID_PROTOCOL_MODE_BOOT) ?
                            HID_DEV_SET_BOOT_EVT : HID_DEV_SET_REPORT_EVT);
      }
      else
      {
        status = ATT_ERR_INVALID_VALUE;
      }
    }
    else
    {
      status = ATT_ERR_INVALID_VALUE_SIZE;
    }
  }

  // Restart idle timer.
  if (status == SUCCESS)
  {
    HidDev_StartIdleTimer();
  }

  return (status);
}

/*********************************************************************
 * @fn      HidDev_StartIdleTimer
 *
 * @brief   Start the idle timer.
 *
 * @return  None.
 */
void HidDev_StartIdleTimer(void)
{
  if ((pHidDevCfg != NULL) && (pHidDevCfg->idleTimeout > 0))
  {
    Util_startClock(&idleTimeoutClock);
  }
}

/*********************************************************************
 * @fn      HidDev_StopIdleTimer
 *
 * @brief   Stop the idle timer.
 *
 * @param   None.
 *
 * @return  None.
 */
void HidDev_StopIdleTimer(void)
{
  if ((pHidDevCfg != NULL) && (pHidDevCfg->idleTimeout > 0))
  {
    Util_stopClock(&idleTimeoutClock);
  }
}

/*********************************************************************
 * @fn      HidDev_StartAdvertising
 *
 * @brief   Start advertising.
 *
 * @param   None.
 *
 * @return  None.
 */
void HidDev_StartAdvertising(void)
{
  // If previously bonded
  if (HidDev_bondCount() > 0)
  {
    // Start high duty cycle advertising.
    HidDev_highAdvertising();
  }
  // Else not bonded.
  else
  {
    // Start initial advertising.
    HidDev_initialAdvertising();
  }
}

/*********************************************************************
 * @fn      HidDev_processAppMsg
 *
 * @brief   Process an incoming callback from a profile.
 *
 * @param   pMsg - message to process
 *
 * @return  none
 */
static void HidDev_processAppMsg(hidDevEvt_t *pMsg)
{
  switch (pMsg->hdr.event)
  {
    case HID_STATE_CHANGE_EVT:
      HidDev_processStateChangeEvt((gaprole_States_t) pMsg->hdr.state);
      break;

    case HID_BATT_SERVICE_EVT:
      HidDev_processBatteryEvt(pMsg->hdr.state);
      break;

    case HID_PAIR_STATE_EVT:
      HidDev_processPairStateEvt(pMsg->hdr.state, *pMsg->pData);

      ICall_free(pMsg->pData);
      break;

    case HID_PASSCODE_EVT:
      {
        hidDevPasscodeEvt_t *pc = (hidDevPasscodeEvt_t *)pMsg->pData;

        HidDev_processPasscodeEvt(pc->deviceAddr, pc->connHandle,
                                  pc->uiInputs, pc->uiOutputs);

        ICall_free(pMsg->pData);
      }
      break;

    default:
      // Do nothing.
      break;
  }
}

/*********************************************************************
 * @fn      HidDev_processStackMsg
 *
 * @brief   Process an incoming task message.
 *
 * @param   pMsg - message to process
 *
 * @return  none
 */
static void HidDev_processStackMsg(ICall_Hdr *pMsg)
{
  switch (pMsg->event)
  {
    case GATT_MSG_EVENT:
      HidDev_processGattMsg((gattMsgEvent_t *) pMsg);
      break;

    default:
      // Do nothing.
      break;
  }
}

/*********************************************************************
 * @fn      HidDev_processGattMsg
 *
 * @brief   Process GATT messages
 *
 * @return  none
 */
static void HidDev_processGattMsg(gattMsgEvent_t *pMsg)
{
  GATT_bm_free(&pMsg->msg, pMsg->method);
}

/*********************************************************************
 * @fn      HidDev_stateChangeCB
 *
 * @brief   Notification from the profile of a state change.
 *
 * @param   newState - new state
 *
 * @return  none
 */
static void HidDev_stateChangeCB(gaprole_States_t newState)
{
  // Enqueue the message.
  HidDev_enqueueMsg(HID_STATE_CHANGE_EVT, newState, NULL);
}

/*********************************************************************
 * @fn      HidDev_processStateChangeEvt
 *
 * @brief   Notification from the profile of a state change.
 *
 * @param   newState - new state
 *
 * @return  none
 */
static void HidDev_processStateChangeEvt(gaprole_States_t newState)
{
  // If connected
  if (newState == GAPROLE_CONNECTED)
  {
    uint8_t param = FALSE;

    // Get connection handle.
    GAPRole_GetParameter(GAPROLE_CONNHANDLE, &gapConnHandle);

    // Connection not secure yet.
    hidDevConnSecure = FALSE;

    // Don't start advertising when connection is closed.
    GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8_t), &param);

    // Start idle timer.
    HidDev_StartIdleTimer();

    // If there are reports in the queue
    if (!reportQEmpty())
    {
      // Set another event.
      events |= HID_SEND_REPORT_EVT;

      // Post the profile's semaphore.
      Semaphore_post(sem);
    }
  }
  // If disconnected
  else if (hidDevGapState == GAPROLE_CONNECTED &&
            newState != GAPROLE_CONNECTED)
  {
    HidDev_disconnected();

    updateConnParams = TRUE;

    if (pairingStatus == SMP_PAIRING_FAILED_CONFIRM_VALUE)
    {
      // Bonding failed due to mismatched confirm values.
      HidDev_initialAdvertising();

      pairingStatus = SUCCESS;
    }
#if AUTO_ADV
    else
    {
      uint8_t advState = TRUE;

      GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8_t), &advState);
    }
#endif //AUTO_ADV
  }
  // If started
  else if (newState == GAPROLE_STARTED)
  {
    // Nothing to do for now!
  }

  // Update GAP state
  hidDevGapState = newState;

  // Notify application
  (*pHidDevCB->evtCB)(HID_DEV_GAPROLE_STATE_CHANGE_EVT);
}

/*********************************************************************
 * @fn      HidDev_disconnected
 *
 * @brief   Handle disconnect.
 *
 * @return  none
 */
static void HidDev_disconnected(void)
{
  // Stop idle timer.
  HidDev_StopIdleTimer();

  // Reset state variables.
  hidDevConnSecure = FALSE;
  hidProtocolMode = HID_PROTOCOL_MODE_REPORT;
  hidDevPairingStarted = FALSE;
  hidDevGapBondPairingState = HID_GAPBOND_PAIRING_STATE_NONE;

  // Reset last report sent out
  memset(&lastReport, 0, sizeof(hidDevReport_t));

  // If bonded and normally connectable start advertising.
  if ((HidDev_bondCount() > 0) &&
      (pHidDevCfg->hidFlags & HID_FLAGS_NORMALLY_CONNECTABLE))
  {
    HidDev_lowAdvertising();
  }

  // Notify application
  (*pHidDevCB->evtCB)(HID_DEV_GAPBOND_STATE_CHANGE_EVT);
}

/*********************************************************************
 * @fn      HidDev_pairStateCB
 *
 * @brief   Pairing state callback.
 *
 * @param   connHandle - connection handle.
 * @param   state      - pairing state
 * @param   status     - status upon entering this state.
 *
 * @return  none
 */
static void HidDev_pairStateCB(uint16_t connHandle, uint8_t state,
                               uint8_t status)
{
  uint8_t *pData;

  // Allocate message data
  if ((pData = ICall_malloc(sizeof(uint8_t))))
  {
    *pData = status;

    // Queue the event.
    HidDev_enqueueMsg(HID_PAIR_STATE_EVT, state, pData);
  }
}

/*********************************************************************
 * @fn      HidDev_processPairStateEvt
 *
 * @brief   Process pairing state callback.
 *
 * @param   state  - pairing state
 * @param   status - status upon entering this state.
 *
 * @return  none
 */
static void HidDev_processPairStateEvt(uint8_t state, uint8_t status)
{
  if (state == GAPBOND_PAIRING_STATE_STARTED)
  {
    hidDevPairingStarted = TRUE;
  }
  else if (state == GAPBOND_PAIRING_STATE_COMPLETE)
  {
    hidDevPairingStarted = FALSE;
    pairingStatus = status;

    if (status == SUCCESS)
    {
      hidDevConnSecure = TRUE;
      Util_restartClock(&reportReadyClock, HID_REPORT_READY_TIME);
    }
  }
  else if (state == GAPBOND_PAIRING_STATE_BONDED)
  {
    if (status == SUCCESS)
    {
      hidDevConnSecure = TRUE;
      Util_restartClock(&reportReadyClock, HID_REPORT_READY_TIME);

#if DEFAULT_SCAN_PARAM_NOTIFY_TEST == TRUE
      ScanParam_RefreshNotify(gapConnHandle);
#endif
    }
  }

  // Update GAP Bond pairing state
  hidDevGapBondPairingState = state;

  // Notify application
  (*pHidDevCB->evtCB)(HID_DEV_GAPBOND_STATE_CHANGE_EVT);

  // Process HID reports
  if (!reportQEmpty() && hidDevConnSecure)
  {
    // Notify our task to send out pending reports.
    events |= HID_SEND_REPORT_EVT;

    // Post the service's semaphore.
    Semaphore_post(sem);
  }
}

/*********************************************************************
 * @fn      HidDev_passcodeCB
 *
 * @brief   Passcode callback.
 *
 * @param   deviceAddr - address of device to pair with, and could be either
 *                       public or random.
 * @param   connHandle - connection handle
 * @param   uiInputs   - pairing User Interface Inputs - Ask user to input
 *                       passcode.
 * @param   uiOutputs  - pairing User Interface Outputs - Display passcode.
 *
 * @return  none
 */
static void HidDev_passcodeCB(uint8_t *deviceAddr, uint16_t connHandle,
                                        uint8_t uiInputs, uint8_t uiOutputs)
{
  hidDevPasscodeEvt_t *pcEvt;

  // Allocate message data.
  if ((pcEvt = ICall_malloc(sizeof(hidDevPasscodeEvt_t))))
  {
    // Store the arguments.
    memcpy(pcEvt->deviceAddr, deviceAddr, B_ADDR_LEN);

    pcEvt->connHandle = connHandle;
    pcEvt->uiInputs = uiInputs;
    pcEvt->uiOutputs = uiOutputs;

    // Queue the event.
    HidDev_enqueueMsg(HID_PASSCODE_EVT, 0, (uint8_t *)pcEvt);
  }
}

/*********************************************************************
 * @fn      HidDev_processPasscodeEvt
 *
 * @brief   Process passcode callback.
 *
 * @param   deviceAddr - address of device to pair with, and could be either
 *                       public or random.
 * @param   connHandle - connection handle
 * @param   uiInputs   - pairing User Interface Inputs - Ask user to input
 *                       passcode.
 * @param   uiOutputs  - pairing User Interface Outputs - Display passcode.
 *
 * @return  none
 */
static void HidDev_processPasscodeEvt(uint8_t *deviceAddr,
                                      uint16_t connHandle,
                                      uint8_t uiInputs, uint8_t uiOutputs)
{
  if (pHidDevCB && pHidDevCB->passcodeCB)
  {
    // Execute HID app passcode callback.
    (*pHidDevCB->passcodeCB)(deviceAddr, connHandle, uiInputs, uiOutputs);
  }
  else
  {
    // Send passcode response.
    GAPBondMgr_PasscodeRsp(connHandle, SUCCESS, 0);
  }
}

/*********************************************************************
 * @fn      HidDev_batteryCB
 *
 * @brief   Callback function for battery service.
 *
 * @param   event - service event
 *
 * @return  none
 */
static void HidDev_batteryCB(uint8_t event)
{
  // Queue the event.
  HidDev_enqueueMsg(HID_BATT_SERVICE_EVT, event, NULL);
}

/*********************************************************************
 * @fn      HidDev_processBatteryEvt
 *
 * @brief   Processes callback from the battery service.
 *
 * @param   event - service event
 *
 * @return  none
 */
static void HidDev_processBatteryEvt(uint8_t event)
{
  if (event == BATT_LEVEL_NOTI_ENABLED)
  {
    // If connected start periodic measurement.
    if (hidDevGapState == GAPROLE_CONNECTED)
    {
      Util_startClock(&battPerClock);
    }
  }
  else if (event == BATT_LEVEL_NOTI_DISABLED)
  {
    // Stop periodic measurement.
    Util_stopClock(&battPerClock);
  }
}

/*********************************************************************
 * @fn      HidDev_scanParamCB
 *
 * @brief   Callback function for scan parameter service.
 *
 * @param   event - service event
 *
 * @return  none
 */
static void HidDev_scanParamCB(uint8_t event)
{
  // Do nothing.
}

/*********************************************************************
 * @fn      HidDev_battPeriodicTask
 *
 * @brief   Perform a periodic task for battery measurement.
 *
 * @param   none
 *
 * @return  none
 */
static void HidDev_battPeriodicTask(void)
{
  if (hidDevGapState == GAPROLE_CONNECTED)
  {
    // Perform battery level check.
    Batt_MeasLevel();

    // Restart clock.
    Util_startClock(&battPerClock);
  }
}

/*********************************************************************
 * @fn      HidDev_reportByHandle
 *
 * @brief   Find the HID report structure for the given handle.
 *
 * @param   handle - ATT handle
 *
 * @return  Pointer to HID report structure
 */
static hidRptMap_t *HidDev_reportByHandle(uint16_t handle)
{
  uint8_t i;
  hidRptMap_t *p = pHidDevRptTbl;

  for (i = hidDevRptTblLen; i > 0; i--, p++)
  {
    if (p->handle == handle && p->mode == hidProtocolMode)
    {
      return p;
    }
  }

  return NULL;
}

/*********************************************************************
 * @fn      HidDev_reportByCccdHandle
 *
 * @brief   Find the HID report structure for the given CCC handle.
 *
 * @param   handle - ATT handle
 *
 * @return  Pointer to HID report structure
 */
static hidRptMap_t *HidDev_reportByCccdHandle(uint16_t handle)
{
  uint8_t i;
  hidRptMap_t *p = pHidDevRptTbl;

  for (i = hidDevRptTblLen; i > 0; i--, p++)
  {
    if ((p->pCccdAttr != NULL) && (p->pCccdAttr->handle == handle))
    {
      return p;
    }
  }

  return NULL;
}

/*********************************************************************
 * @fn      HidDev_reportById
 *
 * @brief   Find the HID report structure for the Report ID and type.
 *
 * @param   id   - HID report ID
 * @param   type - HID report type
 *
 * @return  Pointer to HID report structure
 */
static hidRptMap_t *HidDev_reportById(uint8_t id, uint8_t type)
{
  uint8_t i;
  hidRptMap_t *p = pHidDevRptTbl;

  for (i = hidDevRptTblLen; i > 0; i--, p++)
  {
    if (p->id == id && p->type == type && p->mode == hidProtocolMode)
    {
      return p;
    }
  }

  return NULL;
}

/*********************************************************************
 * @fn      HidDev_sendReport
 *
 * @brief   Send a HID report.
 *
 * @param   id    - HID report ID.
 * @param   type  - HID report type.
 * @param   len   - Length of report.
 * @param   pData - Report data.
 *
 * @return  None.
 */
static void HidDev_sendReport(uint8_t id, uint8_t type, uint8_t len, uint8_t *pData)
{
  hidRptMap_t *pRpt;

  // Get ATT handle for report.
  if ((pRpt = HidDev_reportById(id, type)) != NULL)
  {
    uint8_t value = GATTServApp_ReadCharCfg(gapConnHandle,
                                            GATT_CCC_TBL(pRpt->pCccdAttr->pValue));

    // If notifications are enabled
    if (value & GATT_CLIENT_CFG_NOTIFY)
    {
      // After service discovery and encryption, the HID Device should
      // request to change to the preferred connection parameters that best
      // suit its use case.
      if (updateConnParams)
      {
        GAPRole_SetParameter(GAPROLE_PARAM_UPDATE_REQ, sizeof(uint8_t),
                             &updateConnParams);

        updateConnParams = FALSE;
      }

      // Send report notification
      if (HidDev_sendNoti(pRpt->handle, len, pData) == SUCCESS)
      {
        // Save the report just sent out
        lastReport.id = id;
        lastReport.type = type;
        lastReport.len = len;
        memcpy(lastReport.data, pData, len);
      }

      // Start idle timer.
      HidDev_StartIdleTimer();
    }
  }
}

/*********************************************************************
 * @fn      hidDevSendNoti
 *
 * @brief   Send a HID notification.
 *
 * @param   handle - Attribute handle.
 * @param   len - Length of report.
 * @param   pData - Report data.
 *
 * @return  Success or failure.
 */
static uint8_t HidDev_sendNoti(uint16_t handle, uint8_t len, uint8_t *pData)
{
  uint8_t status;
  attHandleValueNoti_t noti;

  noti.pValue = GATT_bm_alloc(gapConnHandle, ATT_HANDLE_VALUE_NOTI, len, NULL);
  if (noti.pValue != NULL)
  {
    noti.handle = handle;
    noti.len = len;
    memcpy(noti.pValue, pData, len);

    // Send notification
    status = GATT_Notification(gapConnHandle, &noti, FALSE);
    if (status != SUCCESS)
    {
      GATT_bm_free((gattMsg_t *)&noti, ATT_HANDLE_VALUE_NOTI);
    }
  }
  else
  {
    status = bleMemAllocError;
  }

  return status;
}

/*********************************************************************
 * @fn      HidDev_enqueueReport
 *
 * @brief   Enqueue a HID report to be sent later.
 *
 * @param   id    - HID report ID.
 * @param   type  - HID report type.
 * @param   len   - Length of report.
 * @param   pData - Report data.
 *
 * @return  None.
 */
static void HidDev_enqueueReport(uint8_t id, uint8_t type, uint8_t len,
                                 uint8_t *pData)
{
  // Enqueue only if bonded.
  if (HidDev_bondCount() > 0)
  {
    // Update last index.
    lastQIdx = (lastQIdx + 1) % HID_DEV_REPORT_Q_SIZE;

    if (lastQIdx == firstQIdx)
    {
      // Queue overflow; discard oldest report.
      firstQIdx = (firstQIdx + 1) % HID_DEV_REPORT_Q_SIZE;
    }

    // Save report.
    hidDevReportQ[lastQIdx].id = id;
    hidDevReportQ[lastQIdx].type = type;
    hidDevReportQ[lastQIdx].len = len;
    memcpy(hidDevReportQ[lastQIdx].data, pData, len);

    if (hidDevConnSecure)
    {
      // Notify our task to send out pending reports.
      events |= HID_SEND_REPORT_EVT;

      // Post the service's semaphore.
      Semaphore_post(sem);
    }
  }
}

/*********************************************************************
 * @fn      HidDev_dequeueReport
 *
 * @brief   Dequeue a HID report to be sent out.
 *
 * @param   id    - HID report ID.
 * @param   type  - HID report type.
 * @param   len   - Length of report.
 * @param   pData - Report data.
 *
 * @return  None.
 */
static hidDevReport_t *HidDev_dequeueReport(void)
{
  if (reportQEmpty())
  {
    return NULL;
  }

  // Update first index.
  firstQIdx = (firstQIdx + 1) % HID_DEV_REPORT_Q_SIZE;

  return (&(hidDevReportQ[firstQIdx]));
}

/*********************************************************************
 * @fn      HidDev_highAdvertising
 *
 * @brief   Start advertising at a high duty cycle.

 * @param   None.
 *
 * @return  None.
 */
static void HidDev_highAdvertising(void)
{
  uint8_t param;

  VOID GAP_SetParamValue(TGAP_LIM_DISC_ADV_INT_MIN, HID_HIGH_ADV_INT_MIN);
  VOID GAP_SetParamValue(TGAP_LIM_DISC_ADV_INT_MAX, HID_HIGH_ADV_INT_MAX);
  VOID GAP_SetParamValue(TGAP_LIM_ADV_TIMEOUT, HID_HIGH_ADV_TIMEOUT);

  // Setup advertising filter policy first.
  param = HID_AUTO_SYNC_WL ? GAP_FILTER_POLICY_WHITE : GAP_FILTER_POLICY_ALL;
  VOID GAPRole_SetParameter(GAPROLE_ADV_FILTER_POLICY, sizeof(uint8_t), &param);

  param = TRUE;
  GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8_t), &param);
}

/*********************************************************************
 * @fn      HidDev_lowAdvertising
 *
 * @brief   Start advertising at a low duty cycle.
 *
 * @param   None.
 *
 * @return  None.
 */
static void HidDev_lowAdvertising(void)
{
  uint8_t param;

  VOID GAP_SetParamValue(TGAP_LIM_DISC_ADV_INT_MIN, HID_LOW_ADV_INT_MIN);
  VOID GAP_SetParamValue(TGAP_LIM_DISC_ADV_INT_MAX, HID_LOW_ADV_INT_MAX);
  VOID GAP_SetParamValue(TGAP_LIM_ADV_TIMEOUT, HID_LOW_ADV_TIMEOUT);

  // Setup advertising filter policy first.
  param = HID_AUTO_SYNC_WL ? GAP_FILTER_POLICY_WHITE : GAP_FILTER_POLICY_ALL;
  VOID GAPRole_SetParameter(GAPROLE_ADV_FILTER_POLICY, sizeof(uint8_t), &param);

  param = TRUE;
  VOID GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8_t), &param);
}

/*********************************************************************
 * @fn      HidDev_initialAdvertising
 *
 * @brief   Start advertising for initial connection.
 *
 * @return  None.
 */
static void HidDev_initialAdvertising(void)
{
  uint8_t param;

  VOID GAP_SetParamValue(TGAP_LIM_DISC_ADV_INT_MIN, HID_INITIAL_ADV_INT_MIN);
  VOID GAP_SetParamValue(TGAP_LIM_DISC_ADV_INT_MAX, HID_INITIAL_ADV_INT_MAX);
  VOID GAP_SetParamValue(TGAP_LIM_ADV_TIMEOUT, HID_INITIAL_ADV_TIMEOUT);

  // Setup advertising filter policy first.
  param = GAP_FILTER_POLICY_ALL;
  VOID GAPRole_SetParameter(GAPROLE_ADV_FILTER_POLICY, sizeof(uint8_t), &param);

  param = TRUE;
  VOID GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8_t), &param);
}

/*********************************************************************
 * @fn      HidDev_bondCount
 *
 * @brief   Gets the total number of bonded devices.
 *
 * @param   None.
 *
 * @return  number of bonded devices.
 */
static uint8_t HidDev_bondCount(void)
{
  uint8_t bondCnt = 0;

  VOID GAPBondMgr_GetParameter(GAPBOND_BOND_COUNT, &bondCnt);

  return bondCnt;
}

/*********************************************************************
 * @fn      HidDev_isbufset
 *
 * @brief   Is all of the array elements set to a value?
 *
 * @param   buf - buffer to check.
 * @param   val - value to check each array element for.
 * @param   len - length to check.
 *
 * @return  TRUE if all "val".
 *          FALSE otherwise.
 */
static uint8_t HidDev_isbufset(uint8_t *buf, uint8_t val, uint8_t len)
{
  uint8_t x;

  // Validate pointer and length of report
  if ((buf == NULL) || (len >  HID_DEV_DATA_LEN))
  {
    return ( FALSE );
  }

  for ( x = 0; x < len; x++ )
  {
    // Check for non-initialized value
    if ( buf[x] != val )
    {
      return ( FALSE );
    }
  }

  return ( TRUE );
}

/*********************************************************************
 * @fn      HidDev_clockHandler
 *
 * @brief   Clock handle for all clock events.  This function stores an event
 *          flag and wakes up the application's event processor.
 *
 * @param   arg - event flag.
 *
 * @return  None
 */
static void HidDev_clockHandler(UArg arg)
{
  // Store the event.
  events |= arg;

  // Wake up the application.
  Semaphore_post(sem);
}

/*********************************************************************
 * @fn      HidDev_enqueueMsg
 *
 * @brief   Creates a message and puts the message in RTOS queue.
 *
 * @param   event  - message event.
 * @param   state  - message state.
 * @param   pData  - message data pointer.
 *
 * @return  TRUE or FALSE
 */
static uint8_t HidDev_enqueueMsg(uint16_t event, uint8_t state, uint8_t *pData)
{
  hidDevEvt_t *pMsg;

  // Create dynamic pointer to message.
  if (pMsg = ICall_malloc(sizeof(hidDevEvt_t)))
  {
    pMsg->hdr.event = event;
    pMsg->hdr.state = state;
    pMsg->pData = pData;

    // Enqueue the message.
    return Util_enqueueMsg(appMsgQueue, sem, (uint8_t *)pMsg);
  }

  return FALSE;
}

/*********************************************************************
 * @fn      HidDev_reportReadyClockCB
 *
 * @brief   Handles HID reports when delay has expired
 *
 * @param   None.
 *
 * @return  None.
 */
static void HidDev_reportReadyClockCB(UArg a0)
{
  // Allow reports to be sent
  hidDevReportReadyState = TRUE;

  // If there are reports in the queue
  if (!reportQEmpty())
  {
    // Set another event.
    events |= HID_SEND_REPORT_EVT;

    // Post the profile's semaphore.
    Semaphore_post(sem);
  }
}

/*********************************************************************
*********************************************************************/
