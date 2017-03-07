/******************************************************************************

 @file  clicker.c

 @brief This file contains a BLE HID Clicker implementation,
        based on the HID Keyboard service, Simple BLE Peripheral and Project zero examples from the Simplelink BLE SDK.

 Target Device: CC2650

 ******************************************************************************
 
 Copyright (c) 2013-2016, Texas Instruments Incorporated
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

/* RTOS */
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Queue.h>

/* XDC */
#include <xdc/runtime/Log.h>

/* BLE SDK */
#include "gatt.h"
#include "peripheral.h"
#include "gapbondmgr.h"

#include "osal_snv.h"
#include "gattservapp.h"
#include "icall_apimsg.h"
#include "gatt_profile_uuid.h"

/* HOGP */
#include "hiddev.h"
#include "hidkbdservice.h"
#include "battservice.h"

#ifdef USE_RCOSC
#include "rcosc_calibration.h"
#endif //USE_RCOSC

/* Internal */
#include "util.h"
#include "board_key.h"
#include "board.h"

#include "clicker.h"

/*********************************************************************
 * CONSTANTS
 */

// Advertising interval when device is discoverable (units of 625us, 160=100ms)
#define DEFAULT_ADVERTISING_INTERVAL          160

// Limited discoverable mode advertises for 30.72s, and then stops
// General discoverable mode advertises indefinitely
// I set here LIMITED for better battery consumption when using the clicker on batteries.
#define DEFAULT_DISCOVERABLE_MODE             GAP_ADTYPE_FLAGS_LIMITED

/** Connection Parameters **/
/* Connection Parameter Specifications from the internet:
 *
 * Connection interval:
 * Determines how often the Central will ask for data from the Peripheral.
 * When the Peripheral requests an update, it supplies a maximum and a minimum wanted interval.
 * The connection interval must be between 7.5 ms and 4 s.
 *
 * Slave latency:
 * By setting a non-zero slave latency, the Peripheral can choose to not answer when the Central asks for data up to the slave latency number of times.
 * However, if the Peripheral has data to send, it can choose to send data at any time.
 * This enables a peripheral to stay sleeping for a longer time, if it doesn't have data to send, but still send data fast if needed.
 * The text book example of such device is for example keyboard and mice,
 * which want to be sleeping for as long as possible when there is no data to send,
 * but still have low latency (and for the mouse: low connection interval) when needed.
 *
 * Connection supervision timeout:
 * This timeout determines the timeout from the last data exchange till a link is considered lost.
 * A Central will not start trying to reconnect before the timeout has passed, so if you have a device which goes in and out of range often,
 * and you need to notice when that happens, it might make sense to have a short timeout.
 */

// Minimum connection interval (units of 1.25ms, 80=100ms) if automatic
// parameter update request is enabled (and it is)
// Can range from 6 (7.5ms) to 3200 (4s)
// This affects power consumption
// I choose the minimal, for best clicker latency.
#define DEFAULT_DESIRED_MIN_CONN_INTERVAL     6

// Maximum connection interval (units of 1.25ms, 800=1000ms) if automatic
// parameter update request is enabled
// This is much lower then default for better clicker latency.
#define DEFAULT_DESIRED_MAX_CONN_INTERVAL     40

// Slave latency to use if automatic parameter update request is enabled
/**
    According to dev's guide (swru393) section 5.1.3:
 *  Reducing the slave latency (or setting it to zero) does as follows:
        * Increases the power consumption for the peripheral device
        * Reduces the time for the peripheral device to receive the data sent from a central device
 *  Increasing the slave latency does as follows:
        * Reduces power consumption for the peripheral during periods when the peripheral has no data to send
        to the central device
        * Increases the time for the peripheral device to receive the data sent from the central device
    So for our application we have no reason to keep it low, as we want to save power
    and we have nearly to no data sent from the central to the peripheral.
 */
#define DEFAULT_DESIRED_SLAVE_LATENCY         40

// Supervision timeout value (units of 10ms, 1000=10s) if automatic parameter
// update request is enabled
#define DEFAULT_DESIRED_CONN_TIMEOUT          1000

// Whether to enable automatic parameter update request when a connection is
// formed
#define DEFAULT_ENABLE_UPDATE_REQUEST         GAPROLE_LINK_PARAM_UPDATE_INITIATE_BOTH_PARAMS

// Connection Pause Peripheral time value (in seconds)
#define DEFAULT_CONN_PAUSE_PERIPHERAL         6

/** End of Connection Parameters **/

// Task configuration
#define CLKR_TASK_PRIORITY                     1


#ifndef CLKR_TASK_STACK_SIZE
#define CLKR_TASK_STACK_SIZE                   644
#endif

// Internal Events for RTOS application
#define CLKR_STATE_CHANGE_EVT                  0x0001
#define CLICKER_CLK_EVT                        0x0002

/** HID Related **/
// timeout in ms. 0 disables timeout
#define HID_DEV_IDLE_TIMEOUT  30000
// After 30 seconds disconnects. Once clicking a button, reconnects automatically.


/*********************************************************************
 * TYPEDEFS
 */

// App event passed from profiles.
typedef struct
{
  appEvtHdr_t hdr;  // event header.
} clickerEvt_t;

typedef struct
{
    // According to USB_HID1_11, Appendix B, section 1.
    uint8_t modifierKeys; // aka ctrl, alt, etc.
    uint8_t reserved; // constant OEM used value. Recommended 0.
    uint8_t keyCode1;
    uint8_t keyCode2;
    uint8_t keyCode3;
    uint8_t keyCode4;
    uint8_t keyCode5;
    uint8_t keyCode6;
} keyboardInputReport_t;

/*********************************************************************
 * GLOBAL VARIABLES
 */


/*********************************************************************
 * LOCAL VARIABLES
 */

/** App Variables **/

// Entity ID globally used to check for source and/or destination of messages
static ICall_EntityID selfEntity;

// Semaphore globally used to post events to the application thread
static ICall_Semaphore sem;

// Queue object used for app messages
static Queue_Struct appMsg;
static Queue_Handle appMsgQueue;

// Task configuration
Task_Struct clickerTask;
Char clickerTaskStack[CLKR_TASK_STACK_SIZE];

/** BLE Variables **/

// GAP - SCAN RSP data (max size = 31 bytes)
static uint8_t scanRspData[] =
{
  // complete name
  0x8,   // length of this data
  GAP_ADTYPE_LOCAL_NAME_COMPLETE,
  'C',
  'l',
  'i',
  'c',
  'k',
  'e',
  'r',

  // connection interval range
  0x05,   // length of this data
  GAP_ADTYPE_SLAVE_CONN_INTERVAL_RANGE,
  LO_UINT16(DEFAULT_DESIRED_MIN_CONN_INTERVAL),
  HI_UINT16(DEFAULT_DESIRED_MIN_CONN_INTERVAL),
  LO_UINT16(DEFAULT_DESIRED_MAX_CONN_INTERVAL),
  HI_UINT16(DEFAULT_DESIRED_MAX_CONN_INTERVAL),

  // Tx power level
  0x02,   // length of this data
  GAP_ADTYPE_POWER_LEVEL,
  0       // 0dBm
};

// GAP - Advertisement data (max size = 31 bytes, though this is
// best kept short to conserve power while advertisting)
static uint8_t advertData[] =
{
  // Flags; this sets the device to use limited discoverable
  // mode (advertises for 30 seconds at a time) instead of general
  // discoverable mode (advertises indefinitely)
  0x02,   // length of this data
  GAP_ADTYPE_FLAGS,
  DEFAULT_DISCOVERABLE_MODE | GAP_ADTYPE_FLAGS_BREDR_NOT_SUPPORTED,

  // Appearance. Show this device as a keyboard on compatible hosts.
  0x03,
  GAP_ADTYPE_APPEARANCE, // 1 byte
  LO_UINT16(GAP_APPEARE_HID_KEYBOARD),
  HI_UINT16(GAP_APPEARE_HID_KEYBOARD),

  // service UUID, to notify central devices what services are included
  // in this peripheral
  0x03,                         // length of this data
  GAP_ADTYPE_16BIT_MORE,        // some of the UUID's, but not all
  LO_UINT16(HID_SERV_UUID),
  HI_UINT16(HID_SERV_UUID)
};

// GAP GATT Attributes
static uint8_t attDeviceName[GAP_DEVICE_NAME_LEN] = "Clicker";
static uint16_t attDeviceAppearance = GAP_APPEARE_HID_KEYBOARD;


/** HID Dev Variables **/

// HID dev configuration structure
static hidDevCfg_t Clicker_hidDevCfg =
{
  // uint32_t    idleTimeout;       // Idle timeout in milliseconds
  HID_DEV_IDLE_TIMEOUT,
  // uint8_t     hidFlags;          // HID feature flags
  HID_KBD_FLAGS                     // Actually: RemoteWake. Defined in hidkbdservice
};

/*********************************************************************
 * LOCAL FUNCTIONS
 */

static void Clicker_init( void );
static void Clicker_taskFxn(UArg a0, UArg a1);

static uint8_t Clicker_processStackMsg(ICall_Hdr *pMsg);
static uint8_t Clicker_processGATTMsg(gattMsgEvent_t *pMsg);
static void Clicker_processAppMsg(clickerEvt_t *pMsg);

static void Clicker_enqueueMsg(uint8_t event, uint8_t state);

static void Clicker_processClkEvt(uint8_t keysPressed);

/** HID Dev related functions **/
static uint8_t Clicker_hidDevReportCB(uint8_t id, uint8_t type, uint16_t uuid, uint8_t oper, uint16_t *pLen, uint8_t *pData);
static void Clicker_hidDevEvtCB(uint8_t evt);

/** Button clicking related functions **/
static void Clicker_buttonPressedCB(uint8_t keysPressed); // keysPressedCB_t

/** Utils **/
static char *Util_getLocalNameStr(const uint8_t *data);

/*********************************************************************
 * PROFILE CALLBACKS
 */

static hidDevCB_t Clicker_CBs =
{
     Clicker_hidDevReportCB, // HID Report Callback
     Clicker_hidDevEvtCB, // HID event callback
     NULL // HID Passcode callback, nullable
};

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      Clicker_createTask
 *
 * @brief   Task creation function for the Clicker app.
 *
 * @param   None.
 *
 * @return  None.
 */
void Clicker_createTask(void)
{
  Task_Params taskParams;

  // Configure task
  Task_Params_init(&taskParams);
  taskParams.stack = clickerTaskStack;
  taskParams.stackSize = CLKR_TASK_STACK_SIZE;
  taskParams.priority = CLKR_TASK_PRIORITY;

  Task_construct(&clickerTask, Clicker_taskFxn, &taskParams, NULL);
}

/*********************************************************************
 * IMPLEMENTATIONS
 */

/** External Callbacks **/

/*********************************************************************
 * @fn      Clicker_hidDevReportCB
 * @fntype  hidDevReportCB_t
 *
 * @brief   HID Report callback
 *
 * @return  Error code.
 */
static uint8_t Clicker_hidDevReportCB(uint8_t id, uint8_t type, uint16_t uuid,
                                   uint8_t oper, uint16_t *pLen, uint8_t *pData) {
    // This does nothing. It is nice for debugging and showcase purposes.
    Log_info4("id: %d, type: %d, uuid: %d, oper: %d", id, type, uuid, oper);
    return SUCCESS;
}

/*********************************************************************
 * @fn      Clicker_hidDevEvtCB
 * @fntype  hidDevEvtCB_t
 *
 * @brief   HID event callback
 *
 * @return  Error code.
 */
static void Clicker_hidDevEvtCB(uint8_t evt) {
    Log_info1("Clicker hidDevEvtCB evt: %d", evt);
    if (HID_DEV_GAPROLE_STATE_CHANGE_EVT == evt) {
        Log_info0("HID device state change evt!");
    }
    return;
}

/*********************************************************************
 * @fn      Clicker_buttonPressedCB
 *
 * @brief   Callback being called on a button press.
 *
 * @return  Error code.
 */
static void Clicker_buttonPressedCB(uint8_t keysPressed) {
    Log_info5("Button pressed. Select: %d, Up: %d, Down: %d, Left: %d, Right: %d",
              keysPressed & KEY_SELECT,
              keysPressed & KEY_UP,
              keysPressed & KEY_DOWN,
              keysPressed & KEY_LEFT,
              keysPressed & KEY_RIGHT);
    Clicker_enqueueMsg(CLICKER_CLK_EVT, keysPressed);
}

/** End External Callbacks **/

/*********************************************************************
 * @fn      Clicker_processClkEvt
 *
 * @brief   Handle a key press of the clicker and send the relevant keys.
 *
 * @param   keysPressed - a bit mask of the keys pressed, according to board_key.h.
 *
 * @return  None.
 */
static void Clicker_processClkEvt(uint8_t keysPressed) {
    // We don't need an open connection!
    // Reports are queued and will be sent when connection is established.
    // Also, when HidDev is not advertising, calling HidDev_Report will
    // cause advertising to start.
    keyboardInputReport_t reportData = { 0 };
    if (keysPressed & (KEY_NEXT | KEY_PREV)) {
        if (keysPressed & KEY_NEXT) {
            Log_info0("Clicked next");
            reportData.keyCode1 = HID_KEYBOARD_SPACEBAR;
        } else if (keysPressed & KEY_PREV) {
            Log_info0("Clicked previous");
            reportData.keyCode1 = HID_KEYBOARD_DELETE; // this is backspace; delete is del_fwd
        }

        HidDev_Report(HID_RPT_ID_KEY_IN,
                      HID_REPORT_TYPE_INPUT,
                      sizeof(reportData),
                      (uint8_t*)&reportData);

        // Release the key
        reportData.keyCode1 = HID_KEYBOARD_RESERVED;
        HidDev_Report(HID_RPT_ID_KEY_IN,
                      HID_REPORT_TYPE_INPUT,
                      sizeof(reportData),
                      (uint8_t*)&reportData);
    } else {
        Log_warning1("Weird value for Clicker_buttonPressedCB. keysPressed: %d", keysPressed);
    }
}

/*********************************************************************
 * @fn      Clicker_init
 *
 * @brief   Called during initialization and contains application
 *          specific initialization (ie. hardware initialization/setup,
 *          table initialization, power up notification, etc), and
 *          profile initialization/setup.
 *
 * @param   None.
 *
 * @return  None.
 */
static void Clicker_init(void)
{
  // ******************************************************************
  // N0 STACK API CALLS CAN OCCUR BEFORE THIS CALL TO ICall_registerApp
  // ******************************************************************
  // Register the current thread as an ICall dispatcher application
  // so that the application can send and receive messages.
  ICall_registerApp(&selfEntity, &sem);

#ifdef USE_RCOSC
  RCOSC_enableCalibration();
#endif // USE_RCOSC

  // Create an RTOS queue for message from profile to be sent to app.
  appMsgQueue = Util_constructQueue(&appMsg);

  // Setup the GAP
  GAP_SetParamValue(TGAP_CONN_PAUSE_PERIPHERAL, DEFAULT_CONN_PAUSE_PERIPHERAL);

  // Setup the GAP Peripheral Role Profile
  {
    // For all hardware platforms, device starts advertising upon initialization
    uint8_t initialAdvertEnable = TRUE;

    // By setting this to zero, the device will go into the waiting state after
    // being discoverable for 30.72 second, and will not being advertising again
    // until the enabler is set back to TRUE
    uint16_t advertOffTime = 0;

    uint8_t enableUpdateRequest = DEFAULT_ENABLE_UPDATE_REQUEST;
    uint16_t desiredMinInterval = DEFAULT_DESIRED_MIN_CONN_INTERVAL;
    uint16_t desiredMaxInterval = DEFAULT_DESIRED_MAX_CONN_INTERVAL;
    uint16_t desiredSlaveLatency = DEFAULT_DESIRED_SLAVE_LATENCY;
    uint16_t desiredConnTimeout = DEFAULT_DESIRED_CONN_TIMEOUT;

    // Set the GAP Role Parameters
    GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8_t),
                         &initialAdvertEnable);
    GAPRole_SetParameter(GAPROLE_ADVERT_OFF_TIME, sizeof(uint16_t),
                         &advertOffTime);

    GAPRole_SetParameter(GAPROLE_SCAN_RSP_DATA, sizeof(scanRspData), scanRspData);
    GAPRole_SetParameter(GAPROLE_ADVERT_DATA, sizeof(advertData), advertData);

    Log_info1("Name in scanRspData array: \x1b[33m%s\x1b[0m", (IArg)Util_getLocalNameStr(scanRspData));

    GAPRole_SetParameter(GAPROLE_PARAM_UPDATE_ENABLE, sizeof(uint8_t),
                         &enableUpdateRequest);
    GAPRole_SetParameter(GAPROLE_MIN_CONN_INTERVAL, sizeof(uint16_t),
                         &desiredMinInterval);
    GAPRole_SetParameter(GAPROLE_MAX_CONN_INTERVAL, sizeof(uint16_t),
                         &desiredMaxInterval);
    GAPRole_SetParameter(GAPROLE_SLAVE_LATENCY, sizeof(uint16_t),
                         &desiredSlaveLatency);
    GAPRole_SetParameter(GAPROLE_TIMEOUT_MULTIPLIER, sizeof(uint16_t),
                         &desiredConnTimeout);
  }

  // Set the GAP Characteristics
  GGS_SetParameter(GGS_DEVICE_NAME_ATT, GAP_DEVICE_NAME_LEN, attDeviceName);
  GGS_SetParameter(GGS_APPEARANCE_ATT, sizeof(uint16_t), &attDeviceAppearance);

  // Set advertising interval
  {
    uint16_t advInt = DEFAULT_ADVERTISING_INTERVAL;

    GAP_SetParamValue(TGAP_LIM_DISC_ADV_INT_MIN, advInt);
    GAP_SetParamValue(TGAP_LIM_DISC_ADV_INT_MAX, advInt);
    GAP_SetParamValue(TGAP_GEN_DISC_ADV_INT_MIN, advInt);
    GAP_SetParamValue(TGAP_GEN_DISC_ADV_INT_MAX, advInt);
  }

  // Setup the GAP Bond Manager
  {
    uint32_t passkey = 0; // passkey "000000"
    uint8_t pairMode = GAPBOND_PAIRING_MODE_INITIATE; // GAPBondMgr will send a pairing request after connection
    uint8_t mitm = FALSE; // Do not use authenticated pairing
    uint8_t ioCap = GAPBOND_IO_CAP_NO_INPUT_NO_OUTPUT; // device is not capable for interactive authentication.
    uint8_t bonding = TRUE;

    GAPBondMgr_SetParameter(GAPBOND_DEFAULT_PASSCODE, sizeof(uint32_t),
                            &passkey);
    GAPBondMgr_SetParameter(GAPBOND_PAIRING_MODE, sizeof(uint8_t), &pairMode);
    GAPBondMgr_SetParameter(GAPBOND_MITM_PROTECTION, sizeof(uint8_t), &mitm);
    GAPBondMgr_SetParameter(GAPBOND_IO_CAPABILITIES, sizeof(uint8_t), &ioCap);
    GAPBondMgr_SetParameter(GAPBOND_BONDING_ENABLED, sizeof(uint8_t), &bonding);
  }

  {
      uint8_t batteryCriticalLevel = 50; // in precentage
      Batt_SetParameter(BATT_PARAM_CRITICAL_LEVEL, sizeof(uint8_t), &batteryCriticalLevel);
  }

  /** Hid Dev **/
  Log_info0("Registering Hid Device");
  HidDev_Register(&Clicker_hidDevCfg, &Clicker_CBs);

  HidKbd_AddService();

  Log_info0("Starting HID Device");
  HidDev_StartDevice();

  /** Setup button clicking **/
  Log_info0("Setting up key press handling");
  Board_initKeys(Clicker_buttonPressedCB);

  Log_info0("Clicker_init finished");
}

/*********************************************************************
 * @fn      Clicker_taskFxn
 *
 * @brief   Application task entry point for the Clicker app.
 *
 * @param   a0, a1 - not used.
 *
 * @return  None.
 */
static void Clicker_taskFxn(UArg a0, UArg a1)
{
  Log_info0("Starting taskFxn");
  // Initialize application
  Clicker_init();

  // Application main loop
  for (;;)
  {
    // Waits for a signal to the semaphore associated with the calling thread.
    // Note that the semaphore associated with a thread is signaled when a
    // message is queued to the message receive queue of the thread or when
    // ICall_signal() function is called onto the semaphore.
    Log_info0("CLKR is waiting for messages");
    ICall_Errno errno = ICall_wait(ICALL_TIMEOUT_FOREVER);
    Log_info0("A message was queued for Clicker task");

    if (errno == ICALL_ERRNO_SUCCESS)
    {
      ICall_EntityID dest;
      ICall_ServiceEnum src;
      ICall_HciExtEvt *pMsg = NULL;

      if (ICall_fetchServiceMsg(&src, &dest,
                                (void **)&pMsg) == ICALL_ERRNO_SUCCESS)
      {
        uint8 safeToDealloc = TRUE;

        if ((src == ICALL_SERVICE_CLASS_BLE) && (dest == selfEntity))
        {
          // Process inter-task message
          safeToDealloc = Clicker_processStackMsg((ICall_Hdr *)pMsg);
        }

        if (pMsg && safeToDealloc)
        {
          ICall_freeMsg(pMsg);
        }
      }

      // If RTOS queue is not empty, process app message.
      while (!Queue_empty(appMsgQueue))
      {
        clickerEvt_t *pMsg = (clickerEvt_t *)Util_dequeueMsg(appMsgQueue);
        if (pMsg)
        {
          // Process message.
          Clicker_processAppMsg(pMsg);

          // Free the space from the message.
          ICall_free(pMsg);
        }
      }
    }
  }
}

/*********************************************************************
 * @fn      Clicker_processStackMsg
 *
 * @brief   Process an incoming stack message.
 *
 * @param   pMsg - message to process
 *
 * @return  TRUE if safe to deallocate incoming message, FALSE otherwise.
 */
static uint8_t Clicker_processStackMsg(ICall_Hdr *pMsg)
{
  uint8_t safeToDealloc = TRUE;

  switch (pMsg->event)
  {
    case GATT_MSG_EVENT:
      // Process GATT message
      safeToDealloc = Clicker_processGATTMsg((gattMsgEvent_t *)pMsg);
      break;

    default:
      // do nothing
      break;
  }

  return (safeToDealloc);
}

/*********************************************************************
 * @fn      Clicker_processGATTMsg
 *
 * @brief   Process GATT messages and events.
 *
 * @return  TRUE if safe to deallocate incoming message, FALSE otherwise.
 */
static uint8_t Clicker_processGATTMsg(gattMsgEvent_t *pMsg)
{
  Log_info0("Clicker_processGATTMsg() was called.");

  // Free message payload. Needed only for ATT Protocol messages
  GATT_bm_free(&pMsg->msg, pMsg->method);

  // It's safe to free the incoming message
  return (TRUE);
}

/*********************************************************************
 * @fn      Clicker_processAppMsg
 *
 * @brief   Process an incoming callback from a profile.
 *
 * @param   pMsg - message to process
 *
 * @return  None.
 */
static void Clicker_processAppMsg(clickerEvt_t *pMsg)
{
  switch (pMsg->hdr.event)
  {
  case CLICKER_CLK_EVT:
      Clicker_processClkEvt(pMsg->hdr.state);
  default:
      // Do nothing.
      break;
  }
}

/*********************************************************************
 * @fn      Clicker_enqueueMsg
 *
 * @brief   Creates a message and puts the message in RTOS queue.
 *
 * @param   event - message event.
 * @param   state - message state.
 *
 * @return  None.
 */
static void Clicker_enqueueMsg(uint8_t event, uint8_t state)
{
  clickerEvt_t *pMsg;

  // Create dynamic pointer to message.
  if ((pMsg = ICall_malloc(sizeof(clickerEvt_t))))
  {
    pMsg->hdr.event = event;
    pMsg->hdr.state = state;

    // Enqueue the message.
    Util_enqueueMsg(appMsgQueue, sem, (uint8*)pMsg);
  }
}

/*********************************************************************
*********************************************************************/

/*
 * @brief   Extract the LOCALNAME from Scan/AdvData
 *
 * @param   data - Pointer to the advertisement or scan response data
 *
 * @return  Pointer to null-terminated string with the adv local name.
 */
static char *Util_getLocalNameStr(const uint8_t *data) {
  uint8_t nuggetLen = 0;
  uint8_t nuggetType = 0;
  uint8_t advIdx = 0;

  static char localNameStr[32] = { 0 };
  memset(localNameStr, 0, sizeof(localNameStr));

  for (advIdx = 0; advIdx < 32;) {
    nuggetLen = data[advIdx++];
    nuggetType = data[advIdx];
    if ( (nuggetType == GAP_ADTYPE_LOCAL_NAME_COMPLETE ||
          nuggetType == GAP_ADTYPE_LOCAL_NAME_SHORT) && nuggetLen < 31) {
      memcpy(localNameStr, &data[advIdx + 1], nuggetLen - 1);
      break;
    } else {
      advIdx += nuggetLen;
    }
  }

  return localNameStr;
}
