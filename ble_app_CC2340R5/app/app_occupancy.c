/**
 * @file app_occupancy.c
 * @brief Application code for the Occupancy Profile service.
 *
 * This file contains the implementation of the application code using the Occupancy Profile service.
 */

//*****************************************************************************
//! Includes
//*****************************************************************************
#include <string.h>
#include <ti/bleapp/ble_app_util/inc/bleapputil_api.h>
#include <ti/drivers/GPIO.h>
#include <ti/drivers/I2C.h>
#include <app_main.h>
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "semphr.h"
#include "occupancy.h"
#include "occupancy_service.h"
#include <ti/display/Display.h>


//*****************************************************************************
//! Globals
//*****************************************************************************

static void Occupancy_changeCB( uint8_t paramId );
extern void sensor_run_thread(void * pvParameter);
void Occupancy_on_occupancy_evt(uint8_t occupancy);
static void detection_task(void * pvParameter);
static TaskHandle_t m_occupancy_task;
static TaskHandle_t m_sensor_task;
SemaphoreHandle_t appSemHandle;

// Simple GATT Profile Callbacks
static OccupancyProfile_CBs_t occupancy_profileCBs =
{
  Occupancy_changeCB // Simple GATT Characteristic value change callback
};

//*****************************************************************************
//! Functions
//*****************************************************************************

/*********************************************************************
 * @fn      Occupancy_ChangeCB
 *
 * @brief   Callback from Simple Profile indicating a characteristic
 *          value change.
 *
 * @param   paramId - parameter Id of the value that was changed.
 *
 * @return  None.
 */
static void Occupancy_changeCB( uint8_t paramId )
{
  uint16_t newValue = 0;
  OccupancyProfile_getParameter(paramId, &newValue);

  switch( paramId )
  {
    case OCCUPANCYPROFILE_RANGE:
      {
        if(newValue >= MIN_RANGE_VALUE && newValue <= MAX_RANGE_VALUE)
        {
            Display_printf(handle, 0, 0, "Range value = %d", newValue);
            stopSensor();
            setRange(newValue);
            startSensor();
        }
        else
        {
            uint16_t charRange = getRange();
            Display_printf(handle, 0, 0, "New Range value %d is out of rage. Allowed value range[%d, %d]", newValue, MIN_RANGE_VALUE, MAX_RANGE_VALUE);
            OccupancyProfile_setParameter( OCCUPANCYPROFILE_RANGE, sizeof(uint16_t),
                                                &charRange );
        }
      }
      break;

    case OCCUPANCYPROFILE_SENSITIVITY:
      {
          if(newValue >= MIN_SENSITIVITY_VALUE && newValue <= MAX_SENSITIVITY_VALUE)
          {
              Display_printf(handle, 0, 0, "Sensitivity value = %d", newValue);
            stopSensor();
            setSensitivity(newValue);
            startSensor();
          }
          else
          {
              uint8_t charSensitivity = getSensitivity();
              Display_printf(handle, 0, 0, "New Sensitivity value %d is out of rage.Allowed value range[%d, %d]", newValue, MIN_SENSITIVITY_VALUE, MAX_SENSITIVITY_VALUE);
              OccupancyProfile_setParameter( OCCUPANCYPROFILE_SENSITIVITY, sizeof(uint8_t),
                                                  &charSensitivity );
          }
      }
      break;
    case OCCUPANCYPROFILE_TIMEOUT:
      {
          Display_printf(handle, 0, 0, "Timeout value = %d", newValue);
        setPresenceTimeout(newValue);
          break;
      }
    default:
      // should not reach here!
      break;
  }
}

/*********************************************************************
 * @fn      Occupancy_start
 *
 * @brief   This function is called after stack initialization,
 *          the purpose of this function is to initialize and
 *          register the Simple GATT profile.
 *
 * @return  SUCCESS or stack call status
 */
bStatus_t Occupancy_start( void )
{
  bStatus_t status = SUCCESS;

  //Initialize I2C being used by The sensor
  I2C_init();

  // Add Simple GATT service
  status = OccupancyProfile_addService();
  if(status != SUCCESS)
  {
	// Return status value
    return(status);
  }

    uint8_t charOccupancy = 0;
    uint16_t charRange = getRange();
    uint8_t charSensitivity = getSensitivity();
    uint16_t charTimeout = getTimeout();

    OccupancyProfile_setParameter( OCCUPANCYPROFILE_DETECTION, sizeof(uint8_t),
                                    &charOccupancy );
    OccupancyProfile_setParameter( OCCUPANCYPROFILE_RANGE, sizeof(uint16_t),
                                    &charRange );
    OccupancyProfile_setParameter( OCCUPANCYPROFILE_SENSITIVITY, sizeof(uint8_t),
                                    &charSensitivity );
    OccupancyProfile_setParameter( OCCUPANCYPROFILE_TIMEOUT, sizeof(uint16_t),
                                    &charTimeout );
  // Register callback with SimpleGATTprofile
  status = OccupancyProfile_registerAppCBs( &occupancy_profileCBs );
  appSemHandle = xSemaphoreCreateBinary();
  occupancyInit(appSemHandle, Occupancy_on_occupancy_evt);

  if (pdPASS != xTaskCreate(detection_task, "DET", 256, NULL, 1, &m_occupancy_task))
  {
      status = FAILURE;
  }
  if (pdPASS != xTaskCreate(sensor_run_thread, "SEN", 356, NULL, 2, &m_sensor_task))
  {
      status = FAILURE;
  }

  GPIO_setConfig(CONFIG_GPIO_LEDR, GPIO_CFG_OUTPUT | GPIO_CFG_OUT_LOW);
  GPIO_setConfig(CONFIG_GPIO_LEDG, GPIO_CFG_OUTPUT | GPIO_CFG_OUT_LOW);



  // Return status value
  return(status);
}

// Entity ID globally used to check for source and/or destination of messages
static ICall_EntityID selfEntity;

// Event globally used to post local events and pend on system and
// local events.
static ICall_SyncHandle syncEvent;

/**
 * @brief Task handling detection events.
 *
 * This function is a task handling detection events in the background.
 *
 * @param pvParameter Task parameters (not used).
 */
static void detection_task(void * pvParameter)
{
    ICall_registerApp(&selfEntity, &syncEvent);
    for(;;)
    {
        if(pdTRUE == xSemaphoreTake(appSemHandle, portMAX_DELAY))
        {
            processSensorEvent();
        }

    }
}

/**
 * @brief Event handler for BLE occupancy service on occupancy event.
 *
 * This function is an event handler for the BLE occupancy service on occupancy event.
 *
 * @param detection Detection value.
 */
void Occupancy_on_occupancy_evt(uint8_t detection)
{
    static uint8_t last_detection;
    if(last_detection != detection)
    {
        OccupancyProfile_setParameter(OCCUPANCYPROFILE_DETECTION, sizeof(uint8_t), &detection);
        Display_printf(handle, 0, 0, "Detection value = %d", detection);
        last_detection = detection;
        GPIO_write(CONFIG_GPIO_LEDG, detection);
        
    }
}

