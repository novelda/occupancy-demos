/**
 * @file novelda_sensor.c
 * @brief Implementation of the Novelda X4 occupancy sensor module.
 *
 * This module provides the implementation of the Novelda X4 occupancy sensor interface, including
 * initialization, sensor control, and event processing. It interacts with the Novelda chip
 * interface and FreeRTOS-based applications.
 */


#include <nrf_gpio.h>
#include <nrf_drv_gpiote.h>
#include <FreeRTOS.h>
#include <semphr.h>
#include <timers.h>
#include <task.h>
#include "nrf_log.h"
#include <unistd.h>
#include <stdint.h>
#include <stddef.h>
#include <novelda_chipinterface.h>
#include <x4sensor_configuration_blob.h>
#include "novelda_sensor.h"
#include "nrf_log.h"

/* GPIO pin for X4 sensor IRQ */
#define CONFIG_GPIO_X4_IRQ_0 NRF_GPIO_PIN_MAP(1,10)

static SemaphoreHandle_t sensorSemHandle;
volatile uint32_t gSensor_Events;
volatile presence_callback gPresence_cb;
uint8_t gSensitivity;
uint16_t gRange;
bool gRunning = false;

extern void gpio_irq_callback(nrf_drv_gpiote_pin_t index, nrf_gpiote_polarity_t action);

/**
 * @brief Initialize the occupancy sensor module.
 *
 * This function initializes the occupancy sensor module, creating semaphores for messages and events.
 */
void sensor_init(void)
{    
    gSensor_Events |= EVENT_SENSOR_INIT;
    xSemaphoreGive(sensorSemHandle);
}

/**
 * @brief Start the occupancy sensor remotely.
 *
 * This function configures the sensor with sensitivity, range, and a callback for presence detection.
 *
 * @param[in] sensitivity Sensitivity level to set.
 * @param[in] range Range value to set.
 * @param[in] callback Callback function for presence detection.
 */
void sensor_run_remote(uint8_t sensitivity, uint16_t range, presence_callback callback)
{
    gSensitivity = sensitivity;
    gRange = range;
    gPresence_cb = callback;
    gSensor_Events |= EVENT_SENSOR_START;
    xSemaphoreGive(sensorSemHandle);
}


/**
 * @brief Stop the occupancy sensor remotely.
 *
 * This function stops the occupancy sensor, switches the interrupt back to rising edge, and uninitializes
 * the GPIO input.
 */
void sensor_stop_remote(void)
{
    gRunning = false;
    // Switch interrupt back to rising edge to be able to restart the sensor
    nrf_drv_gpiote_in_uninit(CONFIG_GPIO_X4_IRQ_0);
    
    nrf_drv_gpiote_in_config_t in_config = GPIOTE_CONFIG_IN_SENSE_LOTOHI(true);
    in_config.pull = NRF_GPIO_PIN_PULLUP;

    nrf_drv_gpiote_in_init(CONFIG_GPIO_X4_IRQ_0, &in_config, gpio_irq_callback);
    nrf_drv_gpiote_in_event_enable(CONFIG_GPIO_X4_IRQ_0, true);
    x4sensor_stop();
}


/**
 * @brief Sensor running thread.
 *
 * This thread processes sensor events and presence detection while the sensor is active.
 *
 * @param[in] pvParameter Unused thread parameter.
 */
void sensor_run_thread(void * pvParameter)
{
    const x4sensor_info_t *sensor_info;
    /* create semaphores for messages / events */
   sensorSemHandle = xSemaphoreCreateBinary();
   

    while (1)
    {
        if(gRunning)
        {
            //pend on irq semaphore
            if(chipinterface_wait_for_interrupt(portMAX_DELAY) == CHIPINTERFACE_SUCCESS)
            {
                //irq happened report back to application that there is presence
                if(gPresence_cb)
                {
                    gPresence_cb();
                }
            }
        }
        else
        {
            xSemaphoreTake(sensorSemHandle, portMAX_DELAY);
            if(gSensor_Events & EVENT_SENSOR_INIT)
            {
                
                MAIN_ASSERT(x4sensor_initialize_i2c(x4sensor_configuration_blob, x4sensor_configuration_blob_size),
                                                               X4SENSOR_SUCCESS);

                sensor_info = x4sensor_get_info();
                NRF_LOG_INFO("*** Novelda Sensor ID: 0x%X Chip Version: %d ***\n", sensor_info->sample_id, sensor_info->chip_revision);
                

                gSensor_Events ^= EVENT_SENSOR_INIT;

            }
            if(gSensor_Events & EVENT_SENSOR_START)
            {
                NRF_LOG_INFO("Starting normal operation mode. Range: %u cm, sensitivity level: %u\n",
                        gRange, gSensitivity);
                x4sensor_set_range_cm(gRange);
                x4sensor_set_sensitivity_level(gSensitivity);
                x4sensor_start_normal_mode();

                // Switch interrupt to both edges so we don't have to poll the irq line
                // this will trigger an interrupt once the irq line goes down which will restart the occupancy timer
                nrf_drv_gpiote_in_uninit(CONFIG_GPIO_X4_IRQ_0);
    
                nrf_drv_gpiote_in_config_t in_config = GPIOTE_CONFIG_IN_SENSE_TOGGLE(true);
                in_config.pull = NRF_GPIO_PIN_PULLUP;

                nrf_drv_gpiote_in_init(CONFIG_GPIO_X4_IRQ_0, &in_config, gpio_irq_callback);
                nrf_drv_gpiote_in_event_enable(CONFIG_GPIO_X4_IRQ_0, true);

                gRunning = true;
                gSensor_Events ^= EVENT_SENSOR_START;

            }

        }
    }
}
