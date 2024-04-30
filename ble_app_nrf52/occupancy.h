/**
 * @file occupancy.h
 * @brief Header file for the occupancy sensor module.
 *
 * This header file defines the interface for the occupancy sensor module, providing declarations for
 * functions and data types related to occupancy sensing. It is intended for use with the Novelda
 * chip interface and FreeRTOS-based applications.
 */

#ifndef OCCUPANCY_H_
#define OCCUPANCY_H_
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif

#define DEFAULT_SENSITIVITY     3     // default sensitivity
#define MIN_SENSITIVITY_VALUE 1
#define MAX_SENSITIVITY_VALUE 6
#define MIN_RANGE_VALUE 70
#define MAX_RANGE_VALUE 500
#define DEFAULT_RANGE 300   // default range 300cm


#define PRESENCE_TIME_OUT_MS    10000 // 10s timeout to keep presence

typedef void (*updateSensorValueCb_t)( uint8_t newValue );


extern void occupancyInit(void* taskSem, updateSensorValueCb_t updateSensCb);
extern void startSensor();
extern void stopSensor();
extern void setSensitivity(uint8_t sens);
extern void setRange(uint16_t range);
extern uint8_t getSensitivity(void);
extern uint16_t getRange(void);
extern uint16_t getTimeout(void);
extern uint16_t changeRange(void);
extern void setPresenceTimeout(uint16_t tmout);
extern uint8_t getSensorValue();

extern void processSensorEvent();

#ifdef __cplusplus
}
#endif

#endif /* OCCUPANCY_H_ */
