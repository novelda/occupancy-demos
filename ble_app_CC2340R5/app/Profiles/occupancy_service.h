

#ifndef OCCUPANCYPROFILE_H
#define OCCUPANCYPROFILE_H

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
// Profile Parameters
#define OCCUPANCYPROFILE_DETECTION                   0  // R uint8 - Profile Characteristic occupancy value
#define OCCUPANCYPROFILE_RANGE                       1  // RW uint16 - Profile Characteristic range value
#define OCCUPANCYPROFILE_SENSITIVITY                 2  // RW uint8 - Profile Characteristic sensitivity value
#define OCCUPANCYPROFILE_TIMEOUT                     3  // RW uint16 - Profile Characteristic timeout value

// Simple Profile Service UUID
#define OCCUPANCYPROFILE_SERV_UUID               0x20F1

// Key Pressed UUID
#define OCCUPANCYPROFILE_DETECTION_UUID            0x2BAD
#define OCCUPANCYPROFILE_RANGE_UUID                0x2BB1
#define OCCUPANCYPROFILE_SENSITIVITY_UUID          0x2BB2
#define OCCUPANCYPROFILE_TIMEOUT_UUID              0x2BB3


/*********************************************************************
 * Profile Callbacks
 */
// Callback when a characteristic value has changed
typedef void (*pfnOccupancyProfile_Change_t)( uint8 paramID );

typedef struct
{
  pfnOccupancyProfile_Change_t        pfnOccupancyProfile_Change;  // Called when characteristic value changes
} OccupancyProfile_CBs_t;

/*********************************************************************
 * API FUNCTIONS
 */

/*********************************************************************
 * @fn      OccupancyProfile_addService
 *
 * @brief   This function initializes the Simple GATT Server service
 *          by registering GATT attributes with the GATT server.
 *
 * @return  SUCCESS or stack call status
 */
bStatus_t OccupancyProfile_addService( void );

/**
 * @fn      OccupancyProfile_registerAppCBs
 *
 * @brief   Registers the application callback function. Only call
 *          this function once.
 *
 * @param   appCallbacks - pointer to application callback.
 *
 * @return  SUCCESS or INVALIDPARAMETER
 */
bStatus_t OccupancyProfile_registerAppCBs( OccupancyProfile_CBs_t *appCallbacks );

/**
 * @fn      OccupancyProfile_setParameter
 *
 * @brief   Set a Simple GATT Profile parameter.
 *
 * @param   param - Profile parameter ID
 * @param   len - length of data to right
 * @param   value - pointer to data to write.  This is dependent on
 *                  the parameter ID and WILL be cast to the appropriate
 *                  data type (example: data type of uint16 will be cast to
 *                  uint16 pointer).
 *
 * @return  SUCCESS or INVALIDPARAMETER
 */
bStatus_t OccupancyProfile_setParameter( uint8 param, uint8 len, void *value );

/**
 * @fn      OccupancyProfile_getParameter
 *
 * @brief   Get a Simple GATT Profile parameter.
 *
 * @param   param - Profile parameter ID
 * @param   value - pointer to data to write. This is dependent on
 *                  the parameter ID and WILL be cast to the appropriate
 *                  data type (example: data type of uint16 will be cast to
 *                  uint16 pointer).
 *
 * @return  SUCCESS or INVALIDPARAMETER
 */
bStatus_t OccupancyProfile_getParameter( uint8 param, void *value );


/**
 * @brief Handler for the Connect event.
 *
 * This function handles the Connect event by starting the sensor.
 */
extern void OccupancyProfile_on_connect(void);

/**
 * @brief Handler for the Disconnect event.
 *
 * This function handles the Disconnect event by stopping the sensor if no active connections are present.
 */
extern void OccupancyProfile_on_disconnect(void);

/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* OCCUPANCYPROFILE_H */
