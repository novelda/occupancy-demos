/**
 * @file occupancy_service.c
 * @brief BLE profile interacting with higher level APIs for the Novelda X4F103 sensor.
 *
 * This file contains the implementation of a BLE profile responsible for configuring
 * the Novelda X4F103 sensor and relaying information about presence detection.
 */

#include <string.h>
#include <icall.h>
#include "icall_ble_api.h"
#include "occupancy.h"
#include "occupancy_service.h"
#include <ti/bleapp/ble_app_util/inc/bleapputil_api.h>

void OccupancyProfile_callback(uint8_t paramID);
void OccupancyProfile_invokeFromFWContext(char *pData);

/*********************************************************************
 * GLOBAL VARIABLES
 */
// Occupancy Profile Service UUID: 0xFFF0
GATT_BT_UUID(occupancyProfile_ServUUID, OCCUPANCYPROFILE_SERV_UUID);

// Characteristic UUIDs
GATT_BT_UUID(occupancyProfile_DetectionUUID, OCCUPANCYPROFILE_DETECTION_UUID);
GATT_BT_UUID(occupancyProfile_RangeUUID, OCCUPANCYPROFILE_RANGE_UUID);
GATT_BT_UUID(occupancyProfile_SensitivityUUID, OCCUPANCYPROFILE_SENSITIVITY_UUID);
GATT_BT_UUID(occupancyProfile_TimeoutUUID, OCCUPANCYPROFILE_TIMEOUT_UUID);


/*********************************************************************
 * LOCAL VARIABLES
 */
static OccupancyProfile_CBs_t *occupancyProfile_appCBs = NULL;

/*********************************************************************
 * Profile Attributes - variables
 */

// Occupancy Profile Service attribute
static CONST gattAttrType_t occupancyProfile_Service = { ATT_BT_UUID_SIZE, occupancyProfile_ServUUID };

// Characteristic Properties
static uint8_t occupancyProfile_DetectionProps = GATT_PROP_NOTIFY | GATT_PROP_READ;
static uint8_t occupancyProfile_RangeProps = GATT_PROP_READ | GATT_PROP_WRITE | GATT_PROP_WRITE_NO_RSP;
static uint8_t occupancyProfile_SensitivityProps = GATT_PROP_READ | GATT_PROP_WRITE | GATT_PROP_WRITE_NO_RSP;
static uint8_t occupancyProfile_TimeoutProps = GATT_PROP_READ | GATT_PROP_WRITE | GATT_PROP_WRITE_NO_RSP;

static gattCharCfg_t *occupancyProfile_DetectionConfig;

// Characteristic Values
static uint8_t occupancyProfile_Detection = 0;
static uint16_t occupancyProfile_Range = 0;
static uint8_t occupancyProfile_Sensitivity = 0;
static uint16_t occupancyProfile_Timeout = 0;

// Characteristic User Descriptions
static uint8_t occupancyProfile_DetectionUserDesp[] = "Detection";
static uint8_t occupancyProfile_RangeUserDesp[] = "Range";
static uint8_t occupancyProfile_SensitivityUserDesp[] = "Sensitivity";
static uint8_t occupancyProfile_TimeoutUserDesp[] = "Timeout";

/*********************************************************************
 * Profile Attributes - Table
 */
static gattAttribute_t occupancyProfile_attrTbl[] =
{
    // Occupancy Profile Service
    GATT_BT_ATT(primaryServiceUUID, GATT_PERMIT_READ, (uint8_t *)&occupancyProfile_Service),

    // Occupancy Characteristic Declaration
    GATT_BT_ATT(characterUUID, GATT_PERMIT_READ, &occupancyProfile_DetectionProps),
    // Occupancy Characteristic Value
    GATT_BT_ATT(occupancyProfile_DetectionUUID, GATT_PERMIT_READ, &occupancyProfile_Detection),
    GATT_BT_ATT( clientCharCfgUUID,            GATT_PERMIT_READ | GATT_PERMIT_WRITE,  (uint8_t *) &occupancyProfile_DetectionConfig ),
    // Occupancy Characteristic User Description
    GATT_BT_ATT(charUserDescUUID, GATT_PERMIT_READ, occupancyProfile_DetectionUserDesp),

    // Range Characteristic Declaration
    GATT_BT_ATT(characterUUID, GATT_PERMIT_READ, &occupancyProfile_RangeProps),
    // Range Characteristic Value
    GATT_BT_ATT(occupancyProfile_RangeUUID, GATT_PERMIT_READ | GATT_PERMIT_WRITE , (uint8_t *)&occupancyProfile_Range),
    // Range Characteristic User Description
    GATT_BT_ATT(charUserDescUUID, GATT_PERMIT_READ, occupancyProfile_RangeUserDesp),

    // Sensitivity Characteristic Declaration
    GATT_BT_ATT(characterUUID, GATT_PERMIT_READ, &occupancyProfile_SensitivityProps),
    // Sensitivity Characteristic Value
    GATT_BT_ATT(occupancyProfile_SensitivityUUID, GATT_PERMIT_READ | GATT_PERMIT_WRITE , &occupancyProfile_Sensitivity),
    // Sensitivity Characteristic User Description
    GATT_BT_ATT(charUserDescUUID, GATT_PERMIT_READ, occupancyProfile_SensitivityUserDesp),

    // Timeout Characteristic Declaration
    GATT_BT_ATT(characterUUID, GATT_PERMIT_READ, &occupancyProfile_TimeoutProps),
    // Timeout Characteristic Value
    GATT_BT_ATT(occupancyProfile_TimeoutUUID, GATT_PERMIT_READ | GATT_PERMIT_WRITE , (uint8_t *)&occupancyProfile_Timeout),
    // Timeout Characteristic User Description
    GATT_BT_ATT(charUserDescUUID, GATT_PERMIT_READ, occupancyProfile_TimeoutUserDesp),
};

/*********************************************************************
 * LOCAL FUNCTIONS
 */
bStatus_t OccupancyProfile_readAttrCB(uint16_t connHandle,
                                      gattAttribute_t *pAttr,
                                      uint8_t *pValue, uint16_t *pLen,
                                      uint16_t offset, uint16_t maxLen,
                                      uint8_t method);
bStatus_t OccupancyProfile_writeAttrCB(uint16_t connHandle,
                                       gattAttribute_t *pAttr,
                                       uint8_t *pValue, uint16_t len,
                                       uint16_t offset, uint8_t method);

/*********************************************************************
 * PROFILE CALLBACKS
 */

// Occupancy Profile Service Callbacks
CONST gattServiceCBs_t occupancyProfile_CBs =
{
    OccupancyProfile_readAttrCB,  // Read callback function pointer
    OccupancyProfile_writeAttrCB, // Write callback function pointer
    NULL                          // Authorization callback function pointer
};

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/**
 * @brief Adds the Occupancy Profile service.
 *
 * This function adds the Occupancy Profile service to the GATT server.
 *
 * @return Status of the operation (SUCCESS or an error code).
 */
bStatus_t OccupancyProfile_addService(void)
{
    uint8_t status = SUCCESS;

    occupancyProfile_DetectionConfig = (gattCharCfg_t *)ICall_malloc( sizeof( gattCharCfg_t ) *
                                                                   MAX_NUM_BLE_CONNS );
    // Initialize Client Characteristic Configuration attributes
      GATTServApp_InitCharCfg( LINKDB_CONNHANDLE_INVALID, occupancyProfile_DetectionConfig );

    // Register GATT attribute list and CBs with GATT Server App
    status = GATTServApp_RegisterService(occupancyProfile_attrTbl,
                                        GATT_NUM_ATTRS(occupancyProfile_attrTbl),
                                        GATT_MAX_ENCRYPT_KEY_SIZE,
                                        &occupancyProfile_CBs);

    // Return status value
    return (status);
}

/**
 * @brief Registers application callbacks for the Occupancy Profile service.
 *
 * @param appCallbacks Callback functions provided by the application.
 * @return Status of the operation (SUCCESS or an error code).
 */
bStatus_t OccupancyProfile_registerAppCBs(OccupancyProfile_CBs_t *appCallbacks)
{
    if (appCallbacks)
    {
        occupancyProfile_appCBs = appCallbacks;
        return (SUCCESS);
    }
    else
    {
        return (bleAlreadyInRequestedMode);
    }
}


/**
 * @brief Sets a parameter for the Occupancy Profile service.
 *
 * @param param Parameter to set.
 * @param len Length of the value.
 * @param value Pointer to the value to set.
 * @return Status of the operation (SUCCESS or an error code).
 */
bStatus_t OccupancyProfile_setParameter(uint8_t param, uint8_t len, void *value)
{
    bStatus_t status = SUCCESS;

    switch (param)
    {
        case OCCUPANCYPROFILE_DETECTION:
            if (len == sizeof(uint8_t))
            {
                occupancyProfile_Detection = *((uint8_t *)value);
                // See if Notification has been enabled
                GATTServApp_ProcessCharCfg( occupancyProfile_DetectionConfig, &occupancyProfile_Detection, FALSE,
                                            occupancyProfile_attrTbl, GATT_NUM_ATTRS( occupancyProfile_attrTbl ),
                                            INVALID_TASK_ID, OccupancyProfile_readAttrCB );
            }
            else
            {
                status = bleInvalidRange;
            }
            break;

        case OCCUPANCYPROFILE_RANGE:
            if (len == sizeof(uint16_t))
            {
                occupancyProfile_Range = *((uint16_t *)value);
            }
            else
            {
                status = bleInvalidRange;
            }
            break;

        case OCCUPANCYPROFILE_SENSITIVITY:
            if (len == sizeof(uint8_t))
            {
                occupancyProfile_Sensitivity = *((uint8_t *)value);
            }
            else
            {
                status = bleInvalidRange;
            }
            break;

        case OCCUPANCYPROFILE_TIMEOUT:
            if (len == sizeof(uint16_t))
            {
                occupancyProfile_Timeout = *((uint16_t *)value);
            }
            else
            {
                status = bleInvalidRange;
            }
            break;

        default:
            status = INVALIDPARAMETER;
            break;
    }

    // Return status value
    return (status);
}

/**
 * @brief Gets a parameter from the Occupancy Profile service.
 *
 * @param param Parameter to get.
 * @param value Pointer to store the retrieved value.
 * @return Status of the operation (SUCCESS or an error code).
 */
bStatus_t OccupancyProfile_getParameter(uint8_t param, void *value)
{
    bStatus_t status = SUCCESS;

    switch (param)
    {
        case OCCUPANCYPROFILE_DETECTION:
            *((uint8_t *)value) = occupancyProfile_Detection;
            break;

        case OCCUPANCYPROFILE_RANGE:
            *((uint16_t *)value) = occupancyProfile_Range;
            break;

        case OCCUPANCYPROFILE_SENSITIVITY:
            *((uint8_t *)value) = occupancyProfile_Sensitivity;
            break;

        case OCCUPANCYPROFILE_TIMEOUT:
            *((uint16_t *)value) = occupancyProfile_Timeout;
            break;

        default:
            status = INVALIDPARAMETER;
            break;
    }

    // Return status value
    return (status);
}

/**
 * @brief Callback for reading attributes of the Occupancy Profile service.
 *
 * This function is a callback for reading attributes of the Occupancy Profile service.
 *
 * @param connHandle Connection handle.
 * @param pAttr Pointer to the attribute.
 * @param pValue Pointer to store the attribute value.
 * @param pLen Pointer to store the attribute length.
 * @param offset Offset for attribute reading.
 * @param maxLen Maximum length of the attribute.
 * @param method Method of reading the attribute.
 * @return Status of the operation (SUCCESS or an error code).
 */
bStatus_t OccupancyProfile_readAttrCB(uint16_t connHandle,
                                      gattAttribute_t *pAttr,
                                      uint8_t *pValue, uint16_t *pLen,
                                      uint16_t offset, uint16_t maxLen,
                                      uint8_t method)
{
    bStatus_t status = SUCCESS;

    // Make sure it's not a blob operation (no attributes in the profile are long)
    if (offset > 0)
    {
        return (ATT_ERR_ATTR_NOT_LONG);
    }

    if (pAttr->type.len == ATT_BT_UUID_SIZE)
    {
        // 16-bit UUID
        uint16 uuid = BUILD_UINT16(pAttr->type.uuid[0], pAttr->type.uuid[1]);
        switch (uuid)
        {
            case OCCUPANCYPROFILE_DETECTION_UUID:
            case OCCUPANCYPROFILE_SENSITIVITY_UUID:
                *pLen = 1;
                pValue[0] = *pAttr->pValue;
                break;

            case OCCUPANCYPROFILE_RANGE_UUID:
            case OCCUPANCYPROFILE_TIMEOUT_UUID:
                *pLen = sizeof(uint16_t);
                VOID memcpy(pValue, pAttr->pValue, sizeof(uint16_t));
                break;

            default:
                *pLen = 0;
                status = ATT_ERR_ATTR_NOT_FOUND;
                break;
        }
    }
    else
    {
        // 128-bit UUID
        *pLen = 0;
        status = ATT_ERR_INVALID_HANDLE;
    }

    // Return status value
    return (status);
}

/**
 * @brief Callback for writing attributes of the Occupancy Profile service.
 *
 * This function is a callback for writing attributes of the Occupancy Profile service.
 *
 * @param connHandle Connection handle.
 * @param pAttr Pointer to the attribute.
 * @param pValue Pointer to the value to be written.
 * @param len Length of the value.
 * @param offset Offset for attribute writing.
 * @param method Method of writing the attribute.
 * @return Status of the operation (SUCCESS or an error code).
 */
bStatus_t OccupancyProfile_writeAttrCB(uint16_t connHandle,
                                       gattAttribute_t *pAttr,
                                       uint8_t *pValue, uint16_t len,
                                       uint16_t offset, uint8_t method)
{
    bStatus_t status = SUCCESS;
    uint8_t notifyApp = 0xFF;

    if (pAttr->type.len == ATT_BT_UUID_SIZE)
    {
        // 16-bit UUID
        uint16 uuid = BUILD_UINT16(pAttr->type.uuid[0], pAttr->type.uuid[1]);
        switch (uuid)
        {
            case OCCUPANCYPROFILE_RANGE_UUID:
            case OCCUPANCYPROFILE_SENSITIVITY_UUID:
            case OCCUPANCYPROFILE_TIMEOUT_UUID:
                // Validate the value
                // Make sure it's not a blob oper
                if (offset == 0)
                {
                    if (len != 1 && len != sizeof(uint16_t))
                    {
                        status = ATT_ERR_INVALID_VALUE_SIZE;
                    }
                }
                else
                {
                    status = ATT_ERR_ATTR_NOT_LONG;
                }
                //Reset back to 0 the previous value of the attr 
                // for the case where the previous value was len=2 and the new value is length 1
                if(uuid != OCCUPANCYPROFILE_SENSITIVITY_UUID)
                {
                    pAttr->pValue[0] = 0;
                    pAttr->pValue[1] = 0;
                }
                // this takes care of the case where a longer than len == 1 value tries to be written into sensitivity attr
                else if(len == 2)
                {
                    status = ATT_ERR_INVALID_VALUE_SIZE;
                }
                // Write the value
                if (status == SUCCESS)
                {
                    if (len == 1)
                    {
                        uint8_t *pCurValue = (uint8_t *)pAttr->pValue;
                        *pCurValue = pValue[0];
                    }
                    else
                    {
                        uint16_t *pCurValue = (uint16_t *)pAttr->pValue;
                        *pCurValue = BUILD_UINT16(pValue[0], pValue[1]);
                    }
                }
                break;

            case GATT_CLIENT_CHAR_CFG_UUID:
                status = GATTServApp_ProcessCCCWriteReq( connHandle, pAttr, pValue, len,
                                                         offset, GATT_CLIENT_CFG_NOTIFY );
                break;

            default:
                status = ATT_ERR_ATTR_NOT_FOUND;
                break;
        }
        switch (uuid)
                {
                    case OCCUPANCYPROFILE_RANGE_UUID:
                        notifyApp = OCCUPANCYPROFILE_RANGE;
                        break;
                    case OCCUPANCYPROFILE_SENSITIVITY_UUID:
                        notifyApp = OCCUPANCYPROFILE_SENSITIVITY;
                        break;
                    case OCCUPANCYPROFILE_TIMEOUT_UUID:
                        notifyApp = OCCUPANCYPROFILE_TIMEOUT;
                        break;
                    case GATT_CLIENT_CHAR_CFG_UUID:
                        notifyApp = OCCUPANCYPROFILE_DETECTION;
                        break;

                        break;

                }
    }
    else
    {
        // 128-bit UUID
        status = ATT_ERR_INVALID_HANDLE;
    }

    // If a characteristic value changed then callback function to notify application of change
    if ((notifyApp != 0xFF ) && occupancyProfile_appCBs && occupancyProfile_appCBs->pfnOccupancyProfile_Change)
    {
        OccupancyProfile_callback(notifyApp);
    }

    // Return status value
    return (status);
}

/**
 * @brief Handler for the Connect event.
 *
 * This function handles the Connect event by starting the sensor.
 */
void OccupancyProfile_on_connect(void)
{
    startSensor();
}

/**
 * @brief Handler for the Disconnect event.
 *
 * This function handles the Disconnect event by stopping the sensor if no active connections are present.
 */
void OccupancyProfile_on_disconnect(void)
{
    if(linkDB_NumActive() > 0)
    {
        stopSensor();
    }
}

/**
 * @brief Callback function for the Occupancy Profile service.
 *
 * This function is a callback for the Occupancy Profile service.
 *
 * @param paramID Parameter ID for the callback.
 */
void OccupancyProfile_callback(uint8_t paramID)
{
    char *pData = ICall_malloc(sizeof(char));

    if (pData == NULL)
    {
        return;
    }

    pData[0] = paramID;

    BLEAppUtil_invokeFunction(OccupancyProfile_invokeFromFWContext, pData);
}

/**
 * @brief Invokes the Occupancy Profile from the firmware context.
 *
 * This function invokes the Occupancy Profile from the firmware context.
 *
 * @param pData Data to be passed to the invoked function.
 */
void OccupancyProfile_invokeFromFWContext(char *pData)
{
    occupancyProfile_appCBs->pfnOccupancyProfile_Change(pData[0]);
}
