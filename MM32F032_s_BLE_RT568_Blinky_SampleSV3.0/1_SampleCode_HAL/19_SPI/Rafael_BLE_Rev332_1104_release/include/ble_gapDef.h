/**************************************************************************//**
* @file       ble_gapDef.h
* @brief      Provide the Definition of BLE Generic Access Profile (GAP).
*
*
* @defgroup ble_gapDef BLE Generic Access Profile (GAP) Definition
* @{
* @details  Common definitions for the GAP. (ble_gapDef.h).
* @}
*****************************************************************************/

#ifndef _BLE_GAPDEF_H_
#define _BLE_GAPDEF_H_


/** @brief BLE Address Length.
 * @ingroup ble_gapDef
*/
#define SIZE_BLE_ADDR                                                           6


/**
 * @defgroup bleGapAdvTypeLen BLE Gap Data Type Pre-defined Length
 * @{
 * @ingroup ble_gapDef
 * @details  BLE Gap data type pre-defined length.
 */
#define GAP_AD_TYPE_LENGTH_0                                                    0
#define GAP_AD_TYPE_LENGTH_1                                                    1
#define GAP_AD_TYPE_LENGTH_2                                                    2
#define GAP_AD_TYPE_LENGTH_3                                                    3
#define GAP_AD_TYPE_LENGTH_4                                                    4
#define GAP_AD_TYPE_LENGTH_5                                                    5
#define GAP_AD_TYPE_LENGTH_6                                                    6
#define GAP_AD_TYPE_LENGTH_7                                                    7
#define GAP_AD_TYPE_LENGTH_8                                                    8
#define GAP_AD_TYPE_LENGTH_9                                                    9
#define GAP_AD_TYPE_LENGTH_10                                                   10
#define GAP_AD_TYPE_LENGTH_11                                                   11
#define GAP_AD_TYPE_LENGTH_12                                                   12
#define GAP_AD_TYPE_LENGTH_13                                                   13
#define GAP_AD_TYPE_LENGTH_14                                                   14
#define GAP_AD_TYPE_LENGTH_15                                                   15
/** @} */


/**
 * @defgroup bleGapAdvType BLE Gap Data Type Formats
 * @{
 * @ingroup ble_gapDef
 * @details  BLE Gap data type format.
 */
//Bluetooth Spec. Ver4.0 [Vol 3] page 401~405 of 656
//https://www.bluetooth.org/en-us/specification/assigned-numbers/generic-access-profile

#define GAP_AD_TYPE_FLAGS                                                       0x01  /**< Flags for discoverability. @ref bleGapFlags "BLE Gap Flags Mode" */
#define GAP_AD_TYPE_SERVICE_MORE_16B_UUID                                       0x02  /**< Partial list of 16 bit service UUIDs. */
#define GAP_AD_TYPE_SERVICE_CPLT_16B_UUID                                       0x03  /**< Complete list of 16 bit service UUIDs. */
#define GAP_AD_TYPE_SERVICE_MORE_32B_UUID                                       0x04  /**< Partial list of 32 bit service UUIDs. */
#define GAP_AD_TYPE_SERVICE_CPLT_32B_UUID                                       0x05  /**< Complete list of 32 bit service UUIDs. */
#define GAP_AD_TYPE_SERVICE_MORE_128B_UUID                                      0x06  /**< Partial list of 128 bit service UUIDs. */
#define GAP_AD_TYPE_SERVICE_CPLT_128B_UUID                                      0x07  /**< Complete list of 128 bit service UUIDs. */
#define GAP_AD_TYPE_LOCAL_NAME_SHORTENED                                        0x08  /**< Short local device name. */
#define GAP_AD_TYPE_LOCAL_NAME_COMPLETE                                         0x09  /**< Complete local device name. */
#define GAP_AD_TYPE_TX_POWER_LEVEL                                              0x0A  /**< Transmit power level, 1byte : 0xXX:-127 to +127dBm */
#define GAP_AD_TYPE_SIMPLE_PAIRING_OPT_OOB_CLASS_OF_DEVICE                      0x0D  /**< Class of device. */
#define GAP_AD_TYPE_SIMPLE_PAIRING_OPT_OOB_HASH_C                               0x0E  /**< Simple Pairing Hash C. */
#define GAP_AD_TYPE_SIMPLE_PAIRING_OPT_OOB_RANDOMIZER_R                         0x0F  /**< Simple Pairing Randomizer R. */
#define GAP_AD_TYPE_SECURITY_MANAGER_TK_VALUE                                   0x10  /**< Security Manager TK Value. */
#define GAP_AD_TYPE_SECURITY_MANAGER_OOB_FLAGS                                  0x11  /**< Security Manager Out Of Band Flags. */
#define GAP_AD_TYPE_SLAVE_CONNECTION_INTERVAL_RANGE                             0x12  /**< Slave Connection Interval Range. */
#define GAP_AD_TYPE_SERVICE_SOLICITATION_16B_UUID                               0x14  /**< List of 16-bit Service Solicitation UUIDs. */
#define GAP_AD_TYPE_SERVICE_SOLICITATION_128B_UUID                              0x15  /**< List of 128-bit Service Solicitation UUIDs. */
#define GAP_AD_TYPE_SERVICE_DATA                                                0x16  /**< Service Data - 16-bit UUID. */
#define GAP_AD_TYPE_PUBLIC_TARGET_ADDRESS                                       0x17  /**< Public Target Address. */
#define GAP_AD_TYPE_RANDOM_TARGET_ADDRESS                                       0x18  /**< Random Target Address. */
#define GAP_AD_TYPE_APPEARANCE                                                  0x19  /**< Appearance. */
#define GAP_AD_TYPE_ADVERTISING_INTERVAL                                        0x1A  /**< Advertising Interval. */
#define GAP_AD_TYPE_3D_INFORMATION_DATA                                         0x3D  /**< 3D Information Data. */
#define GAP_AD_TYPE_MANUFACTURER_SPECIFIC_DATA                                  0xFF  /**< Manufacturer Specific Data. */
/** @} */


/**
 * @defgroup bleGapFlags BLE Gap Flags Mode
 * @{
 * @ingroup ble_gapDef
 * @details  BLE Gap flags mode.
 */
//Bluetooth CSS. v9 page 12 of 37

#define GAP_FLAGS_LIMITED_DISCOVERABLE_MODE                                     0x01  /**< LE Limited Discoverable Mode */
#define GAP_FLAGS_GENERAL_DISCOVERABLE_MODE                                     0x02  /**< LE General Discoverable Mode */
#define GAP_FLAGS_BR_EDR_NOT_SUPPORTED                                          0x04  /**< BR/EDR Not Supported. */
#define GAP_FLAGS_SIMUL_DEV_CAP_CONTROLLER                                      0x08  /**< Simultaneous LE and BR/EDR to Same Device Capable (Controller). */
#define GAP_FLAGS_SIMUL_DEV_CAP_HOST                                            0x10  /**< Simultaneous LE and BR/EDR to Same Device Capable (Host). */
#define BLE_GAP_FLAGS_LIMITED_DISCOVERABLE_MODE                                 (GAP_FLAGS_LIMITED_DISCOVERABLE_MODE|GAP_FLAGS_BR_EDR_NOT_SUPPORTED)  /**< LE Limited Discoverable Mode, BR/EDR Not Supported. */
#define BLE_GAP_FLAGS_GENERAL_DISCOVERABLE_MODE                                 (GAP_FLAGS_GENERAL_DISCOVERABLE_MODE|GAP_FLAGS_BR_EDR_NOT_SUPPORTED)  /**< LE General Discoverable Mode, BR/EDR Not Supported. */
/** @} */


#endif // _BLE_GAPDEF_H_
