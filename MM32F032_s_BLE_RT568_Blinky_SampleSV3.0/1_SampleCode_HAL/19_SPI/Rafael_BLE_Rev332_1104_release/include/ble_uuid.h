/**************************************************************************//**
* @file       ble_uuid.h
* @brief      Provide the Definition of BLE UUIDs.
*
*
* @defgroup ble_uuid BLE UUIDs
* @{
* @details  BLE UUIDs definition. (ble_uuid.h).
* @}
*****************************************************************************/

#ifndef _BLE_UUID_H_
#define _BLE_UUID_H_



/**
 * @defgroup bleGeneric_uuid BLE GATT UUIDs Definition
 * @{
 * @ingroup ble_uuid
 * @details BLE GATT UUIDs definition.
 * @}
 *
 * @defgroup bleDeclaration_uuid BLE GATT Declaration UUIDs Definition
 * @{
 * @ingroup ble_uuid
 * @details BLE GATT declaration UUIDs definition.
 * @ingroup bleGeneric_uuid
 */
//****************** 0x2800 through 0x28FF are for Attribute Types *******************/
#define GATT_DECLARATIONS                          0x28 /**< The High Byte of Generic UUID.*/
#define GATT_DECLARATIONS_PRIMARY_SERVICE          0x00 /**< The Low Byte of Primary Service UUID. */
#define GATT_DECLARATIONS_SECONDARY_SERVICE        0x01 /**< The Low Byte of Secondary Service UUID. */
#define GATT_DECLARATIONS_INCLUDE                  0x02 /**< The Low Byte of Include UUID. */
#define GATT_DECLARATIONS_CHARACTERISTIC           0x03 /**< The Low Byte of Characteristic UUID. */
//************************************************************************************/
/** @} */


/**
 * @defgroup bleDescriptor_uuid BLE GATT Descriptors UUIDs Definition
 * @{
 * @ingroup bleGeneric_uuid
 * @details BLE GATT descriptors UUIDs definition.
 */
//************* 0x2900 through 0x29FF are for Characteristic Descriptors *************/
#define GATT_DESCRIPTORS                                        0x29  /**< The High Byte of Characteristic Descriptor. */
#define GATT_DESCRIPTORS_CHARACTERISTIC_AGGREGATE_FORMAT        0x05  /**< The Low Byte of Characteristic Aggregate Format Descriptor. */
#define GATT_DESCRIPTORS_CHARACTERISTIC_EXTENDED_PROPERTIES     0x00  /**< The Low Byte of Characteristic Extended Properties Descriptor. */
#define GATT_DESCRIPTORS_CHARACTERISTIC_PRESENTATION_FORMAT     0x04  /**< The Low Byte of Characteristic Presentation Format Descriptor. */
#define GATT_DESCRIPTORS_CHARACTERISTIC_USER_DESCRIPTION        0x01  /**< The Low Byte of Characteristic User Description Descriptor. */
#define GATT_DESCRIPTORS_CLIENT_CHARACTERISTIC_CONFIGURATION    0x02  /**< The Low Byte of Client Characteristic Configuration Descriptor. */
#define GATT_DESCRIPTORS_EXTERNAL_REPORT_REFERENCE              0x07  /**< The Low Byte of External Report Reference Descriptor. */
#define GATT_DESCRIPTORS_REPORT_REFERENCE                       0x08  /**< The Low Byte of Report Reference Descriptor. */
#define GATT_DESCRIPTORS_SERVER_CHARACTERISTIC_CONFIGURATION    0x03  /**< The Low Byte of Server Characteristic Configuration Descriptor. */
#define GATT_DESCRIPTORS_VALID_RANGE                            0x06  /**< The Low Byte of Valid Range Descriptor. */
//************************************************************************************/

/** @} */
/** @} */ // bleGeneric_uuid group



/**
 * @defgroup bleService_uuid BLE GATT Service UUIDs Definition
 * @{
 * @ingroup ble_uuid
 * @details BLE GATT service UUIDs definition.
 * @}
 * @defgroup bleSIGService_uuid BLE SIG Defined Service UUIDs Definition
 * @{
 * @ingroup bleService_uuid
 * @details BLE SIG defined service UUIDs definition.
 */
//******************* 0x1800 through 0x26FF are for Service UUIDs **************************************************************/
#define GATT_SPEC_SERVICES                                  0x18  /**< The High Byte of SIG Defined Service UUID.*/
#define GATT_SPEC_SERVICES_ALERT_NOTIFICATION_SERVICE       0x11  /**< The Low Byte of Alert Notification Service UUID. */
#define GATT_SPEC_SERVICES_AUTOMATION_IO                    0x15  /**< The Low Byte of Automation IO UUID. */
#define GATT_SPEC_SERVICES_BATTERY_SERVICE                  0x0F  /**< The Low Byte of Battery Service UUID. */
#define GATT_SPEC_SERVICES_BLOOD_PRESSURE                   0x10  /**< The Low Byte of Blood Pressure Service UUID. */
#define GATT_SPEC_SERVICES_CURRENT_TIME_SERVICE             0x05  /**< The Low Byte of Current Time Service UUID. */
#define GATT_SPEC_SERVICES_CYCLING_POWER                    0x18  /**< The Low Byte of Cycling Power Service UUID. */
#define GATT_SPEC_SERVICES_CYCLING_SPEED_AND_CADENCE        0x16  /**< The Low Byte of Cycling Speed and Cadence Service UUID. */
#define GATT_SPEC_SERVICES_DEVICE_INFORMATION               0x0A  /**< The Low Byte of Device Information Service UUID. */
#define GATT_SPEC_SERVICES_GENERIC_ACCESS                   0x00  /**< The Low Byte of Generic Access Profile Service UUID */
#define GATT_SPEC_SERVICES_GENERIC_ATTRIBUTE                0x01  /**< The Low Byte of Generic Attribute Profile Service UUID */
#define GATT_SPEC_SERVICES_GLUCOSE                          0x08  /**< The Low Byte of Glucose Service UUID. */
#define GATT_SPEC_SERVICES_HEALTH_THERMOMETER               0x09  /**< The Low Byte of Health Thermometer Service UUID. */
#define GATT_SPEC_SERVICES_HEART_RATE                       0x0D  /**< The Low Byte of Heart Rate Service UUID. */
#define GATT_SPEC_SERVICES_HUMAN_INTERFACE_DEVICE           0x12  /**< The Low Byte of Human Interface Device Service UUID. */
#define GATT_SPEC_SERVICES_IMMEDIATE_ALERT                  0x02  /**< The Low Byte of Immediate Alert Service UUID. */
#define GATT_SPEC_SERVICES_LINK_LOSS                        0x03  /**< The Low Byte of Link Loss Service UUID. */
#define GATT_SPEC_SERVICES_NETWORK_AVAILABILITY_SERVICE     0x0B  /**< The Low Byte of Network Availability Service UUID. */
#define GATT_SPEC_SERVICES_LOCATION_AND_NAVIGATION          0x19  /**< The Low Byte of Location and Navigation Service UUID. */
#define GATT_SPEC_SERVICES_NEXT_DST_CHANGE_SERVICE          0x07  /**< The Low Byte of Next Dst Change Service UUID. */
#define GATT_SPEC_SERVICES_PHONE_ALERT_STATUS_SERVICE       0x0E  /**< The Low Byte of Phone Alert Status Service UUID. */
#define GATT_SPEC_SERVICES_REFERENCE_TIME_UPDATE_SERVICE    0x06  /**< The Low Byte of Reference Time Update Service UUID. */
#define GATT_SPEC_SERVICES_RUNNING_SPEED_AND_CADENCE        0x14  /**< The Low Byte of Running Speed and Cadence Service UUID. */
#define GATT_SPEC_SERVICES_SCAN_PARAMETERS                  0x13  /**< The Low Byte of Scan Parameters Service UUID. */
#define GATT_SPEC_SERVICES_TX_POWER                         0x04  /**< TX Power Service UUID. */
#define GATT_SPEC_SERVICES_USER_DATA                        0x1C  /**< The Low Byte of User Data Service UUID. */
#define GATT_SPEC_SERVICES_BODY_COMPOSITION                 0x1B  /**< The Low Byte of Body Composition Service UUID. */
#define GATT_SPEC_SERVICES_WEIGHT_SCALE                     0x1D  /**< The Low Byte of Weight Scale Service UUID. */
#define GATT_SPEC_SERVICES_BOND_MANAGEMENT                  0x1E  /**< The Low Byte of Bond Management Service UUID. */
//******************************************************************************************************************************/
/** @} */

/**
 * @defgroup bleCustomerService_uuid BLE Customer Service UUIDs Definition
 * @{
 * @ingroup bleService_uuid
 * @details BLE customer defined service UUIDs definition.
 */
//******************* 0xF900 through 0xF9FF are for Customer Service UUIDs *****************************************************/
#define GATT_SPEC_CSTM_SERVICES                             0xF9  /**< The High Byte of Customer Service UUID.*/
#define GATT_SPEC_CSTM_SERVICES_UDF01S                      0x01  /**< The Low Byte of User Defined 01 Service UUID. */
#define GATT_SPEC_CSTM_SERVICES_UDF02S                      0x02  /**< The Low Byte of User Defined 02 Service UUID. */
#define GATT_SPEC_CSTM_SERVICES_UDF03S                      0x03  /**< The Low Byte of User Defined 03 Service UUID. */
#define GATT_SPEC_CSTM_SERVICES_ATCMD                       0x04  /**< The Low Byte of AT command Service UUID. */
//******************************************************************************************************************************/

/** @} */


/**
 * @defgroup bleOta_uuid BLE Over The Air (OTA) Service UUIDs Definition
 * @{
 * @ingroup bleService_uuid
 * @details BLE OTA Service UUIDs definition.
 */
#ifdef _PROFILE_OTA_
//************************** 0xFEBA is for OTA UUIDs *********************************/
#define GATT_SPEC_OTA_SERVICES                              0xFE  /**< The High Byte of OTA Service UUID.*/
#define GATT_SPEC_OTA_SERVICES_OTA                          0xBA  /**< The Low Byte of OTA Service UUID.*/
//************************************************************************************/
#endif
/** @} */


/** @} */




/**
 * @defgroup bleCharacteristic_uuid BLE GATT Characteristic UUIDs Definition
 * @{
 * @ingroup ble_uuid
 * @details BLE GATT characteristic UUIDs definition.
 * @}
 * @defgroup bleSigCharacteristic_uuid BLE SIG Defined Characteristic UUIDs Definition
 * @{
 * @ingroup bleCharacteristic_uuid
 * @details BLE SIG defined characteristic UUIDs definition.
 */
//**************** 0x2A00 through 0x7FFF are for Characteristic Types ****************/
#define GATT_SPEC_CHARACTERISTIC                                                0x2A  /**< The High Byte of SIG Defined Characteristic UUID.*/
#define GATT_SPEC_CHARACTERISTIC_ALERT_CATEGORY_ID                              0x43  /**< The Low Byte of Alert Category Id Characteristic UUID. */
#define GATT_SPEC_CHARACTERISTIC_ALERT_CATEGORY_ID_BIT_MASK                     0x42  /**< The Low Byte of Alert Category Id Bit Mask Characteristic UUID. */
#define GATT_SPEC_CHARACTERISTIC_ALERT_LEVEL                                    0x06  /**< The Low Byte of Alert Level Characteristic UUID. */
#define GATT_SPEC_CHARACTERISTIC_ALERT_NOTIFICATION_CONTROL_POINT               0x44  /**< The Low Byte of Alert Notification Control Point Characteristic UUID. */
#define GATT_SPEC_CHARACTERISTIC_ALERT_STATUS                                   0x3F  /**< The Low Byte of Alert Status Characteristic UUID. */
#define GATT_SPEC_CHARACTERISTIC_APPEARANCE                                     0x01  /**< The Low Byte of Appearance Characteristic UUID. */
#define GATT_SPEC_CHARACTERISTIC_BATTERY_LEVEL                                  0x19  /**< The Low Byte of Battery Level Characteristic UUID. */
#define GATT_SPEC_CHARACTERISTIC_BLOOD_PRESSURE_FEATURE                         0x49  /**< The Low Byte of Blood Pressure Feature Characteristic UUID. */
#define GATT_SPEC_CHARACTERISTIC_BLOOD_PRESSURE_MEASUREMENT                     0x35  /**< The Low Byte of Blood Pressure Measurement Characteristic UUID. */
#define GATT_SPEC_CHARACTERISTIC_BODY_SENSOR_LOCATION                           0x38  /**< The Low Byte of Body Sensor Location Characteristic UUID. */
#define GATT_SPEC_CHARACTERISTIC_BOOT_KEYBOARD_INPUT_REPORT                     0x22  /**< The Low Byte of Boot Keyboard Input Report Characteristic UUID. */
#define GATT_SPEC_CHARACTERISTIC_BOOT_KEYBOARD_OUTPUT_REPORT                    0x32  /**< The Low Byte of Boot Keyboard Output Report Characteristic UUID. */
#define GATT_SPEC_CHARACTERISTIC_BOOT_MOUSE_INPUT_REPORT                        0x33  /**< The Low Byte of Boot Mouse Input Report Characteristic UUID. */
#define GATT_SPEC_CHARACTERISTIC_CSC_FEATURE                                    0x5C  /**< The Low Byte of Cycling Speed and Cadence Feature Characteristic UUID. */
#define GATT_SPEC_CHARACTERISTIC_CSC_MEASUREMENT                                0x5B  /**< The Low Byte of Cycling Speed and Cadence Measurement Characteristic UUID. */
#define GATT_SPEC_CHARACTERISTIC_CURRENT_TIME                                   0x2B  /**< The Low Byte of Current Time Characteristic UUID. */
#define GATT_SPEC_CHARACTERISTIC_CYCLING_POWER_CONTROL_POINT                    0x66  /**< The Low Byte of Cycling Power Control Point Characteristic UUID. */
#define GATT_SPEC_CHARACTERISTIC_CYCLING_POWER_FEATURE                          0x65  /**< The Low Byte of Cycling Power Feature Characteristic UUID. */
#define GATT_SPEC_CHARACTERISTIC_CYCLING_POWER_MEASUREMENT                      0x63  /**< The Low Byte of Cycling Power Measurement Characteristic UUID. */
#define GATT_SPEC_CHARACTERISTIC_CYCLING_POWER_VECTOR                           0x64  /**< The Low Byte of Cycling Power Vector Characteristic UUID. */
#define GATT_SPEC_CHARACTERISTIC_DATE_TIME                                      0x08  /**< The Low Byte of Date Time Characteristic UUID. */
#define GATT_SPEC_CHARACTERISTIC_DAY_DATE_TIME                                  0x0A  /**< The Low Byte of Day Date Time Characteristic UUID. */
#define GATT_SPEC_CHARACTERISTIC_DAY_OF_WEEK                                    0x09  /**< The Low Byte of Day Of Week Characteristic UUID. */
#define GATT_SPEC_CHARACTERISTIC_DEVICE_NAME                                    0x00  /**< The Low Byte of Device Name Characteristic UUID. */
#define GATT_SPEC_CHARACTERISTIC_DST_OFFSET                                     0x0D  /**< The Low Byte of Dst Offset Characteristic UUID. */
#define GATT_SPEC_CHARACTERISTIC_ExACT_TIME_256                                 0x0C  /**< The Low Byte of Exact Time 256 Characteristic UUID. */
#define GATT_SPEC_CHARACTERISTIC_FIRMWARE_REVISION_STRING                       0x26  /**< The Low Byte of Firmware Revision String Characteristic UUID. */
#define GATT_SPEC_CHARACTERISTIC_GLUCOSE_FEATURE                                0x51  /**< The Low Byte of Glucose Feature Characteristic UUID. */
#define GATT_SPEC_CHARACTERISTIC_GLUCOSE_MEASUREMENT                            0x18  /**< The Low Byte of Glucose Measurement Characteristic UUID. */
#define GATT_SPEC_CHARACTERISTIC_GLUCOSE_MEASUREMENT_CONTEXT                    0x34  /**< The Low Byte of Glucose Measurement Context Characteristic UUID. */
#define GATT_SPEC_CHARACTERISTIC_HARDWARE_REVISION_STRING                       0x27  /**< The Low Byte of Hardware Revision String Characteristic UUID. */
#define GATT_SPEC_CHARACTERISTIC_HEART_RATE_CONTROL_POINT                       0x39  /**< The Low Byte of Heart Rate Control Point Characteristic UUID. */
#define GATT_SPEC_CHARACTERISTIC_HEART_RATE_MEASUREMENT                         0x37  /**< The Low Byte of Heart Rate Measurement Characteristic UUID. */
#define GATT_SPEC_CHARACTERISTIC_HID_CONTROL_POINT                              0x4C  /**< The Low Byte of Hid Control Point Characteristic UUID. */
#define GATT_SPEC_CHARACTERISTIC_HID_INFORMATION                                0x4A  /**< The Low Byte of Hid Information Characteristic UUID. */
#define GATT_SPEC_CHARACTERISTIC_IEEE_11073_REGULATORY_CERTIFICATION_DATA_LIST  0x2A  /**< The Low Byte of IEEE Regulatory Certification Data List Characteristic UUID. */
#define GATT_SPEC_CHARACTERISTIC_INTERMEDIATE_CUFF_PRESSURE                     0x36  /**< The Low Byte of Intermediate Cuff Pressure Characteristic UUID. */
#define GATT_SPEC_CHARACTERISTIC_INTERMEDIATE_TEMPERATURE                       0x1E  /**< The Low Byte of Intermediate Temperature Characteristic UUID. */
#define GATT_SPEC_CHARACTERISTIC_NAVIGATION                                     0x68  /**< The Low Byte of Navigation Characteristic UUID. */
#define GATT_SPEC_CHARACTERISTIC_LN_CONTROL_POINT                               0x6B  /**< The Low Byte of Location Navigation Control point Characteristic UUID. */
#define GATT_SPEC_CHARACTERISTIC_LN_FEATURE                                     0x6A  /**< The Low Byte of Location Navigation Feature Characteristic UUID. */
#define GATT_SPEC_CHARACTERISTIC_LOCAL_TIME_INFORMATION                         0x0F  /**< The Low Byte of Local Time Information Characteristic UUID. */
#define GATT_SPEC_CHARACTERISTIC_LOCATION_AND_SPEED                             0x67  /**< The Low Byte of Location and Speed Characteristic UUID. */
#define GATT_SPEC_CHARACTERISTIC_MANUFACTURER_NAME_STRING                       0x29  /**< The Low Byte of Manufacturer Name String Characteristic UUID. */
#define GATT_SPEC_CHARACTERISTIC_MEASUREMENT_INTERVAL                           0x21  /**< The Low Byte of Measurement Interval Characteristic UUID. */
#define GATT_SPEC_CHARACTERISTIC_MODEL_NUMBER_STRING                            0x24  /**< The Low Byte of Model Number String Characteristic UUID. */
#define GATT_SPEC_CHARACTERISTIC_NEW_ALERT                                      0x46  /**< The Low Byte of New Alert Characteristic UUID. */
#define GATT_SPEC_CHARACTERISTIC_PERIPHERAL_PREFERRED_CONNECTION_PARAMETERS     0x04  /**< The Low Byte of Peripheral Preferred Connection Parameters Characteristic UUID. */
#define GATT_SPEC_CHARACTERISTIC_PERIPHERAL_PRIVACY_FLAG                        0x02  /**< The Low Byte of Peripheral Privacy Flag Characteristic UUID. */
#define GATT_SPEC_CHARACTERISTIC_PNP_ID                                         0x50  /**< The Low Byte of PNP Id Characteristic UUID. */
#define GATT_SPEC_CHARACTERISTIC_POSITION_QUALITY                               0x69  /**< The Low Byte of Position Quality Characteristic UUID. */
#define GATT_SPEC_CHARACTERISTIC_PROTOCOL_MODE                                  0x4E  /**< The Low Byte of Protocol Mode Characteristic UUID. */
#define GATT_SPEC_CHARACTERISTIC_RECONNECTION_ADDRESS                           0x03  /**< The Low Byte of Reconnection Address Characteristic UUID. */
#define GATT_SPEC_CHARACTERISTIC_RECORD_ACCESS_CONTROL_POINT                    0x52  /**< The Low Byte of Record Access Control Point Characteristic UUID. */
#define GATT_SPEC_CHARACTERISTIC_REFERENCE_TIME_INFORMATION                     0x14  /**< The Low Byte of Reference Time Information Characteristic UUID. */
#define GATT_SPEC_CHARACTERISTIC_REPORT                                         0x4D  /**< The Low Byte of Report Characteristic UUID. */
#define GATT_SPEC_CHARACTERISTIC_REPORT_MAP                                     0x4B  /**< The Low Byte of Report Map Characteristic UUID. */
#define GATT_SPEC_CHARACTERISTIC_RINGER_CONTROL_POINT                           0x40  /**< The Low Byte of Ringer Control Point Characteristic UUID. */
#define GATT_SPEC_CHARACTERISTIC_RINGER_SETTING                                 0x41  /**< The Low Byte of Ringer Setting Characteristic UUID. */
#define GATT_SPEC_CHARACTERISTIC_RSC_FEATURE                                    0x54  /**< The Low Byte of Running Speed and Cadence Feature Characteristic UUID. */
#define GATT_SPEC_CHARACTERISTIC_RSC_MEASUREMENT                                0x53  /**< The Low Byte of Running Speed and Cadence Measurement Characteristic UUID. */
#define GATT_SPEC_CHARACTERISTIC_SC_CONTROL_POINT                               0x55  /**< The Low Byte of Speed and Cadence Control Point Characteristic UUID. */
#define GATT_SPEC_CHARACTERISTIC_SCAN_INTERVAL_WINDOW                           0x4F  /**< The Low Byte of Scan Interval Window Characteristic UUID. */
#define GATT_SPEC_CHARACTERISTIC_SCAN_REFRESH                                   0x31  /**< The Low Byte of Scan Refresh Characteristic UUID. */
#define GATT_SPEC_CHARACTERISTIC_SENSOR_LOCATION                                0x5D  /**< The Low Byte of Sensor Location Characteristic UUID. */
#define GATT_SPEC_CHARACTERISTIC_SERIAL_NUMBER_STRING                           0x25  /**< The Low Byte of Serial Number String Characteristic UUID. */
#define GATT_SPEC_CHARACTERISTIC_SERVICE_CHANGED                                0x05  /**< The Low Byte of Service Changed Characteristic UUID. */
#define GATT_SPEC_CHARACTERISTIC_SOFTWARE_REVISION_STRING                       0x28  /**< The Low Byte of Software Revision String Characteristic UUID. */
#define GATT_SPEC_CHARACTERISTIC_SUPPORTED_NEW_ALERT_CATEGORY                   0x47  /**< The Low Byte of Supported New Alert Category Characteristic UUID. */
#define GATT_SPEC_CHARACTERISTIC_SUPPORTED_UNREAD_ALERT_CATEGORY                0x48  /**< The Low Byte of Supported Unread Alert Category Characteristic UUID. */
#define GATT_SPEC_CHARACTERISTIC_SYSTEM_ID                                      0x23  /**< The Low Byte of System Id Characteristic UUID. */
#define GATT_SPEC_CHARACTERISTIC_TEMPERATURE_MEASUREMENT                        0x1C  /**< The Low Byte of Temperature Measurement Characteristic UUID. */
#define GATT_SPEC_CHARACTERISTIC_TEMPERATURE_TYPE                               0x1D  /**< The Low Byte of Temperature Type Characteristic UUID. */
#define GATT_SPEC_CHARACTERISTIC_TIME_ACCURACY                                  0x12  /**< The Low Byte of Time Accuracy Characteristic UUID. */
#define GATT_SPEC_CHARACTERISTIC_TIME_SOURCE                                    0x13  /**< The Low Byte of Time Source Characteristic UUID. */
#define GATT_SPEC_CHARACTERISTIC_TIME_UPDATE_CONTROL_POINT                      0x16  /**< The Low Byte of Time Update Control Point Characteristic UUID. */
#define GATT_SPEC_CHARACTERISTIC_TIME_UPDATE_STATE                              0x17  /**< The Low Byte of Time Update State Characteristic UUID. */
#define GATT_SPEC_CHARACTERISTIC_TIME_WITH_DST                                  0x11  /**< The Low Byte of Time With Dst Characteristic UUID. */
#define GATT_SPEC_CHARACTERISTIC_TIME_ZONE                                      0x0E  /**< The Low Byte of Time Zone Characteristic UUID. */
#define GATT_SPEC_CHARACTERISTIC_TX_POWER_LEVEL                                 0x07  /**< The Low Byte of TX Power Level Characteristic UUID. */
#define GATT_SPEC_CHARACTERISTIC_UNREAD_ALERT_STATUS                            0x45  /**< The Low Byte of Unread Alert Status Characteristic UUID. */
#define GATT_SPEC_CHARACTERISTIC_AGGREGATE_INPUT                                0x5A  /**< The Low Byte of Aggregate Input Characteristic UUID. */
#define GATT_SPEC_CHARACTERISTIC_ANALOG_INPUT                                   0x58  /**< The Low Byte of Analog Input Characteristic UUID. */
#define GATT_SPEC_CHARACTERISTIC_ANALOG_OUTPUT                                  0x59  /**< The Low Byte of Analog Output Characteristic UUID. */
#define GATT_SPEC_CHARACTERISTIC_DIGITAL_INPUT                                  0x56  /**< The Low Byte of Digital Input Characteristic UUID. */
#define GATT_SPEC_CHARACTERISTIC_DIGITAL_OUTPUT                                 0x57  /**< The Low Byte of Digital Output Characteristic UUID. */
#define GATT_SPEC_CHARACTERISTIC_EXACT_TIME_100                                 0x0B  /**< The Low Byte of Exact Time 100 Characteristic UUID. */
#define GATT_SPEC_CHARACTERISTIC_NETWORK_AVAILABILITY                           0x3E  /**< The Low Byte of Network Availability Characteristic UUID. */
#define GATT_SPEC_CHARACTERISTIC_SCIENTIFIC_TEMPERATURE_IN_CELSIUS              0x3C  /**< The Low Byte of Scientific Temperature in Celsius Characteristic UUID. */
#define GATT_SPEC_CHARACTERISTIC_SECONDARY_TIME_ZONE                            0x10  /**< The Low Byte of Secondary Time Zone Characteristic UUID. */
#define GATT_SPEC_CHARACTERISTIC_STRING                                         0x3D  /**< The Low Byte of String Characteristic UUID. */
#define GATT_SPEC_CHARACTERISTIC_TEMPERATURE_IN_CELSIUS                         0x1F  /**< The Low Byte of Temperature in Celsius Characteristic UUID. */
#define GATT_SPEC_CHARACTERISTIC_TEMPERATURE_IN_FAHRENHEIT                      0x20  /**< The Low Byte of Temperature in Fahrenheit Characteristic UUID. */
#define GATT_SPEC_CHARACTERISTIC_TIME_BROADCAST                                 0x15  /**< The Low Byte of Time Broadcast Characteristic UUID. */
#define GATT_SPEC_CHARACTERISTIC_BATTERY_LEVEL_STATE                            0x1B  /**< The Low Byte of Battery Level State Characteristic UUID. */
#define GATT_SPEC_CHARACTERISTIC_BATTERY_POWER_STATE                            0x1A  /**< The Low Byte of Battery Power State Characteristic UUID. */
#define GATT_SPEC_CHARACTERISTIC_PULSE_OXIMETRY_CONTINUOUS_MEASUREMENT          0x5F  /**< The Low Byte of Pulse Oximetry Continuous Measurement Characteristic UUID. */
#define GATT_SPEC_CHARACTERISTIC_PULSE_OXIMETRY_CONTROL_POINT                   0x62  /**< The Low Byte of Pulse Oximetry Control Point Characteristic UUID. */
#define GATT_SPEC_CHARACTERISTIC_PULSE_OXIMETRY_FEATURES                        0x61  /**< The Low Byte of Pulse Oximetry Features Characteristic UUID. */
#define GATT_SPEC_CHARACTERISTIC_PULSE_OXIMETRY_PULSATILE_EVENT                 0x60  /**< The Low Byte of Pulse Oximetry Pulsatile Event Characteristic UUID. */

//************************************************************************************/
/** @} */



/**
 * @defgroup bleCustomerCharacteristic_uuid BLE Customer Characteristic UUIDs Definition
 * @{
 * @ingroup bleCharacteristic_uuid
 * @details BLE customer characteristic UUIDs definition.
 */
//*********** 0xFA00 through 0xFAFF are for Customer Characteristic Types ************/
#define GATT_SPEC_CSTM_CHARACTERISTIC                       0xFA  /**< The High Byte of Customer Characteristic UUID.*/
#define GATT_SPEC_CSTM_CHARACTERISTIC_UDATR01               0x01  /**< The Low Byte of User Data Read 01 Characteristic UUID. */
#define GATT_SPEC_CSTM_CHARACTERISTIC_UDATN01               0x02  /**< The Low Byte of User Data Notification 01 Characteristic UUID. */
#define GATT_SPEC_CSTM_CHARACTERISTIC_UDATRW01              0x03  /**< The Low Byte of User Data Read Write 01 Characteristic UUID. */
#define GATT_SPEC_CSTM_CHARACTERISTIC_UDATR02               0x04  /**< The Low Byte of User Data Read 02 Characteristic UUID. */
#define GATT_SPEC_CSTM_CHARACTERISTIC_UDATN02               0x05  /**< The Low Byte of User Data Notification 02 Characteristic UUID. */
#define GATT_SPEC_CSTM_CHARACTERISTIC_UDATRW02              0x06  /**< The Low Byte of User Data Read Write 02 Characteristic UUID. */
#define GATT_SPEC_CSTM_CHARACTERISTIC_UDATR03               0x07  /**< The Low Byte of User Data Read 03 Characteristic UUID. */
#define GATT_SPEC_CSTM_CHARACTERISTIC_UDATN03               0x08  /**< The Low Byte of User Data Notification 03 Characteristic UUID. */
#define GATT_SPEC_CSTM_CHARACTERISTIC_UDATRW03              0x09  /**< The Low Byte of User Data Read Write 03 Characteristic UUID. */
#define GATT_SPEC_CSTM_CHARACTERISTIC_ATCMD_READ            0x01  /**< The Low Byte of AT Command Read Characteristic UUID. */
#define GATT_SPEC_CSTM_CHARACTERISTIC_ATCMD_NOTIFY          0x02  /**< The Low Byte of AT Command Notification Characteristic UUID. */
#define GATT_SPEC_CSTM_CHARACTERISTIC_ATCMD_RW              0x03  /**< The Low Byte of AT Command Read Write Characteristic UUID. */
//************************************************************************************/
/** @} */


/**
 * @defgroup bleOtaCharacteristic_uuid BLE Over The Air (OTA) Characteristic UUIDs Definition
 * @{
 * @ingroup bleCharacteristic_uuid
 * @details BLE OTA characteristic UUIDs definition.
 */
#ifdef _PROFILE_OTA_
//*********** 0xFA00 through 0xFAFF are for OTA Types ************/
#define GATT_SPEC_OTA_CHARACTERISTIC                        0xFA  /**< The High Byte of OTA Characteristic UUID.*/
#define GATT_SPEC_OTA_CHARACTERISTIC_OTA_DATA               0x10  /**< The Low Byte of OTA Data Characteristic UUID.*/
#define GATT_SPEC_OTA_CHARACTERISTIC_OTA_CMD                0x11  /**< The Low Byte of OTA Command Characteristic UUID.*/
//************************************************************************************/
#endif
/** @} */

/** @} */  // bleCharacteristic_uuid group





/**
 * @defgroup bleUnit_uuid BLE Unit UUIDs Definition
 * @{
 * @ingroup ble_uuid
 * @details BLE unit UUIDs definition.
 */
//*********************** 0x2700 through 0x27FF are for Units ************************/
#define GATT_SPEC_UNIT                                                          0x27  /**< The High Byte of Spec. Unit UUID.*/
#define GATT_SPEC_UNIT_UNITLESS                                                 0x00  /**< The Low Byte of Unitless UUID.*/
#define GATT_SPEC_UNIT_LENGTH__METRE                                            0x01  /**< The Low Byte of Length (metre) UUID.*/
#define GATT_SPEC_UNIT_MASS__KILOGRAM                                           0x02  /**< The Low Byte of Mass (kilogram) UUID.*/
#define GATT_SPEC_UNIT_TIME__SECOND                                             0x03  /**< The Low Byte of Time (second) UUID.*/
#define GATT_SPEC_UNIT_ELECTRIC_CURRENT__AMPERE                                 0x04  /**< The Low Byte of Electric Current (ampere) UUID.*/
#define GATT_SPEC_UNIT_THERMODYNAMIC_TEMPERATURE__KELVIN                        0x05  /**< The Low Byte of Thermodynamic Temperature (kelvin) UUID.*/
#define GATT_SPEC_UNIT_AMOUNT_OF_SUBSTANCE__MOLE                                0x06  /**< The Low Byte of Amount of Substance (mole) UUID.*/
#define GATT_SPEC_UNIT_LUMINOUS_INTENSITY__CANDELA                              0x07  /**< The Low Byte of Luminous Intensity (candela) UUID.*/
#define GATT_SPEC_UNIT_AREA__SQUARE_METRES                                      0x10  /**< The Low Byte of Area (square metres) UUID.*/
#define GATT_SPEC_UNIT_VOLUME__CUBIC_METRES                                     0x11  /**< The Low Byte of Volume (cubic metres) UUID.*/
#define GATT_SPEC_UNIT_VELOCITY__METRES_PER_SECOND                              0x12  /**< The Low Byte of Velocity (meres oer second) UUID.*/
#define GATT_SPEC_UNIT_ACCELERATION__METRES_PER_SECOND_SQUARED                  0x13  /**< The Low Byte of Acceleration (metres per second squared) UUID.*/
#define GATT_SPEC_UNIT_WAVENUMBER__RECIPROCAL_METRE                             0x14  /**< The Low Byte of Wavenumber (reciprocal metre) UUID.*/
#define GATT_SPEC_UNIT_DENSITY__KILOGRAM_PER_CUBIC_METRE                        0x15  /**< The Low Byte of Density (kilogram per cubic metre) UUID.*/
#define GATT_SPEC_UNIT_SURFACE_DENSITY__KILOGRAM_PER_SQUARE_METRE               0x16  /**< The Low Byte of Surface Density (kilogram per square metre) UUID.*/
#define GATT_SPEC_UNIT_SPECIFIC_VOLUME__CUBIC_METRE_PER_KILOGRAM                0x17  /**< The Low Byte of Specific Volume (cubic metre per kilogram) UUID.*/
#define GATT_SPEC_UNIT_CURRENT_DENSITY__AMPERE_PER_SQUARE_METRE                 0x18  /**< The Low Byte of Current Density (ampere per square metre) UUID.*/
#define GATT_SPEC_UNIT_MAGNETIC_FIELD_STRENGTH__AMPERE_PER_METRE                0x19  /**< The Low Byte of Magnetic Field Strength (ampere per metre) UUID.*/
#define GATT_SPEC_UNIT_AMOUNT_CONCENTRATION__MOLE_PER_CUBIC_METRE               0x1A  /**< The Low Byte of Aount Concentration (molde per cubic metre) UUID.*/
#define GATT_SPEC_UNIT_MASS_CONCENTRATION__KILOGRAM_PER_CUBIC_METRE             0x1B  /**< The Low Byte of Mass Concentration (kilogram per cubic meter) UUID.*/
#define GATT_SPEC_UNIT_LUMINANCE__CANDELA_PER_SQUARE_METRE                      0x1C  /**< The Low Byte of Luminance (candela per square metre) UUID.*/
#define GATT_SPEC_UNIT_REFRACTIVE_INDEX                                         0x1D  /**< The Low Byte of Refractive (index) UUID.*/
#define GATT_SPEC_UNIT_RELATIVE_PERMEABILITY                                    0x1E  /**< The Low Byte of Relative (permeability) UUID.*/
#define GATT_SPEC_UNIT_PLANE_ANGLE__RADIAN                                      0x20  /**< The Low Byte of PlaneAngle (radian) UUID.*/
#define GATT_SPEC_UNIT_SOLID_ANGLE__STERADIAN                                   0x21  /**< The Low Byte of Solid Angle (steradian) UUID.*/
#define GATT_SPEC_UNIT_FREQUENCY__HERTZ                                         0x22  /**< The Low Byte of Frequency (hertz) UUID.*/
#define GATT_SPEC_UNIT_FORCE__NEWTON                                            0x23  /**< The Low Byte of Force (newton) UUID.*/
#define GATT_SPEC_UNIT_PRESSURE__PASCAL                                         0x24  /**< The Low Byte of Pressure (pascal) UUID.*/
#define GATT_SPEC_UNIT_ENERGY__JOULE                                            0x25  /**< The Low Byte of Energy (joule) UUID.*/
#define GATT_SPEC_UNIT_POWER__WATT                                              0x26  /**< The Low Byte of Power (watt) UUID.*/
#define GATT_SPEC_UNIT_ELECTRIC_CHARGE__COULOMB                                 0x27  /**< The Low Byte of Electric Charge (coulomb) UUID.*/
#define GATT_SPEC_UNIT_ELECTRIC_POTENTIAL_DIFFERENCE__VOLT                      0x28  /**< The Low Byte of Electric Potential Difference (volt) UUID.*/
#define GATT_SPEC_UNIT_CAPACITANCE__FARAD                                       0x29  /**< The Low Byte of Capacitance (farad) UUID.*/
#define GATT_SPEC_UNIT_ELECTRIC_RESISTANCE__OHM                                 0x2A  /**< The Low Byte of Electric Resistance (ohm) UUID.*/
#define GATT_SPEC_UNIT_ELECTRIC_CONDUCTANCE__SIEMENS                            0x2B  /**< The Low Byte of Electric Conductance (siemens) UUID.*/
#define GATT_SPEC_UNIT_MAGNETIC_FLEX__WEBER                                     0x2C  /**< The Low Byte of Magnetic Flex (weber) UUID.*/
#define GATT_SPEC_UNIT_MAGNETIC_FLEX_DENSITY__TESLA                             0x2D  /**< The Low Byte of Magnetic Flex Density (tesla) UUID.*/
#define GATT_SPEC_UNIT_INDUCTANCE__HENRY                                        0x2E  /**< The Low Byte of Inductance (henry) UUID.*/
#define GATT_SPEC_UNIT_CELSIUS_TEMPERATURE__DEGREE_CELSIUS                      0x2F  /**< The Low Byte of Celsius Temperature (degree celsius) UUID.*/
#define GATT_SPEC_UNIT_LUMINOUS_FLUX__LUMEN                                     0x30  /**< The Low Byte of Luminous Flux (lumen) UUID.*/
#define GATT_SPEC_UNIT_ILLUMINANCE__LUX                                         0x31  /**< The Low Byte of Illuminance (lux) UUID.*/
#define GATT_SPEC_UNIT_ACTIVITY_REFERRED_TO_A_RADIONUCLIDE__BECQUEREL           0x32  /**< The Low Byte of Activity Referred to a Radionuclide (becquerel) UUID.*/
#define GATT_SPEC_UNIT_ABSORBED_DOSE__GRAY                                      0x33  /**< The Low Byte of Absorbed Dose (Gray) UUID.*/
#define GATT_SPEC_UNIT_DOSE_EQUIVALENT__SIEVERT                                 0x34  /**< The Low Byte of Dose Equivalent (sievert) UUID.*/
#define GATT_SPEC_UNIT_CATALYTIC_ACTIVITY__KATAL                                0x35  /**< The Low Byte of Catalytic Activity (katal) UUID.*/
#define GATT_SPEC_UNIT_DYNAMIC_VISCOSITY__PASCAL_SECOND                         0x40  /**< The Low Byte of Dynamic Viscosity (pascal second) UUID.*/
#define GATT_SPEC_UNIT_MOMENT_OF_FORCE__NEWTON_METRE                            0x41  /**< The Low Byte of Moment of Force (newton metre) UUID.*/
#define GATT_SPEC_UNIT_SURFACE_TENSION__NEWTON_PER_METRE                        0x42  /**< The Low Byte of Surface Tension (newton per metre) UUID.*/
#define GATT_SPEC_UNIT_ANGULAR_VELOCITY__RADIAN_PER_SECOND                      0x43  /**< The Low Byte of Angular Velocity (radian per second) UUID.*/
#define GATT_SPEC_UNIT_ANGULAR_ACCELERATION__RADIAN_PER_SECOND_SQUARED          0x44  /**< The Low Byte of Angular Acceleration (radian per second squared) UUID.*/
#define GATT_SPEC_UNIT_HEAT_FLUX_DENSITY__WATT_PER_SQUARE_METRE                 0x45  /**< The Low Byte of Heat Flux Density (watt per square metre) UUID.*/
#define GATT_SPEC_UNIT_HEAT_CAPACITY__JOULE_PER_KELVIN                          0x46  /**< The Low Byte of Heat Capacity (joule per kelvin) UUID.*/
#define GATT_SPEC_UNIT_SPECIFIC_HEAT_CAPACITY__JOULE_PER_KILOGRAM_KELVIN        0x47  /**< The Low Byte of Specific Heat Capacity (joule per kilogram kelvin) UUID.*/
#define GATT_SPEC_UNIT_SPECIFIC_ENERGY__JOULE_PER_KILOGRAM                      0x48  /**< The Low Byte of Specific Energy (joule per kilogram) UUID.*/
#define GATT_SPEC_UNIT_THERMAL_CONDUCTIVITY__WATT_PER_METRE_KELVIN              0x49  /**< The Low Byte of Thermal Conductivity (watt per metre kelvin) UUID.*/
#define GATT_SPEC_UNIT_ENERGY_DENSITY__JOULE_PER_CUBIC_METRE                    0x4A  /**< The Low Byte of Energy Density (joule per cubic metre) UUID.*/
#define GATT_SPEC_UNIT_ELECTRIC_FIELD_STRENGTH__VOLT_PER_METRE                  0x4B  /**< The Low Byte of Electric Field Strength (volt per metre) UUID.*/
#define GATT_SPEC_UNIT_ELECTRIC_CHARGE_DENSITY__COULOMB_PER_CUBIC_METRE         0x4C  /**< The Low Byte of Electric Charge Density (coulomb per cubic metre) UUID.*/
#define GATT_SPEC_UNIT_SURFACE_CHARGE_DENSITY__COULOMB_PER_SQUARE_METRE         0x4D  /**< The Low Byte of Surface Charge Density (coulomb per square metre) UUID.*/
#define GATT_SPEC_UNIT_ELECTRIC_FLUX_DENSITY__COULOMB_PER_SQUARE_METRE          0x4E  /**< The Low Byte of Electric Flux Density (coulomb per square metre) UUID.*/
#define GATT_SPEC_UNIT_PERMITTIVITY__FARAD_PER_METRE                            0x4F  /**< The Low Byte of Permittivity (farad per metre) UUID.*/
#define GATT_SPEC_UNIT_PERMEABILITY__HENRY_PER_METRE                            0x50  /**< The Low Byte of Permeability (henry per metre) UUID.*/
#define GATT_SPEC_UNIT_MOLAR_ENERGY__JOULE_PER_MOLE                             0x51  /**< The Low Byte of Molar Energy (joule per mole) UUID.*/
#define GATT_SPEC_UNIT_MOLAR_ENTROPY__JOULE_PER_MOLE_KELVIN                     0x52  /**< The Low Byte of Molar Entropy (joule per mole kelvin) UUID.*/
#define GATT_SPEC_UNIT_EXPOSURE__COULOMB_PER_KILOGRAM                           0x53  /**< The Low Byte of Exposure (coulomb per kilogram) UUID.*/
#define GATT_SPEC_UNIT_ABSORBED_DOSE_RATE__GRAY_PER_SECOND                      0x54  /**< The Low Byte of Absorbed Dose Rate (gray per second) UUID.*/
#define GATT_SPEC_UNIT_RADIANT_INTENSITY__WATT_PER_STERADIAN                    0x55  /**< The Low Byte of Radiant Intensity (watt per steradian) UUID.*/
#define GATT_SPEC_UNIT_RADIANCE__WATT_PER_SQUARE_METER_STERADIAN                0x56  /**< The Low Byte of Radiance (watt per square metre sterdian) UUID.*/
#define GATT_SPEC_UNIT_CATALYTIC_ACTIVITY_CONCENTRATION__KATAL_PER_CUBIC_METRE  0x57  /**< The Low Byte of Catalytic Activity Concentration (katal per cubic metre) UUID.*/
#define GATT_SPEC_UNIT_TIME__MINUTE                                             0x60  /**< The Low Byte of Time (minute) UUID.*/
#define GATT_SPEC_UNIT_TIME__HOUR                                               0x61  /**< The Low Byte of Time (hour) UUID.*/
#define GATT_SPEC_UNIT_TIME__DAY                                                0x62  /**< The Low Byte of Time (day) UUID.*/
#define GATT_SPEC_UNIT_PLANE_ANGLE__DEGREE                                      0x63  /**< The Low Byte of Plane Angle (degree) UUID.*/
#define GATT_SPEC_UNIT_PLANE_ANGLE__MINUTE                                      0x64  /**< The Low Byte of Plane Angle (minute) UUID.*/
#define GATT_SPEC_UNIT_PLANE_ANGLE__SECOND                                      0x65  /**< The Low Byte of Plane Angle (second) UUID.*/
#define GATT_SPEC_UNIT_AREA__HECTARE                                            0x66  /**< The Low Byte of Area (hectare) UUID.*/
#define GATT_SPEC_UNIT_VOLUME__LITRE                                            0x67  /**< The Low Byte of Volume (litre) UUID.*/
#define GATT_SPEC_UNIT_MASS__TONNE                                              0x68  /**< The Low Byte of Mass (tonne) UUID.*/
#define GATT_SPEC_UNIT_PRESSURE__BAR                                            0x80  /**< The Low Byte of Pressure (bar) UUID.*/
#define GATT_SPEC_UNIT_PRESSURE__MILLIMETRE_OF_MERCURY                          0x81  /**< The Low Byte of Pressure (millimeter of mercury) UUID.*/
#define GATT_SPEC_UNIT_LENGTH__ANGSTROM                                         0x82  /**< The Low Byte of Length (angstrom) UUID.*/
#define GATT_SPEC_UNIT_LENGTH__NAUTICAL_MILE                                    0x83  /**< The Low Byte of Length (nautical mile) UUID.*/
#define GATT_SPEC_UNIT_AREA__BARN                                               0x84  /**< The Low Byte of Area (barn) UUID.*/
#define GATT_SPEC_UNIT_VELOCITY__KNOT                                           0x85  /**< The Low Byte of Velocity (knot) UUID.*/
#define GATT_SPEC_UNIT_LOGARITHMIC_RADIO_QUANTITY__NEPER                        0x86  /**< The Low Byte of Logarithmic Radio Quantity (neper) UUID.*/
#define GATT_SPEC_UNIT_LOGARITHMIC_RADIO_QUANTITY__BEL                          0x87  /**< The Low Byte of Logarithmic Radio Quantity (bel) UUID.*/
#define GATT_SPEC_UNIT_LENGTH__YARD                                             0xA0  /**< The Low Byte of Length (yard) UUID.*/
#define GATT_SPEC_UNIT_LENGTH__PARSEC                                           0xA1  /**< The Low Byte of Length (parsec) UUID.*/
#define GATT_SPEC_UNIT_LENGTH__INCH                                             0xA2  /**< The Low Byte of Length (inch) UUID.*/
#define GATT_SPEC_UNIT_LENGTH__FOOT                                             0xA3  /**< The Low Byte of Length (foot) UUID.*/
#define GATT_SPEC_UNIT_LENGTH__MILE                                             0xA4  /**< The Low Byte of Length (mile) UUID.*/
#define GATT_SPEC_UNIT_PRESSURE__POUND_FORCE_PER_SQUARE_INCH                    0xA5  /**< The Low Byte of Pressure (pound force per square inch) UUID.*/
#define GATT_SPEC_UNIT_VELOCITY__KILOMETRE_PER_HOUR                             0xA6  /**< The Low Byte of Velocity (kilometer per hour) UUID.*/
#define GATT_SPEC_UNIT_VELOCITY__MILE_PER_HOUR                                  0xA7  /**< The Low Byte of Velocity (mile per hour) UUID.*/
#define GATT_SPEC_UNIT_ANGULAR_VELOCITY__REVOLUTION_PER_MINUTE                  0xA8  /**< The Low Byte of Angular Velocity (revolution per minute) UUID.*/
#define GATT_SPEC_UNIT_ENERGY__GRAM_CALORIE                                     0xA9  /**< The Low Byte of Energy (gram calorie) UUID.*/
#define GATT_SPEC_UNIT_ENERGY__KILOGRAM_CALORIE                                 0xAA  /**< The Low Byte of Energy (kilogram calorie) UUID.*/
#define GATT_SPEC_UNIT_ENERGY__KILOWATT_HOUR                                    0xAB  /**< The Low Byte of Energy (kilowatt hour) UUID.*/
#define GATT_SPEC_UNIT_THERMODYNAMIC_TEMPERATURE__DEGREE_FAHRENHEIT             0xAC  /**< The Low Byte of Thermodynamic Temperature (degree fahrenheit) UUID.*/
#define GATT_SPEC_UNIT_PERCENTAGE                                               0xAD  /**< The Low Byte of Percentage UUID.*/
#define GATT_SPEC_UNIT_PER_MILLE                                                0xAE  /**< The Low Byte of Per Mile UUID.*/
#define GATT_SPEC_UNIT_PERIOD__BEATS_PER_MINUTE                                 0xAF  /**< The Low Byte of Period (beats per minute) UUID.*/
#define GATT_SPEC_UNIT_ELECTRIC_CHARGE__AMPERE_HOURS                            0xB0  /**< The Low Byte of Electric Charge (ampere hour) UUID.*/
#define GATT_SPEC_UNIT_MASS_DENSITY__MILLIGRAM_PER_DECILITRE                    0xB1  /**< The Low Byte of Mass Density (milligram per decilitre) UUID.*/
#define GATT_SPEC_UNIT_MASS_DENSITY__MILLIMOLE_PER_LITRE                        0xB2  /**< The Low Byte of Mass Density (milligram per litre) UUID.*/
#define GATT_SPEC_UNIT_TIME__YEAR                                               0xB3  /**< The Low Byte of Time (year) UUID.*/
#define GATT_SPEC_UNIT_TIME__MONTH                                              0xB4  /**< The Low Byte of Time (month) UUID.*/
//************************************************************************************/
/** @} */


#endif // _BLE_UUID_H_

