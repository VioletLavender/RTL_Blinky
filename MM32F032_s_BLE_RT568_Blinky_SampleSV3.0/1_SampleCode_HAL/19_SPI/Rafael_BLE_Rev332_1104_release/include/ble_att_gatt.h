/**************************************************************************//**
* @file       ble_att_gatt.h
* @brief      Provide the Definition of BLE Attributes and Generic Attributes.
*
* @defgroup ble_att BLE Attributes Definition (ATT)
* @{
* @details  Common definitions for the ATT. (ble_att_gatt.h).
* @}
* @defgroup ble_gatt BLE Generic Attributes Definition (GATT)
* @{
* @details  Common definitions for the GATT. (ble_att_gatt.h).
* @}
*****************************************************************************/

#ifndef _BLE_ATT_GATT_H_
#define _BLE_ATT_GATT_H_



/**
 * @defgroup bleCharPropertie BLE GATT Characteristic Properties Definition
 * @{
 * @details  BLE GATT characteristic properties definition.
 * @ingroup ble_gatt
 */
//Bluetooth Spec. Ver5.0, Vol 3, Part G, 3.3.1.1
#define GATT_DECLARATIONS_PROPERTIES_BROADCAST                      0x01  /**< Broadcasting of the value permitted. */
#define GATT_DECLARATIONS_PROPERTIES_READ                           0x02  /**< Reading the value permitted. */
#define GATT_DECLARATIONS_PROPERTIES_WRITE_WITHOUT_RESPONSE         0x04  /**< Writing the value with Write Command permitted. */
#define GATT_DECLARATIONS_PROPERTIES_WRITE                          0x08  /**< Writing the value with Write Request permitted. */
#define GATT_DECLARATIONS_PROPERTIES_NOTIFY                         0x10  /**< Notification of the value permitted. */
#define GATT_DECLARATIONS_PROPERTIES_INDICATE                       0x20  /**< Indications of the value permitted. */
#define GATT_DECLARATIONS_PROPERTIES_AUTHENTICATED_SIGNED_WRITES    0x40  /**< Writing the value with Signed Write Command permitted. */
#define GATT_DECLARATIONS_PROPERTIES_EXTENDED_PROPERTIES            0x80  /**< Addional characteristic properties are defined in the Characteristic extended properties descriptor. */
/** @} */


/**
 * @defgroup bleClientConfig BLE Characteristic Client Configuration Descriptor
 * @{
 * @details  BLE characteristic client configuration descriptor.
 * @ingroup ble_gatt
 */
#define GATT_DESCRIPTORS_CLIENT_CHARACTERISTIC_CONFIGURATION_NOTIFICATION           0x01  /**< Client Configuration Characteristic Notification Descriptor. */
#define GATT_DESCRIPTORS_CLIENT_CHARACTERISTIC_CONFIGURATION_INDICATION             0x02  /**< Client Configuration Characteristic Inidication Descriptor. */
/** @} */


/**
 * @defgroup bleExtendedProperties BLE Characteristic Extended Properties Descriptor
 * @{
 * @details  BLE characteristic extended properties descriptor.
 * @ingroup ble_gatt
 */
#define GATT_DESCRIPTORS_CHARACTERISTIC_EXTENDED_PROPERTIES_RELIABLE_WRITE          0x01  /**< Characteristic Extended Properties Reliable Write Descriptor. */
#define GATT_DESCRIPTORS_CHARACTERISTIC_EXTENDED_PROPERTIES_WRITABLE_AUXILIARIES    0x02  /**< Characteristic Extended Properties Writable Auxilaries Descriptor. */
/** @} */


/**
 * @defgroup bleDescriptorValue BLE Characteristic Descriptor
 * @{
 * @details  BLE characteristic descriptor.
 * @ingroup ble_gatt
 */
#define GATT_DESCRIPTORS_SERVER_CHARACTERISTIC_CONFIGURATION_BROADCASTS_ENABLED     0x01  /**< Server Configuration Characteristic Broadcast Enable Descriptor. */
/** @} */


/**
 * @defgroup bleServerConfig BLE Server Characteristic Configuration Descriptor
 * @{
 * @details  BLE server characteristic configuration descriptor.
 * @ingroup ble_gatt
 */
#define GATT_DESCRIPTORS_SERVER_CHARACTERISTIC_CONFIGURATION_BROADCAST          0x01  /**< Server Configuration Characteristic Broadcast Descriptor. */
/** @} */



/**
 * @defgroup blepresentFormat BLE Characteristic Presentation Format
 * @{
 * @details  BLE characteristic presentation format.
 * @ingroup ble_gatt
 */
#define GATT_CHARACTERISTIC_FORMAT_RFU                              0x00  /**< Reserved For Future Use. */
#define GATT_CHARACTERISTIC_FORMAT_BOOLEAN                          0x01  /**< Boolean. */
#define GATT_CHARACTERISTIC_FORMAT_2BIT                             0x02  /**< Unsigned 2-bit integer. */
#define GATT_CHARACTERISTIC_FORMAT_NIBBLE                           0x03  /**< Unsigned 4-bit integer. */
#define GATT_CHARACTERISTIC_FORMAT_UINT8                            0x04  /**< Unsigned 8-bit integer. */
#define GATT_CHARACTERISTIC_FORMAT_UINT12                           0x05  /**< Unsigned 12-bit integer. */
#define GATT_CHARACTERISTIC_FORMAT_UINT16                           0x06  /**< Unsigned 16-bit integer. */
#define GATT_CHARACTERISTIC_FORMAT_UINT24                           0x07  /**< Unsigned 24-bit integer. */
#define GATT_CHARACTERISTIC_FORMAT_UINT32                           0x08  /**< Unsigned 32-bit integer. */
#define GATT_CHARACTERISTIC_FORMAT_UINT48                           0x09  /**< Unsigned 48-bit integer. */
#define GATT_CHARACTERISTIC_FORMAT_UINT64                           0x0a  /**< Unsigned 64-bit integer. */
#define GATT_CHARACTERISTIC_FORMAT_UINT128                          0x0b  /**< Unsigned 128-bit integer. */
#define GATT_CHARACTERISTIC_FORMAT_SINT8                            0x0c  /**< Signed 2-bit integer. */
#define GATT_CHARACTERISTIC_FORMAT_SINT12                           0x0d  /**< Signed 12-bit integer. */
#define GATT_CHARACTERISTIC_FORMAT_SINT16                           0x0e  /**< Signed 16-bit integer. */
#define GATT_CHARACTERISTIC_FORMAT_SINT24                           0x0f  /**< Signed 24-bit integer. */
#define GATT_CHARACTERISTIC_FORMAT_SINT32                           0x10  /**< Signed 32-bit integer. */
#define GATT_CHARACTERISTIC_FORMAT_SINT48                           0x11  /**< Signed 48-bit integer. */
#define GATT_CHARACTERISTIC_FORMAT_SINT64                           0x12  /**< Signed 64-bit integer. */
#define GATT_CHARACTERISTIC_FORMAT_SINT128                          0x13  /**< Signed 128-bit integer. */
#define GATT_CHARACTERISTIC_FORMAT_FLOAT32                          0x14  /**< IEEE-754 32-bit floating point. */
#define GATT_CHARACTERISTIC_FORMAT_FLOAT64                          0x15  /**< IEEE-754 64-bit floating point. */
#define GATT_CHARACTERISTIC_FORMAT_SFLOAT                           0x16  /**< IEEE-11073 16-bit SFLOAT. */
#define GATT_CHARACTERISTIC_FORMAT_FLOAT                            0x17  /**< IEEE-11073 32-bit FLOAT. */
#define GATT_CHARACTERISTIC_FORMAT_DUINT16                          0x18  /**< IEEE-20601 format. */
#define GATT_CHARACTERISTIC_FORMAT_UTF8S                            0x19  /**< UTF-8 string. */
#define GATT_CHARACTERISTIC_FORMAT_UTF16S                           0x1A  /**< UTF-16 string. */
#define GATT_CHARACTERISTIC_FORMAT_STRUCT                           0x1B  /**< Opaque Structure. */
/** @} */


/**
 * @defgroup bleNameSpace BLE GATT Namespace
 * @{
 * @details  BLE GATT namespace.
 * @ingroup ble_gatt
 */
#define GATT_CHARACTERISTIC_BLUETOOTH_NAMESPACE_BLUETOOTH_SIG       0x01  /**< Bluetooth SIG defined Namespace. */
/** @} */



/**
 * @defgroup blePermission BLE ATT Permission Definition
 * @{
 * @details  BLE ATT permission definition.
 * @ingroup ble_att
 */
//Bluetooth Spec. [Vol 3] page 477,516,535~547 of 656
#define ATT_PERMISSION_READ                                         0x01  /**< The attribute is readable without any constraints. */
#define ATT_PERMISSION_WRITE                                        0x02  /**< The attribute is writable without any constraints. */
#define ATT_PERMISSION_ENC_READ                                     0x04  /**< The attribute is encryption required for the remote read access. */
#define ATT_PERMISSION_ENC_WRITE                                    0x08  /**< The attribute is encryption required for the remote write access. */
#define ATT_PERMISSION_AUTHE_READ                                   0x10  /**< The attribute is Authentication required for the remote read access. */
#define ATT_PERMISSION_AUTHE_WRITE                                  0x20  /**< The attribute is Authentication required for the remote write access. */
#define ATT_PERMISSION_AUTHO_READ                                   0x40  /**< The attribute is Authorization required for the remote read access. */
#define ATT_PERMISSION_AUTHO_WRITE                                  0x80  /**< The attribute is Authorization required for the remote write access. */
#define ATT_PERMISSION_NOTIFY                                       0x01  /**< The attribute allows notification without any constraints. */
#define ATT_PERMISSION_INDICATE                                     0x02  /**< The attribute allows indication without any constraints. */
#define ATT_PERMISSION_ENC_NOTIFY                                   0x04  /**< The attribute is encryption required for the notify access. */
#define ATT_PERMISSION_ENC_INDICATE                                 0x08  /**< The attribute is encryption required for the indicate access. */
#define ATT_PERMISSION_AUTHE_NOTIFY                                 0x10  /**< The attribute is Authentication required for the notify access. */
#define ATT_PERMISSION_AUTHE_INDICATE                               0x20  /**< The attribute is Authentication required for the indicate access. */
/** @} */


/**
 * @defgroup blePermissionTable BLE ATT Permission Table Definition
 * @{
 * @details  BLE ATT permission table definition.
 * @ingroup ble_att
 */
#define TAB_ATTRIBUTE_SERVER_PERMISSION_R_W                         0x00  /**< Indicates the permission of read/ write. */
#define TAB_ATTRIBUTE_SERVER_PERMISSION_ATT_TYPE_FORMAT             0x01  /**< Indicates the permission foramt. */
/** @} */

/**
 * @defgroup bleAttType BLE ATT Type Format Definition
 * @{
 * @details  BLE ATT type format definition.
 * @ingroup ble_att
 */
#define ATT_TYPE_FORMAT_16UUID                                      0x01  /**< 16-bits UUID. */
#define ATT_TYPE_FORMAT_128UUID                                     0x02  /**< 128-bits UUID. */
#define ATT_TYPE_FORMAT_DYN_LLENGTH                                 0x04  /**< Dynamic Length. */
#define ATT_TYPE_FORMAT_ATT_CHR_ACSS_USER                           0x08  /**< Access by User. */
/** @} */


/**
 * @defgroup bleAttErrorCode BLE ATT Error Code Definition
 * @{
 * @details  BLE ATT error code definition.
 * @ingroup ble_att
 */
#define ERR_CODE_ATT_INVALID_HANDLE                                     0x01  /**< Invalid Attribute Handle. */
#define ERR_CODE_ATT_READ_NOT_PERMITTED                                 0x02  /**< Read Not Permitted. */
#define ERR_CODE_ATT_WRITE_NOT_PERMITTED                                0x03  /**< Write Not Permitted. */
#define ERR_CODE_ATT_INVALID_PDU                                        0x04  /**< Invalid PDU. */
#define ERR_CODE_ATT_INSUFFICIENT_AUTHENTICATION                        0x05  /**< Authenticated Link Required. */
#define ERR_CODE_ATT_REQUEST_NOT_SUPPORTED                              0x06  /**< Request Not Supported. */
#define ERR_CODE_ATT_INVALID_OFFSET                                     0x07  /**< Invalid Offset*/
#define ERR_CODE_ATT_INSUFFICIENT_AUTHORIZATION                         0x08  /**< Insufficient Authorization. */
#define ERR_CODE_ATT_PREPARE_QUEUE_FULL                                 0x09  /**< Prepare Queue Full. */
#define ERR_CODE_ATT_ATTRIBUTE_NOT_FOUND                                0x0A  /**< Attribute Not Found. */
#define ERR_CODE_ATT_ATTRIBUTE_NOT_LONG                                 0x0B  /**< Attribute Cannot br Read or Written Using Read/Write blob Requests. */
#define ERR_CODE_ATT_INSUFFICIENT_ENCRYPTION_KEY_SIZE                   0x0C  /**< Encryption Key Size is Insufficient. */
#define ERR_CODE_ATT_INVALID_ATTRIBUTE_VALUE_LENGTH                     0x0D  /**< Invalid Value Length. */
#define ERR_CODE_ATT_UNLIKELY_ERROR                                     0x0E  /**< Unlikely Error. */
#define ERR_CODE_ATT_INSUFFICIENT_ENCRYPTION                            0x0F  /**< Encrypted Link Required. */
#define ERR_CODE_ATT_UNSUPPORTED_GROUP_TYPE                             0x10  /**< Attribute Type is Not a Supported Grouping Attribute. */
#define ERR_CODE_ATT_INSUFFICIENT_RESOURCES                             0x11  /**< Encrypted Link Required. */
#define ERR_CODE_ATT_RESERVED                                           0x12  /**< Reserved for Future Use. */
#define ERR_CODE_ATT_APPLICATION_ERROR                                  0x80  /**< ATT Application Error. */
#define ERR_CODE_ATT_CLIENT_CHAR_CONFIG_DESCRTR_IMPROPERLY_CONFIGURED   0xFD  /**< Client Characteristic Configuration Descriptor improperly configured. */
#define ERR_CODE_ATT_PROCEDURE_ALREADY_IN_PROGRESS                      0xFE  /**< Procedure Already in Progress. */
#define ERR_CODE_ATT_OUT_OF_RANGE                                       0xFF  /**< Out Of Range. */
/** @} */



/**
 * @defgroup bleAttOpCode BLE ATT Operation Code Definition
 * @{
 * @details  BLE ATT operation code definition.
 * @ingroup ble_att
 */
#define OPCODE_ATT_ERROR_RESPONSE                                  0X01     /**< Error Response. */
#define OPCODE_ATT_EXCHANGE_MTU_REQUEST                            0X02     /**< Exchange MTU Request. */
#define OPCODE_ATT_EXCHANGE_MTU_RESPONSE                           0X03     /**< Exchange MTU Response. */
#define OPCODE_ATT_FIND_INFORMATION_REQUEST                        0X04     /**< Find Information Request. */
#define OPCODE_ATT_FIND_INFORMATION_RESPONSE                       0X05     /**< Find Information Response. */
#define OPCODE_ATT_FIND_BY_TYPE_VAlUE_REQUEST                      0X06     /**< Find by Type Value Request. */
#define OPCODE_ATT_FIND_BY_TYPE_VAlUE_RESPONSE                     0X07     /**< Find by Type Value Response. */
#define OPCODE_ATT_READ_BY_TYPE_REQUEST                            0X08     /**< Read by Type Request. */
#define OPCODE_ATT_READ_BY_TYPE_RESPONSE                           0X09     /**< Read by Type Response. */
#define OPCODE_ATT_READ_REQUEST                                    0X0A     /**< Read Request. */
#define OPCODE_ATT_READ_RESPONSE                                   0X0B     /**< Read Response. */
#define OPCODE_ATT_READ_BlOB_REQUEST                               0X0C     /**< Read Blob Request. */
#define OPCODE_ATT_READ_BlOB_RESPONSE                              0X0D     /**< Read Blob Response. */
#define OPCODE_ATT_READ_MUlTIPlE_REQUEST                           0X0E     /**< Read Multiple Request. */
#define OPCODE_ATT_READ_MUlTIPlE_RESPONSE                          0X0F     /**< Read Multiple Response. */
#define OPCODE_ATT_READ_BY_GROUP_TYPE_REQUEST                      0X10     /**< Read by Group Type Request. */
#define OPCODE_ATT_READ_BY_GROUP_TYPE_RESPONSE                     0X11     /**< Read by Group Type Response. */
#define OPCODE_ATT_WRITE_REQUEST                                   0X12     /**< Write Request. */
#define OPCODE_ATT_WRITE_RESPONSE                                  0X13     /**< Write Response. */
#define OPCODE_ATT_WRITE_COMMAND                                   0X52     /**< Write Command. */
#define OPCODE_ATT_PREPARE_WRITE_REQUEST                           0X16     /**< Prepare Write Request. */
#define OPCODE_ATT_PREPARE_WRITE_RESPONSE                          0X17     /**< Prepare Write Response. */
#define OPCODE_ATT_EXECUTE_WRITE_REQUEST                           0X18     /**< Execute Write Request. */
#define OPCODE_ATT_EXECUTE_WRITE_RESPONSE                          0X19     /**< Execute Write Response. */
#define OPCODE_ATT_HANDlE_VAlUE_NOTIFICATION                       0X1B     /**< Handle Value Notification. */
#define OPCODE_ATT_HANDlE_VAlUE_INDICATION                         0X1D     /**< Handle Value Indication. */
#define OPCODE_ATT_HANDlE_VAlUE_CONFIRMATION                       0X1E     /**< Handle Value Confirmation. */
#define OPCODE_ATT_SIGNED_WRITE_COMMAND                            0XD2     /**< Signed Write Command. */
/** @} */




/**
 * @defgroup bleAttSize BLE Pre-defined ATT Value Length
 * @{
 * @details  BLE pre-defined ATT value length.
 * @ingroup ble_att
 */
#define SIZE_OF_ATTRIBUTE_VALUE_0                  0
#define SIZE_OF_ATTRIBUTE_VALUE_1                  1
#define SIZE_OF_ATTRIBUTE_VALUE_2                  2
#define SIZE_OF_ATTRIBUTE_VALUE_3                  3
#define SIZE_OF_ATTRIBUTE_VALUE_4                  4
#define SIZE_OF_ATTRIBUTE_VALUE_5                  5
#define SIZE_OF_ATTRIBUTE_VALUE_6                  6
#define SIZE_OF_ATTRIBUTE_VALUE_7                  7
#define SIZE_OF_ATTRIBUTE_VALUE_8                  8
#define SIZE_OF_ATTRIBUTE_VALUE_9                  9
#define SIZE_OF_ATTRIBUTE_VALUE_10                 10
#define SIZE_OF_ATTRIBUTE_VALUE_11                 11
#define SIZE_OF_ATTRIBUTE_VALUE_12                 12
#define SIZE_OF_ATTRIBUTE_VALUE_13                 13
#define SIZE_OF_ATTRIBUTE_VALUE_14                 14
#define SIZE_OF_ATTRIBUTE_VALUE_15                 15
#define SIZE_OF_ATTRIBUTE_VALUE_16                 16
#define SIZE_OF_ATTRIBUTE_VALUE_17                 17
#define SIZE_OF_ATTRIBUTE_VALUE_18                 18
#define SIZE_OF_ATTRIBUTE_VALUE_19                 19
#define SIZE_OF_ATTRIBUTE_VALUE_20                 20
#define SIZE_OF_ATTRIBUTE_VALUE_21                 21
#define SIZE_OF_ATTRIBUTE_VALUE_22                 22
#define SIZE_OF_ATTRIBUTE_VALUE_23                 23
#define SIZE_OF_ATTRIBUTE_VALUE_24                 24
#define SIZE_OF_ATTRIBUTE_VALUE_25                 25
#define SIZE_OF_ATTRIBUTE_VALUE_26                 26
#define SIZE_OF_ATTRIBUTE_VALUE_27                 27
#define SIZE_OF_ATTRIBUTE_VALUE_28                 28
#define SIZE_OF_ATTRIBUTE_VALUE_29                 29
#define SIZE_OF_ATTRIBUTE_VALUE_45                 45
#define SIZE_OF_ATTRIBUTE_VALUE_46                 46
#define SIZE_OF_ATTRIBUTE_VALUE_124                124
#define SIZE_OF_ATTRIBUTE_VALUE_125                125
#define SIZE_OF_ATTRIBUTE_VALUE_244                244
/** @} */

#endif // _BLE_ATT_GATT_H_

