/**************************************************************************//**
* @file       ble_cmd.h
* @brief      This file contains the functions of HOST to HCI interface.
*
*
* @defgroup ble_common BLE Common
* @{
* @details This file shows the common BLE definitions and functions. (ble_cmd.h, ble_event.h)
* @}
*****************************************************************************/

#ifndef _BLE_CMD_H_
#define _BLE_CMD_H_

#include "host.h"
#include "blestack_status.h"

/**************************************************************************
 * Definitions
 **************************************************************************/
/** @defgroup ble_cmd_definition ble_cmd Definition
 * @{
 * @details Here shows the definitions in ble_cmd.h.
 * @ingroup ble_common
 * @}
 * @defgroup adv_definition BLE Advertising Definition
 * @{
 * @details Here shows the advertising related definition.
 * @ingroup ble_cmd_definition
 * @}
 * @defgroup scan_definition BLE Scan Definition
 * @{
 * @ingroup ble_cmd_definition
 * @details Here shows the scan related definition.
 * @}
 * @defgroup conn_definition BLE Connection Parameter Definition
 * @{
 * @ingroup ble_cmd_definition
 * @details Here shows the connection parameter related definition.
 * @}
 **************************************************************************/

/**
 * @ingroup ble_cmd_definition
 * @defgroup ble_cmd_cmdEvent BLE Command Event Definition
 * @{
 * @details BLE Command event definition.
 * @name BleCmdEvent
 * @brief Define the different events that could be received by the BleEventCallBack.
 * @{
 */
typedef Uint8 BleCmdEvent;

/** Null event.*/
#define BLECMD_EVENT_NULL                                 0x00

/** Event indicates the advertisement is enabled or disabled.
 * @note The parameter field indicates the advertisement information though the
 * @ref BLE_Event_ADV_Complete_Param
*/
#define BLECMD_EVENT_ADV_COMPLETE                         0x01


/** Event indicates that receives the advertisement packets.*/
#define BLECMD_EVENT_SCAN_REPORT                          0x02

/** Event indicates the BLE connection is established.
 * @note The parameter field indicates the connection information though the
 * @ref BLE_Event_ConnParam Type
*/
#define BLECMD_EVENT_CONN_COMPLETE                        0x03

/** Event indicates the BLE connection has been canceled.
 * @note The parameter field is always NULL
*/
#define BLECMD_EVENT_CONN_CANCEL_COMPLETE                 0x04


/** Event indicates the Connection Parameter Update Response can be retrieved.
 * @note The parameter field indicates the connection information though the
 * @ref ble_cmd_connUpdateStatus Type \n
 * The event can be ignored, just wait for @ref BLECMD_EVENT_CONN_UPDATE_COMPLETE event
*/
#define BLECMD_EVENT_CONN_PARAMETER_UPDATE_RSP            0x05


/** Event indicates the BLE connection has been updated.
 * @note The parameter field indicates the connection information though the
 * @ref BLE_Event_ConnUpdateParam Type
*/
#define BLECMD_EVENT_CONN_UPDATE_COMPLETE                 0x06

/** Event indicates the BLE connection has been disconnected.
 * @note The parameter field indicates the connection information though the
 * @ref BLE_Event_DisconnParam Type
*/
#define BLECMD_EVENT_DISCONN_COMPLETE                     0x07

/** Event indicates that the Link Layer PHY update procedure is complete.
 * @note The parameter field indicates the connection information though the
 * @ref BLE_Event_Phy_Update_Param Type
*/
#define BLECMD_EVENT_PHY_UPDATE_COMPLETE                  0x08

/** Event indicates the current Link Layer PHY can be retrieved.
 * @note The parameter field indicates the connection information though the
 * @ref BLE_Event_Phy_Param Type
*/
#define BLECMD_EVENT_PHY_READ_COMPLETE                    0x09

/** Event indicates the RSSI value can be retrieved.
 * @note The parameter field indicates the connection information though the
 * @ref BLE_Event_Rssi_Param Type
*/
#define BLECMD_EVENT_READ_RSSI_COMPLETE                   0x0A


/** Event indicates the Security STK Generator Method can be retrieved.
 * @note The parameter field indicates the connection information though the
 * @ref BLE_Event_Stk_Gen_Method Type
*/
#define BLECMD_EVENT_STK_GEN_METHOD                       0x0B


/** Event indicates the PassKey Entry can be retrieved.
 * @note The parameter field indicates the connection information though the
 * @ref BLE_Event_PassKey_Confirm Type
*/
#define BLECMD_EVENT_PASSKEY_CONFIRM                      0x0C
/** @} */
/** @} */


/** Event indicates the Authentication status can be retrieved.
 * @note The parameter field indicates the connection information though the
 * @ref BLE_Event_Auth_Status Type
*/
#define BLECMD_EVENT_AUTH_STATUS                          0x0D
/** @} */
/** @} */

/** Event indicates the scan is enabled or disabled.
 * @note The parameter field is always NULL
*/
#define BLECMD_EVENT_SCAN_COMPLETE                        0x0E
/** @} */
/** @} */

/** Event indicates the create connection.
 * @note The parameter field indicates the connection information though the
 * @ref BLE_Event_CreateConn_Param Type
*/
#define BLECMD_EVENT_CREATE_CONNECTION                    0x0F
/** @} */
/** @} */

/** Event indicates the MTU Size can be retrieved.
 * @note The parameter field indicates the MTU Size though the
 * @ref BLE_Event_Mtu Type
*/
#define BLECMD_EVENT_EXCHANGE_MTU_SIZE                    0x10
/** @} */
/** @} */

/** Event indicates the Data Length Update can be retrieved.
 * @note The parameter field indicates the data length update though the
 * @ref BLE_Event_Data_Length_Param Type
*/
#define BLECMD_EVENT_DATA_LENGTH_UPDATE                   0x11
/** @} */
/** @} */

/**
 * @ingroup ble_cmd_definition
 * @defgroup ble_cmd_disconnReason BLE Disconnect Reason Definition
 * @{
 * @details BLE disconnect reason definition.
 * @name BleDisConnReason
 * @brief Define the BLE disconnect reason.
 * @{
 */
typedef Uint8 BleDisConnReason;
#define BLEDISCONNREASON_AUTHENTICATION_FAILURE                                       (0x05u) /**< Authentication failure. */
#define BLEDISCONNREASON_PIN_OR_KEY_MISSING                                           (0x06u) /**< Encryption key is missing or not saved on the remote device. */
#define BLEDISCONNREASON_CONNECTION_TIMEOUT                                           (0x08u) /**< Connection timeout. */
#define BLEDISCONNREASON_REMOTE_USER_TERMINATED_CONNECTION                            (0x13u) /**< Disconnection requested by the remote device users. */
#define BLEDISCONNREASON_REMOTE_DEVICE_TERMINATED_CONNECTION_DUE_TO_LOW_RESOURCES     (0x14u) /**< Disconnection by the remote device due to low resources. */
#define BLEDISCONNREASON_REMOTE_DEVICE_TERMINATED_CONNECTION_DUE_TO_POWER_OFF         (0x15u) /**< Disconnection by the remote device due to power off. */
#define BLEDISCONNREASON_CONNECTION_TERMINATED_BY_LOCAL_HOST                          (0x16u) /**< Disconnection by the local device Host stack. */
#define BLEDISCONNREASON_UNSUPPORTED_REMOTE_FEATURE_UNSUPPORTED_LMP_FEATURE           (0x1Au) /**< Unsupported Remote Feature. */
#define BLEDISCONNREASON_LMP_RESPONSE_TIMEOUT_LL_RESPONSE_TIMEOUT                     (0x22u) /**< LMP Response Timeout. */
#define BLEDISCONNREASON_PAIRING_WITH_UNIT_KEY_NOT_SUPPORTED                          (0x29u) /**< Pairing with Unit Key Unsupported. */
#define BLEDISCONNREASON_UNACCEPTABLE_CONNECTION_INTERVAL                             (0x3Bu) /**< Connection Interval Unacceptable. */
#define BLEDISCONNREASON_CONNECTION_TERMINATED_DUE_TO_MIC_FAILURE                     (0x3Du) /**< Connection Terminated due to MIC Failure. */
#define BLEDISCONNREASON_CONNECTION_FAILED_TO_BE_ESTABLISHED                          (0x3Eu) /**< Connection Failed to be Established. */
/** @} */
/** @} */




/**
 * @ingroup ble_cmd_definition
 * @defgroup ble_cmd_connRole BLE Connection Role Definition
 * @{
 * @details BLE connection role.
 * @name BleConnRole
 * @brief Define BLE connection role.
 * @{
 */
typedef Uint8 BleConnRole;
#define BLEROLE_MASTER                      (0x00u) /**< Master role. */
#define BLEROLE_SLAVE                       (0x01u) /**< Slave role. */
/** @} */
/** @} */


/**
 * @defgroup ble_cmd_advChannelMap BLE Advertisement Channel Definition
 * @{
 * @ingroup adv_definition
 * @details BLE advertisement channel definition.
 * @name BleAdvChannelMap
 * @brief The definition for advertising channel.
 * @{
 */
typedef Uint8 BleAdvChannelMap;

/** Channel 37 */
#define ADV_CHANNEL_37         0x01

/** Channel 38 */
#define ADV_CHANNEL_38         0x02

/** Channel 39 */
#define ADV_CHANNEL_39         0x04

/** Channel 37 and Channel 38 */
#define ADV_CHANNEL_37_38      (ADV_CHANNEL_37 | ADV_CHANNEL_38)

/** Channel 37 and Channel 39 */
#define ADV_CHANNEL_37_39      (ADV_CHANNEL_37 | ADV_CHANNEL_39)

/** Channel 38 and Channel 39 */
#define ADV_CHANNEL_38_39      (ADV_CHANNEL_38 | ADV_CHANNEL_39)

/** Channel 37 and Channel 38 and channel 39 */
#define ADV_CHANNEL_ALL        (ADV_CHANNEL_37 | ADV_CHANNEL_38 | ADV_CHANNEL_39)
/** @} */
/** @} */


/**
 * @defgroup ble_cmd_advFilter BLE Advertisement Filter Policy Definition
 * @{
 * @ingroup adv_definition
 * @details BLE advertisement filter policy definition.
 * @name BleAdvFilterPolicy
 * @brief The definition for advertising filter policy.
 * @{
 */
typedef Uint8 BleAdvFilterPolicy;

/** Filter scan requests and connect requests from any device */
#define ADV_FILTER_POLICY_ACCEPT_ALL               0x00

/** Filter scan requests with whitelist,connect requests with from any . */
#define ADV_FILTER_POLICY_ACCEPT_SCAN_REQ_WL       0x01

/** Filter scan requests from any, connect requests with whitelist. */
#define ADV_FILTER_POLICY_ACCEPT_CONN_REQ_WL       0x02

/** Filter both scan and connect requests with whitelist. */
#define ADV_FILTER_POLICY_ACCEPT_SCAN_CONN_REQ_WL  0x03
/** @} */
/** @} */



/**
 * @defgroup ble_cmd_advInterval BLE Advertising Interval Minimum and Maximum Definition
 * @{
 * @ingroup adv_definition
 * @details BLE advertising Interval range definition.
 * @name BleAdvInterval
 * @brief  BLE Advertising Interval Minimum and Maximum.
 * @{
 */
typedef Uint8 BleAdvIntervalRange;

/** The unit of advertising interval is 0.625ms \n
 * Minimum advertising interval is 20ms = (0x20 * 0.625) ms
*/
#define ADV_INTERVAL_MIN            0x0020

/** The unit of advertising interval is 0.625ms \n
 * Maximum advertising interval is 10.24s = (0x4000 * 0.625) ms
*/
#define ADV_INTERVAL_MAX            0x4000
/** @} */
/** @} */


/**
 * @defgroup ble_cmd_advType BLE Advertising Type Definition
 * @{
 * @ingroup adv_definition
 * @details BLE advertising type definition.
 * @name BleAdvType
 * @brief  Advertising type for LE set advertising parameter.
 * @{
 */
typedef Uint8 BleAdvType;

#define ADV_TYPE_ADV_IND            0x00    /**< Connectable and scannable undirected advertising. */
#define ADV_TYPE_ADV_DIRECT_IND     0x01    /**< Connectable directed advertising. */
#define ADV_TYPE_SCAN_IND           0x02    /**< Scanable undirected advertising. */
#define ADV_TYPE_ADV_NONCONN_IND    0x03    /**< Non-Connectable undirected advertising. */
#define ADV_TYPE_SCAN_RSP           0x04    /**< Scan Response. */
/** @} */
/** @} */



/**
 * @defgroup ble_cmd_scanType BLE Scan Type Definition
 * @{
 * @ingroup scan_definition
 * @details BLE scan type definition.
 * @name BleScanType
 * @brief  Scan type for LE set scan request.
 * @{
 */
typedef Uint8 BleScanType;
#define SCAN_TYPE_PASSIVE           0x00 /**< Passive scan type. */
#define SCAN_TYPE_ACTIVE            0x01 /**< Active scan type. */
/** @} */
/** @} */



/**
 * @defgroup ble_cmd_scanInterval BLE Scan Interval Minimum and Maximum Definition
 * @{
 * @ingroup scan_definition
 * @details BLE scan Interval range definition.
 * @name BleScanInterval
 * @brief  BLE Scan Interval Minimum and Maximum.
 * @{
 */
typedef Uint8 BleScanIntervalRange;

/** The unit of scan interval is 0.625ms \n
 * Minimum scan interval is 2.5ms = (0x0004 * 0.625) ms
*/
#define SCAN_INTERVAL_MIN            0x0004

/** The unit of scan interval is 0.625ms \n
 * Maximum scan interval is 10.24s = (0x4000 * 0.625) ms
*/
#define SCAN_INTERVAL_MAX            0x4000
/** @} */
/** @} */


/**
 * @defgroup ble_cmd_scanWindow BLE Scan Window Minimum and Maximum Definition
 * @{
 * @ingroup scan_definition
 * @details BLE scan window range definition.
 * @name BleScanWindow
 * @brief  BLE Scan Window Minimum and Maximum.
 * @{
 */
typedef Uint8 BleScanWindowRange;

/** The unit of scan window is 0.625ms \n
 * Minimum scan window is 2.5ms = (0x0004 * 0.625) ms
*/
#define SCAN_WINDOW_MIN            0x0004

/** The unit of scan window is 0.625ms \n
 * Maximum scan window is 10.24s = (0x4000 * 0.625) ms
*/
#define SCAN_WINDOW_MAX            0x4000
/** @} */
/** @} */


/**
 * @defgroup ble_cmd_scanFilterPolicy BLE Scan Filter Policy Definition
 * @{
 * @ingroup scan_definition
 * @details BLE scan filter policy definition.
 * @name BleScanFilterPolicy
 * @brief  The definition for scan filter policy.
 * @{
 */
typedef Uint8 BleScanFilterPolicy;

/** Filter advertisement packets except directed advertising and scan response from any device */
#define SCAN_FILTER_POLICY_ACCEPT_ALL               0x00

/** Filter advertisement from the device which is in the white list . */
#define ADV_FILTER_POLICY_ACCEPT_WL                 0x01
/** @} */
/** @} */


/**
 * @defgroup ble_cmd_connInterval BLE Connection Interval Definition
 * @{
 * @ingroup conn_definition
 * @details BLE connection interval range definition.
 * @name BleConnInt
 * @brief  BLE Connection Interval Range Definition.
 * @{
 */
typedef Uint8 BleConnectionIntervalRange;

/** The unit of connection interval is 1.25ms \n
 * Minimum connection interval is 7.5ms = (0x0006 * 1.25) ms
*/
#define CONN_INTERVAL_MIN            0x0006

/** The unit of connection interval is 1.25ms \n
 * Maximum connection interval is 4s = (0x0C80 * 1.25) ms
*/
#define CONN_INTERVAL_MAX            0x0C80
/** @} */
/** @} */


/**
 * @defgroup ble_cmd_connLatency BLE Connection Latency Definition
 * @{
 * @ingroup conn_definition
 * @details BLE connection latency range definition.
 * @name BleConnLatency
 * @brief  BLE Connection Latency Range Definition.
 * @{
 */
typedef Uint8 BleConnectionLatencyRange;

/** Minimum connection latency is 0 */
#define CONN_Latency_MIN            0

/** Maximum connection latency is 0x01F3 */
#define CONN_Latency_MAX            0x01F3
/** @} */
/** @} */


/**
 * @defgroup ble_cmd_connSupervisionTimeout BLE Connection Supervision Timeout Range Definition
 * @{
 * @ingroup conn_definition
 * @details BLE connection supervision timeout range definition.
 * @name BleConnSupTimeout
 * @brief  BLE Connection Supervision Timeout Range Definition.
 * @{
 */
typedef Uint8 BleConnectionSupTimeoutRange;

/** The unit of connection supervision timeout is 10ms \n
 * Minimum connection supervision timeout is 100ms = (0x000A * 10) ms
*/
#define CONN_SUBTIMEOUT_MIN            0x000A

/** The unit of connection supervision timeout is 10ms \n
 * Maximum connection supervision timeout is 32s = (0x0C80 * 10) ms
*/
#define CONN_SUBTIMEOUT_MAX            0x0C80
/** @} */
/** @} */


/**
 * @ingroup ble_cmd_definition
 * @defgroup ble_cmd_addrType BLE Address Type Definition
 * @{
 * @details BLE address type.
 * @name BleAddrType
 * @brief Define different BLE address type.
 * @{
 */
typedef Uint8 BleAddrType;
#define PUBLIC_ADDR                     0x00  /**< Public BLE address.  */
#define RANDOM_ADDR                     0x01  /**< Random BLE address.  */
#define PUBLIC_IDENTITY_ADDR            0x02  /**< Corresponds to Resloved Private Address. */
#define RANDOM_IDENTITY_ADDR            0x03  /**< Corresponds to Resloved Private Address. */

/** @} */
/** @} */


/**
 * @ingroup ble_cmd_definition
 * @defgroup ble_cmd_blePhy BLE PHY Definition
 * @{
 * @details BLE PHY definition.
 * @name BlePhy
 * @brief  The definition of BLE PHY.
 * @{
 */
typedef Uint8 BlePhy;
#define BLE_PHY_1M         0x01   /**< 1Mbps  */
#define BLE_PHY_2M         0x02   /**< 2Mbps  */
/** @} */
/** @} */



/**
 * @ingroup ble_cmd_definition
 * @defgroup ble_cmd_bleState BLE State Definition
 * @{
 * @details BLE State Definition.
 * @name BleState
 * @brief  The definition of BLE State.
 * @{
 */
typedef Uint8 BleMode;
#define STATE_BLE_STANDBY           0x00    /**< Stanby mode.        */
#define STATE_BLE_ADVERTISING       0x01    /**< Advertising mode.   */
#define STATE_BLE_CONNECTION        0x02    /**< Connection mode.    */
#define STATE_BLE_SCANNING          0x03    /**< Scanning mode.      */
#define STATE_BLE_INITIATING        0x04    /**< Initialating mode.  */
/** @} */
/** @} */



/**
 * @ingroup ble_cmd_definition
 * @defgroup ble_cmd_STK_GenMethod BLE STK Generator Method Definition
 * @{
 * @details BLE STK Generator Method definition.
 * @name BleSTK_GenMethod
 * @brief  The definition of BLE STK Generator Method.
 * @{
 */
typedef Uint8 BleSTK_GenMethod;
#define PASSKEY_ENTRY            0x01  /**< Entry only.   */
#define PASSKEY_DISPLAY          0x02  /**< Display only. */
/** @} */
/** @} */


/**
 * @ingroup ble_cmd_definition
 * @defgroup ble_cmd_connUpdateStatus BLE Connect Update Status Definition
 * @{
 * @details BLE Connect Update Status definition.
 * @name BleConnUpdateStatus
 * @brief  The definition of BLE Connect Update Status.
 * @{
 */
typedef Uint8 BleConnUpdateStatus;
#define CMD_SUCCESS              0x00  /**< Success.  */
#define CMD_REJECTED             0x01  /**< Rejected. */
#define CMD_TIMEOUT              0x02  /**< Timeout.  */
/** @} */
/** @} */


/**
 * @ingroup ble_cmd_definition
 * @defgroup ble_cmd_Auth_Status BLE Authentication Status Definition
 * @{
 * @details BLE Authentication Status definition.
 * @name BleAuthenticationStatus
 * @brief  The definition of BLE Authentication Status.
 * @{
 */
typedef Uint8 BleAuthStatus;
#define AUTH_SUCCESS                                    0x00  /**< Success.  */
#define AUTH_ERR_PASSKEY_ENTRY_FAILED                   0x01  /**< Passkey entry failed.  */
#define AUTH_ERR_OOB_NOT_AVAILABLE                      0x02  /**< Out of Band Key not available.  */
#define AUTH_ERR_AUTHENTICATION_REQUIREMENTS            0x03  /**< Error authentication requirement.  */
#define AUTH_ERR_CONFIRM_VALUE_FAILED                   0x04  /**< Confirm value failed.  */
#define AUTH_ERR_PAIRING_NOT_SUPPORTED                  0x05  /**< Pairing not supported.  */
#define AUTH_ERR_ENCRYPTION_KEY_SIZE                    0x06  /**< Encryption key size.  */
#define AUTH_ERR_COMMAND_NOT_SUPPORTED                  0x07  /**< Command not supported.  */
#define AUTH_ERR_UNSPECIFIED_REASON                     0x08  /**< Unspecified reason.  */
#define AUTH_ERR_REPEATED_ATTEMPTS                      0x09  /**< Too little time has elapsed since last attempt.  */
#define AUTH_ERR_INVALID_PARAMETERS                     0x0A  /**< Invalid parameters.  */
#define AUTH_ERR_DHKEY_CHECK_FAILED                     0x0B  /**< DHKey check failed.  */
#define AUTH_ERR_NUMERIC_COMPARISON_FAILED              0x0C  /**< Numeric comparison failed.  */
#define AUTH_ERR_BR_EDR_PAIRING_IN_PROGRESS             0x0D  /**< BR/EDR paring in progress.  */
#define AUTH_ERR_CROSS_TRANS_KEY_GEN_NOT_ALLOWED        0x0E  /**< BR/EDR Link Key cannot be used for LE keys.  */
#define AUTH_ERR_TIMEOUT                                0x0F  /**< Procedure time out.  */
#define AUTH_ERR_PIN_OR_KEY_MISSING                     0x10  /**< Bonding PIN / KEY Missing.  */
/** @} */
/** @} */



/**
 * @ingroup ble_cmd_definition
 * @defgroup ble_cmd_PhyUpdateStatus BLE Phy Update Status Definition
 * @{
 * @details BLE Phy Update Status definition.
 * @name BlePhyUpdateStatus
 * @brief  The definition of BLE Phy Update Status.
 * @{
 */
typedef Uint8 BlePhyUpdateStatus;
#define PHY_UPDATE_SUCCESS                         0x00  /**< Success.  */
#define PHY_UPDATE_REMOTE_FEATURE_UNSUPPORTED      0x1A  /**< Command Cancel Due to Remote feature Unsupported.  */
#define PHY_UDDATE_TRANSACTION_COLLISION           0x2A  /**< Command Cancel Due to Transaction Collision. */

/** @} */
/** @} */



/**
 * @ingroup ble_cmd_definition
 * @defgroup ble_cmd_Adv_Complete_Event_Status BLE ADV Complete Event Status Definition
 * @{
 * @details BLE ADV Complete Event Status definition.
 * @name BleAdvCompleteStatus
 * @brief  The definition of BLE ADV Complete Event Status.
 * @{
 */
typedef Uint8 BleAdvCompleteStatus;
#define ADV_ENABLE_CMD_SUCCESS                   0x00  /**< Success.  */
#define CMD_REJECTED_DUE_TO_LIMITED_RESOURCES    0x0D  /**< Rejected. */

/** @} */
/** @} */



/**
 * @ingroup ble_cmd_definition
 * @defgroup ble_cmd_IOCaps BLE IO Capabilities Definition
 * @{
 * @details BLE IO Capabilities definition.
 * @name BleIOCaps
 * @brief  The definition of BLE IO Capabilities.
 * @{
 */
typedef Uint8 IOCaps;
#define DISPLAY_ONLY                      0x00   /**< Display Only.              */
#define DISPLAY_YESNO                     0x01   /**< Display and Yes/No Entry.  */
#define KEYBOARD_ONLY                     0x02   /**< Keyboard Only.             */
#define NOINPUT_NOOUTPUT                  0x03   /**< No IO Capabilities.        */
#define KEYBOARD_DISPLAY                  0x04   /**< Keyboard and Display.      */
/** @} */
/** @} */


/**
 * @ingroup ble_cmd_definition
 * @defgroup ble_cmd_BondingFlags BLE Bonding Flags Definition
 * @{
 * @details BLE Bonding Flags definition.
 * @name BondingFlags
 * @brief  The definition of BLE Bonding Flags.
 * @{
 */
typedef Uint8 BondingFlags;
#define NO_BONDING                        0x00   /**< NO Bonding.  */
#define BONDING                           0x01   /**< Bonding.     */
/** @} */
/** @} */

/**
 * @ingroup ble_cmd_definition
 * @defgroup ble_cmd_Conn_status BLE Connection Event Status Definition
 * @{
 * @details BLE Connection Event Status definition.
 * @name BleConnStatus
 * @brief  The definition of BLE Connection Event Status.
 * @{
 */
typedef Uint8 BleConnStatus;
#define COMMAND_SUCCESS                              0x00  /**< Success.  */
#define COMMAND_DISALLOWED                           0x0C  /**< Disallowed. */
#define COMMAND_REJECTED_DUE_TO_LIMITED_RESOURCES    0x0D  /**< Rejected. */
#define COMMAND_ADVERTISING_TIMEOUT                  0x3C  /**< Advertising Timeout. */

/** @} */
/** @} */
/**************************************************************************
 * Structures
 **************************************************************************/
/** @defgroup ble_cmd_structure ble_cmd Structure
 * @{
 * @details Here shows the structures in ble_cmd.h.
 * @ingroup ble_common
 * @}
 **************************************************************************/

/**
 * @brief BLE address.
 * @ingroup ble_cmd_structure
*/
typedef struct BLE_Addr_Param
{

    /**
     * @ref ble_cmd_addrType "BLE address type"
     */
    BleAddrType addrType;

    /** BLE address 48-bit in LSB */
    Uint8 addr[SIZE_BLE_ADDR];

} BLE_Addr_Param;




/** @brief BLE advertising parameter.
 * @ingroup ble_cmd_structure
*/
typedef struct BLE_Adv_Param
{
    /**
     * @ref ble_cmd_advType "BLE advertising type"
     */
    BleAdvType Adv_Type;

    /** Minimum advertising interval. \n
      *  @note Advertising interval = value * 0.625ms \n
      *        Advertising interval Range = @ref ADV_INTERVAL_MIN ~ @ref ADV_INTERVAL_MAX
      */
    Uint16 Adv_Interval_Min;

    /** Maximum advertising  interval.
      *  @note Advertising interval = value * 0.625ms \n
      *        Advertising interval Range = @ref ADV_INTERVAL_MIN ~ @ref ADV_INTERVAL_MAX
      */
    Uint16 Adv_Interval_Max;


    /** The BLE address and BLE address type for directed advertising */
    BLE_Addr_Param Adv_DirectAddr_Param;

    /**
     * @ref ble_cmd_advChannelMap "Advertising channel map"
     */
    BleAdvChannelMap    Adv_Channel_Map;

    /**
     * @ref ble_cmd_advFilter "Advertising filter policy"
     */
    BleAdvFilterPolicy  Adv_Filter_Policy;
} BLE_Adv_Param;



/** @brief BLE scan parameter.
 * @ingroup ble_cmd_structure
*/
typedef struct BLE_Scan_Param
{

    /**
     * @ref ble_cmd_scanType "BLE scan type"
     */
    BleScanType           Scan_Type;

    /** Scan interval.
      *  @note Scan interval = value * 0.625ms \n
      *        Scan interval Range = 2.5ms ~ 10.24s
      */
    Uint16                Scan_Interval;

    /** Scan window.
      *  @note Scan window = value * 0.625ms \n
      *        Scan window Range = 2.5ms ~ 10.24s
      */
    Uint16                Scan_Window;

    /**
     * @ref ble_cmd_scanFilterPolicy "BLE scan filter policy"
     */
    BleScanFilterPolicy   Scan_FilterPolicy;

} BLE_Scan_Param;



/** @brief BLE connection parameter.
 * @ingroup ble_cmd_structure
*/
typedef struct BLE_Conn_Param
{

    /** Minimum connection interval.
      *  @note Connection interval = value * 1.25ms \n
      *        Connection interval Range = 7.5ms ~ 4s
      */
    Uint16 Conn_IntervalMin;

    /** Maximum connection interval.
    *  @note Connection interval = value * 1.25ms \n
    *        Connection interval Range = 7.5ms ~ 4s
    */
    Uint16 Conn_IntervalMax;

    /** Slave latency for the connection in number of connection event.
     *  @note Latency Range = 0x0000 to 0x01F3
     */
    Uint16 Conn_Latency;

    /** Connection supervision timeout.
     *  @note Supervision timeout = value * 10ms \n
     *        Supervision timeout Range = 100ms ~ 32s
     */
    Uint16 Conn_SvisionTimeout;

} BLE_Conn_Param;


/** @brief BLE connection update parameter.
 * @ingroup ble_cmd_structure
*/
typedef struct BLE_ConnUpdate_Param
{
    Uint8           connId;           /**< Connection id. */
    BLE_Conn_Param  connUpdateParam;  /**< Connection parameter. */

} BLE_ConnUpdate_Param;


/** @brief BLE IO Capabilities.
 * @ingroup ble_cmd_structure
*/
typedef struct BLE_IO_Caps
{
    IOCaps          IO_Param;         /**< IO Capabilities. */

} BLE_IO_Caps;




/** @brief BLE Bonding Flags.
 * @ingroup ble_cmd_structure
*/
typedef struct BLE_Bonding_Flags
{
    BondingFlags         Flags;         /**< Bonding Flags. */

} BLE_Bonding_Flags;




/** @brief BLE Data Length.
 * @ingroup ble_cmd_structure
*/
typedef struct BLE_DataLength_Param
{
    uint8_t           connId;           /**< Connection id. */
    uint16_t          TxMaxOctets;      /**< Maximum number of Tx payload octets. */
    uint16_t          RxMaxOctets;      /**< Maximum number of Rx payload octets. */

} BLE_DataLength_Param;


/**
 * @brief BLE Phy Parameters.
 * @ingroup ble_cmd_structure
*/
typedef struct BLE_Phy_Param
{
    BlePhy             tx_Phy;   /**< Tx Phy : @ref BLE_PHY_1M or @ref BLE_PHY_2M */
    BlePhy             rx_Phy;   /**< Rx Phy : @ref BLE_PHY_1M or @ref BLE_PHY_2M  */

} BLE_Phy_Param;




/**
 * @brief BLE Event Connection Parameter.
 * @ingroup ble_cmd_structure
*/
typedef struct BLE_Event_ADV_Complete_Param
{
    BleAdvCompleteStatus  status;               /**< @ref ble_cmd_Adv_Complete_Event_Status "Event Status." */

} BLE_Event_ADV_Complete_Param;




/**
 * @brief BLE Event Connection Parameter.
 * @ingroup ble_cmd_structure
*/
typedef struct BLE_Event_ConnParam
{
    BleConnStatus      status;               /**< Connection status. */
    Uint8              connId;               /**< Connection id. */
    BleConnRole        connRole;             /**< @ref ble_cmd_connRole "Connection role" */
    BLE_Addr_Param     peerAddr;             /**< Remote device address type and address. */
    Uint16             connInterval;         /**< The current link connection interval value. */
    Uint16             connLatency;          /**< The current link connection latency value. */
    Uint16             supervisionTimeout;   /**< The current link connection supervision timeout value. */

} BLE_Event_ConnParam;


/**
 * @brief BLE Event Disconnection Parameter.
 * @ingroup ble_cmd_structure
*/
typedef struct BLE_Event_DisconnParam
{
    Uint8              connId;               /**< Connection id. */
    BleDisConnReason   disconnectReason;     /**< @ref ble_cmd_disconnReason "Disconnection reason". */

} BLE_Event_DisconnParam;




/**
 * @brief BLE Event Connection Update Parameter.
 * @ingroup ble_cmd_structure
*/
typedef struct BLE_Event_ConnUpdateParam
{
    Uint8               connId;               /**< Connection id. */
    BleConnUpdateStatus status;               /**< @ref ble_cmd_connUpdateStatus "Event Status." */
    Uint16              connInterval;         /**< The current link connection interval value. */
    Uint16              connLatency;          /**< The current link connection latency value. */
    Uint16              supervisionTimeout;   /**< The current link connection supervision timeout value. */

} BLE_Event_ConnUpdateParam;




/** @brief BLE RSSI value.
 * @ingroup ble_cmd_structure
*/
typedef struct BLE_Event_Rssi_Param
{
    Uint8 connID; /**< Connection id. */
    Sint8 rssi;   /**< RSSI value. */
} BLE_Event_Rssi_Param;





/**
 * @brief BLE Event STK Generator Method.
 * @ingroup ble_cmd_structure
*/
typedef struct BLE_Event_Stk_Gen_Method
{
    Uint8              connId;          /**< Connection id. */
    BleSTK_GenMethod   Stk_Gen_Method;  /**< @ref ble_cmd_STK_GenMethod "STK generate method." */

} BLE_Event_Stk_Gen_Method;




/**
 * @brief BLE Event PassKey Confirm.
 * @ingroup ble_cmd_structure
*/
typedef struct BLE_Event_PassKey_Confirm
{
    Uint8              connId;   /**< Connection id. */
} BLE_Event_PassKey_Confirm;




/**
 * @brief BLE Event Authentication Status.
 * @ingroup ble_cmd_structure
*/
typedef struct BLE_Event_Auth_Status
{
    Uint8              connId;   /**< Connection id. */
    BleAuthStatus      status;   /**< @ref ble_cmd_Auth_Status "Event Status." */

} BLE_Event_Auth_Status;


/**
 * @brief BLE Event Phy Parameters.
 * @ingroup ble_cmd_structure
*/
typedef struct BLE_Event_Phy_Param
{
    Uint8              connId;   /**< Connection id. */
    BlePhy             tx_Phy;   /**< Tx Phy. */
    BlePhy             rx_Phy;   /**< Rx Phy. */

} BLE_Event_Phy_Param;


/**
 * @brief BLE Event Phy Update Complete Parameters.
 * @ingroup ble_cmd_structure
*/
typedef struct BLE_Event_Phy_Update_Param
{
    BlePhyUpdateStatus     status;    /**< Phy update status. */
    BLE_Event_Phy_Param    phyParam;  /**< Phy parameters. */

} BLE_Event_Phy_Update_Param;



/**
 * @brief BLE Event Scan Report.
 * @ingroup ble_cmd_structure
*/
typedef struct BLE_Event_Scan_Report
{
    BleAdvType      Rpt_Type;                 /**< @ref ble_cmd_addrType "BLE advertising type". */
    BLE_Addr_Param  Rpt_peerAddr;             /**< Remote device address type and address. */
    Uint8           Rpt_DataLength;           /**< Received advertising or scan response data length. */
    Uint8           Rpt_Data[31];             /**< Received advertising or scan response data. */
    Sint8           Rpt_Rssi;                 /**< Received Signal Strength Indication in dBm. */

} BLE_Event_Scan_Report;



/**
 * @brief BLE Event Create Connection Status.
 * @ingroup ble_cmd_structure
*/
typedef struct BLE_Event_CreateConn_Param
{
    BleConnStatus      status;  /**< Connection status. */

} BLE_Event_CreateConn_Param;



/**
 * @brief BLE Event MTU Size value.
 * @ingroup ble_cmd_structure
*/
typedef struct BLE_Event_Mtu
{
    uint8_t      mtu_size;

} BLE_Event_Mtu;



/**
 * @brief BLE Event Data Length Update.
 * @ingroup ble_cmd_structure
*/
typedef struct BLE_Event_Data_Length_Param
{
    BLE_DataLength_Param    param;

} BLE_Event_Data_Length_Param;
/**************************************************************************
 * Functions
 **************************************************************************/
/** @defgroup ble_cmd_function ble_cmd Function
 * @{
 * @details Here shows the function in ble_cmd.h.
 * @ingroup ble_common
 * @}
 **************************************************************************/


/** Set BLE Device Address and Device Address Type
 *
 *  This function is used to set BLE Local Device Address and Device Address Type.
 *
 * @ingroup ble_cmd_function
 * @param[in] addrParam : Local Device BLE Address and BLE Address Type \n
 * @attention MUST to set BLE Device Address and Address Type after @ref RF_Init \n
 *
 * @attention  If the address changes, MUST re-init @ref setBLE_AdvParam "advertisement parameter" or @ref setBLE_ScanParam "scan parameter" or 
 *            @ref setBLE_ConnCreate "create connection" to use the updated address.
 * 
 * @note      BLE Address Type : MUST to be either @ref PUBLIC_ADDR or @ref RANDOM_ADDR \n
 *            If BLE Address Type is set to @ref RANDOM_ADDR (static random address), \n
 *            the two most significant bits of the address shall be equal to 1. \n
 *
 * @note      BLE Address : Little Endian and the length is @ref SIZE_BLE_ADDR \n
 *            If Device BLE Address is set to "01:02:03:04:05:06", addrParam->addr[0] = 0x06
 *
 * @retval BLESTACK_STATUS_ERR_INVALID_STATE : ONLY can set during stand by state.
 * @retval BLESTACK_STATUS_ERR_INVALID_PARAM : Invalid BLE Address type.
 * @retval BLESTACK_STATUS_SUCCESS  : Setting completed.
 */
BleStackStatus setBLE_BleDeviceAddr(BLE_Addr_Param *addrParam);


/** Get BLE Device Address and Device Address Type
 *
 *  This function is used to get BLE Local Device Address and Device Address Type.
 *
 * @ingroup ble_cmd_function
 * @param[out] addrParam : pointer to Local Device BLE Address and BLE Address Type. \n
 * @note       BLE Address Type : @ref PUBLIC_ADDR or @ref RANDOM_ADDR \n
 *             BLE Address : Little Endian and the length is @ref SIZE_BLE_ADDR \n
 *             If Device BLE Address is set to "01:02:03:04:05:06", addrParam->addr[0] = 0x06
 *
 * @return none
 */
void getBLE_BleDeviceAddr(BLE_Addr_Param *addrParam);


/** Set BLE White List
 *
 * @ingroup ble_cmd_function
 * @note    The function is not turned on yet!
 *
 * @return none
 */
void setBLE_WhiteList(void);


/** Set BLE Advertising Parameter
*
* @ingroup ble_cmd_function
* @param[in] advParam : Advertising parameter
*
* @note    Adv Type : Must be @ref ADV_TYPE_ADV_IND, @ref ADV_TYPE_ADV_DIRECT_IND, @ref ADV_TYPE_SCAN_IND or @ref ADV_TYPE_ADV_NONCONN_IND \n
*          MUST to set Adv_DirectAddr_Param if Adv_Type is set to @ref ADV_TYPE_ADV_DIRECT_IND.
* @note    Adv Interval Min, Adv Interval Max : @ref ADV_INTERVAL_MIN to @ref ADV_INTERVAL_MAX \n
*          Adv Interval Min must less than Adv Interval Max
* @note    Adv Channel Map : @ref ble_cmd_advChannelMap
*
* @retval BLESTACK_STATUS_ERR_BUSY : Message queue buffer full.
* @retval BLESTACK_STATUS_ERR_INVALID_STATE : ONLY can set during stand by state.
* @retval BLESTACK_STATUS_ERR_INVALID_PARAM : Invalid parameter.
* @retval BLESTACK_STATUS_SUCCESS  : Setting Completed.
*/
BleStackStatus setBLE_AdvParam(BLE_Adv_Param advParam);


/** Set BLE Advertising Data
*
*
* @ingroup ble_cmd_function
* @param[in] advData : pointer to advertising data.
* @param[in] length : The ADV_Data length.
*
* @note    The ADV_Data length should less than 32 bytes (BLE Advertising Data Payload = 31 bytes, added 1 bytes to the Length)
*
* @retval BLESTACK_STATUS_ERR_BUSY : Message queue buffer full.
* @retval BLESTACK_STATUS_ERR_INVALID_PARAM : Invalid parameter.
* @retval BLESTACK_STATUS_SUCCESS  : Setting Completed.
*/
BleStackStatus setBLE_AdvData(Uint8 *advData, Uint8 length);


/** Set BLE Scan Response Data
*
*
* @ingroup ble_cmd_function
* @param[in] scanRspData : pointer to scan response data.
* @param[in] length : The SCAN_RspData length.
*
* @note    The SCAN_RspData length should less than 32 bytes (BLE Scan Response Data Payload = 31 bytes, added 1 bytes to the Length)
*
* @retval BLESTACK_STATUS_ERR_BUSY : Message queue buffer full.
* @retval BLESTACK_STATUS_ERR_INVALID_PARAM : Invalid parameter.
* @retval BLESTACK_STATUS_SUCCESS  : Setting Completed.
*/
BleStackStatus setBLE_ScanRspData(Uint8 *scanRspData, Uint8 length);



/** Set BLE Start Advertising
 *
 * @ingroup ble_cmd_function
 * @note    Wait for @ref BLECMD_EVENT_ADV_COMPLETE event  to indicates the advertisement has been started
 *
 * @retval BLESTACK_STATUS_ERR_BUSY : Message queue buffer full.
 * @retval BLESTACK_STATUS_ERR_INVALID_STATE : ONLY can set during stand by state.
 * @retval BLESTACK_STATUS_SUCCESS  : Setting Completed.
 */
BleStackStatus setBLE_AdvEnable(void);



/** Set BLE Stop Advertising
 *
 * @ingroup ble_cmd_function
 * @note    Wait for @ref BLECMD_EVENT_ADV_COMPLETE event to indicates the advertisement has been stopped
 *
 * @retval BLESTACK_STATUS_ERR_BUSY : Message queue buffer full.
 * @retval BLESTACK_STATUS_ERR_INVALID_STATE : ONLY can set during advertising.
 * @retval BLESTACK_STATUS_SUCCESS  : Setting Completed.
 */
BleStackStatus setBLE_AdvDisable(void);


/** Set BLE Scanning Parameter
 *
 *
 * @param[in] Scan_Param : Scanning parameter.
 *
 * @note    Scan Type : Must be @ref SCAN_TYPE_PASSIVE or @ref SCAN_TYPE_ACTIVE.
 * @note    Scan Interval : @ref SCAN_INTERVAL_MIN - @ref SCAN_INTERVAL_MAX\n
 * @note    Scan Window : @ref SCAN_WINDOW_MIN - @ref SCAN_WINDOW_MAX\n
 *
 * @retval BLESTACK_STATUS_ERR_BUSY : Message queue buffer full.
 * @retval BLESTACK_STATUS_ERR_INVALID_STATE : CANNOT set during advertising, scanning and initiating.
 * @retval BLESTACK_STATUS_ERR_INVALID_PARAM : Invalid parameter.
 * @retval BLESTACK_STATUS_SUCCESS  : Setting Completed.
 */
BleStackStatus setBLE_ScanParam(BLE_Scan_Param scanParam);



/** Set BLE Start Scanning
 *
 * @ingroup ble_cmd_function
 * @param[in] scanFilter : Scan filter.
 * @note    Wait for @ref BLECMD_EVENT_SCAN_REPORT event to receive scanned devices information
 *
 * @retval BLESTACK_STATUS_ERR_BUSY : Message queue buffer full.
 * @retval BLESTACK_STATUS_ERR_INVALID_STATE : ONLY can set during stand by state.
 * @retval BLESTACK_STATUS_SUCCESS  : Setting Completed.
 */
BleStackStatus setBLE_ScanEnable(BleScanFilterPolicy scanFilter);



/** Set BLE Stop Scanning
 *
 * @ingroup ble_cmd_function
 *
 * @retval BLESTACK_STATUS_ERR_BUSY : Message queue buffer full.
 * @retval BLESTACK_STATUS_ERR_INVALID_STATE : ONLY can set during scanning.
 * @retval BLESTACK_STATUS_SUCCESS  : Setting Completed.
 */
BleStackStatus setBLE_ScanDisable(void);



/** Set BLE Terminate the Connection
 *
 * @ingroup ble_cmd_function
 * @param[in] connID : The link's connection id.
 * @note    Wait for @ref BLECMD_EVENT_DISCONN_COMPLETE event to indicates the connection has been disconnected.
 *
 * @retval BLESTACK_STATUS_ERR_BUSY : Message queue buffer full.
 * @retval BLESTACK_STATUS_ERR_INVALID_CONNID : Error connection id.
 * @retval BLESTACK_STATUS_SUCCESS  : Setting Completed.
 */
BleStackStatus setBLE_Disconnect(Uint8 connID);



/** Set BLE Create Connection
 *
 * @ingroup ble_cmd_function
 * @param[in] peerAddrParam : The target peer device address and address type.
 * @param[in] scanParam     : Scan parameter.
 * @param[in] connParam     : Connection parameter.
 *
 * @note    Wait for @ref BLECMD_EVENT_CONN_COMPLETE event to indicates the connection has been created.
 *
 * @retval BLESTACK_STATUS_ERR_BUSY : Message queue buffer full.
 * @retval BLESTACK_STATUS_ERR_INVALID_STATE : ONLY can set during stand by state.
 * @retval BLESTACK_STATUS_SUCCESS  : Setting Completed.
 */
BleStackStatus setBLE_ConnCreate(BLE_Addr_Param peerAddrParam, BLE_Scan_Param scanParam,BLE_Conn_Param connParam);



/** Set BLE Cancel the On-going Connection Procedure
 *
 * @ingroup ble_cmd_function
 * @param[in] connID : The link's connection id.
 * @note    Wait for @ref BLECMD_EVENT_CONN_CANCEL_COMPLETE event to indicates the connection has been canceled.
 *
 * @retval BLESTACK_STATUS_ERR_BUSY : Message queue buffer full.
 * @retval BLESTACK_STATUS_ERR_INVALID_STATE : ONLY can set during initial connection.
 * @retval BLESTACK_STATUS_SUCCESS  : Setting Completed.
 */
BleStackStatus setBLE_ConnCancel(void);



/** BLE Update Connection Parameter
 *
 * @ingroup ble_cmd_function
 *
 * @param[in] connParam : Connection parameter.
 *
 * @note    Wait for @ref BLECMD_EVENT_CONN_UPDATE_COMPLETE event to indicates the connection parameter has been updated.
 * @note    Conn Interval Min/Max : @ref CONN_INTERVAL_MIN - @ref CONN_INTERVAL_MAX
 * @note    Conn Latency : @ref CONN_Latency_MIN - @ref CONN_Latency_MAX
 * @note    Conn Svision Timeout : @ref CONN_SUBTIMEOUT_MIN - @ref CONN_SUBTIMEOUT_MAX \n
 *          The Svision Timeout in milliseconds shall be larger than (1 + Conn_Latency) * CONN_INTERVAL_MAX * 2 \n
 *          , where CONN_INTERVAL_MAX is given in milliseconds.
 *
 * @retval BLESTACK_STATUS_ERR_BUSY : Message queue buffer full.
 * @retval BLESTACK_STATUS_ERR_INVALID_PARAM : Invalid parameter.
 * @retval BLESTACK_STATUS_ERR_TIMER_BUSY : The connection Parameter Update timer is busy. Meaning there are still outstanding connection parameter update events.
 * @retval BLESTACK_STATUS_ERR_INVALID_CONNID : Error connection id.
 * @retval BLESTACK_STATUS_SUCCESS  : Setting Completed.
 */
BleStackStatus setBLE_ConnUpdate(BLE_ConnUpdate_Param connParam);



/** Set BLE TX Power
 *
 * @ingroup ble_cmd_function
 *
 * @param[in] txPowerReg : Requested TX power level in dBm. \n
 *                         The TX power register settings please refer to "rf_txpower_definition.h".
 * @param[in] bleMode : BLE Mode to set TX power. \n
 *                 - @ref STATE_BLE_ADVERTISING :  Set advertising transmit power.
 *                 - @ref STATE_BLE_SCANNING    :  Set scan transmit power.
 *                 - @ref STATE_BLE_INITIATING  :  Set initiator and connection transmit power.
 *
 * @note Default TX power is +8dBm.
 * @note Set advertising transmit power MUST before enable advertising.
 * @note Set scan transmit power MUST before enable scan.
 * @note Set initiator and connection transmit power MUST before creating connection.
 * @note The setting will be ignored if setting transmit power during the connection.
 *
 *
 * @retval BLESTACK_STATUS_ERR_INVALID_STATE : ONLY can set during stand by state.
 * @retval BLESTACK_STATUS_ERR_INVALID_PARAM  : Invalid parameter.
 * @retval BLESTACK_STATUS_SUCCESS  : Setting Completed.
 */
BleStackStatus setBLE_TxPower(uint8_t *txPowerReg, BleMode bleMode);


/** Set BLE TX/ RX PHY
 * @ingroup ble_cmd_function
 *
 * @param[in] connID : The link's connection id.
 * @param[in] BLE_Phy_Param : Prefered TX PHY/ RX PHY ( @ref BLE_PHY_1M or @ref BLE_PHY_2M)
 * @note      Wait for @ref BLECMD_EVENT_PHY_UPDATE_COMPLETE event
 * @note      The setting of TX PHY must equal to RX PHY.
 *
 * @retval BLESTACK_STATUS_ERR_INVALID_CONNID : Error connection id.
 * @retval BLESTACK_STATUS_ERR_INVALID_PARAM  : Invalid parameter(s).
 * @retval BLESTACK_STATUS_ERR_NOT_SUPPORTED  : Command not supported.
 * @retval BLESTACK_STATUS_ERR_BUSY : Message queue buffer full.
 * @retval BLESTACK_STATUS_SUCCESS  : Setting Completed.
 */
BleStackStatus setBLE_Phy(Uint8 connID, BLE_Phy_Param blePhy);



/** Get BLE TX/ RX PHY
 * @ingroup ble_cmd_function
 *
 * @param[in] connID : The link's connection id.
 *
 * @note Wait for @ref BLECMD_EVENT_PHY_READ_COMPLETE event to get TX/ RX PHY @ref BLE_Event_Phy_Param
 *
 * @retval BLESTACK_STATUS_ERR_INVALID_CONNID : Error connection id.
 * @retval BLESTACK_STATUS_ERR_BUSY : Message queue buffer full.
 * @retval BLESTACK_STATUS_SUCCESS  : Setting Completed.
 */
BleStackStatus getBLE_Phy(Uint8 connID);




/** Get BLE RSSI Value
 *
 * @ingroup ble_cmd_function
 * @param[in] connID : The link's connection id.
 * @note Wait for @ref BLECMD_EVENT_READ_RSSI_COMPLETE event to get last RSSI value @ref BLE_Event_Rssi_Param
 *
 * @retval BLESTACK_STATUS_ERR_INVALID_STATE : ONLY can set during scanning and connection.
 * @retval BLESTACK_STATUS_ERR_BUSY : Message queue buffer full.
 * @retval BLESTACK_STATUS_SUCCESS  : Setting Completed.
 */
BleStackStatus getBLE_RssiValue(Uint8 connID);




/** BLE Security Request
 *
 * @ingroup ble_cmd_function
 * @param[in] connID     : The link's connection id.
 *
 * @note Wait for @ref BLECMD_EVENT_AUTH_STATUS event indicates the Authentication status can be retrieved. \n
 *       The parameter field indicates the connection information though the @ref BLE_Event_Auth_Status Type
 *
 * @retval BLESTACK_STATUS_ERR_BUSY           : Message queue buffer full.
 * @retval BLESTACK_STATUS_ERR_TIMER_BUSY     : The Security command timer is busy. Meaning there are still unfinished security events.
 * @retval BLESTACK_STATUS_ERR_INVALID_CONNID : Error connection id.
 * @retval BLESTACK_STATUS_ERR_NOT_SUPPORTED  : Command Not Supported.
 * @retval BLESTACK_STATUS_SUCCESS            : Setting Completed.
 */
BleStackStatus setBLE_SecurityRequest(Uint8 connID);




/** Set BLE Pairing PassKey Value
 *
 * @ingroup ble_cmd_function
 * @param[in] connID     : The link's connection id.
 * @param[in] hexPasskey : Passkey for 6-digit code.
 * @note Set PassKey value to pairing.
 *
 * @retval BLESTACK_STATUS_ERR_BUSY           : Message queue buffer full.
 * @retval BLESTACK_STATUS_ERR_INVALID_CONNID : Error connection id.
 * @retval BLESTACK_STATUS_ERR_NOT_SUPPORTED  : Command Not Supported.
 * @retval BLESTACK_STATUS_SUCCESS            : Setting Completed.
 */
BleStackStatus setBLE_Pairing_PassKey(Uint8 connID, Uint32 hexPasskey);




/** Set BLE IO Capabilities
 *
 * @ingroup ble_cmd_function
 *
 * @param[in] BLE_IO_Caps     : IO Capabilities.
 * @note      IO Capabilities : setting to @ref DISPLAY_ONLY or @ref DISPLAY_YESNO or @ref KEYBOARD_ONLY or @ref NOINPUT_NOOUTPUT or @ref KEYBOARD_DISPLAY \n
 *
 * @retval BLESTACK_STATUS_ERR_INVALID_STATE : CANNOT set BLE bonding flag during creating connection.
 * @retval BLESTACK_STATUS_ERR_INVALID_PARAM : Invalid parameter.
 * @retval BLESTACK_STATUS_SUCCESS  : Setting Completed.
 */
BleStackStatus setBLE_IOCapabilities(BLE_IO_Caps *Param);




/** Set BLE Bonding Flags
 *
 * @ingroup ble_cmd_function
 *
 * @param[in] BLE_Bonding_Flags   : Bonding Flags.
 * @note      BLE Bonding Flags   : setting to @ref NO_BONDING or @ref BONDING \n
 *
 * @retval BLESTACK_STATUS_ERR_INVALID_STATE : CANNOT set BLE bonding flag during creating connection.
 * @retval BLESTACK_STATUS_SUCCESS  : Setting Completed.
 */
BleStackStatus setBLE_BondingFlags(BLE_Bonding_Flags *Param);




/** Set BLE Data Length Update
 *
 * @ingroup ble_cmd_function
 *
 * @param[in] BLE_DataLength_Param : Data Length Parameters, the range of data length is 27~251 bytes.
 * @note Wait for @ref BLECMD_EVENT_DATA_LENGTH_UPDATE event to get updated connection data length @ref BLE_Event_Data_Length_Param
 *
 * @retval BLESTACK_STATUS_ERR_BUSY : Message queue buffer full.
 * @retval BLESTACK_STATUS_ERR_INVALID_CONNID : Error connection id.
 * @retval BLESTACK_STATUS_ERR_INVALID_PARAM : Invalid parameter.
 * @retval BLESTACK_STATUS_SUCCESS  : Setting Completed.
 */
BleStackStatus setBLE_DataLength(BLE_DataLength_Param *Param);




/** Set BLE Exchange MTU Request
 *
 * @ingroup ble_cmd_function
 *
 * @param[in] connID        : The link's connection id.
 * @param[in] client_rx_mtu : Client RX MTU size.
 *
 * @retval BLESTACK_STATUS_ERR_BUSY           : Message queue buffer full.
 * @retval BLESTACK_STATUS_ERR_INVALID_STATE  : ONLY can set during connection.
 * @retval BLESTACK_STATUS_ERR_INVALID_ROLE   : Invalid Role.
 * @retval BLESTACK_STATUS_SUCCESS            : Setting Completed.
 */
BleStackStatus setBLE_ExchangeMtuRequest(uint8_t connID, uint8_t client_rx_mtu);

#endif // _BLE_CMD_H_



