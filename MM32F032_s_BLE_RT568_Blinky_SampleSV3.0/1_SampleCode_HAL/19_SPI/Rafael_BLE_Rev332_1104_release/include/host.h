/**************************************************************************//**
* @file       host.h
* @brief      Provide the declarations that for BLE Host subsystem needed.
*
*
* @defgroup host BLE Host
* @{
* @details Provide the declarations that for BLE Host subsystem needed. (host.h, ble_serviceDef.h)
* @}
*****************************************************************************/

#ifndef _BLE_HOST_H_
#define _BLE_HOST_H_

#include "BleAppSetting.h"
#include "ble_basicType.h"
#include "ble_gapDef.h"
#include "ble_uuid.h"
#include "ble_att_gatt.h"
#include "ble_serviceDef.h"
#include "blestack_status.h"

/** @ingroup host
 * @{
 */
#define _SMP_ON_                    /**< Enable encryption if defined. */
//#define _BOND_ON_                   /**< Enable bonding if defined.    */
/** @} */


/**
 * @defgroup ble_advSize BLE Advertisement Related Size Definition
 * @{
 * @ingroup host
 * @details BLE advertisement related size definition.
 */
#define SIZE_OF_CODE_BLE_ADV_DATA_AD_TYPE_SERVICE       4   /**< Size of advertisement type service. */
#define SIZE_OF_CODE_BLE_ADV_DATA_AD_TYPE_APPEARANCE    4   /**< Size of advertisement type appearance. */
#define SIZE_OF_CODE_BLE_ADV_DATA_FLAGS                 3   /**< Size of advertisement data flags. */
/** @} */



/**
 * @defgroup ble_attSize BLE Profile (Bond) Attribute Size Definition
 * @{
 * @ingroup host
 * @details BLE profile (Bond) attribute size definition.
 * @}
 * @defgroup ble_attProfileSize BLE Profile Attribute Size Definition
 * @{
 * @details BLE profile attribute size definition.
 * @ingroup ble_attSize
 */

/** Null attribute size. */
#define SIZE_ATTRIBUTE_NULL                             1


/**
 * @defgroup ble_attGapSize GAP Attribute Size.
 * @{
 */
#define _GAP_HDL_OFFSET_BASE_                           0x01
#define SIZE_ATTRIBUTE_GAP                              11
/** @} */


/**
 * @defgroup ble_attGattSize GATT Attribute Size.
 * @{
 */
#define _GATT_HDL_OFFSET_BASE_                          (_GAP_HDL_OFFSET_BASE_+SIZE_ATTRIBUTE_GAP)
#define SIZE_ATTRIBUTE_GATT                             4
/** @} */


/**
 * @defgroup ble_attDISSize Device Information Service Attribute Size.
 * @{
 * @details Device information service attribute size.
 */
#define _DIS_HDL_OFFSET_BASE_                           (_GATT_HDL_OFFSET_BASE_+SIZE_ATTRIBUTE_GATT)
#define SIZE_ATTRIBUTE_DIS                              23
/** @} */


/**
 * @defgroup ble_attHRPSize Heart Rate Profile Attribute Size.
 * @{
 * @details Heart Rate Profile attribute size.
 */
#ifdef _PROFILE_HRP_
#define _HRS_HDL_OFFSET_BASE_                           (_DIS_HDL_OFFSET_BASE_+SIZE_ATTRIBUTE_DIS)
#define SIZE_ATTRIBUTE_HRS                              6
#else
#define _HRS_HDL_OFFSET_BASE_                           (_DIS_HDL_OFFSET_BASE_+SIZE_ATTRIBUTE_DIS)
#define SIZE_ATTRIBUTE_HRS                              0
#endif
/** @} */


/**
 * @defgroup ble_attBASSize Battery Service Attribute Size.
 * @{
 * @details Battery Service attribute size.
 */
#ifdef _PROFILE_BAS_
#define _BAS_HDL_OFFSET_BASE_                           (_HRS_HDL_OFFSET_BASE_+SIZE_ATTRIBUTE_HRS)
#define SIZE_ATTRIBUTE_BAS                              6
#else
#define _BAS_HDL_OFFSET_BASE_                           (_HRS_HDL_OFFSET_BASE_+SIZE_ATTRIBUTE_HRS)
#define SIZE_ATTRIBUTE_BAS                              0
#endif
/** @} */



/**
 * @defgroup ble_attHOGPSize Human Interface Over GATT Protocol Service Attribute Size.
 * @{
 * @details Human Interface Over GATT Protocol Service attribute size.
 */
#ifdef _PROFILE_HOGP_
#ifndef _PROFILE_HOGP_EMC_
#define SIZE_ATTRIBUTE_HIDS_BASE_                       7
#ifdef _PROFILE_HOGP_COMSUMER_
#define SIZE_ATTRIBUTE_HIDS_COMSUMER_                   4
#else
#define SIZE_ATTRIBUTE_HIDS_COMSUMER_                   0
#endif
#ifdef _PROFILE_HOGP_MOUSE_
#define SIZE_ATTRIBUTE_HIDS_MOUSE_                      8
#else
#define SIZE_ATTRIBUTE_HIDS_MOUSE_                      0
#endif
#ifdef _PROFILE_HOGP_KEYBOARD_
#define SIZE_ATTRIBUTE_HIDS_KEYBOARD_                   7
#else
#define SIZE_ATTRIBUTE_HIDS_KEYBOARD_                   0
#endif
#define SIZE_ATTRIBUTE_HIDS_RPT_REF                     2

#ifdef _PROFILE_HOGP_BOOT_
#ifdef _PROFILE_HOGP_KEYBOARD_
#define SIZE_ATTRIBUTE_HIDS_BOOT_KB                     5
#else
#define SIZE_ATTRIBUTE_HIDS_BOOT_KB                     0
#endif
#ifdef _PROFILE_HOGP_MOUSE_
#define SIZE_ATTRIBUTE_HIDS_BOOT_MS                     3
#else
#define SIZE_ATTRIBUTE_HIDS_BOOT_MS                     0
#endif
#else
#define SIZE_ATTRIBUTE_HIDS_BOOT_KB                     0
#define SIZE_ATTRIBUTE_HIDS_BOOT_MS                     0
#endif
#define SIZE_ATTRIBUTE_HIDS_81_                         0
#define SIZE_ATTRIBUTE_HIDS_82_                         0
#define SIZE_ATTRIBUTE_HIDS_5D_                         0
#define SIZE_ATTRIBUTE_HIDS_05_                         0
#define SIZE_ATTRIBUTE_HIDS_08_                         0
#define SIZE_ATTRIBUTE_HIDS_09_                         0
#else       //#ifndef _PROFILE_HOGP_EMC_
#define SIZE_ATTRIBUTE_HIDS_BASE_                       7
#ifdef _PROFILE_HOGP_MOUSE_
#define SIZE_ATTRIBUTE_HIDS_MOUSE_                      8
#else
//#define SIZE_ATTRIBUTE_HIDS_MOUSE_                      0
#endif
#ifdef _PROFILE_HOGP_KEYBOARD_
#define SIZE_ATTRIBUTE_HIDS_KEYBOARD_                   7
#else
#define SIZE_ATTRIBUTE_HIDS_KEYBOARD_                   0
#endif
#ifdef _PROFILE_HOGP_BOOT_
#ifdef _PROFILE_HOGP_KEYBOARD_
#define SIZE_ATTRIBUTE_HIDS_BOOT_KB                     5
#else
#define SIZE_ATTRIBUTE_HIDS_BOOT_KB                     0
#endif
#ifdef _PROFILE_HOGP_MOUSE_
#define SIZE_ATTRIBUTE_HIDS_BOOT_MS                     3
#else
#define SIZE_ATTRIBUTE_HIDS_BOOT_MS                     0
#endif
#else
#define SIZE_ATTRIBUTE_HIDS_BOOT_KB                     0
#define SIZE_ATTRIBUTE_HIDS_BOOT_MS                     0
#endif
#define SIZE_ATTRIBUTE_HIDS_COMSUMER_                   0
#define SIZE_ATTRIBUTE_HIDS_RPT_REF                     2
#define SIZE_ATTRIBUTE_HIDS_81_                         (7-3)
#define SIZE_ATTRIBUTE_HIDS_82_                         (7-3)
#define SIZE_ATTRIBUTE_HIDS_5D_                         (7-3)
#define SIZE_ATTRIBUTE_HIDS_05_                         (7-3-1)
#define SIZE_ATTRIBUTE_HIDS_08_                         (7-3-1)
#define SIZE_ATTRIBUTE_HIDS_09_                         (7-3-1)
#endif      //#ifndef _PROFILE_HOGP_EMC_

#define _HOGP_HDL_OFFSET_BASE_                          (_BAS_HDL_OFFSET_BASE_+SIZE_ATTRIBUTE_BAS)
//#define _HOGP_HDL_OFFSET_CONSUMER_                      (_HOGP_HDL_OFFSET_BASE_+SIZE_ATTRIBUTE_HIDS_BASE_)
#define _HOGP_HDL_OFFSET_KB_                            (_HOGP_HDL_OFFSET_BASE_+SIZE_ATTRIBUTE_HIDS_BASE_)
#define _HOGP_HDL_OFFSET_MOUSE_                         (_HOGP_HDL_OFFSET_KB_+SIZE_ATTRIBUTE_HIDS_KEYBOARD_)
//#define _HOGP_HDL_OFFSET_KB_                            (_HOGP_HDL_OFFSET_MOUSE_+SIZE_ATTRIBUTE_HIDS_MOUSE_)

#ifndef _PROFILE_HOGP_EMC_
#define _HOGP_HDL_OFFSET_CONSUMER_                      (_HOGP_HDL_OFFSET_MOUSE_+SIZE_ATTRIBUTE_HIDS_MOUSE_)
#define _HOGP_HDL_OFFSET_RPT_REF_                       (_HOGP_HDL_OFFSET_CONSUMER_+SIZE_ATTRIBUTE_HIDS_COMSUMER_)
#else       //#ifndef _PROFILE_HOGP_EMC_
#define _HOGP_HDL_OFFSET_81_                            (_HOGP_HDL_OFFSET_MOUSE_+SIZE_ATTRIBUTE_HIDS_MOUSE_)
#define _HOGP_HDL_OFFSET_82_                            (_HOGP_HDL_OFFSET_81_+SIZE_ATTRIBUTE_HIDS_81_)
#define _HOGP_HDL_OFFSET_5D_                            (_HOGP_HDL_OFFSET_82_+SIZE_ATTRIBUTE_HIDS_82_)
#define _HOGP_HDL_OFFSET_05_                            (_HOGP_HDL_OFFSET_5D_+SIZE_ATTRIBUTE_HIDS_5D_)
#define _HOGP_HDL_OFFSET_08_                            (_HOGP_HDL_OFFSET_05_+SIZE_ATTRIBUTE_HIDS_05_)
#define _HOGP_HDL_OFFSET_09_                            (_HOGP_HDL_OFFSET_08_+SIZE_ATTRIBUTE_HIDS_08_)
#define _HOGP_HDL_OFFSET_RPT_REF_                       (_HOGP_HDL_OFFSET_09_+SIZE_ATTRIBUTE_HIDS_09_)
#endif      //#ifndef _PROFILE_HOGP_EMC_

#define _HOGP_HDL_OFFSET_BOOT_KB_                       (_HOGP_HDL_OFFSET_RPT_REF_+SIZE_ATTRIBUTE_HIDS_RPT_REF)
#define _HOGP_HDL_OFFSET_BOOT_MS_                       (_HOGP_HDL_OFFSET_BOOT_KB_+SIZE_ATTRIBUTE_HIDS_BOOT_KB)

#define _HIDS_HDL_OFFSET_BASE_                          (_BAS_HDL_OFFSET_BASE_+SIZE_ATTRIBUTE_BAS)
#define SIZE_ATTRIBUTE_HIDS                             (\
                                                        SIZE_ATTRIBUTE_HIDS_BASE_+\
                                                        SIZE_ATTRIBUTE_HIDS_COMSUMER_+\
                                                        SIZE_ATTRIBUTE_HIDS_MOUSE_+\
                                                        SIZE_ATTRIBUTE_HIDS_KEYBOARD_+\
                                                        SIZE_ATTRIBUTE_HIDS_81_+\
                                                        SIZE_ATTRIBUTE_HIDS_82_+\
                                                        SIZE_ATTRIBUTE_HIDS_5D_+\
                                                        SIZE_ATTRIBUTE_HIDS_05_+\
                                                        SIZE_ATTRIBUTE_HIDS_08_+\
                                                        SIZE_ATTRIBUTE_HIDS_09_+\
                                                        SIZE_ATTRIBUTE_HIDS_RPT_REF+\
                                                        SIZE_ATTRIBUTE_HIDS_BOOT_KB+\
                                                        SIZE_ATTRIBUTE_HIDS_BOOT_MS\
                                                        )
#ifdef _PROFILE_HOGP_MULTI_
#define SIZE_OF_ATT_VALUE_HID_OFFSET                    0
#else
#define SIZE_OF_ATT_VALUE_HID_OFFSET                    2
#endif  //_PROFILE_HOGP_MULTI_

#ifdef _PROFILE_HOGP_MOUSE_
#define SIZE_OF_ATTRIBUTE_VALUE_MS                      (101-SIZE_OF_ATT_VALUE_HID_OFFSET)
#else
#define SIZE_OF_ATTRIBUTE_VALUE_MS                      0
#endif //_PROFILE_HOGP_MOUSE_

#ifdef _PROFILE_HOGP_KEYBOARD_

#ifdef _PROFILE_HOGP_KEYBOARD_PHOTO_
#define SIZE_OF_ATTRIBUTE_VALUE_KB                      (86-SIZE_OF_ATT_VALUE_HID_OFFSET)
#else
#define SIZE_OF_ATTRIBUTE_VALUE_KB                      (56-SIZE_OF_ATT_VALUE_HID_OFFSET)
#endif  //_PROFILE_HOGP_KEYBOARD_PHOTO_

#else
#define SIZE_OF_ATTRIBUTE_VALUE_KB                      0
#endif  //_PROFILE_HOGP_KEYBOARD_

#ifdef _PROFILE_HOGP_COMSUMER_
#define SIZE_OF_ATTRIBUTE_VALUE_CS                      (25-SIZE_OF_ATT_VALUE_HID_OFFSET)
#else
#define SIZE_OF_ATTRIBUTE_VALUE_CS                      0
#endif //_PROFILE_HOGP_COMSUMER_

#else
#define _HIDS_HDL_OFFSET_BASE_                          (_BAS_HDL_OFFSET_BASE_+SIZE_ATTRIBUTE_BAS)
#define _HOGP_HDL_OFFSET_BASE_                          (_BAS_HDL_OFFSET_BASE_+SIZE_ATTRIBUTE_BAS)
#define SIZE_ATTRIBUTE_HIDS                             0
#endif      //#ifdef _PROFILE_HOGP_
/** @} */


/**
 * @defgroup ble_attSCPPSize Scan Parameters Profile Attribute Size.
 * @{
 * @details Scan Parameters Profile attribute size.
 */
#ifdef _PROFILE_SCPP_
#define _SCPS_HDL_OFFSET_BASE_                          (_HOGP_HDL_OFFSET_BASE_+SIZE_ATTRIBUTE_HIDS)
#define SIZE_ATTRIBUTE_SCPS                             6
#else
#define _SCPS_HDL_OFFSET_BASE_                          (_HOGP_HDL_OFFSET_BASE_+SIZE_ATTRIBUTE_HIDS)
#define SIZE_ATTRIBUTE_SCPS                             0
#endif
/** @} */


/**
 * @defgroup ble_attIASSize Immediate Alert Service Attribute Size.
 * @{
 * @details Immediate Alert Service attribute size.
 */
#ifdef _PROFILE_IAS_
#define _IAS_HDL_OFFSET_BASE_                           (_SCPS_HDL_OFFSET_BASE_+SIZE_ATTRIBUTE_SCPS)
#define SIZE_ATTRIBUTE_IAS                              3
#else
#define _IAS_HDL_OFFSET_BASE_                           (_SCPS_HDL_OFFSET_BASE_+SIZE_ATTRIBUTE_SCPS)
#define SIZE_ATTRIBUTE_IAS                              0
#endif
/** @} */


/**
 * @defgroup ble_attLLSSize Link Loss Service Attribute Size.
 * @{
 * @details Link Loss Service attribute size.
 */
#ifdef _PROFILE_LLS_
#define _LLS_HDL_OFFSET_BASE_                           (_IAS_HDL_OFFSET_BASE_+SIZE_ATTRIBUTE_IAS)
#define SIZE_ATTRIBUTE_LLS                              3
#else
#define _LLS_HDL_OFFSET_BASE_                           (_IAS_HDL_OFFSET_BASE_+SIZE_ATTRIBUTE_IAS)
#define SIZE_ATTRIBUTE_LLS                              0
#endif
/** @} */


/**
 * @defgroup ble_attTPSSize TX Power Service Attribute Size.
 * @{
 * @details TX Power Service attribute size.
 */
#ifdef _PROFILE_TPS_
#define _TPS_HDL_OFFSET_BASE_                           (_LLS_HDL_OFFSET_BASE_+SIZE_ATTRIBUTE_LLS)
#define SIZE_ATTRIBUTE_TPS                              3
#else
#define _TPS_HDL_OFFSET_BASE_                           (_LLS_HDL_OFFSET_BASE_+SIZE_ATTRIBUTE_LLS)
#define SIZE_ATTRIBUTE_TPS                              0
#endif
/** @} */


/**
 * @defgroup ble_attBLPSize Blood Pressure Profile Attribute Size.
 * @{
 * @details Blood Pressure Profile attribute size.
 */
#ifdef _PROFILE_BLP_
#define _BLS_HDL_OFFSET_BASE_                           (_TPS_HDL_OFFSET_BASE_+SIZE_ATTRIBUTE_TPS)
#define SIZE_ATTRIBUTE_BLS                              6
#else
#define _BLS_HDL_OFFSET_BASE_                           (_TPS_HDL_OFFSET_BASE_+SIZE_ATTRIBUTE_TPS)
#define SIZE_ATTRIBUTE_BLS                              0
#endif
/** @} */



/**
 * @defgroup ble_attHTPSize Health Thermometer Profile Attribute Size.
 * @{
 * @details Health Thermometer Profile attribute size.
 */
#ifdef _PROFILE_HTP_
#define _HTS_HDL_OFFSET_BASE_                           (_BLS_HDL_OFFSET_BASE_+SIZE_ATTRIBUTE_BLS)
#define SIZE_ATTRIBUTE_HTS                              6
#else
#define _HTS_HDL_OFFSET_BASE_                           (_BLS_HDL_OFFSET_BASE_+SIZE_ATTRIBUTE_BLS)
#define SIZE_ATTRIBUTE_HTS                              0
#endif
/** @} */



/**
 * @defgroup ble_attRSCPSize Running Speed and Cadence Profile Attribute Size.
 * @{
 * @details Running Speed and Cadence Profile attribute size.
 */
#ifdef _PROFILE_RSCP_
#define _RSCS_HDL_OFFSET_BASE_                          (_HTS_HDL_OFFSET_BASE_+SIZE_ATTRIBUTE_HTS)
#define SIZE_ATTRIBUTE_RSCS                             9
#else
#define _RSCS_HDL_OFFSET_BASE_                          (_HTS_HDL_OFFSET_BASE_+SIZE_ATTRIBUTE_HTS)
#define SIZE_ATTRIBUTE_RSCS                             0
#endif
/** @} */



/**
 * @defgroup ble_attCSCPSize Cycling Speed and Cadence Profile Attribute Size.
 * @{
 * @details Cycling Speed and Cadence Profile attribute size.
 */
#ifdef _PROFILE_CSCP_
#define _CSCS_HDL_OFFSET_BASE_                          (_RSCS_HDL_OFFSET_BASE_+SIZE_ATTRIBUTE_RSCS)
#define SIZE_ATTRIBUTE_CSCS                             9
#else
#define _CSCS_HDL_OFFSET_BASE_                          (_RSCS_HDL_OFFSET_BASE_+SIZE_ATTRIBUTE_RSCS)
#define SIZE_ATTRIBUTE_CSCS                             0
#endif
/** @} */



/**
 * @defgroup ble_attLNSSize Location and Navigation Service Attribute Size.
 * @{
 * @details Location and Navigation Service attribute size.
 */
#ifdef _PROFILE_LNS_
#define _LNS_HDL_OFFSET_BASE_                           (_CSCS_HDL_OFFSET_BASE_+SIZE_ATTRIBUTE_CSCS)
#define SIZE_ATTRIBUTE_LNS                              14
#else
#define _LNS_HDL_OFFSET_BASE_                           (_CSCS_HDL_OFFSET_BASE_+SIZE_ATTRIBUTE_CSCS)
#define SIZE_ATTRIBUTE_LNS                              0
#endif
/** @} */



/**
 * @defgroup ble_attGLSSize Glucose Service Attribute Size.
 * @{
 * @details Glucose Service attribute size.
 */
#ifdef _PROFILE_GLS_
#define _GLS_HDL_OFFSET_BASE_                           (_LNS_HDL_OFFSET_BASE_+SIZE_ATTRIBUTE_LNS)
#define SIZE_ATTRIBUTE_GLS                              12
#else
#define _GLS_HDL_OFFSET_BASE_                           (_LNS_HDL_OFFSET_BASE_+SIZE_ATTRIBUTE_LNS)
#define SIZE_ATTRIBUTE_GLS                              0
#endif
/** @} */


/**
 * @defgroup ble_attCPSSize Cycling Power Service Attribute Size.
 * @{
 * @details Cycling Power Service attribute size.
 */
#ifdef _PROFILE_CPS_
#define _CPS_HDL_OFFSET_BASE_                           (_GLS_HDL_OFFSET_BASE_+SIZE_ATTRIBUTE_GLS)
#define SIZE_ATTRIBUTE_CPS                              15
#else
#define _CPS_HDL_OFFSET_BASE_                           (_GLS_HDL_OFFSET_BASE_+SIZE_ATTRIBUTE_GLS)
#define SIZE_ATTRIBUTE_CPS                              0
#endif
/** @} */



/**
 * @defgroup ble_attUDF01SSize Custom UDF01S Service Attribute Size.
 * @{
 * @details Custom UDF01S Service attribute size.
 */
#ifdef _PROFILE_CSTM_UDF01S_
#define SIZE_ATTRIBUTE_UDF01S_BASE_                     16
#else
#define SIZE_ATTRIBUTE_UDF01S_BASE_                     0
#endif
#define _UDF01S_HDL_OFFSET_BASE_                        (_CPS_HDL_OFFSET_BASE_+SIZE_ATTRIBUTE_CPS)
/** @} */


/**
 * @defgroup ble_attUDF02SSize Custom UDF01S Service Attribute Size.
 * @{
 * @details Custom UDF02S Service attribute size.
 */
#ifdef _PROFILE_CSTM_UDF02S_
#define SIZE_ATTRIBUTE_UDF02S_BASE_                     8
#else
#define SIZE_ATTRIBUTE_UDF02S_BASE_                     0
#endif
#define _UDF02S_HDL_OFFSET_BASE_                        (_UDF01S_HDL_OFFSET_BASE_+SIZE_ATTRIBUTE_UDF01S_BASE_)
/** @} */



/**
 * @defgroup ble_attUDF03SSize Custom UDF01S Service Attribute Size.
 * @{
 * @details Custom UDF03S Service attribute size.
 */
#ifdef _PROFILE_CSTM_UDF03S_
#define SIZE_ATTRIBUTE_UDF03S_BASE_                     8
#else
#define SIZE_ATTRIBUTE_UDF03S_BASE_                     0
#endif
#define _UDF03S_HDL_OFFSET_BASE_                        (_UDF02S_HDL_OFFSET_BASE_+SIZE_ATTRIBUTE_UDF02S_BASE_)
/** @} */

/**
 * @defgroup ble_attATCMDSize Over the Air Service Attribute Size.
 * @{
 * @details Custom ATCMD Service attribute size.
 */
#ifdef _PROFILE_ATCMD_
#define SIZE_ATTRIBUTE_ATCMD 8
#else
#define SIZE_ATTRIBUTE_ATCMD 0
#endif
#define _ATCMD_HDL_OFFSET_BASE_ (_UDF03S_HDL_OFFSET_BASE_ + SIZE_ATTRIBUTE_UDF03S_BASE_)

/** @} */

/**
 * @defgroup ble_attDESSize Custom DATAEXCHANGE_S Service Attribute Size.
 * @{
 * @details Custom DATAEXCHANGE_S Service attribute size.
 */
#ifdef _PROFILE_CSTM_DATAEXCHANGE_S_
#define SIZE_ATTRIBUTE_DATAEXCHANGE_BASE_               6
#else
#define SIZE_ATTRIBUTE_DATAEXCHANGE_BASE_               0
#endif
#define _DATAEXCHANGE_HDL_OFFSET_BASE_ (_ATCMD_HDL_OFFSET_BASE_ + SIZE_ATTRIBUTE_ATCMD)
/** @} */


/**
 * @defgroup ble_attOTASize Over the Air Service Attribute Size.
 * @{
 * @details Over the Air Service attribute size.
 */
#ifdef _PROFILE_OTA_
#define SIZE_ATTRIBUTE_OTA                              9
#else
#define SIZE_ATTRIBUTE_OTA                              0
#endif
#define _OTA_HDL_OFFSET_BASE_                           (_DATAEXCHANGE_HDL_OFFSET_BASE_+SIZE_ATTRIBUTE_DATAEXCHANGE_BASE_)

/** @} */


/** @} */



/**
 * @defgroup ble_attServerSize BLE Server Attribute Total Size Definition
 * @{
 * @note MUST add all the used service in it.
 * @ingroup host
 * @param refer to @ref ble_attProfileSize
 */
#define SIZE_ATTRIBUTE_SERVER (         \
    SIZE_ATTRIBUTE_NULL +               \
    SIZE_ATTRIBUTE_GAP +                \
    SIZE_ATTRIBUTE_GATT +               \
    SIZE_ATTRIBUTE_DIS +                \
    SIZE_ATTRIBUTE_HRS +                \
    SIZE_ATTRIBUTE_BAS +                \
    SIZE_ATTRIBUTE_HIDS +               \
    SIZE_ATTRIBUTE_SCPS +               \
    SIZE_ATTRIBUTE_IAS +                \
    SIZE_ATTRIBUTE_LLS +                \
    SIZE_ATTRIBUTE_TPS +                \
    SIZE_ATTRIBUTE_BLS +                \
    SIZE_ATTRIBUTE_HTS +                \
    SIZE_ATTRIBUTE_RSCS +               \
    SIZE_ATTRIBUTE_CSCS +               \
    SIZE_ATTRIBUTE_LNS +                \
    SIZE_ATTRIBUTE_GLS +                \
    SIZE_ATTRIBUTE_CPS +                \
    SIZE_ATTRIBUTE_UDF01S_BASE_ +       \
    SIZE_ATTRIBUTE_UDF02S_BASE_ +       \
    SIZE_ATTRIBUTE_UDF03S_BASE_ +       \
    SIZE_ATTRIBUTE_ATCMD +              \
    SIZE_ATTRIBUTE_DATAEXCHANGE_BASE_ + \
    SIZE_ATTRIBUTE_OTA)
/** @} */





#ifdef _SMP_ON_

/**
 * @defgroup ble_attProfileBondSize BLE Profile Attribute Bonding Size Definition
 * @{
 * @ingroup ble_attSize
 */

#define SIZE_ATTRIBUTE_BOND_NULL                        0   /**< Size of bonding NULL.*/
#define SIZE_ATTRIBUTE_BOND_GAP                         1   /**< Size of bonding GAP.*/
#define SIZE_ATTRIBUTE_BOND_GATT                        0   /**< Size of bonding GATT.*/
#define SIZE_ATTRIBUTE_BOND_DIS                         0   /**< Size of bonding Device Information Service.*/
#define SIZE_ATTRIBUTE_BOND_HRS                         0   /**< Size of bonding Heart Rate Profile.*/

#ifdef _PROFILE_BAS_
#define SIZE_ATTRIBUTE_BOND_BAS                         1   /**< Size of bonding Battery Service.*/
#else
#define SIZE_ATTRIBUTE_BOND_BAS                         0   /**< Size of bonding Battery Service.*/
#endif


/**
 * @defgroup ble_attHOGPBondSize Size of bonding Human Interface Over GATT Protocol Service
 * @{
 * @ingroup ble_attProfileBondSize
 */
#ifdef _PROFILE_HOGP_
#ifdef _PROFILE_HOGP_COMSUMER_
#define SIZE_ATTRIBUTE_BOND_HIDS_COMSUMER_              1
#else
#define SIZE_ATTRIBUTE_BOND_HIDS_COMSUMER_              0
#endif
#ifdef _PROFILE_HOGP_MOUSE_
#define SIZE_ATTRIBUTE_BOND_HIDS_MOUSE_                 2
#else
#define SIZE_ATTRIBUTE_BOND_HIDS_MOUSE_                 0
#endif
#ifdef _PROFILE_HOGP_KEYBOARD_
#define SIZE_ATTRIBUTE_BOND_HIDS_KEYBOARD_              1
#else
#define SIZE_ATTRIBUTE_BOND_HIDS_KEYBOARD_              0
#endif
#ifdef _PROFILE_HOGP_BOOT_
#ifdef _PROFILE_HOGP_KEYBOARD_
#define SIZE_ATTRIBUTE_BOND_HIDS_BOOT_KB                1
#else
#define SIZE_ATTRIBUTE_BOND_HIDS_BOOT_KB                0
#endif
#ifdef _PROFILE_HOGP_MOUSE_
#define SIZE_ATTRIBUTE_BOND_HIDS_BOOT_MS                1
#else
#define SIZE_ATTRIBUTE_BOND_HIDS_BOOT_MS                0
#endif
#else
#define SIZE_ATTRIBUTE_BOND_HIDS_BOOT_KB                0
#define SIZE_ATTRIBUTE_BOND_HIDS_BOOT_MS                0
#endif
#endif

#ifdef _PROFILE_HOGP_
#define SIZE_ATTRIBUTE_BOND_HIDS                        (\
                                                        SIZE_ATTRIBUTE_BOND_HIDS_COMSUMER_+\
                                                        SIZE_ATTRIBUTE_BOND_HIDS_MOUSE_+\
                                                        SIZE_ATTRIBUTE_BOND_HIDS_KEYBOARD_+\
                                                        SIZE_ATTRIBUTE_BOND_HIDS_BOOT_KB+\
                                                        SIZE_ATTRIBUTE_BOND_HIDS_BOOT_MS\
                                                        )
#else
#define SIZE_ATTRIBUTE_BOND_HIDS                        0
#endif  //#ifdef _PROFILE_HOGP_

/** @} */




#define SIZE_ATTRIBUTE_BOND_SCPS                        0 /**< Size of bonding Scan Parameters Profile.*/
#define SIZE_ATTRIBUTE_BOND_IAS                         0 /**< Size of bonding Immediate Alert Service.*/
#define SIZE_ATTRIBUTE_BOND_LLS                         0 /**< Size of bonding Link Loss Service.*/
#define SIZE_ATTRIBUTE_BOND_TPS                         0 /**< Size of bonding TX Power Service.*/
#define SIZE_ATTRIBUTE_BOND_BLS                         0 /**< Size of bonding Blood Pressure Profile.*/
#define SIZE_ATTRIBUTE_BOND_HTS                         0 /**< Size of bonding Health Thermometer Profile.*/
#define SIZE_ATTRIBUTE_BOND_RSCS                        0 /**< Size of bonding Running Speed and Cadence Profile.*/
#define SIZE_ATTRIBUTE_BOND_CSCS                        0 /**< Size of bonding Cycling Speed and Cadence Profile.*/




#ifdef _PROFILE_GLS_
#define SIZE_ATTRIBUTE_BOND_GLS                         3 /**< Size of bonding Glucose Service.*/
#else
#define SIZE_ATTRIBUTE_BOND_GLS                         0 /**< Size of bonding Glucose Service.*/
#endif


#define SIZE_ATTRIBUTE_BOND_UDF01S_BASE_                0 /**< Size of bonding Custom UDF01S Service.*/
#define SIZE_ATTRIBUTE_BOND_UDF02S_BASE_                0 /**< Size of bonding Custom UDF02S Service.*/
#define SIZE_ATTRIBUTE_BOND_UDF03S_BASE_                0 /**< Size of bonding Custom UDF03S Service.*/

/** @} */


/**
 * @defgroup ble_attServerBondSize BLE Server Attribute Bonding Total Size Definition
 * @{
 * @note MUST add all the used bonding service in it.
 * @ingroup host
 * @param refer to @ref ble_attProfileBondSize
 */
#define SIZE_ATTRIBUTE_SERVER_BOND                      (\
SIZE_ATTRIBUTE_BOND_NULL+\
SIZE_ATTRIBUTE_BOND_GAP+\
SIZE_ATTRIBUTE_BOND_GATT+\
SIZE_ATTRIBUTE_BOND_DIS+\
SIZE_ATTRIBUTE_BOND_HRS+\
SIZE_ATTRIBUTE_BOND_BAS+\
SIZE_ATTRIBUTE_BOND_HIDS+\
SIZE_ATTRIBUTE_BOND_SCPS+\
SIZE_ATTRIBUTE_BOND_IAS+\
SIZE_ATTRIBUTE_BOND_LLS+\
SIZE_ATTRIBUTE_BOND_TPS+\
SIZE_ATTRIBUTE_BOND_BLS+\
SIZE_ATTRIBUTE_BOND_HTS+\
SIZE_ATTRIBUTE_BOND_RSCS+\
SIZE_ATTRIBUTE_BOND_CSCS+\
SIZE_ATTRIBUTE_BOND_GLS+\
SIZE_ATTRIBUTE_BOND_UDF01S_BASE_+\
SIZE_ATTRIBUTE_BOND_UDF02S_BASE_+\
SIZE_ATTRIBUTE_BOND_UDF03S_BASE_\
)
/** @} */

#endif  //(#ifdef _SMP_ON_)


/**************************************************************************
 * Extern Variable
 **************************************************************************/
/** @defgroup host_variable Variable
 * @{
 * @details Here shows the Variable in host.h.
 * @ingroup host
 **************************************************************************/

extern const Uint8 *ATTRIBUTE_SERVER[];             /**< Table combines the description of handles corresponding to each GATT declarations and characteristics. */
extern const Uint8 *ATTRIBUTE_SERVER_PARAM[];       /**< Table combines the content of handles corresponding to each GATT declarations and characteristics. */
extern const Uint8 *ATTRIBUTE_SERVER_PERMISSION[];  /**< Table combines the permission setting corresponding to each GATT declarations and characteristics. */
#ifdef _SMP_ON_
#ifdef _BOND_ON_
extern const Uint8 *ATTRIBUTE_SERVER_BOND[SIZE_ATTRIBUTE_SERVER_BOND];            /**< Table combines the description of handles corresponding to each bonding GATT declarations and characteristics. */
extern const Uint8 *ATTRIBUTE_SERVER_BOND_PARAM[SIZE_ATTRIBUTE_SERVER_BOND];      /**< Table combines the content of handles corresponding to each bonding GATT declarations and characteristics.*/
#endif
#endif

extern Uint8 size_ATTRIBUTE_SERVER;                                               /**< Size of @ref ATTRIBUTE_SERVER. */
#ifdef _SMP_ON_
#ifdef _BOND_ON_
extern Uint8 size_ATTRIBUTE_SERVER_BOND;                                          /**< Size of @ref ATTRIBUTE_SERVER_BOND. */
#endif
#endif
/** @} */

/**************************************************************************
 * Functions
 **************************************************************************/
/** @defgroup host_function Function
 * @{
 * @details Here shows the function in host.h.
 * @ingroup host
 * @}
 **************************************************************************/

/** ATT Server Handle Sending a Notification to a Client
 *
 * @ingroup host_function
 * @param[in] connID : The link's connection id.
 * @param[in] addrATT_HDL_INIT : The pointer to the characteristic attribute
 * @param[in] addrATT_HDL_clientCFG :   The pointer to the characteristic attribute parameter
 * @param[in] data : The data to send
 * @param[in] dataLength : The data length of the data to send
 *
 * @retval BLESTACK_STATUS_ERR_BUSY : Message queue buffer full.
 * @retval BLESTACK_STATUS_ERR_INVALID_CONNID : Error connection id.
 * @retval BLESTACK_STATUS_ERR_CLIENT_CONFIGURATION_DISABLE : Profile client configuration disable.
 * @retval BLESTACK_STATUS_SUCCESS : function process status.
 */
BleStackStatus ATT_HDL_Notify(uint8_t connID, uint8_t *addrATT_HDL_INIT, uint8_t *addrATT_HDL_clientCFG, uint8_t *data, uint8_t dataLength);


/** ATT Server Handle Sending a indication to a Client
 *
 * @ingroup host_function
 * @param[in] connID : The link's connection id.
 * @param[in] addrATT_HDL_INIT : The pointer to the characteristic attribute
 * @param[in] addrATT_HDL_clientCFG :   The pointer to the characteristic attribute parameter
 * @param[in] data : The data to send
 * @param[in] dataLength : The data length of the data to send
 *
 * @retval BLESTACK_STATUS_ERR_BUSY : Message queue buffer full.
 * @retval BLESTACK_STATUS_ERR_INVALID_CONNID : Error connection id.
 * @retval BLESTACK_STATUS_ERR_CLIENT_CONFIGURATION_DISABLE : Profile client configuration disable.
 * @retval BLESTACK_STATUS_SUCCESS : function process status.
 */
BleStackStatus ATT_HDL_Indicate(uint8_t connID, uint8_t *addrATT_HDL_INIT, uint8_t *addrATT_HDL_clientCFG, uint8_t *data, uint8_t dataLength);



/** ATT Server Handle Setting the value to a Given Attribute.
 *
 * @ingroup host_function
 * @param[in] addr_attHDL : The pointer to the attribute parameter
 * @param[in] data : The data to send
 * @param[in] dataLength : The data length of the data to send
 *
 */
void ATT_HDL_ValueSet(uint8_t *addr_attHDL, uint8_t *data, uint8_t dataLength);

#endif
