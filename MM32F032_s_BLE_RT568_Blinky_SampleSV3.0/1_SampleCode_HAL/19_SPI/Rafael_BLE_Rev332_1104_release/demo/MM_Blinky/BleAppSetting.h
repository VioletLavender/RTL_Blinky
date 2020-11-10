#ifndef _BLEAPPSETTING_H_
#define _BLEAPPSETTING_H_

#include "mcu_definition.h"

#define  DEMO_HRS                           0
#define  DEMO_TRSPX_LED_SLAVE               1
#define  DEMO_TRSPX_UART_SLAVE              2      //use UART0
#define  DEMO_HRS_DTM                       8      //use UART0
#define  DEMO_DTM                           9      //use UART0
//Select a demo application
#define  BLE_DEMO                           DEMO_TRSPX_UART_SLAVE

#if (BLE_DEMO == DEMO_TRSPX_UART_SLAVE)
    // Enable data length extension
    #define BLE_DATA_LENGTH_EXTENSION_SUPPORT     ENABLE_ // ENABLE_;DISABLE_
#endif

/* ************************************
 * Select a demo BLE Server Profile
 **************************************/

/* ------------------------------------
* SIG Defined Profiles
 -------------------------------------*/
//#define    _PROFILE_BAS_
//#define    _PROFILE_BLP_
//#define    _PROFILE_CPS_
//#define    _PROFILE_CSCP_
//#define    _PROFILE_GLS_
//#define    _PROFILE_HOGP_
//#define    _PROFILE_HOGP_BOOT_
//#define    _PROFILE_HOGP_COMSUMER_
//#define    _PROFILE_HOGP_EMC_
//#define    _PROFILE_HOGP_MOUSE_
//#define    _PROFILE_HOGP_MULTI_
//#define    _PROFILE_HOGP_KEYBOARD_
//#define    _PROFILE_HOGP_KEYBOARD_PHOTO_
#if (BLE_DEMO==DEMO_HRS)
    #define    _PROFILE_HRP_
#endif
//#define    _PROFILE_HTP_
//#define    _PROFILE_IAS_
//#define    _PROFILE_LLS_
//#define    _PROFILE_LNS_
//#define    _PROFILE_OTA_
//#define    _PROFILE_RSCP_
//#define    _PROFILE_SCPP_
//#define    _PROFILE_TPS_


/* ------------------------------------
 * Customer Defined Profiles
  -------------------------------------*/
#if (BLE_DEMO==DEMO_TRSPX_LED_SLAVE || BLE_DEMO==DEMO_TRSPX_UART_SLAVE)
    #define    _PROFILE_CSTM_UDF01S_
#endif
//#define    _PROFILE_CSTM_UDF02S_
//#define    _PROFILE_CSTM_UDF03S_
//#define    _PROFILE_CSTM_DATAEXCHANGE_S_


/* ------------------------------------
 * Select BLE Client Profile
  -------------------------------------*/
#ifdef _HOST_CLIENT_
    //#define _PROFILE_CLI_HRP_
    //#define _PROFILE_CLI_HTP_
    //#define _PROFILE_CLI_CSCP_
    //#define _PROFILE_CLI_BLP_
    //#define _PROFILE_CLI_FMP_
    //#define _PROFILE_CLI_SCPP_
    //#define _PROFILE_CLI_PXP_
    //#define _PROFILE_CLI_RSCP_
    //#define _PROFILE_CLI_BAS_
    //#define _PROFILE_CLI_IAS_
    //#define _PROFILE_CLI_LLS_
    //#define _PROFILE_CLI_TPS_
    //#define _PROFILE_CLI_LNS_
    //#define _PROFILE_CLI_GLS_
    //#define _PROFILE_CLI_CPS_
    //#define _PROFILE_CLI_CSTM_UDF01S_
    //#define _PROFILE_CLI_CSTM_UDF02S_
    //#define _PROFILE_CLI_CSTM_UDF03S_
#endif


/*---- For future application use ----*/
//#define _PROFILE_HOGP_
//#define _PROFILE_HOGP_MOUSE_
//#define _PROFILE_HOGP_KEYBOARD_
//#define _PROFILE_HOGP_COMSUMER_
//#define _PROFILE_HOGP_RUEMANN_

#ifdef _PROFILE_HOGP_
    #ifndef _PROFILE_HOGP_KEYBOARD_
        #ifdef _PROFILE_HOGP_KEYBOARD_PHOTO_
            #define _PROFILE_HOGP_KEYBOARD_
        #endif  //(#ifdef _PROFILE_HOGP_KEYBOARD_PHOTO_)
    #endif  //(#ifndef _PROFILE_HOGP_KEYBOARD_)

    #ifdef _PROFILE_HOGP_MOUSE_
        #if defined _PROFILE_HOGP_KEYBOARD_
            #define _PROFILE_HOGP_MULTI_
        #elif defined _PROFILE_HOGP_COMSUMER_
            #define _PROFILE_HOGP_MULTI_
        #endif  //(#if defined _PROFILE_HOGP_KEYBOARD_)
    #endif  //(#ifdef _PROFILE_HOGP_MOUSE_)

    #ifndef _PROFILE_HOGP_MULTI_
        #ifdef _PROFILE_HOGP_KEYBOARD_
            #ifdef _PROFILE_HOGP_COMSUMER_
                #define _PROFILE_HOGP_MULTI_
            #endif  //(#ifdef _PROFILE_HOGP_COMSUMER_)
        #endif  //(#ifdef _PROFILE_HOGP_KEYBOARD_)
    #else   //(#ifndef _PROFILE_HOGP_MULTI_)
    #endif  //(#ifndef _PROFILE_HOGP_MULTI_)

    #ifndef _PROFILE_HOGP_MULTI_
        #define __PROFILE_HOGP_KEYBOARD_RPIDOFST_   0
        #define __PROFILE_HOGP_CS_RPIDOFST_         0
        #define __PROFILE_HOGP_MS_RPIDOFST_         0
    #else   //(#ifndef _PROFILE_HOGP_MULTI_)
        #define __PROFILE_HOGP_CS_RPIDOFST_         3
        #define __PROFILE_HOGP_MS_RPIDOFST_         2
        #ifdef _PROFILE_HOGP_EMC_1501_
            #define __PROFILE_HOGP_KEYBOARD_RPIDOFST_   2
        #else   //(#ifdef _PROFILE_HOGP_EMC_1501_)
            #define __PROFILE_HOGP_KEYBOARD_RPIDOFST_   1
        #endif  //(#ifdef _PROFILE_HOGP_EMC_1501_)
    #endif  //(#ifndef _PROFILE_HOGP_MULTI_)
#endif  //(#ifdef _PROFILE_HOGP_)



/* ------------------------------------
 * Application Setting
  -------------------------------------*/

// Removed in SDK
enum
{
    BLE_LL_SLAVE_ONLY = 0U,
    BLE_LL_MASTER_ONLY = 1U,
    BLE_LL_SLAVE_MASTER = 2U
};

//#define _HCI_ON_       //remove in SDK
#define _HCI_NEW_        //define in SDK
////#define _HCI_HW_       //remove in SDK. However, it is a must for test
//#define _HCI_HW_MSG_   //remove in SDK. (msg2uart)
//#define _LE_SCAN_ON_   //no use

#ifndef _HCI_NEW_
    #undef _HCI_HW_
#endif
#ifndef _HCI_HW_
    #undef _HCI_HW_MSG_      //remove in SDK
#endif

//#define _DEBUG_DISPLAY_      //remove in SDK
//#define _DEBUG_ICE_          //remove in SDK, no use
//#define _DEBUG_PINS_           //remove in SDK
//#define _DEBUG_MSG_USER_       //define in SDK
//#define _DEBUG_MSG_USER_SUB1 //remove in SDK


#ifdef _HCI_HW_
    #undef _DEBUG_MSG_USER_
#endif  //(#ifdef _HCI_HW_)

#ifndef _DEBUG_MSG_USER_
    #undef _DEBUG_MSG_USER_SUB1    //detailed debug message
#endif  //(#ifndef _DEBUG_MSG_USER_)

//#define _USED_EXCEPT_SDK_      //remove in SDK


// SDK: Company ID must to be changed to 0xFFFF
#define BLE_COMPANY_ID_L        0x64    //HCI__004
#define BLE_COMPANY_ID_H        0x08


#endif
