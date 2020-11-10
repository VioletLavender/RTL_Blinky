/**************************************************************************//**
 * @file     user.c
 * @version  V0.9
 * $Revision: 01 $
 * @brief
 *           Demonstrate how to use LIB pre-build function to start and stop a BLE
 *           connection.
 * @note
 *
 ******************************************************************************/
#include <stdio.h>
#include "BleAppSetting.h"
#include "host.h"
#include "rf_phy.h"
#include "porting_misc.h"
#include "ble_cmd.h"
#include "ble_event.h"

#define BIT0     (0x00000001UL)       ///< Bit 0 mask of an 32 bit integer
#define BIT1     (0x00000002UL)       ///< Bit 1 mask of an 32 bit integer

#ifdef _DEBUG_MSG_USER_
    #include <stdio.h>
#endif  //(#ifdef _DEBUG_MSG_USER_)

uint8_t connId = 0;   //Current connection Id
Uint8 ble_state = STATE_BLE_STANDBY;

#define HOGP_PAIRING_KEY  654321                 //pairing key. uint32

/* Function prototype declaration */
static void BleEvent_Callback(BleCmdEvent event, void *param);

#if (BLE_DEMO==DEMO_TRSPX_UART_SLAVE)
    extern void UART_TX_Send(uint32_t len, uint8_t *ptr);                         //show data on UART

    #if (BLE_DATA_LENGTH_EXTENSION_SUPPORT == ENABLE_)
        uint8_t txDataBuffer[247];
    #else
        uint8_t txDataBuffer[23];
    #endif

    uint8_t txDataLength;
    uint8_t tx_data_transmit_enable = 0;

#endif
#ifdef _DEBUG_MSG_USER_
    uint8_t gateTimeline = 0;
#endif  //(#ifdef _DEBUG_MSG_USER_)


#pragma push
//#pragma Otime
#pragma Ospace

//Advertising parameters
#define APP_ADV_INTERVAL_MIN       160U       //160*0.625ms=100ms
#define APP_ADV_INTERVAL_MAX       160U       //160*0.625ms=100ms


//Target BD_ADDR (eg: 1CBA8C20C501, SensorTag)
#define TARGET_ADDR0  0x01
#define TARGET_ADDR1  0xC5
#define TARGET_ADDR2  0x20
#define TARGET_ADDR3  0x8C
#define TARGET_ADDR4  0xBA
#define TARGET_ADDR5  0x1C

//Directed Target BD_ADDR (TBD)
#define DIR_TARGET_ADDR0  0x66
#define DIR_TARGET_ADDR1  0x55
#define DIR_TARGET_ADDR2  0x44
#define DIR_TARGET_ADDR3  0x33
#define DIR_TARGET_ADDR4  0x22
#define DIR_TARGET_ADDR5  0x11

//Scan Parameters
#define SCAN_WINDOW                10U    //10*0.625ms=6.25ms
#define SCAN_INTERVAL              10U    //10*0.625ms=6.25ms

//Initial Connection Parameters
#define APP_CONN_INTERVAL_MIN      38U    //38*1.25ms=47.5ms
#define APP_CONN_INTERVAL_MAX      42U    //42*1.25ms=52.5ms
#define CONN_SUPERVISION_TIMEOUT   60U    //60*10ms=600ms
#define CONN_SLAVE_LATENCY         0U     //0~499, 0 means slave listen at every anchor point


#define SIZE_OF_CODE_BLE_ADV_DATA_USER               (\
  SIZE_OF_CODE_BLE_ADV_DATA_FLAGS+\
  SIZE_OF_CODE_BLE_ADV_DATA_AD_TYPE_SERVICE+\
  SIZE_OF_CODE_BLE_ADV_DATA_AD_TYPE_APPEARANCE+\
  0x00)


const uint8_t SET_ADV_DATA[] =   //Adv data format. Set according to user select profile. See Bluetooth Spec. Ver5.0 [Vol 3], Part C, Section 11
{

    /* Length(1 byte) */
    0x1a,                     //Uint8 HCI_Adv_Data_Length; 1 byte
    //Uint8 HCI_Adv_Data[LEN_ADV_SCAN_DATA_MAX];
    GAP_AD_TYPE_LENGTH_3, GAP_AD_TYPE_APPEARANCE, 0x00, 0x00,   //0x0000: 0 -> Reserved
    /* Data: include AD_Type(n bytes), AD_Data(length-n bytes) */
    //GAP_AD_TYPE_LENGTH_2, GAP_AD_TYPE_FLAGS, BLE_GAP_FLAGS_GENERAL_DISCOVERABLE_MODE,    //LE General Discoverable Mode, Bluetooth Spec. Ver5.0 [Vol 3], Part C, Section 11
    GAP_AD_TYPE_LENGTH_2, GAP_AD_TYPE_FLAGS, BLE_GAP_FLAGS_GENERAL_DISCOVERABLE_MODE,      //LE Limit Discoverable Mode, Bluetooth Spec. Ver5.0 [Vol 3], Part C, Section 11

    //GAP_AD_TYPE_LENGTH_2, GAP_AD_TYPE_FLAGS, GAP_FLAGS_BR_EDR_NOT_SUPPORTED,            //LE Non Discoverable Mode, Bluetooth Spec. Ver5.0 [Vol 3], Part C, Section 11
    0x11, GAP_AD_TYPE_SERVICE_CPLT_128B_UUID, 0x23, 0xd1, 0xbc, 0xea, 0x5f, 0x78, 0x23, 0x15, 0xde, 0xef, 0x12, 0x12, 0x23, 0x15, 0x00, 0x00,

#if defined _PROFILE_HOGP_
    GAP_AD_TYPE_LENGTH_3, GAP_AD_TYPE_SERVICE_MORE_16B_UUID, GATT_SPEC_SERVICES_HUMAN_INTERFACE_DEVICE, GATT_SPEC_SERVICES,
#if defined _PROFILE_HOGP_MULTI_
    GAP_AD_TYPE_LENGTH_3, GAP_AD_TYPE_APPEARANCE, 0xC0, 0x03,   //0x03C0: 960 -> Human Interface Device (HID), HID Generic
    //GAP_AD_TYPE_LENGTH_3, GAP_AD_TYPE_APPEARANCE, 0xC1, 0x03,   //0x03C0: 961 -> Keyboard
#elif defined _PROFILE_HOGP_MOUSE_
    GAP_AD_TYPE_LENGTH_3, GAP_AD_TYPE_APPEARANCE, 0xC2, 0x03,   //0x03C2: 962 -> Mouse
#elif defined _PROFILE_HOGP_KEYBOARD_
    GAP_AD_TYPE_LENGTH_3, GAP_AD_TYPE_APPEARANCE, 0xC1, 0x03,   //0x03C1: 961 -> Keyboard
#elif defined _PROFILE_HOGP_COMSUMER_
    GAP_AD_TYPE_LENGTH_3, GAP_AD_TYPE_APPEARANCE, 0xC0, 0x03,   //0x03C0: 960 -> Human Interface Device (HID), HID Generic
#endif
#elif defined _PROFILE_BLP_
    GAP_AD_TYPE_LENGTH_3, GAP_AD_TYPE_SERVICE_MORE_16B_UUID, GATT_SPEC_SERVICES_BLOOD_PRESSURE, GATT_SPEC_SERVICES,
    //GAP_AD_TYPE_LENGTH_3, GAP_AD_TYPE_APPEARANCE, 0x80, 0x03,   //0x0380: 896 -> Generic Blood Pressure
    GAP_AD_TYPE_LENGTH_3, GAP_AD_TYPE_APPEARANCE, 0x81, 0x03,   //0x0381: 897 -> Blood Pressure: Arm
    //GAP_AD_TYPE_LENGTH_3, GAP_AD_TYPE_APPEARANCE, 0x82, 0x03,   //0x0382: 898 -> Blood Pressure: Wrist
#elif defined _PROFILE_HTP_
    GAP_AD_TYPE_LENGTH_3, GAP_AD_TYPE_SERVICE_MORE_16B_UUID, GATT_SPEC_SERVICES_HEALTH_THERMOMETER, GATT_SPEC_SERVICES,
    //GAP_AD_TYPE_LENGTH_3, GAP_AD_TYPE_APPEARANCE, 0x00, 0x03,   //0x0300: 768 -> Generic Thermometer
    GAP_AD_TYPE_LENGTH_3, GAP_AD_TYPE_APPEARANCE, 0x01, 0x03,   //0x0301: 769 -> Thermometer: Ear
#elif defined _PROFILE_RSCP_
    GAP_AD_TYPE_LENGTH_3, GAP_AD_TYPE_SERVICE_MORE_16B_UUID, GATT_SPEC_SERVICES_RUNNING_SPEED_AND_CADENCE, GATT_SPEC_SERVICES,
    GAP_AD_TYPE_LENGTH_3, GAP_AD_TYPE_APPEARANCE, 0x40, 0x04,   //0x0440: 1088 -> Running Walking Sensor
    //GAP_AD_TYPE_LENGTH_3, GAP_AD_TYPE_APPEARANCE, 0x41, 0x04,   //0x0441: 1089 -> Running Walking Sensor: In-Shoe
    //GAP_AD_TYPE_LENGTH_3, GAP_AD_TYPE_APPEARANCE, 0x42, 0x04,   //0x0442: 1090 -> Running Walking Sensor: On-Shoe
    //GAP_AD_TYPE_LENGTH_3, GAP_AD_TYPE_APPEARANCE, 0x43, 0x04,   //0x0443: 1091 -> Running Walking Sensor: On-Hip
#elif defined _PROFILE_CSCP_
    GAP_AD_TYPE_LENGTH_3, GAP_AD_TYPE_SERVICE_MORE_16B_UUID, GATT_SPEC_SERVICES_CYCLING_SPEED_AND_CADENCE, GATT_SPEC_SERVICES,
    //GAP_AD_TYPE_LENGTH_3, GAP_AD_TYPE_APPEARANCE, 0x80, 0x04,   //0x0480: 1152 -> Generic: Cycling
    //GAP_AD_TYPE_LENGTH_3, GAP_AD_TYPE_APPEARANCE, 0x81, 0x04,   //0x0481: 1153 -> Cycling: Cycling Computer
    //GAP_AD_TYPE_LENGTH_3, GAP_AD_TYPE_APPEARANCE, 0x82, 0x04,   //0x0482: 1154 -> Cycling: Speed Sensor
    //GAP_AD_TYPE_LENGTH_3, GAP_AD_TYPE_APPEARANCE, 0x83, 0x04,   //0x0483: 1155 -> Cycling: Cadence Sensor
    //GAP_AD_TYPE_LENGTH_3, GAP_AD_TYPE_APPEARANCE, 0x84, 0x04,   //0x0484: 1156 -> Cycling: Power Sensor
    GAP_AD_TYPE_LENGTH_3, GAP_AD_TYPE_APPEARANCE, 0x85, 0x04,   //0x0485: 1157 -> Cycling: Speed and Cadence Sensor
#elif defined _PROFILE_HRP_
    GAP_AD_TYPE_LENGTH_3, GAP_AD_TYPE_SERVICE_MORE_16B_UUID, GATT_SPEC_SERVICES_HEART_RATE, GATT_SPEC_SERVICES,
    GAP_AD_TYPE_LENGTH_3, GAP_AD_TYPE_APPEARANCE, 0x40, 0x03,   //0x0340: 832 -> Generic Heart rate Sensor
    //GAP_AD_TYPE_LENGTH_3, GAP_AD_TYPE_APPEARANCE, 0x41, 0x03,   //0x0341: 833 -> Heart Rate Sensor: Heart Rate Belt
#elif defined _PROFILE_LNS_
    GAP_AD_TYPE_LENGTH_3, GAP_AD_TYPE_SERVICE_MORE_16B_UUID, GATT_SPEC_SERVICES_LOCATION_AND_NAVIGATION, GATT_SPEC_SERVICES,
    //GAP_AD_TYPE_LENGTH_3, GAP_AD_TYPE_APPEARANCE, 0x41, 0x14,   //0x1441: 5185 -> Location Display Device
    GAP_AD_TYPE_LENGTH_3, GAP_AD_TYPE_APPEARANCE, 0x42, 0x14,   //0x1442: 5186 -> Location and Navigation Display Device
    //GAP_AD_TYPE_LENGTH_3, GAP_AD_TYPE_APPEARANCE, 0x43, 0x14,   //0x1443: 5187 -> Location Pod
    //GAP_AD_TYPE_LENGTH_3, GAP_AD_TYPE_APPEARANCE, 0x44, 0x14,   //0x1444: 5188 -> Location and Navigation Pod
#elif defined _PROFILE_GLS_
    GAP_AD_TYPE_LENGTH_3, GAP_AD_TYPE_SERVICE_MORE_16B_UUID, GATT_SPEC_SERVICES_GLUCOSE, GATT_SPEC_SERVICES,
    GAP_AD_TYPE_LENGTH_3, GAP_AD_TYPE_APPEARANCE, 0x00, 0x04,   //0x0400: 1024 -> Generic Glucose Meter
#elif defined _PROFILE_CPS_
    GAP_AD_TYPE_LENGTH_3, GAP_AD_TYPE_SERVICE_MORE_16B_UUID, GATT_SPEC_SERVICES_CYCLING_POWER, GATT_SPEC_SERVICES,
    GAP_AD_TYPE_LENGTH_3, GAP_AD_TYPE_APPEARANCE, 0x84, 0x04,   //0x0484: 1156 -> Cycling: Power Sensor
#else//////cp
//    GAP_AD_TYPE_LENGTH_3, GAP_AD_TYPE_SERVICE_MORE_16B_UUID, GATT_SPEC_SERVICES_DEVICE_INFORMATION, GATT_SPEC_SERVICES,
//    GAP_AD_TYPE_LENGTH_3, GAP_AD_TYPE_APPEARANCE, 0x00, 0x00,   //0x0000: 0 -> Reserved
#endif


//    GAP_AD_TYPE_LENGTH_15, GAP_AD_TYPE_LOCAL_NAME_COMPLETE,
//    0x57, 0x61, 0x68, 0x6f,
//    0x6f, 0x20, 0x48, 0x52,
//    0x4d, 0x20, 0x56, 0x31,
//    0x2e, 0x37,
//    GAP_AD_TYPE_LENGTH_3, GAP_AD_TYPE_MANUFACTURER_SPECIFIC_DATA,
//    0xC0, 0x00,    //https://www.bluetooth.com/specifications/assigned-numbers/company-identifiers/
//    GAP_AD_TYPE_LENGTH_2, GAP_AD_TYPE_TX_POWER_LEVEL, 0x88,
//    GAP_AD_TYPE_LENGTH_5, GAP_AD_TYPE_SLAVE_CONNECTION_INTERVAL_RANGE,
//    0x64, 0x00, 0xC8, 0x00,
//    GAP_AD_TYPE_LENGTH_3, GAP_AD_TYPE_SERVICE_SOLICITATION_16B_UUID,
//    0x09, 0x53,
//    GAP_AD_TYPE_LENGTH_3, GAP_AD_TYPE_SERVICE_DATA, 0x00, 0x05,
};


const uint8_t SET_SCAN_RSP[] =          //Scan response data format. See Bluetooth Spec. Ver5.0 [Vol 3], Part C, Section 11
{
    0x0f,                                  //Length: 1byte; Uint8 HCI_Scan_Rsp_Length;
    0x0e,                                  //AD length, Uint8 HCI_Scan_Rsp[LEN_ADV_SCAN_DATA_MAX];
    GAP_AD_TYPE_LOCAL_NAME_COMPLETE,    //AD Data: 1st byte
    'N', 'o', 'r',                      //AD Data: other bytes
    'd', 'i', 'c', '_', 'B', 'l', 'i', 'n', 'k', 'y',               //"RF_DEMO" => 7 characters, it's name shown on scan list
};


void BleApp_Init(void)
{
    /* register command event callback function */
    setBLE_RegisterBleEvent(BleEvent_Callback);
}

void Ble_Slave_AdvInit(void)
{
    BLE_Adv_Param advParam;

    advParam.Adv_Type = ADV_TYPE_ADV_IND;
    advParam.Adv_Interval_Min = APP_ADV_INTERVAL_MIN;
    advParam.Adv_Interval_Max = APP_ADV_INTERVAL_MAX;
    advParam.Adv_Channel_Map = ADV_CHANNEL_ALL;
    advParam.Adv_Filter_Policy = ADV_FILTER_POLICY_ACCEPT_ALL;

    setBLE_AdvParam(advParam);
}

void Ble_Slave_StartADV(void)
{
    //setBLE_AdvParam(SET_ADV_PARA);                                             //SET_ADV_PARA can be modified by user
    Ble_Slave_AdvInit();
    setBLE_AdvData((uint8_t *)SET_ADV_DATA, sizeof(SET_ADV_DATA));             //SET_ADV_DATA can be modified by user. may depend on profile
    setBLE_ScanRspData((uint8_t *)SET_SCAN_RSP, sizeof(SET_SCAN_RSP));         //SET_SCAN_RSP can be modified by user
    setBLE_AdvEnable();
}


void Ble_Master_Init(void)
{
    BLE_Addr_Param peerAddr_Param = {PUBLIC_ADDR, {TARGET_ADDR0, TARGET_ADDR1, TARGET_ADDR2, TARGET_ADDR3, TARGET_ADDR4, TARGET_ADDR5}};
    BLE_Conn_Param connParam;
    BLE_Scan_Param scanParam;

    connParam.Conn_IntervalMin = APP_CONN_INTERVAL_MIN;
    connParam.Conn_IntervalMax = APP_CONN_INTERVAL_MAX;
    connParam.Conn_Latency = CONN_SLAVE_LATENCY;
    connParam.Conn_SvisionTimeout = CONN_SUPERVISION_TIMEOUT;

    scanParam.Scan_Type = SCAN_TYPE_PASSIVE;
    scanParam.Scan_Interval = SCAN_INTERVAL;
    scanParam.Scan_Window = SCAN_WINDOW;
    scanParam.Scan_FilterPolicy = ADV_FILTER_POLICY_ACCEPT_ALL;

    setBLE_ConnCreate(peerAddr_Param, scanParam, connParam);
}
/*
void Ble_Reset(void)
{
    // reset BLE HW/SW /
    //MCU re-start from main()
    SYS_UnlockReg();
    SYS_ResetChip();
    SYS_LockReg();
}
*/

void Ble_Initial(uint8_t role)  //0: slave, 1: master, 2: both(1, 2 not support yet)
{
    if (role == BLE_LL_SLAVE_ONLY)        //Slave only
    {
        Ble_Slave_StartADV();               //This is for Slave Role. Do advertising, parameter cound be configured in user.c
    }
    else if (role == BLE_LL_MASTER_ONLY)  //Master only
    {
        Ble_Master_Init();                  //This is for Master Role
    }
    else     //Slave and Master
    {
        Ble_Slave_StartADV();
        Ble_Master_Init();
    }
}

void BleApp_Main(void)
{
#ifdef _DEBUG_MSG_USER_
    extern void msg2uart(Uint8 * CodeStr, Uint8 * HexStr, Uint8 HexStrLength);
#endif  //(#ifdef _DEBUG_MSG_USER_)

#if (BLE_DEMO==DEMO_HRS)
    uint32_t TmrGet;
    static uint32_t TmrCmp = 0;

    TmrGet = LLTimeline_Get();
#endif

    if (ble_state == STATE_BLE_STANDBY)
    {
        Ble_Slave_StartADV();
        ble_state = STATE_BLE_ADVERTISING;
    }
    else if (ble_state == STATE_BLE_ADVERTISING)
    {
    }
    else if (ble_state == STATE_BLE_CONNECTION)
    {

#if (BLE_DEMO==DEMO_HRS)  //Application Part - HRS
#ifdef _PROFILE_HRP_
#ifdef _TMR_USE_INTERNAL_
        if (((TmrGet - TmrCmp) & 0x0000FFFF) < 0x00001F40) //<8000 * 125us = 1sec
        {
            return;
        }
        else
        {
            TmrCmp = TmrGet;
        }
#else   //(#ifdef _TMR_USE_INTERNAL_)
        if (((TmrGet - TmrCmp) & 0x00FFFFFF) < 0x00027100) //<160000
        {
            return;
        }
        else
        {
            TmrCmp = TmrGet;
        }
#endif  //(#ifdef _TMR_USE_INTERNAL_)

        //Hearte Rate type
        if ((att_HDL_HRS_HEART_RATE_MEASUREMENT[0]&BIT1) == 0) //initial is 0x14 & 0x02 = 0, toggle "device detected" / "device not detected" information
        {
            att_HDL_HRS_HEART_RATE_MEASUREMENT[0] |= BIT1;  //set Sensor Contact Status bit
        }
        else
        {
            att_HDL_HRS_HEART_RATE_MEASUREMENT[0] &= ~BIT1;  //clear Sensor Contact Status bit
        }

        //If (att_HDL_HRS_CLIENT_CHARACTERISTIC_CONFIGURATION - Notify), then send out data
        if (ATT_HDL_Notify(connId, (uint8_t *)ATT_HDL_HRS_HEART_RATE_MEASUREMENT_INIT, att_HDL_HRS_CLIENT_CHARACTERISTIC_CONFIGURATION, att_HDL_HRS_HEART_RATE_MEASUREMENT, 4) == BLESTACK_STATUS_SUCCESS)
        {
            att_HDL_HRS_HEART_RATE_MEASUREMENT[1]++;             //+1, Heart Rate Data. Here just a simulation, increase 1 about every second
            att_HDL_HRS_HEART_RATE_MEASUREMENT[2]++;             //+1, Heart Rate RR-Interval
        }

#endif  //(#ifdef _PROFILE_HRP_)
#endif    //#if (BLE_DEMO==DEMO_HRS)

#if (BLE_DEMO==DEMO_TRSPX_UART_SLAVE)
        if (tx_data_transmit_enable == 1)
        {
            int i = 0;

            tx_data_transmit_enable = 0;

            //put UART data in att_HDL_UDF01S_UDATN01[]
            for (i = 0; i < txDataLength; i++)
            {
                att_HDL_UDF01S_UDATN01[i] = txDataBuffer[i];
            }

            //Send out UART data by RF
            /* function parameter please see _PROFILE_CSTM_UDF01S_ in "profile_tab.c"  */
            /* Be careful that Central device should enable NOTIFY, then Perapheral device can start sending notification data */
            ATT_HDL_Notify(connId, (uint8_t *)ATT_HDL_UDF01S_UDATN01_INIT, att_HDL_UDF01S_UDATN01_MEASUREMENT_CLIENT_CHARACTERISTIC_CONFIGURATION, att_HDL_UDF01S_UDATN01, txDataLength);
        }
#endif //#if(BLE_DEMO==DEMO_TRSPX_UART_SLAVE)

#ifdef _DEBUG_MSG_USER_SUB1
        if (gateTimeline)
        {
            msg2uart("Time: ", (uint8_t *)&TmrCmp, 4);   //leftest byte is LSB byte
        };
#endif  //(#ifdef _DEBUG_MSG_USER_)

    }
} //end of BleApp_Main()
#pragma pop

//#ifdef _PROFILE_CSTM_UDF01S_
//Transparent data receive callback
void trspx_receive_data_callback(uint8_t length, uint8_t *data)
{
#if (BLE_DEMO==DEMO_TRSPX_LED_SLAVE)   //User demo code example 1 - LED control
    //if(att_HDL_UDF01S_UDATRW01[0]==0x31)  {//user write '1'
    if (data[0] == 0x31)  //user write '1'
    {
        //turn on LED
        LED_1 = 0;
        //LED_2 = 0;
    }
    else
    {
        //turn off LED
        LED_1 = 1;
        //LED_2 = 1;
    }
#elif (BLE_DEMO==DEMO_TRSPX_UART_SLAVE) //User demo code example 2 - transparent transmission UART
    //show received RF data on UART
    UART_TX_Send(length, data);

#endif  //#if (BLE_DEMO==DEMO_TRSPX_LED_SLAVE)
}

#if (BLE_DEMO==DEMO_TRSPX_UART_SLAVE)
//Send out RF data
void trspx_send(uint8_t *data, uint16_t len)
{
    int i = 0;

    tx_data_transmit_enable = 1;
    txDataLength = len;

    //put UART data in att_HDL_UDF01S_UDATN01[]
    for (i = 0; i < len; i++)
    {
        txDataBuffer[i] = data[i];
    }
}
#endif //#if (BLE_DEMO==DEMO_TRSPX_UART_SLAVE)
//#endif  //(#ifdef _PROFILE_CSTM_UDF01S_)



/* GAP Callback Function */
static void BleEvent_Callback(BleCmdEvent event, void *param)
{
    switch (event)
    {
    case BLECMD_EVENT_ADV_COMPLETE:
        if (ble_state == STATE_BLE_ADVERTISING)
        {
            printf("Advertising...\n");
        }
        break;

    case BLECMD_EVENT_SCAN_REPORT:
    {
//        BLE_Event_Scan_Report *rpt = (BLE_Event_Scan_Report *)param;

//        printf("RSSI:%d\n",rpt->Rpt_Rssi);
    }
    break;

    case BLECMD_EVENT_CONN_COMPLETE:
    {
        BLE_Event_ConnParam *connParam = (BLE_Event_ConnParam *)param;

        connId = connParam->connId;
        ble_state = STATE_BLE_CONNECTION;

        printf("Status:0x%02x\n", connParam->status);
        printf("Connected to %02x:%02x:%02x:%02x:%02x:%02x\n", connParam->peerAddr.addr[5],
               connParam->peerAddr.addr[4],
               connParam->peerAddr.addr[3],
               connParam->peerAddr.addr[2],
               connParam->peerAddr.addr[1],
               connParam->peerAddr.addr[0]);
    }
    break;

    case BLECMD_EVENT_DISCONN_COMPLETE:
    {
        BLE_Event_DisconnParam *disconnParam = (BLE_Event_DisconnParam *)param;
        ble_state = STATE_BLE_STANDBY;

        printf("Disconnected, ID:%d, Reason:0x%X\n", disconnParam->connId, disconnParam->disconnectReason);
    }
    break;

    case BLECMD_EVENT_CONN_UPDATE_COMPLETE:
    {
        BLE_Event_ConnUpdateParam *updateConnParam = (BLE_Event_ConnUpdateParam *)param;
        printf("Connection updated, ID:%d, Status:%d, Interval:%d, Latency:%d, Timeout:%d\n", updateConnParam->connId, updateConnParam->status, updateConnParam->connInterval, updateConnParam->connLatency, updateConnParam->supervisionTimeout);
    }
    break;

    case BLECMD_EVENT_PHY_UPDATE_COMPLETE:
    {
        BLE_Event_Phy_Update_Param *phy = (BLE_Event_Phy_Update_Param *)param;
        printf("STATUS: %d, TX PHY: %d, RX PHY: %d\n", phy->status, phy->phyParam.tx_Phy, phy->phyParam.tx_Phy);
    }
    break;

    case BLECMD_EVENT_PHY_READ_COMPLETE:
    {
        BLE_Event_Phy_Param *phy = (BLE_Event_Phy_Param *)param;
        printf("ID:%d, TX:%d, RX:%d\n", phy->connId, phy->tx_Phy, phy->tx_Phy);
    }
    break;

    case BLECMD_EVENT_READ_RSSI_COMPLETE:
        break;

    case BLECMD_EVENT_STK_GEN_METHOD:
    {
        BLE_Event_Stk_Gen_Method *passkey_method = (BLE_Event_Stk_Gen_Method *)param;
        if (passkey_method->Stk_Gen_Method == PASSKEY_ENTRY)
        {
            //Start scanning user-entered passkey.
        }
        else if (passkey_method->Stk_Gen_Method == PASSKEY_DISPLAY)
        {
            //user can generate a 6-digit random code, and display it for pairing.
        }
    }
    break;

    case BLECMD_EVENT_PASSKEY_CONFIRM:
    {
        BLE_Event_PassKey_Confirm *event_param = (BLE_Event_PassKey_Confirm *)param;

        //enter a scanned Passkey or use a randomly generaated passkey.
        setBLE_Pairing_PassKey(event_param->connId, HOGP_PAIRING_KEY);
    }
    break;

    case BLECMD_EVENT_AUTH_STATUS:
    {
        BLE_Event_Auth_Status *auth_result = (BLE_Event_Auth_Status *)param;
        printf("AUTH Report, ID:%d , STATUS:%d\n", auth_result->connId, auth_result->status);
    }
    break;

    case BLECMD_EVENT_EXCHANGE_MTU_SIZE:
        break;

    case BLECMD_EVENT_DATA_LENGTH_UPDATE:
        break;

    default:
        break;
    }
}

