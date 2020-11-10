/*******************************************************************
 *
 * File Name  : KNL_PBLC.H
 * Description: This file provide the kernel declerations that all
 *              layers needed.
 *
 * $Log:  $
 *
 *
 *
 *******************************************************************
 *
 *      Copyright (c) 2020, All Right Reserved
 *      Rafael Microelectronics Co. Ltd.
 *      Taiwan, R.O.C.
 *
 *******************************************************************/
/*! \file knl_pblc.h
    \brief  Provide the kernel declerations that all
            layers needed.
*/

#ifndef _KNL_PBLC_H_
#define _KNL_PBLC_H_

#include "BleAppSetting.h"
#include "ble_basicType.h"
#include "_ble_host.h"

//#ifdef _USED_EXCEPT_SDK_
/*--------------------------------------*/
/* Block number definition              */
/*--------------------------------------*/
/* mailbox port(queue) id */    /*    Priority  */
#define Q_2LL           0       /*    highest   */
#define Q_2L2CAP        1       /*    middle    */
#define Q_2HOST         2       /*    middle    */
#define Q_2MMI          3       /*    lowest    */
#ifndef _HCI_NEW_
#define MAX_QUEUE_NO    4       /* maximum message queue number */
#else   //#ifndef _HCI_NEW_
#define Q_2HCI          4       /*    lowest    */
#define MAX_QUEUE_NO    5       /* maximum message queue number */
#endif  //#ifndef _HCI_NEW_


#define MAX_MBLK_RSV_LL_ROLE    5
#define MAX_MBLK_RSV_LL     (MAX_MBLK_RSV_LL_ROLE*5+5)  /* reserved message block number, should be same as MAX_MBLK_RSV_LL_ROLE*MAX_NUM_CONN_HDL+5 */
#define MAX_MBLK_RSV_L2     MAX_MBLK_RSV_LL
#define MAX_MBLK_RSV_L1     (MAX_MBLK_RSV_LL_ROLE+5)
//#define MAX_MBLK_NO         (MAX_MBLK_RSV_LL+6*5)          /* maximum message block number */


#define LEN_BD_ACCESS_ADDR      4
#define LEN_INIT_LL_DATA        22
#define LEN_CONN_DATA_MAX       27
#define LEN_CONN_MIC            4

#define LEN_CONN_DATA_INITIAL       27
#define LEN_CONN_DATA_DEFAULT       251
#define LEN_CONN_PKT_EXCEPT_DATA    10
#define LEN_CONN_MIC_REF            4
#define LEN_CONN_PKT_EXCEPT_DATA_W_CCM  (LEN_CONN_PKT_EXCEPT_DATA+LEN_CONN_MIC_REF)
#define LEN_LL_CH_MAP           5
#define LEN_LL_INSTANT          2
//#endif  //(#ifdef _USED_EXCEPT_SDK_)

#define LEN_BD_ADDR             6
#define LEN_ADV_SCAN_DATA_MAX   31
//#define MAX_MBLK_SIZE           (LEN_BD_ADDR+LEN_ADV_SCAN_DATA_MAX+7)      //7: rest of MHC_Le_Adv_Report_Para, 0:padding for 32bit MCU

/*******************************************************************
 *              TYPE DEFINITION
 *******************************************************************/
//#ifdef _USED_EXCEPT_SDK_

typedef struct CmdTimerMessage
{
    Uint8   Event;
    Uint32  Current_T;
    Uint32  TimeOut_Base;
} CmdTimerMessage;


typedef union Udat32
{
    Uint32 Val32;
    Uint16 Val16[2];
    Uint8 Val8[4];
} Udat32;


typedef union Udat16
{
    Uint16 Val16;
    Uint8 Val8[2];
} Udat16;

typedef struct TBLK_LL
{
    Uint8   TmrId;      /* Timer Id */
    Uint8   ConnId;      /* Timer Id */
    Uint8   Next;       /* next TBLK in list */
    Uint32  Ticks;      /* timeout duration (ticks no.) */
} TBLK_LL;
typedef TBLK_LL TBLK_LLx;
//typedef TBLK_LL xdata TBLK_LLx;
//#endif  //(#ifdef _USED_EXCEPT_SDK_)

#ifdef _HCI_HW_
#define HCLL_HCI_HW_OPCODE      Uint8 HCI_Pckt_Typ; Uint8 HCI_Ocf; Uint8 HCI_Ogf; Uint8 HCI_Cmd_Length;
#define SIZE_HCLL_HCI_HW_OPCODE     4
#define HCI_CONN_HDL            Uint8 HCI_Conn_Hdl_L; Uint8 HCI_Conn_Hdl_H;
#define SIZE_HCI_CONN_HDL           2
#define MHC_HCI_HW_OPCODE       Uint8 HCI_Ocf; Uint8 HCI_Ogf;
#define SIZE_MHC_HCI_HW_OPCODE      2
#define MHC_HCI_HW_PKT_TYPE     Uint8 HCI_Pckt_Typ; Uint8 HCI_Event_Code;
#define SIZE_MHC_HCI_HW_PKT_TYPE    2
#define HCI_DATA_CONN_HDL       Uint8 HCI_Pckt_Typ; Uint8 HCI_Conn_Hdl_L; Uint8 HCI_Conn_Hdl_H;
#define SIZE_HCI_DATA_CONN_HDL      3
#define HCI_DATA_PKT_LTH_BYTE   Uint8 HCI_DataPkt_LthL; Uint8 HCI_DataPkt_LthH;
#define SIZE_HCI_DATA_PKT_LTH_BYTE  2
//#define HCI_DATA_CONN_IDX_HDL   Uint8 HCI_Pckt_Typ; Uint8 HCI_Conn_Idx; Uint8 HCI_Conn_Hdl_H;
#define HCI_DATA_CONN_IDX_HDL   Uint8 HCI_Pckt_Typ; Uint8 HCI_Data_Idx0; Uint8 HCI_Data_Idx1;
#define SIZE_HCI_DATA_CONN_IDX_HDL  3
#else
#define HCLL_HCI_HW_OPCODE
#define SIZE_HCLL_HCI_HW_OPCODE     0
#define HCI_CONN_HDL            Uint8 HCI_Conn_Hdl_L;
#define SIZE_HCI_CONN_HDL           1
#define MHC_HCI_HW_OPCODE       Uint8 By_Primitive;
#define SIZE_MHC_HCI_HW_OPCODE      1
#define MHC_HCI_HW_PKT_TYPE
#define SIZE_MHC_HCI_HW_PKT_TYPE    0
#define HCI_DATA_CONN_HDL       Uint8 HCI_Conn_Hdl_L; Uint8 HCI_Conn_Hdl_H;    //HCI_Conn_Hdl_H: LL_LLid
#define SIZE_HCI_DATA_CONN_HDL      2
#define HCI_DATA_PKT_LTH_BYTE   Uint8 HCI_DataPkt_LthL;
#define SIZE_HCI_DATA_PKT_LTH_BYTE  1
//#define HCI_DATA_CONN_IDX_HDL   Uint8 HCI_Conn_Idx; Uint8 HCI_Conn_Hdl_H;
#define HCI_DATA_CONN_IDX_HDL   Uint8 HCI_Data_Idx1; Uint8 HCI_Data_Idx0;
#define SIZE_HCI_DATA_CONN_IDX_HDL  2
#endif  //#ifdef _HCI_HW_

typedef struct Len_Def_MHC_Le_Adv_Report_Para
{
    MHC_HCI_HW_PKT_TYPE
    Uint8 HCI_Para_Length;
    Uint8 HCI_Subevent_Code;
    Uint8 HCI_Num_Reports;
    Uint8 HCI_Event_Type0;
    Uint8 HCI_Addr_Type0;
    Uint8 HCI_Addr0[LEN_BD_ADDR];
    Uint8 HCI_Length0;
    Uint8 HCI_Data0[LEN_ADV_SCAN_DATA_MAX+1];      //1: Uint8 HCI_RSSI0;
    //Uint8 HCI_RSSI0;
} Len_Def_MHC_Le_Adv_Report_Para;
#define MAX_MBLK_SIZE           (sizeof(Len_Def_MHC_Le_Adv_Report_Para))

struct HCLL_Disconnect_Para                              //OGF: Link Control, 0x01
{
    HCLL_HCI_HW_OPCODE
    HCI_CONN_HDL
    Uint8 HCI_Reason;
};


struct HCLL_Read_Remote_Ver_Info_Para                    //OGF: Link Control, 0x01
{
    HCLL_HCI_HW_OPCODE
    HCI_CONN_HDL
};


struct HCLL_Set_Event_Mask_Para                          //OGF: Controller & Baseband, 0x03
{
    HCLL_HCI_HW_OPCODE
    Uint8 HCI_Event_Mask[8];
};


struct HCLL_Reset_Para                                   //OGF: Controller & Baseband, 0x03
{
    HCLL_HCI_HW_OPCODE
    Uint8 HCI_ResvMem[(MAX_MBLK_SIZE-SIZE_HCLL_HCI_HW_OPCODE)];
};


struct HCLL_Read_Transmit_Pwr_Level_Para                 //OGF: Controller & Baseband, 0x03
{
    HCLL_HCI_HW_OPCODE
    HCI_CONN_HDL
    Uint8 HCI_Type;
};


struct HCLL_Set_Ctrler_To_Host_Flow_Ctrl_Para            //OGF: Controller & Baseband, 0x03
{
    HCLL_HCI_HW_OPCODE
    Uint8 HCI_FlowCtrlEn;
};


struct HCLL_Host_Buffer_Size_Para                        //OGF: Controller & Baseband, 0x03
{
    HCLL_HCI_HW_OPCODE
    Uint8 HCI_HostACLDataPcktLengthL;
    Uint8 HCI_HostACLDataPcktLengthH;
    Uint8 HCI_HostSyncDataPcktLength;
    Uint8 HCI_HostTotalNumACLDataPcktL;
    Uint8 HCI_HostTotalNumACLDataPcktH;
    Uint8 HCI_HostTotalNumSyncDataPcktL;
    Uint8 HCI_HostTotalNumSyncDataPcktH;
};


struct HCLL_Host_Num_Of_Completed_Packets_Para           //OGF: Controller & Baseband, 0x03
{
    HCLL_HCI_HW_OPCODE
    Uint8 HCI_Num_Of_Hdl;
    HCI_CONN_HDL
    Uint16 HCI_HostNumCpltdPkt0;
};


struct HCLL_Set_Event_Mask_Page_2_Para                   //OGF: Controller & Baseband, 0x03, Spec. 4.1
{
    HCLL_HCI_HW_OPCODE
    Uint8 HCI_Event_Mask_Page2[8];
};


struct HCLL_Read_Le_Host_Support_Para                    //OGF: Controller & Baseband, 0x03
{
    HCLL_HCI_HW_OPCODE
    Uint8 HCI_ResvMem[(MAX_MBLK_SIZE-SIZE_HCLL_HCI_HW_OPCODE)];
};


struct HCLL_Write_Le_Host_Support_Para                   //OGF: Controller & Baseband, 0x03
{
    HCLL_HCI_HW_OPCODE
    Uint8 HCI_LE_Supp_Host;
    Uint8 HCI_Simul_LE_Host;
};


struct HCLL_Read_Authen_Payload_Timeout_Para             //OGF: Controller & Baseband, 0x03, Spec. 4.1
{
    HCLL_HCI_HW_OPCODE
    HCI_CONN_HDL
};


struct HCLL_Write_Authen_Payload_Timeout_Para            //OGF: Controller & Baseband, 0x03, Spec. 4.1
{
    HCLL_HCI_HW_OPCODE
    HCI_CONN_HDL
    Uint8 HCI_AuthenPayloadTimeoutL;
    Uint8 HCI_AuthenPayloadTimeoutH;
};


struct HCLL_Read_Local_Ver_Info_Para                     //OGF: Informational Parameters, 0x04
{
    HCLL_HCI_HW_OPCODE
    Uint8 HCI_ResvMem[(MAX_MBLK_SIZE-SIZE_HCLL_HCI_HW_OPCODE)];
};


struct HCLL_Read_Local_Supported_Cmd_Para                //OGF: Informational Parameters, 0x04
{
    HCLL_HCI_HW_OPCODE
    Uint8 HCI_ResvMem[(MAX_MBLK_SIZE-SIZE_HCLL_HCI_HW_OPCODE)];
};


struct HCLL_Read_Local_Supported_Feat_Para               //OGF: Informational Parameters, 0x04
{
    HCLL_HCI_HW_OPCODE
    Uint8 HCI_ResvMem[(MAX_MBLK_SIZE-SIZE_HCLL_HCI_HW_OPCODE)];
};


struct HCLL_Read_Buffer_Size_Para                        //OGF: Informational Parameters, 0x04
{
    HCLL_HCI_HW_OPCODE
    Uint8 HCI_ResvMem[(MAX_MBLK_SIZE-SIZE_HCLL_HCI_HW_OPCODE)];
};


struct HCLL_Read_Bd_Addr_Para                            //OGF: Informational Parameters, 0x04
{
    HCLL_HCI_HW_OPCODE
    Uint8 HCI_ResvMem[(MAX_MBLK_SIZE-SIZE_HCLL_HCI_HW_OPCODE)];
};


struct HCLL_Read_Rssi_Para                               //OGF: Status Parameters, 0x05
{
    HCLL_HCI_HW_OPCODE
    HCI_CONN_HDL
};


struct HCLL_LE_Set_Event_Mask_Para                       //OGF: LE Controller, 0x08
{
    HCLL_HCI_HW_OPCODE
    Uint8 HCI_LE_Event_Mask[8];
};


struct HCLL_LE_Read_Buffer_Size_Para                     //OGF: LE Controller, 0x08
{
    HCLL_HCI_HW_OPCODE
    Uint8 HCI_ResvMem[(MAX_MBLK_SIZE-SIZE_HCLL_HCI_HW_OPCODE)];
};


struct HCLL_LE_Read_Local_Supported_Feat_Para            //OGF: LE Controller, 0x08
{
    HCLL_HCI_HW_OPCODE
    Uint8 HCI_ResvMem[(MAX_MBLK_SIZE-SIZE_HCLL_HCI_HW_OPCODE)];
};


struct HCLL_LE_Set_Random_Addr_Para                   //OGF: LE Controller, 0x08
{
    HCLL_HCI_HW_OPCODE
    Uint8 HCI_Random_Addr[LEN_BD_ADDR];
};


struct HCLL_LE_Set_Adv_Param_Para                        //OGF: LE Controller, 0x08
{
    HCLL_HCI_HW_OPCODE
    Uint16 HCI_Adv_Interval_Min;
    Uint16 HCI_Adv_Interval_Max;
    Uint8 HCI_Adv_Type;
    Uint8 HCI_Own_Addr_Type;
    Uint8 HCI_Direct_Addr_Type;
    Uint8 HCI_Direct_Addr[LEN_BD_ADDR];
    Uint8 HCI_Adv_Channel_Map;
    Uint8 HCI_Adv_Filter_Policy;
};
#define LEN_HCLL_LE_SET_ADV_PARAM_PARA       (2*2+1*5+1*LEN_BD_ADDR)

struct HCLL_LE_Read_Adv_Ch_Tx_Pwr_Para                   //OGF: LE Controller, 0x08
{
    HCLL_HCI_HW_OPCODE
    Uint8 HCI_ResvMem[(MAX_MBLK_SIZE-SIZE_HCLL_HCI_HW_OPCODE)];
};


struct HCLL_LE_Set_Adv_Data_Para                         //OGF: LE Controller, 0x08
{
    HCLL_HCI_HW_OPCODE
    Uint8 HCI_Adv_Data_Length;
    Uint8 HCI_Adv_Data[LEN_ADV_SCAN_DATA_MAX];
};


struct HCLL_LE_Set_Scan_Response_Data_Para               //OGF: LE Controller, 0x08
{
    HCLL_HCI_HW_OPCODE
    Uint8 HCI_Scan_Rsp_Length;
    Uint8 HCI_Scan_Rsp[LEN_ADV_SCAN_DATA_MAX];
};


struct HCLL_LE_Set_Advertise_Enable_Para                //OGF: LE Controller, 0x08
{
    HCLL_HCI_HW_OPCODE
    Uint8 HCI_Adv_En;
};


struct HCLL_LE_Set_Scan_Param_Para                      //OGF: LE Controller, 0x08
{
    HCLL_HCI_HW_OPCODE
    Uint8 HCI_LEScanType;
#ifdef _HCI_HW_
    __packed Uint16 HCI_LE_Scan_Interval;
    __packed Uint16 HCI_LE_Scan_Window;
    Uint8 HCI_Own_Addr_Type;
    Uint8 HCI_ScannFilterPolicy;
#else
    Uint8 Padding0;
    Uint16 HCI_LE_Scan_Interval;
    Uint16 HCI_LE_Scan_Window;
    Uint8 HCI_Own_Addr_Type;
    Uint8 HCI_ScannFilterPolicy;
#endif
};


struct HCLL_LE_Set_Scan_Enable_Para                     //OGF: LE Controller, 0x08
{
    HCLL_HCI_HW_OPCODE
    Uint8 HCI_LE_Scan_En;
    Uint8 HCI_Filter_Dupl;
};


struct HCLL_LE_Create_Conn_Para                          //OGF: LE Controller, 0x08
{
    HCLL_HCI_HW_OPCODE
    Uint16 HCI_LE_Scan_Interval;
    Uint16 HCI_LE_Scan_Window;
    Uint8 HCI_InitFilterPolicy;
    Uint8 HCI_PeerAddrType;
    Uint8 HCI_PeerAddr[LEN_BD_ADDR];
    Uint8 HCI_Own_Addr_Type;
#ifdef _HCI_HW_
    __packed Uint16 HCI_ConnIntervalMin;
    __packed Uint16 HCI_ConnIntervalMax;
    __packed Uint16 HCI_ConnLatency;
    __packed Uint16 HCI_SvisionTimeout;
    __packed Uint16 HCI_MinCELength;
    __packed Uint16 HCI_MaxCELength;
#else
    Uint8 Padding0;
    Uint16 HCI_ConnIntervalMin;
    Uint16 HCI_ConnIntervalMax;
    Uint16 HCI_ConnLatency;
    Uint16 HCI_SvisionTimeout;
    Uint16 HCI_MinCELength;
    Uint16 HCI_MaxCELength;
#endif
};


struct HCLL_LE_Create_Conn_Cancel_Para                   //OGF: LE Controller, 0x08
{
    HCLL_HCI_HW_OPCODE
    Uint8 HCI_ResvMem[(MAX_MBLK_SIZE-SIZE_HCLL_HCI_HW_OPCODE)];
};


struct HCLL_LE_Read_White_List_Size_Para                 //OGF: LE Controller, 0x08
{
    HCLL_HCI_HW_OPCODE
    Uint8 HCI_ResvMem[(MAX_MBLK_SIZE-SIZE_HCLL_HCI_HW_OPCODE)];
};


struct HCLL_LE_Clear_White_List_Para                     //OGF: LE Controller, 0x08
{
    HCLL_HCI_HW_OPCODE
    Uint8 HCI_ResvMem[(MAX_MBLK_SIZE-SIZE_HCLL_HCI_HW_OPCODE)];
};


struct HCLL_LE_Add_Device_To_White_List_Para             //OGF: LE Controller, 0x08
{
    HCLL_HCI_HW_OPCODE
    Uint8 HCI_Addr_Type;
    Uint8 HCI_Addr[LEN_BD_ADDR];
};


struct HCLL_LE_Rmv_Device_From_White_List_Para           //OGF: LE Controller, 0x08
{
    HCLL_HCI_HW_OPCODE
    Uint8 HCI_Addr_Type;
    Uint8 HCI_Addr[LEN_BD_ADDR];
};


struct HCLL_LE_Conn_Update_Para                          //OGF: LE Controller, 0x08
{
    HCLL_HCI_HW_OPCODE
    HCI_CONN_HDL
#ifndef _HCI_HW_
    Uint8 padding0;
#endif
    Uint16 HCI_ConnIntervalMin;
    Uint16 HCI_ConnIntervalMax;
    Uint16 HCI_ConnLatency;
    Uint16 HCI_SvisionTimeout;
    Uint16 HCI_MinCELength;
    Uint16 HCI_MaxCELength;
};


struct HCLL_LE_Set_Host_Ch_Classification_Para           //OGF: LE Controller, 0x08
{
    HCLL_HCI_HW_OPCODE
#ifdef _HCI_HW_
    Uint8 HCI_Channel_Map[5];
    __packed Uint16 LL_EventCounterUpd;
#else
    Uint8 padding0;
    Uint8 HCI_Channel_Map[5];
    Uint16 LL_EventCounterUpd;
#endif
};


struct HCLL_LE_Read_Ch_map_Para                          //OGF: LE Controller, 0x08
{
    HCLL_HCI_HW_OPCODE
    HCI_CONN_HDL
};


struct HCLL_LE_Read_Remote_Used_Feat_Para                //OGF: LE Controller, 0x08
{
    HCLL_HCI_HW_OPCODE
    HCI_CONN_HDL
};


struct HCLL_LE_Encrypt_Para                              //OGF: LE Controller, 0x08
{
    HCLL_HCI_HW_OPCODE
    Uint8 HCI_Key[16];
    Uint8 HCI_Plain_Data[16];
};


struct HCLL_LE_Rand_Para                                 //OGF: LE Controller, 0x08
{
    HCLL_HCI_HW_OPCODE
    Uint8 HCI_ResvMem[(MAX_MBLK_SIZE-SIZE_HCLL_HCI_HW_OPCODE)];
};


struct HCLL_LE_Start_Encryption_Para                     //OGF: LE Controller, 0x08
{
    HCLL_HCI_HW_OPCODE
    HCI_CONN_HDL
    Uint8 HCI_Random_Num[8];
    Uint8 HCI_Encry_Div[2];
    Uint8 HCI_LongTermKey[16];
};


struct HCLL_LE_Long_Term_key_Req_Reply_Para              //OGF: LE Controller, 0x08
{
    HCLL_HCI_HW_OPCODE
    HCI_CONN_HDL
    Uint8 HCI_LongTermKey[16];
};


struct HCLL_LE_Long_Term_key_Req_Neg_Reply_Para          //OGF: LE Controller, 0x08
{
    HCLL_HCI_HW_OPCODE
    HCI_CONN_HDL
};


struct HCLL_LE_Read_Supported_States_Para                //OGF: LE Controller, 0x08
{
    HCLL_HCI_HW_OPCODE
    Uint8 HCI_ResvMem[(MAX_MBLK_SIZE-SIZE_HCLL_HCI_HW_OPCODE)];
};


struct HCLL_LE_Receiver_Test_Para                        //OGF: LE Controller, 0x08
{
    HCLL_HCI_HW_OPCODE
    Uint8 HCI_RX_Channel;
};


struct HCLL_LE_Transmitter_Test_Para                     //OGF: LE Controller, 0x08
{
    HCLL_HCI_HW_OPCODE
    Uint8 HCI_TX_Channel;
    Uint8 HCI_LengthOfTestData;
    Uint8 HCI_Packet_Payload;
};


struct HCLL_LE_Test_End_Para                             //OGF: LE Controller, 0x08
{
    HCLL_HCI_HW_OPCODE
    Uint8 HCI_ResvMem[(MAX_MBLK_SIZE-SIZE_HCLL_HCI_HW_OPCODE)];
};


struct HCLL_LE_Remote_Conn_Param_Req_Reply_Para          //OGF: LE Controller, 0x08, Spec. 4.1
{
    HCLL_HCI_HW_OPCODE
    HCI_CONN_HDL
    Uint16 HCI_ConnIntervalMin;
    Uint16 HCI_ConnIntervalMax;
    Uint16 HCI_ConnLatency;
    Uint16 HCI_SvisionTimeout;
    Uint16 HCI_MinCELength;
    Uint16 HCI_MaxCELength;
};
#define LEN_HCLL_LE_REMOTE_CONN_PARAM_REQ_REPLY_PARA        (2*6)
#define FLD_BEGIN_HCLL_LE_REMOTE_CONN_PARAM_REQ_REPLY_PARA  (SIZE_HCLL_HCI_HW_OPCODE+SIZE_HCI_CONN_HDL)


struct HCLL_LE_Remote_Conn_Param_Req_Neg_Reply_Para      //OGF: LE Controller, 0x08, Spec. 4.1
{
    HCLL_HCI_HW_OPCODE
    HCI_CONN_HDL
    Uint8 HCI_Reason;
};


#define SIZE_MBLK_ACL_DATA_UNIT (MAX_MBLK_SIZE-SIZE_HCI_DATA_CONN_HDL-SIZE_HCI_DATA_PKT_LTH_BYTE)
struct HCLL_LE_ACL_Data_Pkt_Para
{
    HCI_DATA_CONN_HDL
    HCI_DATA_PKT_LTH_BYTE
    Uint8 HCI_DataPkt_Data[SIZE_MBLK_ACL_DATA_UNIT];
};


struct HCLL_LE_ACL_Data_Pkt_Extend_Para             //format like HCLL_LE_ACL_Data_Pkt_Para
{
    HCI_DATA_CONN_IDX_HDL
    HCI_DATA_PKT_LTH_BYTE
    Uint8 HCI_DataPkt_Data[SIZE_MBLK_ACL_DATA_UNIT];
};


struct LLHC_LE_Ctrl_Pkt_Para
{
    HCI_DATA_CONN_HDL
    HCI_DATA_PKT_LTH_BYTE
    Uint8 HCI_CtrlPkt_Opcode;
    Uint8 HCI_CtrlPkt_Data[MAX_MBLK_SIZE-SIZE_HCI_DATA_CONN_HDL-SIZE_HCI_DATA_PKT_LTH_BYTE-1];
};


struct HCLL_LE_Ctrl_Pkt_Para
{
    HCI_DATA_CONN_HDL
    HCI_DATA_PKT_LTH_BYTE
    Uint8 HCI_CtrlPkt_Opcode;
    Uint8 HCI_CtrlPkt_Data[MAX_MBLK_SIZE-SIZE_HCI_DATA_CONN_HDL-SIZE_HCI_DATA_PKT_LTH_BYTE-1];
};


struct HCLL_LE_Ctrl_Pkt_Conn_Update_Para
{
    HCI_DATA_CONN_HDL
    HCI_DATA_PKT_LTH_BYTE
    Uint8 Padding0;
    Uint8 HCI_CtrlPkt_Opcode;
    Uint8 HCI_CtrlPkt_WinSize;
    Uint16 HCI_CtrlPkt_WinOffset;
    Uint16 HCI_CtrlPkt_Interval;
    Uint16 HCI_CtrlPkt_Latency;
    Uint16 HCI_CtrlPkt_Timeout;
    Uint16 HCI_CtrlPkt_Instant;
};


struct HCLL_LE_Ctrl_Pkt_Set_Host_Ch_Para
{
    HCI_DATA_CONN_HDL
    HCI_DATA_PKT_LTH_BYTE
    Uint8 HCI_CtrlPkt_Opcode;
    Uint8 HCI_CtrlPkt_ChM[5];
#ifdef _HCI_HW_
    __packed Uint16 HCI_CtrlPkt_Instant;
    Uint8 Padding0;
#else
    Uint8 Padding0;
    Uint16 HCI_CtrlPkt_Instant;
#endif
    Uint8 HCI_ResvMem[MAX_MBLK_SIZE-SIZE_HCI_DATA_CONN_HDL-SIZE_HCI_DATA_PKT_LTH_BYTE-9];
};


//#ifdef _USED_EXCEPT_SDK_
struct MLL_LE_Conn_Para_SMP
{
    Uint8 LL_Random_Num[8];
    Uint8 LL_Encry_Div[2];
    Uint8 LL_SMP_SKD[8];                //8: LEN_SMP_SKDM
    Uint8 LL_SMP_IV[(4+4)];             //4: LEN_SMP_IVS, LEN_SMP_IVM
    Uint8 LL_SMP_Key[16];               //16: LEN_AES_KEY
};


struct MLL_LL_Enc_Req_Para
{
    Uint8 LL_Conn_No;
    Uint8 ResvMem[(MAX_MBLK_SIZE-1)];
};


struct MLL_LE_CCM_Manual_Para
{
    HCI_DATA_CONN_HDL
    HCI_DATA_PKT_LTH_BYTE
    Uint8 *mblk_LL_ConnDataQ;                         //LL_PATCH_MIC_FAIL_DATA_SIZE_LESS, less than 7 will cause MIC error
};


struct LLEC_LE_CCM_Manual_Para
{
    HCI_DATA_CONN_HDL
    HCI_DATA_PKT_LTH_BYTE
    Uint8 HCI_DataPkt_Data[7];          //LL_PATCH_MIC_FAIL_DATA_SIZE_LESS, less than LEN_LL_PATCH_MIC_FAIL_CONDITION+1 will cause MIC error
    Uint32 LL_SMP_packetCounterT;       //LL_SMP_packetCounterR or LL_SMP_packetCounterT
    Uint8  LL_SMP_packetCounterTd;
    Uint8 LL_SMP_IV[8];                 //LEN_SMP_IVS+LEN_SMP_IVM
    Uint8 HCI_DataPkt_Length_CCM;       //should be HCI_DataPkt_LthL+4, 4: MIC length
    Uint8 HCI_DataPkt_Data_CCM[16];     //large than 7+4, 4: MIC length
};


struct MHC_Cmd_Complete_Para
{
    MHC_HCI_HW_PKT_TYPE
    Uint8 HCI_Para_Length;
    Uint8 HCI_Num_Cmd_Pckts;
    MHC_HCI_HW_OPCODE
    Uint8 HCI_EventPara[(MAX_MBLK_SIZE-6)];
};

struct MHC_Cmd_Status_Para
{
    MHC_HCI_HW_PKT_TYPE
    Uint8 HCI_Para_Length;
    Uint8 HCI_Status;
    Uint8 HCI_Num_Cmd_Pckts;
    MHC_HCI_HW_OPCODE
    Uint8 HCI_ResvMem[(MAX_MBLK_SIZE-7)];
};

struct MHC_Disconn_Complete_Para
{
    MHC_HCI_HW_PKT_TYPE
    Uint8 HCI_Para_Length;
    Uint8 HCI_Status;
    HCI_CONN_HDL
    Uint8 HCI_Reason;
    Uint8 HCI_ResvMem[(MAX_MBLK_SIZE-7)];
};

struct MHC_Num_Of_Completed_Pckts_Para
{
    MHC_HCI_HW_PKT_TYPE
    Uint8 HCI_Para_Length;
    Uint8 HCI_Num_of_Hdl;
    HCI_CONN_HDL
    Uint8 HCI_HC_Num_Cmplt_Pkt_L;
    Uint8 HCI_HC_Num_Cmplt_Pkt_H;
    Uint8 HCI_ResvMem[(MAX_MBLK_SIZE-8)];
};


struct MHC_Read_Remote_Ver_Info_Complete_Para
{
    MHC_HCI_HW_PKT_TYPE
    Uint8 HCI_Para_Length;
    Uint8 HCI_Status;
    HCI_CONN_HDL
    Uint8 HCI_Ver;
    Uint8 HCI_Manufct_Name_L;
    Uint8 HCI_Manufct_Name_H;
    Uint8 HCI_SubVer_L;
    Uint8 HCI_SubVer_H;
    Uint8 HCI_ResvMem[(MAX_MBLK_SIZE-11)];
};

struct MHC_Encrypt_Change_Para
{
    MHC_HCI_HW_PKT_TYPE
    Uint8 HCI_Para_Length;
    Uint8 HCI_Status;
    HCI_CONN_HDL
    Uint8 HCI_Encrpt_En;
    Uint8 HCI_ResvMem[(MAX_MBLK_SIZE-7)];
};

struct MHC_Encrypt_Key_Refresh_Complete_Para
{
    MHC_HCI_HW_PKT_TYPE
    Uint8 HCI_Para_Length;
    Uint8 HCI_Status;
    HCI_CONN_HDL
    Uint8 HCI_ResvMem[(MAX_MBLK_SIZE-6)];
};

struct MHC_Data_Buffer_Overflow_Para
{
    MHC_HCI_HW_PKT_TYPE
    Uint8 HCI_Para_Length;
    Uint8 HCI_Link_Type;
    Uint8 HCI_ResvMem[(MAX_MBLK_SIZE-4)];
};

struct MHC_Hardware_Error_Para
{
    MHC_HCI_HW_PKT_TYPE
    Uint8 HCI_Para_Length;
    Uint8 HCI_EventPara;
    Uint8 HCI_ResvMem[(MAX_MBLK_SIZE-4)];
};

struct MHC_Le_Conn_Complete_Para
{
    MHC_HCI_HW_PKT_TYPE
    Uint8 HCI_Para_Length;
    Uint8 HCI_Subevent_Code;
    Uint8 HCI_Status;
    HCI_CONN_HDL
    Uint8 HCI_Role;
    Uint8 HCI_PeerAddrType;
    Uint8 HCI_PeerAddr[LEN_BD_ADDR];
    Uint8 HCI_Conn_IntervalL;
    Uint8 HCI_Conn_IntervalH;
    Uint8 HCI_ConnLatencyL;
    Uint8 HCI_ConnLatencyH;
    Uint8 HCI_SvisionTimeoutL;
    Uint8 HCI_SvisionTimeoutH;
    Uint8 HCI_Master_Clk_Accuracy;
    Uint8 HCI_ResvMem[(MAX_MBLK_SIZE-22)];
};

struct MHC_Le_Adv_Report_Para
{
    MHC_HCI_HW_PKT_TYPE
    Uint8 HCI_Para_Length;
    Uint8 HCI_Subevent_Code;
    Uint8 HCI_Num_Reports;
    Uint8 HCI_Event_Type0;
    Uint8 HCI_Addr_Type0;
    Uint8 HCI_Addr0[LEN_BD_ADDR];
    Uint8 HCI_Length0;
    Uint8 HCI_Data0[LEN_ADV_SCAN_DATA_MAX+1];      //1: Uint8 HCI_RSSI0;
    //Uint8 HCI_RSSI0;
};

struct MHC_Le_Conn_Update_Complete_Para
{
    MHC_HCI_HW_PKT_TYPE
    Uint8 HCI_Para_Length;
    Uint8 HCI_Subevent_Code;
    Uint8 HCI_Status;
    HCI_CONN_HDL
    Uint8 HCI_ConnIntervalL;
    Uint8 HCI_ConnIntervalH;
    Uint8 HCI_ConnLatencyL;
    Uint8 HCI_ConnLatencyH;
    Uint8 HCI_SvisionTimeoutL;
    Uint8 HCI_SvisionTimeoutH;
    Uint8 HCI_ResvMem[(MAX_MBLK_SIZE-13)];
};

struct MHC_Le_Read_Remote_Used_Feat_Complete_Para
{
    MHC_HCI_HW_PKT_TYPE
    Uint8 HCI_Para_Length;
    Uint8 HCI_Subevent_Code;
    Uint8 HCI_Status;
    HCI_CONN_HDL
    Uint8 HCI_LE_Feat[8];
    Uint8 HCI_ResvMem[(MAX_MBLK_SIZE-15)];
};

struct MHC_Le_Long_Term_Key_Req_Para
{
    MHC_HCI_HW_PKT_TYPE
    Uint8 HCI_Para_Length;
    Uint8 HCI_Subevent_Code;
    HCI_CONN_HDL
    Uint8 HCI_Random_Num[8];
    Uint8 HCI_Encry_Div[2];
    Uint8 HCI_ResvMem[(MAX_MBLK_SIZE-16)];
};


struct MHC_Le_Remote_Conn_param_Req_Event_Para     //HCI__019
{
    MHC_HCI_HW_PKT_TYPE
    Uint8 HCI_Para_Length;
    Uint8 HCI_Subevent_Code;
    HCI_CONN_HDL
    Uint8 HCI_ConnIntervalMinL;
    Uint8 HCI_ConnIntervalMinH;
    Uint8 HCI_ConnIntervalMaxL;
    Uint8 HCI_ConnIntervalMaxH;
    Uint8 HCI_ConnLatencyL;
    Uint8 HCI_ConnLatencyH;
    Uint8 HCI_SvisionTimeoutL;
    Uint8 HCI_SvisionTimeoutH;
    Uint8 HCI_ResvMem[(MAX_MBLK_SIZE-14)];
};


struct HCLL_Le_Read_Phy_Para
{
    HCLL_HCI_HW_OPCODE
    HCI_CONN_HDL
};

struct HCLL_Le_Set_Default_Phy_Para
{
    HCLL_HCI_HW_OPCODE
    Uint8 HCI_AllPhys;
    Uint8 HCI_TxPhys;
    Uint8 HCI_RxPhys;
};

struct HCLL_Le_Set_Phy_Para
{
    HCLL_HCI_HW_OPCODE
    HCI_CONN_HDL
    Uint8 HCI_AllPhys;
    Uint8 HCI_TxPhys;
    Uint8 HCI_RxPhys;
    Uint8 HCI_PhyOptions;
};


struct HCLL_LE_Set_Data_Length_Para
{
    HCLL_HCI_HW_OPCODE
    HCI_CONN_HDL
#ifndef _HCI_HW_
    Uint16 Padding;
#endif
    Uint16 HCI_RxOctets0;
    Uint16 HCI_RxTime;
    Uint16 HCI_TxOctets0;
    Uint16 HCI_TxTime;
};


struct MHC_Authen_Payload_Timeout_Expired_Para     //HCI__030
{
    MHC_HCI_HW_PKT_TYPE
    Uint8 HCI_Para_Length;
    HCI_CONN_HDL
    Uint8 HCI_ResvMem[(MAX_MBLK_SIZE-5)];
};


struct MHC_Event_HCI_Raw_Para
{
    Uint8 HCI_Para_Length;
    Uint8 Data[(MAX_MBLK_SIZE-1)];
};

#ifdef _HCI_HW_
struct MHC_Le_Data_Length_Change_Para
{
    MHC_HCI_HW_PKT_TYPE
    Uint8 HCI_Para_Length;
    Uint8 HCI_Subevent_Code;
    HCI_CONN_HDL
    Uint16 HCI_MaxTxOctets;
    Uint16 HCI_MaxTxTime;
    Uint16 HCI_MaxRxOctets;
    Uint16 HCI_MaxRxTime;
    Uint8 HCI_ResvMem[(MAX_MBLK_SIZE-10-SIZE_HCI_CONN_HDL-SIZE_MHC_HCI_HW_PKT_TYPE)];
};
#else
struct MHC_Le_Data_Length_Change_Para
{
    MHC_HCI_HW_PKT_TYPE
    Uint8 HCI_Para_Length;
    Uint8 HCI_Subevent_Code;
    HCI_CONN_HDL
    Uint8 Padding;
    Uint16 HCI_MaxTxOctets;
    Uint16 HCI_MaxRxOctets;
    Uint8 HCI_ResvMem[(MAX_MBLK_SIZE-7-SIZE_HCI_CONN_HDL-SIZE_MHC_HCI_HW_PKT_TYPE)];
};
#endif


struct MHC_Le_PHY_Update_Complete_Para
{
    MHC_HCI_HW_PKT_TYPE
    Uint8 HCI_Para_Length;
    Uint8 HCI_Subevent_Code;
    Uint8 HCI_Status;
    HCI_CONN_HDL
    Uint8 HCI_TX_PHY;
    Uint8 HCI_RX_PHY;
    Uint8 HCI_ResvMem[(MAX_MBLK_SIZE-5-SIZE_HCI_CONN_HDL-SIZE_MHC_HCI_HW_PKT_TYPE)];
};


struct MLL_LL_CHN_Map_Upd_Para
{
    Uint8 LL_Conn_No;
    Uint8 ResvMem[(MAX_MBLK_SIZE-1)];
};


struct HSPF_Att_Write_Para
{
    HCI_DATA_CONN_HDL
    HCI_DATA_PKT_LTH_BYTE
    Uint8 HCI_DataPkt_Data[MAX_MBLK_SIZE-SIZE_HCI_DATA_CONN_HDL-SIZE_HCI_DATA_PKT_LTH_BYTE];
};


struct HSPF_Att_Read_Para
{
    HCI_DATA_CONN_HDL
    HCI_DATA_PKT_LTH_BYTE
    Uint8 HCI_DataPkt_Data[MAX_MBLK_SIZE-SIZE_HCI_DATA_CONN_HDL-SIZE_HCI_DATA_PKT_LTH_BYTE];
};


struct HSPF_Att_Notify_Para
{
    HCI_DATA_CONN_HDL
    HCI_DATA_PKT_LTH_BYTE
    Uint8 HCI_DataPkt_Data[MAX_MBLK_SIZE-SIZE_HCI_DATA_CONN_HDL-SIZE_HCI_DATA_PKT_LTH_BYTE];
};


struct HSPF_Att_Indicate_Para
{
    HCI_DATA_CONN_HDL
    HCI_DATA_PKT_LTH_BYTE
    Uint8 HCI_DataPkt_Data[MAX_MBLK_SIZE-SIZE_HCI_DATA_CONN_HDL-SIZE_HCI_DATA_PKT_LTH_BYTE];
};


struct PFHS_Att_Chk_Client_Tab_Para
{
    HCI_DATA_CONN_HDL
    HCI_DATA_PKT_LTH_BYTE
    Uint8 HCI_DataPkt_Data[MAX_MBLK_SIZE-SIZE_HCI_DATA_CONN_HDL-SIZE_HCI_DATA_PKT_LTH_BYTE];
};


struct HSPF_Att_Cfm_Client_Tab_Para
{
    HCI_DATA_CONN_HDL
    HCI_DATA_PKT_LTH_BYTE
    Uint8 HCI_DataPkt_Data[MAX_MBLK_SIZE-SIZE_HCI_DATA_CONN_HDL-SIZE_HCI_DATA_PKT_LTH_BYTE];
};

struct MHC_Le_Stk_Gen_Para
{
    HCI_CONN_HDL
    Uint8 HCI_STK_GEN_MTHD;
};
struct MHC_Le_Auth_status_Para
{
    HCI_CONN_HDL
    Uint8 HCI_AUTH_STATUS;
};

/* Message block */
typedef struct MBLK
{
    void  *Next;       /* point to next mblk */
    Uint8 Primitive;  /* primitive type */
    union
    {
        Uint8 Data[MAX_MBLK_SIZE];
        struct HCLL_Disconnect_Para                         HCLL_Disconnect_Para;
        struct HCLL_Read_Remote_Ver_Info_Para               HCLL_Read_Remote_Ver_Info_Para;
        struct HCLL_Set_Event_Mask_Para                     HCLL_Set_Event_Mask_Para;
        struct HCLL_Reset_Para                              HCLL_Reset_Para;
        struct HCLL_Read_Transmit_Pwr_Level_Para            HCLL_Read_Transmit_Pwr_Level_Para;
        struct HCLL_Set_Ctrler_To_Host_Flow_Ctrl_Para       HCLL_Set_Ctrler_To_Host_Flow_Ctrl_Para;
        struct HCLL_Host_Buffer_Size_Para                   HCLL_Host_Buffer_Size_Para;
        struct HCLL_Host_Num_Of_Completed_Packets_Para      HCLL_Host_Num_Of_Completed_Packets_Para;
        struct HCLL_Set_Event_Mask_Page_2_Para              HCLL_Set_Event_Mask_Page_2_Para;
        struct HCLL_Read_Le_Host_Support_Para               HCLL_Read_Le_Host_Support_Para;
        struct HCLL_Write_Le_Host_Support_Para              HCLL_Write_Le_Host_Support_Para;
        struct HCLL_Read_Authen_Payload_Timeout_Para        HCLL_Read_Authen_Payload_Timeout_Para;
        struct HCLL_Write_Authen_Payload_Timeout_Para       HCLL_Write_Authen_Payload_Timeout_Para;
        struct HCLL_Read_Local_Ver_Info_Para                HCLL_Read_Local_Ver_Info_Para;
        struct HCLL_Read_Local_Supported_Cmd_Para           HCLL_Read_Local_Supported_Cmd_Para;
        struct HCLL_Read_Local_Supported_Feat_Para          HCLL_Read_Local_Supported_Feat_Para;
        struct HCLL_Read_Buffer_Size_Para                   HCLL_Read_Buffer_Size_Para;
        struct HCLL_Read_Bd_Addr_Para                       HCLL_Read_Bd_Addr_Para;
        struct HCLL_Read_Rssi_Para                          HCLL_Read_Rssi_Para;
        struct HCLL_LE_Set_Event_Mask_Para                  HCLL_LE_Set_Event_Mask_Para;
        struct HCLL_LE_Read_Buffer_Size_Para                HCLL_LE_Read_Buffer_Size_Para;
        struct HCLL_LE_Read_Local_Supported_Feat_Para       HCLL_LE_Read_Local_Supported_Feat_Para;
        struct HCLL_LE_Set_Random_Addr_Para                 HCLL_LE_Set_Random_Addr_Para;
        struct HCLL_LE_Set_Adv_Param_Para                   HCLL_LE_Set_Adv_Param_Para;
        struct HCLL_LE_Read_Adv_Ch_Tx_Pwr_Para              HCLL_LE_Read_Adv_Ch_Tx_Pwr_Para;
        struct HCLL_LE_Set_Adv_Data_Para                    HCLL_LE_Set_Adv_Data_Para;
        struct HCLL_LE_Set_Scan_Response_Data_Para          HCLL_LE_Set_Scan_Response_Data_Para;
        struct HCLL_LE_Set_Advertise_Enable_Para            HCLL_LE_Set_Advertise_Enable_Para;
        struct HCLL_LE_Set_Scan_Param_Para                  HCLL_LE_Set_Scan_Param_Para;
        struct HCLL_LE_Set_Scan_Enable_Para                 HCLL_LE_Set_Scan_Enable_Para;
        struct HCLL_LE_Create_Conn_Para                     HCLL_LE_Create_Conn_Para;
        struct HCLL_LE_Create_Conn_Cancel_Para              HCLL_LE_Create_Conn_Cancel_Para;
        struct HCLL_LE_Read_White_List_Size_Para            HCLL_LE_Read_White_List_Size_Para;
        struct HCLL_LE_Clear_White_List_Para                HCLL_LE_Clear_White_List_Para;
        struct HCLL_LE_Add_Device_To_White_List_Para        HCLL_LE_Add_Device_To_White_List_Para;
        struct HCLL_LE_Rmv_Device_From_White_List_Para      HCLL_LE_Rmv_Device_From_White_List_Para;
        struct HCLL_LE_Conn_Update_Para                     HCLL_LE_Conn_Update_Para;
        struct HCLL_LE_Set_Host_Ch_Classification_Para      HCLL_LE_Set_Host_Ch_Classification_Para;
        struct HCLL_LE_Read_Ch_map_Para                     HCLL_LE_Read_Ch_map_Para;
        struct HCLL_LE_Read_Remote_Used_Feat_Para           HCLL_LE_Read_Remote_Used_Feat_Para;
        struct HCLL_LE_Encrypt_Para                         HCLL_LE_Encrypt_Para;
        struct HCLL_LE_Rand_Para                            HCLL_LE_Rand_Para;
        struct HCLL_LE_Start_Encryption_Para                HCLL_LE_Start_Encryption_Para;
        struct HCLL_LE_Long_Term_key_Req_Reply_Para         HCLL_LE_Long_Term_key_Req_Reply_Para;
        struct HCLL_LE_Long_Term_key_Req_Neg_Reply_Para     HCLL_LE_Long_Term_key_Req_Neg_Reply_Para;
        struct HCLL_LE_Read_Supported_States_Para           HCLL_LE_Read_Supported_States_Para;
        struct HCLL_LE_Receiver_Test_Para                   HCLL_LE_Receiver_Test_Para;
        struct HCLL_LE_Transmitter_Test_Para                HCLL_LE_Transmitter_Test_Para;
        struct HCLL_LE_Test_End_Para                        HCLL_LE_Test_End_Para;
        struct HCLL_LE_Remote_Conn_Param_Req_Reply_Para     HCLL_LE_Remote_Conn_Param_Req_Reply_Para;
        struct HCLL_LE_Remote_Conn_Param_Req_Neg_Reply_Para HCLL_LE_Remote_Conn_Param_Req_Neg_Reply_Para;
        struct HCLL_LE_ACL_Data_Pkt_Para                    HCLL_LE_ACL_Data_Pkt_Para;
        struct HCLL_LE_ACL_Data_Pkt_Extend_Para             HCLL_LE_ACL_Data_Pkt_Extend_Para;
        struct LLHC_LE_Ctrl_Pkt_Para                        LLHC_LE_Ctrl_Pkt_Para;
        struct HCLL_LE_Ctrl_Pkt_Para                        HCLL_LE_Ctrl_Pkt_Para;
        struct HCLL_LE_Ctrl_Pkt_Conn_Update_Para            HCLL_LE_Ctrl_Pkt_Conn_Update_Para;
        struct HCLL_LE_Ctrl_Pkt_Set_Host_Ch_Para            HCLL_LE_Ctrl_Pkt_Set_Host_Ch_Para;
        struct MLL_LE_Conn_Para_SMP                         MLL_LE_Conn_Para_SMP;
        struct MLL_LL_Enc_Req_Para                          MLL_LL_Enc_Req_Para;
        struct MLL_LE_CCM_Manual_Para                       MLL_LE_CCM_Manual_Para;
        struct LLEC_LE_CCM_Manual_Para                      LLEC_LE_CCM_Manual_Para;
        struct MHC_Cmd_Complete_Para                        MHC_Cmd_Complete_Para;
        struct MHC_Cmd_Status_Para                          MHC_Cmd_Status_Para;
        struct MHC_Disconn_Complete_Para                    MHC_Disconn_Complete_Para;
        struct MHC_Num_Of_Completed_Pckts_Para              MHC_Num_Of_Completed_Pckts_Para;
        struct MHC_Read_Remote_Ver_Info_Complete_Para       MHC_Read_Remote_Ver_Info_Complete_Para;
        struct MHC_Encrypt_Change_Para                      MHC_Encrypt_Change_Para;
        struct MHC_Encrypt_Key_Refresh_Complete_Para        MHC_Encrypt_Key_Refresh_Complete_Para;
        struct MHC_Data_Buffer_Overflow_Para                MHC_Data_Buffer_Overflow_Para;
        struct MHC_Hardware_Error_Para                      MHC_Hardware_Error_Para;
        struct MHC_Le_Conn_Complete_Para                    MHC_Le_Conn_Complete_Para;
        struct MHC_Le_Adv_Report_Para                       MHC_Le_Adv_Report_Para;
        struct MHC_Le_Conn_Update_Complete_Para             MHC_Le_Conn_Update_Complete_Para;
        struct MHC_Le_Read_Remote_Used_Feat_Complete_Para   MHC_Le_Read_Remote_Used_Feat_Complete_Para;
        struct MHC_Le_Long_Term_Key_Req_Para                MHC_Le_Long_Term_Key_Req_Para;
        struct MHC_Le_Remote_Conn_param_Req_Event_Para      MHC_Le_Remote_Conn_param_Req_Event_Para;
        struct MHC_Authen_Payload_Timeout_Expired_Para      MHC_Authen_Payload_Timeout_Expired_Para;
        struct MHC_Event_HCI_Raw_Para                       MHC_Event_HCI_Raw_Para;
        struct MHC_Le_Data_Length_Change_Para               MHC_Le_Data_Length_Change_Para;
        struct MHC_Le_PHY_Update_Complete_Para              MHC_Le_PHY_Update_Complete_Para;    //PHY Update
        struct HCLL_Le_Read_Phy_Para                        HCLL_Le_Read_Phy_Para;              //PHY Update
        struct HCLL_Le_Set_Default_Phy_Para                 HCLL_Le_Set_Default_Phy_Para;       //PHY Update
        struct HCLL_Le_Set_Phy_Para                         HCLL_Le_Set_Phy_Para;               //PHY Update
        struct HCLL_LE_Set_Data_Length_Para                 HCLL_LE_Set_Data_Length_Para;       //Data Length
        struct MLL_LL_CHN_Map_Upd_Para                      MLL_LL_CHN_Map_Upd_Para;
        struct HSPF_Att_Write_Para                          HSPF_Att_Write_Para;
        struct HSPF_Att_Read_Para                           HSPF_Att_Read_Para;
        struct HSPF_Att_Notify_Para                         HSPF_Att_Notify_Para;
        struct HSPF_Att_Indicate_Para                       HSPF_Att_Indicate_Para;
        struct PFHS_Att_Chk_Client_Tab_Para                 PFHS_Att_Chk_Client_Tab_Para;
        struct HSPF_Att_Cfm_Client_Tab_Para                 HSPF_Att_Cfm_Client_Tab_Para;
        struct MHC_Le_Stk_Gen_Para                          MHC_Le_Stk_Gen_Para;
        struct MHC_Le_Auth_status_Para                      MHC_Le_Auth_status_Para;
    } Para;             /* parameters */
} MBLK;
//typedef MBLK xdata MBLK;


/* Message Queue */
typedef struct MQUEUE
{
    MBLK *QIn;
    MBLK *QOut;
} MQUEUE;

extern const Uint8 MAX_MBLK_NO;
extern MBLK *GetMsgBlk(void);
extern MBLK *GetMsgBlk_wSize(Uint8 len);
extern MBLK *GetMsgBlk_L2_wSize(Uint8 len);
extern MBLK *GetMsgBlk_Isr(void);
extern MBLK *GetMsgBlk_L2_wSize_Isr(Uint8 len);
extern MBLK *GetMsgBlk_L1_wSize_Isr(Uint8 len);
extern Uint8 CheckMsgBlk_L1_wSizeUsed_Isr(Uint8 len);
extern Uint8 CheckMsgBlk_L2_wSizeUsed_Isr(Uint8 len);
extern void FreeMsgBlk(MBLK *pMsgBlk);
extern void FreeMsgBlk_Isr(MBLK *pMsgBlk);
extern void SndMsgBlks_Isr(MBLK *pMsgBlk, Uint8 QueueId);
extern void FreeMsgBlks_Isr(MBLK *pMsgBlk);
extern void SndMsgBlk(MBLK *pMsgBlk, Uint8 QueueId);
extern void SndMsgBlk_Isr(MBLK *pMsgBlk, Uint8 QueueId);
extern void SndMsgBlks(MBLK *pMsgBlk, Uint8 QueueId);
extern void FreeMsgBlks(MBLK *pMsgBlk);
extern MBLK *RcvMsgBlk(Uint8 QueueId);
extern Uint8 CheckMsgQueue(void);

//#endif  //(#ifdef _USED_EXCEPT_SDK_)
extern void Knl_MemCpy(Uint8 *pDst, Uint8 *pSrc, Uint8 len);
extern void Knl_MemCpy_Fwd(Uint8 *pDst, Uint8 *pSrc, Uint8 len);
extern void Knl_MemCpyInv(Uint8 *pDst, Uint8 *pSrc, Uint8 len);
extern void Knl_CodeCpy(Uint8 *pDst, Uint8 const *pSrc, Uint8 len);
extern void Knl_CodeCpyInv(Uint8 *pDst, Uint8 const *pSrc, Uint8 len);
extern Uint8 Knl_MemComp(Uint8 *pDst, Uint8 *pSrc, Uint8 len);
extern Uint8 Knl_CodeComp(Uint8 *pDst, Uint8 const *pSrc, Uint8 len);
extern void Knl_MemCpy_Isr(Uint8 *pDst, Uint8 *pSrc, Uint8 len);
extern void Knl_MemCpyInv_Isr(Uint8 *pDst, Uint8 *pSrc, Uint8 len);
extern void Knl_CodeCpy_Isr(Uint8 *pDst, Uint8 const *pSrc, Uint8 len);
extern void Knl_CodeCpyInv_Isr(Uint8 *pDst, Uint8 const *pSrc, Uint8 len);
extern Uint8 Knl_MemComp_Isr(Uint8 *pDst, Uint8 *pSrc, Uint8 len);
extern Uint8 Knl_CodeComp_Isr(Uint8 *pDst, Uint8 const *pSrc, Uint8 len);
extern Uint16 EndianSwap16_Isr(Uint16 value);
extern void EndianSwap16addr_Isr(Uint16 *value);
extern void EndianSwap16addrM8_Isr(Uint16 *value);

extern void Knl_Delay(Uint8 ticks);
extern Uint16 EndianSwap16(Uint16 value);
extern Uint32 EndianSwap32(Uint32 value);
extern void EndianSwap16addr(Uint16 *value);
extern void EndianSwap32addr(Uint32 *value);
extern void EndianSwap16addrM4(Uint16 *value);
extern void EndianSwap16addrM8(Uint16 *value);
extern void ErrorEntry(Uint32 errID);
extern void KernelCollect(void);
#endif
