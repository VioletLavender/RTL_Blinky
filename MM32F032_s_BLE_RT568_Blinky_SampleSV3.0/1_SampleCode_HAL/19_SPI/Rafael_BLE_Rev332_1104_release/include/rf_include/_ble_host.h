/**************************************************************************//**
* @file       _ble_host.h
* @brief
*
* @copyright  All rights reserved. \n
*             (C) 2020 Rafael Microelectronics Co. Ltd. Taiwan, R.O.C.
*
*
*****************************************************************************/

#ifndef __BLE_HOST_H_
#define __BLE_HOST_H_


#include "ble_basicType.h"
#include "ble_cmd.h"
#include "host.h"                       // define _SMP_ON_


/* ----------------- Temp Setting -------------------------*/
#define BLE_ROLESLAVE_ONELINK_CONNID_FIXED    (MAX_NUM_CONN_SLV_HDL + MAX_NUM_CONN_MAS_HDL -1)   // temp for only one role and one connection link
#define BLE_SDK_Version                       "1.1.0.332"                                        // "X.X.X".0 for SDK formal release version, 1.1.0."X" SVN version



/* ----------------- Host Compile Setting -------------------------*/

// Define BLE Version
//#define BLE_VERSION             0x06    //Bluetooth Core Specification 4.0
//#define BLE_VERSION             0x07    //Bluetooth Core Specification 4.1
//#define BLE_VERSION             0x08    //Bluetooth Core Specification 4.2
#define BLE_VERSION             0x09    //Bluetooth Core Specification 5.0
//#define BLE_VERSION             0x0A    //Bluetooth Core Specification 5.1


#define MAX_SIZE_ATT_VALUE_OVER_MTU_WR      45    //Maximum Attribute Data Size
/* ------------------------------------------------------------------------------------- */


#define SIZE_PACKET_MAX             251  /**< BLE Max Packet Size. \n  maping to L2CAPBuf.DataBuf size
                                            PDU(39 bytes max.)+CRC(3 bytes)*/
#define SIZE_KEYBUFFER_MAX          42  /**< BLE Max Packet Size. \n
                                            PDU(39 bytes max.)+CRC(3 bytes)*/

//Both a GATT client and server implementations shall support an ATT_MTU not less than the default value
#define SIZE_DEFAULT_ATT_MTU                                23      //the default value, BLE
#define SIZE_PLANNED_ATT_MTU                                247      //one value by IOS, BLE
#define SIZE_MINIMUM_ENCRYPTION_KEY                         16
#define SIZE_AES_KEY                                        16



#define HOST_E0                                             0x90
#define MHS_EVENT_NULL                                      (HOST_E0+0x00)
#define HSPF_ATT_WRITE                                      (HOST_E0+0x01)
#define HSPF_ATT_READ                                       (HOST_E0+0x02)
#define HSPF_ATT_NOTIFY                                     (HOST_E0+0x03)
#define HSPF_ATT_INDICATE                                   (HOST_E0+0x04)
#define PFHS_ATT_CHK_CLIENT_TAB                             (HOST_E0+0x05)
#define HSPF_ATT_CFM_CLIENT_TAB                             (HOST_E0+0x06)



#define L2CAP_CID_NULL_IDENTIFIER                           0x00
#define L2CAP_CID_L2CAP_SIGNALING_CHANNEL                   0x01    //ACL
#define L2CAP_CID_CONNECTIONLESS_CHANNEL                    0x02    //ACL
#define L2CAP_CID_AMP_MANAGER_PROTOCOL                      0x03    //ACL
#define L2CAP_CID_ATTRIBUTE_PROTOCOL                        0x04    //LE
#define L2CAP_CID_LE_L2CAP_SIGNALING_CHANNEL                0x05    //LE
#define L2CAP_CID_SECURITY_MANAGER_PROTOCOL                 0x06    //LE


#define CODE_SIGNL_CMD_RESERVED                                 0x00
#define CODE_SIGNL_CMD_COMMAND_REJECT                           0x01
#define CODE_SIGNL_CMD_CONNECTION_REQUEST                       0x02
#define CODE_SIGNL_CMD_CONNECTION_RESPONSE                      0x03
#define CODE_SIGNL_CMD_CONFIGURE_REQUEST                        0x04
#define CODE_SIGNL_CMD_CONFIGURE_RESPONSE                       0x05
#define CODE_SIGNL_CMD_DISCONNECTION_REQUEST                    0x06
#define CODE_SIGNL_CMD_DISCONNECTION_RESPONSE                   0x07
#define CODE_SIGNL_CMD_ECHO_REQUEST                             0x08
#define CODE_SIGNL_CMD_ECHO_RESPONSE                            0x09
#define CODE_SIGNL_CMD_INFORMATION_REQUEST                      0x0A
#define CODE_SIGNL_CMD_INFORMATION_RESPONSE                     0x0B
#define CODE_SIGNL_CMD_CREATE_CHANNEL_REQUEST                   0x0C
#define CODE_SIGNL_CMD_CREATE_CHANNEL_RESPONSE                  0x0D
#define CODE_SIGNL_CMD_MOVE_CHANNEL_REQUEST                     0x0E
#define CODE_SIGNL_CMD_MOVE_CHANNEL_RESPONSE                    0x0F
#define CODE_SIGNL_CMD_MOVE_CHANNEL_CONFIRMATION                0x10
#define CODE_SIGNL_CMD_MOVE_CHANNEL_CONFIRMATION_RESPONSE       0x11
#define CODE_SIGNL_CMD_CONNECTION_PARAMETER_UPDATE_REQUEST      0x12
#define CODE_SIGNL_CMD_CONNECTION_PARAMETER_UPDATE_RESPONSE     0x13



#define FLH_BND_ERR_CODE_NO_ERR                         0x00
#define FLH_BND_ERR_CODE_FLASH_NOT_INI                  0x01
#define FLH_BND_ERR_CODE_IN_PROCESS                     0x02
#define FLH_BND_ERR_CODE_NO_FREE_PBID                   0x03
#define FLH_BND_ERR_CODE_NO_EXST_PBID                   0x04
#define FLH_BND_ERR_CODE_PID_DNT_MATCH                  0x05
#define ERR_CODE_FLH_BND_NO_ENOUGH_REST_SPACE           0x06
#define ERR_CODE_FLH_BND_SYS_ERROR                      0xFF


#define SIZE_FLASH_SECTOR_ODD_CHKSUM                    1
#define SIZE_FLASH_SECTOR_EVEN_CHKSUM                   2

#define TAB_INFO_FLASHBOND_PID                          0
#define TAB_INFO_FLASHBOND_PBID                         2
#define TAB_INFO_FLASHBOND_INI_ADDR                     4
#define TAB_INFO_FLASHBOND_SMP_PARA_BOND                (TAB_INFO_FLASHBOND_INI_ADDR+SIZE_BLE_ADDR)

#define TAB_PARA_DATA_ERROR_CODE                        0
#define TAB_PARA_DATA_PID                               1
#define TAB_PARA_DATA_PBID                              2
#define TAB_PARA_DATA_INI_ADDR                          3
#define TAB_PARA_DATA_DAT_START                         3
#define TAB_PARA_DATA_DAT_PTR                           (TAB_PARA_DATA_DAT_START+1)
#define TAB_PARA_DATA_DAT_SIZE                          (TAB_PARA_DATA_DAT_START+2)
#define TAB_PARA_DATA_DAT                               (TAB_PARA_DATA_DAT_START+3)


#define TAB_DATA_FLASHBOND_PID                          0
#define TAB_DATA_FLASHBOND_PBID                         2
#define TAB_DATA_FLASHBOND_DAT_START                    4


#define SIZE_BASIC_L2CAP_HEADER                         4       //2bytes Length + 2 bytes CID
#define SIZE_SMP_PAIRING_REQUEST                        7
#define SIZE_SMP_PAIRING_RESPONSE                       7
#define SIZE_SMP_SECURITY_REQUEST                       2



#define TAB_BASIC_L2CAP_PDU_LENGTH_L                    0                                   //Reference to BLE dedicated FIFO
#define TAB_BASIC_L2CAP_PDU_LENGTH_H                    (TAB_BASIC_L2CAP_PDU_LENGTH_L+1)
#define TAB_BASIC_L2CAP_CID_L                           (TAB_BASIC_L2CAP_PDU_LENGTH_L+2)
#define TAB_BASIC_L2CAP_CID_H                           (TAB_BASIC_L2CAP_PDU_LENGTH_L+3)
#define TAB_BASIC_L2CAP_OPCODE                          (TAB_BASIC_L2CAP_PDU_LENGTH_L+4)
#define TAB_BASIC_L2CAP_CODE_SMP                        (TAB_BASIC_L2CAP_PDU_LENGTH_L+4)
#define TAB_BASIC_L2CAP_SIGNALING_IDENTIFIER            (TAB_BASIC_L2CAP_OPCODE+1)          //Bluetooth Spec. [Vol 3] page 55 of 656
#define TAB_BASIC_L2CAP_SIGNALING_COMMAND_LENGTH_L      (TAB_BASIC_L2CAP_OPCODE+2)          //Bluetooth Spec. [Vol 3] page 56 of 656
#define TAB_BASIC_L2CAP_SIGNALING_COMMAND_LENGTH_H      (TAB_BASIC_L2CAP_OPCODE+3)
#define TAB_BASIC_L2CAP_SIGNALING_RESULT_L              (TAB_BASIC_L2CAP_OPCODE+4)          //Bluetooth Spec. [Vol 3] page 56 of 656
#define TAB_BASIC_L2CAP_SIGNALING_RESULT_H              (TAB_BASIC_L2CAP_OPCODE+5)

#define TAB_BASIC_L2CAP_FORMAT                          (TAB_BASIC_L2CAP_OPCODE+1)


#define TAB_ATTRIBUTE_LENGTH                (TAB_BASIC_L2CAP_OPCODE+1)   //0~3:PDU length+channel ID, 4:OPCODE
#define TAB_ATTRIBUTE_VALUE                 (TAB_BASIC_L2CAP_OPCODE+1)   //0~3:PDU length+channel ID, 4:OPCODE
#define TAB_ATTRIBUTE_HANDLE_L              (TAB_BASIC_L2CAP_OPCODE+1)   //0~3:PDU length+channel ID, 4:OPCODE
#define TAB_ATTRIBUTE_HANDLE_H              (TAB_BASIC_L2CAP_OPCODE+2)   //0~3:PDU length+channel ID, 4:OPCODE

#define TAB_ATTRIBUTE_HANDLE_START_L        (TAB_BASIC_L2CAP_OPCODE+1)   //0~3:PDU length+channel ID, 4:OPCODE
#define TAB_ATTRIBUTE_HANDLE_START_H        (TAB_BASIC_L2CAP_OPCODE+2)   //0~3:PDU length+channel ID, 4:OPCODE
#define TAB_ATTRIBUTE_HANDLE_END_L          (TAB_BASIC_L2CAP_OPCODE+3)   //0~3:PDU length+channel ID, 4:OPCODE
#define TAB_ATTRIBUTE_HANDLE_END_H          (TAB_BASIC_L2CAP_OPCODE+4)   //0~3:PDU length+channel ID, 4:OPCODE

#define TAB_ATTRIBUTE_TYPE_L                (TAB_ATTRIBUTE_HANDLE_END_H+1)   //0~3:PDU length+channel ID, 4:OPCODE
#define TAB_ATTRIBUTE_TYPE_H                (TAB_ATTRIBUTE_HANDLE_END_H+2)   //0~3:PDU length+channel ID, 4:OPCODE

#define TAB_ATTRIBUTE_SERVER_RX_MTU_L       (TAB_BASIC_L2CAP_OPCODE+1)
#define TAB_ATTRIBUTE_SERVER_RX_MTU_H       (TAB_BASIC_L2CAP_OPCODE+2)

#define TAB_ATTRIBUTE_VALUE_OFFSET          (TAB_ATTRIBUTE_HANDLE_H+1)  //0~3:PDU length+channel ID, 4:OPCODE, Bluetooth Spec. Ver4.0 [Vol 3] page 496 of 656

#define TAB_ATT_VALUE_OFFSET_L              (TAB_ATTRIBUTE_HANDLE_H+1)  //0~3:PDU length+channel ID, 4:OPCODE, Bluetooth Spec. Ver4.0 [Vol 3] page 496 of 656
#define TAB_ATT_VALUE_OFFSET_H              (TAB_ATTRIBUTE_HANDLE_H+2)  //0~3:PDU length+channel ID, 4:OPCODE, Bluetooth Spec. Ver4.0 [Vol 3] page 496 of 656
#define TAB_ATT_VALUE_VALUE_BGN             (TAB_ATT_VALUE_OFFSET_H+1)

#define TAB_Q_ATT_HDL_PREWR_HDL_L           0
#define TAB_Q_ATT_HDL_PREWR_HDL_H           1
#define TAB_Q_ATT_HDL_PREWR_LTH             2
#define TAB_Q_ATT_HDL_PREWR_VAL             3

#define TAB_ATTRIBUTE_SERVER_HANDLE_H                   0
#define TAB_ATTRIBUTE_SERVER_HANDLE_L                   (TAB_ATTRIBUTE_SERVER_HANDLE_H+1)
#define TAB_ATTRIBUTE_SERVER_TYPE_H                     (TAB_ATTRIBUTE_SERVER_HANDLE_H+2)
#define TAB_ATTRIBUTE_SERVER_TYPE_L                     (TAB_ATTRIBUTE_SERVER_HANDLE_H+3)
#define TAB_ATTRIBUTE_SERVER_SIZE_OF_VALUE              (TAB_ATTRIBUTE_SERVER_HANDLE_H+4)
#define TAB_ATTRIBUTE_SERVER_SIZE_OF_VALUE_128UUID      (TAB_ATTRIBUTE_SERVER_HANDLE_H+4+14)
#define TAB_ATTRIBUTE_SERVER_VALUE                      (TAB_ATTRIBUTE_SERVER_HANDLE_H+5)
#define TAB_ATTRIBUTE_SERVER_CHARACTERISTIC_PROPERTIES  (TAB_ATTRIBUTE_SERVER_HANDLE_H+5)

#define TAB_ATTRIBUTE_PARAMETER_SERVER_VALUE            0

#define TAB_IO_CAPABILITY                               (TAB_BASIC_L2CAP_CODE_SMP+1)  //for SMP Paring Req/Res
#define TAB_OOB_DATA_FLAG                               (TAB_BASIC_L2CAP_CODE_SMP+2)  //for SMP Paring Req/Res
#define TAB_AUTHREQ                                     (TAB_BASIC_L2CAP_CODE_SMP+3)  //for SMP Paring Req/Res
#define TAB_MAX_ENCRYPTION_KEY_SIZE                     (TAB_BASIC_L2CAP_CODE_SMP+4)  //for SMP Paring Req/Res
#define TAB_INITIATOR_KEY_DISTRIBUTION                  (TAB_BASIC_L2CAP_CODE_SMP+5)  //for SMP Paring Req/Res
#define TAB_RESPONDER_KEY_DISTRIBUTION                  (TAB_BASIC_L2CAP_CODE_SMP+6)  //for SMP Paring Req/Res


#define SIZE_PDU_WRITE_RESPONSE                         1
#define SIZE_PDU_READ_RESPONSE                          3
#define SIZE_PDU_ATT_FIND_BY_TYPE_VAlUE_RESPONSE        5
#define SIZE_PDU_ATT_ERROR_RESPONSE                     5
#define SIZE_PDU_ATT_READ_BY_TYPE_RSP_DEVICE_NAME       4
#define SIZE_PDU_ATT_READ_BY_TYPE_RSP_CHARACTERISTIC    9
#define SIZE_PDU_ATT_READ_BY_TYPE_RESPONSE              23
#define SIZE_PDU_ATT_FIND_INFORMATION_RESPONSE_14_15    10
#define SIZE_PDU_ATT_FIND_INFORMATION_RESPONSE_54_55    14
#define SIZE_PDU_ATT_READ_BY_GROUP_TYPE_RESPONSE_DEVICE_INFORMATION  20
#define SIZE_PDU_ATT_READ_BY_GROUP_TYPE_RESPONSE_UNKNOWN_UUID        14

typedef Uint8 CmdTimerEvent;
/** Null event.*/
#define TIMER_EVENT_NULL                                 0xFF
#define TIMER_EVENT_CONN_PARAMETER_UPDATE_RSP            0x00
#define TIMER_EVENT_CONN_UPDATE_COMPLETE                 0x01
#define TIMER_EVENT_AUTH_STATUS                          0x02

#define TOTAL_TIMER_EVENT                                (TIMER_EVENT_AUTH_STATUS+1)

/** @brief BLE device parameter.
*/
typedef struct BLE_Device_Param
{
    uint8_t         ble_deviceChipId;     /** Chip Id */
    BLE_Addr_Param  ble_deviceAddr_param; /** BLE device address*/
} BLE_Device_Param;

extern BLE_Device_Param ble_device_param;

#ifdef _SMP_ON_

#define CODE_SMP_RESERVED                               0x00
#define CODE_SMP_PAIRING_REQUEST                        0x01
#define CODE_SMP_PAIRING_RESPONSE                       0x02
#define CODE_SMP_PAIRING_CONFIRM                        0x03
#define CODE_SMP_PAIRING_RANDOM                         0x04
#define CODE_SMP_PAIRING_FAIlED                         0x05
#define CODE_SMP_ENCRYPTION_INFORMATION                 0x06
#define CODE_SMP_MASTER_IDENTIFICATION                  0x07
#define CODE_SMP_IDENTITY_INFORMATION                   0x08
#define CODE_SMP_IDENTITY_ADDRESS_INFORMATION           0x09
#define CODE_SMP_SIGNING_INFORMATION                    0x0A
#define CODE_SMP_SECURITY_REQUEST                       0x0B
#define CODE_SMP_PAIRING_PUBLIC_KEY                     0x0C
#define CODE_SMP_PAIRING_DHKEY_CHECK                    0x0D
#define CODE_SMP_PAIRING_KEYPRESS_NOTIFICATION          0x0E
#define CODE_SMP_MAX_HANDLE_RANGE                       CODE_SMP_SECURITY_REQUEST

#define IO_CAPABILITY_DISPLAY_ONLY                      0x00
#define IO_CAPABILITY_DISPLAY_YESNO                     0x01
#define IO_CAPABILITY_KEYBOARD_ONLY                     0x02
#define IO_CAPABILITY_NOINPUT_NOOUTPUT                  0x03
#define IO_CAPABILITY_KEYBOARD_DISPLAY                  0x04

#define OOB_AUTHENTICATION_DATA_NOT_PRESENT                 0x00
#define OOB_AUTHENTICATION_DATA_FROM_REMOTE_DEVICE_PRESENT  0x01

#define AUTHREQ_BONDING_FLAGS_NO_BONDING                0x00
#define AUTHREQ_BONDING_FLAGS_BONDING                   0x01

#define MITM_PROTECTION_YES                             0x04
#define MITM_PROTECTION_NO                              0x00

#define SECURE_CONN_PAIRING_NO                          0x00
#define SECURE_CONN_PAIRING_YES                         0x08

#define SMP_PAIRING_KEYPRESS_NOTIFICATION_NO            0x00
#define SMP_PAIRING_KEYPRESS_NOTIFICATION_YES           0x10

#define CT2_SUPPORTS_H7_FUNCTION_NO                     0x00
#define CT2_SUPPORTS_H7_FUNCTION_YES                    0x20

#define KEY_DISTRIBUTION_ENCKEY_0                       0x00
#define KEY_DISTRIBUTION_ENCKEY_1                       0x01
#define KEY_DISTRIBUTION_IDKEY_0                        0x00
#define KEY_DISTRIBUTION_IDKEY_1                        0x02
#define KEY_DISTRIBUTION_SIGN_0                         0x00
#define KEY_DISTRIBUTION_SIGN_1                         0x04
#define KEY_DISTRIBUTION_LINK_0                         0x00
#define KEY_DISTRIBUTION_LINK_1                         0x08

#define STK_GEN_MTHD_JUST_WORKS                         0x00
#define STK_GEN_MTHD_PASSKEY_ENTRY                      0x01
#define STK_GEN_MTHD_PASSKEY_ENTRY_DISP                 0x02

#define ERR_CODE_SMP_RESERVED                           0x00
#define ERR_CODE_SMP_PASSKEY_ENTRY_FAILED               0x01
#define ERR_CODE_SMP_OOB_NOT_AVAILABLE                  0x02
#define ERR_CODE_SMP_AUTHENTICATION_REQUIREMENTS        0x03
#define ERR_CODE_SMP_CONFIRM_VALUE_FAILED               0x04
#define ERR_CODE_SMP_PAIRING_NOT_SUPPORTED              0x05
#define ERR_CODE_SMP_ENCRYPTION_KEY_SIZE                0x06
#define ERR_CODE_SMP_COMMAND_NOT_SUPPORTED              0x07
#define ERR_CODE_SMP_UNSPECIFIED_REASON                 0x08
#define ERR_CODE_SMP_REPEATED_ATTEMPTS                  0x09
#define ERR_CODE_SMP_INVALID_PARAMETERS                 0x0A
#define ERR_CODE_SMP_DHKEY_CHECK_FAILED                 0x0B
#define ERR_CODE_SMP_NUMERIC_COMPARISON_FAILED          0x0C
#define ERR_CODE_SMP_BR_EDR_PAIRING_IN_PROGRESS         0x0D
#define ERR_CODE_SMP_CROSS_TRANS_KEY_GEN_NOT_ALLOWED    0x0E

#define SIZE_SMP_RAND                                   8
#define SIZE_SMP_EDIV                                   2
#define SIZE_SMP_SKDM                                   8
#define SIZE_SMP_IVM                                    4
#define SIZE_SMP_SKDS                                   8
#define SIZE_SMP_IVS                                    4
#define SIZE_SMP_IRK                                    16
#define SIZE_SMP_CSRK                                   16



#define SMP_INI_ADDR_TYPE                               0       //Used on smp_Para_Bond[];
#define SMP_INI_ADDR                                    1       //Used on smp_Para_Bond[];
#define SMP_RAND                                        7       //Used on smp_Para_Bond[];
#define SMP_EDIV                                        15      //Used on smp_Para_Bond[];
#define SMP_PREQ_MAX_ENCR_KEY_SIZE                      17      //Used on smp_Para_Bond[];
#define SMP_KEY_USE                                     18      //Used on smp_Para_Bond[];
#define SEL_STK_GEN_METHOD                              19      //Used on smp_Para_Bond[];
#define SMP_LTK                                         20      //Used on smp_Para_Bond[];
#define SMP_PREQ_AUTHREQ                                36      //Used on smp_Para_Bond[];
#define SMP_PARA_BOND_SIZE                              (SMP_PREQ_AUTHREQ+1)

#define SMP_KEY_USE_STK                                         0x00
#define SMP_KEY_USE_LTK                                         0x01

#define SMP_TEMPDEF_NONE                                        0x00
#define SMP_TEMPDEF_MCONFIRM                                    0x01
#define SMP_TEMPDEF_MRAND                                       0x02
#define SMP_TEMPDEF_LTK                                         0x03

#define SMP_SECURITY_MODE_1_NO_SECURITY                         0x00
#define SMP_SECURITY_MODE_1_UNAUTHEN_PAIRING_W_ENCYPT           0x01
#define SMP_SECURITY_MODE_1_AUTHEN_PAIRING_W_ENCYPT             0x02
#define SMP_SECURITY_MODE_2_UNAUTHEN_PAIRING_W_DATA_SIGN        0x03
#define SMP_SECURITY_MODE_2_AUTHEN_PAIRING_W_DATA_SIGN          0x04

#define SMP_SECURITY_MODE_MINIMUN                               SMP_SECURITY_MODE_1_UNAUTHEN_PAIRING_W_ENCYPT

#ifdef _BOND_ON_
#define CMD_FB_GET_NXT_PBID                                     0x01
#define CMD_FB_INIT_INFO_FLASHBOND                              0x02
#define CMD_FB_INIT_DATA_FLASHBOND                              0x03
#define CMD_FB_CHK_IF_FLASH_INITED                              0x04
#define CMD_FB_GET_EXST_PBID                                    0x05
#define CMD_FB_GET_INFO_FLASHBOND_PARA_BOND                     0x06
#define CMD_FB_GET_DATA_FLASHBOND_EXST_PBID_DBLK_START          0x07
#define CMD_FB_GET_DATA_FLASHBOND_NXT_PBID_DBLK_START           0x08
#define CMD_FB_PSH_DATA_FLASHBOND_EXST_PBID_DBLK                0x09
#define CMD_FB_CHK_IF_FLASHBOND_NEED_TO_ERASE_PAGE              0x0A
#endif

/*
extern const Uint8 RSP_PAIRING_REQUEST[];
extern const Uint8 SKDS_FIXED[];
extern const Uint8 IVS_FIXED[];
extern const Uint8 LTK_FIXED[16];
extern const Uint8 EDIV_FIXED[2];
extern const Uint8 RAND_FIXED[8];
extern const Uint8 IRK_FIXED[][16];
extern const Uint8 CSRK_FIXED[][16];
extern const Uint8 SEL_STK_GEN_MTHD[5][5];

extern const Uint8 k_AES[16];
extern const Uint8 r_AES_INI[16];
extern const Uint8 P1_AES[16];
extern const Uint8 P2_AES[16];
*/
extern Uint8 RSP_PAIRING_REQUEST[];
extern Uint8 SKDS_FIXED[];
extern Uint8 IVS_FIXED[];
extern Uint8 LTK_FIXED[16];
extern Uint8 EDIV_FIXED[2];
extern Uint8 RAND_FIXED[8];
extern Uint8 IRK_FIXED[][16];
extern Uint8 CSRK_FIXED[][16];
extern Uint8 SEL_STK_GEN_MTHD[5][5];
extern Uint8 k_AES[16];
extern Uint8 r_AES_INI[16];
//extern const Uint8 code R1_AES[16];
//extern const Uint8 code R2_AES[16];

extern Uint8 smp_Authorization;
extern Uint8 smp_Encryption;
extern Uint8 smp_Phase;
extern Uint8 smp_TempDef;
extern Uint8 smp_Temp16[16];
extern Uint8 smp_Rand[8];
extern Uint8 smp_EDIV[2];
extern Uint8 smp_IRKm[SIZE_SMP_IRK];
extern Uint8 smp_CSRKm[SIZE_SMP_CSRK];
extern Uint8 smp_SKDm[8];
extern Uint8 smp_IVm[4];
extern Uint8 smp_Security_Mode;

extern Uint8 smp_PReq_IO_Capab;
extern Uint8 smp_PReq_OOB_data;
extern Uint8 smp_PReq_Init_Key_Distr;
extern Uint8 smp_PReq_Resp_Key_Distr;

extern Uint8 k_AES_passkey[16];
extern Uint8 r_AES[16];
extern Uint8 LTK[16];
extern Uint8 LTK_SEED[SIZE_SMP_RAND+SIZE_SMP_EDIV];
extern Uint8 smp_Para_Bond[SMP_PARA_BOND_SIZE];
//extern Uint8 INFO_FLASHBOND[(NUM_FLASH_BONDING_INFO_BLK/(SIZE_FLASH_SECTOR/TAB_FLASH_BONDING_INFO_BLK))][SIZE_FLASH_SECTOR];
//extern Uint8 DATA_FLASHBOND[NUM_FLASH_BONDING_DAT_GRP_BLK][SIZE_FLASH_SECTOR];
extern Uint8 smp_Para_Bond_tmp[70];
extern Uint8 SECBuffer[SIZE_KEYBUFFER_MAX];
extern Uint8 rsp_pairing_request_table[11];

//extern void smpAES(Uint8 * key, Uint8 * plaintextData);
extern void smpC1(Uint8 * k, Uint8 * r, Uint8 * smpC1_out);
extern void smpS1(Uint8 * k, Uint8 * r1, Uint8 * r2, Uint8 * smpS1_out);
extern void smpPasskey_init(void);
extern Uint8 smpPasskey_set(Uint32 hexPasskey, Uint8 connID);
//extern void smpPasskey_req(Uint8 connID);
extern Uint8 setBLE_ConnTxData_SMP(Uint8 connID, Uint8 L2CAP_Code_SMP, Uint8 * L2CAP_RspDataSMP, Uint8 Length);
extern Uint8 setBLE_ConnTxData_SMP_Isr(Uint8 connID, Uint8 L2CAP_Code_SMP, Uint8 * L2CAP_RspDataSMP, Uint8 Length);
extern Uint8 PassEC_BLE_CODE_SMP(Uint8 connID, Uint8 Primitive, Uint8 *source, Uint8 length);

#else
#define CODE_SMP_PAIRING_FAIlED                         0x05
#define ERR_CODE_SMP_COMMAND_NOT_SUPPORTED              0x07
#endif  //(#ifdef _SMP_ON_)



//extern Uint8 *ConfigTable;
extern Uint8 SPIBuffer[SIZE_PACKET_MAX];
extern Uint8 aTT_MTU[];
//extern const Uint8 CODE_BLE_CONN_PARAM_UPDATE_EXPECT[];
extern const Uint8 UUID_BLUETOOTH_BASE[16];
extern Uint8 q_att_HDL_PreWr[MAX_SIZE_ATT_VALUE_OVER_MTU_WR+3];

extern Uint8 setBLE_ConnTxData(Uint8 connID, Uint8 * ScanRspData, Uint8 Length);
//extern Uint8 setBLE_ConnTxCtrl(Uint8 connID, Uint8 * ScanRspData, Uint8 Length);
//extern Uint8 setBLE_chkConnParam(Uint8 * expTable);

extern Uint8 setBLE_ConnTxData_ATT_ERROR_RESPONSE(Uint8 connID, Uint8 Request_Opcode_In_Error, Uint8 ATT_handle_in_error_L, Uint8 ATT_handle_in_error_H, Uint8 Error_code);

extern void init_q_att_HDL_PreWr(void);
extern void initBLE_AttServParamter(void);

extern Uint8 chkBLE_ATTRIBUTE_PERMISSION_Read(Uint8 ATT_Handle);
extern Uint8 chkBLE_ATTRIBUTE_PERMISSION_Write(Uint8 ATT_Handle);
extern Uint8 getBLE_ATTRIBUTE_TYPE_Size(Uint8 ATT_Handle);
extern Uint8 getBLE_ATTRIBUTE_Value_DynmcLngth(Uint8 AttHandle, Uint8 SizeIdx);
extern Uint8 chkIfBLE_ATT_Chr_Acss_ByUser(Uint8 AttHandle);
extern Uint8 chkBLE_ATT_Chr_AcssRd_ByUser(Uint8 *AddrAttHandle);
extern Uint8 chkBLE_ATT_Chr_AcssWr_ByUser(Uint8 *AddrAttHandle, Uint8 *wrDataBuf);



#endif // __BLE_HOST_H_


