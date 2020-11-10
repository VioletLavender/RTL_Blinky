#ifndef _HCI_H_
#define _HCI_H_

#include "BleAppSetting.h"
#include "ble_cmd.h"
#include "_ble_host.h"
#include "knl_pblc.h"

#ifdef _HCI_NEW_
#define HCI_E0                                              0x60
#define MHC_EVENT_NULL                                      (HCI_E0+0x00)
#define MHC_EVENT_CMD_COMPLETE                              (HCI_E0+0x01)
#define MHC_EVENT_CMD_STATUS                                (HCI_E0+0x02)
#define MHC_EVENT_DISCONN_COMPLETE                          (HCI_E0+0x03)
#define MHC_EVENT_NUM_OF_COMPLETED_PCKTS                    (HCI_E0+0x04)
#define MHC_EVENT_READ_REMOTE_VER_INFO_COMPLETE             (HCI_E0+0x05)
#define MHC_EVENT_ENCRYPT_CHANGE                            (HCI_E0+0x06)
#define MHC_EVENT_ENCRYPT_KEY_REFRESH_COMPLETE              (HCI_E0+0x07)
#define MHC_EVENT_DATA_BUFFER_OVERFLOW                      (HCI_E0+0x08)
#define MHC_EVENT_HARDWARE_ERROR                            (HCI_E0+0x09)
#define MHC_EVENT_LE_CONN_COMPLETE                          (HCI_E0+0x0A)
#define MHC_EVENT_LE_ADV_REPORT                             (HCI_E0+0x0B)
#define MHC_EVENT_LE_CONN_UPDATE_COMPLETE                   (HCI_E0+0x0C)
#define MHC_EVENT_LE_READ_REMOTE_USED_FEATURES_COMPLETE     (HCI_E0+0x0D)
#define MHC_EVENT_LE_LONG_TERM_KEY_REQ                      (HCI_E0+0x0E)
#define MHC_EVENT_LE_REMOTE_CONN_PARAM_REQ_EVENT            (HCI_E0+0x0F)   //Spec. 4.1
#define MHC_EVENT_AUTHEN_PAYLOAD_TIMEOUT_EXPIRED            (HCI_E0+0x10)   //Spec. 4.1
#define MHC_LE_ACL_DATA_PKT                                 (HCI_E0+0x11)   //Jeffrey defined
#define MHC_EVENT_HCI_RAW                                   (HCI_E0+0x12)   //Jeffrey defined
#define MHC_EVENT_LE_DATA_LENGTH_CHANGE                     (HCI_E0+0x13)   //Spec. 5.0
#define MHC_EVENT_LE_PHY_UPDATE_COMPLETE                    (HCI_E0+0x14)   //Spec. 5.0
#define MHC_EVENT_LE_STK_GEN_METHOD                         (HCI_E0+0x15)   //Yuwei defined
#define MHC_EVENT_LE_WAIT_PASSKEY_CONFIRM                   (HCI_E0+0x16)   //Yuwei defined
#define MHC_EVENT_AUTH_STATUS                               (HCI_E0+0x17)   //Yuwei defined
#define MHC_LE_ACL_DATA_PKT_EXTEND                          (HCI_E0+0x18)   //Jeffrey defined

//Four kinds of HCI packets that can be sent via the UART Transport Layer
//HCI__001
#define HCI_PKT_COMMAND                         0x01
#define HCI_PKT_ACL_DATA                        0x02
#define HCI_PKT_SYNCHRONOUS_DATA                0x03
#define HCI_PKT_EVENT                           0x04

//LL__023
#define HCI_VERSION                             BLE_VERSION     //HCI__002
#define HCI_REVISION_L                          0x01
#define HCI_REVISION_H                          0x00
#define LMP_VERSION                             BLE_VERSION     //HCI__003
#define LMP_VERSION_SUBVER_L                    0x00
#define LMP_VERSION_SUBVER_H                    0x00

//HCI__016
#define HCI_EVTMSK_0_DISCONN_CMPLT                          0x10
#define HCI_EVTMSK_0_ENC_CHANGE                             0x80
#define HCI_EVTMSK_1_RD_RMT_VER_INFO_CMPLT                  0x08
#define HCI_EVTMSK_1_HW_ERROR                               0x80
#define HCI_EVTMSK_3_DATA_BUF_OVERFLOW                      0x02
#define HCI_EVTMSK_5_ENC_KEY_RFRSH_CMPLT                    0x80
#define HCI_EVTMSK_7_LE_META                                0x20

#define HCI_EVTMSK2_2_AUTHEN_PYLD_TIMEOUT_EXPRD             0x80

#define HCI_LEEVTMSK_LE_CONN_CMPLT                          0x01
#define HCI_LEEVTMSK_LE_ADV_REPORT                          0x02
#define HCI_LEEVTMSK_LE_CONN_UPD_CMPLT                      0x04
#define HCI_LEEVTMSK_LE_RD_RMT_USED_FEAT_CMPLT              0x08
#define HCI_LEEVTMSK_LE_LONG_TRM_KEY_REQ                    0x10
#define HCI_LEEVTMSK_LE_RMT_CONN_PARAM_REQ                  0x20
#define HCI_LEEVTMSK_LE_DATA_LENGTH_CHANGE                  0x40
#define HCI_LEEVTMSK_LE_RD_LOCAL_P256_PUBLIC_KEY_CMPLT      0x80
#define HCI_LEEVTMSK_LE_GENERATE_DHKEY_CMPLT                0x01
#define HCI_LEEVTMSK_LE_ENHANCED_CONN_CMPLT                 0x02
#define HCI_LEEVTMSK_LE_DIRECTED_ADV_REPORT                 0x04
#define HCI_LEEVTMSK_LE_PHY_UPDATE_CMPLT                    0x08
#define HCI_LEEVTMSK_LE_EXTENDED_ADV_REPORT                 0x10
#define HCI_LEEVTMSK_LE_PERIODIC_ADV_SYNC_ESTABLISHED       0x20
#define HCI_LEEVTMSK_LE_PERIODIC_ADV_REPORT                 0x40
#define HCI_LEEVTMSK_LE_PERIODIC_ADV_SYNC_LOST              0x80
#define HCI_LEEVTMSK_LE_SCAN_TIMEOUT                        0x01
#define HCI_LEEVTMSK_LE_ADV_SET_TERMINATED                  0x02
#define HCI_LEEVTMSK_LE_SCAN_REQ_RECEIVED                   0x04
#define HCI_LEEVTMSK_LE_CHANNEL_SELECTION_ALGORITHM         0x08
#define HCI_LEEVTMSK_LE_CONNLESS_IQ_REPORT                  0x10
#define HCI_LEEVTMSK_LE_CONN_IQ_REPORT                      0x20
#define HCI_LEEVTMSK_LE_CTE_REQ_FAILED                      0x40
#define HCI_LEEVTMSK_LE_PERIODIC_ADV_SYNC_TRANSFER_RECEIVED 0x80

//HCI__032
#define HCI_ADDR_TYPE_PUBLIC                        0x00
#define HCI_ADDR_TYPE_RANDOM                        0x01
//#define HCI_ADDR_TYPE_RESOLVABLE_PRIVATE_PUBLIC     0x02
#define HCI_ADDR_TYPE_RESOLVABLE_PRIVATE_RANDOM     0x03

#define HCI_ROLE_MASTER                             0x00
#define HCI_ROLE_SLAVE                              0x01
#endif


#ifdef _HCI_NEW_
//HCI__005
#define FLD_HCI_PKT_LTH_COMMAND                 3       //0:Packet Type, 1,2:OpCode
#define FLD_HCI_PKT_LTH_ACL_DATA                3       //0:Packet Type, 1,2:Handle & Flags
#define FLD_HCI_PKT_LTH_SYNCHRONOUS_DATA        3       //0:Packet Type, 1,2:Connection_Handle & Flag
#define FLD_HCI_PKT_LTH_EVENT                   2       //0:Packet Type, 1:Event Code
#define FLD_HCI_PARAMETER_0_COMMAND             (FLD_HCI_PKT_LTH_COMMAND+1)             //0:Packet Type, 1,2:OpCode, 3:Length
#define FLD_HCI_PARAMETER_0_ACL_DATA            (FLD_HCI_PKT_LTH_ACL_DATA+2)            //0:Packet Type, 1,2:OpCode, 3,4:Length
#define FLD_HCI_PARAMETER_0_SYNCHRONOUS_DATA    (FLD_HCI_PKT_LTH_SYNCHRONOUS_DATA+1)    //0:Packet Type, 1,2:OpCode, 3:Length
#define FLD_HCI_PARAMETER_0_EVENT               (FLD_HCI_PKT_LTH_EVENT+1)               //0:Packet Type, 1,2:OpCode, 3:Length

//HCI__006
#define OGF_HCI_CMD_LINK_CONTROL                0x01
#define OGF_HCI_CMD_LINK_POLICY                 0x02
#define OGF_HCI_CMD_CONTROLLER_BASEBAND         0x03
#define OGF_HCI_CMD_INFORMATIONAL_PARAM         0x04
#define OGF_HCI_CMD_STATUS_PARAM                0x05
#define OGF_HCI_CMD_TESTING                     0x06
#define OGF_HCI_CMD_LE_CONTROLLER               0x08

//HCI__007
#define BC_HCI_FLAG_POINT_TO_POINT              0x00
#define PB_HCI_FLAG_FIRST_AUTO_FLUSH_PKT        0x20
#define PB_HCI_FLAG_CONT_FRAGMENT               0x10
#define PB_HCI_FLAG_FIRST_NONAUTO_FLUSH_PKT     0x00
#define FLD_MSK_PB_HCI                          0x30
#define FLD_MSK_BC_HCI                          0xC0

//LE support, vol.2 page 415~
#define HCI_COMMAND_DISCONNECT                                  0x06    //OGF: Link Control, 0x01
#define HCI_COMMAND_READ_REMOTE_VERSION_INFORMATION             0x1D    //OGF: Link Control, 0x01
#define HCI_COMMAND_SET_EVENT_MASK                              0x01    //OGF: Controller & Baseband, 0x03
#define HCI_COMMAND_RESET                                       0x03    //OGF: Controller & Baseband, 0x03
#define HCI_COMMAND_READ_TRANSMIT_POWER_LEVEL                   0x2D    //OGF: Controller & Baseband, 0x03
#define HCI_COMMAND_SET_CONTROLLER_TO_HOST_FLOW_CONTROL         0x31    //OGF: Controller & Baseband, 0x03
#define HCI_COMMAND_HOST_BUFFER_SIZE                            0x33    //OGF: Controller & Baseband, 0x03
#define HCI_COMMAND_HOST_NUMBER_OF_COMPLETED_PACKETS            0x35    //OGF: Controller & Baseband, 0x03
#define HCI_COMMAND_SET_EVENT_MASK_PAGE_2                       0x63    //OGF: Controller & Baseband, 0x03, HCI__020
#define HCI_COMMAND_READ_LE_HOST_SUPPORT                        0x6C    //OGF: Controller & Baseband, 0x03
#define HCI_COMMAND_WRITE_LE_HOST_SUPPORT                       0x6D    //OGF: Controller & Baseband, 0x03
#define HCI_COMMAND_READ_AUTHENTICATED_PAYLOAD_TIMEOUT          0x7B    //OGF: Controller & Baseband, 0x03, HCI__020
#define HCI_COMMAND_WRITE_AUTHENTICATED_PAYLOAD_TIMEOUT         0x7C    //OGF: Controller & Baseband, 0x03, HCI__020
#define HCI_COMMAND_READ_LOCAL_VERSION_INFORMATION              0x01    //OGF: Informational Parameters, 0x04
#define HCI_COMMAND_READ_LOCAL_SUPPORTED_COMMANDS               0x02    //OGF: Informational Parameters, 0x04
#define HCI_COMMAND_READ_LOCAL_SUPPORTED_FEATURES               0x03    //OGF: Informational Parameters, 0x04
#define HCI_COMMAND_READ_BUFFER_SIZE                            0x05    //OGF: Informational Parameters, 0x04
#define HCI_COMMAND_READ_BD_ADDR                                0x09    //OGF: Informational Parameters, 0x04
#define HCI_COMMAND_READ_RSSI                                   0x05    //OGF: Status Parameters, 0x05
#define HCI_COMMAND_LE_SET_EVENT_MASK                           0x01    //OGF: LE Controller, 0x08
#define HCI_COMMAND_LE_READ_BUFFER_SIZE                         0x02    //OGF: LE Controller, 0x08
#define HCI_COMMAND_LE_READ_LOCAL_SUPPORTED_FEATURES            0x03    //OGF: LE Controller, 0x08
#define HCI_COMMAND_LE_SET_RANDOM_ADDRESS                       0x05    //OGF: LE Controller, 0x08
#define HCI_COMMAND_LE_SET_ADVERTISING_PARAMETERS               0x06    //OGF: LE Controller, 0x08
#define HCI_COMMAND_LE_READ_ADVERTISING_CHANNEL_TX_POWER        0x07    //OGF: LE Controller, 0x08
#define HCI_COMMAND_LE_SET_ADVERTISING_DATA                     0x08    //OGF: LE Controller, 0x08
#define HCI_COMMAND_LE_SET_SCAN_RESPONSE_DATA                   0x09    //OGF: LE Controller, 0x08
#define HCI_COMMAND_LE_SET_ADVERTISE_ENABLE                     0x0A    //OGF: LE Controller, 0x08
#define HCI_COMMAND_LE_SET_SCAN_PARAMETERS                      0x0B    //OGF: LE Controller, 0x08
#define HCI_COMMAND_LE_SET_SCAN_ENABLE                          0x0C    //OGF: LE Controller, 0x08
#define HCI_COMMAND_LE_CREATE_CONNECTION                        0x0D    //OGF: LE Controller, 0x08
#define HCI_COMMAND_LE_CREATE_CONNECTION_CANCEL                 0x0E    //OGF: LE Controller, 0x08
#define HCI_COMMAND_LE_READ_WHITE_LIST_SIZE                     0x0F    //OGF: LE Controller, 0x08
#define HCI_COMMAND_LE_CLEAR_WHITE_LIST                         0x10    //OGF: LE Controller, 0x08
#define HCI_COMMAND_LE_ADD_DEVICE_TO_WHITE_LIST                 0x11    //OGF: LE Controller, 0x08
#define HCI_COMMAND_LE_REMOVE_DEVICE_FROM_WHITE_LIST_           0x12    //OGF: LE Controller, 0x08
#define HCI_COMMAND_LE_CONNECTION_UPDATE                        0x13    //OGF: LE Controller, 0x08
#define HCI_COMMAND_LE_SET_HOST_CHANNEL_CLASSIFICATION          0x14    //OGF: LE Controller, 0x08
#define HCI_COMMAND_LE_READ_CHANNEL_MAP                         0x15    //OGF: LE Controller, 0x08
#define HCI_COMMAND_LE_READ_REMOTE_USED_FEATURES                0x16    //OGF: LE Controller, 0x08
#define HCI_COMMAND_LE_ENCRYPT                                  0x17    //OGF: LE Controller, 0x08
#define HCI_COMMAND_LE_RAND                                     0x18    //OGF: LE Controller, 0x08
#define HCI_COMMAND_LE_START_ENCRYPTION                         0x19    //OGF: LE Controller, 0x08
#define HCI_COMMAND_LE_LONG_TERM_KEY_REQUEST_REPLY              0x1A    //OGF: LE Controller, 0x08
#define HCI_COMMAND_LE_LONG_TERM_KEY_REQUEST_NEGATIVE_REPLY     0x1B    //OGF: LE Controller, 0x08
#define HCI_COMMAND_LE_READ_SUPPORTED_STATES_                   0x1C    //OGF: LE Controller, 0x08
#define HCI_COMMAND_LE_RECEIVER_TEST                            0x1D    //OGF: LE Controller, 0x08
#define HCI_COMMAND_LE_TRANSMITTER_TEST                         0x1E    //OGF: LE Controller, 0x08
#define HCI_COMMAND_LE_TEST_END                                 0x1F    //OGF: LE Controller, 0x08
#define HCI_COMMAND_LE_REMOTE_CONN_PARAM_REQUEST_REPLY          0x20    //OGF: LE Controller, 0x08, HCI__020
#define HCI_COMMAND_LE_REMOTE_CONN_PARAM_REQUEST_NEG_REPLY      0x21    //OGF: LE Controller, 0x08, HCI__020

//BLUETOOTH SPECIFICATION Version 4.0 [Vol 2] page 733 of 1114
#define HCI_ERR_CODE_COMMAND_CURRENTLY_IN_PENDING                               0x00
#define HCI_ERR_CODE_ENCRYPTION_CHANGE_HAS_OCCURRED                             0x00
#endif

typedef uint8_t HCIStatus;
//HCI__008
#define HCI_ERR_CODE_SUCCESS                                                    0x00
#define HCI_ERR_CODE_UNKNOWN_HCI_COMMAND                                        0x01
#define HCI_ERR_CODE_UNKNOWN_CONNECTION_IDENTIFIER                              0x02
#define HCI_ERR_CODE_HARDWARE_FAILURE                                           0x03
#define HCI_ERR_CODE_PAGE_TIMEOUT                                               0x04
#define HCI_ERR_CODE_AUTHENTICATION_FAILURE                                     0x05
#define HCI_ERR_CODE_PIN_OR_KEY_MISSING                                         0x06
#define HCI_ERR_CODE_MEMORY_CAPACITY_EXCEEDED                                   0x07
#define HCI_ERR_CODE_CONNECTION_TIMEOUT                                         0x08
#define HCI_ERR_CODE_CONNECTION_LIMIT_EXCEEDED                                  0x09
#define HCI_ERR_CODE_SYNCHRONOUS_CONNECTION_LIMIT_TO_A_DEVICE_EXCEEDED          0x0A
#define HCI_ERR_CODE_ACL_CONNECTION_ALREADY_EXISTS                              0x0B
#define HCI_ERR_CODE_COMMAND_DISALLOWED                                         0x0C
#define HCI_ERR_CODE_CONNECTION_REJECTED_DUE_TO_LIMITED_RESOURCES               0x0D
#define HCI_ERR_CODE_CONNECTION_REJECTED_DUE_TO_SECURITY_REASONS                0x0E
#define HCI_ERR_CODE_CONNECTION_REJECTED_DUE_TO_UNACCEPTABLE_BD_ADDR            0x0F
#define HCI_ERR_CODE_CONNECTION_ACCEPT_TIMEOUT_EXCEEDED                         0x10
#define HCI_ERR_CODE_UNSUPPORTED_FEATURE_OR_PARAMETER_VALUE                     0x11
#define HCI_ERR_CODE_INVALID_HCI_COMMAND_PARAMETERS                             0x12
#define HCI_ERR_CODE_REMOTE_USER_TERMINATED_CONNECTION                          0x13
#define HCI_ERR_CODE_REMOTE_DEVICE_TERMINATED_CONNECTION_DUE_TO_LOW_RESOURCES   0x14
#define HCI_ERR_CODE_REMOTE_DEVICE_TERMINATED_CONNECTION_DUE_TO_POWER_OFF       0x15
#define HCI_ERR_CODE_CONNECTION_TERMINATED_BY_LOCAL_HOST                        0x16
#define HCI_ERR_CODE_REPEATED_ATTEMPTS                                          0x17
#define HCI_ERR_CODE_PAIRING_NOT_ALLOWED                                        0x18
#define HCI_ERR_CODE_UNKNOWN_LMP_PDU                                            0x19
#define HCI_ERR_CODE_UNSUPPORTED_REMOTE_FEATURE_UNSUPPORTED_LMP_FEATURE         0x1A
#define HCI_ERR_CODE_SCO_OFFSET_REJECTED                                        0x1B
#define HCI_ERR_CODE_SCO_INTERVAL_REJECTED                                      0x1C
#define HCI_ERR_CODE_SCO_AIR_MODE_REJECTED                                      0x1D
#define HCI_ERR_CODE_INVALID_LMP_PARAMETERS                                     0x1E
#define HCI_ERR_CODE_UNSPECIFIED_ERROR                                          0x1F
#define HCI_ERR_CODE_UNSUPPORTED_LMP_PARAMETER_VALUE                            0x20
#define HCI_ERR_CODE_ROLE_CHANGE_NOT_ALLOWED                                    0x21
#define HCI_ERR_CODE_LMP_RESPONSE_TIMEOUT_LL_RESPONSE_TIMEOUT                   0x22
#define HCI_ERR_CODE_LMP_ERROR_TRANSACTION_COLLISION                            0x23
#define HCI_ERR_CODE_LMP_PDU_NOT_ALLOWED                                        0x24
#define HCI_ERR_CODE_ENCRYPTION_MODE_NOT_ACCEPTABLE                             0x25
#define HCI_ERR_CODE_LINK_KEY_CANNOT_BE_CHANGED                                 0x26
#define HCI_ERR_CODE_REQUESTED_QOS_NOT_SUPPORTED                                0x27
#define HCI_ERR_CODE_INSTANT_PASSED                                             0x28
#define HCI_ERR_CODE_PAIRING_WITH_UNIT_KEY_NOT_SUPPORTED                        0x29
#define HCI_ERR_CODE_DIFFERENT_TRANSACTION_COLLISION                            0x2A
#define HCI_ERR_CODE_QOS_UNACCEPTABLE_PARAMETER                                 0x2C
#define HCI_ERR_CODE_QOS_REJECTED                                               0x2D
#define HCI_ERR_CODE_CHANNEL_CLASSIFICATION_NOT_SUPPORTED                       0x2E
#define HCI_ERR_CODE_INSUFFICIENT_SECURITY                                      0x2F
#define HCI_ERR_CODE_PARAMETER_OUT_OF_MANDATORY_RANGE                           0x30
#define HCI_ERR_CODE_ROLE_SWITCH_PENDING                                        0x32
#define HCI_ERR_CODE_RESERVED_SLOT_VIOLATION                                    0x34
#define HCI_ERR_CODE_ROLE_SWITCH_FAILED                                         0x35
#define HCI_ERR_CODE_EXTENDED_INQUIRY_RESPONSE_TOO_LARGE                        0x36
#define HCI_ERR_CODE_SECURE_SIMPLE_PAIRING_NOT_SUPPORTED_BY_HOST                0x37
#define HCI_ERR_CODE_HOST_BUSY_PAIRING                                          0x38
#define HCI_ERR_CODE_CONNECTION_REJECTED_DUE_TO_NO_SUITABLE_CHANNEL_FOUND       0x39
#define HCI_ERR_CODE_CONTROLLER_BUSY                                            0x3A
#define HCI_ERR_CODE_UNACCEPTABLE_CONNECTION_INTERVAL                           0x3B
#define HCI_ERR_CODE_DIRECTED_ADVERTISING_TIMEOUT                               0x3C
#define HCI_ERR_CODE_CONNECTION_TERMINATED_DUE_TO_MIC_FAILURE                   0x3D
#define HCI_ERR_CODE_CONNECTION_FAILED_TO_BE_ESTABLISHED                        0x3E
#define HCI_ERR_CODE_MAC_CONNECTION_FAILED                                      0x3F
#define HCI_ERR_CODE_COARSE_CLOCK_ADJUSTMENT_REJECTED                           0x40    //HCI__020

//HCI__009
#ifdef _HCI_NEW_
#define NUM_FLD_HCI_CFG_PARAMS_SUPP_CMDS                                 64
#define HCI_SUPP_CMD_INQUIRY                                                    0x01
#define HCI_SUPP_CMD_INQUIRY_CANCEL                                             0x02
#define HCI_SUPP_CMD_PERIODIC_INQUIRY_MODE                                      0x04
#define HCI_SUPP_CMD_EXIT_PERIODIC_INQUIRY_MODE                                 0x08
#define HCI_SUPP_CMD_CREATE_CONNECTION                                          0x10
#define HCI_SUPP_CMD_DISCONNECT                                                 0x20
#define HCI_SUPP_CMD_ADD_SCO_CONNECTION                                         0x40
#define HCI_SUPP_CMD_CANCEL_CREATE_CONNECTION                                   0x80
#define HCI_SUPP_CMD_ACCEPT_CONNECTION_REQUEST                                  0x01
#define HCI_SUPP_CMD_REJECT_CONNECTION_REQUEST                                  0x02
#define HCI_SUPP_CMD_LINK_KEY_REQUEST_REPLY                                     0x04
#define HCI_SUPP_CMD_LINK_KEY_REQUEST_NEGATIVE_REPLY                            0x08
#define HCI_SUPP_CMD_PIN_CODE_REQUEST_REPLY                                     0x10
#define HCI_SUPP_CMD_PIN_CODE_REQUEST_NEGATIVE_REPLY                            0x20
#define HCI_SUPP_CMD_CHANGE_CONNECTION_PACKET_TYPE                              0x40
#define HCI_SUPP_CMD_AUTHENTICATION_REQUEST                                     0x80
#define HCI_SUPP_CMD_SET_CONNECTION_ENCRYPTION                                  0x01
#define HCI_SUPP_CMD_CHANGE_CONNECTION_LINK_KEY                                 0x02
#define HCI_SUPP_CMD_MASTER_LINK_KEY                                            0x04
#define HCI_SUPP_CMD_REMOTE_NAME_REQUEST                                        0x08
#define HCI_SUPP_CMD_CANCEL_REMOTE_NAME_REQUEST                                 0x10
#define HCI_SUPP_CMD_READ_REMOTE_SUPPORTED_FEATURES                             0x20
#define HCI_SUPP_CMD_READ_REMOTE_EXTENDED_FEATURES                              0x40
#define HCI_SUPP_CMD_READ_REMOTE_VERSION_INFORMATION                            0x80
#define HCI_SUPP_CMD_READ_CLOCK_OFFSET                                          0x01
#define HCI_SUPP_CMD_READ_LMP_HANDLE                                            0x02
#define HCI_SUPP_CMD_HOLD_MODE                                                  0x02
#define HCI_SUPP_CMD_SNIFF_MODE                                                 0x04
#define HCI_SUPP_CMD_EXIT_SNIFF_MODE                                            0x08
#define HCI_SUPP_CMD_PARK_STATE                                                 0x10
#define HCI_SUPP_CMD_EXIT_PARK_STATE                                            0x20
#define HCI_SUPP_CMD_QOS_SETUP                                                  0x40
#define HCI_SUPP_CMD_ROLE_DISCOVERY                                             0x80
#define HCI_SUPP_CMD_SWITCH_ROLE                                                0x01
#define HCI_SUPP_CMD_READ_LINK_POLICY_SETTINGS                                  0x02
#define HCI_SUPP_CMD_WRITE_LINK_POLICY_SETTINGS                                 0x04
#define HCI_SUPP_CMD_READ_DEFAULT_LINK_POLICY_SETTINGS                          0x08
#define HCI_SUPP_CMD_WRITE_DEFAULT_LINK_POLICY_SETTINGS                         0x10
#define HCI_SUPP_CMD_FLOW_SPECIFICATION                                         0x20
#define HCI_SUPP_CMD_SET_EVENT_MARK                                             0x40
#define HCI_SUPP_CMD_RESET                                                      0x80
#define HCI_SUPP_CMD_SET_EVENT_FILTER                                           0x01
#define HCI_SUPP_CMD_FLUSH                                                      0x02
#define HCI_SUPP_CMD_READ_PIN_TYPE                                              0x04
#define HCI_SUPP_CMD_WRITE_PIN_TYPE                                             0x08
#define HCI_SUPP_CMD_CREATE_NEW_UNIT_KEY                                        0x10
#define HCI_SUPP_CMD_READ_STORED_LINK_KEY                                       0x20
#define HCI_SUPP_CMD_WRITE_STORED_LINK_KEY                                      0x40
#define HCI_SUPP_CMD_DELETE_STORED_LINK_KEY                                     0x80
#define HCI_SUPP_CMD_WRITE_LOCAL_NAME                                           0x01
#define HCI_SUPP_CMD_READ_LOCAL_NAME                                            0x02
#define HCI_SUPP_CMD_READ_CONNECTION_ACCEPT_TIMEOUT                             0x04
#define HCI_SUPP_CMD_WRITE_CONNECTION_ACCEPT_TIMEOUT                            0x08
#define HCI_SUPP_CMD_READ_PAGE_TIMEOUT                                          0x10
#define HCI_SUPP_CMD_WRITE_PAGE_TIMEOUT                                         0x20
#define HCI_SUPP_CMD_READ_SCAN_ENABLE                                           0x40
#define HCI_SUPP_CMD_WRITE_SCAN_ENABLE                                          0x80
#define HCI_SUPP_CMD_READ_PAGE_SCAN_ACTIVITY                                    0x01
#define HCI_SUPP_CMD_WRITE_PAGE_SCAN_ACTIVITY                                   0x02
#define HCI_SUPP_CMD_READ_INQUIRY_SCAN_ACTIVITY                                 0x04
#define HCI_SUPP_CMD_WRITE_INQUIRY_SCAN_ACTIVITY                                0x08
#define HCI_SUPP_CMD_READ_AUTHENTICATION_ENABLE                                 0x10    //HCI__020
#define HCI_SUPP_CMD_WRITE_AUTHENTICATION_ENABLE                                0x20    //HCI__020
#define HCI_SUPP_CMD_READ_ENCRYPTION_MODE                                       0x40
#define HCI_SUPP_CMD_WRITE_ENCRYPTION_MODE                                      0x80
#define HCI_SUPP_CMD_READ_CLASS_OF_DEVICE                                       0x01
#define HCI_SUPP_CMD_WRITE_CLASS_OF_DEVICE                                      0x02
#define HCI_SUPP_CMD_READ_VOICE_SETTING                                         0x04
#define HCI_SUPP_CMD_WRITE_VOICE_SETTING                                        0x08
#define HCI_SUPP_CMD_READ_AUTOMATIC_FLUSH_TIMEOUT                               0x10
#define HCI_SUPP_CMD_WRITE_AUTOMATIC_FLUSH_TIMEOUT                              0x20
#define HCI_SUPP_CMD_READ_NUM_BROADCAST_RETRANSMISSIONS                         0x40
#define HCI_SUPP_CMD_WRITE_NUM_BROADCAST_RETRANSMISSION                         0x80
#define HCI_SUPP_CMD_READ_HOLD_MODE_ACTIVITY                                    0x01
#define HCI_SUPP_CMD_WRITE_HOLD_MODE_ACTIVITY                                   0x02
#define HCI_SUPP_CMD_READ_TRANSMIT_POWER_LEVEL                                  0x04
#define HCI_SUPP_CMD_READ_SYNCHRONOUS_FLOW_CONTROL_ENABLE                       0x08
#define HCI_SUPP_CMD_WRITE_SYNCHRONOUS_FLOW_CONTROL_ENABLE                      0x10
#define HCI_SUPP_CMD_SET_HOST_CONTROLLER_TO_HOST_FLOW_CONTROL                   0x20
#define HCI_SUPP_CMD_HOST_BUFFER_SIZE                                           0x40
#define HCI_SUPP_CMD_HOST_NUMBER_OF_COMPLETED_PACKETS                           0x80
#define HCI_SUPP_CMD_READ_LINK_SUPERVISION_TIMEOUT_                             0x01
#define HCI_SUPP_CMD_WRITE_LINK_SUPERVISION_TIMEOUT                             0x02
#define HCI_SUPP_CMD_READ_NUMBER_OF_SUPPORTED_IAC                               0x04
#define HCI_SUPP_CMD_READ_CURRENT_IAC_LAP                                       0x08
#define HCI_SUPP_CMD_WRITE_CURRENT_IAC_LAP                                      0x10
#define HCI_SUPP_CMD_READ_PAGE_SCAN_MODE_PERIOD                                 0x20
#define HCI_SUPP_CMD_WRITE_PAGE_SCAN_MODE_PERIOD                                0x40
#define HCI_SUPP_CMD_READ_PAGE_SCAN_MODE                                        0x80
#define HCI_SUPP_CMD_WRITE_PAGE_SCAN_MODE                                       0x01
#define HCI_SUPP_CMD_SET_AFH_CHANNEL_CLASSIFICATION                             0x02
#define HCI_SUPP_CMD_READ_INQUIRY_SCAN_TYPE                                     0x10
#define HCI_SUPP_CMD_WRITE_INQUIRY_SCAN_TYPE                                    0x20
#define HCI_SUPP_CMD_READ_INQUIRY_MODE_                                         0x40
#define HCI_SUPP_CMD_WRITE_INQUIRY_MODE                                         0x80
#define HCI_SUPP_CMD_READ_PAGE_SCAN_TYPE                                        0x01
#define HCI_SUPP_CMD_WRITE_PAGE_SCAN_TYPE                                       0x02
#define HCI_SUPP_CMD_READ_AFH_CHANNEL_ASSESSMENT_MODE                           0x04
#define HCI_SUPP_CMD_WRITE_AFH_CHANNEL_ASSESSMENT_MODE                          0x08
#define HCI_SUPP_CMD_READ_LOCAL_VERSION_INFORMATION                             0x08
#define HCI_SUPP_CMD_READ_LOCAL_SUPPORTED_FEATURES                              0x20
#define HCI_SUPP_CMD_READ_LOCAL_EXTENDED_FEATURES                               0x40
#define HCI_SUPP_CMD_READ_BUFFER_SIZE                                           0x80
#define HCI_SUPP_CMD_READ_COUNTRY_CODE                                          0x01
#define HCI_SUPP_CMD_READ_BD_ADDR                                               0x02
#define HCI_SUPP_CMD_READ_FAILED_CONTACT_COUNT                                  0x04
#define HCI_SUPP_CMD_RESET_FAILED_CONTACT_COUNT                                 0x08
#define HCI_SUPP_CMD_GET_LINK_QUALITY                                           0x10
#define HCI_SUPP_CMD_READ_RSSI                                                  0x20
#define HCI_SUPP_CMD_READ_AFH_CHANNEL_MAP                                       0x40
#define HCI_SUPP_CMD_READ_BD_CLOCK                                              0x80
#define HCI_SUPP_CMD_READ_LOOPBACK_MODE                                         0x01
#define HCI_SUPP_CMD_WRITE_LOOPBACK_MODE                                        0x02
#define HCI_SUPP_CMD_ENABLE_DEVICE_UNDER_TEST_MODE                              0x04
#define HCI_SUPP_CMD_SETUP_SYNCHRONOUS_CONNECTION                               0x08
#define HCI_SUPP_CMD_ACCEPT_SYNCHRONOUS_CONNECTION                              0x10
#define HCI_SUPP_CMD_REJECT_SYNCHRONOUS_CONNECTION                              0x20
#define HCI_SUPP_CMD_READ_EXTENDED_INQUIRY_RESPONSE                             0x01
#define HCI_SUPP_CMD_WRITE_EXTENDED_INQUIRY_RESPONSE                            0x02
#define HCI_SUPP_CMD_REFRESH_ENCRYPTION_KEY                                     0x04
#define HCI_SUPP_CMD_SNIFF_SUBRATING                                            0x10
#define HCI_SUPP_CMD_READ_SIMPLE_PAIRING_MODE                                   0x20
#define HCI_SUPP_CMD_WRITE_SIMPLE_PAIRING_MODE                                  0x40
#define HCI_SUPP_CMD_READ_LOCAL_OOB_DATA                                        0x80
#define HCI_SUPP_CMD_READ_INQUIRY_RESPONSE_TRANSMIT_POWER                       0x01
#define HCI_SUPP_CMD_WRITE_INQUIRY_TRANSMIT_POWER_LEVEL                         0x02
#define HCI_SUPP_CMD_READ_DEFAULT_ERRONEOUS_DATA_REPORTING                      0x04
#define HCI_SUPP_CMD_WRITE_DEFAULT_ERRONEOUS_DATA_REPORTING                     0x08
#define HCI_SUPP_CMD_IO_CAPABILITY_REQUEST_REPLY                                0x80
#define HCI_SUPP_CMD_USER_CONFIRMATION_REQUEST_REPLY                            0x01
#define HCI_SUPP_CMD_USER_CONFIRMATION_REQUEST_NEGATIVE_REPLY                   0x02
#define HCI_SUPP_CMD_USER_PASSKEY_REQUEST_REPLY_                                0x04
#define HCI_SUPP_CMD_USER_PASSKEY_REQUEST_NEGATIVE_REPLY                        0x08
#define HCI_SUPP_CMD_REMOTE_OOB_DATA_REQUEST_REPLY                              0x10
#define HCI_SUPP_CMD_WRITE_SIMPLE_PAIRING_DEBUG_MODE                            0x20
#define HCI_SUPP_CMD_ENHANCED_FLUSH_                                            0x40
#define HCI_SUPP_CMD_REMOTE_OOB_DATA_REQUEST_NEGATIVE_REPLY                     0x80
#define HCI_SUPP_CMD_SEND_KEYPRESS_NOTIFICATION_                                0x04
#define HCI_SUPP_CMD_IO_CAPABILITY_REQUEST_NEGATIVE_REPLY                       0x08
#define HCI_SUPP_CMD_READ_ENCRYPTION_KEY_SIZE                                   0x10
#define HCI_SUPP_CMD_RESERVED                                                   0x00
#define HCI_SUPP_CMD_CREATE_PHYSICAL_LINK                                       0x01
#define HCI_SUPP_CMD_ACCEPT_PHYSICAL_LINK                                       0x02
#define HCI_SUPP_CMD_DISCONNECT_PHYSICAL_LINK                                   0x04
#define HCI_SUPP_CMD_CREATE_LOGICAL_LINK                                        0x08
#define HCI_SUPP_CMD_ACCEPT_LOGICAL_LINK                                        0x10
#define HCI_SUPP_CMD_DISCONNECT_LOGICAL_LINK                                    0x20
#define HCI_SUPP_CMD_LOGICAL_LINK_CANCEL                                        0x40
#define HCI_SUPP_CMD_FLOW_SPEC_MODIFY                                           0x80
#define HCI_SUPP_CMD_READ_LOGICAL_LINK_ACCEPT_TIMEOUT                           0x01
#define HCI_SUPP_CMD_WRITE_LOGICAL_LINK_ACCEPT_TIMEOUT                          0x02
#define HCI_SUPP_CMD_SET_EVENT_MASK_PAGE_2                                      0x04
#define HCI_SUPP_CMD_READ_LOCATION_DATA                                         0x08
#define HCI_SUPP_CMD_WRITE_LOCATION_DATA                                        0x10
#define HCI_SUPP_CMD_READ_LOCAL_AMP_INFO                                        0x20
#define HCI_SUPP_CMD_READ_LOCAL_AMP_ASSOC                                       0x40
#define HCI_SUPP_CMD_WRITE_REMOTE_AMP_ASSOC                                     0x80
#define HCI_SUPP_CMD_READ_FLOW_CONTROL_MODE                                     0x01
#define HCI_SUPP_CMD_WRITE_FLOW_CONTROL_MODE                                    0x02
#define HCI_SUPP_CMD_READ_DATA_BLOCK_SIZE                                       0x04
#define HCI_SUPP_CMD_ENABLE_AMP_RECEIVER_REPORTS                                0x20
#define HCI_SUPP_CMD_MP_TEST_END                                                0x40
#define HCI_SUPP_CMD_AMP_TEST_COMMAND                                           0x80
#define HCI_SUPP_CMD_READ_ENHANCED_TRANSMIT_POWER_LEVEL                         0x01
#define HCI_SUPP_CMD_READ_BEST_EFFORT_FLUSH_TIMEOUT                             0x04
#define HCI_SUPP_CMD_WRITE_BEST_EFFORT_FLUSH_TIMEOUT                            0x08
#define HCI_SUPP_CMD_SHORT_RANGE_MODE                                           0x10
#define HCI_SUPP_CMD_READ_LE_HOST_SUPPORT                                       0x20
#define HCI_SUPP_CMD_WRITE_LE_HOST_SUPPORT                                      0x40
#define HCI_SUPP_CMD_LE_SET_EVENT_MASK                                          0x01
#define HCI_SUPP_CMD_LE_READ_BUFFER_SIZE                                        0x02
#define HCI_SUPP_CMD_LE_READ_LOCAL_SUPPORTED_FEATURES                           0x04
#define HCI_SUPP_CMD_LE_SET_RANDOM_ADDRESS                                      0x10
#define HCI_SUPP_CMD_LE_SET_ADVERTISING_PARAMETERS                              0x20
#define HCI_SUPP_CMD_LE_READ_ADVERTISING_CHANNEL_TX_POWER                       0x40
#define HCI_SUPP_CMD_LE_SET_ADVERTISING_DATA                                    0x80
#define HCI_SUPP_CMD_LE_SET_SCAN_RESPONSE_DATA                                  0x01
#define HCI_SUPP_CMD_LE_SET_ADVERTISE_ENABLE                                    0x02
#define HCI_SUPP_CMD_LE_SET_SCAN_PARAMETERS                                     0x04
#define HCI_SUPP_CMD_LE_SET_SCAN_ENABLE                                         0x08
#define HCI_SUPP_CMD_LE_CREATE_CONNECTION                                       0x10
#define HCI_SUPP_CMD_LE_CREATE_CONNECTION_CANCEL                                0x20
#define HCI_SUPP_CMD_LE_READ_WHITE_LIST_SIZE                                    0x40
#define HCI_SUPP_CMD_LE_CLEAR_WHITE_LIST                                        0x80
#define HCI_SUPP_CMD_LE_ADD_DEVICE_TO_WHITE_LIST                                0x01
#define HCI_SUPP_CMD_LE_REMOVE_DEVICE_FROM_WHITE_LIST                           0x02
#define HCI_SUPP_CMD_LE_CONNECTION_UPDATE                                       0x04
#define HCI_SUPP_CMD_LE_SET_HOST_CHANNEL_CLASSIFICATION                         0x08
#define HCI_SUPP_CMD_LE_READ_CHANNEL_MAP                                        0x10
#define HCI_SUPP_CMD_LE_READ_REMOTE_USED_FEATURES                               0x20
#define HCI_SUPP_CMD_LE_ENCRYPT                                                 0x40
#define HCI_SUPP_CMD_LE_RAND                                                    0x80
#define HCI_SUPP_CMD_LE_START_ENCRYPTION                                        0x01
#define HCI_SUPP_CMD_LE_LONG_TERM_KEY_REQUEST_REPLY                             0x02
#define HCI_SUPP_CMD_LE_LONG_TERM_KEY_REQUESTED_NEGATIVE_REPLY                  0x04
#define HCI_SUPP_CMD_LE_READ_SUPPORTED_STATES                                   0x08
#define HCI_SUPP_CMD_LE_RECEIVER_TEST                                           0x10
#define HCI_SUPP_CMD_LE_TRANSMITTER_TEST                                        0x20
#define HCI_SUPP_CMD_LE_TEST_END                                                0x40
#define HCI_SUPP_CMD_SET_MWS_CHANNEL_PARAMETERS_COMMAND                         0x40    //HCI__020
#define HCI_SUPP_CMD_SET_EXTERNAL_FRAME_CONFIGURATION_COMMAND                   0x80    //HCI__020
#define HCI_SUPP_CMD_SET_MWS_SIGNALING_COMMAND                                  0x01    //HCI__020
#define HCI_SUPP_CMD_SET_TRANSPORT_LAYER_COMMAND                                0x02    //HCI__020
#define HCI_SUPP_CMD_SET_MWS_SCAN_FREQUENCY_TABLE_COMMAND                       0x04    //HCI__020
#define HCI_SUPP_CMD_GET_TRANSPORT_LAYER_CONFIGURATION_COMMAND                  0x08    //HCI__020
#define HCI_SUPP_CMD_SET_MWS_PATTERN_CONFIGURATION_COMMAND                      0x10    //HCI__020
#define HCI_SUPP_CMD_SET_TRIGGERED_CLOCK_CAPTURE                                0x20    //HCI__020
#define HCI_SUPP_CMD_TRUNCATED_PAGE                                             0x40    //HCI__020
#define HCI_SUPP_CMD_TRUNCATED_PAGE_CANCEL                                      0x80    //HCI__020
#define HCI_SUPP_CMD_SET_CONNECTIONLESS_SLAVE_BROADCAST                         0x01    //HCI__020
#define HCI_SUPP_CMD_SET_CONNECTIONLESS_SLAVE_BROADCAST_RECEIVE                 0x02    //HCI__020
#define HCI_SUPP_CMD_START_SYNCHRONIZATION_TRAIN                                0x04    //HCI__020
#define HCI_SUPP_CMD_RECEIVE_SYNCHRONIZATION_TRAIN                              0x08    //HCI__020
#define HCI_SUPP_CMD_SET_RESERVED_LT_ADDR                                       0x10    //HCI__020
#define HCI_SUPP_CMD_DELETE_RESERVED_LT_ADDR                                    0x20    //HCI__020
#define HCI_SUPP_CMD_SET_CONNECTIONLESS_SLAVE_BROADCAST_DATA                    0x40    //HCI__020
#define HCI_SUPP_CMD_READ_SYNCHRONIZATION_TRAIN_PARAMETERS                      0x80    //HCI__020
#define HCI_SUPP_CMD_WRITE_SYNCHRONIZATION_TRAIN_PARAMETERS                     0x01    //HCI__020
#define HCI_SUPP_CMD_REMOTE_OOB_EXTENDED_DATA_REQUEST_REPLY                     0x02    //HCI__020
#define HCI_SUPP_CMD_READ_SECURE_CONNECTIONS_HOST_SUPPORT                       0x04    //HCI__020
#define HCI_SUPP_CMD_WRITE_SECURE_CONNECTIONS_HOST_SUPPORT                      0x08    //HCI__020
#define HCI_SUPP_CMD_READ_AUTHENTICATED_PAYLOAD_TIMEOUT                         0x10    //HCI__020
#define HCI_SUPP_CMD_WRITE_AUTHENTICATED_PAYLOAD_TIMEOUT                        0x20    //HCI__020
#define HCI_SUPP_CMD_READ_LOCAL_OOB_EXTENDED_DATA                               0x40    //HCI__020
#define HCI_SUPP_CMD_WRITE_SECURE_CONNECTIONS_TEST_MODE                         0x80    //HCI__020
#define HCI_SUPP_CMD_READ_EXTENDED_PAGE_TIMEOUT                                 0x01    //HCI__020
#define HCI_SUPP_CMD_WRITE_EXTENDED_PAGE_TIMEOUT                                0x02    //HCI__020
#define HCI_SUPP_CMD_READ_EXTENDED_INQUIRY_LENGTH                               0x04    //HCI__020
#define HCI_SUPP_CMD_WRITE_EXTENDED_INQUIRY_LENGTH                              0x08    //HCI__020
#define HCI_SUPP_CMD_LE_REMOTE_CONNECTION_PARAMETER_REQUEST_REPLY_COMMAND       0x10    //HCI__020
#define HCI_SUPP_CMD_LE_REMOTE_CONNECTION_PARAMETER_REQUEST_NEGATIVE_REPLY_COMMAND  0x20//HCI__020


//HCI__010
#define HCI_LMP_FEAT_3_SLOT_PACKETS                                             0x01
#define HCI_LMP_FEAT_5_SLOT_PACKETS                                             0x02
#define HCI_LMP_FEAT_ENCRYPTION                                                 0x04
#define HCI_LMP_FEAT_SLOT_OFFSET                                                0x08
#define HCI_LMP_FEAT_TIMING_ACCURACY                                            0x10
#define HCI_LMP_FEAT_ROLE_SWITCH                                                0x20
#define HCI_LMP_FEAT_HOLD_MODE                                                  0x40
#define HCI_LMP_FEAT_SNIFF_MODE                                                 0x80
#define HCI_LMP_FEAT_PARK_STATE                                                 0x01
#define HCI_LMP_FEAT_POWER_CONTROL_REQUESTS                                     0x02
#define HCI_LMP_FEAT_CHANNEL_QUALITY_DRIVEN_DATA_RATE_CQDDR                     0x04
#define HCI_LMP_FEAT_SCO_LINK                                                   0x08
#define HCI_LMP_FEAT_HV2_PACKETS                                                0x10
#define HCI_LMP_FEAT_HV3_PACKETS                                                0x20
#define HCI_LMP_FEAT_U_LAW_LOG_SYNCHRONOUS_DATA                                 0x40
#define HCI_LMP_FEAT_A_LAW_LOG_SYNCHRONOUS_DATA                                 0x80
#define HCI_LMP_FEAT_CVSD_SYNCHRONOUS_DATA                                      0x01
#define HCI_LMP_FEAT_PAGING_PARAMETER_NEGOTIATION                               0x02
#define HCI_LMP_FEAT_POWER_CONTROL                                              0x04
#define HCI_LMP_FEAT_TRANSPARENT_SYNCHRONOUS_DATA                               0x08
#define HCI_LMP_FEAT_FLOW_CONTROL_LAG_LEAST_SIGNIFICANT_BIT                     0x10
#define HCI_LMP_FEAT_FLOW_CONTROL_LAG_MIDDLE_BIT                                0x20
#define HCI_LMP_FEAT_FLOW_CONTROL_LAG_MOST_SIGNIFICANT_BIT                      0x40
#define HCI_LMP_FEAT_BROADCAST_ENCRYPTION                                       0x80
#define HCI_LMP_FEAT_ENHANCED_DATA_RATE_ACL_2MBPS_MODE                          0x02
#define HCI_LMP_FEAT_ENHANCED_DATA_RATE_ACL_3MBPS_MODE                          0x04
#define HCI_LMP_FEAT_ENHANCED_INQUIRY_SCAN                                      0x08
#define HCI_LMP_FEAT_INTERLACED_INQUIRY_SCAN                                    0x10
#define HCI_LMP_FEAT_INTERLACED_PAGE_SCAN                                       0x20
#define HCI_LMP_FEAT_RSSI_WITH_INQUIRY_RESULTS                                  0x40
#define HCI_LMP_FEAT_EXTENDED_SCO_LINK_EV3_PACKETS                              0x80
#define HCI_LMP_FEAT_EV4_PACKETS                                                0x01
#define HCI_LMP_FEAT_EV5_PACKETS                                                0x02
#define HCI_LMP_FEAT_AFH_CAPABLE_SLAVE                                          0x08
#define HCI_LMP_FEAT_AFH_CLASSIFICATION_SLAVE                                   0x10
#define HCI_LMP_FEAT_BR_EDR_NOT_SUPPORTED                                       0x20
#define HCI_LMP_FEAT_LE_SUPPORTED_CONTROLLER                                    0x40
#define HCI_LMP_FEAT_3_SLOT_ENHANCED_DATA_RATE_ACL_PACKETS                      0x80
#define HCI_LMP_FEAT_5_SLOT_ENHANCED_DATA_RATE_ACL_PACKETS                      0x01
#define HCI_LMP_FEAT_SNIFF_SUBRATING                                            0x02
#define HCI_LMP_FEAT_PAUSE_ENCRYPTION                                           0x04
#define HCI_LMP_FEAT_AFH_CAPABLE_MASTER                                         0x08
#define HCI_LMP_FEAT_AFH_CLASSIFICATION_MASTER                                  0x10
#define HCI_LMP_FEAT_ENHANCED_DATA_RATE_ESCO_2MBPS_MODE                         0x20
#define HCI_LMP_FEAT_ENHANCED_DATA_RATE_ESCO_3MBPS_MODE                         0x40
#define HCI_LMP_FEAT_3_SLOT_ENHANCED_DATA_RATE_ESCO_PACKETS                     0x80
#define HCI_LMP_FEAT_EXTENDED_INQUIRY_RESPONSE                                  0x01
#define HCI_LMP_FEAT_SIMULTANEOUS_LE_AND_BR_EDR_TO_SAME_DEVICE_CAPA_BLE_CONTROLLER   0x02
#define HCI_LMP_FEAT_SECURE_SIMPLE_PAIRING                                      0x08
#define HCI_LMP_FEAT_ENCAPSULATED_PDU                                           0x10
#define HCI_LMP_FEAT_ERRONEOUS_DATA_REPORTING                                   0x20
#define HCI_LMP_FEAT_NON_FLUSHABLE_PACKET_BOUNDARY_FLAG                         0x40
#define HCI_LMP_FEAT_LINK_SUPERVISION_TIMEOUT_CHANGED_EVENT                     0x01
#define HCI_LMP_FEAT_INQUIRY_TX_POWER_LEVEL                                     0x02
#define HCI_LMP_FEAT_ENHANCED_POWER_CONTROL                                     0x04
#define HCI_LMP_FEAT_EXTENDED_FEATURES                                          0x80

#define HCI_LMP_FEAT_EXTD_P1_SECURE_SIMPLE_PAIRING                              0x01
#define HCI_LMP_FEAT_EXTD_P1_LE_SUPPORTED                                       0x02
#define HCI_LMP_FEAT_EXTD_P1_SIMULTANEOUS_LE_AND_BREDR_TO_SAME_DEVICE_CAPABLE   0x04
#define HCI_LMP_FEAT_EXTD_P1_SECURE_CONNECTIONS                                 0x08    //HCI__020

#define HCI_LMP_FEAT_EXTD_P2_CONNECTIONLESS_SLAVE_BROADCAST_MASTER_OPERATION    0x01    //HCI__020
#define HCI_LMP_FEAT_EXTD_P2_CONNECTIONLESS_SLAVE_BROADCAST_SLAVE_OPERATION     0x02    //HCI__020
#define HCI_LMP_FEAT_EXTD_P2_SYNCHRONIZATION_TRAIN                              0x04    //HCI__020
#define HCI_LMP_FEAT_EXTD_P2_SYNCHRONIZATION_SCAN                               0x08    //HCI__020
#define HCI_LMP_FEAT_EXTD_P2_INQUIRY_RESPONSE_NOTIFICATION_EVENT                0x10    //HCI__020
#define HCI_LMP_FEAT_EXTD_P2_GENERALIZED_INTERLACED_SCAN                        0x20    //HCI__020
#define HCI_LMP_FEAT_EXTD_P2_COARSE_CLOCK_ADJUSTMENT                            0x40    //HCI__020
#define HCI_LMP_FEAT_EXTD_P2_SECURE_CONNECTIONS_CONTROLLER_SUPPORT              0x01    //HCI__020
#define HCI_LMP_FEAT_EXTD_P2_PING                                               0x02    //HCI__020
#define HCI_LMP_FEAT_EXTD_P2_TRAIN_NUDGING                                      0x08    //HCI__020


//HCI__011
#define HCI_EVENT_COMMAND_COMPLETE                                              0x0E
#define HCI_EVENT_COMMAND_STATUS                                                0x0F
#define HCI_EVENT_DISCONNECTION_COMPLETE                                        0x05
#define HCI_EVENT_NUMBER_OF_COMPLETED_PACKETS                                   0x13
#define HCI_EVENT_READ_REMOTE_VERSION_INFORMATION_COMPLETE                      0x0C
#define HCI_EVENT_ENCRYPTION_CHANGE                                             0x08
#define HCI_EVENT_ENCRYPTION_KEY_REFRESH_COMPLETE                               0x30
#define HCI_EVENT_DATA_BUFFER_OVERFLOW                                          0x1A
#define HCI_EVENT_HARDWARE_ERROR                                                0x10
#define HCI_EVENT_LE_META                                                       0x3E
#define HCI_EVENT_AUTHENTICATED_PAYLOAD_TIMEOUT_EXPIRED                         0x57    //HCI__030

#define HCI_SUBEVENT_CODE_LE_CONNECTION_COMPLETE                                0x01
#define HCI_SUBEVENT_CODE_LE_ADVERTISING_REPORT                                 0x02
#define HCI_SUBEVENT_CODE_LE_CONNECTION_UPDATE_COMPLETE                         0x03
#define HCI_SUBEVENT_CODE_LE_READ_REMOTE_USED_FEATURES_COMPLETE                 0x04
#define HCI_SUBEVENT_CODE_LE_LONG_TERM_KEY_REQUEST                              0x05
#define HCI_SUBEVENT_CODE_LE_REMOTE_CONNECTION_PARAMETER_REQ                    0x06    //HCI__019
#define HCI_SUBEVENT_CODE_LE_DATA_LENGTH_CHANGE                                 0x07    //HCI__021
#define HCI_SUBEVENT_CODE_LE_READ_LOCAL_P256_PUBLIC_KEY_COMPLETE                0x08    //HCI__021, optional
#define HCI_SUBEVENT_CODE_LE_GENERATE_DHKEY_COMPLETE                            0x09    //HCI__021, optional
#define HCI_SUBEVENT_CODE_LE_ENHANCED_CONNECTION_COMPLETE                       0x0A    //HCI__021
#define HCI_SUBEVENT_CODE_LE_DIRECT_ADVERTISING_REPORT                          0x0B    //HCI__021
#define HCI_SUBEVENT_CODE_LE_PHY_UPDATE_COMPLETE                                0x0C

//LL__022
#define HCI_ADV_TYPE_ADV_IND                                            0x00
#define HCI_ADV_TYPE_ADV_DIRECT_IND_HIGH                                0x01
#define HCI_ADV_TYPE_ADV_SCAN_IND                                       0x02
#define HCI_ADV_TYPE_ADV_NONCONN_IND                                    0x03
#define HCI_ADV_TYPE_ADV_DIRECT_IND_LOW                                 0x04
#define HCI_ADV_TYPE_EVT_SCAN_RSP                                       0x04    //HCI__031
#define HCI_ADV_TYPE_EVT_UNKNOWN                                        0x05    //Jeffrey defined

#define HCI_ENCRPT_EN_OFF                                               0
#define HCI_ENCRPT_EN_AES_CCM_LE                                        1
#define HCI_ENCRPT_EN_AES_CCM_BEDR                                      2


//#define SIZE_UART_BUFFER                            MAX_MBLK_SIZE
#define SIZE_UART_BUFFER                            255

//extern Uint8 STATE_hci;

//extern Uint8 BLE_PDU_Type_HCI;

#ifdef _HCI_HW_
#ifdef _HCI_VIRTUAL_
extern uint8_t HCIDataPending;
extern uint8_t HCIBufferT[MAX_MBLK_SIZE];
extern uint8_t HCIBufferLen;
#else
extern Uint8 UARTBufValidT;             //For Tx
extern Uint8 UARTBufferT[SIZE_UART_BUFFER];    //UART buffer for Tx
extern Uint8 UARTBufferR[SIZE_UART_BUFFER];    //UART buffer for Rx
extern Uint8 UARTBufIdxR;         //UART buffer index for Rx
extern Uint8 UARTBufIdxT;         //UART buffer index for Tx
extern Uint8 UART_Tx_size;
extern Uint8 UART_Rx_size;
#endif
#endif

extern Uint8 Event_Mask[5];
extern Uint8 Event_Mask2[1];
extern Uint8 LE_Event_Mask[3];

extern const Uint8 SIZE_HCI_COMMAND_LE_CONTROLLER_PARAMETERS[(30+2)];

extern const Uint8 TAB_HCI_PKT_LTH_TABLE[5];

extern const Uint8 TAB_SUPPORTED_COMMANDS_MASK[NUM_FLD_HCI_CFG_PARAMS_SUPP_CMDS];
extern const Uint8 TAB_LMP_FEATURES_MASK[8];

extern void initUart(void);
#ifdef _HCI_HW_
extern void setUART_Tx(Uint8 * bufAddr, Uint8 length);
extern const uint8_t NUM_OGF_TABLE[];
extern const uint8_t *HCI_OGF_TABLE[];
extern const uint8_t *HCI_LENGTH_CMD_TABLE[];
extern const uint8_t *HCI_CMD_TABLE[];
#ifdef _HCI_HW_MSG_
//extern void msg2uart(Uint8 * CodeStr, Uint8 *HexStr, Uint8 HexStrLength);
#endif  //#ifdef _HCI_HW_MSG_
#ifdef _HCI_VIRTUAL_
extern void HCI_VIRTUAL_Init(void);
extern void LL2HCI_Return_Data(Uint8 * bufAddr, Uint8 length);
extern void LL_To_HCI(uint8_t *pHCIbufAddr, uint8_t *length);
extern void HCI_To_LL(uint8_t *pHCIcmd, uint8_t length);
#endif
#endif  //#ifdef _HCI_HW_
#endif  //#ifdef _HCI_NEW_

typedef void (*MHCCallBack)(BleCmdEvent event,
                            void *param);

#ifdef _DEBUG_MSG_USER_
extern void msg2uart(Uint8 * CodeStr, Uint8 *HexStr, Uint8 HexStrLength);
#endif

#endif   //#define _HCI_H_
