#ifndef _MMI_H_
#define _MMI_H_


/*      For STATE_Program        */
//#define PROGRAM_S0       0x00
//#define PROGRAM_S1       (PROGRAM_S0 + 1)
//#define PROGRAM_S2       (PROGRAM_S0 + 2)
//#define PROGRAM_S3       (PROGRAM_S0 + 3)
//#define PROGRAM_S4       (PROGRAM_S0 + 4)
//#define PROGRAM_S5       (PROGRAM_S0 + 5)
//#define PROGRAM_S6       (PROGRAM_S0 + 6)
//#define PROGRAM_S7       (PROGRAM_S0 + 7)
//#define PROGRAM_S8       (PROGRAM_S0 + 8)

//#ifdef _PROFILE_HOGP_
//#ifndef _PROFILE_HOGP_EMC_
//#define STATE_HID_REPORT_CS_INITIAL             0
//#define STATE_HID_REPORT_CS_CONN_UPD_REQ        0x01
//#define STATE_HID_REPORT_CS_DATA_UPD_AVLBLE     0x02
//#define STATE_HID_REPORT_CS_DATA_UPD            0x04
//#define STATE_HID_REPORT_CS_DATA_UPD_RELS       0x08
//#define STATE_HID_REPORT_CS_DATA_UPD_ONCE       0x10
//#else       //#ifndef _PROFILE_HOGP_EMC_
//#define STATE_HID_REPORT_81_INITIAL             0
//#define STATE_HID_REPORT_81_CONN_UPD_REQ        0x01
//#define STATE_HID_REPORT_81_DATA_UPD_AVLBLE     0x02
//#define STATE_HID_REPORT_81_DATA_UPD            0x04
//#define STATE_HID_REPORT_81_DATA_UPD_RELS       0x08
//#define STATE_HID_REPORT_81_DATA_UPD_ONCE       0x10
//#endif      //#ifndef _PROFILE_HOGP_EMC_
//#define STATE_HID_REPORT_KB_INITIAL             0
//#define STATE_HID_REPORT_KB_CONN_UPD_REQ        0x01
//#define STATE_HID_REPORT_KB_DATA_UPD_AVLBLE     0x02
//#define STATE_HID_REPORT_KB_DATA_UPD            0x04
//#define STATE_HID_REPORT_KB_DATA_UPD_RELS       0x08
//#define STATE_HID_REPORT_KB_DATA_UPD_ONCE       0x10
//#define STATE_HID_REPORT_MS_INITIAL             0
//#define STATE_HID_REPORT_MS_CONN_UPD_REQ        0x01
//#define STATE_HID_REPORT_MS_DATA_UPD_AVLBLE     0x02
//#define STATE_HID_REPORT_MS_DATA_UPD            0x04
//#define STATE_HID_REPORT_MS_DATA_UPD_RELS       0x08
//#define STATE_HID_REPORT_MS_DATA_UPD_ONCE       0x10
//#endif      //#ifdef _PROFILE_HOGP_

//#ifdef _PROFILE_LNS_
//#define STATE_LNS_LN_CP_INITIAL                 0
//#define STATE_LNS_LN_CP_OP_WRITED               0x20
//#define STATE_LNS_LN_CP_OP_INDICATED            0x40
//#endif
//#ifdef _PROFILE_GLS_
//#define STATE_GLS_RA_CP_INITIAL                 0
//#define STATE_GLS_RA_CP_OP_WRITED               0x20
//#define STATE_GLS_RA_CP_OP_INDICATED            0x40
//#define STATE2_GLS_RA_CP_OP_PRCSS_RUN            0x80
//#endif
//#ifdef _PROFILE_CPS_
//#define STATE_CPS_CP_CP_INITIAL                 0
//#define STATE_CPS_CP_CP_OP_WRITED               0x20
//#define STATE_CPS_CP_CP_OP_INDICATED            0x40
//#endif

//#ifdef _PROFILE_OTA_
//#define STATE_OTA_DATA_INITIAL                  0
//#define STATE_OTA_CMD_INITIAL                   0
//#define STATE_OTA_CMD_CP_OP_WRITED              0x20
//#define STATE_OTA_CMD_CP_OP_INDICATED             0x40
//#define STATE_OTA_DATA_WRITED                   0x20
//#define STATE_OTA_DATA_NOTIFIED                 0x40
//#endif

////extern Uint8 STATE_Program;
//#ifdef _PROFILE_HOGP_
//#ifdef _PROFILE_HOGP_COMSUMER_
//extern Uint8 STATE_HID_reportCS;
//extern Uint8 HID_report_CS_key_temp[2];
//#endif
//#ifdef _PROFILE_HOGP_EMC_
//extern Uint8 STATE_HID_report81;
//extern Uint8 HID_report_81_key_temp;
//#endif
//#ifdef _PROFILE_HOGP_KEYBOARD_
//extern Uint8 STATE_HID_reportKB;
//extern Uint8 HID_report_KB_key_temp;
//#endif
//#ifdef _PROFILE_HOGP_MOUSE_
//extern Uint8 STATE_HID_reportMS;
//extern Uint8 HID_report_MS_key_temp;
//#endif
//#endif      //#ifdef _PROFILE_HOGP_

//#ifdef _PROFILE_CSCP_
//extern Uint8 CSCS_report_temp;
//#endif

//#ifdef _PROFILE_LNS_
//extern Uint8 STATE_LNS_LN_cp;
//extern Uint8 LNS_cumulative[3];
//extern Uint8 mask_LNS[1];
//#endif
//#ifdef _PROFILE_GLS_
//extern Uint8 STATE_GLS_RA_cp;
//extern Uint8 Code_Prcss_GLS_RACP_OP_RUN_STOR[];
//#define SZIE_GLS_MEAS_REC               15
//extern const Uint8 GLS_MEAS_REC[][SZIE_GLS_MEAS_REC];
//extern Uint8 length_GLS_MEAS_REC;
//extern Uint16 sn_base_GLS_MEAS_REC;
//extern Uint8 tab_length_GLS_MEAS_REC;
//extern Uint8 tab_length_max_GLS_MEAS_REC;
//#endif
//#ifdef _PROFILE_CPS_
//extern Uint8 STATE_CPS_CP_cp;
//extern Uint8 CPS_crank_lgth[2];
//#endif

//#ifdef _PROFILE_OTA_
//extern Uint8 state0_OTA_DATA;
//extern Uint8 state0_OTA_CMD;
//#endif

extern Uint8 ATT_INDICATION_Packet_Sent;

#endif
