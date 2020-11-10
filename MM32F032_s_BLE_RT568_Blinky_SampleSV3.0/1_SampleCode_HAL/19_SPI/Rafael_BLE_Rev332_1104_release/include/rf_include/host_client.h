/**************************************************************************//**
* @file       host_client.h
* @brief      Provide the declerations that for BLE Client Host and Prfile subsystem needed.
*
* @copyright  All rights reserved. \n
*             (C) 2020 Rafael Microelectronics Co. Ltd. Taiwan, R.O.C.
*
*
*****************************************************************************/

#ifndef _BLE_HOST_CLIENT_H_
#define _BLE_HOST_CLIENT_H_

#include "host.h"

//Ref LL.h, same name
#define MAX_NUM_CONN_SLV_HDL                    1
#define MAX_NUM_CONN_MAS_HDL                    0
#define MAX_NUM_CONN_HDL                        MAX_NUM_CONN_MAS_HDL+MAX_NUM_CONN_SLV_HDL

#ifdef _HOST_CLIENT_

typedef struct ATT_Handle
{
    Uint16 handle_Num;
    Uint8 uuidType;
    Uint8 refProperty;
} ATT_Handle;

typedef struct ATT_Value
{
    Uint16 sizeATTvalue;
    const Uint8 *data;
} ATT_Value;


typedef struct ATTRIBUTE_CLIENT
{
    ATT_Handle *handle;
    const Uint8 *type;
    const ATT_Value *value;
    void (*ATTcallFunc)(uint8_t connID, uint8_t length, uint8_t *srcCMD);
} ATTRIBUTE_CLIENT;


typedef struct ATT_Rsp_RbGT
{
    Uint16 HDL_start;
    Uint16 HDL_end;
    Uint8 UUID[2];
} ATT_Rsp_RbGT;


typedef struct ATT_Rsp_RbGT_128
{
    Uint16 HDL_start;
    Uint16 HDL_end;
    Uint8 UUID[16];
} ATT_Rsp_RbGT_128;


typedef struct ATT_Rsp_RbT_CharC
{
    Uint16 HDL_start;
    Uint16 HDL_end;
    Uint8 UUID[2];
} ATT_Rsp_RbT_CharC;


typedef struct ATT_Rsp_RbT_CharC_128
{
    Uint16 HDL_start;
    Uint16 HDL_end;
    Uint8 UUID[16];
} ATT_Rsp_RbT_CharC_128;


///******************* 0x1800 through 0x26FF are for Service UUIDs ********************/
#define GATT_SERVICES_ALERT_NOTIFICATION_SERVICE            0x1811
#define GATT_SERVICES_AUTOMATION_IO                         0x1815
#define GATT_SERVICES_BATTERY_SERVICE                       0x180F
#define GATT_SERVICES_BLOOD_PRESSURE                        0x1810
#define GATT_SERVICES_CURRENT_TIME_SERVICE                  0x1805
#define GATT_SERVICES_CYCLING_POWER                         0x1818
#define GATT_SERVICES_CYCLING_SPEED_AND_CADENCE             0x1816
#define GATT_SERVICES_DEVICE_INFORMATION                    0x180A
#define GATT_SERVICES_GENERIC_ACCESS                        0x1800
#define GATT_SERVICES_GENERIC_ATTRIBUTE                     0x1801
#define GATT_SERVICES_GLUCOSE                               0x1808
#define GATT_SERVICES_HEALTH_THERMOMETER                    0x1809
#define GATT_SERVICES_HEART_RATE                            0x180D
#define GATT_SERVICES_HUMAN_INTERFACE_DEVICE                0x1812
#define GATT_SERVICES_IMMEDIATE_ALERT                       0x1802
#define GATT_SERVICES_LINK_LOSS                             0x1803
#define GATT_SERVICES_NETWORK_AVAILABILITY_SERVICE          0x180B
#define GATT_SERVICES_LOCATION_AND_NAVIGATION               0x1819
#define GATT_SERVICES_NEXT_DST_CHANGE_SERVICE               0x1807
#define GATT_SERVICES_PHONE_ALERT_STATUS_SERVICE            0x180E
#define GATT_SERVICES_REFERENCE_TIME_UPDATE_SERVICE         0x1806
#define GATT_SERVICES_RUNNING_SPEED_AND_CADENCE             0x1814
#define GATT_SERVICES_SCAN_PARAMETERS                       0x1813
#define GATT_SERVICES_TX_POWER                              0x1804
#define GATT_SERVICES_USER_DATA                             0x181C
#define GATT_SERVICES_AUTOMATION_IO                         0x1815
#define GATT_SERVICES_BODY_COMPOSITION                      0x181B
#define GATT_SERVICES_USER_DATA                             0x181C
#define GATT_SERVICES_WEIGHT_SCALE                          0x181D
#define GATT_SERVICES_BOND_MANAGEMENT                       0x181E
///************************************************************************************/

///******************* 0xF900 through 0xF9FF are for Customer Service UUIDs ********************/
#define GATT_CSTM_SERVICES_UDF01S                           0xF901
#define GATT_CSTM_SERVICES_UDF02S                           0xF902
#define GATT_CSTM_SERVICES_UDF03S                           0xF903
///************************************************************************************/

///*********** 0xFA00 through 0xFAFF are for Customer Characteristic Types ************/
#define GATT_CSTM_CHARC_UDATR01                             0xFA01
#define GATT_CSTM_CHARC_UDATN01                             0xFA02
#define GATT_CSTM_CHARC_UDATRW01                            0xFA03
#define GATT_CSTM_CHARC_UDATR02                             0xFA04
#define GATT_CSTM_CHARC_UDATN02                             0xFA05
#define GATT_CSTM_CHARC_UDATRW02                            0xFA06
#define GATT_CSTM_CHARC_UDATR03                             0xFA07
#define GATT_CSTM_CHARC_UDATN03                             0xFA08
#define GATT_CSTM_CHARC_UDATRW03                            0xFA09
///************************************************************************************/

///*********************** 0x2700 through 0x27FF are for Units ************************/
#define GATT_UNIT_UNITLESS                                                  0x2700
#define GATT_UNIT_LENGTH__METRE                                             0x2701
#define GATT_UNIT_MASS__KILOGRAM                                            0x2702
#define GATT_UNIT_TIME__SECOND                                              0x2703
#define GATT_UNIT_ELECTRIC_CURRENT__AMPERE                                  0x2704
#define GATT_UNIT_THERMODYNAMIC_TEMPERATURE__KELVIN                         0x2705
#define GATT_UNIT_AMOUNT_OF_SUBSTANCE__MOLE                                 0x2706
#define GATT_UNIT_LUMINOUS_INTENSITY__CANDELA                               0x2707
#define GATT_UNIT_AREA__SQUARE_METRES                                       0x2710
#define GATT_UNIT_VOLUME__CUBIC_METRES                                      0x2711
#define GATT_UNIT_VELOCITY__METRES_PER_SECOND                               0x2712
#define GATT_UNIT_ACCELERATION__METRES_PER_SECOND_SQUARED                   0x2713
#define GATT_UNIT_WAVENUMBER__RECIPROCAL_METRE                              0x2714
#define GATT_UNIT_DENSITY__KILOGRAM_PER_CUBIC_METRE                         0x2715
#define GATT_UNIT_SURFACE_DENSITY__KILOGRAM_PER_SQUARE_METRE                0x2716
#define GATT_UNIT_SPECIFIC_VOLUME__CUBIC_METRE_PER_KILOGRAM                 0x2717
#define GATT_UNIT_CURRENT_DENSITY__AMPERE_PER_SQUARE_METRE                  0x2718
#define GATT_UNIT_MAGNETIC_FIELD_STRENGTH__AMPERE_PER_METRE                 0x2719
#define GATT_UNIT_AMOUNT_CONCENTRATION__MOLE_PER_CUBIC_METRE                0x271A
#define GATT_UNIT_MASS_CONCENTRATION__KILOGRAM_PER_CUBIC_METRE              0x271B
#define GATT_UNIT_LUMINANCE__CANDELA_PER_SQUARE_METRE                       0x271C
#define GATT_UNIT_REFRACTIVE_INDEX                                          0x271D
#define GATT_UNIT_RELATIVE_PERMEABILITY                                     0x271E
#define GATT_UNIT_PLANE_ANGLE__RADIAN                                       0x2720
#define GATT_UNIT_SOLID_ANGLE__STERADIAN                                    0x2721
#define GATT_UNIT_FREQUENCY__HERTZ                                          0x2722
#define GATT_UNIT_FORCE__NEWTON                                             0x2723
#define GATT_UNIT_PRESSURE__PASCAL                                          0x2724
#define GATT_UNIT_ENERGY__JOULE                                             0x2725
#define GATT_UNIT_POWER__WATT                                               0x2726
#define GATT_UNIT_ELECTRIC_CHARGE__COULOMB                                  0x2727
#define GATT_UNIT_ELECTRIC_POTENTIAL_DIFFERENCE__VOLT                       0x2728
#define GATT_UNIT_CAPACITANCE__FARAD                                        0x2729
#define GATT_UNIT_ELECTRIC_RESISTANCE__OHM                                  0x272A
#define GATT_UNIT_ELECTRIC_CONDUCTANCE__SIEMENS                             0x272B
#define GATT_UNIT_MAGNETIC_FLEX__WEBER                                      0x272C
#define GATT_UNIT_MAGNETIC_FLEX_DENSITY__TESLA                              0x272D
#define GATT_UNIT_INDUCTANCE__HENRY                                         0x272E
#define GATT_UNIT_CELSIUS_TEMPERATURE__DEGREE_CELSIUS                       0x272F
#define GATT_UNIT_LUMINOUS_FLUX__LUMEN                                      0x2730
#define GATT_UNIT_ILLUMINANCE__LUX                                          0x2731
#define GATT_UNIT_ACTIVITY_REFERRED_TO_A_RADIONUCLIDE__BECQUEREL            0x2732
#define GATT_UNIT_ABSORBED_DOSE__GRAY                                       0x2733
#define GATT_UNIT_DOSE_EQUIVALENT__SIEVERT                                  0x2734
#define GATT_UNIT_CATALYTIC_ACTIVITY__KATAL                                 0x2735
#define GATT_UNIT_DYNAMIC_VISCOSITY__PASCAL_SECOND                          0x2740
#define GATT_UNIT_MOMENT_OF_FORCE__NEWTON_METRE                             0x2741
#define GATT_UNIT_SURFACE_TENSION__NEWTON_PER_METRE                         0x2742
#define GATT_UNIT_ANGULAR_VELOCITY__RADIAN_PER_SECOND                       0x2743
#define GATT_UNIT_ANGULAR_ACCELERATION__RADIAN_PER_SECOND_SQUARED           0x2744
#define GATT_UNIT_HEAT_FLUX_DENSITY__WATT_PER_SQUARE_METRE                  0x2745
#define GATT_UNIT_HEAT_CAPACITY__JOULE_PER_KELVIN                           0x2746
#define GATT_UNIT_SPECIFIC_HEAT_CAPACITY__JOULE_PER_KILOGRAM_KELVIN         0x2747
#define GATT_UNIT_SPECIFIC_ENERGY__JOULE_PER_KILOGRAM                       0x2748
#define GATT_UNIT_THERMAL_CONDUCTIVITY__WATT_PER_METRE_KELVIN               0x2749
#define GATT_UNIT_ENERGY_DENSITY__JOULE_PER_CUBIC_METRE                     0x274A
#define GATT_UNIT_ELECTRIC_FIELD_STRENGTH__VOLT_PER_METRE                   0x274B
#define GATT_UNIT_ELECTRIC_CHARGE_DENSITY__COULOMB_PER_CUBIC_METRE          0x274C
#define GATT_UNIT_SURFACE_CHARGE_DENSITY__COULOMB_PER_SQUARE_METRE          0x274D
#define GATT_UNIT_ELECTRIC_FLUX_DENSITY__COULOMB_PER_SQUARE_METRE           0x274E
#define GATT_UNIT_PERMITTIVITY__FARAD_PER_METRE                             0x274F
#define GATT_UNIT_PERMEABILITY__HENRY_PER_METRE                             0x2750
#define GATT_UNIT_MOLAR_ENERGY__JOULE_PER_MOLE                              0x2751
#define GATT_UNIT_MOLAR_ENTROPY__JOULE_PER_MOLE_KELVIN                      0x2752
#define GATT_UNIT_EXPOSURE__COULOMB_PER_KILOGRAM                            0x2753
#define GATT_UNIT_ABSORBED_DOSE_RATE__GRAY_PER_SECOND                       0x2754
#define GATT_UNIT_RADIANT_INTENSITY__WATT_PER_STERADIAN                     0x2755
#define GATT_UNIT_RADIANCE__WATT_PER_SQUARE_METER_STERADIAN                 0x2756
#define GATT_UNIT_CATALYTIC_ACTIVITY_CONCENTRATION__KATAL_PER_CUBIC_METRE   0x2757
#define GATT_UNIT_TIME__MINUTE                                              0x2760
#define GATT_UNIT_TIME__HOUR                                                0x2761
#define GATT_UNIT_TIME__DAY                                                 0x2762
#define GATT_UNIT_PLANE_ANGLE__DEGREE                                       0x2763
#define GATT_UNIT_PLANE_ANGLE__MINUTE                                       0x2764
#define GATT_UNIT_PLANE_ANGLE__SECOND                                       0x2765
#define GATT_UNIT_AREA__HECTARE                                             0x2766
#define GATT_UNIT_VOLUME__LITRE                                             0x2767
#define GATT_UNIT_MASS__TONNE                                               0x2768
#define GATT_UNIT_PRESSURE__BAR                                             0x2780
#define GATT_UNIT_PRESSURE__MILLIMETRE_OF_MERCURY                           0x2781
#define GATT_UNIT_LENGTH__ANGSTROM                                          0x2782
#define GATT_UNIT_LENGTH__NAUTICAL_MILE                                     0x2783
#define GATT_UNIT_AREA__BARN                                                0x2784
#define GATT_UNIT_VELOCITY__KNOT                                            0x2785
#define GATT_UNIT_LOGARITHMIC_RADIO_QUANTITY__NEPER                         0x2786
#define GATT_UNIT_LOGARITHMIC_RADIO_QUANTITY__BEL                           0x2787
#define GATT_UNIT_LENGTH__YARD                                              0x27A0
#define GATT_UNIT_LENGTH__PARSEC                                            0x27A1
#define GATT_UNIT_LENGTH__INCH                                              0x27A2
#define GATT_UNIT_LENGTH__FOOT                                              0x27A3
#define GATT_UNIT_LENGTH__MILE                                              0x27A4
#define GATT_UNIT_PRESSURE__POUND_FORCE_PER_SQUARE_INCH                     0x27A5
#define GATT_UNIT_VELOCITY__KILOMETRE_PER_HOUR                              0x27A6
#define GATT_UNIT_VELOCITY__MILE_PER_HOUR                                   0x27A7
#define GATT_UNIT_ANGULAR_VELOCITY__REVOLUTION_PER_MINUTE                   0x27A8
#define GATT_UNIT_ENERGY__GRAM_CALORIE                                      0x27A9
#define GATT_UNIT_ENERGY__KILOGRAM_CALORIE                                  0x27AA
#define GATT_UNIT_ENERGY__KILOWATT_HOUR                                     0x27AB
#define GATT_UNIT_THERMODYNAMIC_TEMPERATURE__DEGREE_FAHRENHEIT              0x27AC
#define GATT_UNIT_PERCENTAGE                                                0x27AD
#define GATT_UNIT_PER_MILLE                                                 0x27AE
#define GATT_UNIT_PERIOD__BEATS_PER_MINUTE                                  0x27AF
#define GATT_UNIT_ELECTRIC_CHARGE__AMPERE_HOURS                             0x27B0
#define GATT_UNIT_MASS_DENSITY__MILLIGRAM_PER_DECILITRE                     0x27B1
#define GATT_UNIT_MASS_DENSITY__MILLIMOLE_PER_LITRE                         0x27B2
#define GATT_UNIT_TIME__YEAR                                                0x27B3
#define GATT_UNIT_TIME__MONTH                                               0x27B4
///************************************************************************************/

//Bluetooth Spec. Ver5.0, Vol 3, Part G, 3
///****************** 0x2800 through 0x28FF are for Attribute Types *******************/
#define GATT_DECL_PRIMARY_SERVICE                                           0x2800
#define GATT_DECL_SECONDARY_SERVICE                                         0x2801
#define GATT_DECL_INCLUDE                                                   0x2802
#define GATT_DECL_CHARACTERISTIC                                            0x2803


//https://www.bluetooth.com/specifications/gatt/descriptors/
///************* 0x2900 through 0x29FF are for Characteristic Descriptors *************/
#define GATT_DESC_CHARC_AGGREGATE_FORMAT                            0x2905    //Bluetooth Spec. Ver5.0, Vol 3, Part G, 3.3.3.6
#define GATT_DESC_CHARC_EXTENDED_PROPERTIES                         0x2900    //Bluetooth Spec. Ver5.0, Vol 3, Part G, 3.3.3.1
#define GATT_DESC_CHARC_PRESENTATION_FORMAT                         0x2904    //Bluetooth Spec. Ver5.0, Vol 3, Part G, 3.3.3.5
#define GATT_DESC_CHARC_USER_DESCRIPTION                            0x2901    //Bluetooth Spec. Ver5.0, Vol 3, Part G, 3.3.3.2
#define GATT_DESC_CLIENT_CHAR_CONFIGURATION                         0x2902    //Bluetooth Spec. Ver5.0, Vol 3, Part G, 3.3.3.3
#define GATT_DESC_EXTERNAL_REPORT_REFERENCE                         0x2907
#define GATT_DESC_REPORT_REFERENCE                                  0x2908
#define GATT_DESC_SERVER_CHAR_CONFIGURATION                         0x2903    //Bluetooth Spec. Ver5.0, Vol 3, Part G, 3.3.3.4
#define GATT_DESC_VALID_RANGE                                       0x2906
///************************************************************************************/


//https://www.bluetooth.com/specifications/gatt/characteristics/
///**************** 0x2A00 through 0x7FFF are for Characteristic Types ****************/
#define GATT_SPEC_CHARC_ALERT_CATEGORY_ID                               0x2A43
#define GATT_SPEC_CHARC_ALERT_CATEGORY_ID_BIT_MASK                      0x2A42
#define GATT_SPEC_CHARC_ALERT_LEVEL                                     0x2A06
#define GATT_SPEC_CHARC_ALERT_NOTIFICATION_CONTROL_POINT                0x2A44
#define GATT_SPEC_CHARC_ALERT_STATUS                                    0x2A3F
#define GATT_SPEC_CHARC_APPEARANCE                                      0x2A01
#define GATT_SPEC_CHARC_BATTERY_LEVEL                                   0x2A19
#define GATT_SPEC_CHARC_BLOOD_PRESSURE_FEATURE                          0x2A49
#define GATT_SPEC_CHARC_BLOOD_PRESSURE_MEASUREMENT                      0x2A35
#define GATT_SPEC_CHARC_BODY_SENSOR_LOCATION                            0x2A38
#define GATT_SPEC_CHARC_BOOT_KEYBOARD_INPUT_REPORT                      0x2A22
#define GATT_SPEC_CHARC_BOOT_KEYBOARD_OUTPUT_REPORT                     0x2A32
#define GATT_SPEC_CHARC_BOOT_MOUSE_INPUT_REPORT                         0x2A33
#define GATT_SPEC_CHARC_CSC_FEATURE                                     0x2A5C
#define GATT_SPEC_CHARC_CSC_MEASUREMENT                                 0x2A5B
#define GATT_SPEC_CHARC_CURRENT_TIME                                    0x2A2B
#define GATT_SPEC_CHARC_CYCLING_POWER_CONTROL_POINT                     0x2A66
#define GATT_SPEC_CHARC_CYCLING_POWER_FEATURE                           0x2A65
#define GATT_SPEC_CHARC_CYCLING_POWER_MEASUREMENT                       0x2A63
#define GATT_SPEC_CHARC_CYCLING_POWER_VECTOR                            0x2A64
#define GATT_SPEC_CHARC_DATE_TIME                                       0x2A08
#define GATT_SPEC_CHARC_DAY_DATE_TIME                                   0x2A0A
#define GATT_SPEC_CHARC_DAY_OF_WEEK                                     0x2A09
#define GATT_SPEC_CHARC_DEVICE_NAME                                     0x2A00
#define GATT_SPEC_CHARC_DST_OFFSET                                      0x2A0D
#define GATT_SPEC_CHARC_ExACT_TIME_256                                  0x2A0C
#define GATT_SPEC_CHARC_FIRMWARE_REVISION_STRING                        0x2A26
#define GATT_SPEC_CHARC_GLUCOSE_FEATURE                                 0x2A51
#define GATT_SPEC_CHARC_GLUCOSE_MEASUREMENT                             0x2A18
#define GATT_SPEC_CHARC_GLUCOSE_MEASUREMENT_CONTEXT                     0x2A34
#define GATT_SPEC_CHARC_HARDWARE_REVISION_STRING                        0x2A27
#define GATT_SPEC_CHARC_HEART_RATE_CONTROL_POINT                        0x2A39
#define GATT_SPEC_CHARC_HEART_RATE_MEASUREMENT                          0x2A37
#define GATT_SPEC_CHARC_HID_CONTROL_POINT                               0x2A4C
#define GATT_SPEC_CHARC_HID_INFORMATION                                 0x2A4A
#define GATT_SPEC_CHARC_IEEE_11073_REGULATORY_CERTIFICATION_DATA_LIST   0x2A2A
#define GATT_SPEC_CHARC_INTERMEDIATE_CUFF_PRESSURE                      0x2A36
#define GATT_SPEC_CHARC_INTERMEDIATE_TEMPERATURE                        0x2A1E
#define GATT_SPEC_CHARC_NAVIGATION                                      0x2A68
#define GATT_SPEC_CHARC_LN_CONTROL_POINT                                0x2A6B
#define GATT_SPEC_CHARC_LN_FEATURE                                      0x2A6A
#define GATT_SPEC_CHARC_LOCAL_TIME_INFORMATION                          0x2A0F
#define GATT_SPEC_CHARC_LOCATION_AND_SPEED                              0x2A67
#define GATT_SPEC_CHARC_MANUFACTURER_NAME_STRING                        0x2A29
#define GATT_SPEC_CHARC_MEASUREMENT_INTERVAL                            0x2A21
#define GATT_SPEC_CHARC_MODEL_NUMBER_STRING                             0x2A24
#define GATT_SPEC_CHARC_NEW_ALERT                                       0x2A46
#define GATT_SPEC_CHARC_PERIPHERAL_PREFERRED_CONNECTION_PARAMETERS      0x2A04
#define GATT_SPEC_CHARC_PERIPHERAL_PRIVACY_FLAG                         0x2A02
#define GATT_SPEC_CHARC_PNP_ID                                          0x2A50
#define GATT_SPEC_CHARC_POSITION_QUALITY                                0x2A69
#define GATT_SPEC_CHARC_PROTOCOL_MODE                                   0x2A4E
#define GATT_SPEC_CHARC_RECONNECTION_ADDRESS                            0x2A03
#define GATT_SPEC_CHARC_RECORD_ACCESS_CONTROL_POINT                     0x2A52
#define GATT_SPEC_CHARC_REFERENCE_TIME_INFORMATION                      0x2A14
#define GATT_SPEC_CHARC_REPORT                                          0x2A4D
#define GATT_SPEC_CHARC_REPORT_MAP                                      0x2A4B
#define GATT_SPEC_CHARC_RINGER_CONTROL_POINT                            0x2A40
#define GATT_SPEC_CHARC_RINGER_SETTING                                  0x2A41
#define GATT_SPEC_CHARC_RSC_FEATURE                                     0x2A54
#define GATT_SPEC_CHARC_RSC_MEASUREMENT                                 0x2A53
#define GATT_SPEC_CHARC_SC_CONTROL_POINT                                0x2A55
#define GATT_SPEC_CHARC_SCAN_INTERVAL_WINDOW                            0x2A4F
#define GATT_SPEC_CHARC_SCAN_REFRESH                                    0x2A31
#define GATT_SPEC_CHARC_SENSOR_LOCATION                                 0x2A5D
#define GATT_SPEC_CHARC_SERIAL_NUMBER_STRING                            0x2A25
#define GATT_SPEC_CHARC_SERVICE_CHANGED                                 0x2A05
#define GATT_SPEC_CHARC_SOFTWARE_REVISION_STRING                        0x2A28
#define GATT_SPEC_CHARC_SUPPORTED_NEW_ALERT_CATEGORY                    0x2A47
#define GATT_SPEC_CHARC_SUPPORTED_UNREAD_ALERT_CATEGORY                 0x2A48
#define GATT_SPEC_CHARC_SYSTEM_ID                                       0x2A23
#define GATT_SPEC_CHARC_TEMPERATURE_MEASUREMENT                         0x2A1C
#define GATT_SPEC_CHARC_TEMPERATURE_TYPE                                0x2A1D
#define GATT_SPEC_CHARC_TIME_ACCURACY                                   0x2A12
#define GATT_SPEC_CHARC_TIME_SOURCE                                     0x2A13
#define GATT_SPEC_CHARC_TIME_UPDATE_CONTROL_POINT                       0x2A16
#define GATT_SPEC_CHARC_TIME_UPDATE_STATE                               0x2A17
#define GATT_SPEC_CHARC_TIME_WITH_DST                                   0x2A11
#define GATT_SPEC_CHARC_TIME_ZONE                                       0x2A0E
#define GATT_SPEC_CHARC_TX_POWER_LEVEL                                  0x2A07
#define GATT_SPEC_CHARC_UNREAD_ALERT_STATUS                             0x2A45
#define GATT_SPEC_CHARC_AGGREGATE_INPUT                                 0x2A5A
#define GATT_SPEC_CHARC_ANALOG_INPUT                                    0x2A58
#define GATT_SPEC_CHARC_ANALOG_OUTPUT                                   0x2A59
#define GATT_SPEC_CHARC_DIGITAL_INPUT                                   0x2A56
#define GATT_SPEC_CHARC_DIGITAL_OUTPUT                                  0x2A57
#define GATT_SPEC_CHARC_EXACT_TIME_100                                  0x2A0B
#define GATT_SPEC_CHARC_NETWORK_AVAILABILITY                            0x2A3E
#define GATT_SPEC_CHARC_SCIENTIFIC_TEMPERATURE_IN_CELSIUS               0x2A3C
#define GATT_SPEC_CHARC_SECONDARY_TIME_ZONE                             0x2A10
#define GATT_SPEC_CHARC_STRING                                          0x2A3D
#define GATT_SPEC_CHARC_TEMPERATURE_IN_CELSIUS                          0x2A1F
#define GATT_SPEC_CHARC_TEMPERATURE_IN_FAHRENHEIT                       0x2A20
#define GATT_SPEC_CHARC_TIME_BROADCAST                                  0x2A15
#define GATT_SPEC_CHARC_BATTERY_LEVEL_STATE                             0x2A1B
#define GATT_SPEC_CHARC_BATTERY_POWER_STATE                             0x2A1A
#define GATT_SPEC_CHARC_PULSE_OXIMETRY_CONTINUOUS_MEASUREMENT           0x2A5F
#define GATT_SPEC_CHARC_PULSE_OXIMETRY_CONTROL_POINT                    0x2A62
#define GATT_SPEC_CHARC_PULSE_OXIMETRY_FEATURES                         0x2A61
#define GATT_SPEC_CHARC_PULSE_OXIMETRY_PULSATILE_EVENT                  0x2A60
///************************************************************************************/

#define HS_ATT_CLI_SRV_S0       0
#define HS_ATT_CLI_SRV_S1       1
#define HS_ATT_CLI_SRV_END      0xFFFF

#define HS_ATT_CLI_CHR_S0       0
#define HS_ATT_CLI_CHR_S255     0xFF

#endif  //(_HOST_CLIENT_)
#endif
