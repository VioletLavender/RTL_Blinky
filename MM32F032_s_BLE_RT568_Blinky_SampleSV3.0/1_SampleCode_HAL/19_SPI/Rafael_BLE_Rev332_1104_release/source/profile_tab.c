#ifndef _RFFUNCTION_C_
#define _PROFILE_TAB_C_

#include <stdio.h>
#include "BleAppSetting.h"
#include "host.h"
#include "user.h"

const Uint8 ATT_HDL_INVALID[] =
{
    0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00
};                                                                                             //Handle, invalid


const Uint8 ATT_HDL_GAP_PRIMARY_SERVICE[] =                                                    //GAP service: Mandatory
{
    0x00, _GAP_HDL_OFFSET_BASE_,                                            //0x00 0x01                                   //Handle
    GATT_DECLARATIONS, GATT_DECLARATIONS_PRIMARY_SERVICE,                   //0x28 0x00                                   //Attribute Type (0x28, 0x00)
    SIZE_OF_ATTRIBUTE_VALUE_2,                                              //0x02
    GATT_SPEC_SERVICES_GENERIC_ACCESS, GATT_SPEC_SERVICES                   //0x00 0x18                                   //Attribute Value (0x00, 0x18)
};


const Uint8 ATT_HDL_GAP_CHARACTERISTIC_DEVICE_NAME[] =                                         //Mandatory
{
    0x00, (_GAP_HDL_OFFSET_BASE_ + 1),                                      //0x00  0x02                                  //Handle
    GATT_DECLARATIONS, GATT_DECLARATIONS_CHARACTERISTIC,                    //0x28  0x03                                 //Attribute Type (0x28, 0x03)
    SIZE_OF_ATTRIBUTE_VALUE_5,                                              //0x05
    //0x02,
    (
        //Characteristic Properties, Read: Mandatory
        //GATT_DECLARATIONS_PROPERTIES_BROADCAST |
        GATT_DECLARATIONS_PROPERTIES_READ |                                 //0x02
        //GATT_DECLARATIONS_PROPERTIES_WRITE_WITHOUT_RESPONSE |
        GATT_DECLARATIONS_PROPERTIES_WRITE |                                //0x08
        //GATT_DECLARATIONS_PROPERTIES_NOTIFY |
        //GATT_DECLARATIONS_PROPERTIES_INDICATE |
        //GATT_DECLARATIONS_PROPERTIES_AUTHENTICATED_SIGNED_WRITES |
        //GATT_DECLARATIONS_PROPERTIES_EXTENDED_PROPERTIES |
        0x00                                                                //0x00
    ),
    (_GAP_HDL_OFFSET_BASE_ + 2), 0x00,                                      //0x02 0x00
    GATT_SPEC_CHARACTERISTIC_DEVICE_NAME, GATT_SPEC_CHARACTERISTIC          //0x00 0x2A                   //Attribute Value (0x00, 0x2A)
};


const Uint8 ATT_HDL_GAP_DEVICE_NAME_INIT[] =
{
    0x00, (_GAP_HDL_OFFSET_BASE_ + 2),                                     //0x00 0x03                   //Handle
    GATT_SPEC_CHARACTERISTIC, GATT_SPEC_CHARACTERISTIC_DEVICE_NAME,        //0x2A 0x00                    //Attribute Type (0x2A, 0x00)
#ifdef _PROFILE_HOGP_
    SIZE_OF_ATTRIBUTE_VALUE_7,
    'R', 'F', '_',
    'H', 'O', 'G', 'P',                                                                        //Attribute Value, "RF_HOGP"
#else
    SIZE_OF_ATTRIBUTE_VALUE_8,                                             //0x08
    'B', 'L', 'E', '_',
    'D', 'E', 'M', 'O',                                                                        //Attribute Value, "BLE_DEMO"
#endif
};


Uint8 att_HDL_GAP_DEVICE_NAME[] =
{
    //0x00, (_GAP_HDL_OFFSET_BASE_+2),                                                         //Handle
    //GATT_SPEC_CHARACTERISTIC, GATT_SPEC_CHARACTERISTIC_DEVICE_NAME,                          //Attribute Type
    //SIZE_OF_ATTRIBUTE_VALUE_8,
#ifdef _PROFILE_HOGP_
    'R', 'F', '_',                                                                             //Attribute Value, "RF_HOGP"
    'H', 'O', 'G', 'P',
#else
    'B', 'L', 'E', '_',
    'D', 'E', 'M', 'O',                                                                        //Attribute Value, "BLE_DEMO"
#endif
};


const Uint8 ATT_HDL_GAP_CHARACTERISTIC_APPEARANCE[] =                                          //Mandatory
{
    0x00, (_GAP_HDL_OFFSET_BASE_ + 3),                                      //0x00 0x04                   //Handle
    GATT_DECLARATIONS, GATT_DECLARATIONS_CHARACTERISTIC,                    //0x28 0x03                  //Attribute Type (0x28, 0x03)
    SIZE_OF_ATTRIBUTE_VALUE_5,                                              //0x05
    (
        //Characteristic Properties, Read: Mandatory
        //GATT_DECLARATIONS_PROPERTIES_BROADCAST |
        GATT_DECLARATIONS_PROPERTIES_READ |                                 //0x02
        //GATT_DECLARATIONS_PROPERTIES_WRITE_WITHOUT_RESPONSE |
        //GATT_DECLARATIONS_PROPERTIES_WRITE |
        //GATT_DECLARATIONS_PROPERTIES_NOTIFY |
        //GATT_DECLARATIONS_PROPERTIES_INDICATE |
        //GATT_DECLARATIONS_PROPERTIES_AUTHENTICATED_SIGNED_WRITES |
        //GATT_DECLARATIONS_PROPERTIES_EXTENDED_PROPERTIES |
        0x00                                                                //0x00
    ),
    (_GAP_HDL_OFFSET_BASE_ + 4), 0x00,                                      //0x05 0x00
    GATT_SPEC_CHARACTERISTIC_APPEARANCE, GATT_SPEC_CHARACTERISTIC           //0x01 0x2A                       //Attribute Value (0x01, 0x2A)
};


const Uint8 ATT_HDL_GAP_APPEARANCE[] =
{
    0x00, (_GAP_HDL_OFFSET_BASE_ + 4),                                      //0x00 0x05                   //Handle
    GATT_SPEC_CHARACTERISTIC, GATT_SPEC_CHARACTERISTIC_APPEARANCE,          //0x2A 0x01                    //Attribute Type (0x2A, 0x01)
    SIZE_OF_ATTRIBUTE_VALUE_2,                                              //0x02
    //0x11, 0x00                                                                               //Attribute Value, according to Test Attribute Server D09r17
#if defined _PROFILE_HOGP_
#if defined _PROFILE_HOGP_MULTI_
    0xC0, 0x03,     //0x03C0: 960 -> Human Interface Device (HID), HID Generic
    //0xC1, 0x03,     //0x03C1: 961 -> Keyboard
#elif defined _PROFILE_HOGP_MOUSE_
    0xC2, 0x03,     //0x03C2: 962 -> Mouse
#elif defined _PROFILE_HOGP_KEYBOARD_
    0xC1, 0x03,     //0x03C1: 961 -> Keyboard
#elif defined _PROFILE_HOGP_COMSUMER_
    0xC0, 0x03,     //0x03C0: 960 -> Human Interface Device (HID), HID Generic
#endif
#elif defined _PROFILE_BLP_
    //0x80, 0x03,   //0x0380: 896 -> Generic Blood Pressure
    0x81, 0x03,     //0x0381: 897 -> Blood Pressure: Arm
    //0x82, 0x03,   //0x0382: 898 -> Blood Pressure: Wrist
#elif defined _PROFILE_HTP_
    //0x00, 0x03,   //0x0300: 768 -> Generic Thermometer
    0x01, 0x03,     //0x0301: 769 -> Thermometer: Ear
#elif defined _PROFILE_RSCP_
    0x40, 0x04,     //0x0440: 1088 -> Running Walking Sensor
    //0x41, 0x04,   //0x0441: 1089 -> Running Walking Sensor: In-Shoe
    //0x42, 0x04,   //0x0442: 1090 -> Running Walking Sensor: On-Shoe
    //0x43, 0x04,   //0x0443: 1091 -> Running Walking Sensor: On-Hip
#elif defined _PROFILE_CSCP_
    //0x80, 0x04,   //0x0480: 1152 -> Generic: Cycling
    //0x81, 0x04,   //0x0481: 1153 -> Cycling: Cycling Computer
    //0x82, 0x04,   //0x0482: 1154 -> Cycling: Speed Sensor
    //0x83, 0x04,   //0x0483: 1155 -> Cycling: Cadence Sensor
    //0x84, 0x04,   //0x0484: 1156 -> Cycling: Power Sensor
    0x85, 0x04,     //0x0485: 1157 -> Cycling: Speed and Cadence Sensor #endif
#elif defined _PROFILE_HRP_
    0x40, 0x03,     //0x0340: 832 -> Generic Heart rate Sensor
    //0x41, 0x03,   //0x0341: 833 -> Heart Rate Sensor: Heart Rate Belt
#else
    0x00, 0x00      //None                                                      //0x00 0x00
#endif
};
//https://developer.bluetooth.org/gatt/characteristics/Pages/CharacteristicViewer.aspx?u=org.bluetooth.characteristic.gap.appearance.xml


const Uint8 ATT_HDL_GAP_CHARACTERISTIC_RECONNECTION_ADDRESS[] =                                //Optional, if privacy feature and Peripheral privacy Flag characteristic are supported, else excluded
{
    0x00, (_GAP_HDL_OFFSET_BASE_ + 5),                                          //0x00 0x06                //Handle
    GATT_DECLARATIONS, GATT_DECLARATIONS_CHARACTERISTIC,                        //0x28 0x03                 //Attribute Type
    SIZE_OF_ATTRIBUTE_VALUE_5,                                                  //0x05
    (
        //Characteristic Properties
        //GATT_DECLARATIONS_PROPERTIES_BROADCAST |
        //GATT_DECLARATIONS_PROPERTIES_READ |
        //GATT_DECLARATIONS_PROPERTIES_WRITE_WITHOUT_RESPONSE |
        GATT_DECLARATIONS_PROPERTIES_WRITE |                                    //0x08
        //GATT_DECLARATIONS_PROPERTIES_NOTIFY |
        //GATT_DECLARATIONS_PROPERTIES_INDICATE |
        //GATT_DECLARATIONS_PROPERTIES_AUTHENTICATED_SIGNED_WRITES |
        //GATT_DECLARATIONS_PROPERTIES_EXTENDED_PROPERTIES |
        0x00                                                                    //0x00
    ),
    (_GAP_HDL_OFFSET_BASE_ + 6), 0x00,                                          //0x07 0x00
    GATT_SPEC_CHARACTERISTIC_RECONNECTION_ADDRESS, GATT_SPEC_CHARACTERISTIC     //0x03 0x2A                 //Attribute Value
};


const Uint8 ATT_HDL_GAP_RECONNECTION_ADDRESS_INIT[] =
{
    0x00, (_GAP_HDL_OFFSET_BASE_ + 6),                                              //0x00 0x07            //Handle
    GATT_SPEC_CHARACTERISTIC, GATT_SPEC_CHARACTERISTIC_RECONNECTION_ADDRESS,        //0x2A 0x03            //Attribute Type
    SIZE_OF_ATTRIBUTE_VALUE_6,                                                      //0x06
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00                                              //0x00 0x00 0x00 0x00 0x00 0x00           //Attribute Value, according to Test Attribute Server D09r17
};


Uint8 att_HDL_GAP_RECONNECTION_ADDRESS[] =
{
    //0x00, (_GAP_HDL_OFFSET_BASE_+6),                                                        //Handle
    //GATT_SPEC_CHARACTERISTIC, GATT_SPEC_CHARACTERISTIC_RECONNECTION_ADDRESS,                //Attribute Type
    //SIZE_OF_ATTRIBUTE_VALUE_6,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00                                                        //Attribute Value, according to Test Attribute Server D09r17
};


const Uint8 ATT_HDL_GAP_CHARACTERISTIC_PERIPHERAL_PRIVACY_FLAG[] =                             //Mandatory if privacy feature is supported, else excluded
{
    0x00, (_GAP_HDL_OFFSET_BASE_ + 7),                                          //0x00 0x08               //Handle
    GATT_DECLARATIONS, GATT_DECLARATIONS_CHARACTERISTIC,                        //0x28 0x03               //Attribute Type
    SIZE_OF_ATTRIBUTE_VALUE_5,
    (
        //Characteristic Properties
        //GATT_DECLARATIONS_PROPERTIES_BROADCAST |
        GATT_DECLARATIONS_PROPERTIES_READ |                                     //0x02
        //GATT_DECLARATIONS_PROPERTIES_WRITE_WITHOUT_RESPONSE |
        //GATT_DECLARATIONS_PROPERTIES_WRITE |
        //GATT_DECLARATIONS_PROPERTIES_NOTIFY |
        //GATT_DECLARATIONS_PROPERTIES_INDICATE |
        //GATT_DECLARATIONS_PROPERTIES_AUTHENTICATED_SIGNED_WRITES |
        //GATT_DECLARATIONS_PROPERTIES_EXTENDED_PROPERTIES |
        0x00                                                                    //0x00
    ),
    (_GAP_HDL_OFFSET_BASE_ + 8), 0x00,                                          //0x09 0x00
    GATT_SPEC_CHARACTERISTIC_PERIPHERAL_PRIVACY_FLAG, GATT_SPEC_CHARACTERISTIC  //0x02 0x2A              //Attribute Value
};


const Uint8 ATT_HDL_GAP_PERIPHERAL_PRIVACY_FLAG[] =
{
    0x00, (_GAP_HDL_OFFSET_BASE_ + 8),                                          //0x00  0x09                                         //Handle
    GATT_SPEC_CHARACTERISTIC, GATT_SPEC_CHARACTERISTIC_PERIPHERAL_PRIVACY_FLAG, //0x2A  0x02             //Attribute Type
    SIZE_OF_ATTRIBUTE_VALUE_1,                                                  //0x01
    0x00                                                                        //0x00               //Attribute Value, according to Test Attribute Server D09r17
};


const Uint8 ATT_HDL_GAP_CHARACTERISTIC_PERIPHERAL_PREFERRED_CONNECTION_PARAMETERS[] =          //Optional
{
    0x00, (_GAP_HDL_OFFSET_BASE_ + 9),                                         //0x00  0x0A              //Handle
    GATT_DECLARATIONS, GATT_DECLARATIONS_CHARACTERISTIC,                       //0x28  0x03              //Attribute Type
    SIZE_OF_ATTRIBUTE_VALUE_5,                                                 //0x05
    //0x02,
    (
        //Characteristic Properties, Read: Mandatory
        //GATT_DECLARATIONS_PROPERTIES_BROADCAST |
        GATT_DECLARATIONS_PROPERTIES_READ |                                     //0x02
        //GATT_DECLARATIONS_PROPERTIES_WRITE_WITHOUT_RESPONSE |
        //GATT_DECLARATIONS_PROPERTIES_WRITE |
        //GATT_DECLARATIONS_PROPERTIES_NOTIFY |
        //GATT_DECLARATIONS_PROPERTIES_INDICATE |
        //GATT_DECLARATIONS_PROPERTIES_AUTHENTICATED_SIGNED_WRITES |
        //GATT_DECLARATIONS_PROPERTIES_EXTENDED_PROPERTIES |
        0x00                                                                    //0x00
    ),
    (_GAP_HDL_OFFSET_BASE_ + 10), 0x00,                                         //0x0B 0x00
    GATT_SPEC_CHARACTERISTIC_PERIPHERAL_PREFERRED_CONNECTION_PARAMETERS, GATT_SPEC_CHARACTERISTIC      //0x04 0x2A          //Attribute Value
};


const Uint8 ATT_HDL_GAP_PERIPHERAL_PREFERRED_CONNECTION_PARAMETERS[] =
{
    0x00, (_GAP_HDL_OFFSET_BASE_ + 10),                                         //0x00 0x0B                    //Handle
    GATT_SPEC_CHARACTERISTIC, GATT_SPEC_CHARACTERISTIC_PERIPHERAL_PREFERRED_CONNECTION_PARAMETERS,     //0x04 0x2A                 //Attribute Type
    SIZE_OF_ATTRIBUTE_VALUE_8,                                                  //0x08
    0x06, 0x00,                                                                 //0x06, 0x00,               //Attribute Value, Minimum Connection Interval: 7.500 ms
    0x08, 0x00,                                                                 //0x08, 0x00,               //Attribute Value, Maximum Connection Interval: 10.00 ms
    0x00, 0x00,                                                                 //0x00, 0x00,               //Attribute Value, Slave Latency: 0
    0x58, 0x02,                                                                 //0x58, 0x02,                //Attribute Value, Timeout Multiplier: 600
};


const Uint8 ATT_HDL_GATT_PRIMARY_SERVICE[] =                                                        //GATT service: Mandatory
{
    0x00, _GATT_HDL_OFFSET_BASE_,                                               //0x00 0x0C                    //Handle
    GATT_DECLARATIONS, GATT_DECLARATIONS_PRIMARY_SERVICE,                       //0x28 0x00                   //Attribute Type
    SIZE_OF_ATTRIBUTE_VALUE_2,                                                  //0x02
    GATT_SPEC_SERVICES_GENERIC_ATTRIBUTE, GATT_SPEC_SERVICES                    //0x01 0x18                    //Attribute Value
};


const Uint8 ATT_HDL_GATT_CHARACTERISTIC_SERVICE_CHANGED[] =                                         //Optional
{
    0x00, (_GATT_HDL_OFFSET_BASE_ + 1),                                         //0x00  0x0c                    //Handle
    GATT_DECLARATIONS, GATT_DECLARATIONS_CHARACTERISTIC,                        //0x28  0x03                  //Attribute Type
    SIZE_OF_ATTRIBUTE_VALUE_5,                                                  //0x05
    //0x02,
    (
        //Characteristic Properties
        //GATT_DECLARATIONS_PROPERTIES_BROADCAST |
        GATT_DECLARATIONS_PROPERTIES_READ |                                    //0x02                     //Read: Mandatory
        //GATT_DECLARATIONS_PROPERTIES_WRITE_WITHOUT_RESPONSE |
        //GATT_DECLARATIONS_PROPERTIES_WRITE |
        //GATT_DECLARATIONS_PROPERTIES_NOTIFY |
        GATT_DECLARATIONS_PROPERTIES_INDICATE |                                //0x08                     //Indicate: Mandatory
        //GATT_DECLARATIONS_PROPERTIES_AUTHENTICATED_SIGNED_WRITES |
        //GATT_DECLARATIONS_PROPERTIES_EXTENDED_PROPERTIES |
        0x00                                                                   //0x00
    ),
    (_GATT_HDL_OFFSET_BASE_ + 2), 0x00,                                        //0x0e 0x00
    GATT_SPEC_CHARACTERISTIC_SERVICE_CHANGED, GATT_SPEC_CHARACTERISTIC         //0x05 0x2A             //Attribute Value
};


const Uint8 ATT_HDL_GATT_SERVICE_CHANGED_INIT[] =
{
    0x00, (_GATT_HDL_OFFSET_BASE_ + 2),                                       //0x00  0x0e                     //Handle
    GATT_SPEC_CHARACTERISTIC, GATT_SPEC_CHARACTERISTIC_SERVICE_CHANGED,       //0x2A 0x05                          //Attribute Type
    SIZE_OF_ATTRIBUTE_VALUE_4,                                                //0x04
    0x01, 0x00,                                                               //0x01, 0x00,                  //Start of Affected Attribute Handle Range
    0xFF, 0xFF,                                                               //0xFF, 0xFF,                  //End of Affected Attribute Handle Range
};      //Bluetooth Spec. Ver4.0 [Vol 3] page 529 of 656


const Uint8 ATT_HDL_GATT_CLIENT_CHARACTERISTIC_CONFIGURATION_INIT[] =
{
    0x00, (_GATT_HDL_OFFSET_BASE_ + 3),                                        //0x00 0x0f                     //Handle
    GATT_DESCRIPTORS, GATT_DESCRIPTORS_CLIENT_CHARACTERISTIC_CONFIGURATION,    //0x29 0x02                     //Attribute Type
    SIZE_OF_ATTRIBUTE_VALUE_2,                                                  //0x02
    0x00,                                                                       //0x00                    //Attribute Value
    0x00                                                                        //0x00                    //Attribute Value,
};


Uint8 att_HDL_GATT_SERVICE_CHANGED[] =
{
    //0x00, (_GATT_HDL_OFFSET_BASE_+2),                                                             //Handle
    //GATT_SPEC_CHARACTERISTIC, GATT_SPEC_CHARACTERISTIC_SERVICE_CHANGED,  //Attribute Type
    //SIZE_OF_ATTRIBUTE_VALUE_4,
    0x00, 0x00,                                                                 //0x00 0x00                   //Start of Affected Attribute Handle Range
    0x00, 0x00,                                                                 //0x00 0x00                    //End of Affected Attribute Handle Range
};      //Bluetooth Spec. Ver4.0 [Vol 3] page 529 of 656


Uint8 att_HDL_GATT_CLIENT_CHARACTERISTIC_CONFIGURATION[] =
{
    //0x00, (_GATT_HDL_OFFSET_BASE_+3),                                                             //Handle
    //GATT_DESCRIPTORS, GATT_DESCRIPTORS_CLIENT_CHARACTERISTIC_CONFIGURATION,                       //Attribute Type
    //SIZE_OF_ATTRIBUTE_VALUE_2,
    (
        //GATT_DESCRIPTORS_CLIENT_CHARACTERISTIC_CONFIGURATION_NOTIFICATION |
        //GATT_DESCRIPTORS_CLIENT_CHARACTERISTIC_CONFIGURATION_INDICATION |
        0x00                                                                    //0x00
    ),                                                                                              //Attribute Value, R/W: Mandatory
    0x00                                                                        //0x00                    //Attribute Value,
};


const Uint8 ATT_HDL_DIS_PRIMARY_SERVICE[] =                                                         //Device Information Service
{
    0x00, _DIS_HDL_OFFSET_BASE_,                                              //0x00  0x10                     //Handle
    GATT_DECLARATIONS, GATT_DECLARATIONS_PRIMARY_SERVICE,                     //0x28  0x00                      //Attribute Type
    SIZE_OF_ATTRIBUTE_VALUE_2,                                                 //0x02
    GATT_SPEC_SERVICES_DEVICE_INFORMATION, GATT_SPEC_SERVICES                 //0x0A  0x18                    //Attribute Value
};


const Uint8 ATT_HDL_DIS_CHARACTERISTIC_SERIAL_NUMBER_STRING[] =                                     //Optional
{
    0x00, (_DIS_HDL_OFFSET_BASE_ + 1),                                         //0x00 0x11                     //Handle
    GATT_DECLARATIONS, GATT_DECLARATIONS_CHARACTERISTIC,                       //0x28 0x03                    //Attribute Type
    SIZE_OF_ATTRIBUTE_VALUE_5,                                                  //0x05
    //0x02,
    (
        //Characteristic Properties, Read: Mandatory
        //GATT_DECLARATIONS_PROPERTIES_BROADCAST |
        GATT_DECLARATIONS_PROPERTIES_READ |                                     //0x02
        //GATT_DECLARATIONS_PROPERTIES_WRITE_WITHOUT_RESPONSE |
        //GATT_DECLARATIONS_PROPERTIES_WRITE |
        //GATT_DECLARATIONS_PROPERTIES_NOTIFY |
        //GATT_DECLARATIONS_PROPERTIES_INDICATE |
        //GATT_DECLARATIONS_PROPERTIES_AUTHENTICATED_SIGNED_WRITES |
        //GATT_DECLARATIONS_PROPERTIES_EXTENDED_PROPERTIES |
        0x00                                                                    //0x00
    ),
    (_DIS_HDL_OFFSET_BASE_ + 2), 0x00,                                          //0x12 0x00
    GATT_SPEC_CHARACTERISTIC_SERIAL_NUMBER_STRING, GATT_SPEC_CHARACTERISTIC     //0x25 0x2A                    //Attribute Value
};


const Uint8 ATT_HDL_DIS_SERIAL_NUMBER_STRING[] =
{
    0x00, (_DIS_HDL_OFFSET_BASE_ + 2),                                         //0x00   0x12                  //Handle
    GATT_SPEC_CHARACTERISTIC, GATT_SPEC_CHARACTERISTIC_SERIAL_NUMBER_STRING,   //0x25   0x2A                  //Attribute Type
    SIZE_OF_ATTRIBUTE_VALUE_4,                                                  //0x04
    0x31, 0x35, 0x38, 0x37                                                       //0x31, 0x35, 0x38, 0x37                   //Attribute Value, "1587"
};


const Uint8 ATT_HDL_DIS_SERIAL_NUMBER_STRING_PRESENTATION_FORMAT[] =
{
    0x00, (_DIS_HDL_OFFSET_BASE_ + 3),                                         //0x00   0x13                   //Handle
    GATT_DESCRIPTORS, GATT_DESCRIPTORS_CHARACTERISTIC_PRESENTATION_FORMAT,     //0x29    0x04                   //Attribute Type
    SIZE_OF_ATTRIBUTE_VALUE_7,                                                  //0x07
    GATT_CHARACTERISTIC_FORMAT_UTF8S,                                           //0x19                    //Attribute Value, Format: UTF-8 string
    0x00,                                                                        //0x00                   //Attribute Value, Exponent: 0
    0x00, 0x00,                                                                  //0x00 0x00                   //Attribute Value, Unit: [0x0000] **The Unit is a UUID**
    GATT_CHARACTERISTIC_BLUETOOTH_NAMESPACE_BLUETOOTH_SIG,                      //0x01                    //Attribute Value, Name Space: 0x01
    0x00, 0x00                                                                  //0x00 0x00                    //Attribute Value, Description: 0x0000
};


const Uint8 ATT_HDL_DIS_CHARACTERISTIC_MANUFACTURER_NAME_STRING[] =                                 //Optional
{
    0x00, (_DIS_HDL_OFFSET_BASE_ + 4),                                          //0x00  0x14                    //Handle
    GATT_DECLARATIONS, GATT_DECLARATIONS_CHARACTERISTIC,                        //0x28  0x03                  //Attribute Type
    SIZE_OF_ATTRIBUTE_VALUE_5,                                                  //0x05
    //0x02,
    (
        //Characteristic Properties, Read: Mandatory
        //GATT_DECLARATIONS_PROPERTIES_BROADCAST |
        GATT_DECLARATIONS_PROPERTIES_READ |                                     //0x02
        //GATT_DECLARATIONS_PROPERTIES_WRITE_WITHOUT_RESPONSE |
        //GATT_DECLARATIONS_PROPERTIES_WRITE |
        //GATT_DECLARATIONS_PROPERTIES_NOTIFY |
        //GATT_DECLARATIONS_PROPERTIES_INDICATE |
        //GATT_DECLARATIONS_PROPERTIES_AUTHENTICATED_SIGNED_WRITES |
        //GATT_DECLARATIONS_PROPERTIES_EXTENDED_PROPERTIES |
        0x00                                                                    //0x00
    ),
    (_DIS_HDL_OFFSET_BASE_ + 5), 0x00,                                          //0x15 0x00
    GATT_SPEC_CHARACTERISTIC_MANUFACTURER_NAME_STRING, GATT_SPEC_CHARACTERISTIC  //0x29 0x2A                   //Attribute Value
};


const Uint8 ATT_HDL_DIS_MANUFACTURER_NAME_STRING[] =
{
    0x00, (_DIS_HDL_OFFSET_BASE_ + 5),                                            //0x00   0x15                //Handle
    GATT_SPEC_CHARACTERISTIC, GATT_SPEC_CHARACTERISTIC_MANUFACTURER_NAME_STRING,  //0x2A 0x29                  //Attribute Type
    SIZE_OF_ATTRIBUTE_VALUE_6,                                                    //0x06
    'R', 'F', '_', 'B', 'L', 'E'                                                                    //Attribute Value, "RF_BLE"
};


const Uint8 ATT_HDL_DIS_MANUFACTURER_NAME_STRING_PRESENTATION_FORMAT[] =
{
    0x00, (_DIS_HDL_OFFSET_BASE_ + 6),                                              //0x00 0x16                 //Handle
    GATT_DESCRIPTORS, GATT_DESCRIPTORS_CHARACTERISTIC_PRESENTATION_FORMAT,    //0x29   0x04                    //Attribute Type
    SIZE_OF_ATTRIBUTE_VALUE_7,                                                //0x07
    GATT_CHARACTERISTIC_FORMAT_UTF8S,                                         // 0x19                  //Attribute Value, Format: UTF-8 string
    0x00,                                                                     // 0x00                  //Attribute Value, Exponent: 0
    0x00, 0x00,                                                               // 0x00 0x00                  //Attribute Value, Unit: [0x0000] **The Unit is a UUID**
    GATT_CHARACTERISTIC_BLUETOOTH_NAMESPACE_BLUETOOTH_SIG,                    // 0x01                  //Attribute Value, Name Space: 0x01
    0x00, 0x00                                                                //0x00 0x00                   //Attribute Value, Description: 0x0000
};                                                                            //
//
//
const Uint8 ATT_HDL_DIS_CHARACTERISTIC_SYSTEM_ID[] =                          //                   //Optonal
{
    //
    0x00, (_DIS_HDL_OFFSET_BASE_ + 7),                                        //0x00 0x17                   //Handle
    GATT_DECLARATIONS, GATT_DECLARATIONS_CHARACTERISTIC,                      //0x28  0x03                  //Attribute Type
    SIZE_OF_ATTRIBUTE_VALUE_5,                                                //0x05
    //0x02,                                                                   //
    (                                                                         //
        //Characteristic Properties, Read: Mandatory                          //
        //GATT_DECLARATIONS_PROPERTIES_BROADCAST |                            //
        GATT_DECLARATIONS_PROPERTIES_READ |                                   //0x02
        //GATT_DECLARATIONS_PROPERTIES_WRITE_WITHOUT_RESPONSE |               //
        //GATT_DECLARATIONS_PROPERTIES_WRITE |                                //
        //GATT_DECLARATIONS_PROPERTIES_NOTIFY |                               //
        //GATT_DECLARATIONS_PROPERTIES_INDICATE |                             //
        //GATT_DECLARATIONS_PROPERTIES_AUTHENTICATED_SIGNED_WRITES |          //
        //GATT_DECLARATIONS_PROPERTIES_EXTENDED_PROPERTIES |                  //
        0x00                                                                  //0x00
    ),                                                                        //
    (_DIS_HDL_OFFSET_BASE_ + 8), 0x00,                                        //0x18 0x00
    GATT_SPEC_CHARACTERISTIC_SYSTEM_ID, GATT_SPEC_CHARACTERISTIC              // 0x23 0x2A                  //Attribute Value
};                                                                            //
//
//
const Uint8 ATT_HDL_DIS_SYSTEM_ID[] =                                         //
{
    //
    0x00, (_DIS_HDL_OFFSET_BASE_ + 8),                                        //0x00 0x18                   //Handle
    GATT_SPEC_CHARACTERISTIC, GATT_SPEC_CHARACTERISTIC_SYSTEM_ID,             //0x2A 0x23                   //Attribute Type
    SIZE_OF_ATTRIBUTE_VALUE_8,                                                //0x08
    0x55, 0xAA, 0x55, 0xAA, 0x55,                                             //0x55 0xAA 0x55 0xAA 0X55                   //Attribute Value, Manufacturer
    0xAA, 0x55, 0xAA                                                          //0XAA 0X55 0XAA                   //Attribute Value, Organizationally Unique ID
};                                                                            //
//
//
const Uint8 ATT_HDL_DIS_CHARACTERISTIC_FIRMWARE_REVISION_STRING[] =           //                   //Optional
{
    //
    0x00, (_DIS_HDL_OFFSET_BASE_ + 9),                                        //0X00 0X19                   //Handle
    GATT_DECLARATIONS, GATT_DECLARATIONS_CHARACTERISTIC,                      //0X28  0X03                 //Attribute Type
    SIZE_OF_ATTRIBUTE_VALUE_5,                                                //0X05
    //0x02,                                                                   //
    (                                                                         //
        //Characteristic Properties, Read: Mandatory                          //
        //GATT_DECLARATIONS_PROPERTIES_BROADCAST |                            //
        GATT_DECLARATIONS_PROPERTIES_READ |                                   //0X02
        //GATT_DECLARATIONS_PROPERTIES_WRITE_WITHOUT_RESPONSE |               //
        //GATT_DECLARATIONS_PROPERTIES_WRITE |                                //
        //GATT_DECLARATIONS_PROPERTIES_NOTIFY |                               //
        //GATT_DECLARATIONS_PROPERTIES_INDICATE |                             //
        //GATT_DECLARATIONS_PROPERTIES_AUTHENTICATED_SIGNED_WRITES |          //
        //GATT_DECLARATIONS_PROPERTIES_EXTENDED_PROPERTIES |                         //0X00
        0x00                                                                         //
    ),                                                                               //
    (_DIS_HDL_OFFSET_BASE_ + 10), 0x00,                                              //0X1A 0X00
    GATT_SPEC_CHARACTERISTIC_FIRMWARE_REVISION_STRING, GATT_SPEC_CHARACTERISTIC      //0X26  0X2A              //Attribute Value
};                                                                                   //
//
//
const Uint8 ATT_HDL_DIS_FIRMWARE_REVISION_STRING[] =                                 //
{
    //
    0x00, (_DIS_HDL_OFFSET_BASE_ + 10),                                              //0X00 0X1A             //Handle
    GATT_SPEC_CHARACTERISTIC, GATT_SPEC_CHARACTERISTIC_FIRMWARE_REVISION_STRING,     //0X2A 0X26              //Attribute Type
    SIZE_OF_ATTRIBUTE_VALUE_4,                                                       //0X04
    0x30, 0x31, 0x2E, 0x31                                                           //0X30 0X31 0X2E 0X31              //Attribute Value, "01.1"
};                                                                                   //
//
//
const Uint8 ATT_HDL_DIS_FIRMWARE_REVISION_STRING_PRESENTATION_FORMAT[] =             //
{
    //
    0x00, (_DIS_HDL_OFFSET_BASE_ + 11),                                              //0X00 0X1B              //Handle
    GATT_DESCRIPTORS, GATT_DESCRIPTORS_CHARACTERISTIC_PRESENTATION_FORMAT,           //0X29 0X04             //Attribute Type
    SIZE_OF_ATTRIBUTE_VALUE_7,                                                       //0X07
    GATT_CHARACTERISTIC_FORMAT_UTF8S,                                                //0X19              //Attribute Value, Format: UTF-8 string
    0x00,                                                                            //0X00              //Attribute Value, Exponent: 0
    0x00, 0x00,                                                                      //0X00 0X00              //Attribute Value, Unit: [0x0000] **The Unit is a UUID**
    GATT_CHARACTERISTIC_BLUETOOTH_NAMESPACE_BLUETOOTH_SIG,                           //0X01              //Attribute Value, Name Space: 0x01
    0x00, 0x00                                                                       //0X00 0X00              //Attribute Value, Description: 0x0000
};                                                                                   //
//
//
const Uint8 ATT_HDL_DIS_CHARACTERISTIC_MODEL_NUMBER_STRING[] =                       //              //Optional
{
    //
    0x00, (_DIS_HDL_OFFSET_BASE_ + 12),                                              //0X00 0X1C              //Handle
    GATT_DECLARATIONS, GATT_DECLARATIONS_CHARACTERISTIC,                             //0X28 0X03              //Attribute Type
    SIZE_OF_ATTRIBUTE_VALUE_5,                                                       //0X05
    //0x02,                                                                          //
    (                                                                                //
        //Characteristic Properties, Read: Mandatory                                 //
        //GATT_DECLARATIONS_PROPERTIES_BROADCAST |                                   //
        GATT_DECLARATIONS_PROPERTIES_READ |                                          //0X02
        //GATT_DECLARATIONS_PROPERTIES_WRITE_WITHOUT_RESPONSE |                      //
        //GATT_DECLARATIONS_PROPERTIES_WRITE |                                       //
        //GATT_DECLARATIONS_PROPERTIES_NOTIFY |                                      //
        //GATT_DECLARATIONS_PROPERTIES_INDICATE |                                    //
        //GATT_DECLARATIONS_PROPERTIES_AUTHENTICATED_SIGNED_WRITES |                 //
        //GATT_DECLARATIONS_PROPERTIES_EXTENDED_PROPERTIES |                         //
        0x00                                                                         //0X00
    ),                                                                               //
    (_DIS_HDL_OFFSET_BASE_ + 13), 0x00,                                              //0X1D 0X00
    GATT_SPEC_CHARACTERISTIC_MODEL_NUMBER_STRING, GATT_SPEC_CHARACTERISTIC           //0X24 0X2A             //Attribute Value
};                                                                                   //
//
//
const Uint8 ATT_HDL_DIS_MODEL_NUMBER_STRING[] =                                      //
{
    //
    0x00, (_DIS_HDL_OFFSET_BASE_ + 13),                                              //0X00 0X1D              //Handle
    GATT_SPEC_CHARACTERISTIC, GATT_SPEC_CHARACTERISTIC_MODEL_NUMBER_STRING,          // 0X2A 0X24             //Attribute Type
    SIZE_OF_ATTRIBUTE_VALUE_6,                                                       //0X06
    'R', 'F', '_', 'B', 'L', 'E'                                                     //              //Attribute Value, "RF_BLE"
};                                                                                   //
//
//
const Uint8 ATT_HDL_DIS_MODEL_NUMBER_STRING_PRESENTATION_FORMAT[] =                  //
{
    //
    0x00, (_DIS_HDL_OFFSET_BASE_ + 14),                                              //0X00 0X1E              //Handle
    GATT_DESCRIPTORS, GATT_DESCRIPTORS_CHARACTERISTIC_PRESENTATION_FORMAT,           // 0X29 0X04             //Attribute Type
    SIZE_OF_ATTRIBUTE_VALUE_7,                                                       //0X07
    GATT_CHARACTERISTIC_FORMAT_UTF8S,                                                // 0X19             //Attribute Value, Format: UTF-8 string
    0x00,                                                                            // 0X00             //Attribute Value, Exponent: 0
    0x00, 0x00,                                                                      //0X00 0X00              //Attribute Value, Unit: [0x0000] **The Unit is a UUID**
    GATT_CHARACTERISTIC_BLUETOOTH_NAMESPACE_BLUETOOTH_SIG,                           // 0X01             //Attribute Value, Name Space: 0x01
    0x00, 0x00                                                                       // 0X00 0X00             //Attribute Value, Description: 0x0000
};                                                                                   //
//
//
const Uint8 ATT_HDL_DIS_CHARACTERISTIC_HARDWARE_REVISION_STRING[] =                  //              //Optional
{
    //
    0x00, (_DIS_HDL_OFFSET_BASE_ + 15),                                              // 0X00 0X1F             //Handle
    GATT_DECLARATIONS, GATT_DECLARATIONS_CHARACTERISTIC,                             //  0X28 0X03            //Attribute Type
    SIZE_OF_ATTRIBUTE_VALUE_5,                                                       //0X05
    //0x02,                                                                          //
    (                                                                                //
        //Characteristic Properties, Read: Mandatory                                 //
        //GATT_DECLARATIONS_PROPERTIES_BROADCAST |                                   //
        GATT_DECLARATIONS_PROPERTIES_READ |                                          //0X02
        //GATT_DECLARATIONS_PROPERTIES_WRITE_WITHOUT_RESPONSE |                      //
        //GATT_DECLARATIONS_PROPERTIES_WRITE |                                       //
        //GATT_DECLARATIONS_PROPERTIES_NOTIFY |                                      //
        //GATT_DECLARATIONS_PROPERTIES_INDICATE |                                    //
        //GATT_DECLARATIONS_PROPERTIES_AUTHENTICATED_SIGNED_WRITES |                 //
        //GATT_DECLARATIONS_PROPERTIES_EXTENDED_PROPERTIES |                         //
        0x00                                                                         //0X00
    ),                                                                               //
    (_DIS_HDL_OFFSET_BASE_ + 16), 0x00,                                              //0X20 0X00
    GATT_SPEC_CHARACTERISTIC_HARDWARE_REVISION_STRING, GATT_SPEC_CHARACTERISTIC      //0X27 0X2A              //Attribute Value
};                                                                                   //
//
//
const Uint8 ATT_HDL_DIS_HARDWARE_REVISION_STRING[] =                                 //
{
    //
    0x00, (_DIS_HDL_OFFSET_BASE_ + 16),                                              //0X00 0X20              //Handle
    GATT_SPEC_CHARACTERISTIC, GATT_SPEC_CHARACTERISTIC_HARDWARE_REVISION_STRING,     //0X2A 0X27              //Attribute Type
    SIZE_OF_ATTRIBUTE_VALUE_3,                                                       //0X03
    0x55, 0x30, 0x31                                                                 //0X55 0X30 0X31              //Attribute Value, "U01"
};                                                                                   //
//
//
const Uint8 ATT_HDL_DIS_HARDWARE_REVISION_STRING_PRESENTATION_FORMAT[] =             //
{
    //
    0x00, (_DIS_HDL_OFFSET_BASE_ + 17),                                              //0X00 0X21              //Handle
    GATT_DESCRIPTORS, GATT_DESCRIPTORS_CHARACTERISTIC_PRESENTATION_FORMAT,           //0X29  0x04             //Attribute Type
    SIZE_OF_ATTRIBUTE_VALUE_7,                                                       //0x07
    GATT_CHARACTERISTIC_FORMAT_UTF8S,                                                //0x19              //Attribute Value, Format: UTF-8 string
    0x00,                                                                            //0x00              //Attribute Value, Exponent: 0
    0x00, 0x00,                                                                      //0x00 0x00              //Attribute Value, Unit: [0x0000] **The Unit is a UUID**
    GATT_CHARACTERISTIC_BLUETOOTH_NAMESPACE_BLUETOOTH_SIG,                           //0x01              //Attribute Value, Name Space: 0x01
    0x00, 0x00                                                                       //0x00 0x00              //Attribute Value, Description: 0x0000
};                                                                                   //
//
//
const Uint8 ATT_HDL_DIS_CHARACTERISTIC_SOFTWARE_REVISION_STRING[] =                  //              //Optional
{
    //
    0x00, (_DIS_HDL_OFFSET_BASE_ + 18),                                              //0x00  0x22            //Handle
    GATT_DECLARATIONS, GATT_DECLARATIONS_CHARACTERISTIC,                             //0x28  0x03            //Attribute Type
    SIZE_OF_ATTRIBUTE_VALUE_5,                                                       //0x05
    //0x02,                                                                          //
    (                                                                                //
        //Characteristic Properties, Read: Mandatory                                 //
        //GATT_DECLARATIONS_PROPERTIES_BROADCAST |                                   //
        GATT_DECLARATIONS_PROPERTIES_READ |                                          //0x02
        //GATT_DECLARATIONS_PROPERTIES_WRITE_WITHOUT_RESPONSE |                      //
        //GATT_DECLARATIONS_PROPERTIES_WRITE |                                       //
        //GATT_DECLARATIONS_PROPERTIES_NOTIFY |                                      //
        //GATT_DECLARATIONS_PROPERTIES_INDICATE |                                    //
        //GATT_DECLARATIONS_PROPERTIES_AUTHENTICATED_SIGNED_WRITES |                 //
        //GATT_DECLARATIONS_PROPERTIES_EXTENDED_PROPERTIES |                         //
        0x00                                                                         //0x00
    ),                                                                               //
    (_DIS_HDL_OFFSET_BASE_ + 19), 0x00,                                              //0x23 0x00
    GATT_SPEC_CHARACTERISTIC_SOFTWARE_REVISION_STRING, GATT_SPEC_CHARACTERISTIC      //0x28 0x2A             //Attribute Value
};                                                                                   //
//
//
const Uint8 ATT_HDL_DIS_SOFTWARE_REVISION_STRING[] =                                 //
{
    //
    0x00, (_DIS_HDL_OFFSET_BASE_ + 19),                                              //0x00 0x23              //Handle
    GATT_SPEC_CHARACTERISTIC, GATT_SPEC_CHARACTERISTIC_SOFTWARE_REVISION_STRING,     //0x2A 0x28              //Attribute Type
    SIZE_OF_ATTRIBUTE_VALUE_4,                                                       //0x04
    0x30, 0x30, 0x39, 0x33                                                           //0x30, 0x30, 0x39, 0x33               //Attribute Value, "0093"
};                                                                                   //
//
//
const Uint8 ATT_HDL_DIS_SOFTWARE_REVISION_STRING_PRESENTATION_FORMAT[] =             //
{
    //
    0x00, (_DIS_HDL_OFFSET_BASE_ + 20),                                              //0x00 0x24              //Handle
    GATT_DESCRIPTORS, GATT_DESCRIPTORS_CHARACTERISTIC_PRESENTATION_FORMAT,           //0x29  0x04             //Attribute Type
    SIZE_OF_ATTRIBUTE_VALUE_7,                                                       //0x07
    GATT_CHARACTERISTIC_FORMAT_UTF8S,                                                //0x19              //Attribute Value, Format: UTF-8 string
    0x00,                                                                            //0x00              //Attribute Value, Exponent: 0
    0x00, 0x00,                                                                      //0x00 0x00              //Attribute Value, Unit: [0x0000] **The Unit is a UUID**
    GATT_CHARACTERISTIC_BLUETOOTH_NAMESPACE_BLUETOOTH_SIG,                           //0x01              //Attribute Value, Name Space: 0x01
    0x00, 0x00                                                                       //0x00 0x00              //Attribute Value, Description: 0x0000
};                                                                                   //
//
//
const Uint8 ATT_HDL_DIS_CHARACTERISTIC_PNP_ID[] =                                    //              //Optional
{
    //
    0x00, (_DIS_HDL_OFFSET_BASE_ + 21),                                              //0x00 0x25              //Handle
    GATT_DECLARATIONS, GATT_DECLARATIONS_CHARACTERISTIC,                             //0x28 0x03             //Attribute Type
    SIZE_OF_ATTRIBUTE_VALUE_5,                                                       //0x05
    //0x02,                                                                          //
    (                                                                                //
        //Characteristic Properties, Read: Mandatory                                 //
        //GATT_DECLARATIONS_PROPERTIES_BROADCAST |                                   //
        GATT_DECLARATIONS_PROPERTIES_READ |                                          //0x02
        //GATT_DECLARATIONS_PROPERTIES_WRITE_WITHOUT_RESPONSE |                      //
        //GATT_DECLARATIONS_PROPERTIES_WRITE |                                       //
        //GATT_DECLARATIONS_PROPERTIES_NOTIFY |                                      //
        //GATT_DECLARATIONS_PROPERTIES_INDICATE |                                    //
        //GATT_DECLARATIONS_PROPERTIES_AUTHENTICATED_SIGNED_WRITES |                 //
        //GATT_DECLARATIONS_PROPERTIES_EXTENDED_PROPERTIES |                         //
        0x00                                                                         //0x00
    ),                                                                               //
    (_DIS_HDL_OFFSET_BASE_ + 22), 0x00,                                              //0x26 0x00
    GATT_SPEC_CHARACTERISTIC_PNP_ID, GATT_SPEC_CHARACTERISTIC                        //0x50 0x2A             //Attribute Value
};                                                                                   //
//
//
const Uint8 ATT_HDL_DIS_PNP_ID[] =                                                   //
{
    //
    0x00, (_DIS_HDL_OFFSET_BASE_ + 22),                                              //0x00 0x26              //Handle
    GATT_SPEC_CHARACTERISTIC, GATT_SPEC_CHARACTERISTIC_PNP_ID,                       //0x2A 0x50             //Attribute Type
    SIZE_OF_ATTRIBUTE_VALUE_7,                                                       //0x07
    BLE_GATT_DIS_PNPID_VID_SOURCE_BLUETOOTH_SIG,                                     //0x01              //Vendor ID Source, identifies the source of the Vendor ID field.
    BLE_COMPANY_ID_L, BLE_COMPANY_ID_H,                                              //0x64 0x08              //Vendor ID
    0x00, 0x00,                                                                      //0x00 0x00              //Product ID: Manufacturer managed identifier for this product
    0x00, 0x00                                                                       //0x00 0x00              //Product Version: Manufacturer managed version for this product
};                                                                                   //
//
//
#ifdef _PROFILE_HRP_                                                                 //
const Uint8 ATT_HDL_HRS_PRIMARY_SERVICE[] =                                          //
{
    //
    0x00, _HRS_HDL_OFFSET_BASE_,                                                     //  ///Attribute Handle, service space, big endian
    GATT_DECLARATIONS, GATT_DECLARATIONS_PRIMARY_SERVICE,                            //  ///Attribute Type, big endian (0x28, 0x00)
    SIZE_OF_ATTRIBUTE_VALUE_2,                                                       //  ///Attribute Value, UUID Size
    GATT_SPEC_SERVICES_HEART_RATE, GATT_SPEC_SERVICES                                //  ///Attribute Value, 16bits UUID, little endian (0x0D, 0x18)
};                                                                                   //
//
//
const Uint8 ATT_HDL_HRS_CHARACTERISTIC_HEART_RATE_MEASUREMENT[] =                    //
{
    //
    0x00, (_HRS_HDL_OFFSET_BASE_ + 1),                                               //  ///Attribute Handle, big endian
    GATT_DECLARATIONS, GATT_DECLARATIONS_CHARACTERISTIC,                             //  ///Attribute Type, big endian (0x28, 0x03)
    SIZE_OF_ATTRIBUTE_VALUE_5,                                                       //  ///Attribute Value, size of The remaining fields
    //0x10,                                                                          //
    (                                                                                //
        ///Attribute Value, Characteristic Properties, Notify: Mandatory             //
        //GATT_DECLARATIONS_PROPERTIES_BROADCAST |                                   //
        //GATT_DECLARATIONS_PROPERTIES_READ |                                        //
        //GATT_DECLARATIONS_PROPERTIES_WRITE_WITHOUT_RESPONSE |                      //
        //GATT_DECLARATIONS_PROPERTIES_WRITE |                                       //
        GATT_DECLARATIONS_PROPERTIES_NOTIFY |                                        //
        //GATT_DECLARATIONS_PROPERTIES_INDICATE |                                    //
        //GATT_DECLARATIONS_PROPERTIES_AUTHENTICATED_SIGNED_WRITES |                 //
        //GATT_DECLARATIONS_PROPERTIES_EXTENDED_PROPERTIES |                         //
        0x00                                                                         //
    ),                                                                               //
    (_HRS_HDL_OFFSET_BASE_ + 2), 0x00,                                               //  ///Attribute Value, the next attribute handle to be described, little endian
    GATT_SPEC_CHARACTERISTIC_HEART_RATE_MEASUREMENT, GATT_SPEC_CHARACTERISTIC        //  ///Attribute Value, the next attribute type to be described, little endian (0x37, 0x2A)
};                                                                                   //
//
//
const Uint8 ATT_HDL_HRS_HEART_RATE_MEASUREMENT_INIT[] =                              //
{
    //
    0x00, (_HRS_HDL_OFFSET_BASE_ + 2),                                               //  ///Attribute Handle, big endian
    GATT_SPEC_CHARACTERISTIC, GATT_SPEC_CHARACTERISTIC_HEART_RATE_MEASUREMENT,       //  ///Attribute Type, big endian (0x2A, 0x37)
    //SIZE_OF_ATTRIBUTE_VALUE_2,                                                     //    ///Attribute Value, size of The remaining fields
    //0x04,                                                                          //
    //0x00                                                                           //
    SIZE_OF_ATTRIBUTE_VALUE_4,                                                       //  ///Attribute Value, size of The remaining fields
    0x14,                                                                            //  ///Attribute Value, Flags, Heart Rate:UINT8, RR-Interval values are present.
    0x00,                                                                            //  ///Attribute Value, Heart Rate
    0x00, 0x00                                                                       //  ///Attribute Value, RR-Interval, UINT16
};                                                                                   //
//
//
const Uint8 ATT_HDL_HRS_CLIENT_CHARACTERISTIC_CONFIGURATION_INIT[] =                 //  ///The Client Characteristic Configuration descriptor of  the Heart Rate Measurement characteristic
{
    //
    0x00, (_HRS_HDL_OFFSET_BASE_ + 3),                                               //  ///Attribute Handle, big endian
    GATT_DESCRIPTORS, GATT_DESCRIPTORS_CLIENT_CHARACTERISTIC_CONFIGURATION,          //  ///Attribute Type, big endian
    SIZE_OF_ATTRIBUTE_VALUE_2,                                                       //  ///Attribute Value, size of The remaining fields
    0x00,                                                                            //  ///Attribute Value, little endian, UINT16
    0x00                                                                             //
};                                                                                   //
//
//
Uint8 att_HDL_HRS_HEART_RATE_MEASUREMENT[] =                                         //
{
    //
    //0x00, (_HRS_HDL_OFFSET_BASE_+2),                                               //
    //GATT_SPEC_CHARACTERISTIC, GATT_SPEC_CHARACTERISTIC_HEART_RATE_MEASUREMENT,     //
    //SIZE_OF_ATTRIBUTE_VALUE_2,                                                     //
    //0x04,                                                                          //
    //0x00                                                                           //
    //SIZE_OF_ATTRIBUTE_VALUE_4,                                                     //
    0x14,                                                                            //  ///Attribute Value, Flags, Heart Rate:UINT8, RR-Interval values are present.
    0x00,                                                                            //  ///Attribute Value, Heart Rate
    0x00, 0x00                                                                       //  ///Attribute Value, RR-Interval
};                                                                                   //
//
//
Uint8 att_HDL_HRS_CLIENT_CHARACTERISTIC_CONFIGURATION[] =                            //
{
    //
    //0x00, (_HRS_HDL_OFFSET_BASE_+3),                                               //
    //GATT_DESCRIPTORS, GATT_DESCRIPTORS_CLIENT_CHARACTERISTIC_CONFIGURATION,        //
    //SIZE_OF_ATTRIBUTE_VALUE_2,                                                     //
    (                                                                                //
        //GATT_DESCRIPTORS_CLIENT_CHARACTERISTIC_CONFIGURATION_NOTIFICATION |        //
        //GATT_DESCRIPTORS_CLIENT_CHARACTERISTIC_CONFIGURATION_INDICATION |          //
        0x00                                                                         //
    ),                                                                               //  ///Attribute Value, little endian, UINT16
    0x00                                                                             //
};                                                                                   //
//
//
const Uint8 ATT_HDL_HRS_CHARACTERISTIC_BODY_SENSOR_LOCATION[] =                      //         //Option
{
    //
    0x00, (_HRS_HDL_OFFSET_BASE_ + 4),                                               //  ///Attribute Handle, big endian
    GATT_DECLARATIONS, GATT_DECLARATIONS_CHARACTERISTIC,                             //  ///Attribute Type, big endian
    SIZE_OF_ATTRIBUTE_VALUE_5,                                                       //  ///Attribute Value, size of The remaining fields
    //0x02,                                                                          //
    (                                                                                //
        ///Attribute Value, Characteristic Properties, Notify: Mandatory             //
        //GATT_DECLARATIONS_PROPERTIES_BROADCAST |                                   //
        GATT_DECLARATIONS_PROPERTIES_READ |                                          //
        //GATT_DECLARATIONS_PROPERTIES_WRITE_WITHOUT_RESPONSE |                      //
        //GATT_DECLARATIONS_PROPERTIES_WRITE |                                       //
        //GATT_DECLARATIONS_PROPERTIES_NOTIFY |                                      //
        //GATT_DECLARATIONS_PROPERTIES_INDICATE |                                    //
        //GATT_DECLARATIONS_PROPERTIES_AUTHENTICATED_SIGNED_WRITES |                 //
        //GATT_DECLARATIONS_PROPERTIES_EXTENDED_PROPERTIES |                         //
        0x00                                                                         //
    ),                                                                               //
    (_HRS_HDL_OFFSET_BASE_ + 5), 0x00,                                               //  ///Attribute Value, the next attribute handle to be described, little endian
    GATT_SPEC_CHARACTERISTIC_BODY_SENSOR_LOCATION, GATT_SPEC_CHARACTERISTIC          //  ///Attribute Value, the next attribute type to be described, little endian
};                                                                                   //
//
//
const Uint8 ATT_HDL_HRS_BODY_SENSOR_LOCATION_INIT[] =                                //
{
    //
    0x00, (_HRS_HDL_OFFSET_BASE_ + 5),                                               //  ///Attribute Handle, big endian
    GATT_SPEC_CHARACTERISTIC, GATT_SPEC_CHARACTERISTIC_BODY_SENSOR_LOCATION,         //  ///Attribute Type, big endian
    SIZE_OF_ATTRIBUTE_VALUE_1,                                                       //  ///Attribute Value, size of The remaining fields
    0x01                                                                             //  ///Attribute Value, BODY SENSOR LOCATION: Chest
};                                                                                   //  ///Describe the intended location of the heart rate measurement for the device.
//
//
Uint8 att_HDL_HRS_BODY_SENSOR_LOCATION[] =                                           //
{
    //
    //0x00, (_HRS_HDL_OFFSET_BASE_+5),                                               //
    //GATT_SPEC_CHARACTERISTIC, GATT_SPEC_CHARACTERISTIC_BODY_SENSOR_LOCATION,       //
    //SIZE_OF_ATTRIBUTE_VALUE_1,                                                     //
    0x01                                                                             //  ///Attribute Value, BODY SENSOR LOCATION: Chest
};                                                                                   //  ///Describe the intended location of the heart rate measurement for the device.
#endif                                                                               //
//
//
#ifdef _PROFILE_BAS_                                                                 //
const Uint8 ATT_HDL_BAS_PRIMARY_SERVICE[] =                                          //
{
    //
    0x00, _BAS_HDL_OFFSET_BASE_,                                                     //              //Handle
    GATT_DECLARATIONS, GATT_DECLARATIONS_PRIMARY_SERVICE,                            //              //Attribute Type
    SIZE_OF_ATTRIBUTE_VALUE_2,                                                       //
    GATT_SPEC_SERVICES_BATTERY_SERVICE, GATT_SPEC_SERVICES                           //              //Attribute Value
};                                                                                   //
//
//
const Uint8 ATT_HDL_BAS_CHARACTERISTIC_BATTERY_LEVEL[] =                             //
{
    //
    0x00, (_BAS_HDL_OFFSET_BASE_ + 1),                                               //              //Handle
    GATT_DECLARATIONS, GATT_DECLARATIONS_CHARACTERISTIC,                             //              //Attribute Type
    SIZE_OF_ATTRIBUTE_VALUE_5,                                                       //
    //0x11,                                                                          //
    (                                                                                //
        //Characteristic Properties                                                  //
        //GATT_DECLARATIONS_PROPERTIES_BROADCAST |                                   //              //Read: Mandatory
        GATT_DECLARATIONS_PROPERTIES_READ |                                          //              //Notify: Optional
        //GATT_DECLARATIONS_PROPERTIES_WRITE_WITHOUT_RESPONSE |                      //
        //GATT_DECLARATIONS_PROPERTIES_WRITE |                                       //
        GATT_DECLARATIONS_PROPERTIES_NOTIFY |                                        //
        //GATT_DECLARATIONS_PROPERTIES_INDICATE |                                    //
        //GATT_DECLARATIONS_PROPERTIES_AUTHENTICATED_SIGNED_WRITES |                 //
        //GATT_DECLARATIONS_PROPERTIES_EXTENDED_PROPERTIES |                         //
        0x00                                                                         //
    ),                                                                               //
    (_BAS_HDL_OFFSET_BASE_ + 2), 0x00,                                               //
    GATT_SPEC_CHARACTERISTIC_BATTERY_LEVEL, GATT_SPEC_CHARACTERISTIC                 //              //Attribute Value
};                                                                                   //
//
//
const Uint8 ATT_HDL_BAS_BATTERY_LEVEL_INIT[] =                                       //
{
    //
    0x00, (_BAS_HDL_OFFSET_BASE_ + 2),                                               //              //Handle
    GATT_SPEC_CHARACTERISTIC, GATT_SPEC_CHARACTERISTIC_BATTERY_LEVEL,                //              //Attribute Type
    SIZE_OF_ATTRIBUTE_VALUE_1,                                                       //
    100,                                                                             //              //Attribute Value
};                                                                                   //
//
/*                                                                                   //
const Uint8 ATT_HDL_BAS_BATTERY_LEVEL_PRESENTATION_FORMAT[] =                        //
{                                                                                    //
    0x00, 0x43,                                                                      //              //Handle
    GATT_DESCRIPTORS, GATT_DESCRIPTORS_CHARACTERISTIC_PRESENTATION_FORMAT,           //              //Attribute Type
    SIZE_OF_ATTRIBUTE_VALUE_7,                                                       //
    GATT_CHARACTERISTIC_FORMAT_UINT8,                                                //              //Format: unsigned 8-bit integer
    0x00,                                                                            //              //Exponent
    0x00,0x00,                                                                       //              //UNIT: UUID
    GATT_CHARACTERISTIC_BLUETOOTH_NAMESPACE_BLUETOOTH_SIG,                           //              //Namespace: Bluetooth SIG Assigned Numbers
    0x00,0x00                                                                        //              //Description: 0x0000
};                                                                                   //
*/                                                                                   //
//
const Uint8 ATT_HDL_BAS_CLIENT_CHARACTERISTIC_CONFIGURATION_INIT[] =                 //              //Mandatory if the Battery Level characteristic properties supports notification
{
    //
    0x00, (_BAS_HDL_OFFSET_BASE_ + 3),                                               //              //Handle
    GATT_DESCRIPTORS, GATT_DESCRIPTORS_CLIENT_CHARACTERISTIC_CONFIGURATION,          //              //Attribute Type
    SIZE_OF_ATTRIBUTE_VALUE_2,                                                       //
    0x00,                                                                            //              //Attribute Value
    0x00                                                                             //              //Attribute Value,
};                                                                                   //
//
//
Uint8 att_HDL_BAS_BATTERY_LEVEL[] =                                                  //
{
    //
    //0x00, (_BAS_HDL_OFFSET_BASE_+2),                                               //              //Handle
    //GATT_SPEC_CHARACTERISTIC, GATT_SPEC_CHARACTERISTIC_BATTERY_LEVEL,              //              //Attribute Type
    //SIZE_OF_ATTRIBUTE_VALUE_1,                                                     //
    100,                                                                             //              //Attribute Value
};                                                                                   //
//
//
Uint8 att_HDL_BAS_CLIENT_CHARACTERISTIC_CONFIGURATION[] =                            //
{
    //
    //0x00, (_BAS_HDL_OFFSET_BASE_+3),                                               //              //Handle
    //GATT_DESCRIPTORS, GATT_DESCRIPTORS_CLIENT_CHARACTERISTIC_CONFIGURATION,        //              //Attribute Type
    //SIZE_OF_ATTRIBUTE_VALUE_2,                                                     //
    (                                                                                //
        //GATT_DESCRIPTORS_CLIENT_CHARACTERISTIC_CONFIGURATION_NOTIFICATION |        //
        //GATT_DESCRIPTORS_CLIENT_CHARACTERISTIC_CONFIGURATION_INDICATION |          //
        0x00                                                                         //
    ),                                                                               //              //Attribute Value, R/W: Mandatory
    0x00                                                                             //              //Attribute Value,
};                                                                                   //
//
//
const Uint8 ATT_HDL_BAS_CHARACTERISTIC_BATTERY_POWER_STATE[] =                       //
{
    //
    0x00, (_BAS_HDL_OFFSET_BASE_ + 4),                                               //              //Handle
    GATT_DECLARATIONS, GATT_DECLARATIONS_CHARACTERISTIC,                             //              //Attribute Type
    SIZE_OF_ATTRIBUTE_VALUE_5,                                                       //
    //0x11,                                                                          //
    (                                                                                //
        //Characteristic Properties                                                  //
        //GATT_DECLARATIONS_PROPERTIES_BROADCAST |                                   //              //Read: Mandatory
        GATT_DECLARATIONS_PROPERTIES_READ |                                          //              //Notify: Optional
        //GATT_DECLARATIONS_PROPERTIES_WRITE_WITHOUT_RESPONSE |                      //
        //GATT_DECLARATIONS_PROPERTIES_WRITE |                                       //
        //GATT_DECLARATIONS_PROPERTIES_NOTIFY |                                      //
        //GATT_DECLARATIONS_PROPERTIES_INDICATE |                                    //
        //GATT_DECLARATIONS_PROPERTIES_AUTHENTICATED_SIGNED_WRITES |                 //
        //GATT_DECLARATIONS_PROPERTIES_EXTENDED_PROPERTIES |                         //
        0x00                                                                         //
    ),                                                                               //
    (_BAS_HDL_OFFSET_BASE_ + 5), 0x00,                                               //
    GATT_SPEC_CHARACTERISTIC_BATTERY_POWER_STATE, GATT_SPEC_CHARACTERISTIC           //              //Attribute Value
};                                                                                   //
//
//
const Uint8 ATT_HDL_BAS_BATTERY_POWER_STATE_INIT[] =                                 //
{
    //
    0x00, (_BAS_HDL_OFFSET_BASE_ + 5),                                               //              //Handle
    GATT_SPEC_CHARACTERISTIC, GATT_SPEC_CHARACTERISTIC_BATTERY_POWER_STATE,          //              //Attribute Type
    SIZE_OF_ATTRIBUTE_VALUE_1,                                                       //
    0,                                                                               //              //Attribute Value
};                                                                                   //
//
//
Uint8 att_HDL_BAS_BATTERY_POWER_STATE[] =                                            //
{
    //
    //0x00, (_BAS_HDL_OFFSET_BASE_+5),                                               //              //Handle
    //GATT_SPEC_CHARACTERISTIC, GATT_SPEC_CHARACTERISTIC_BATTERY_LEVEL,              //              //Attribute Type
    //SIZE_OF_ATTRIBUTE_VALUE_1,                                                     //
    0,                                                                               //              //Attribute Value
};                                                                                   //
#endif                                                                               //
//
//
#ifdef _PROFILE_HOGP_                                                                //
const Uint8 ATT_HDL_HIDS_PRIMARY_SERVICE[] =                                         //
{
    //
    0x00, (_HOGP_HDL_OFFSET_BASE_ + 0x00),                                           //              //Handle
    GATT_DECLARATIONS, GATT_DECLARATIONS_PRIMARY_SERVICE,                            //              //Attribute Type
    SIZE_OF_ATTRIBUTE_VALUE_2,                                                       //
    GATT_SPEC_SERVICES_HUMAN_INTERFACE_DEVICE, GATT_SPEC_SERVICES                    //              //Attribute Value
};                                                                                   //
//
//
const Uint8 ATT_HDL_HIDS_CHARACTERISTIC_HID_INFORMATION[] =                          //
{
    //
    0x00, (_HOGP_HDL_OFFSET_BASE_ + 0x01),                                           //
    GATT_DECLARATIONS, GATT_DECLARATIONS_CHARACTERISTIC,                             //              //Attribute Type
    SIZE_OF_ATTRIBUTE_VALUE_5,                                                       //
    //0x11,                                                                          //
    (                                                                                //
        //Characteristic Properties                                                  //
        //GATT_DECLARATIONS_PROPERTIES_BROADCAST |                                   //              //Read: Mandatory
        GATT_DECLARATIONS_PROPERTIES_READ |                                          //              //Notify: Optional
        //GATT_DECLARATIONS_PROPERTIES_WRITE_WITHOUT_RESPONSE |                      //
        //GATT_DECLARATIONS_PROPERTIES_WRITE |                                       //
        //GATT_DECLARATIONS_PROPERTIES_NOTIFY |                                      //
        //GATT_DECLARATIONS_PROPERTIES_INDICATE |                                    //
        //GATT_DECLARATIONS_PROPERTIES_AUTHENTICATED_SIGNED_WRITES |                 //
        //GATT_DECLARATIONS_PROPERTIES_EXTENDED_PROPERTIES |                         //
        0x00                                                                         //
    ),                                                                               //
    (_HOGP_HDL_OFFSET_BASE_ + 0x02), 0x00,                                           //
    GATT_SPEC_CHARACTERISTIC_HID_INFORMATION, GATT_SPEC_CHARACTERISTIC               //              //Attribute Value
};                                                                                   //
//
//
const Uint8 ATT_HDL_HIDS_HID_INFORMATION[] =                                         //
{
    //
    0x00, (_HOGP_HDL_OFFSET_BASE_ + 0x02),                                           //
    GATT_SPEC_CHARACTERISTIC, GATT_SPEC_CHARACTERISTIC_HID_INFORMATION,              //              //Attribute Type
    SIZE_OF_ATTRIBUTE_VALUE_4,                                                       //
    0x13, 0x02,     //bcdHID: 0x0213                                                 //
    0x40,           //bCountryCode: 0x40                                             //
    0x01,           //Flags: 0x_1 (providing wake-up signal to a HID host)           //
};                                                                                   //
//
//
const Uint8 ATT_HDL_HIDS_CHARACTERISTIC_HID_CONTROL_POINT[] =                        //
{
    //
    0x00, (_HOGP_HDL_OFFSET_BASE_ + 0x03),                                           //
    GATT_DECLARATIONS, GATT_DECLARATIONS_CHARACTERISTIC,                             //              //Attribute Type
    SIZE_OF_ATTRIBUTE_VALUE_5,                                                       //
    //0x11,                                                                          //
    (                                                                                //
        //Characteristic Properties                                                  //
        //GATT_DECLARATIONS_PROPERTIES_BROADCAST |                                   //              //Read: Mandatory
        //GATT_DECLARATIONS_PROPERTIES_READ |                                        //              //Notify: Optional
        GATT_DECLARATIONS_PROPERTIES_WRITE_WITHOUT_RESPONSE |                        //
        //GATT_DECLARATIONS_PROPERTIES_WRITE |                                       //
        //GATT_DECLARATIONS_PROPERTIES_NOTIFY |                                      //
        //GATT_DECLARATIONS_PROPERTIES_INDICATE |                                    //
        //GATT_DECLARATIONS_PROPERTIES_AUTHENTICATED_SIGNED_WRITES |                 //
        //GATT_DECLARATIONS_PROPERTIES_EXTENDED_PROPERTIES |                         //
        0x00                                                                         //
    ),                                                                               //
    (_HOGP_HDL_OFFSET_BASE_ + 0x04), 0x00,                                           //
    GATT_SPEC_CHARACTERISTIC_HID_CONTROL_POINT, GATT_SPEC_CHARACTERISTIC             //              //Attribute Value
};                                                                                   //
//
//
const Uint8 ATT_HDL_HIDS_HID_CONTROL_POINT_INIT[] =                                  //
{
    //
    0x00, (_HOGP_HDL_OFFSET_BASE_ + 0x04),                                           //
    GATT_SPEC_CHARACTERISTIC, GATT_SPEC_CHARACTERISTIC_HID_CONTROL_POINT,            //              //Attribute Type
    SIZE_OF_ATTRIBUTE_VALUE_1,                                                       //
    0x01,                       //0: Suspend, 1: Exit Suspend                        //
};                                                                                   //
//
//
Uint8 att_HDL_HIDS_HID_CONTROL_POINT[] =                                             //
{
    //
    //0x00, (_HOGP_HDL_OFFSET_BASE_+0x04),                                           //              //Handle
    //GATT_SPEC_CHARACTERISTIC, GATT_SPEC_CHARACTERISTIC_HID_CONTROL_POINT,          //              //Attribute Type
    //SIZE_OF_ATTRIBUTE_VALUE_1,                                                     //
    0x01,                       //0: Suspend, 1: Exit Suspend                        //
};                                                                                   //
//
//
const Uint8 ATT_HDL_HIDS_CHARACTERISTIC_REPORT_MAP[] =                               //
{
    //
    0x00, (_HOGP_HDL_OFFSET_BASE_ + 0x05),                                           //              //Handle
    GATT_DECLARATIONS, GATT_DECLARATIONS_CHARACTERISTIC,                             //              //Attribute Type
    SIZE_OF_ATTRIBUTE_VALUE_5,                                                       //
    //0x11,                                                                          //
    (                                                                                //
        //Characteristic Properties                                                  //
        //GATT_DECLARATIONS_PROPERTIES_BROADCAST |                                   //              //Read: Mandatory
        GATT_DECLARATIONS_PROPERTIES_READ |                                          //              //Notify: Optional
        //GATT_DECLARATIONS_PROPERTIES_WRITE_WITHOUT_RESPONSE |                      //
        //GATT_DECLARATIONS_PROPERTIES_WRITE |                                       //
        //GATT_DECLARATIONS_PROPERTIES_NOTIFY |                                      //
        //GATT_DECLARATIONS_PROPERTIES_INDICATE |                                    //
        //GATT_DECLARATIONS_PROPERTIES_AUTHENTICATED_SIGNED_WRITES |                 //
        //GATT_DECLARATIONS_PROPERTIES_EXTENDED_PROPERTIES |                         //
        0x00                                                                         //
    ),                                                                               //
    (_HOGP_HDL_OFFSET_BASE_ + 0x06), 0x00,                                           //
    GATT_SPEC_CHARACTERISTIC_REPORT_MAP, GATT_SPEC_CHARACTERISTIC                                   //Attribute Value
};


const Uint8 ATT_HDL_HIDS_REPORT_MAP[] =    //Device Class Definition for Human Interface Devices (HID) Version 1.11, 6.2.2 Report Descriptor
{
    0x00, (_HOGP_HDL_OFFSET_BASE_ + 0x06),                                                          //Handle
    GATT_SPEC_CHARACTERISTIC, GATT_SPEC_CHARACTERISTIC_REPORT_MAP,                                  //Attribute Type
    //(SIZE_OF_ATTRIBUTE_VALUE_0+33+2+82+2+54+2),
#ifndef _PROFILE_HOGP_EMC_
    (SIZE_OF_ATTRIBUTE_VALUE_MS + SIZE_OF_ATTRIBUTE_VALUE_KB + SIZE_OF_ATTRIBUTE_VALUE_CS),

#ifdef _PROFILE_HOGP_KEYBOARD_
    0x05, 0x01,
    0x09, 0x06,
    0xa1, 0x01,
#ifdef _PROFILE_HOGP_MULTI_
    0x85, (0x00 + __PROFILE_HOGP_KEYBOARD_RPIDOFST_),
#endif
    0x05, 0x08,
    0x19, 0x01,
    0x29, 0x03,
    0x75, 0x01,
    0x95, 0x03,
    0x15, 0x00,
    0x25, 0x01,
    0x91, 0x02,
    0x95, 0x05,
    0x91, 0x01,
    0x05, 0x07,
    0x19, 0xe0,
    0x29, 0xe7,
    0x95, 0x08,
    0x81, 0x02,
    0x75, 0x08,
    0x95, 0x01,
    0x81, 0x01,
    0x19, 0x00,
    0x29, 0x91,
    0x26, 0xff, 0x00,
    0x95, 0x06,
    0x81, 0x00,
#ifdef _PROFILE_HOGP_KEYBOARD_PHOTO_
    0x05, 0x0C,
    0x09, 0x01,
    0x15, 0x00,
    0x25, 0x01,
    0x09, 0xE9,
    0x09, 0xEA,
    0x75, 0x01,
    0x95, 0x02,
    0x81, 0x02,
    0x09, 0xE2,
    0x09, 0x00,
    0x81, 0x06,
    0x09, 0x00,
    0x95, 0x04,
    0x81, 0x02,
#endif
    0xc0,
#endif
#ifdef _PROFILE_HOGP_MOUSE_
    0x05, 0x01,
    0x09, 0x02,
    0xA1, 0x01,
    0x09, 0x01,
    0xA1, 0x00,
    0x05, 0x09,
#ifdef _PROFILE_HOGP_MULTI_
    0x85, 0x02,
#endif
    0x19, 0x01,
    0x29, 0x05,
    0x15, 0x00,
    0x25, 0x01,
    0x95, 0x05,
    0x75, 0x01,
    0x81, 0x02,
    0x95, 0x01,
    0x75, 0x03,
    0x81, 0x03,
    0x05, 0x01,
    0x09, 0x30,
    0x09, 0x31,
    0x16, 0x00, 0xF8,
    0x26, 0xFF, 0x07,
    0x75, 0x10,
    0x95, 0x02,
    0x81, 0x06,
    0x09, 0x38,
    0x15, 0x80,
    0x25, 0x7F,
    0x75, 0x08,
    0x95, 0x01,
    0x81, 0x06,
    0x09, 0x01,
    0x15, 0x00,
    0x25, 0x03,
    0x95, 0x01,
    0x75, 0x02,
    0xB1, 0x02,
    0x95, 0x01,
    0x75, 0x06,
    0xB1, 0x03,

    0x05, 0x0C,
    0x95, 0x01,
    0x75, 0x10,
    0x16, 0x01, 0x80,
    0x26, 0xFF, 0x7F,
    0x0A, 0x38, 0x02,
    0x81, 0x06,

    0xC0, 0xC0,
#endif      //#ifdef _PROFILE_HOGP_MOUSE_
#ifdef _PROFILE_HOGP_COMSUMER_
    0x05, 0x0C,
    0x09, 0x01,
    0xA1, 0x01,
#ifdef _PROFILE_HOGP_MULTI_
    0x85, 0x03,
#endif
    0x19, 0x00,
    0x2A, 0x9C, 0x02,
    0x15, 0x00,
    0x26, 0x9C, 0x02,
    0x75, 0x10,
    0x95, 0x01,
    0x81, 0x00,
    0xC0,
#endif
#else       //#ifndef _PROFILE_HOGP_EMC_
#ifdef _PROFILE_HOGP_EMC_1501_
#ifdef _PROFILE_HOGP_EMC_1503_
    (SIZE_OF_ATTRIBUTE_VALUE_124 - 23 + 5 - 6 + 20 + 66),
#else
    (SIZE_OF_ATTRIBUTE_VALUE_124 - 23 + 5 - 6 + 66),
#endif
    0x05, 0x01,
    0x09, 0x06,
    0xA1, 0x01,
    0x85, 0x02,
    0x05, 0x07,
    0x19, 0xE0,
    0x29, 0xE7,
    0x15, 0x00,
    0x25, 0x01,
    0x75, 0x01,
    0x95, 0x08,
    0x81, 0x02,
    0x75, 0x08,
    0x95, 0x01,
    0x81, 0x01,
    0x19, 0x00,
    0x29, 0x65,
    0x26, 0xFF, 0x00,
    0x75, 0x08,
    0x95, 0x06,
    0x81, 0x00,
    0x05, 0x08,
    0x19, 0x01,
    0x29, 0x05,
    0x15, 0x00,
    0x25, 0x01,
    0x75, 0x01,
    0x95, 0x05,
    0x91, 0x02,
    0x75, 0x03,
    0x95, 0x01,
    0x91, 0x01,
    0xC0,

    0x05, 0x01,
    0x09, 0x02,
    0xA1, 0x01,
    0x85, 0x01,
    0x09, 0x01,
    0xA1, 0x00,
    0x05, 0x09,
    0x19, 0x01,
    0x29, 0x02,
    0x15, 0x00,
    0x25, 0x01,
    0x75, 0x01,
    0x95, 0x02,
    0x81, 0x02,
    0x95, 0x06,
    0x81, 0x03,
    0x05, 0x01,
    0x09, 0x30,
    0x09, 0x31,
    0x09, 0x38,
    0x15, 0x81,
    0x25, 0x7F,
    0x75, 0x08,
    0x95, 0x03,
    0x81, 0x06,
    0xC0,
    0x06, 0x00, 0xFF,
    0x85, 0x0D,
    0x09, 0xC5,
    0x15, 0x00,
    0x26, 0xFF, 0x00,
    0x75, 0x08,
    0x95, 0x14,
    0xB1, 0x02,
    0x85, 0x0C,
    0x09, 0xC6,
    0x95, 0x14,
    0x75, 0x08,
    0xB1, 0x02,
    0x85, 0x0B,
    0x09, 0xC7,
    0x95, 0x14,
    0x75, 0x08,
    0xB1, 0x02,
    0x09, 0x01,
    0x85, 0x5D,
    0x95, 0x14,
    0x75, 0x08,
    0x81, 0x06,
#ifdef _PROFILE_HOGP_EMC_1503_
    0x85, 0x81,
    0x09, 0x01,
    0x95, 0x14,
    0x75, 0x08,
    0x81, 0x06,
    0x85, 0x82,
    0x09, 0x01,
    0x95, 0x14,
    0x75, 0x08,
    0x81, 0x06,
#endif
    0xC0
#else
    (SIZE_OF_ATTRIBUTE_VALUE_124 + 51),
    0x05, 0x01,
    0x09, 0x06,
    0xA1, 0x01,
    0x85, 0x02,
    0x05, 0x07,
    0x19, 0xE0,
    0x29, 0xE7,
    0x15, 0x00,
    0x25, 0x01,
    0x75, 0x01,
    0x95, 0x08,
    0x81, 0x02,
    0x75, 0x08,
    0x95, 0x01,
    0x81, 0x01,
    0x19, 0x00,
    0x29, 0x65,
    //0x15, 0x00,
    //0x25, 0x65,
    0x26, 0xFF, 0x00,
    0x75, 0x08,
    0x95, 0x06,
    0x81, 0x00,
    0x05, 0x08,
    0x19, 0x01,
    0x29, 0x05,
    0x15, 0x00,
    0x25, 0x01,
    0x75, 0x01,
    0x95, 0x05,
    0x91, 0x02,
    0x75, 0x03,
    0x95, 0x01,
    0x91, 0x01,
    0xC0,
    0x05, 0x01,
    0x09, 0x02,
    0xA1, 0x01,
    0x09, 0x01,
    0xA1, 0x00,
    0x05, 0x09,
    0x85, 0x01,
    0x19, 0x01,
    0x29, 0x03,
    0x15, 0x00,
    0x25, 0x01,
    0x75, 0x01,
    0x95, 0x03,
    0x81, 0x02,
    0x75, 0x05,
    0x95, 0x01,
    0x81, 0x01,
    0x05, 0x01,
    0x09, 0x30,
    0x09, 0x31,
    0x15, 0x81,
    0x25, 0x7F,
    0x75, 0x08,
    0x95, 0x02,
    0x81, 0x06,
    0x09, 0x38,
    0x95, 0x01,
    0x81, 0x06,
    0xC0,
    0xC0,
    0x06, 0x12, 0x0F,
    0x0A, 0x12, 0x0F,
    0xA1, 0x01,
    0x85, 0x81,
    0x09, 0x01,
    0x75, 0x08,
    0x95, 0x14,
    0x16, 0x00, 0x00,
    0x26, 0xFF, 0x00,
    0x81, 0x00,
    0x09, 0x00,
    0x85, 0x05,
    0x75, 0x08,
    0x95, 0x07,
    0xB1, 0x02,
    0x09, 0x00,
    0x85, 0x08,
    0x95, 0x07,
    0xB1, 0x02,
    0x09, 0x00,
    0x85, 0x09,
    0x95, 0x07,
    0xB1, 0x02,
    0xC0,
#endif
#endif      //#ifndef _PROFILE_HOGP_EMC_
};


#ifdef _PROFILE_HOGP_COMSUMER_
const Uint8 ATT_HDL_HIDS_CHARACTERISTIC_REPORT_CSI[] =
{
    0x00, _HOGP_HDL_OFFSET_CONSUMER_,                                                               //Handle
    GATT_DECLARATIONS, GATT_DECLARATIONS_CHARACTERISTIC,                                            //Attribute Type
    SIZE_OF_ATTRIBUTE_VALUE_5,
    //0x11,
    (
        //Characteristic Properties
        //GATT_DECLARATIONS_PROPERTIES_BROADCAST |                                                  //Read: Mandatory
        GATT_DECLARATIONS_PROPERTIES_READ |                                                         //Notify: Optional
        //GATT_DECLARATIONS_PROPERTIES_WRITE_WITHOUT_RESPONSE |
        GATT_DECLARATIONS_PROPERTIES_WRITE |
        GATT_DECLARATIONS_PROPERTIES_NOTIFY |
        //GATT_DECLARATIONS_PROPERTIES_INDICATE |
        //GATT_DECLARATIONS_PROPERTIES_AUTHENTICATED_SIGNED_WRITES |
        //GATT_DECLARATIONS_PROPERTIES_EXTENDED_PROPERTIES |
        0x00
    ),
    (_HOGP_HDL_OFFSET_CONSUMER_ + 0x01), 0x00,
    GATT_SPEC_CHARACTERISTIC_REPORT, GATT_SPEC_CHARACTERISTIC                                       //Attribute Value
};


const Uint8 ATT_HDL_HIDS_REPORT_CSI_INIT[] =
{
    0x00, (_HOGP_HDL_OFFSET_CONSUMER_ + 0x01),                                                      //Handle
    GATT_SPEC_CHARACTERISTIC, GATT_SPEC_CHARACTERISTIC_REPORT,                                      //Attribute Type
    SIZE_OF_ATTRIBUTE_VALUE_2,
    0x00, 0x00,
};


Uint8 att_HDL_HIDS_REPORT_CSI[] =
{
    //0x00, (_HOGP_HDL_OFFSET_CONSUMER_+0x01),                                                      //Handle
    //GATT_SPEC_CHARACTERISTIC, GATT_SPEC_CHARACTERISTIC_REPORT,                                    //Attribute Type
    //SIZE_OF_ATTRIBUTE_VALUE_6,
    0x00, 0x00,
};


const Uint8 ATT_HDL_HIDS_REPORT_CSI_CLIENT_CHARACTERISTIC_CONFIGURATION_INIT[] =                    //Mandatory if the Battery Level characteristic properties supports notification
{
    0x00, (_HOGP_HDL_OFFSET_CONSUMER_ + 0x02),                                                      //Handle
    GATT_DESCRIPTORS, GATT_DESCRIPTORS_CLIENT_CHARACTERISTIC_CONFIGURATION,                         //Attribute Type
    SIZE_OF_ATTRIBUTE_VALUE_2,
    0x00,                                                                                           //Attribute Value
    0x00                                                                                            //Attribute Value,
};


Uint8 att_HDL_HIDS_REPORT_CSI_CLIENT_CHARACTERISTIC_CONFIGURATION[] =
{
    //0x00, (_HOGP_HDL_OFFSET_CONSUMER_+0x02),                                                      //Handle
    //GATT_DESCRIPTORS, GATT_DESCRIPTORS_CLIENT_CHARACTERISTIC_CONFIGURATION,                       //Attribute Type
    //SIZE_OF_ATTRIBUTE_VALUE_2,
    (
        //GATT_DESCRIPTORS_CLIENT_CHARACTERISTIC_CONFIGURATION_NOTIFICATION |
        //GATT_DESCRIPTORS_CLIENT_CHARACTERISTIC_CONFIGURATION_INDICATION |
        0x00
    ),                                                                                              //Attribute Value, R/W: Mandatory
    0x00                                                                                            //Attribute Value,
};


const Uint8 ATT_HDL_HIDS_REPORT_CSI_REPORT_REFERENCE[] =
{
    0x00, (_HOGP_HDL_OFFSET_CONSUMER_ + 0x03),                                                      //Handle
    GATT_DESCRIPTORS, GATT_DESCRIPTORS_REPORT_REFERENCE,                                            //Attribute Type
    SIZE_OF_ATTRIBUTE_VALUE_2,
    (0x00 + __PROFILE_HOGP_CS_RPIDOFST_),                                                           //Attribute Value, Report ID
    0x01                                                                                            //Attribute Value, Report Type: Input (0x01)
};
#endif      //#ifdef _PROFILE_HOGP_COMSUMER_

#ifndef _PROFILE_HOGP_EMC_
#ifdef _PROFILE_HOGP_MOUSE_
const Uint8 ATT_HDL_HIDS_CHARACTERISTIC_REPORT_MSI[] =
{
    0x00, _HOGP_HDL_OFFSET_MOUSE_,                                                                  //Handle
    GATT_DECLARATIONS, GATT_DECLARATIONS_CHARACTERISTIC,                                            //Attribute Type
    SIZE_OF_ATTRIBUTE_VALUE_5,
    //0x11,
    (
        //Characteristic Properties
        //GATT_DECLARATIONS_PROPERTIES_BROADCAST |                                                  //Read: Mandatory
        GATT_DECLARATIONS_PROPERTIES_READ |                                                         //Notify: Optional
        //GATT_DECLARATIONS_PROPERTIES_WRITE_WITHOUT_RESPONSE |
        GATT_DECLARATIONS_PROPERTIES_WRITE |
        GATT_DECLARATIONS_PROPERTIES_NOTIFY |
        //GATT_DECLARATIONS_PROPERTIES_INDICATE |
        //GATT_DECLARATIONS_PROPERTIES_AUTHENTICATED_SIGNED_WRITES |
        //GATT_DECLARATIONS_PROPERTIES_EXTENDED_PROPERTIES |
        0x00
    ),
    (_HOGP_HDL_OFFSET_MOUSE_ + 0x01), 0x00,
    GATT_SPEC_CHARACTERISTIC_REPORT, GATT_SPEC_CHARACTERISTIC                                       //Attribute Value
};


const Uint8 ATT_HDL_HIDS_REPORT_MSI_INIT[] =
{
    0x00, (_HOGP_HDL_OFFSET_MOUSE_ + 0x01),                                                         //Handle
    GATT_SPEC_CHARACTERISTIC, GATT_SPEC_CHARACTERISTIC_REPORT,                                      //Attribute Type
    SIZE_OF_ATTRIBUTE_VALUE_8,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  0x00, 0x00,
};


Uint8 att_HDL_HIDS_REPORT_MSI[] =
{
    //0x00, (_HOGP_HDL_OFFSET_MOUSE_+0x01),                                                         //Handle
    //GATT_SPEC_CHARACTERISTIC, GATT_SPEC_CHARACTERISTIC_REPORT,                                    //Attribute Type
    //SIZE_OF_ATTRIBUTE_VALUE_6,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
};


const Uint8 ATT_HDL_HIDS_REPORT_MSI_CLIENT_CHARACTERISTIC_CONFIGURATION_INIT[] =                    //Mandatory if the Battery Level characteristic properties supports notification
{
    0x00, (_HOGP_HDL_OFFSET_MOUSE_ + 0x02),                                                         //Handle
    GATT_DESCRIPTORS, GATT_DESCRIPTORS_CLIENT_CHARACTERISTIC_CONFIGURATION,                         //Attribute Type
    SIZE_OF_ATTRIBUTE_VALUE_2,
    0x00,                                                                                           //Attribute Value
    0x00                                                                                            //Attribute Value,
};


Uint8 att_HDL_HIDS_REPORT_MSI_CLIENT_CHARACTERISTIC_CONFIGURATION[] =
{
    //0x00, (_HOGP_HDL_OFFSET_MOUSE_+0x02),                                                         //Handle
    //GATT_DESCRIPTORS, GATT_DESCRIPTORS_CLIENT_CHARACTERISTIC_CONFIGURATION,                       //Attribute Type
    //SIZE_OF_ATTRIBUTE_VALUE_2,
    (
        //GATT_DESCRIPTORS_CLIENT_CHARACTERISTIC_CONFIGURATION_NOTIFICATION |
        //GATT_DESCRIPTORS_CLIENT_CHARACTERISTIC_CONFIGURATION_INDICATION |
        0x00
    ),                                                                                              //Attribute Value, R/W: Mandatory
    0x00                                                                                            //Attribute Value,
};


const Uint8 ATT_HDL_HIDS_REPORT_MSI_REPORT_REFERENCE[] =
{
    0x00, (_HOGP_HDL_OFFSET_MOUSE_ + 0x03),                                                         //Handle
    GATT_DESCRIPTORS, GATT_DESCRIPTORS_REPORT_REFERENCE,                                            //Attribute Type
    SIZE_OF_ATTRIBUTE_VALUE_2,
    (0x00 + __PROFILE_HOGP_MS_RPIDOFST_),                                                           //Attribute Value, Report ID
    0x01                                                                                            //Attribute Value, Report Type: Input (0x01)
};


const Uint8 ATT_HDL_HIDS_CHARACTERISTIC_REPORT_MSF[] =
{
    0x00, (_HOGP_HDL_OFFSET_MOUSE_ + 0x04),
    GATT_DECLARATIONS, GATT_DECLARATIONS_CHARACTERISTIC,                                            //Attribute Type
    SIZE_OF_ATTRIBUTE_VALUE_5,
    //0x11,
    (
        //Characteristic Properties
        //GATT_DECLARATIONS_PROPERTIES_BROADCAST |                                                  //Read: Mandatory
        GATT_DECLARATIONS_PROPERTIES_READ |                                                         //Notify: Optional
        //GATT_DECLARATIONS_PROPERTIES_WRITE_WITHOUT_RESPONSE |
        GATT_DECLARATIONS_PROPERTIES_WRITE |
        //GATT_DECLARATIONS_PROPERTIES_NOTIFY |
        //GATT_DECLARATIONS_PROPERTIES_INDICATE |
        //GATT_DECLARATIONS_PROPERTIES_AUTHENTICATED_SIGNED_WRITES |
        //GATT_DECLARATIONS_PROPERTIES_EXTENDED_PROPERTIES |
        0x00
    ),
    (_HOGP_HDL_OFFSET_MOUSE_ + 0x05), 0x00,
    GATT_SPEC_CHARACTERISTIC_REPORT, GATT_SPEC_CHARACTERISTIC                                       //Attribute Value
};


const Uint8 ATT_HDL_HIDS_REPORT_MSF_INIT[] =
{
    0x00, (_HOGP_HDL_OFFSET_MOUSE_ + 0x05),
    GATT_SPEC_CHARACTERISTIC, GATT_SPEC_CHARACTERISTIC_REPORT,                                      //Attribute Type
    SIZE_OF_ATTRIBUTE_VALUE_1,
    0x03                                                                                            //Attribute Value
};


Uint8 att_HDL_HIDS_REPORT_MSF[] =
{
    //0x00, (_HOGP_HDL_OFFSET_MOUSE_+0x05),                                                         //Handle
    //GATT_SPEC_CHARACTERISTIC, GATT_SPEC_CHARACTERISTIC_REPORT,                                    //Attribute Type
    //SIZE_OF_ATTRIBUTE_VALUE_1,
    0x03,                                                                                           //Attribute Value
};


const Uint8 ATT_HDL_HIDS_REPORT_MSF_REPORT_REFERENCE[] =
{
    0x00, (_HOGP_HDL_OFFSET_MOUSE_ + 0x06),
    GATT_DESCRIPTORS, GATT_DESCRIPTORS_REPORT_REFERENCE,                                            //Attribute Type
    SIZE_OF_ATTRIBUTE_VALUE_2,
    (0x00 + __PROFILE_HOGP_MS_RPIDOFST_),                                                           //Attribute Value, Report ID
    0x03                                                                                            //Attribute Value, Report Type: Feature (0x03)
};


const Uint8 ATT_HDL_HIDS_REPORT_MSF_USER_DESCRIPTION[] =
{
    0x00, (_HOGP_HDL_OFFSET_MOUSE_ + 0x07),
    GATT_DESCRIPTORS, GATT_DESCRIPTORS_CHARACTERISTIC_USER_DESCRIPTION,                             //Attribute Type
    (SIZE_OF_ATTRIBUTE_VALUE_0 + 65),
    0x42, 0x4C, 0x45, 0x20, 0x4D, 0x4F, 0x55, 0x53,
    0x45, 0x20, 0x52, 0x45, 0x53, 0x4F, 0x4C, 0x55,
    0x54, 0x49, 0x4F, 0x4E, 0x20, 0x49, 0x4E, 0x20,
    0x43, 0x50, 0x49, 0x0A, 0x20, 0x30, 0x20, 0x3D,
    0x20, 0x34, 0x30, 0x30, 0x0A, 0x20, 0x31, 0x20,
    0x3D, 0x20, 0x38, 0x30, 0x30, 0x0A, 0x20, 0x32,
    0x20, 0x3D, 0x20, 0x31, 0x32, 0x30, 0x30, 0x0A,
    0x20, 0x33, 0x20, 0x3D, 0x20, 0x31, 0x36, 0x30,
    0x30,
};
#endif      //#ifdef _PROFILE_HOGP_MOUSE_


#ifdef _PROFILE_HOGP_KEYBOARD_
const Uint8 ATT_HDL_HIDS_CHARACTERISTIC_REPORT_KBI[] =
{
    0x00, _HOGP_HDL_OFFSET_KB_,                                                                     //Handle
    GATT_DECLARATIONS, GATT_DECLARATIONS_CHARACTERISTIC,                                            //Attribute Type
    SIZE_OF_ATTRIBUTE_VALUE_5,
    //0x11,
    (
        //Characteristic Properties
        //GATT_DECLARATIONS_PROPERTIES_BROADCAST |                                                  //Read: Mandatory
        GATT_DECLARATIONS_PROPERTIES_READ |                                                         //Notify: Optional
        //GATT_DECLARATIONS_PROPERTIES_WRITE_WITHOUT_RESPONSE |
        GATT_DECLARATIONS_PROPERTIES_WRITE |
        GATT_DECLARATIONS_PROPERTIES_NOTIFY |
        //GATT_DECLARATIONS_PROPERTIES_INDICATE |
        //GATT_DECLARATIONS_PROPERTIES_AUTHENTICATED_SIGNED_WRITES |
        //GATT_DECLARATIONS_PROPERTIES_EXTENDED_PROPERTIES |
        0x00
    ),
    (_HOGP_HDL_OFFSET_KB_ + 0x01), 0x00,
    GATT_SPEC_CHARACTERISTIC_REPORT, GATT_SPEC_CHARACTERISTIC                                       //Attribute Value
};


const Uint8 ATT_HDL_HIDS_REPORT_KBI_INIT[] =
{
    0x00, (_HOGP_HDL_OFFSET_KB_ + 0x01),                                                            //Handle
    GATT_SPEC_CHARACTERISTIC, GATT_SPEC_CHARACTERISTIC_REPORT,                                      //Attribute Type
#ifdef _PROFILE_HOGP_KEYBOARD_PHOTO_
    SIZE_OF_ATTRIBUTE_VALUE_9,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
#else
    SIZE_OF_ATTRIBUTE_VALUE_8,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
#endif
};


Uint8 att_HDL_HIDS_REPORT_KBI[] =
{
    //0x00, (_HOGP_HDL_OFFSET_KB_+0x01),                                                            //Handle
    //GATT_SPEC_CHARACTERISTIC, GATT_SPEC_CHARACTERISTIC_REPORT,                                    //Attribute Type
    //SIZE_OF_ATTRIBUTE_VALUE_9,
#ifdef _PROFILE_HOGP_KEYBOARD_PHOTO_
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
#else
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
#endif
};


const Uint8 ATT_HDL_HIDS_REPORT_KBI_CLIENT_CHARACTERISTIC_CONFIGURATION_INIT[] =                    //Mandatory if the Battery Level characteristic properties supports notification
{
    0x00, (_HOGP_HDL_OFFSET_KB_ + 0x02),                                                            //Handle
    GATT_DESCRIPTORS, GATT_DESCRIPTORS_CLIENT_CHARACTERISTIC_CONFIGURATION,                         //Attribute Type
    SIZE_OF_ATTRIBUTE_VALUE_2,
    0x00,                                                                                           //Attribute Value
    0x00                                                                                            //Attribute Value,
};


Uint8 att_HDL_HIDS_REPORT_KBI_CLIENT_CHARACTERISTIC_CONFIGURATION[] =
{
    //0x00, (_HOGP_HDL_OFFSET_KB_+0x02),                                                            //Handle
    //GATT_DESCRIPTORS, GATT_DESCRIPTORS_CLIENT_CHARACTERISTIC_CONFIGURATION,                       //Attribute Type
    //SIZE_OF_ATTRIBUTE_VALUE_2,
    (
        //GATT_DESCRIPTORS_CLIENT_CHARACTERISTIC_CONFIGURATION_NOTIFICATION |
        //GATT_DESCRIPTORS_CLIENT_CHARACTERISTIC_CONFIGURATION_INDICATION |
        0x00
    ),                                                                                              //Attribute Value, R/W: Mandatory
    0x00                                                                                            //Attribute Value,
};


const Uint8 ATT_HDL_HIDS_REPORT_KBI_REPORT_REFERENCE[] =
{
    0x00, (_HOGP_HDL_OFFSET_KB_ + 0x03),                                                            //Handle
    GATT_DESCRIPTORS, GATT_DESCRIPTORS_REPORT_REFERENCE,                                            //Attribute Type
    SIZE_OF_ATTRIBUTE_VALUE_2,
    (0x00 + __PROFILE_HOGP_KEYBOARD_RPIDOFST_),                                                     //Attribute Value, Report ID
    0x01                                                                                            //Attribute Value, Report Type: Input (0x01)
};


const Uint8 ATT_HDL_HIDS_CHARACTERISTIC_REPORT_KBO[] =
{
    0x00, (_HOGP_HDL_OFFSET_KB_ + 0x04),
    GATT_DECLARATIONS, GATT_DECLARATIONS_CHARACTERISTIC,                                            //Attribute Type
    SIZE_OF_ATTRIBUTE_VALUE_5,
    //0x11,
    (
        //Characteristic Properties
        //GATT_DECLARATIONS_PROPERTIES_BROADCAST |                                                  //Read: Mandatory
        GATT_DECLARATIONS_PROPERTIES_READ |                                                         //Notify: Optional
        GATT_DECLARATIONS_PROPERTIES_WRITE_WITHOUT_RESPONSE |
        GATT_DECLARATIONS_PROPERTIES_WRITE |
        //GATT_DECLARATIONS_PROPERTIES_NOTIFY |
        //GATT_DECLARATIONS_PROPERTIES_INDICATE |
        //GATT_DECLARATIONS_PROPERTIES_AUTHENTICATED_SIGNED_WRITES |
        //GATT_DECLARATIONS_PROPERTIES_EXTENDED_PROPERTIES |
        0x00
    ),
    (_HOGP_HDL_OFFSET_KB_ + 0x05), 0x00,
    GATT_SPEC_CHARACTERISTIC_REPORT, GATT_SPEC_CHARACTERISTIC                                       //Attribute Value
};


const Uint8 ATT_HDL_HIDS_REPORT_KBO_INIT[] =
{
    0x00, (_HOGP_HDL_OFFSET_KB_ + 0x05),
    GATT_SPEC_CHARACTERISTIC, GATT_SPEC_CHARACTERISTIC_REPORT,                                      //Attribute Type
    SIZE_OF_ATTRIBUTE_VALUE_1,
    0x03                                                                                            //Attribute Value
};


Uint8 att_HDL_HIDS_REPORT_KBO[] =
{
    //0x00, (_HOGP_HDL_OFFSET_KB_+0x05),                                                            //Handle
    //GATT_SPEC_CHARACTERISTIC, GATT_SPEC_CHARACTERISTIC_REPORT,                                    //Attribute Type
    //SIZE_OF_ATTRIBUTE_VALUE_1,
    0x03,                                                                                           //Attribute Value
};


const Uint8 ATT_HDL_HIDS_REPORT_KBO_REPORT_REFERENCE[] =
{
    0x00, (_HOGP_HDL_OFFSET_KB_ + 0x06),
    GATT_DESCRIPTORS, GATT_DESCRIPTORS_REPORT_REFERENCE,                                            //Attribute Type
    SIZE_OF_ATTRIBUTE_VALUE_2,
    (0x00 + __PROFILE_HOGP_KEYBOARD_RPIDOFST_),                                                     //Attribute Value, Report ID : 01
    0x02                                                                                            //Attribute Value, Report Type: Output (0x02)
};
#endif      //#ifdef _PROFILE_HOGP_MOUSE_
#else       //#ifndef _PROFILE_HOGP_EMC_
#ifdef _PROFILE_HOGP_MOUSE_
const Uint8 ATT_HDL_HIDS_CHARACTERISTIC_REPORT_MSI[] =
{
    0x00, _HOGP_HDL_OFFSET_MOUSE_,                                                                  //Handle
    GATT_DECLARATIONS, GATT_DECLARATIONS_CHARACTERISTIC,                                            //Attribute Type
    SIZE_OF_ATTRIBUTE_VALUE_5,
    //0x11,
    (
        //Characteristic Properties
        //GATT_DECLARATIONS_PROPERTIES_BROADCAST |                                                  //Read: Mandatory
        GATT_DECLARATIONS_PROPERTIES_READ |                                                         //Notify: Optional
        //GATT_DECLARATIONS_PROPERTIES_WRITE_WITHOUT_RESPONSE |
        GATT_DECLARATIONS_PROPERTIES_WRITE |
        GATT_DECLARATIONS_PROPERTIES_NOTIFY |
        //GATT_DECLARATIONS_PROPERTIES_INDICATE |
        //GATT_DECLARATIONS_PROPERTIES_AUTHENTICATED_SIGNED_WRITES |
        //GATT_DECLARATIONS_PROPERTIES_EXTENDED_PROPERTIES |
        0x00
    ),
    (_HOGP_HDL_OFFSET_MOUSE_ + 0x01), 0x00,
    GATT_SPEC_CHARACTERISTIC_REPORT, GATT_SPEC_CHARACTERISTIC                                       //Attribute Value
};


const Uint8 ATT_HDL_HIDS_REPORT_MSI_INIT[] =
{
    0x00, (_HOGP_HDL_OFFSET_MOUSE_ + 0x01),                                                         //Handle
    GATT_SPEC_CHARACTERISTIC, GATT_SPEC_CHARACTERISTIC_REPORT,                                      //Attribute Type
    SIZE_OF_ATTRIBUTE_VALUE_4,
    0x00, 0x00, 0x00, 0x00,
};


Uint8 att_HDL_HIDS_REPORT_MSI[] =
{
    //0x00, (_HOGP_HDL_OFFSET_MOUSE_+0x01),                                                         //Handle
    //GATT_SPEC_CHARACTERISTIC, GATT_SPEC_CHARACTERISTIC_REPORT,                                    //Attribute Type
    //SIZE_OF_ATTRIBUTE_VALUE_4,
    0x00, 0x00, 0x00, 0x00,                                                                         //Attribute Value, 0x00: Boot Protocol Mode, 0x01: Report Protocol Mode
};


const Uint8 ATT_HDL_HIDS_REPORT_MSI_CLIENT_CHARACTERISTIC_CONFIGURATION_INIT[] =                    //Mandatory if the Battery Level characteristic properties supports notification
{
    0x00, (_HOGP_HDL_OFFSET_MOUSE_ + 0x02),                                                         //Handle
    GATT_DESCRIPTORS, GATT_DESCRIPTORS_CLIENT_CHARACTERISTIC_CONFIGURATION,                         //Attribute Type
    SIZE_OF_ATTRIBUTE_VALUE_2,
    0x00,                                                                                           //Attribute Value
    0x00                                                                                            //Attribute Value,
};


Uint8 att_HDL_HIDS_REPORT_MSI_CLIENT_CHARACTERISTIC_CONFIGURATION[] =
{
    //0x00, (_HOGP_HDL_OFFSET_MOUSE_+0x02),                                                         //Handle
    //GATT_DESCRIPTORS, GATT_DESCRIPTORS_CLIENT_CHARACTERISTIC_CONFIGURATION,                       //Attribute Type
    //SIZE_OF_ATTRIBUTE_VALUE_2,
    (
        //GATT_DESCRIPTORS_CLIENT_CHARACTERISTIC_CONFIGURATION_NOTIFICATION |
        //GATT_DESCRIPTORS_CLIENT_CHARACTERISTIC_CONFIGURATION_INDICATION |
        0x00
    ),                                                                                              //Attribute Value, R/W: Mandatory
    0x00                                                                                            //Attribute Value,
};


const Uint8 ATT_HDL_HIDS_REPORT_MSI_REPORT_REFERENCE[] =
{
    0x00, (_HOGP_HDL_OFFSET_MOUSE_ + 0x03),                                                         //Handle
    GATT_DESCRIPTORS, GATT_DESCRIPTORS_REPORT_REFERENCE,                                            //Attribute Type
    SIZE_OF_ATTRIBUTE_VALUE_2,
#ifdef _PROFILE_HOGP_EMC_1501_
    0x01,
#else
    0x02,                                                                                           //Attribute Value, Report ID
#endif
    0x01                                                                                            //Attribute Value, Report Type: Input (0x01)
};


const Uint8 ATT_HDL_HIDS_CHARACTERISTIC_REPORT_MSF[] =
{
    0x00, (_HOGP_HDL_OFFSET_MOUSE_ + 0x04),
    GATT_DECLARATIONS, GATT_DECLARATIONS_CHARACTERISTIC,                                            //Attribute Type
    SIZE_OF_ATTRIBUTE_VALUE_5,
    //0x11,
    (
        //Characteristic Properties
        //GATT_DECLARATIONS_PROPERTIES_BROADCAST |                                                  //Read: Mandatory
        GATT_DECLARATIONS_PROPERTIES_READ |                                                         //Notify: Optional
        //GATT_DECLARATIONS_PROPERTIES_WRITE_WITHOUT_RESPONSE |
        GATT_DECLARATIONS_PROPERTIES_WRITE |
        //GATT_DECLARATIONS_PROPERTIES_NOTIFY |
        //GATT_DECLARATIONS_PROPERTIES_INDICATE |
        //GATT_DECLARATIONS_PROPERTIES_AUTHENTICATED_SIGNED_WRITES |
        //GATT_DECLARATIONS_PROPERTIES_EXTENDED_PROPERTIES |
        0x00
    ),
    (_HOGP_HDL_OFFSET_MOUSE_ + 0x05), 0x00,
    GATT_SPEC_CHARACTERISTIC_REPORT, GATT_SPEC_CHARACTERISTIC                                       //Attribute Value
};


const Uint8 ATT_HDL_HIDS_REPORT_MSF_INIT[] =
{
    0x00, (_HOGP_HDL_OFFSET_MOUSE_ + 0x05),
    GATT_SPEC_CHARACTERISTIC, GATT_SPEC_CHARACTERISTIC_REPORT,                                      //Attribute Type
    SIZE_OF_ATTRIBUTE_VALUE_1,
    0x03                                                                                            //Attribute Value
};


Uint8 att_HDL_HIDS_REPORT_MSF[] =
{
    //0x00, (_HOGP_HDL_OFFSET_MOUSE_+0x05),                                                         //Handle
    //GATT_SPEC_CHARACTERISTIC, GATT_SPEC_CHARACTERISTIC_REPORT,                                    //Attribute Type
    //SIZE_OF_ATTRIBUTE_VALUE_1,
    0x03,                                                                                           //Attribute Value
};


const Uint8 ATT_HDL_HIDS_REPORT_MSF_REPORT_REFERENCE[] =
{
    0x00, (_HOGP_HDL_OFFSET_MOUSE_ + 0x06),
    GATT_DESCRIPTORS, GATT_DESCRIPTORS_REPORT_REFERENCE,                                            //Attribute Type
    SIZE_OF_ATTRIBUTE_VALUE_2,
#ifdef _PROFILE_HOGP_EMC_1501_
    0x01,
#else
    0x02,                                                                                           //Attribute Value, Report ID
#endif
    0x03                                                                                            //Attribute Value, Report Type: Feature (0x03)
};


const Uint8 ATT_HDL_HIDS_REPORT_MSF_USER_DESCRIPTION[] =
{
    0x00, (_HOGP_HDL_OFFSET_MOUSE_ + 0x07),
    GATT_DESCRIPTORS, GATT_DESCRIPTORS_CHARACTERISTIC_USER_DESCRIPTION,                             //Attribute Type
    (SIZE_OF_ATTRIBUTE_VALUE_0 + 7),
    0x46, 0x6F, 0x72, 0x20, 0x45, 0x4D, 0x43
};
#endif      //#ifdef _PROFILE_HOGP_MOUSE_


#ifdef _PROFILE_HOGP_KEYBOARD_
const Uint8 ATT_HDL_HIDS_CHARACTERISTIC_REPORT_KBI[] =
{
    0x00, _HOGP_HDL_OFFSET_KB_,                                                                     //Handle
    GATT_DECLARATIONS, GATT_DECLARATIONS_CHARACTERISTIC,                                            //Attribute Type
    SIZE_OF_ATTRIBUTE_VALUE_5,
    //0x11,
    (
        //Characteristic Properties
        //GATT_DECLARATIONS_PROPERTIES_BROADCAST |                                                  //Read: Mandatory
        GATT_DECLARATIONS_PROPERTIES_READ |                                                         //Notify: Optional
        //GATT_DECLARATIONS_PROPERTIES_WRITE_WITHOUT_RESPONSE |
        GATT_DECLARATIONS_PROPERTIES_WRITE |
        GATT_DECLARATIONS_PROPERTIES_NOTIFY |
        //GATT_DECLARATIONS_PROPERTIES_INDICATE |
        //GATT_DECLARATIONS_PROPERTIES_AUTHENTICATED_SIGNED_WRITES |
        //GATT_DECLARATIONS_PROPERTIES_EXTENDED_PROPERTIES |
        0x00
    ),
    (_HOGP_HDL_OFFSET_KB_ + 0x01), 0x00,
    GATT_SPEC_CHARACTERISTIC_REPORT, GATT_SPEC_CHARACTERISTIC                                       //Attribute Value
};


const Uint8 ATT_HDL_HIDS_REPORT_KBI_INIT[] =
{
    0x00, (_HOGP_HDL_OFFSET_KB_ + 0x01),                                                            //Handle
    GATT_SPEC_CHARACTERISTIC, GATT_SPEC_CHARACTERISTIC_REPORT,                                      //Attribute Type
    SIZE_OF_ATTRIBUTE_VALUE_8,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
};


Uint8 att_HDL_HIDS_REPORT_KBI[] =
{
    //0x00, (_HOGP_HDL_OFFSET_KB_+0x01),                                                            //Handle
    //GATT_SPEC_CHARACTERISTIC, GATT_SPEC_CHARACTERISTIC_REPORT,                                    //Attribute Type
    //SIZE_OF_ATTRIBUTE_VALUE_8,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
};


const Uint8 ATT_HDL_HIDS_REPORT_KBI_CLIENT_CHARACTERISTIC_CONFIGURATION_INIT[] =                    //Mandatory if the Battery Level characteristic properties supports notification
{
    0x00, (_HOGP_HDL_OFFSET_KB_ + 0x02),                                                            //Handle
    GATT_DESCRIPTORS, GATT_DESCRIPTORS_CLIENT_CHARACTERISTIC_CONFIGURATION,                         //Attribute Type
    SIZE_OF_ATTRIBUTE_VALUE_2,
    0x00,                                                                                           //Attribute Value
    0x00                                                                                            //Attribute Value,
};


Uint8 att_HDL_HIDS_REPORT_KBI_CLIENT_CHARACTERISTIC_CONFIGURATION[] =
{
    //0x00, (_HOGP_HDL_OFFSET_KB_+0x02),                                                            //Handle
    //GATT_DESCRIPTORS, GATT_DESCRIPTORS_CLIENT_CHARACTERISTIC_CONFIGURATION,                       //Attribute Type
    //SIZE_OF_ATTRIBUTE_VALUE_2,
    (
        //GATT_DESCRIPTORS_CLIENT_CHARACTERISTIC_CONFIGURATION_NOTIFICATION |
        //GATT_DESCRIPTORS_CLIENT_CHARACTERISTIC_CONFIGURATION_INDICATION |
        0x00
    ),                                                                                              //Attribute Value, R/W: Mandatory
    0x00                                                                                            //Attribute Value,
};


const Uint8 ATT_HDL_HIDS_REPORT_KBI_REPORT_REFERENCE[] =
{
    0x00, (_HOGP_HDL_OFFSET_KB_ + 0x03),                                                            //Handle
    GATT_DESCRIPTORS, GATT_DESCRIPTORS_REPORT_REFERENCE,                                            //Attribute Type
    SIZE_OF_ATTRIBUTE_VALUE_2,
    (0x00 + __PROFILE_HOGP_KEYBOARD_RPIDOFST_),                                                     //Attribute Value, Report ID
    0x01                                                                                            //Attribute Value, Report Type: Input (0x01)
};
#endif      //#ifdef _PROFILE_HOGP_KEYBOARD_


const Uint8 ATT_HDL_HIDS_CHARACTERISTIC_REPORT_81[] =
{
    0x00, _HOGP_HDL_OFFSET_81_,
    GATT_DECLARATIONS, GATT_DECLARATIONS_CHARACTERISTIC,
    SIZE_OF_ATTRIBUTE_VALUE_5,
    //0x11,
    (
        //GATT_DECLARATIONS_PROPERTIES_BROADCAST |
        GATT_DECLARATIONS_PROPERTIES_READ |
        //GATT_DECLARATIONS_PROPERTIES_WRITE_WITHOUT_RESPONSE |
        GATT_DECLARATIONS_PROPERTIES_WRITE |
        GATT_DECLARATIONS_PROPERTIES_NOTIFY |
        //GATT_DECLARATIONS_PROPERTIES_INDICATE |
        //GATT_DECLARATIONS_PROPERTIES_AUTHENTICATED_SIGNED_WRITES |
        //GATT_DECLARATIONS_PROPERTIES_EXTENDED_PROPERTIES |
        0x00
    ),
    (_HOGP_HDL_OFFSET_81_ + 0x01), 0x00,
    GATT_SPEC_CHARACTERISTIC_REPORT, GATT_SPEC_CHARACTERISTIC
};


const Uint8 ATT_HDL_HIDS_REPORT_81_INIT[] =
{
    0x00, (_HOGP_HDL_OFFSET_81_ + 0x01),
    GATT_SPEC_CHARACTERISTIC, GATT_SPEC_CHARACTERISTIC_REPORT,
    SIZE_OF_ATTRIBUTE_VALUE_20,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00,
};


Uint8 att_HDL_HIDS_REPORT_81[] =
{
    //0x00, (_HOGP_HDL_OFFSET_81_+0x01),
    //GATT_SPEC_CHARACTERISTIC, GATT_SPEC_CHARACTERISTIC_REPORT,
    //SIZE_OF_ATTRIBUTE_VALUE_20,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00,
};


const Uint8 ATT_HDL_HIDS_REPORT_81_CLIENT_CHARACTERISTIC_CONFIGURATION_INIT[] =
{
    0x00, (_HOGP_HDL_OFFSET_81_ + 0x02),
    GATT_DESCRIPTORS, GATT_DESCRIPTORS_CLIENT_CHARACTERISTIC_CONFIGURATION,
    SIZE_OF_ATTRIBUTE_VALUE_2,
    0x00,
    0x00
};


Uint8 att_HDL_HIDS_REPORT_81_CLIENT_CHARACTERISTIC_CONFIGURATION[] =
{
    //0x00, (_HOGP_HDL_OFFSET_81_+0x02),
    //GATT_DESCRIPTORS, GATT_DESCRIPTORS_CLIENT_CHARACTERISTIC_CONFIGURATION,
    //SIZE_OF_ATTRIBUTE_VALUE_2,
    (
        //GATT_DESCRIPTORS_CLIENT_CHARACTERISTIC_CONFIGURATION_NOTIFICATION |
        //GATT_DESCRIPTORS_CLIENT_CHARACTERISTIC_CONFIGURATION_INDICATION |
        0x00
    ),
    0x00
};


const Uint8 ATT_HDL_HIDS_REPORT_81_REPORT_REFERENCE[] =
{
    0x00, (_HOGP_HDL_OFFSET_81_ + 0x03),
    GATT_DESCRIPTORS, GATT_DESCRIPTORS_REPORT_REFERENCE,
    SIZE_OF_ATTRIBUTE_VALUE_2,
//#ifdef _PROFILE_HOGP_EMC_1501_
//    0x5D,
//#else
    0x81,
//#endif
    0x01
};


const Uint8 ATT_HDL_HIDS_CHARACTERISTIC_REPORT_82[] =
{
    0x00, _HOGP_HDL_OFFSET_82_,
    GATT_DECLARATIONS, GATT_DECLARATIONS_CHARACTERISTIC,
    SIZE_OF_ATTRIBUTE_VALUE_5,
    //0x11,
    (
        //GATT_DECLARATIONS_PROPERTIES_BROADCAST |
        GATT_DECLARATIONS_PROPERTIES_READ |
        //GATT_DECLARATIONS_PROPERTIES_WRITE_WITHOUT_RESPONSE |
        GATT_DECLARATIONS_PROPERTIES_WRITE |
        GATT_DECLARATIONS_PROPERTIES_NOTIFY |
        //GATT_DECLARATIONS_PROPERTIES_INDICATE |
        //GATT_DECLARATIONS_PROPERTIES_AUTHENTICATED_SIGNED_WRITES |
        //GATT_DECLARATIONS_PROPERTIES_EXTENDED_PROPERTIES |
        0x00
    ),
    (_HOGP_HDL_OFFSET_82_ + 0x01), 0x00,
    GATT_SPEC_CHARACTERISTIC_REPORT, GATT_SPEC_CHARACTERISTIC
};


const Uint8 ATT_HDL_HIDS_REPORT_82_INIT[] =
{
    0x00, (_HOGP_HDL_OFFSET_82_ + 0x01),
    GATT_SPEC_CHARACTERISTIC, GATT_SPEC_CHARACTERISTIC_REPORT,
    SIZE_OF_ATTRIBUTE_VALUE_20,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00,
};


Uint8 att_HDL_HIDS_REPORT_82[] =
{
    //0x00, (_HOGP_HDL_OFFSET_82_+0x01),
    //GATT_SPEC_CHARACTERISTIC, GATT_SPEC_CHARACTERISTIC_REPORT,
    //SIZE_OF_ATTRIBUTE_VALUE_20,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00,
};


const Uint8 ATT_HDL_HIDS_REPORT_82_CLIENT_CHARACTERISTIC_CONFIGURATION_INIT[] =
{
    0x00, (_HOGP_HDL_OFFSET_82_ + 0x02),
    GATT_DESCRIPTORS, GATT_DESCRIPTORS_CLIENT_CHARACTERISTIC_CONFIGURATION,
    SIZE_OF_ATTRIBUTE_VALUE_2,
    0x00,
    0x00
};


Uint8 att_HDL_HIDS_REPORT_82_CLIENT_CHARACTERISTIC_CONFIGURATION[] =
{
    //0x00, (_HOGP_HDL_OFFSET_82_+0x02),
    //GATT_DESCRIPTORS, GATT_DESCRIPTORS_CLIENT_CHARACTERISTIC_CONFIGURATION,
    //SIZE_OF_ATTRIBUTE_VALUE_2,
    (
        //GATT_DESCRIPTORS_CLIENT_CHARACTERISTIC_CONFIGURATION_NOTIFICATION |
        //GATT_DESCRIPTORS_CLIENT_CHARACTERISTIC_CONFIGURATION_INDICATION |
        0x00
    ),
    0x00
};


const Uint8 ATT_HDL_HIDS_REPORT_82_REPORT_REFERENCE[] =
{
    0x00, (_HOGP_HDL_OFFSET_82_ + 0x03),
    GATT_DESCRIPTORS, GATT_DESCRIPTORS_REPORT_REFERENCE,
    SIZE_OF_ATTRIBUTE_VALUE_2,
    0x82,
    0x01
};


const Uint8 ATT_HDL_HIDS_CHARACTERISTIC_REPORT_5D[] =
{
    0x00, _HOGP_HDL_OFFSET_5D_,
    GATT_DECLARATIONS, GATT_DECLARATIONS_CHARACTERISTIC,
    SIZE_OF_ATTRIBUTE_VALUE_5,
    //0x11,
    (
        //GATT_DECLARATIONS_PROPERTIES_BROADCAST |
        GATT_DECLARATIONS_PROPERTIES_READ |
        //GATT_DECLARATIONS_PROPERTIES_WRITE_WITHOUT_RESPONSE |
        GATT_DECLARATIONS_PROPERTIES_WRITE |
        GATT_DECLARATIONS_PROPERTIES_NOTIFY |
        //GATT_DECLARATIONS_PROPERTIES_INDICATE |
        //GATT_DECLARATIONS_PROPERTIES_AUTHENTICATED_SIGNED_WRITES |
        //GATT_DECLARATIONS_PROPERTIES_EXTENDED_PROPERTIES |
        0x00
    ),
    (_HOGP_HDL_OFFSET_5D_ + 0x01), 0x00,
    GATT_SPEC_CHARACTERISTIC_REPORT, GATT_SPEC_CHARACTERISTIC
};


const Uint8 ATT_HDL_HIDS_REPORT_5D_INIT[] =
{
    0x00, (_HOGP_HDL_OFFSET_5D_ + 0x01),
    GATT_SPEC_CHARACTERISTIC, GATT_SPEC_CHARACTERISTIC_REPORT,
    SIZE_OF_ATTRIBUTE_VALUE_20,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00,
};


Uint8 att_HDL_HIDS_REPORT_5D[] =
{
    //0x00, (_HOGP_HDL_OFFSET_5D_+0x01),
    //GATT_SPEC_CHARACTERISTIC, GATT_SPEC_CHARACTERISTIC_REPORT,
    //SIZE_OF_ATTRIBUTE_VALUE_20,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00,
};


const Uint8 ATT_HDL_HIDS_REPORT_5D_CLIENT_CHARACTERISTIC_CONFIGURATION_INIT[] =
{
    0x00, (_HOGP_HDL_OFFSET_5D_ + 0x02),
    GATT_DESCRIPTORS, GATT_DESCRIPTORS_CLIENT_CHARACTERISTIC_CONFIGURATION,
    SIZE_OF_ATTRIBUTE_VALUE_2,
    0x00,
    0x00
};


Uint8 att_HDL_HIDS_REPORT_5D_CLIENT_CHARACTERISTIC_CONFIGURATION[] =
{
    //0x00, (_HOGP_HDL_OFFSET_5D_+0x02),
    //GATT_DESCRIPTORS, GATT_DESCRIPTORS_CLIENT_CHARACTERISTIC_CONFIGURATION,
    //SIZE_OF_ATTRIBUTE_VALUE_2,
    (
        //GATT_DESCRIPTORS_CLIENT_CHARACTERISTIC_CONFIGURATION_NOTIFICATION |
        //GATT_DESCRIPTORS_CLIENT_CHARACTERISTIC_CONFIGURATION_INDICATION |
        0x00
    ),
    0x00
};


const Uint8 ATT_HDL_HIDS_REPORT_5D_REPORT_REFERENCE[] =
{
    0x00, (_HOGP_HDL_OFFSET_5D_ + 0x03),
    GATT_DESCRIPTORS, GATT_DESCRIPTORS_REPORT_REFERENCE,
    SIZE_OF_ATTRIBUTE_VALUE_2,
    0x5D,
    0x01
};


const Uint8 ATT_HDL_HIDS_CHARACTERISTIC_REPORT_05[] =
{
    0x00, _HOGP_HDL_OFFSET_05_,
    GATT_DECLARATIONS, GATT_DECLARATIONS_CHARACTERISTIC,
    SIZE_OF_ATTRIBUTE_VALUE_5,
    //0x11,
    (
        //GATT_DECLARATIONS_PROPERTIES_BROADCAST |
        GATT_DECLARATIONS_PROPERTIES_READ |
        //GATT_DECLARATIONS_PROPERTIES_WRITE_WITHOUT_RESPONSE |
        GATT_DECLARATIONS_PROPERTIES_WRITE |
        //GATT_DECLARATIONS_PROPERTIES_NOTIFY |
        //GATT_DECLARATIONS_PROPERTIES_INDICATE |
        //GATT_DECLARATIONS_PROPERTIES_AUTHENTICATED_SIGNED_WRITES |
        //GATT_DECLARATIONS_PROPERTIES_EXTENDED_PROPERTIES |
        0x00
    ),
    (_HOGP_HDL_OFFSET_05_ + 0x01), 0x00,
    GATT_SPEC_CHARACTERISTIC_REPORT, GATT_SPEC_CHARACTERISTIC
};


const Uint8 ATT_HDL_HIDS_REPORT_05_INIT[] =
{
    0x00, (_HOGP_HDL_OFFSET_05_ + 0x01),
    GATT_SPEC_CHARACTERISTIC, GATT_SPEC_CHARACTERISTIC_REPORT,
#ifdef _PROFILE_HOGP_EMC_1501_
    SIZE_OF_ATTRIBUTE_VALUE_20,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00,
#else
    SIZE_OF_ATTRIBUTE_VALUE_7,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
#endif
};


Uint8 att_HDL_HIDS_REPORT_05[] =
{
    //0x00, (_HOGP_HDL_OFFSET_05_+0x01),
    //GATT_SPEC_CHARACTERISTIC, GATT_SPEC_CHARACTERISTIC_REPORT,
#ifdef _PROFILE_HOGP_EMC_1501_
    //SIZE_OF_ATTRIBUTE_VALUE_20,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00,
#else
    //SIZE_OF_ATTRIBUTE_VALUE_7,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
#endif
};


/*
const Uint8 ATT_HDL_HIDS_REPORT_05_CLIENT_CHARACTERISTIC_CONFIGURATION_INIT[] =
{
    0x00, (_HOGP_HDL_OFFSET_05_+0x02),
    GATT_DESCRIPTORS, GATT_DESCRIPTORS_CLIENT_CHARACTERISTIC_CONFIGURATION,
    SIZE_OF_ATTRIBUTE_VALUE_2,
    0x00,
    0x00
};


Uint8 att_HDL_HIDS_REPORT_05_CLIENT_CHARACTERISTIC_CONFIGURATION[] =
{
    //0x00, (_HOGP_HDL_OFFSET_05_+0x02),
    //GATT_DESCRIPTORS, GATT_DESCRIPTORS_CLIENT_CHARACTERISTIC_CONFIGURATION,
    //SIZE_OF_ATTRIBUTE_VALUE_2,
    {
        //GATT_DESCRIPTORS_CLIENT_CHARACTERISTIC_CONFIGURATION_NOTIFICATION |
        //GATT_DESCRIPTORS_CLIENT_CHARACTERISTIC_CONFIGURATION_INDICATION |
        0x00
    },
    0x00
};
*/


const Uint8 ATT_HDL_HIDS_REPORT_05_REPORT_REFERENCE[] =
{
    0x00, (_HOGP_HDL_OFFSET_05_ + 0x03 - 0x01),
    GATT_DESCRIPTORS, GATT_DESCRIPTORS_REPORT_REFERENCE,
    SIZE_OF_ATTRIBUTE_VALUE_2,
#ifdef _PROFILE_HOGP_EMC_1501_
    0x0B,
#else
    0x05,
#endif
    0x03
};


const Uint8 ATT_HDL_HIDS_CHARACTERISTIC_REPORT_08[] =
{
    0x00, _HOGP_HDL_OFFSET_08_,
    GATT_DECLARATIONS, GATT_DECLARATIONS_CHARACTERISTIC,
    SIZE_OF_ATTRIBUTE_VALUE_5,
    //0x11,
    (
        //GATT_DECLARATIONS_PROPERTIES_BROADCAST |
        GATT_DECLARATIONS_PROPERTIES_READ |
        //GATT_DECLARATIONS_PROPERTIES_WRITE_WITHOUT_RESPONSE |
        GATT_DECLARATIONS_PROPERTIES_WRITE |
        //GATT_DECLARATIONS_PROPERTIES_NOTIFY |
        //GATT_DECLARATIONS_PROPERTIES_INDICATE |
        //GATT_DECLARATIONS_PROPERTIES_AUTHENTICATED_SIGNED_WRITES |
        //GATT_DECLARATIONS_PROPERTIES_EXTENDED_PROPERTIES |
        0x00
    ),
    (_HOGP_HDL_OFFSET_08_ + 0x01), 0x00,
    GATT_SPEC_CHARACTERISTIC_REPORT, GATT_SPEC_CHARACTERISTIC
};


const Uint8 ATT_HDL_HIDS_REPORT_08_INIT[] =
{
    0x00, (_HOGP_HDL_OFFSET_08_ + 0x01),
    GATT_SPEC_CHARACTERISTIC, GATT_SPEC_CHARACTERISTIC_REPORT,
#ifdef _PROFILE_HOGP_EMC_1501_
    SIZE_OF_ATTRIBUTE_VALUE_20,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00,
#else
    SIZE_OF_ATTRIBUTE_VALUE_7,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
#endif
};


Uint8 att_HDL_HIDS_REPORT_08[] =
{
    //0x00, (_HOGP_HDL_OFFSET_08_+0x01),
    //GATT_SPEC_CHARACTERISTIC, GATT_SPEC_CHARACTERISTIC_REPORT,
#ifdef _PROFILE_HOGP_EMC_1501_
    //SIZE_OF_ATTRIBUTE_VALUE_20,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00,
#else
    //SIZE_OF_ATTRIBUTE_VALUE_7,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
#endif
};


/*
const Uint8 ATT_HDL_HIDS_REPORT_08_CLIENT_CHARACTERISTIC_CONFIGURATION_INIT[] =
{
    0x00, (_HOGP_HDL_OFFSET_08_+0x02),
    GATT_DESCRIPTORS, GATT_DESCRIPTORS_CLIENT_CHARACTERISTIC_CONFIGURATION,
    SIZE_OF_ATTRIBUTE_VALUE_2,
    0x00,
    0x00
};


Uint8 att_HDL_HIDS_REPORT_08_CLIENT_CHARACTERISTIC_CONFIGURATION[] =
{
    //0x00, (_HOGP_HDL_OFFSET_08_+0x02),
    //GATT_DESCRIPTORS, GATT_DESCRIPTORS_CLIENT_CHARACTERISTIC_CONFIGURATION,
    //SIZE_OF_ATTRIBUTE_VALUE_2,
    {
        //GATT_DESCRIPTORS_CLIENT_CHARACTERISTIC_CONFIGURATION_NOTIFICATION |
        //GATT_DESCRIPTORS_CLIENT_CHARACTERISTIC_CONFIGURATION_INDICATION |
        0x00
    },
    0x00
};
*/


const Uint8 ATT_HDL_HIDS_REPORT_08_REPORT_REFERENCE[] =
{
    0x00, (_HOGP_HDL_OFFSET_08_ + 0x03 - 0x01),
    GATT_DESCRIPTORS, GATT_DESCRIPTORS_REPORT_REFERENCE,
    SIZE_OF_ATTRIBUTE_VALUE_2,
#ifdef _PROFILE_HOGP_EMC_1501_
    0x0C,
#else
    0x08,
#endif
    0x03
};


const Uint8 ATT_HDL_HIDS_CHARACTERISTIC_REPORT_09[] =
{
    0x00, _HOGP_HDL_OFFSET_09_,
    GATT_DECLARATIONS, GATT_DECLARATIONS_CHARACTERISTIC,
    SIZE_OF_ATTRIBUTE_VALUE_5,
    //0x11,
    (
        //GATT_DECLARATIONS_PROPERTIES_BROADCAST |
        GATT_DECLARATIONS_PROPERTIES_READ |
        //GATT_DECLARATIONS_PROPERTIES_WRITE_WITHOUT_RESPONSE |
        GATT_DECLARATIONS_PROPERTIES_WRITE |
        //GATT_DECLARATIONS_PROPERTIES_NOTIFY |
        //GATT_DECLARATIONS_PROPERTIES_INDICATE |
        //GATT_DECLARATIONS_PROPERTIES_AUTHENTICATED_SIGNED_WRITES |
        //GATT_DECLARATIONS_PROPERTIES_EXTENDED_PROPERTIES |
        0x00
    ),
    (_HOGP_HDL_OFFSET_09_ + 0x01), 0x00,
    GATT_SPEC_CHARACTERISTIC_REPORT, GATT_SPEC_CHARACTERISTIC
};


const Uint8 ATT_HDL_HIDS_REPORT_09_INIT[] =
{
    0x00, (_HOGP_HDL_OFFSET_09_ + 0x01),
    GATT_SPEC_CHARACTERISTIC, GATT_SPEC_CHARACTERISTIC_REPORT,
#ifdef _PROFILE_HOGP_EMC_1501_
    SIZE_OF_ATTRIBUTE_VALUE_20,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00,
#else
    SIZE_OF_ATTRIBUTE_VALUE_7,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
#endif
};


Uint8 att_HDL_HIDS_REPORT_09[] =
{
    //0x00, (_HOGP_HDL_OFFSET_09_+0x01),
    //GATT_SPEC_CHARACTERISTIC, GATT_SPEC_CHARACTERISTIC_REPORT,
#ifdef _PROFILE_HOGP_EMC_1501_
    //SIZE_OF_ATTRIBUTE_VALUE_20,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00,
#else
    //SIZE_OF_ATTRIBUTE_VALUE_7,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
#endif
};


const Uint8 ATT_HDL_HIDS_REPORT_09_REPORT_REFERENCE[] =
{
    0x00, (_HOGP_HDL_OFFSET_09_ + 0x03 - 0x01),
    GATT_DESCRIPTORS, GATT_DESCRIPTORS_REPORT_REFERENCE,
    SIZE_OF_ATTRIBUTE_VALUE_2,
#ifdef _PROFILE_HOGP_EMC_1501_
    0x0D,
#else
    0x09,
#endif
    0x03
};
#endif      //#ifndef _PROFILE_HOGP_EMC_


const Uint8 ATT_HDL_HIDS_CHARACTERISTIC_PROTOCOL_MODE[] =
{
    0x00, _HOGP_HDL_OFFSET_RPT_REF_,                                                                //Handle
    GATT_DECLARATIONS, GATT_DECLARATIONS_CHARACTERISTIC,                                            //Attribute Type
    SIZE_OF_ATTRIBUTE_VALUE_5,
    //0x11,
    (
        //Characteristic Properties
        //GATT_DECLARATIONS_PROPERTIES_BROADCAST |                                                  //Read: Mandatory
        GATT_DECLARATIONS_PROPERTIES_READ |                                                         //Notify: Optional
        GATT_DECLARATIONS_PROPERTIES_WRITE_WITHOUT_RESPONSE |
        //GATT_DECLARATIONS_PROPERTIES_WRITE |
        //GATT_DECLARATIONS_PROPERTIES_NOTIFY |
        //GATT_DECLARATIONS_PROPERTIES_INDICATE |
        //GATT_DECLARATIONS_PROPERTIES_AUTHENTICATED_SIGNED_WRITES |
        //GATT_DECLARATIONS_PROPERTIES_EXTENDED_PROPERTIES |
        0x00
    ),
    (_HOGP_HDL_OFFSET_RPT_REF_ + 0x01), 0x00,
    GATT_SPEC_CHARACTERISTIC_PROTOCOL_MODE, GATT_SPEC_CHARACTERISTIC                                //Attribute Value
};


const Uint8 ATT_HDL_HIDS_PROTOCOL_MODE_INIT[] =
{
    0x00, (_HOGP_HDL_OFFSET_RPT_REF_ + 0x01),                                                       //Handle
    GATT_SPEC_CHARACTERISTIC, GATT_SPEC_CHARACTERISTIC_PROTOCOL_MODE,                               //Attribute Type
    SIZE_OF_ATTRIBUTE_VALUE_1,
    1,                                                                                              //Attribute Value, 0x00: Boot Protocol Mode, 0x01: Report Protocol Mode
};


Uint8 att_HDL_HIDS_PROTOCOL_MODE[] =
{
    //0x00, (_HOGP_HDL_OFFSET_RPT_REF_+0x01),                                                       //Handle
    //GATT_SPEC_CHARACTERISTIC, GATT_SPEC_CHARACTERISTIC_PROTOCOL_MODE,                             //Attribute Type
    //SIZE_OF_ATTRIBUTE_VALUE_1,
    1,                                                                                              //Attribute Value, 0x00: Boot Protocol Mode, 0x01: Report Protocol Mode
};



#ifdef _PROFILE_HOGP_BOOT_
#ifdef _PROFILE_HOGP_KEYBOARD_
const Uint8 ATT_HDL_HIDS_CHARACTERISTIC_BOOT_KEYBOARD_INPUT_REPORT[] =
{
    0x00, _HOGP_HDL_OFFSET_BOOT_KB_,                                                                //Handle
    GATT_DECLARATIONS, GATT_DECLARATIONS_CHARACTERISTIC,                                            //Attribute Type
    SIZE_OF_ATTRIBUTE_VALUE_5,
    //0x11,
    (
        //Characteristic Properties
        //GATT_DECLARATIONS_PROPERTIES_BROADCAST |                                                  //Read: Mandatory
        GATT_DECLARATIONS_PROPERTIES_READ |                                                         //Notify: Optional
        //GATT_DECLARATIONS_PROPERTIES_WRITE_WITHOUT_RESPONSE |
        GATT_DECLARATIONS_PROPERTIES_WRITE |
        GATT_DECLARATIONS_PROPERTIES_NOTIFY |
        //GATT_DECLARATIONS_PROPERTIES_INDICATE |
        //GATT_DECLARATIONS_PROPERTIES_AUTHENTICATED_SIGNED_WRITES |
        //GATT_DECLARATIONS_PROPERTIES_EXTENDED_PROPERTIES |
        0x00
    ),
    (_HOGP_HDL_OFFSET_BOOT_KB_ + 0x01), 0x00,
    GATT_SPEC_CHARACTERISTIC_BOOT_KEYBOARD_INPUT_REPORT, GATT_SPEC_CHARACTERISTIC                   //Attribute Value
};


const Uint8 ATT_HDL_HIDS_BOOT_KEYBOARD_INPUT_REPORT_INIT[] =
{
    0x00, (_HOGP_HDL_OFFSET_BOOT_KB_ + 0x01),                                                       //Handle
    GATT_SPEC_CHARACTERISTIC, GATT_SPEC_CHARACTERISTIC_BOOT_KEYBOARD_INPUT_REPORT,                  //Attribute Type
    SIZE_OF_ATTRIBUTE_VALUE_9,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00                                            //Attribute Value, 0x00: Boot Protocol Mode, 0x01: Report Protocol Mode
};


Uint8 att_HDL_HIDS_BOOT_KEYBOARD_INPUT_REPORT[] =
{
    //0x00, (_HOGP_HDL_OFFSET_BOOT_KB_+0x01),                                                       //Handle
    //GATT_SPEC_CHARACTERISTIC, GATT_SPEC_CHARACTERISTIC_REPORT,                                    //Attribute Type
    //SIZE_OF_ATTRIBUTE_VALUE_9,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00                                            //Attribute Value, 0x00: Boot Protocol Mode, 0x01: Report Protocol Mode
};


const Uint8 ATT_HDL_HIDS_BOOT_KEYBOARD_INPUT_REPORT_CLIENT_CHARACTERISTIC_CONFIGURATION_INIT[] =    //Mandatory if the Battery Level characteristic properties supports notification
{
    0x00, (_HOGP_HDL_OFFSET_BOOT_KB_ + 0x02),                                                       //Handle
    GATT_DESCRIPTORS, GATT_DESCRIPTORS_CLIENT_CHARACTERISTIC_CONFIGURATION,                         //Attribute Type
    SIZE_OF_ATTRIBUTE_VALUE_2,
    0x00,                                                                                           //Attribute Value
    0x00                                                                                            //Attribute Value,
};


Uint8 att_HDL_HIDS_BOOT_KEYBOARD_INPUT_REPORT_CLIENT_CHARACTERISTIC_CONFIGURATION[] =
{
    //0x00, (_HOGP_HDL_OFFSET_BOOT_KB_+0x02),                                                       //Handle
    //GATT_DESCRIPTORS, GATT_DESCRIPTORS_CLIENT_CHARACTERISTIC_CONFIGURATION,                       //Attribute Type
    //SIZE_OF_ATTRIBUTE_VALUE_2,
    (
        //GATT_DESCRIPTORS_CLIENT_CHARACTERISTIC_CONFIGURATION_NOTIFICATION |
        //GATT_DESCRIPTORS_CLIENT_CHARACTERISTIC_CONFIGURATION_INDICATION |
        0x00
    ),                                                                                              //Attribute Value, R/W: Mandatory
    0x00                                                                                            //Attribute Value,
};


const Uint8 ATT_HDL_HIDS_CHARACTERISTIC_BOOT_KEYBOARD_OUTPUT_REPORT[] =
{
    0x00, (_HOGP_HDL_OFFSET_BOOT_KB_ + 0x03),                                                       //Handle
    GATT_DECLARATIONS, GATT_DECLARATIONS_CHARACTERISTIC,                                            //Attribute Type
    SIZE_OF_ATTRIBUTE_VALUE_5,
    //0x11,
    (
        //Characteristic Properties
        //GATT_DECLARATIONS_PROPERTIES_BROADCAST |                                                  //Read: Mandatory
        GATT_DECLARATIONS_PROPERTIES_READ |                                                         //Notify: Optional
        GATT_DECLARATIONS_PROPERTIES_WRITE_WITHOUT_RESPONSE |
        GATT_DECLARATIONS_PROPERTIES_WRITE |
        //GATT_DECLARATIONS_PROPERTIES_NOTIFY |
        //GATT_DECLARATIONS_PROPERTIES_INDICATE |
        //GATT_DECLARATIONS_PROPERTIES_AUTHENTICATED_SIGNED_WRITES |
        //GATT_DECLARATIONS_PROPERTIES_EXTENDED_PROPERTIES |
        0x00
    ),
    (_HOGP_HDL_OFFSET_BOOT_KB_ + 0x04), 0x00,
    GATT_SPEC_CHARACTERISTIC_BOOT_KEYBOARD_OUTPUT_REPORT, GATT_SPEC_CHARACTERISTIC                  //Attribute Value
};


const Uint8 ATT_HDL_HIDS_BOOT_KEYBOARD_OUTPUT_REPORT_INIT[] =
{
    0x00, (_HOGP_HDL_OFFSET_BOOT_KB_ + 0x04),                                                       //Handle
    GATT_SPEC_CHARACTERISTIC, GATT_SPEC_CHARACTERISTIC_BOOT_KEYBOARD_OUTPUT_REPORT,                 //Attribute Type
    SIZE_OF_ATTRIBUTE_VALUE_1,
    0x00,                                                                                           //Attribute Value,
};


Uint8 att_HDL_HIDS_BOOT_KEYBOARD_OUTPUT_REPORT[] =
{
    //0x00, (_HOGP_HDL_OFFSET_BOOT_KB_+0x04),                                                       //Handle
    //GATT_SPEC_CHARACTERISTIC, GATT_SPEC_CHARACTERISTIC_REPORT,                                    //Attribute Type
    //SIZE_OF_ATTRIBUTE_VALUE_1,
    0x00,                                                                                           //Attribute Value,
};
#endif //#ifdef _PROFILE_HOGP_KEYBOARD_


#ifdef _PROFILE_HOGP_MOUSE_
const Uint8 ATT_HDL_HIDS_CHARACTERISTIC_BOOT_MOUSE_INPUT_REPORT[] =
{
    0x00, _HOGP_HDL_OFFSET_BOOT_MS_,                                                                //Handle
    GATT_DECLARATIONS, GATT_DECLARATIONS_CHARACTERISTIC,                                            //Attribute Type
    SIZE_OF_ATTRIBUTE_VALUE_5,
    //0x11,
    (
        //Characteristic Properties
        //GATT_DECLARATIONS_PROPERTIES_BROADCAST |                                                  //Read: Mandatory
        GATT_DECLARATIONS_PROPERTIES_READ |                                                         //Notify: Optional
        //GATT_DECLARATIONS_PROPERTIES_WRITE_WITHOUT_RESPONSE |
        GATT_DECLARATIONS_PROPERTIES_WRITE |
        GATT_DECLARATIONS_PROPERTIES_NOTIFY |
        //GATT_DECLARATIONS_PROPERTIES_INDICATE |
        //GATT_DECLARATIONS_PROPERTIES_AUTHENTICATED_SIGNED_WRITES |
        //GATT_DECLARATIONS_PROPERTIES_EXTENDED_PROPERTIES |
        0x00
    ),
    (_HOGP_HDL_OFFSET_BOOT_MS_ + 0x01), 0x00,
    GATT_SPEC_CHARACTERISTIC_BOOT_MOUSE_INPUT_REPORT, GATT_SPEC_CHARACTERISTIC                      //Attribute Value
};


const Uint8 ATT_HDL_HIDS_BOOT_MOUSE_INPUT_REPORT_INIT[] =
{
    0x00, (_HOGP_HDL_OFFSET_BOOT_MS_ + 0x01),                                                       //Handle
    GATT_SPEC_CHARACTERISTIC, GATT_SPEC_CHARACTERISTIC_BOOT_MOUSE_INPUT_REPORT,                     //Attribute Type
    SIZE_OF_ATTRIBUTE_VALUE_6,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,                                                             //Attribute Value, 0x00: Boot Protocol Mode, 0x01: Report Protocol Mode
};


Uint8 att_HDL_HIDS_BOOT_MOUSE_INPUT_REPORT[] =
{
    //0x00, (_HOGP_HDL_OFFSET_BOOT_MS_+0x01),                                                       //Handle
    //GATT_SPEC_CHARACTERISTIC, GATT_SPEC_CHARACTERISTIC_REPORT,                                    //Attribute Type
    //SIZE_OF_ATTRIBUTE_VALUE_6,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,                                                             //Attribute Value, 0x00: Boot Protocol Mode, 0x01: Report Protocol Mode
};


const Uint8 ATT_HDL_HIDS_BOOT_MOUSE_INPUT_REPORT_CLIENT_CHARACTERISTIC_CONFIGURATION_INIT[] =       //Mandatory if the Battery Level characteristic properties supports notification
{
    0x00, (_HOGP_HDL_OFFSET_BOOT_MS_ + 0x02),                                                       //Handle
    GATT_DESCRIPTORS, GATT_DESCRIPTORS_CLIENT_CHARACTERISTIC_CONFIGURATION,                         //Attribute Type
    SIZE_OF_ATTRIBUTE_VALUE_2,
    0x00,                                                                                           //Attribute Value
    0x00                                                                                            //Attribute Value,
};


Uint8 att_HDL_HIDS_BOOT_MOUSE_INPUT_REPORT_CLIENT_CHARACTERISTIC_CONFIGURATION[] =
{
    //0x00, (_HOGP_HDL_OFFSET_BOOT_MS_+0x02),                                                       //Handle
    //GATT_DESCRIPTORS, GATT_DESCRIPTORS_CLIENT_CHARACTERISTIC_CONFIGURATION,                       //Attribute Type
    //SIZE_OF_ATTRIBUTE_VALUE_2,
    (
        //GATT_DESCRIPTORS_CLIENT_CHARACTERISTIC_CONFIGURATION_NOTIFICATION |
        //GATT_DESCRIPTORS_CLIENT_CHARACTERISTIC_CONFIGURATION_INDICATION |
        0x00
    ),                                                                                              //Attribute Value, R/W: Mandatory
    0x00                                                                                            //Attribute Value,
};
#endif      //#ifdef _PROFILE_HOGP_MOUSE_
#endif      //#ifdef _PROFILE_HOGP_BOOT_
#endif


#ifdef _PROFILE_SCPP_
const Uint8 ATT_HDL_SCPS_PRIMARY_SERVICE[] =
{
    0x00, _SCPS_HDL_OFFSET_BASE_,                                                                   //Handle
    GATT_DECLARATIONS, GATT_DECLARATIONS_PRIMARY_SERVICE,                                           //Attribute Type
    SIZE_OF_ATTRIBUTE_VALUE_2,
    GATT_SPEC_SERVICES_SCAN_PARAMETERS, GATT_SPEC_SERVICES                                          //Attribute Value
};


const Uint8 ATT_HDL_SCPS_CHARACTERISTIC_SCAN_INTERVAL_WINDOW[] =
{
    0x00, (_SCPS_HDL_OFFSET_BASE_ + 1),                                                             //Handle
    GATT_DECLARATIONS, GATT_DECLARATIONS_CHARACTERISTIC,                                            //Attribute Type
    SIZE_OF_ATTRIBUTE_VALUE_5,
    //0x11,
    (
        //Characteristic Properties
        //GATT_DECLARATIONS_PROPERTIES_BROADCAST |                                                  //Read: Mandatory
        //GATT_DECLARATIONS_PROPERTIES_READ |                                                       //Notify: Optional
        GATT_DECLARATIONS_PROPERTIES_WRITE_WITHOUT_RESPONSE |
        //GATT_DECLARATIONS_PROPERTIES_WRITE |
        //GATT_DECLARATIONS_PROPERTIES_NOTIFY |
        //GATT_DECLARATIONS_PROPERTIES_INDICATE |
        //GATT_DECLARATIONS_PROPERTIES_AUTHENTICATED_SIGNED_WRITES |
        //GATT_DECLARATIONS_PROPERTIES_EXTENDED_PROPERTIES |
        0x00
    ),
    (_SCPS_HDL_OFFSET_BASE_ + 2), 0x00,
    GATT_SPEC_CHARACTERISTIC_SCAN_INTERVAL_WINDOW, GATT_SPEC_CHARACTERISTIC                         //Attribute Value
};


const Uint8 ATT_HDL_SCPS_SCAN_INTERVAL_WINDOW_INIT[] =
{
    0x00, (_SCPS_HDL_OFFSET_BASE_ + 2),                                                             //Handle
    GATT_SPEC_CHARACTERISTIC, GATT_SPEC_CHARACTERISTIC_SCAN_INTERVAL_WINDOW,                        //Attribute Type
    SIZE_OF_ATTRIBUTE_VALUE_4,
    0x00, 0x00, 0x00, 0x00                                                                          //Attribute Value
};


Uint8 att_HDL_SCPS_SCAN_INTERVAL_WINDOW[] =
{
    //0x00, (_SCPS_HDL_OFFSET_BASE_+2),                                                             //Handle
    //GATT_SPEC_CHARACTERISTIC, GATT_SPEC_CHARACTERISTIC_SCAN_INTERVAL_WINDOW,                      //Attribute Type
    //SIZE_OF_ATTRIBUTE_VALUE_4,
    0x00, 0x00, 0x00, 0x00                                                                          //Attribute Value
};


const Uint8 ATT_HDL_SCPS_CHARACTERISTIC_SCAN_REFRESH[] =
{
    0x00, (_SCPS_HDL_OFFSET_BASE_ + 3),                                                             //Handle
    GATT_DECLARATIONS, GATT_DECLARATIONS_CHARACTERISTIC,                                            //Attribute Type
    SIZE_OF_ATTRIBUTE_VALUE_5,
    //0x11,
    (
        //Characteristic Properties
        //GATT_DECLARATIONS_PROPERTIES_BROADCAST |                                                  //Read: Mandatory
        //GATT_DECLARATIONS_PROPERTIES_READ |                                                       //Notify: Optional
        //GATT_DECLARATIONS_PROPERTIES_WRITE_WITHOUT_RESPONSE |
        //GATT_DECLARATIONS_PROPERTIES_WRITE |
        GATT_DECLARATIONS_PROPERTIES_NOTIFY |
        //GATT_DECLARATIONS_PROPERTIES_INDICATE |
        //GATT_DECLARATIONS_PROPERTIES_AUTHENTICATED_SIGNED_WRITES |
        //GATT_DECLARATIONS_PROPERTIES_EXTENDED_PROPERTIES |
        0x00
    ),
    (_SCPS_HDL_OFFSET_BASE_ + 4), 0x00,
    GATT_SPEC_CHARACTERISTIC_SCAN_REFRESH, GATT_SPEC_CHARACTERISTIC                                 //Attribute Value
};


const Uint8 ATT_HDL_SCPS_SCAN_REFRESH_INIT[] =
{
    0x00, (_SCPS_HDL_OFFSET_BASE_ + 4),                                                             //Handle
    GATT_SPEC_CHARACTERISTIC, GATT_SPEC_CHARACTERISTIC_SCAN_REFRESH,                                //Attribute Type
    SIZE_OF_ATTRIBUTE_VALUE_1,
    0x00                                                                                            //Attribute Value
};


Uint8 att_HDL_SCPS_SCAN_REFRESH[] =
{
    //0x00, (_SCPS_HDL_OFFSET_BASE_+4),                                                             //Handle
    //GATT_SPEC_CHARACTERISTIC, GATT_SPEC_CHARACTERISTIC_SCAN_REFRESH,                              //Attribute Type
    //SIZE_OF_ATTRIBUTE_VALUE_1,
    0x00                                                                                            //Attribute Value
};


const Uint8 ATT_HDL_SCPS_SCAN_REFRESH_CLIENT_CHARACTERISTIC_CONFIGURATION_INIT[] =
{
    0x00, (_SCPS_HDL_OFFSET_BASE_ + 5),                                                             //Handle
    GATT_DESCRIPTORS, GATT_DESCRIPTORS_CLIENT_CHARACTERISTIC_CONFIGURATION,                         //Attribute Type
    SIZE_OF_ATTRIBUTE_VALUE_2,
    0x00,                                                                                           //Attribute Value
    0x00                                                                                            //Attribute Value,
};


Uint8 att_HDL_SCPS_SCAN_REFRESH_CLIENT_CHARACTERISTIC_CONFIGURATION[] =
{
    //0x00, (_SCPS_HDL_OFFSET_BASE_+5),                                                             //Handle
    //GATT_DESCRIPTORS, GATT_DESCRIPTORS_CLIENT_CHARACTERISTIC_CONFIGURATION,                       //Attribute Type
    //SIZE_OF_ATTRIBUTE_VALUE_2,
    (
        //GATT_DESCRIPTORS_CLIENT_CHARACTERISTIC_CONFIGURATION_NOTIFICATION |
        //GATT_DESCRIPTORS_CLIENT_CHARACTERISTIC_CONFIGURATION_INDICATION |
        0x00
    ),                                                                                              //Attribute Value, R/W: Mandatory
    0x00                                                                                            //Attribute Value,
};
#endif


#ifdef _PROFILE_IAS_
const Uint8 ATT_HDL_IAS_PRIMARY_SERVICE[] =
{
    0x00, _IAS_HDL_OFFSET_BASE_,                                                                    //Handle
    GATT_DECLARATIONS, GATT_DECLARATIONS_PRIMARY_SERVICE,                                           //Attribute Type
    SIZE_OF_ATTRIBUTE_VALUE_2,
    GATT_SPEC_SERVICES_IMMEDIATE_ALERT, GATT_SPEC_SERVICES                                          //Attribute Value
};


const Uint8 ATT_HDL_IAS_CHARACTERISTIC_ALERT_LEVEL[] =
{
    0x00, (_IAS_HDL_OFFSET_BASE_ + 1),                                                              //Handle
    GATT_DECLARATIONS, GATT_DECLARATIONS_CHARACTERISTIC,                                            //Attribute Type
    SIZE_OF_ATTRIBUTE_VALUE_5,
    //0x11,
    (
        //Characteristic Properties
        //GATT_DECLARATIONS_PROPERTIES_BROADCAST |
        //GATT_DECLARATIONS_PROPERTIES_READ |
        GATT_DECLARATIONS_PROPERTIES_WRITE_WITHOUT_RESPONSE |
        //GATT_DECLARATIONS_PROPERTIES_WRITE |
        //GATT_DECLARATIONS_PROPERTIES_NOTIFY |
        //GATT_DECLARATIONS_PROPERTIES_INDICATE |
        //GATT_DECLARATIONS_PROPERTIES_AUTHENTICATED_SIGNED_WRITES |
        //GATT_DECLARATIONS_PROPERTIES_EXTENDED_PROPERTIES |
        0x00
    ),
    (_IAS_HDL_OFFSET_BASE_ + 2), 0x00,
    GATT_SPEC_CHARACTERISTIC_ALERT_LEVEL, GATT_SPEC_CHARACTERISTIC                                  //Attribute Value
};


const Uint8 ATT_HDL_IAS_ALERT_LEVEL_INIT[] =
{
    0x00, (_IAS_HDL_OFFSET_BASE_ + 2),                                                              //Handle
    GATT_SPEC_CHARACTERISTIC, GATT_SPEC_CHARACTERISTIC_ALERT_LEVEL,                                 //Attribute Type
    SIZE_OF_ATTRIBUTE_VALUE_1,
    0x00                                                                                            //Attribute Value
};


Uint8 att_HDL_IAS_ALERT_LEVEL[] =
{
    //0x00, (_IAS_HDL_OFFSET_BASE_+2),                                                              //Handle
    //GATT_SPEC_CHARACTERISTIC, GATT_SPEC_CHARACTERISTIC_ALERT_LEVEL,                               //Attribute Type
    //SIZE_OF_ATTRIBUTE_VALUE_1,
    0x00                                                                                            //Attribute Value
};
#endif


#ifdef _PROFILE_LLS_
const Uint8 ATT_HDL_LLS_PRIMARY_SERVICE[] =
{
    0x00, _LLS_HDL_OFFSET_BASE_,                                                                    //Handle
    GATT_DECLARATIONS, GATT_DECLARATIONS_PRIMARY_SERVICE,                                           //Attribute Type
    SIZE_OF_ATTRIBUTE_VALUE_2,
    GATT_SPEC_SERVICES_LINK_LOSS, GATT_SPEC_SERVICES                                                //Attribute Value
};


const Uint8 ATT_HDL_LLS_CHARACTERISTIC_ALERT_LEVEL[] =
{
    0x00, (_LLS_HDL_OFFSET_BASE_ + 1),                                                              //Handle
    GATT_DECLARATIONS, GATT_DECLARATIONS_CHARACTERISTIC,                                            //Attribute Type
    SIZE_OF_ATTRIBUTE_VALUE_5,
    //0x11,
    (
        //Characteristic Properties
        //GATT_DECLARATIONS_PROPERTIES_BROADCAST |
        GATT_DECLARATIONS_PROPERTIES_READ |
        //GATT_DECLARATIONS_PROPERTIES_WRITE_WITHOUT_RESPONSE |
        GATT_DECLARATIONS_PROPERTIES_WRITE |
        //GATT_DECLARATIONS_PROPERTIES_NOTIFY |
        //GATT_DECLARATIONS_PROPERTIES_INDICATE |
        //GATT_DECLARATIONS_PROPERTIES_AUTHENTICATED_SIGNED_WRITES |
        //GATT_DECLARATIONS_PROPERTIES_EXTENDED_PROPERTIES |
        0x00
    ),
    (_LLS_HDL_OFFSET_BASE_ + 2), 0x00,
    GATT_SPEC_CHARACTERISTIC_ALERT_LEVEL, GATT_SPEC_CHARACTERISTIC                                  //Attribute Value
};


const Uint8 ATT_HDL_LLS_ALERT_LEVEL_INIT[] =
{
    0x00, (_LLS_HDL_OFFSET_BASE_ + 2),                                                              //Handle
    GATT_SPEC_CHARACTERISTIC, GATT_SPEC_CHARACTERISTIC_ALERT_LEVEL,                                 //Attribute Type
    SIZE_OF_ATTRIBUTE_VALUE_1,
    0x00                                                                                            //Attribute Value
};


Uint8 att_HDL_LLS_ALERT_LEVEL[] =
{
    //0x00, (_LLS_HDL_OFFSET_BASE_+2),                                                              //Handle
    //GATT_SPEC_CHARACTERISTIC, GATT_SPEC_CHARACTERISTIC_ALERT_LEVEL,                               //Attribute Type
    //SIZE_OF_ATTRIBUTE_VALUE_1,
    0x00                                                                                            //Attribute Value
};
#endif



#ifdef _PROFILE_TPS_
const Uint8 ATT_HDL_TPS_PRIMARY_SERVICE[] =
{
    0x00, _TPS_HDL_OFFSET_BASE_,                                                                    //Handle
    GATT_DECLARATIONS, GATT_DECLARATIONS_PRIMARY_SERVICE,                                           //Attribute Type
    SIZE_OF_ATTRIBUTE_VALUE_2,
    GATT_SPEC_SERVICES_TX_POWER, GATT_SPEC_SERVICES                                                 //Attribute Value
};


const Uint8 ATT_HDL_TPS_CHARACTERISTIC_TX_POWER_LEVEL[] =
{
    0x00, (_TPS_HDL_OFFSET_BASE_ + 1),                                                              //Handle
    GATT_DECLARATIONS, GATT_DECLARATIONS_CHARACTERISTIC,                                            //Attribute Type
    SIZE_OF_ATTRIBUTE_VALUE_5,
    //0x11,
    (
        //Characteristic Properties
        //GATT_DECLARATIONS_PROPERTIES_BROADCAST |
        GATT_DECLARATIONS_PROPERTIES_READ |
        //GATT_DECLARATIONS_PROPERTIES_WRITE_WITHOUT_RESPONSE |
        //GATT_DECLARATIONS_PROPERTIES_WRITE |
        //GATT_DECLARATIONS_PROPERTIES_NOTIFY |
        //GATT_DECLARATIONS_PROPERTIES_INDICATE |
        //GATT_DECLARATIONS_PROPERTIES_AUTHENTICATED_SIGNED_WRITES |
        //GATT_DECLARATIONS_PROPERTIES_EXTENDED_PROPERTIES |
        0x00
    ),
    (_TPS_HDL_OFFSET_BASE_ + 2), 0x00,
    GATT_SPEC_CHARACTERISTIC_TX_POWER_LEVEL, GATT_SPEC_CHARACTERISTIC                               //Attribute Value
};


const Uint8 ATT_HDL_TPS_TX_POWER_LEVEL_INIT[] =
{
    0x00, (_TPS_HDL_OFFSET_BASE_ + 2),                                                              //Handle
    GATT_SPEC_CHARACTERISTIC, GATT_SPEC_CHARACTERISTIC_TX_POWER_LEVEL,                              //Attribute Type
    SIZE_OF_ATTRIBUTE_VALUE_1,
    0x9C                                                                                            //Attribute Value
};


Uint8 att_HDL_TPS_TX_POWER_LEVEL[] =
{
    //0x00, (_TPS_HDL_OFFSET_BASE_+2),                                                              //Handle
    //GATT_SPEC_CHARACTERISTIC, GATT_SPEC_CHARACTERISTIC_TX_POWER_LEVEL,                            //Attribute Type
    //SIZE_OF_ATTRIBUTE_VALUE_1,
    0x9C                                                                                            //Attribute Value
};
#endif


#ifdef _PROFILE_BLP_
const Uint8 ATT_HDL_BLS_PRIMARY_SERVICE[] =
{
    0x00, _BLS_HDL_OFFSET_BASE_,                                                                    //Handle
    GATT_DECLARATIONS, GATT_DECLARATIONS_PRIMARY_SERVICE,                                           //Attribute Type
    SIZE_OF_ATTRIBUTE_VALUE_2,
    GATT_SPEC_SERVICES_BLOOD_PRESSURE, GATT_SPEC_SERVICES                                           //Attribute Value
};


const Uint8 ATT_HDL_BLS_CHARACTERISTIC_BLOOD_PRESSURE_MEASUREMENT[] =
{
    0x00, (_BLS_HDL_OFFSET_BASE_ + 1),                                                              //Handle
    GATT_DECLARATIONS, GATT_DECLARATIONS_CHARACTERISTIC,                                            //Attribute Type
    SIZE_OF_ATTRIBUTE_VALUE_5,
    //0x11,
    (
        //Characteristic Properties
        //GATT_DECLARATIONS_PROPERTIES_BROADCAST |
        //GATT_DECLARATIONS_PROPERTIES_READ |
        //GATT_DECLARATIONS_PROPERTIES_WRITE_WITHOUT_RESPONSE |
        //GATT_DECLARATIONS_PROPERTIES_WRITE |
        //GATT_DECLARATIONS_PROPERTIES_NOTIFY |
        GATT_DECLARATIONS_PROPERTIES_INDICATE |
        //GATT_DECLARATIONS_PROPERTIES_AUTHENTICATED_SIGNED_WRITES |
        //GATT_DECLARATIONS_PROPERTIES_EXTENDED_PROPERTIES |
        0x00
    ),
    (_BLS_HDL_OFFSET_BASE_ + 2), 0x00,
    GATT_SPEC_CHARACTERISTIC_BLOOD_PRESSURE_MEASUREMENT, GATT_SPEC_CHARACTERISTIC                   //Attribute Value
};


const Uint8 ATT_HDL_BLS_BLOOD_PRESSURE_MEASUREMENT_INIT[] =
{
    0x00, (_BLS_HDL_OFFSET_BASE_ + 2),                                                              //Handle
    GATT_SPEC_CHARACTERISTIC, GATT_SPEC_CHARACTERISTIC_BLOOD_PRESSURE_MEASUREMENT,                  //Attribute Type
    SIZE_OF_ATTRIBUTE_VALUE_16,
    (
        //Bit Field
        //BLS_MEAS_BLOOD_PRESSURE_UNITS_KPA |                                                       //Masked means Blood pressure for Systolic, Diastolic and MAP in units of mmHg
        BLS_MEAS_TIME_STAMP |
        BLS_MEAS_PULSE_RATE |
        //BLS_MEAS_USER_ID |
        //BLS_MEAS_MEASUREMENT_STATUS |
        0x00
    ),
    0x00, 0x00,                                                                                     //Blood Pressure Measurement Compound Value - Systolic (mmHg), format: SFLOAT
    0x00, 0x00,                                                                                     //Blood Pressure Measurement Compound Value - Diastolic (mmHg), format: SFLOAT
    0x00, 0x00,                                                                                     //Blood Pressure Measurement Compound Value - Mean Arterial Pressure (mmHg), format: SFLOAT
    0xDD, 0x07,                                                                                     //Time Stamp, Year: 1582~9999, 0x07DD: 2013
    0x09, 0x11,                                                                                     //Time Stamp, Month & Day, Sep.17
    0x0F, 0x00, 0x00,                                                                               //Time Stamp, Hours, Minutes & Seconds, 15:00:00
    0x00, 0x00,                                                                                     //Pulse Rate, format: SFLOAT
};
//format: SFLOAT, IEEE Std 11073-20601-2008, P102


Uint8 att_HDL_BLS_BLOOD_PRESSURE_MEASUREMENT[] =
{
    //0x00, (_BLS_HDL_OFFSET_BASE_+2),                                                              //Handle
    //GATT_SPEC_CHARACTERISTIC, GATT_SPEC_CHARACTERISTIC_BLOOD_PRESSURE_MEASUREMENT,                //Attribute Type
    //SIZE_OF_ATTRIBUTE_VALUE_16,
    (
        //Bit Field
        //BLS_MEAS_BLOOD_PRESSURE_UNITS_KPA |                                                       //Masked means Blood pressure for Systolic, Diastolic and MAP in units of mmHg
        BLS_MEAS_TIME_STAMP |
        BLS_MEAS_PULSE_RATE |
        //BLS_MEAS_USER_ID |
        //BLS_MEAS_MEASUREMENT_STATUS |
        0x00
    ),
    0x00, 0x00,                                                                                     //Blood Pressure Measurement Compound Value - Systolic (mmHg), format: SFLOAT
    0x00, 0x00,                                                                                     //Blood Pressure Measurement Compound Value - Diastolic (mmHg), format: SFLOAT
    0x00, 0x00,                                                                                     //Blood Pressure Measurement Compound Value - Mean Arterial Pressure (mmHg), format: SFLOAT
    0xDD, 0x07,                                                                                     //Time Stamp, Year: 1582~9999, 0x07DD: 2013
    0x09, 0x11,                                                                                     //Time Stamp, Month & Day, Sep.17
    0x0F, 0x00, 0x00,                                                                               //Time Stamp, Hours, Minutes & Seconds, 15:00:00
    0x00, 0x00,                                                                                     //Pulse Rate, format: SFLOAT
};


Uint8 idx_ATT_HDL_BLS_BLOOD_PRESSURE_MEASUREMENT_DATABASE[6];


const Uint8 ATT_HDL_BLS_BLOOD_PRESSURE_MEASUREMENT_DATABASE[6][16] =
{
    {
        //0x01,                                                                                     //index
        /*
        {                                                                                           //Bit Field
            //BLS_MEAS_BLOOD_PRESSURE_UNITS_KPA |                                                   //Masked means Blood pressure for Systolic, Diastolic and MAP in units of mmHg
            BLS_MEAS_TIME_STAMP |
            BLS_MEAS_PULSE_RATE |
            //BLS_MEAS_USER_ID |
            //BLS_MEAS_MEASUREMENT_STATUS |
            0x00
        },
        */
        0x06,
        0x00, 0x00,                                                                                 //Blood Pressure Measurement Compound Value - Systolic (mmHg), format: SFLOAT
        0x00, 0x00,                                                                                 //Blood Pressure Measurement Compound Value - Diastolic (mmHg), format: SFLOAT
        0x00, 0x00,                                                                                 //Blood Pressure Measurement Compound Value - Mean Arterial Pressure (mmHg), format: SFLOAT
        0xDD, 0x07,                                                                                 //Time Stamp, Year: 1582~9999, 0x07DD: 2013
        0x09, 0x12,                                                                                 //Time Stamp, Month & Day, Sep.17
        0x0F, 0x00, 0x00,                                                                           //Time Stamp, Hours, Minutes & Seconds, 15:00:00
        0x00, 0x00,                                                                                 //Pulse Rate, format: SFLOAT
    },
    {
        //0x02,                                                                                     //index
        /*
        {                                                                                           //Bit Field
            //BLS_MEAS_BLOOD_PRESSURE_UNITS_KPA |                                                   //Masked means Blood pressure for Systolic, Diastolic and MAP in units of mmHg
            BLS_MEAS_TIME_STAMP |
            BLS_MEAS_PULSE_RATE |
            //BLS_MEAS_USER_ID |
            //BLS_MEAS_MEASUREMENT_STATUS |
            0x00
        },
        */
        0x06,
        0x00, 0x00,                                                                                 //Blood Pressure Measurement Compound Value - Systolic (mmHg), format: SFLOAT
        0x00, 0x00,                                                                                 //Blood Pressure Measurement Compound Value - Diastolic (mmHg), format: SFLOAT
        0x00, 0x00,                                                                                 //Blood Pressure Measurement Compound Value - Mean Arterial Pressure (mmHg), format: SFLOAT
        0xDD, 0x07,                                                                                 //Time Stamp, Year: 1582~9999, 0x07DD: 2013
        0x09, 0x13,                                                                                 //Time Stamp, Month & Day, Sep.17
        0x0F, 0x00, 0x00,                                                                           //Time Stamp, Hours, Minutes & Seconds, 15:00:00
        0x00, 0x00,                                                                                 //Pulse Rate, format: SFLOAT
    },
    {
        //0x03,                                                                                     //index
        /*
        {                                                                                           //Bit Field
            //BLS_MEAS_BLOOD_PRESSURE_UNITS_KPA |                                                   //Masked means Blood pressure for Systolic, Diastolic and MAP in units of mmHg
            BLS_MEAS_TIME_STAMP |
            BLS_MEAS_PULSE_RATE |
            //BLS_MEAS_USER_ID |
            //BLS_MEAS_MEASUREMENT_STATUS |
            0x00
        },
        */
        0x06,
        0x00, 0x00,                                                                                     //Blood Pressure Measurement Compound Value - Systolic (mmHg), format: SFLOAT
        0x00, 0x00,                                                                                     //Blood Pressure Measurement Compound Value - Diastolic (mmHg), format: SFLOAT
        0x00, 0x00,                                                                                     //Blood Pressure Measurement Compound Value - Mean Arterial Pressure (mmHg), format: SFLOAT
        0xDD, 0x07,                                                                                     //Time Stamp, Year: 1582~9999, 0x07DD: 2013
        0x09, 0x14,                                                                                     //Time Stamp, Month & Day, Sep.17
        0x0F, 0x00, 0x00,                                                                               //Time Stamp, Hours, Minutes & Seconds, 15:00:00
        0x00, 0x00,                                                                                     //Pulse Rate, format: SFLOAT
    },
    {
        //0x04,                                                                                         //index
        /*
        {                                                                                               //Bit Field
            //BLS_MEAS_BLOOD_PRESSURE_UNITS_KPA |                                                       //Masked means Blood pressure for Systolic, Diastolic and MAP in units of mmHg
            BLS_MEAS_TIME_STAMP |
            BLS_MEAS_PULSE_RATE |
            //BLS_MEAS_USER_ID |
            //BLS_MEAS_MEASUREMENT_STATUS |
            0x00
        },
        */
        0x06,
        0x00, 0x00,                                                                                     //Blood Pressure Measurement Compound Value - Systolic (mmHg), format: SFLOAT
        0x00, 0x00,                                                                                     //Blood Pressure Measurement Compound Value - Diastolic (mmHg), format: SFLOAT
        0x00, 0x00,                                                                                     //Blood Pressure Measurement Compound Value - Mean Arterial Pressure (mmHg), format: SFLOAT
        0xDD, 0x07,                                                                                     //Time Stamp, Year: 1582~9999, 0x07DD: 2013
        0x09, 0x15,                                                                                     //Time Stamp, Month & Day, Sep.17
        0x0F, 0x00, 0x00,                                                                               //Time Stamp, Hours, Minutes & Seconds, 15:00:00
        0x00, 0x00,                                                                                     //Pulse Rate, format: SFLOAT
    },
    {
        //0x05,                                                                                         //index
        /*
        {                                                                                               //Bit Field
            //BLS_MEAS_BLOOD_PRESSURE_UNITS_KPA |                                                       //Masked means Blood pressure for Systolic, Diastolic and MAP in units of mmHg
            BLS_MEAS_TIME_STAMP |
            BLS_MEAS_PULSE_RATE |
            //BLS_MEAS_USER_ID |
            //BLS_MEAS_MEASUREMENT_STATUS |
            0x00
        },
        */
        0x06,
        0x00, 0x00,                                                                                     //Blood Pressure Measurement Compound Value - Systolic (mmHg), format: SFLOAT
        0x00, 0x00,                                                                                     //Blood Pressure Measurement Compound Value - Diastolic (mmHg), format: SFLOAT
        0x00, 0x00,                                                                                     //Blood Pressure Measurement Compound Value - Mean Arterial Pressure (mmHg), format: SFLOAT
        0xDD, 0x07,                                                                                     //Time Stamp, Year: 1582~9999, 0x07DD: 2013
        0x09, 0x16,                                                                                     //Time Stamp, Month & Day, Sep.17
        0x0F, 0x00, 0x00,                                                                               //Time Stamp, Hours, Minutes & Seconds, 15:00:00
        0x00, 0x00,                                                                                     //Pulse Rate, format: SFLOAT
    },
    {
        //0x00,                                                                                         //index
        /*
        {                                                                                               //Bit Field
            //BLS_MEAS_BLOOD_PRESSURE_UNITS_KPA |                                                       //Masked means Blood pressure for Systolic, Diastolic and MAP in units of mmHg
            BLS_MEAS_TIME_STAMP |
            BLS_MEAS_PULSE_RATE |
            //BLS_MEAS_USER_ID |
            //BLS_MEAS_MEASUREMENT_STATUS |
            0x00
        },
        */
        0x06,
        0x00, 0x00,                                                                                     //Blood Pressure Measurement Compound Value - Systolic (mmHg), format: SFLOAT
        0x00, 0x00,                                                                                     //Blood Pressure Measurement Compound Value - Diastolic (mmHg), format: SFLOAT
        0x00, 0x00,                                                                                     //Blood Pressure Measurement Compound Value - Mean Arterial Pressure (mmHg), format: SFLOAT
        0xDD, 0x07,                                                                                     //Time Stamp, Year: 1582~9999, 0x07DD: 2013
        0x09, 0x17,                                                                                     //Time Stamp, Month & Day, Sep.17
        0x0F, 0x00, 0x00,                                                                               //Time Stamp, Hours, Minutes & Seconds, 15:00:00
        0x00, 0x00,                                                                                     //Pulse Rate, format: SFLOAT
    },
};


const Uint8 ATT_HDL_BLS_BLOOD_PRESSURE_CLIENT_CHARACTERISTIC_CONFIGURATION_INIT[] =                 //Mandatory if the Battery Level characteristic properties supports notification
{
    0x00, (_BLS_HDL_OFFSET_BASE_ + 3),                                                              //Handle
    GATT_DESCRIPTORS, GATT_DESCRIPTORS_CLIENT_CHARACTERISTIC_CONFIGURATION,                         //Attribute Type
    SIZE_OF_ATTRIBUTE_VALUE_2,
    0x00,                                                                                           //Attribute Value
    0x00                                                                                            //Attribute Value,
};


Uint8 att_HDL_BLS_BLOOD_PRESSURE_CLIENT_CHARACTERISTIC_CONFIGURATION[] =
{
    //0x00, (_BLS_HDL_OFFSET_BASE_+3),                                                              //Handle
    //GATT_DESCRIPTORS, GATT_DESCRIPTORS_CLIENT_CHARACTERISTIC_CONFIGURATION,                       //Attribute Type
    //SIZE_OF_ATTRIBUTE_VALUE_2,
    (
        //GATT_DESCRIPTORS_CLIENT_CHARACTERISTIC_CONFIGURATION_NOTIFICATION |
        //GATT_DESCRIPTORS_CLIENT_CHARACTERISTIC_CONFIGURATION_INDICATION |
        0x00
    ),                                                                                              //Attribute Value, R/W: Mandatory
    0x00                                                                                            //Attribute Value,
};


const Uint8 ATT_HDL_BLS_CHARACTERISTIC_BLOOD_PRESSURE_FEATURE[] =
{
    0x00, (_BLS_HDL_OFFSET_BASE_ + 4),                                                              //Handle
    GATT_DECLARATIONS, GATT_DECLARATIONS_CHARACTERISTIC,                                            //Attribute Type
    SIZE_OF_ATTRIBUTE_VALUE_5,
    //0x11,
    (
        //Characteristic Properties
        //GATT_DECLARATIONS_PROPERTIES_BROADCAST |
        GATT_DECLARATIONS_PROPERTIES_READ |
        //GATT_DECLARATIONS_PROPERTIES_WRITE_WITHOUT_RESPONSE |
        //GATT_DECLARATIONS_PROPERTIES_WRITE |
        //GATT_DECLARATIONS_PROPERTIES_NOTIFY |
        //GATT_DECLARATIONS_PROPERTIES_INDICATE |
        //GATT_DECLARATIONS_PROPERTIES_AUTHENTICATED_SIGNED_WRITES |
        //GATT_DECLARATIONS_PROPERTIES_EXTENDED_PROPERTIES |
        0x00
    ),
    (_BLS_HDL_OFFSET_BASE_ + 4), 0x00,
    GATT_SPEC_CHARACTERISTIC_BLOOD_PRESSURE_FEATURE, GATT_SPEC_CHARACTERISTIC                       //Attribute Value
};


const Uint8 ATT_HDL_BLS_BLOOD_PRESSURE_FEATURE_INIT[] =
{
    0x00, (_BLS_HDL_OFFSET_BASE_ + 5),                                                              //Handle
    GATT_SPEC_CHARACTERISTIC, GATT_SPEC_CHARACTERISTIC_BLOOD_PRESSURE_FEATURE,                      //Attribute Type
    SIZE_OF_ATTRIBUTE_VALUE_2,
    (
        //Bit Field
        //BLS_FEAT_BODY_MOVEMENT_DETECTION |
        //BLS_FEAT_CUFF_FIT_DETECTION |
        //BLS_FEAT_IRREGULAR_PULSE_DETECTION |
        //BLS_FEAT_PULSE_RATE_RANGE_DETECTION |
        //BLS_FEAT_MEASUREMENT_POSITION_DETECTION |
        //BLS_FEAT_MULTIPLE_BOND |
        0x00
    ),
    0x00
};
#endif


#ifdef _PROFILE_HTP_
const Uint8 ATT_HDL_HTS_PRIMARY_SERVICE[] =
{
    0x00, _HTS_HDL_OFFSET_BASE_,                                                                    //Handle
    GATT_DECLARATIONS, GATT_DECLARATIONS_PRIMARY_SERVICE,                                           //Attribute Type
    SIZE_OF_ATTRIBUTE_VALUE_2,
    GATT_SPEC_SERVICES_HEALTH_THERMOMETER, GATT_SPEC_SERVICES                                       //Attribute Value
};


const Uint8 ATT_HDL_HTS_CHARACTERISTIC_TEMPERATURE_MEASUREMENT[] =
{
    0x00, (_HTS_HDL_OFFSET_BASE_ + 1),                                                              //Handle
    GATT_DECLARATIONS, GATT_DECLARATIONS_CHARACTERISTIC,                                            //Attribute Type
    SIZE_OF_ATTRIBUTE_VALUE_5,
    //0x11,
    (
        //Characteristic Properties
        //GATT_DECLARATIONS_PROPERTIES_BROADCAST |
        //GATT_DECLARATIONS_PROPERTIES_READ |
        //GATT_DECLARATIONS_PROPERTIES_WRITE_WITHOUT_RESPONSE |
        //GATT_DECLARATIONS_PROPERTIES_WRITE |
        //GATT_DECLARATIONS_PROPERTIES_NOTIFY |
        GATT_DECLARATIONS_PROPERTIES_INDICATE |
        //GATT_DECLARATIONS_PROPERTIES_AUTHENTICATED_SIGNED_WRITES |
        //GATT_DECLARATIONS_PROPERTIES_EXTENDED_PROPERTIES |
        0x00
    ),
    (_HTS_HDL_OFFSET_BASE_ + 2), 0x00,
    GATT_SPEC_CHARACTERISTIC_TEMPERATURE_MEASUREMENT, GATT_SPEC_CHARACTERISTIC                      //Attribute Value
};


const Uint8 ATT_HDL_HTS_TEMPERATURE_MEASUREMENT_INIT[] =
{
    0x00, (_HTS_HDL_OFFSET_BASE_ + 2),                                                              //Handle
    GATT_SPEC_CHARACTERISTIC, GATT_SPEC_CHARACTERISTIC_TEMPERATURE_MEASUREMENT,                     //Attribute Type
    SIZE_OF_ATTRIBUTE_VALUE_13,
    (
        //Bit Field
        //HTS_MEAS_TEMPERATURE_MEASUREMENT_VALUE_FAHRENHEIT |                                       //Masked means Blood pressure for Systolic, Diastolic and MAP in units of mmHg
        HTS_MEAS_TIME_STAMP |
        HTS_MEAS_TEMPERATURE_TYPE |
        //BLS_MEAS_USER_ID |
        //BLS_MEAS_MEASUREMENT_STATUS |
        0x00
    ),
    0x00, 0x00, 0x00, 0x00,                                                                         //Temperature Measurement Value  - celsius (C), format: FLOAT
    0xDD, 0x07,                                                                                     //Time Stamp, Year: 1582~9999, 0x07DD: 2013
    0x09, 0x11,                                                                                     //Time Stamp, Month & Day, Sep.17
    0x0F, 0x00, 0x00,                                                                               //Time Stamp, Hours, Minutes & Seconds, 15:00:00
    0x06                                                                                            //Temperature Type, Mouse
};
//format: SFLOAT, IEEE Std 11073-20601-2008, P102


Uint8 att_HDL_HTS_TEMPERATURE_MEASUREMENT[] =
{
    //0x00, (_HTS_HDL_OFFSET_BASE_+2),                                                              //Handle
    //GATT_SPEC_CHARACTERISTIC, GATT_SPEC_CHARACTERISTIC_TEMPERATURE_MEASUREMENT,                   //Attribute Type
    //SIZE_OF_ATTRIBUTE_VALUE_13,
    (
        //Bit Field
        //HTS_MEAS_TEMPERATURE_MEASUREMENT_VALUE_FAHRENHEIT |                                       //Masked means Blood pressure for Systolic, Diastolic and MAP in units of mmHg
        HTS_MEAS_TIME_STAMP |
        HTS_MEAS_TEMPERATURE_TYPE |
        //BLS_MEAS_USER_ID |
        //BLS_MEAS_MEASUREMENT_STATUS |
        0x00
    ),
    0x00, 0x00, 0x00, 0x00,                                                                         //Temperature Measurement Value  - celsius (C), format: FLOAT
    0xDD, 0x07,                                                                                     //Time Stamp, Year: 1582~9999, 0x07DD: 2013
    0x09, 0x11,                                                                                     //Time Stamp, Month & Day, Sep.17
    0x0F, 0x00, 0x00,                                                                               //Time Stamp, Hours, Minutes & Seconds, 15:00:00
    0x06                                                                                            //Temperature Type, Mouse
};


Uint8 idx_ATT_HDL_HTS_TEMPERATURE_MEASUREMENT_DATABASE[6];


const Uint8 ATT_HDL_HTS_TEMPERATURE_MEASUREMENT_DATABASE[6][SIZE_OF_ATTRIBUTE_VALUE_13] =
{
    {
        //0x01,                                                                                         //index
        {
            //Bit Field
            //HTS_MEAS_TEMPERATURE_MEASUREMENT_VALUE_FAHRENHEIT |                                       //Masked means Blood pressure for Systolic, Diastolic and MAP in units of mmHg
            HTS_MEAS_TIME_STAMP |
            HTS_MEAS_TEMPERATURE_TYPE |
            //BLS_MEAS_USER_ID |
            //BLS_MEAS_MEASUREMENT_STATUS |
            0x00
        },
        0x00, 0x00, 0x00, 0x00,                                                                         //Temperature Measurement Value  - celsius (C), format: FLOAT
        0xDD, 0x07,                                                                                     //Time Stamp, Year: 1582~9999, 0x07DD: 2013
        0x09, 0x11,                                                                                     //Time Stamp, Month & Day, Sep.17
        0x0F, 0x00, 0x00,                                                                               //Time Stamp, Hours, Minutes & Seconds, 15:00:00
        0x06                                                                                            //Temperature Type, Mouse
    },
    {
        //0x02,                                                                                         //index
        {
            //Bit Field
            //HTS_MEAS_TEMPERATURE_MEASUREMENT_VALUE_FAHRENHEIT |                                       //Masked means Blood pressure for Systolic, Diastolic and MAP in units of mmHg
            HTS_MEAS_TIME_STAMP |
            HTS_MEAS_TEMPERATURE_TYPE |
            //BLS_MEAS_USER_ID |
            //BLS_MEAS_MEASUREMENT_STATUS |
            0x00
        },
        0x00, 0x00, 0x00, 0x00,                                                                         //Temperature Measurement Value  - celsius (C), format: FLOAT
        0xDD, 0x07,                                                                                     //Time Stamp, Year: 1582~9999, 0x07DD: 2013
        0x09, 0x11,                                                                                     //Time Stamp, Month & Day, Sep.17
        0x0F, 0x00, 0x00,                                                                               //Time Stamp, Hours, Minutes & Seconds, 15:00:00
        0x06                                                                                            //Temperature Type, Mouse
    },
    {
        //0x03,                                                                                         //index
        {
            //Bit Field
            //HTS_MEAS_TEMPERATURE_MEASUREMENT_VALUE_FAHRENHEIT |                                       //Masked means Blood pressure for Systolic, Diastolic and MAP in units of mmHg
            HTS_MEAS_TIME_STAMP |
            HTS_MEAS_TEMPERATURE_TYPE |
            //BLS_MEAS_USER_ID |
            //BLS_MEAS_MEASUREMENT_STATUS |
            0x00
        },
        0x00, 0x00, 0x00, 0x00,                                                                         //Temperature Measurement Value  - celsius (C), format: FLOAT
        0xDD, 0x07,                                                                                     //Time Stamp, Year: 1582~9999, 0x07DD: 2013
        0x09, 0x11,                                                                                     //Time Stamp, Month & Day, Sep.17
        0x0F, 0x00, 0x00,                                                                               //Time Stamp, Hours, Minutes & Seconds, 15:00:00
        0x06                                                                                            //Temperature Type, Mouse
    },
    {
        //0x04,                                                                                         //index
        {
            //Bit Field
            //HTS_MEAS_TEMPERATURE_MEASUREMENT_VALUE_FAHRENHEIT |                                       //Masked means Blood pressure for Systolic, Diastolic and MAP in units of mmHg
            HTS_MEAS_TIME_STAMP |
            HTS_MEAS_TEMPERATURE_TYPE |
            //BLS_MEAS_USER_ID |
            //BLS_MEAS_MEASUREMENT_STATUS |
            0x00
        },
        0x00, 0x00, 0x00, 0x00,                                                                         //Temperature Measurement Value  - celsius (C), format: FLOAT
        0xDD, 0x07,                                                                                     //Time Stamp, Year: 1582~9999, 0x07DD: 2013
        0x09, 0x11,                                                                                     //Time Stamp, Month & Day, Sep.17
        0x0F, 0x00, 0x00,                                                                               //Time Stamp, Hours, Minutes & Seconds, 15:00:00
        0x06                                                                                            //Temperature Type, Mouse
    },
    {
        //0x05,                                                                                         //index
        {
            //Bit Field
            //HTS_MEAS_TEMPERATURE_MEASUREMENT_VALUE_FAHRENHEIT |                                       //Masked means Blood pressure for Systolic, Diastolic and MAP in units of mmHg
            HTS_MEAS_TIME_STAMP |
            HTS_MEAS_TEMPERATURE_TYPE |
            //BLS_MEAS_USER_ID |
            //BLS_MEAS_MEASUREMENT_STATUS |
            0x00
        },
        0x00, 0x00, 0x00, 0x00,                                                                         //Temperature Measurement Value  - celsius (C), format: FLOAT
        0xDD, 0x07,                                                                                     //Time Stamp, Year: 1582~9999, 0x07DD: 2013
        0x09, 0x11,                                                                                     //Time Stamp, Month & Day, Sep.17
        0x0F, 0x00, 0x00,                                                                               //Time Stamp, Hours, Minutes & Seconds, 15:00:00
        0x06                                                                                            //Temperature Type, Mouse
    },
    {
        //0x00,                                                                                         //index
        {
            //Bit Field
            //HTS_MEAS_TEMPERATURE_MEASUREMENT_VALUE_FAHRENHEIT |                                       //Masked means Blood pressure for Systolic, Diastolic and MAP in units of mmHg
            HTS_MEAS_TIME_STAMP |
            HTS_MEAS_TEMPERATURE_TYPE |
            //BLS_MEAS_USER_ID |
            //BLS_MEAS_MEASUREMENT_STATUS |
            0x00
        },
        0x00, 0x00, 0x00, 0x00,                                                                         //Temperature Measurement Value  - celsius (C), format: FLOAT
        0xDD, 0x07,                                                                                     //Time Stamp, Year: 1582~9999, 0x07DD: 2013
        0x09, 0x11,                                                                                     //Time Stamp, Month & Day, Sep.17
        0x0F, 0x00, 0x00,                                                                               //Time Stamp, Hours, Minutes & Seconds, 15:00:00
        0x06                                                                                            //Temperature Type, Mouse
    },
};


const Uint8 ATT_HDL_HTS_TEMPERATURE_MEASUREMENT_CLIENT_CHARACTERISTIC_CONFIGURATION_INIT[] =        //Mandatory if the Battery Level characteristic properties supports notification
{
    0x00, (_HTS_HDL_OFFSET_BASE_ + 3),                                                              //Handle
    GATT_DESCRIPTORS, GATT_DESCRIPTORS_CLIENT_CHARACTERISTIC_CONFIGURATION,                         //Attribute Type
    SIZE_OF_ATTRIBUTE_VALUE_2,
    0x00,                                                                                           //Attribute Value
    0x00                                                                                            //Attribute Value,
};


Uint8 att_HDL_HTS_TEMPERATURE_MEASUREMENT_CLIENT_CHARACTERISTIC_CONFIGURATION[] =
{
    //0x00, (_HTS_HDL_OFFSET_BASE_+3),                                                              //Handle
    //GATT_DESCRIPTORS, GATT_DESCRIPTORS_CLIENT_CHARACTERISTIC_CONFIGURATION,                       //Attribute Type
    //SIZE_OF_ATTRIBUTE_VALUE_2,
    (
        //GATT_DESCRIPTORS_CLIENT_CHARACTERISTIC_CONFIGURATION_NOTIFICATION |
        //GATT_DESCRIPTORS_CLIENT_CHARACTERISTIC_CONFIGURATION_INDICATION |
        0x00
    ),                                                                                              //Attribute Value, R/W: Mandatory
    0x00                                                                                            //Attribute Value,
};


const Uint8 ATT_HDL_HTS_CHARACTERISTIC_TEMPERATURE_TYPE[] =
{
    0x00, (_HTS_HDL_OFFSET_BASE_ + 4),                                                              //Handle
    GATT_DECLARATIONS, GATT_DECLARATIONS_CHARACTERISTIC,                                            //Attribute Type
    SIZE_OF_ATTRIBUTE_VALUE_5,
    //0x11,
    (
        //Characteristic Properties
        //GATT_DECLARATIONS_PROPERTIES_BROADCAST |
        GATT_DECLARATIONS_PROPERTIES_READ |
        //GATT_DECLARATIONS_PROPERTIES_WRITE_WITHOUT_RESPONSE |
        //GATT_DECLARATIONS_PROPERTIES_WRITE |
        //GATT_DECLARATIONS_PROPERTIES_NOTIFY |
        //GATT_DECLARATIONS_PROPERTIES_INDICATE |
        //GATT_DECLARATIONS_PROPERTIES_AUTHENTICATED_SIGNED_WRITES |
        //GATT_DECLARATIONS_PROPERTIES_EXTENDED_PROPERTIES |
        0x00
    ),
    (_HTS_HDL_OFFSET_BASE_ + 5), 0x00,
    GATT_SPEC_CHARACTERISTIC_TEMPERATURE_TYPE, GATT_SPEC_CHARACTERISTIC                             //Attribute Value
};


const Uint8 ATT_HDL_HTS_TEMPERATURE_TYPE_INIT[] =
{
    0x00, (_HTS_HDL_OFFSET_BASE_ + 5),                                                              //Handle
    GATT_SPEC_CHARACTERISTIC, GATT_SPEC_CHARACTERISTIC_TEMPERATURE_TYPE,                            //Attribute Type
    SIZE_OF_ATTRIBUTE_VALUE_1,
    0x06                                                                                            //Mouse
};
#endif


#ifdef _PROFILE_RSCP_
const Uint8 ATT_HDL_RSCS_PRIMARY_SERVICE[] =
{
    0x00, _RSCS_HDL_OFFSET_BASE_,                                                                   //Handle
    GATT_DECLARATIONS, GATT_DECLARATIONS_PRIMARY_SERVICE,                                           //Attribute Type
    SIZE_OF_ATTRIBUTE_VALUE_2,
    GATT_SPEC_SERVICES_RUNNING_SPEED_AND_CADENCE, GATT_SPEC_SERVICES                                //Attribute Value
};


const Uint8 ATT_HDL_RSCS_CHARACTERISTIC_RSC_MEASUREMENT[] =
{
    0x00, (_RSCS_HDL_OFFSET_BASE_ + 1),                                                             //Handle
    GATT_DECLARATIONS, GATT_DECLARATIONS_CHARACTERISTIC,                                            //Attribute Type
    SIZE_OF_ATTRIBUTE_VALUE_5,
    //0x11,
    (
        //Characteristic Properties
        //GATT_DECLARATIONS_PROPERTIES_BROADCAST |
        //GATT_DECLARATIONS_PROPERTIES_READ |
        //GATT_DECLARATIONS_PROPERTIES_WRITE_WITHOUT_RESPONSE |
        //GATT_DECLARATIONS_PROPERTIES_WRITE |
        GATT_DECLARATIONS_PROPERTIES_NOTIFY |
        //GATT_DECLARATIONS_PROPERTIES_INDICATE |
        //GATT_DECLARATIONS_PROPERTIES_AUTHENTICATED_SIGNED_WRITES |
        //GATT_DECLARATIONS_PROPERTIES_EXTENDED_PROPERTIES |
        0x00
    ),
    (_RSCS_HDL_OFFSET_BASE_ + 2), 0x00,
    GATT_SPEC_CHARACTERISTIC_RSC_MEASUREMENT, GATT_SPEC_CHARACTERISTIC                              //Attribute Value
};


const Uint8 ATT_HDL_RSCS_RSC_MEASUREMENT_INIT[] =
{
    0x00, (_RSCS_HDL_OFFSET_BASE_ + 2),                                                             //Handle
    GATT_SPEC_CHARACTERISTIC, GATT_SPEC_CHARACTERISTIC_RSC_MEASUREMENT,                             //Attribute Type
    SIZE_OF_ATTRIBUTE_VALUE_10,
    (
        //Bit Field
        RSCS_MEAS_INSTANTANEOUS_STRIDE_LENGTH_PRESENT |
        RSCS_MEAS_TOTAL_DISTANCE_PRESENT |
        RSCS_MEAS_WALKING_OR_RUNNING_STATUS_BITS |
        0x00
    ),
    0x00, 0x00,                                                                                     //Instantaneous Speed(ms, 1/256 s)
    0x00,                                                                                           //Instantaneous Cadence(RPM)
    0x00, 0x00,                                                                                     //Instantaneous Stride Length (m, 1/100 m)
    0x00, 0x00, 0x00, 0x00                                                                          //Total Distance (m, 1/10 m)
};


Uint8 att_HDL_RSCS_RSC_MEASUREMENT[] =
{
    //0x00, (_RSCS_HDL_OFFSET_BASE_+2),                                                             //Handle
    //GATT_SPEC_CHARACTERISTIC, GATT_SPEC_CHARACTERISTIC_RSC_MEASUREMENT,                           //Attribute Type
    //SIZE_OF_ATTRIBUTE_VALUE_10,
    (
        //Bit Field
        RSCS_MEAS_INSTANTANEOUS_STRIDE_LENGTH_PRESENT |
        RSCS_MEAS_TOTAL_DISTANCE_PRESENT |
        RSCS_MEAS_WALKING_OR_RUNNING_STATUS_BITS |
        0x00
    ),
    0x00, 0x00,                                                                                     //Instantaneous Speed(ms, 1/256 s)
    0x00,                                                                                           //Instantaneous Cadence(RPM)
    0x00, 0x00,                                                                                     //Instantaneous Stride Length (m, 1/100 m)
    0x00, 0x00, 0x00, 0x00                                                                          //Total Distance (m, 1/10 m)
};


const Uint8 ATT_HDL_RSCS_RSC_MEASUREMENT_CLIENT_CHARACTERISTIC_CONFIGURATION_INIT[] =               //Mandatory if the Battery Level characteristic properties supports notification
{
    0x00, (_RSCS_HDL_OFFSET_BASE_ + 3),                                                             //Handle
    GATT_DESCRIPTORS, GATT_DESCRIPTORS_CLIENT_CHARACTERISTIC_CONFIGURATION,                         //Attribute Type
    SIZE_OF_ATTRIBUTE_VALUE_2,
    0x00,                                                                                           //Attribute Value
    0x00                                                                                            //Attribute Value,
};


Uint8 att_HDL_RSCS_RSC_MEASUREMENT_CLIENT_CHARACTERISTIC_CONFIGURATION[] =
{
    //0x00, (_RSCS_HDL_OFFSET_BASE_+3),                                                             //Handle
    //GATT_DESCRIPTORS, GATT_DESCRIPTORS_CLIENT_CHARACTERISTIC_CONFIGURATION,                       //Attribute Type
    //SIZE_OF_ATTRIBUTE_VALUE_2,
    (
        //GATT_DESCRIPTORS_CLIENT_CHARACTERISTIC_CONFIGURATION_NOTIFICATION |
        //GATT_DESCRIPTORS_CLIENT_CHARACTERISTIC_CONFIGURATION_INDICATION |
        0x00
    ),                                                                                              //Attribute Value, R/W: Mandatory
    0x00                                                                                            //Attribute Value,
};


const Uint8 ATT_HDL_RSCS_CHARACTERISTIC_RSC_FEATURE[] =
{
    0x00, (_RSCS_HDL_OFFSET_BASE_ + 4),                                                             //Handle
    GATT_DECLARATIONS, GATT_DECLARATIONS_CHARACTERISTIC,                                            //Attribute Type
    SIZE_OF_ATTRIBUTE_VALUE_5,
    //0x11,
    (
        //Characteristic Properties
        //GATT_DECLARATIONS_PROPERTIES_BROADCAST |
        GATT_DECLARATIONS_PROPERTIES_READ |
        //GATT_DECLARATIONS_PROPERTIES_WRITE_WITHOUT_RESPONSE |
        //GATT_DECLARATIONS_PROPERTIES_WRITE |
        //GATT_DECLARATIONS_PROPERTIES_NOTIFY |
        //GATT_DECLARATIONS_PROPERTIES_INDICATE |
        //GATT_DECLARATIONS_PROPERTIES_AUTHENTICATED_SIGNED_WRITES |
        //GATT_DECLARATIONS_PROPERTIES_EXTENDED_PROPERTIES |
        0x00
    ),
    (_RSCS_HDL_OFFSET_BASE_ + 5), 0x00,
    GATT_SPEC_CHARACTERISTIC_RSC_FEATURE, GATT_SPEC_CHARACTERISTIC                                  //Attribute Value
};


const Uint8 ATT_HDL_RSCS_RSC_FEATURE_INIT[] =
{
    0x00, (_RSCS_HDL_OFFSET_BASE_ + 5),                                                             //Handle
    GATT_SPEC_CHARACTERISTIC, GATT_SPEC_CHARACTERISTIC_RSC_FEATURE,                                 //Attribute Type
    SIZE_OF_ATTRIBUTE_VALUE_2,
    (
        RSCS_FEAT_INSTANTANEOUS_STRIDE_LENGTH_MEASUREMENT_SUPPORTED |
        RSCS_FEAT_TOTAL_DISTANCE_MEASUREMENT_SUPPORTED |
        RSCS_FEAT_WALKING_OR_RUNNING_STATUS_SUPPORTED |
        RSCS_FEAT_CALIBRATION_PROCEDURE_NOT_SUPPORTED |
        //RSCS_FEAT_MULTIPLE_SENSOR_LOCATIONS_NOT_SUPPORTED |
        0x00
    ),
    0x00
};


const Uint8 ATT_HDL_RSCS_CHARACTERISTIC_SC_CONTROL_POINT[] =
{
    0x00, (_RSCS_HDL_OFFSET_BASE_ + 6),                                                             //Handle
    GATT_DECLARATIONS, GATT_DECLARATIONS_CHARACTERISTIC,                                            //Attribute Type
    SIZE_OF_ATTRIBUTE_VALUE_5,
    //0x11,
    (
        //Characteristic Properties
        //GATT_DECLARATIONS_PROPERTIES_BROADCAST |
        //GATT_DECLARATIONS_PROPERTIES_READ |
        //GATT_DECLARATIONS_PROPERTIES_WRITE_WITHOUT_RESPONSE |
        GATT_DECLARATIONS_PROPERTIES_WRITE |
        //GATT_DECLARATIONS_PROPERTIES_NOTIFY |
        GATT_DECLARATIONS_PROPERTIES_INDICATE |
        //GATT_DECLARATIONS_PROPERTIES_AUTHENTICATED_SIGNED_WRITES |
        //GATT_DECLARATIONS_PROPERTIES_EXTENDED_PROPERTIES |
        0x00
    ),
    (_RSCS_HDL_OFFSET_BASE_ + 7), 0x00,
    GATT_SPEC_CHARACTERISTIC_SC_CONTROL_POINT, GATT_SPEC_CHARACTERISTIC                             //Attribute Value
};


const Uint8 ATT_HDL_RSCS_SC_CONTROL_POINT_INIT[] =
{
    0x00, (_RSCS_HDL_OFFSET_BASE_ + 7),                                                             //Handle
    GATT_SPEC_CHARACTERISTIC, GATT_SPEC_CHARACTERISTIC_SC_CONTROL_POINT,                            //Attribute Type
    SIZE_OF_ATTRIBUTE_VALUE_5,
    0x00,                                                                                           //Op Code: Reserved
    0x00,                                                                                           //Cumulative Value: Total Distance B0 / Request Op Code
    0x00,                                                                                           //Cumulative Value: Total Distance B1 / Response Value
    0x00,                                                                                           //Cumulative Value: Total Distance B2 / Not used
    0x00,                                                                                           //Cumulative Value: Total Distance B3 / Not used
};
//format: SFLOAT, IEEE Std 11073-20601-2008, P102


Uint8 att_HDL_RSCS_SC_CONTROL_POINT[] =
{
    //0x00, (_RSCS_HDL_OFFSET_BASE_+7),                                                             //Handle
    //GATT_SPEC_CHARACTERISTIC, GATT_SPEC_CHARACTERISTIC_RSC_MEASUREMENT,                           //Attribute Type
    //SIZE_OF_ATTRIBUTE_VALUE_5,
    0x00,                                                                                           //Op Code: Reserved
    0x00,                                                                                           //Cumulative Value: Total Distance B0 / Request Op Code
    0x00,                                                                                           //Cumulative Value: Total Distance B1 / Response Value
    0x00,                                                                                           //Cumulative Value: Total Distance B2 / Not used
    0x00,                                                                                           //Cumulative Value: Total Distance B3 / Not used
};


const Uint8 ATT_HDL_RSCS_SC_CONTROL_POINT_CLIENT_CHARACTERISTIC_CONFIGURATION_INIT[] =              //Mandatory if the Battery Level characteristic properties supports notification
{
    0x00, (_RSCS_HDL_OFFSET_BASE_ + 8),                                                             //Handle
    GATT_DESCRIPTORS, GATT_DESCRIPTORS_CLIENT_CHARACTERISTIC_CONFIGURATION,                         //Attribute Type
    SIZE_OF_ATTRIBUTE_VALUE_2,
    0x00,                                                                                           //Attribute Value
    0x00                                                                                            //Attribute Value,
};


Uint8 att_HDL_RSCS_SC_CONTROL_POINT_CLIENT_CHARACTERISTIC_CONFIGURATION[] =
{
    //0x00, (_RSCS_HDL_OFFSET_BASE_+8),                                                             //Handle
    //GATT_DESCRIPTORS, GATT_DESCRIPTORS_CLIENT_CHARACTERISTIC_CONFIGURATION,                       //Attribute Type
    //SIZE_OF_ATTRIBUTE_VALUE_2,
    (
        //GATT_DESCRIPTORS_CLIENT_CHARACTERISTIC_CONFIGURATION_NOTIFICATION |
        //GATT_DESCRIPTORS_CLIENT_CHARACTERISTIC_CONFIGURATION_INDICATION |
        0x00
    ),                                                                                              //Attribute Value, R/W: Mandatory
    0x00                                                                                            //Attribute Value,
};
#endif


#ifdef _PROFILE_CSCP_
const Uint8 ATT_HDL_CSCS_PRIMARY_SERVICE[] =
{
    0x00, _CSCS_HDL_OFFSET_BASE_,                                                                   //Handle
    GATT_DECLARATIONS, GATT_DECLARATIONS_PRIMARY_SERVICE,                                           //Attribute Type
    SIZE_OF_ATTRIBUTE_VALUE_2,
    GATT_SPEC_SERVICES_CYCLING_SPEED_AND_CADENCE, GATT_SPEC_SERVICES                                //Attribute Value
};


const Uint8 ATT_HDL_CSCS_CHARACTERISTIC_CSC_MEASUREMENT[] =
{
    0x00, (_CSCS_HDL_OFFSET_BASE_ + 1),                                                             //Handle
    GATT_DECLARATIONS, GATT_DECLARATIONS_CHARACTERISTIC,                                            //Attribute Type
    SIZE_OF_ATTRIBUTE_VALUE_5,
    //0x11,
    (
        //Characteristic Properties
        //GATT_DECLARATIONS_PROPERTIES_BROADCAST |
        //GATT_DECLARATIONS_PROPERTIES_READ |
        //GATT_DECLARATIONS_PROPERTIES_WRITE_WITHOUT_RESPONSE |
        //GATT_DECLARATIONS_PROPERTIES_WRITE |
        GATT_DECLARATIONS_PROPERTIES_NOTIFY |
        //GATT_DECLARATIONS_PROPERTIES_INDICATE |
        //GATT_DECLARATIONS_PROPERTIES_AUTHENTICATED_SIGNED_WRITES |
        //GATT_DECLARATIONS_PROPERTIES_EXTENDED_PROPERTIES |
        0x00
    ),
    (_CSCS_HDL_OFFSET_BASE_ + 2), 0x00,
    GATT_SPEC_CHARACTERISTIC_CSC_MEASUREMENT, GATT_SPEC_CHARACTERISTIC                              //Attribute Value
};


const Uint8 ATT_HDL_CSCS_CSC_MEASUREMENT_INIT[] =
{
    0x00, (_CSCS_HDL_OFFSET_BASE_ + 2),                                                             //Handle
    GATT_SPEC_CHARACTERISTIC, GATT_SPEC_CHARACTERISTIC_CSC_MEASUREMENT,                             //Attribute Type
    SIZE_OF_ATTRIBUTE_VALUE_11,
    (
        //Bit Field
        CSCS_MEAS_WHEEL_REVOLUTION_DATA_PRESENT |
        CSCS_MEAS_CRANK_REVOLUTION_DATA_PRESENT |
        0x00
    ),
    0x00, 0x00, 0x00, 0x00,                                                                         //Cumulative Wheel Revolutions
    0x00, 0x00,                                                                                     //Last Wheel Event Time
    0x00, 0x00,                                                                                     //Cumulative Crank Revolutions
    0x00, 0x00,                                                                                     //Last Crank Event Time
};


Uint8 att_HDL_CSCS_CSC_MEASUREMENT[] =
{
    //0x00, (_CSCS_HDL_OFFSET_BASE_+2),                                                             //Handle
    //GATT_SPEC_CHARACTERISTIC, GATT_SPEC_CHARACTERISTIC_CSC_MEASUREMENT,                           //Attribute Type
    //SIZE_OF_ATTRIBUTE_VALUE_11,
    (
        //Bit Field
        CSCS_MEAS_WHEEL_REVOLUTION_DATA_PRESENT |
        CSCS_MEAS_CRANK_REVOLUTION_DATA_PRESENT |
        0x00
    ),
    0x00, 0x00, 0x00, 0x00,                                                                         //Cumulative Wheel Revolutions
    0x00, 0x00,                                                                                     //Last Wheel Event Time
    0x00, 0x00,                                                                                     //Cumulative Crank Revolutions
    0x00, 0x00,                                                                                     //Last Crank Event Time
};


const Uint8 ATT_HDL_CSCS_CSC_MEASUREMENT_CLIENT_CHARACTERISTIC_CONFIGURATION_INIT[] =               //Mandatory if the Battery Level characteristic properties supports notification
{
    0x00, (_CSCS_HDL_OFFSET_BASE_ + 3),                                                             //Handle
    GATT_DESCRIPTORS, GATT_DESCRIPTORS_CLIENT_CHARACTERISTIC_CONFIGURATION,                         //Attribute Type
    SIZE_OF_ATTRIBUTE_VALUE_2,
    0x00,                                                                                           //Attribute Value
    0x00                                                                                            //Attribute Value,
};


Uint8 att_HDL_CSCS_CSC_MEASUREMENT_CLIENT_CHARACTERISTIC_CONFIGURATION[] =
{
    //0x00, (_CSCS_HDL_OFFSET_BASE_+3),                                                             //Handle
    //GATT_DESCRIPTORS, GATT_DESCRIPTORS_CLIENT_CHARACTERISTIC_CONFIGURATION,                       //Attribute Type
    //SIZE_OF_ATTRIBUTE_VALUE_2,
    (
        //GATT_DESCRIPTORS_CLIENT_CHARACTERISTIC_CONFIGURATION_NOTIFICATION |
        //GATT_DESCRIPTORS_CLIENT_CHARACTERISTIC_CONFIGURATION_INDICATION |
        0x00
    ),                                                                                              //Attribute Value, R/W: Mandatory
    0x00                                                                                            //Attribute Value,
};


const Uint8 ATT_HDL_CSCS_CHARACTERISTIC_CSC_FEATURE[] =
{
    0x00, (_CSCS_HDL_OFFSET_BASE_ + 4),                                                             //Handle
    GATT_DECLARATIONS, GATT_DECLARATIONS_CHARACTERISTIC,                                            //Attribute Type
    SIZE_OF_ATTRIBUTE_VALUE_5,
    //0x11,
    (
        //Characteristic Properties
        //GATT_DECLARATIONS_PROPERTIES_BROADCAST |
        GATT_DECLARATIONS_PROPERTIES_READ |
        //GATT_DECLARATIONS_PROPERTIES_WRITE_WITHOUT_RESPONSE |
        //GATT_DECLARATIONS_PROPERTIES_WRITE |
        //GATT_DECLARATIONS_PROPERTIES_NOTIFY |
        //GATT_DECLARATIONS_PROPERTIES_INDICATE |
        //GATT_DECLARATIONS_PROPERTIES_AUTHENTICATED_SIGNED_WRITES |
        //GATT_DECLARATIONS_PROPERTIES_EXTENDED_PROPERTIES |
        0x00
    ),
    (_CSCS_HDL_OFFSET_BASE_ + 5), 0x00,
    GATT_SPEC_CHARACTERISTIC_CSC_FEATURE, GATT_SPEC_CHARACTERISTIC                                  //Attribute Value
};


const Uint8 ATT_HDL_CSCS_CSC_FEATURE_INIT[] =
{
    0x00, (_CSCS_HDL_OFFSET_BASE_ + 5),                                                             //Handle
    GATT_SPEC_CHARACTERISTIC, GATT_SPEC_CHARACTERISTIC_CSC_FEATURE,                                 //Attribute Type
    SIZE_OF_ATTRIBUTE_VALUE_2,
    (
        CSCS_FEAT_WHEEL_REVOLUTION_DATA_SUPPORTED |
        CSCS_FEAT_CRANK_REVOLUTION_DATA_SUPPORTED |
        //CSCS_FEAT_MULTIPLE_SENSOR_LOCATIONS_SUPPORTED |
        0x00
    ),
    0x00
};


const Uint8 ATT_HDL_CSCS_CHARACTERISTIC_SC_CONTROL_POINT[] =
{
    0x00, (_CSCS_HDL_OFFSET_BASE_ + 6),                                                             //Handle
    GATT_DECLARATIONS, GATT_DECLARATIONS_CHARACTERISTIC,                                            //Attribute Type
    SIZE_OF_ATTRIBUTE_VALUE_5,
    //0x11,
    (
        //Characteristic Properties
        //GATT_DECLARATIONS_PROPERTIES_BROADCAST |
        //GATT_DECLARATIONS_PROPERTIES_READ |
        //GATT_DECLARATIONS_PROPERTIES_WRITE_WITHOUT_RESPONSE |
        GATT_DECLARATIONS_PROPERTIES_WRITE |
        //GATT_DECLARATIONS_PROPERTIES_NOTIFY |
        GATT_DECLARATIONS_PROPERTIES_INDICATE |
        //GATT_DECLARATIONS_PROPERTIES_AUTHENTICATED_SIGNED_WRITES |
        //GATT_DECLARATIONS_PROPERTIES_EXTENDED_PROPERTIES |
        0x00
    ),
    (_CSCS_HDL_OFFSET_BASE_ + 7), 0x00,
    GATT_SPEC_CHARACTERISTIC_SC_CONTROL_POINT, GATT_SPEC_CHARACTERISTIC                             //Attribute Value
};


const Uint8 ATT_HDL_CSCS_SC_CONTROL_POINT_INIT[] =
{
    0x00, (_CSCS_HDL_OFFSET_BASE_ + 7),                                                             //Handle
    GATT_SPEC_CHARACTERISTIC, GATT_SPEC_CHARACTERISTIC_SC_CONTROL_POINT,                            //Attribute Type
    SIZE_OF_ATTRIBUTE_VALUE_5,
    0x00,                                                                                           //Op Code: Reserved
    0x00,                                                                                           //Cumulative Value: Wheel Revolutions B0 / Request Op Code
    0x00,                                                                                           //Cumulative Value: Wheel Revolutions B1 / Response Value
    0x00,                                                                                           //Cumulative Value: Wheel Revolutions B2 / Not used
    0x00,                                                                                           //Cumulative Value: Wheel Revolutions B3 / Not used
};


Uint8 att_HDL_CSCS_SC_CONTROL_POINT[] =
{
    //0x00, (_CSCS_HDL_OFFSET_BASE_+7),                                                             //Handle
    //GATT_SPEC_CHARACTERISTIC, GATT_SPEC_CHARACTERISTIC_RSC_MEASUREMENT,                           //Attribute Type
    //SIZE_OF_ATTRIBUTE_VALUE_5,
    0x00,                                                                                           //Op Code: Reserved
    0x00,                                                                                           //Cumulative Value: Wheel Revolutions B0 / Request Op Code
    0x00,                                                                                           //Cumulative Value: Wheel Revolutions B1 / Response Value
    0x00,                                                                                           //Cumulative Value: Wheel Revolutions B2 / Not used
    0x00,                                                                                           //Cumulative Value: Wheel Revolutions B3 / Not used
};


const Uint8 ATT_HDL_CSCS_SC_CONTROL_POINT_CLIENT_CHARACTERISTIC_CONFIGURATION_INIT[] =              //Mandatory if the Battery Level characteristic properties supports notification
{
    0x00, (_CSCS_HDL_OFFSET_BASE_ + 8),                                                             //Handle
    GATT_DESCRIPTORS, GATT_DESCRIPTORS_CLIENT_CHARACTERISTIC_CONFIGURATION,                         //Attribute Type
    SIZE_OF_ATTRIBUTE_VALUE_2,
    0x00,                                                                                           //Attribute Value
    0x00                                                                                            //Attribute Value,
};


Uint8 att_HDL_CSCS_SC_CONTROL_POINT_CLIENT_CHARACTERISTIC_CONFIGURATION[] =
{
    //0x00, (_CSCS_HDL_OFFSET_BASE_+8),                                                             //Handle
    //GATT_DESCRIPTORS, GATT_DESCRIPTORS_CLIENT_CHARACTERISTIC_CONFIGURATION,                       //Attribute Type
    //SIZE_OF_ATTRIBUTE_VALUE_2,
    (
        //GATT_DESCRIPTORS_CLIENT_CHARACTERISTIC_CONFIGURATION_NOTIFICATION |
        //GATT_DESCRIPTORS_CLIENT_CHARACTERISTIC_CONFIGURATION_INDICATION |
        0x00
    ),                                                                                              //Attribute Value, R/W: Mandatory
    0x00                                                                                            //Attribute Value,
};
#endif


#ifdef _PROFILE_LNS_
const Uint8 ATT_HDL_LNS_PRIMARY_SERVICE[] =
{
    0x00, _LNS_HDL_OFFSET_BASE_,
    GATT_DECLARATIONS, GATT_DECLARATIONS_PRIMARY_SERVICE,
    SIZE_OF_ATTRIBUTE_VALUE_2,
    GATT_SPEC_SERVICES_LOCATION_AND_NAVIGATION, GATT_SPEC_SERVICES
};


const Uint8 ATT_HDL_LNS_CHARACTERISTIC_LOCATION_AND_SPEED[] =
{
    0x00, (_LNS_HDL_OFFSET_BASE_ + 1),
    GATT_DECLARATIONS, GATT_DECLARATIONS_CHARACTERISTIC,
    SIZE_OF_ATTRIBUTE_VALUE_5,
    (
        //GATT_DECLARATIONS_PROPERTIES_BROADCAST |
        //GATT_DECLARATIONS_PROPERTIES_READ |
        //GATT_DECLARATIONS_PROPERTIES_WRITE_WITHOUT_RESPONSE |
        //GATT_DECLARATIONS_PROPERTIES_WRITE |
        GATT_DECLARATIONS_PROPERTIES_NOTIFY |
        //GATT_DECLARATIONS_PROPERTIES_INDICATE |
        //GATT_DECLARATIONS_PROPERTIES_AUTHENTICATED_SIGNED_WRITES |
        //GATT_DECLARATIONS_PROPERTIES_EXTENDED_PROPERTIES |
        0x00
    ),
    (_LNS_HDL_OFFSET_BASE_ + 2), 0x00,
    GATT_SPEC_CHARACTERISTIC_LOCATION_AND_SPEED, GATT_SPEC_CHARACTERISTIC
};


const Uint8 ATT_HDL_LNS_LOCATION_AND_SPEED_INIT[] =
{
    0x00, (_LNS_HDL_OFFSET_BASE_ + 2),
    GATT_SPEC_CHARACTERISTIC, GATT_SPEC_CHARACTERISTIC_LOCATION_AND_SPEED,
    SIZE_OF_ATTRIBUTE_VALUE_15,
    0x00, 0x00,                                                                                     //Flags
    0x00, 0x00,                                                                                     //Instantaneous Speed
    0x00, 0x00, 0x00,                                                                               //Total Distance
    0x00, 0x00, 0x00, 0x00,                                                                         //Location - Latitude
    0x00, 0x00, 0x00, 0x00,                                                                         //Location - Longitude
    /*
    0x00, 0x00, 0x00,                                                                               //Elevation
    0x00, 0x00,                                                                                     //Heading
    0x00,                                                                                           //Rolling Time
    0x00, 0x00,                                                                                     //UTC Time: Year
    0x00,                                                                                           //UTC Time: Month
    0x00,                                                                                           //UTC Time: Day
    0x00,                                                                                           //UTC Time: Hours
    0x00,                                                                                           //UTC Time: Minutes
    0x00,                                                                                           //UTC Time: Seconds
    */
};


Uint8 att_HDL_LNS_LOCATION_AND_SPEED[] =
{
    //0x00, (_LNS_HDL_OFFSET_BASE_+2),
    //GATT_SPEC_CHARACTERISTIC, GATT_SPEC_CHARACTERISTIC_LOCATION_AND_SPEED,
    //SIZE_OF_ATTRIBUTE_VALUE_15,
    0x00, 0x00,                                                                                     //Flags
    0x00, 0x00,                                                                                     //Instantaneous Speed
    0x00, 0x00, 0x00,                                                                               //Total Distance
    0x00, 0x00, 0x00, 0x00,                                                                         //Location - Latitude
    0x00, 0x00, 0x00, 0x00,                                                                         //Location - Longitude
    /*
    0x00, 0x00, 0x00,                                                                               //Elevation
    0x00, 0x00,                                                                                     //Heading
    0x00,                                                                                           //Rolling Time
    0x00, 0x00,                                                                                     //UTC Time: Year
    0x00,                                                                                           //UTC Time: Month
    0x00,                                                                                           //UTC Time: Day
    0x00,                                                                                           //UTC Time: Hours
    0x00,                                                                                           //UTC Time: Minutes
    0x00,                                                                                           //UTC Time: Seconds
    */
};


const Uint8 ATT_HDL_LNS_LOCATION_AND_SPEED_CLIENT_CHARACTERISTIC_CONFIGURATION_INIT[] =
{
    0x00, (_LNS_HDL_OFFSET_BASE_ + 3),
    GATT_DESCRIPTORS, GATT_DESCRIPTORS_CLIENT_CHARACTERISTIC_CONFIGURATION,
    SIZE_OF_ATTRIBUTE_VALUE_2,
    0x00,
    0x00
};


Uint8 att_HDL_LNS_LOCATION_AND_SPEED_CLIENT_CHARACTERISTIC_CONFIGURATION[] =
{
    //0x00, (_LNS_HDL_OFFSET_BASE_+3),
    //GATT_DESCRIPTORS, GATT_DESCRIPTORS_CLIENT_CHARACTERISTIC_CONFIGURATION,
    //SIZE_OF_ATTRIBUTE_VALUE_2,
    (
        //GATT_DESCRIPTORS_CLIENT_CHARACTERISTIC_CONFIGURATION_NOTIFICATION |
        //GATT_DESCRIPTORS_CLIENT_CHARACTERISTIC_CONFIGURATION_INDICATION |
        0x00
    ),
    0x00
};


const Uint8 ATT_HDL_LNS_CHARACTERISTIC_LN_FEATURE[] =
{
    0x00, (_LNS_HDL_OFFSET_BASE_ + 4),
    GATT_DECLARATIONS, GATT_DECLARATIONS_CHARACTERISTIC,
    SIZE_OF_ATTRIBUTE_VALUE_5,
    //0x11,
    (
        //GATT_DECLARATIONS_PROPERTIES_BROADCAST |
        GATT_DECLARATIONS_PROPERTIES_READ |
        //GATT_DECLARATIONS_PROPERTIES_WRITE_WITHOUT_RESPONSE |
        //GATT_DECLARATIONS_PROPERTIES_WRITE |
        //GATT_DECLARATIONS_PROPERTIES_NOTIFY |
        //GATT_DECLARATIONS_PROPERTIES_INDICATE |
        //GATT_DECLARATIONS_PROPERTIES_AUTHENTICATED_SIGNED_WRITES |
        //GATT_DECLARATIONS_PROPERTIES_EXTENDED_PROPERTIES |
        0x00
    ),
    (_LNS_HDL_OFFSET_BASE_ + 5), 0x00,
    GATT_SPEC_CHARACTERISTIC_LN_FEATURE, GATT_SPEC_CHARACTERISTIC
};


const Uint8 ATT_HDL_LNS_LN_FEATURE_INIT[] =
{
    0x00, (_LNS_HDL_OFFSET_BASE_ + 5),
    GATT_SPEC_CHARACTERISTIC, GATT_SPEC_CHARACTERISTIC_LN_FEATURE,
    SIZE_OF_ATTRIBUTE_VALUE_4,
    (
        //Bit Field
        LNS_LN_FEAT_INSTANTANEOUS_SPEED_SUPPORTED |
        LNS_LN_FEAT_TOTAL_DISTANCE_SUPPORTED |
        LNS_LN_FEAT_LOCATION_SUPPORTED |
        LNS_LN_FEAT_ELEVATION_SUPPORTED |
        LNS_LN_FEAT_HEADING_SUPPORTED |
        LNS_LN_FEAT_ROLLING_TIME_SUPPORTED |
        LNS_LN_FEAT_UTC_TIME_SUPPORTED |
        LNS_LN_FEAT_REMAINING_DISTANCE_SUPPORTED |
        0x00
    ),
    (
        LNS_LN_FEAT_REMAINING_VERTICAL_DISTANCE_SUPPORTED |
        LNS_LN_FEAT_ESTIMATED_TIME_OF_ARRIVAL_SUPPORTED |
        LNS_LN_FEAT_NUM_OF_BEACONS_IN_SOLUTION_SUPPORTED |
        LNS_LN_FEAT_NUM_OF_BEACONS_IN_VIEW_SUPPORTED |
        LNS_LN_FEAT_TIME_TO_FIRST_FIX_SUPPORTED |
        LNS_LN_FEAT_ESTIM_HORIZONTAL_POSITION_ERROR_SUPPORTED |
        LNS_LN_FEAT_ESTIM_VERTICAL_POSITION_ERROR_SUPPORTED |
        LNS_LN_FEAT_HORIZONTAL_DILUTION_OF_PRECISION_SUPPORTED |
        0x00
    ),
    (
        LNS_LN_FEAT_VERTICAL_DILUTION_OF_PRECISION_SUPPORTED |
        LNS_LN_FEAT_LOCAT_AND_SPD_CHARCT_CONTENT_MASKING_SUPPORTED |
        LNS_LN_FEAT_FIX_RATE_SETTING_SUPPORTED |
        LNS_LN_FEAT_ELEVATION_SETTING_SUPPORTED |
        LNS_LN_FEAT_POSITION_STATUS_SUPPORTED |
        0x00
    ),
    0x00                                                                                            //Reserved for future use
};


const Uint8 ATT_HDL_LNS_CHARACTERISTIC_POSITION_QUALITY[] =
{
    0x00, (_LNS_HDL_OFFSET_BASE_ + 6),                                                              //Handle
    GATT_DECLARATIONS, GATT_DECLARATIONS_CHARACTERISTIC,                                            //Attribute Type
    SIZE_OF_ATTRIBUTE_VALUE_5,
    //0x11,
    (
        //Characteristic Properties
        //GATT_DECLARATIONS_PROPERTIES_BROADCAST |
        GATT_DECLARATIONS_PROPERTIES_READ |
        //GATT_DECLARATIONS_PROPERTIES_WRITE_WITHOUT_RESPONSE |
        //GATT_DECLARATIONS_PROPERTIES_WRITE |
        //GATT_DECLARATIONS_PROPERTIES_NOTIFY |
        //GATT_DECLARATIONS_PROPERTIES_INDICATE |
        //GATT_DECLARATIONS_PROPERTIES_AUTHENTICATED_SIGNED_WRITES |
        //GATT_DECLARATIONS_PROPERTIES_EXTENDED_PROPERTIES |
        0x00
    ),
    (_LNS_HDL_OFFSET_BASE_ + 7), 0x00,
    GATT_SPEC_CHARACTERISTIC_POSITION_QUALITY, GATT_SPEC_CHARACTERISTIC                             //Attribute Value
};


const Uint8 ATT_HDL_LNS_POSITION_QUALITY_INIT[] =
{
    0x00, (_LNS_HDL_OFFSET_BASE_ + 7),                                                              //Handle
    GATT_SPEC_CHARACTERISTIC, GATT_SPEC_CHARACTERISTIC_POSITION_QUALITY,                            //Attribute Type
#if defined _PROFILE_LNS_CR_BV_02_
    SIZE_OF_ATTRIBUTE_VALUE_2,
    0x00, 0x00,                                                                                     //Flags
#elif defined _PROFILE_LNS_CR_BV_03_
    SIZE_OF_ATTRIBUTE_VALUE_3,
    0x01, 0x00,                                                                                     //Flags
    0x00,                                                                                           //Number of Beacons in Solution
#elif defined _PROFILE_LNS_CR_BV_04_
    SIZE_OF_ATTRIBUTE_VALUE_3,
    0x02, 0x00,                                                                                     //Flags
    0x00,                                                                                           //Number of Beacons in View
#elif defined _PROFILE_LNS_CR_BV_05_
    SIZE_OF_ATTRIBUTE_VALUE_4,
    0x04, 0x00,                                                                                     //Flags
    0x00, 0x00,                                                                                     //Time to First Fix
#elif defined _PROFILE_LNS_CR_BV_06_
    SIZE_OF_ATTRIBUTE_VALUE_6,
    0x08, 0x00,                                                                                     //Flags
    0x00, 0x00, 0x00, 0x00,                                                                         //EHPE
#elif defined _PROFILE_LNS_CR_BV_07_
    SIZE_OF_ATTRIBUTE_VALUE_6,
    0x10, 0x00,                                                                                     //Flags
    0x00, 0x00, 0x00, 0x00,                                                                         //EVPE
#elif defined _PROFILE_LNS_CR_BV_08_
    SIZE_OF_ATTRIBUTE_VALUE_3,
    0x20, 0x00,                                                                                     //Flags
    0x00,                                                                                           //HDOP
#elif defined _PROFILE_LNS_CR_BV_09_
    SIZE_OF_ATTRIBUTE_VALUE_3,
    0x40, 0x00,                                                                                     //Flags
    0x00                                                                                            //VDOP
#else
    SIZE_OF_ATTRIBUTE_VALUE_16,
    0x00, 0x00,                                                                                     //Flags
    0x00,                                                                                           //Number of Beacons in Solution
    0x00,                                                                                           //Number of Beacons in View
    0x00, 0x00,                                                                                     //Time to First Fix
    0x00, 0x00, 0x00, 0x00,                                                                         //EHPE
    0x00, 0x00, 0x00, 0x00,                                                                         //EVPE
    0x00,                                                                                           //HDOP
    0x00                                                                                            //VDOP
#endif
};


Uint8 att_HDL_LNS_POSITION_QUALITY[] =
{
    //0x00, (_LNS_HDL_OFFSET_BASE_+7),                                                              //Handle
    //GATT_SPEC_CHARACTERISTIC, GATT_SPEC_CHARACTERISTIC_POSITION_QUALITY,                          //Attribute Type
    //SIZE_OF_ATTRIBUTE_VALUE_16,
    0x00, 0x00,                                                                                     //Flags
    0x00,                                                                                           //Number of Beacons in Solution
    0x00,                                                                                           //Number of Beacons in View
    0x00, 0x00,                                                                                     //Time to First Fix
    0x00, 0x00, 0x00, 0x00,                                                                         //EHPE
    0x00, 0x00, 0x00, 0x00,                                                                         //EVPE
    0x00,                                                                                           //HDOP
    0x00                                                                                            //VDOP
};


const Uint8 ATT_HDL_LNS_CHARACTERISTIC_LN_CONTROL_POINT[] =
{
    0x00, (_LNS_HDL_OFFSET_BASE_ + 8),                                                              //Handle
    GATT_DECLARATIONS, GATT_DECLARATIONS_CHARACTERISTIC,                                            //Attribute Type
    SIZE_OF_ATTRIBUTE_VALUE_5,
    //0x11,
    (
        //Characteristic Properties
        //GATT_DECLARATIONS_PROPERTIES_BROADCAST |
        //GATT_DECLARATIONS_PROPERTIES_READ |
        //GATT_DECLARATIONS_PROPERTIES_WRITE_WITHOUT_RESPONSE |
        GATT_DECLARATIONS_PROPERTIES_WRITE |
        //GATT_DECLARATIONS_PROPERTIES_NOTIFY |
        GATT_DECLARATIONS_PROPERTIES_INDICATE |
        //GATT_DECLARATIONS_PROPERTIES_AUTHENTICATED_SIGNED_WRITES |
        //GATT_DECLARATIONS_PROPERTIES_EXTENDED_PROPERTIES |
        0x00
    ),
    (_LNS_HDL_OFFSET_BASE_ + 9), 0x00,
    GATT_SPEC_CHARACTERISTIC_LN_CONTROL_POINT, GATT_SPEC_CHARACTERISTIC                             //Attribute Value
};


const Uint8 ATT_HDL_LNS_LN_CONTROL_POINT_INIT[] =
{
    0x00, (_LNS_HDL_OFFSET_BASE_ + 9),                                                              //Handle
    GATT_SPEC_CHARACTERISTIC, GATT_SPEC_CHARACTERISTIC_LN_CONTROL_POINT,                            //Attribute Type
    SIZE_OF_ATTRIBUTE_VALUE_5,
    0x00,                                                                                           //Op Code: Reserved
    0x00,                                                                                           //Parameter Value, Defined per Service specification
    0x00,                                                                                           //Request Op Code
    0x00,                                                                                           //Response Value
    0x00,                                                                                           //Response Parameter, Note: The Response Parameter Value of the response to the Control Point is a variable length field to allow a list of different values defined by the Service Specification.
};


Uint8 att_HDL_LNS_LN_CONTROL_POINT[] =
{
    //0x00, (_LNS_HDL_OFFSET_BASE_+9),                                                              //Handle
    //GATT_SPEC_CHARACTERISTIC, GATT_SPEC_CHARACTERISTIC_LN_CONTROL_POINT,                          //Attribute Type
    //SIZE_OF_ATTRIBUTE_VALUE_5,
    0x00,                                                                                           //Op Code: Reserved
    0x00,                                                                                           //Parameter Value, Defined per Service specification
    0x00,                                                                                           //Request Op Code
    0x00,                                                                                           //Response Value
    0x00,                                                                                           //Response Parameter, Note: The Response Parameter Value of the response to the Control Point is a variable length field to allow a list of different values defined by the Service Specification.
};


const Uint8 ATT_HDL_LNS_LN_CONTROL_POINT_CLIENT_CHARACTERISTIC_CONFIGURATION_INIT[] =
{
    0x00, (_LNS_HDL_OFFSET_BASE_ + 10),                                                             //Handle
    GATT_DESCRIPTORS, GATT_DESCRIPTORS_CLIENT_CHARACTERISTIC_CONFIGURATION,                         //Attribute Type
    SIZE_OF_ATTRIBUTE_VALUE_2,
    0x00,                                                                                           //Attribute Value
    0x00                                                                                            //Attribute Value,
};


Uint8 att_HDL_LNS_LN_CONTROL_POINT_CLIENT_CHARACTERISTIC_CONFIGURATION[] =
{
    //0x00, (_LNS_HDL_OFFSET_BASE_+10),                                                             //Handle
    //GATT_DESCRIPTORS, GATT_DESCRIPTORS_CLIENT_CHARACTERISTIC_CONFIGURATION,                       //Attribute Type
    //SIZE_OF_ATTRIBUTE_VALUE_2,
    (
        //GATT_DESCRIPTORS_CLIENT_CHARACTERISTIC_CONFIGURATION_NOTIFICATION |
        //GATT_DESCRIPTORS_CLIENT_CHARACTERISTIC_CONFIGURATION_INDICATION |
        0x00
    ),                                                                                              //Attribute Value
    0x00                                                                                            //Attribute Value,
};


const Uint8 ATT_HDL_LNS_CHARACTERISTIC_NAVIGATION[] =
{
    0x00, (_LNS_HDL_OFFSET_BASE_ + 11),
    GATT_DECLARATIONS, GATT_DECLARATIONS_CHARACTERISTIC,
    SIZE_OF_ATTRIBUTE_VALUE_5,
    (
        //GATT_DECLARATIONS_PROPERTIES_BROADCAST |
        //GATT_DECLARATIONS_PROPERTIES_READ |
        //GATT_DECLARATIONS_PROPERTIES_WRITE_WITHOUT_RESPONSE |
        //GATT_DECLARATIONS_PROPERTIES_WRITE |
        GATT_DECLARATIONS_PROPERTIES_NOTIFY |
        //GATT_DECLARATIONS_PROPERTIES_INDICATE |
        //GATT_DECLARATIONS_PROPERTIES_AUTHENTICATED_SIGNED_WRITES |
        //GATT_DECLARATIONS_PROPERTIES_EXTENDED_PROPERTIES |
        0x00
    ),
    (_LNS_HDL_OFFSET_BASE_ + 12), 0x00,
    GATT_SPEC_CHARACTERISTIC_NAVIGATION, GATT_SPEC_CHARACTERISTIC
};


const Uint8 ATT_HDL_LNS_NAVIGATION_INIT[] =
{
    0x00, (_LNS_HDL_OFFSET_BASE_ + 12),                                                             //Handle
    GATT_SPEC_CHARACTERISTIC, GATT_SPEC_CHARACTERISTIC_NAVIGATION,                                  //Attribute Type
    SIZE_OF_ATTRIBUTE_VALUE_19,
    0x00, 0x00,                                                                                     //Flags
    0x00, 0x00,                                                                                     //Bearing
    0x00, 0x00,                                                                                     //Heading
    0x00, 0x00, 0x00,                                                                               //Remaining Distance
    0x00, 0x00, 0x00,                                                                               //Remaining Vertical Distance
    0x00, 0x00,                                                                                     //UTC Time: Year
    0x00,                                                                                           //UTC Time: Month
    0x00,                                                                                           //UTC Time: Day
    0x00,                                                                                           //UTC Time: Hours
    0x00,                                                                                           //UTC Time: Minutes
    0x00,                                                                                           //UTC Time: Seconds
};


Uint8 att_HDL_LNS_NAVIGATION[] =
{
    //0x00, (_LNS_HDL_OFFSET_BASE_+12),                                                             //Handle
    //GATT_SPEC_CHARACTERISTIC, GATT_SPEC_CHARACTERISTIC_NAVIGATION,                                //Attribute Type
    //SIZE_OF_ATTRIBUTE_VALUE_19,
    0x00, 0x00,                                                                                     //Flags
    0x00, 0x00,                                                                                     //Bearing
    0x00, 0x00,                                                                                     //Heading
    0x00, 0x00, 0x00,                                                                               //Remaining Distance
    0x00, 0x00, 0x00,                                                                               //Remaining Vertical Distance
    0x00, 0x00,                                                                                     //UTC Time: Year
    0x00,                                                                                           //UTC Time: Month
    0x00,                                                                                           //UTC Time: Day
    0x00,                                                                                           //UTC Time: Hours
    0x00,                                                                                           //UTC Time: Minutes
    0x00,                                                                                           //UTC Time: Seconds
};


const Uint8 ATT_HDL_LNS_NAVIGATION_CLIENT_CHARACTERISTIC_CONFIGURATION_INIT[] =
{
    0x00, (_LNS_HDL_OFFSET_BASE_ + 13),                                                             //Handle
    GATT_DESCRIPTORS, GATT_DESCRIPTORS_CLIENT_CHARACTERISTIC_CONFIGURATION,                         //Attribute Type
    SIZE_OF_ATTRIBUTE_VALUE_2,
    0x00,                                                                                           //Attribute Value
    0x00                                                                                            //Attribute Value,
};


Uint8 att_HDL_LNS_NAVIGATION_CLIENT_CHARACTERISTIC_CONFIGURATION[] =
{
    //0x00, (_LNS_HDL_OFFSET_BASE_+13),                                                             //Handle
    //GATT_DESCRIPTORS, GATT_DESCRIPTORS_CLIENT_CHARACTERISTIC_CONFIGURATION,                       //Attribute Type
    //SIZE_OF_ATTRIBUTE_VALUE_2,
    (
        //GATT_DESCRIPTORS_CLIENT_CHARACTERISTIC_CONFIGURATION_NOTIFICATION |
        //GATT_DESCRIPTORS_CLIENT_CHARACTERISTIC_CONFIGURATION_INDICATION |
        0x00
    ),                                                                                              //Attribute Value,
    0x00                                                                                            //Attribute Value,
};
#endif


#ifdef _PROFILE_GLS_
const Uint8 ATT_HDL_GLS_PRIMARY_SERVICE[] =
{
    0x00, _GLS_HDL_OFFSET_BASE_,
    GATT_DECLARATIONS, GATT_DECLARATIONS_PRIMARY_SERVICE,
    SIZE_OF_ATTRIBUTE_VALUE_2,
    GATT_SPEC_SERVICES_GLUCOSE, GATT_SPEC_SERVICES
};


const Uint8 ATT_HDL_GLS_CHARACTERISTIC_GLUCOSE_MEASUREMENT[] =
{
    0x00, (_GLS_HDL_OFFSET_BASE_ + 1),
    GATT_DECLARATIONS, GATT_DECLARATIONS_CHARACTERISTIC,
    SIZE_OF_ATTRIBUTE_VALUE_5,
    (
        //Characteristic Properties
        //GATT_DECLARATIONS_PROPERTIES_BROADCAST |
        //GATT_DECLARATIONS_PROPERTIES_READ |
        //GATT_DECLARATIONS_PROPERTIES_WRITE_WITHOUT_RESPONSE |
        //GATT_DECLARATIONS_PROPERTIES_WRITE |
        GATT_DECLARATIONS_PROPERTIES_NOTIFY |
        //GATT_DECLARATIONS_PROPERTIES_INDICATE |
        //GATT_DECLARATIONS_PROPERTIES_AUTHENTICATED_SIGNED_WRITES |
        //GATT_DECLARATIONS_PROPERTIES_EXTENDED_PROPERTIES |
        0x00
    ),
    (_GLS_HDL_OFFSET_BASE_ + 2), 0x00,
    GATT_SPEC_CHARACTERISTIC_GLUCOSE_MEASUREMENT, GATT_SPEC_CHARACTERISTIC                          //Attribute Value
};


const Uint8 ATT_HDL_GLS_GLUCOSE_MEASUREMENT_INIT[] =
{
    0x00, (_GLS_HDL_OFFSET_BASE_ + 2),                                                              //Handle
    GATT_SPEC_CHARACTERISTIC, GATT_SPEC_CHARACTERISTIC_GLUCOSE_MEASUREMENT,                         //Attribute Type
    SIZE_OF_ATTRIBUTE_VALUE_17,
    0x00,                                                                                           //Flags
    0x00, 0x00,                                                                                     //Sequence Number
    0x00, 0x00,                                                                                     //UTC Time: Year
    0x00,                                                                                           //UTC Time: Month
    0x00,                                                                                           //UTC Time: Day
    0x00,                                                                                           //UTC Time: Hours
    0x00,                                                                                           //UTC Time: Minutes
    0x00,                                                                                           //UTC Time: Seconds
    0x00, 0x00,                                                                                     //Time Offset
    0x00, 0x00,                                                                                     //Glucose Concentration - units of kg/L (0) or mol/L (1)
    0x00,                                                                                           //Type, Sample Location
    0x00, 0x00,                                                                                     //Sensor Status Annunciation
};


Uint8 att_HDL_GLS_GLUCOSE_MEASUREMENT[] =
{
    //0x00, (_GLS_HDL_OFFSET_BASE_+2),                                                              //Handle
    //GATT_SPEC_CHARACTERISTIC, GATT_SPEC_CHARACTERISTIC_GLUCOSE_MEASUREMENT,                       //Attribute Type
    SIZE_OF_ATTRIBUTE_VALUE_17,
    0x00,                                                                                           //Flags
    0x00, 0x00,                                                                                     //Sequence Number
    0x00, 0x00,                                                                                     //UTC Time: Year
    0x00,                                                                                           //UTC Time: Month
    0x00,                                                                                           //UTC Time: Day
    0x00,                                                                                           //UTC Time: Hours
    0x00,                                                                                           //UTC Time: Minutes
    0x00,                                                                                           //UTC Time: Seconds
    0x00, 0x00,                                                                                     //Time Offset
    0x00, 0x00,                                                                                     //Glucose Concentration - units of kg/L (0) or mol/L (1)
    0x00,                                                                                           //Type, Sample Location
    0x00, 0x00,                                                                                     //Sensor Status Annunciation
};


const Uint8 ATT_HDL_GLS_GLUCOSE_MEASUREMENT_CLIENT_CHARACTERISTIC_CONFIGURATION_INIT[] =
{
    0x00, (_GLS_HDL_OFFSET_BASE_ + 3),                                                              //Handle
    GATT_DESCRIPTORS, GATT_DESCRIPTORS_CLIENT_CHARACTERISTIC_CONFIGURATION,                         //Attribute Type
    SIZE_OF_ATTRIBUTE_VALUE_2,
    0x00,                                                                                           //Attribute Value
    0x00                                                                                            //Attribute Value,
};


Uint8 att_HDL_GLS_GLUCOSE_MEASUREMENT_CLIENT_CHARACTERISTIC_CONFIGURATION[] =
{
    //0x00, (_GLS_HDL_OFFSET_BASE_+3),                                                              //Handle
    //GATT_DESCRIPTORS, GATT_DESCRIPTORS_CLIENT_CHARACTERISTIC_CONFIGURATION,                       //Attribute Type
    //SIZE_OF_ATTRIBUTE_VALUE_2,
    (
        //GATT_DESCRIPTORS_CLIENT_CHARACTERISTIC_CONFIGURATION_NOTIFICATION |
        //GATT_DESCRIPTORS_CLIENT_CHARACTERISTIC_CONFIGURATION_INDICATION |
        0x00
    ),                                                                                              //Attribute Value
    0x00                                                                                            //Attribute Value,
};


const Uint8 ATT_HDL_GLS_CHARACTERISTIC_GLUCOSE_MEASUREMENT_CONTEXT[] =
{
    0x00, (_GLS_HDL_OFFSET_BASE_ + 4),
    GATT_DECLARATIONS, GATT_DECLARATIONS_CHARACTERISTIC,
    SIZE_OF_ATTRIBUTE_VALUE_5,
    (
        //Characteristic Properties
        //GATT_DECLARATIONS_PROPERTIES_BROADCAST |
        //GATT_DECLARATIONS_PROPERTIES_READ |
        //GATT_DECLARATIONS_PROPERTIES_WRITE_WITHOUT_RESPONSE |
        //GATT_DECLARATIONS_PROPERTIES_WRITE |
        GATT_DECLARATIONS_PROPERTIES_NOTIFY |
        //GATT_DECLARATIONS_PROPERTIES_INDICATE |
        //GATT_DECLARATIONS_PROPERTIES_AUTHENTICATED_SIGNED_WRITES |
        //GATT_DECLARATIONS_PROPERTIES_EXTENDED_PROPERTIES |
        0x00
    ),
    (_GLS_HDL_OFFSET_BASE_ + 5), 0x00,
    GATT_SPEC_CHARACTERISTIC_GLUCOSE_MEASUREMENT_CONTEXT, GATT_SPEC_CHARACTERISTIC                  //Attribute Value
};


const Uint8 ATT_HDL_GLS_GLUCOSE_MEASUREMENT_CONTEXT_INIT[] =
{
    0x00, (_GLS_HDL_OFFSET_BASE_ + 5),                                                              //Handle
    GATT_SPEC_CHARACTERISTIC, GATT_SPEC_CHARACTERISTIC_GLUCOSE_MEASUREMENT_CONTEXT,                 //Attribute Type
    SIZE_OF_ATTRIBUTE_VALUE_17,
    0x00,                                                                                           //Flags
    0x00, 0x00,                                                                                     //Sequence Number
    0x00,                                                                                           //Extended Flags, Reserved for future use
    0x00,                                                                                           //Carbohydrate ID
    0x00, 0x00,                                                                                     //Carbohydrate - units of kilograms
    0x00,                                                                                           //Meal
    0x00,                                                                                           //Tester, Health
    0x00, 0x00,                                                                                     //Exercise Duration
    0x00,                                                                                           //Exercise Intensity
    0x00,                                                                                           //Medication ID
    0x00, 0x00,                                                                                     //Medication - units of kilograms (0) or liters (1)
    0x00, 0x00,                                                                                     //HbA1c
};


Uint8 att_HDL_GLS_GLUCOSE_MEASUREMENT_CONTEXT[] =
{
    //0x00, (_GLS_HDL_OFFSET_BASE_+5),                                                              //Handle
    //GATT_SPEC_CHARACTERISTIC, GATT_SPEC_CHARACTERISTIC_GLUCOSE_MEASUREMENT_CONTEXT,               //Attribute Type
    //SIZE_OF_ATTRIBUTE_VALUE_17,
    0x00,                                                                                           //Flags
    0x00, 0x00,                                                                                     //Sequence Number
    0x00,                                                                                           //Extended Flags, Reserved for future use
    0x00,                                                                                           //Carbohydrate ID
    0x00, 0x00,                                                                                     //Carbohydrate - units of kilograms
    0x00,                                                                                           //Meal
    0x00,                                                                                           //Tester, Health
    0x00, 0x00,                                                                                     //Exercise Duration
    0x00,                                                                                           //Exercise Intensity
    0x00,                                                                                           //Medication ID
    0x00, 0x00,                                                                                     //Medication - units of kilograms (0) or liters (1)
    0x00, 0x00,                                                                                     //HbA1c
};


const Uint8 ATT_HDL_GLS_GLUCOSE_MEASUREMENT_CONTEXT_CLIENT_CHARACTERISTIC_CONFIGURATION_INIT[] =
{
    0x00, (_GLS_HDL_OFFSET_BASE_ + 6),                                                              //Handle
    GATT_DESCRIPTORS, GATT_DESCRIPTORS_CLIENT_CHARACTERISTIC_CONFIGURATION,                         //Attribute Type
    SIZE_OF_ATTRIBUTE_VALUE_2,
    0x00,                                                                                           //Attribute Value
    0x00                                                                                            //Attribute Value,
};


Uint8 att_HDL_GLS_GLUCOSE_MEASUREMENT_CONTEXT_CLIENT_CHARACTERISTIC_CONFIGURATION[] =
{
    //0x00, (_GLS_HDL_OFFSET_BASE_+6),                                                              //Handle
    //GATT_DESCRIPTORS, GATT_DESCRIPTORS_CLIENT_CHARACTERISTIC_CONFIGURATION,                       //Attribute Type
    //SIZE_OF_ATTRIBUTE_VALUE_2,
    (
        //GATT_DESCRIPTORS_CLIENT_CHARACTERISTIC_CONFIGURATION_NOTIFICATION |
        //GATT_DESCRIPTORS_CLIENT_CHARACTERISTIC_CONFIGURATION_INDICATION |
        0x00
    ),                                                                                              //Attribute Value, R/W: Mandatory
    0x00                                                                                            //Attribute Value,
};


const Uint8 ATT_HDL_GLS_CHARACTERISTIC_GLUCOSE_FEATURE[] =
{
    0x00, (_GLS_HDL_OFFSET_BASE_ + 7),                                                              //Handle
    GATT_DECLARATIONS, GATT_DECLARATIONS_CHARACTERISTIC,                                            //Attribute Type
    SIZE_OF_ATTRIBUTE_VALUE_5,
    //0x11,
    (
        //Characteristic Properties
        //GATT_DECLARATIONS_PROPERTIES_BROADCAST |
        GATT_DECLARATIONS_PROPERTIES_READ |
        //GATT_DECLARATIONS_PROPERTIES_WRITE_WITHOUT_RESPONSE |
        //GATT_DECLARATIONS_PROPERTIES_WRITE |
        //GATT_DECLARATIONS_PROPERTIES_NOTIFY |
        //GATT_DECLARATIONS_PROPERTIES_INDICATE |
        //GATT_DECLARATIONS_PROPERTIES_AUTHENTICATED_SIGNED_WRITES |
        //GATT_DECLARATIONS_PROPERTIES_EXTENDED_PROPERTIES |
        0x00
    ),
    (_GLS_HDL_OFFSET_BASE_ + 8), 0x00,
    GATT_SPEC_CHARACTERISTIC_GLUCOSE_FEATURE, GATT_SPEC_CHARACTERISTIC                              //Attribute Value
};


const Uint8 ATT_HDL_GLS_GLUCOSE_FEATURE_INIT[] =
{
    0x00, (_GLS_HDL_OFFSET_BASE_ + 8),                                                              //Handle
    GATT_SPEC_CHARACTERISTIC, GATT_SPEC_CHARACTERISTIC_GLUCOSE_FEATURE,                             //Attribute Type
    SIZE_OF_ATTRIBUTE_VALUE_2,
    (
        //Bit Field
        GLS_FEAT_LOW_BATTERY_DETECTION_DURING_MEAS_SUPPORTED |
        GLS_FEAT_SENSOR_MALFUNCTION_DETECTION_SUPPORTED |
        GLS_FEAT_SENSOR_SAMPLE_SIZE_SUPPORTED |
        GLS_FEAT_SENSOR_STRIP_INSERTION_ERROR_DETECTION_SUPPORTED |
        GLS_FEAT_SENSOR_STRIP_TYPE_ERROR_DETECTION_SUPPORTED |
        GLS_FEAT_SENSOR_RESULT_HIGHLOW_DETECTION_SUPPORTED |
        GLS_FEAT_SENSOR_TEMPERATURE_HIGHLOW_DETECTION_SUPPORTED |
        GLS_FEAT_SENSOR_READ_INTERRUPT_DETECTION_SUPPORTED |
        0x00
    ),
    (
        GLS_FEAT_GENERAL_DEVICE_FAULT_SUPPORTED |
        GLS_FEAT_TIME_FAULT_SUPPORTED |
        GLS_FEAT_MULTIPLE_BOND_SUPPORTED |
        0x00
    ),
};


const Uint8 ATT_HDL_GLS_CHARACTERISTIC_RECORD_ACCESS_CONTROL_POINT[] =
{
    0x00, (_GLS_HDL_OFFSET_BASE_ + 9),                                                              //Handle
    GATT_DECLARATIONS, GATT_DECLARATIONS_CHARACTERISTIC,                                            //Attribute Type
    SIZE_OF_ATTRIBUTE_VALUE_5,
    //0x11,
    (
        //Characteristic Properties
        //GATT_DECLARATIONS_PROPERTIES_BROADCAST |
        //GATT_DECLARATIONS_PROPERTIES_READ |
        //GATT_DECLARATIONS_PROPERTIES_WRITE_WITHOUT_RESPONSE |
        GATT_DECLARATIONS_PROPERTIES_WRITE |
        //GATT_DECLARATIONS_PROPERTIES_NOTIFY |
        GATT_DECLARATIONS_PROPERTIES_INDICATE |
        //GATT_DECLARATIONS_PROPERTIES_AUTHENTICATED_SIGNED_WRITES |
        //GATT_DECLARATIONS_PROPERTIES_EXTENDED_PROPERTIES |
        0x00
    ),
    (_GLS_HDL_OFFSET_BASE_ + 10), 0x00,
    GATT_SPEC_CHARACTERISTIC_RECORD_ACCESS_CONTROL_POINT, GATT_SPEC_CHARACTERISTIC                  //Attribute Value
};


const Uint8 ATT_HDL_GLS_RECORD_ACCESS_CONTROL_POINT_INIT[] =
{
    0x00, (_GLS_HDL_OFFSET_BASE_ + 10),                                                             //Handle
    GATT_SPEC_CHARACTERISTIC, GATT_SPEC_CHARACTERISTIC_RECORD_ACCESS_CONTROL_POINT,                 //Attribute Type
    SIZE_OF_ATTRIBUTE_VALUE_7,
    0x00,                                                                                           //Op Code
    0x00,                                                                                           //Operator
    0x00,                                                                                           //Operand, Filter Type
    0x00, 0x00, 0x00, 0x00                                                                          //Operand, Filter Parameters
};


Uint8 att_HDL_GLS_RECORD_ACCESS_CONTROL_POINT[] =
{
    //0x00, (_GLS_HDL_OFFSET_BASE_+10),                                                             //Handle
    //GATT_SPEC_CHARACTERISTIC, GATT_SPEC_CHARACTERISTIC_RECORD_ACCESS_CONTROL_POINT,               //Attribute Type
    //SIZE_OF_ATTRIBUTE_VALUE_7,
    0x00,                                                                                           //Op Code
    0x00,                                                                                           //Operator
    0x00,                                                                                           //Operand, Filter Type
    0x00, 0x00, 0x00, 0x00                                                                          //Operand, Filter Parameters
};


const Uint8 ATT_HDL_GLS_RECORD_ACCESS_CONTROL_POINT_CLIENT_CHARACTERISTIC_CONFIGURATION_INIT[] =
{
    0x00, (_GLS_HDL_OFFSET_BASE_ + 11),                                                             //Handle
    GATT_DESCRIPTORS, GATT_DESCRIPTORS_CLIENT_CHARACTERISTIC_CONFIGURATION,                         //Attribute Type
    SIZE_OF_ATTRIBUTE_VALUE_2,
    0x00,                                                                                           //Attribute Value
    0x00                                                                                            //Attribute Value,
};


Uint8 att_HDL_GLS_RECORD_ACCESS_CONTROL_POINT_CLIENT_CHARACTERISTIC_CONFIGURATION[] =
{
    //0x00, (_GLS_HDL_OFFSET_BASE_+11),                                                             //Handle
    //GATT_DESCRIPTORS, GATT_DESCRIPTORS_CLIENT_CHARACTERISTIC_CONFIGURATION,                       //Attribute Type
    //SIZE_OF_ATTRIBUTE_VALUE_2,
    (
        //GATT_DESCRIPTORS_CLIENT_CHARACTERISTIC_CONFIGURATION_NOTIFICATION |
        //GATT_DESCRIPTORS_CLIENT_CHARACTERISTIC_CONFIGURATION_INDICATION |
        0x00
    ),                                                                                              //Attribute Value
    0x00                                                                                            //Attribute Value,
};
#endif


#ifdef _PROFILE_CPS_
const Uint8 ATT_HDL_CPS_PRIMARY_SERVICE[] =
{
    0x00, _CPS_HDL_OFFSET_BASE_,
    GATT_DECLARATIONS, GATT_DECLARATIONS_PRIMARY_SERVICE,
    SIZE_OF_ATTRIBUTE_VALUE_2,
    GATT_SPEC_SERVICES_CYCLING_POWER, GATT_SPEC_SERVICES
};


const Uint8 ATT_HDL_CPS_CHARACTERISTIC_CYCLING_POWER_MEASUREMENT[] =
{
    0x00, (_CPS_HDL_OFFSET_BASE_ + 1),
    GATT_DECLARATIONS, GATT_DECLARATIONS_CHARACTERISTIC,
    SIZE_OF_ATTRIBUTE_VALUE_5,
    (
        //Characteristic Properties
        GATT_DECLARATIONS_PROPERTIES_BROADCAST |
        //GATT_DECLARATIONS_PROPERTIES_READ |
        //GATT_DECLARATIONS_PROPERTIES_WRITE_WITHOUT_RESPONSE |
        //GATT_DECLARATIONS_PROPERTIES_WRITE |
        GATT_DECLARATIONS_PROPERTIES_NOTIFY |
        //GATT_DECLARATIONS_PROPERTIES_INDICATE |
        //GATT_DECLARATIONS_PROPERTIES_AUTHENTICATED_SIGNED_WRITES |
        //GATT_DECLARATIONS_PROPERTIES_EXTENDED_PROPERTIES |
        0x00
    ),
    (_CPS_HDL_OFFSET_BASE_ + 2), 0x00,
    GATT_SPEC_CHARACTERISTIC_CYCLING_POWER_MEASUREMENT, GATT_SPEC_CHARACTERISTIC                    //Attribute Value
};


const Uint8 ATT_HDL_CPS_CYCLING_POWER_MEASUREMENT_INIT[] =
{
    0x00, (_CPS_HDL_OFFSET_BASE_ + 2),                                                              //Handle
    GATT_SPEC_CHARACTERISTIC, GATT_SPEC_CHARACTERISTIC_CYCLING_POWER_MEASUREMENT,                   //Attribute Type
    SIZE_OF_ATTRIBUTE_VALUE_20,
    0x00, 0x00,                                                                                     //Flags
    0x00, 0x00,                                                                                     //Instantaneous Power
    0x00,                                                                                           //Pedal Power Balance
    0x00, 0x00,                                                                                     //Accumulated Torque

    0x00, 0x00, 0x00, 0x00,                                                                         //Wheel Revolution Data - Cumulative Wheel Revolutions
    0x00, 0x00,                                                                                     //Wheel Revolution Data - Last Wheel Event Time

    0x00, 0x00,                                                                                     //Crank Revolution Data- Cumulative Crank Revolutions
    0x00, 0x00,                                                                                     //Crank Revolution Data- Last Crank Event Time

    0x00, 0x00,                                                                                     //Extreme Force Magnitudes - Maximum Force Magnitude
    0x00, 0x00,                                                                                     //Extreme Force Magnitudes - Minimum Force Magnitude

    0x00, 0x00,                                                                                     //Extreme Torque Magnitudes- Maximum Torque Magnitude
    0x00, 0x00,                                                                                     //Extreme Torque Magnitudes- Minimum Torque Magnitude

    0x00, 0x00, 0x00,                                                                               //Extreme Angles - Maximum & Minimum Angle
    0x00, 0x00,                                                                                     //Top Dead Spot Angle
    0x00, 0x00,                                                                                     //Bottom Dead Spot Angle
    0x00, 0x00,                                                                                     //Accumulated Energy
};


Uint8 att_HDL_CPS_CYCLING_POWER_MEASUREMENT[] =
{
    //0x00, (_CPS_HDL_OFFSET_BASE_+2),                                                              //Handle
    //GATT_SPEC_CHARACTERISTIC, GATT_SPEC_CHARACTERISTIC_BATTERY_LEVEL,                             //Attribute Type
    //SIZE_OF_ATTRIBUTE_VALUE_20,
    0x00, 0x00,                                                                                     //Flags
    0x00, 0x00,                                                                                     //Instantaneous Power

    0x00,                                                                                           //Pedal Power Balance

    0x00, 0x00,                                                                                     //Accumulated Torque

    0x00, 0x00, 0x00, 0x00,                                                                         //Wheel Revolution Data - Cumulative Wheel Revolutions
    0x00, 0x00,                                                                                     //Wheel Revolution Data - Last Wheel Event Time

    0x00, 0x00,                                                                                     //Crank Revolution Data- Cumulative Crank Revolutions
    0x00, 0x00,                                                                                     //Crank Revolution Data- Last Crank Event Time

    0x00, 0x00,                                                                                     //Extreme Force Magnitudes - Maximum Force Magnitude
    0x00, 0x00,                                                                                     //Extreme Force Magnitudes - Minimum Force Magnitude

    0x00, 0x00,                                                                                     //Extreme Torque Magnitudes- Maximum Torque Magnitude
    0x00, 0x00,                                                                                     //Extreme Torque Magnitudes- Minimum Torque Magnitude

    0x00, 0x00, 0x00,                                                                               //Extreme Angles - Maximum & Minimum Angle

    0x00, 0x00,                                                                                     //Top Dead Spot Angle

    0x00, 0x00,                                                                                     //Bottom Dead Spot Angle

    0x00, 0x00,                                                                                     //Accumulated Energy
};


const Uint8 ATT_HDL_CPS_CYCLING_POWER_MEASUREMENT_CLIENT_CHARACTERISTIC_CONFIGURATION_INIT[] =
{
    0x00, (_CPS_HDL_OFFSET_BASE_ + 3),                                                              //Handle
    GATT_DESCRIPTORS, GATT_DESCRIPTORS_CLIENT_CHARACTERISTIC_CONFIGURATION,                         //Attribute Type
    SIZE_OF_ATTRIBUTE_VALUE_2,
    0x00,                                                                                           //Attribute Value
    0x00                                                                                            //Attribute Value,
};


Uint8 att_HDL_CPS_CYCLING_POWER_MEASUREMENT_CLIENT_CHARACTERISTIC_CONFIGURATION[] =
{
    //0x00, (_CPS_HDL_OFFSET_BASE_+3),                                                              //Handle
    //GATT_DESCRIPTORS, GATT_DESCRIPTORS_CLIENT_CHARACTERISTIC_CONFIGURATION,                       //Attribute Type
    //SIZE_OF_ATTRIBUTE_VALUE_2,
    (
        //GATT_DESCRIPTORS_CLIENT_CHARACTERISTIC_CONFIGURATION_NOTIFICATION |
        //GATT_DESCRIPTORS_CLIENT_CHARACTERISTIC_CONFIGURATION_INDICATION |
        0x00
    ),                                                                                              //Attribute Value,
    0x00                                                                                            //Attribute Value,
};


const Uint8 ATT_HDL_CPS_CYCLING_POWER_MEASUREMENT_SERVER_CHARACTERISTIC_CONFIGURATION_INIT[] =
{
    0x00, (_CPS_HDL_OFFSET_BASE_ + 4),                                                              //Handle
    GATT_DESCRIPTORS, GATT_DESCRIPTORS_SERVER_CHARACTERISTIC_CONFIGURATION,                         //Attribute Type
    SIZE_OF_ATTRIBUTE_VALUE_2,
    0x00,                                                                                           //Attribute Value
    0x00                                                                                            //Attribute Value,
};


Uint8 att_HDL_CPS_CYCLING_POWER_MEASUREMENT_SERVER_CHARACTERISTIC_CONFIGURATION[] =
{
    //0x00, (_CPS_HDL_OFFSET_BASE_+4),                                                              //Handle
    //GATT_DESCRIPTORS, GATT_DESCRIPTORS_SERVER_CHARACTERISTIC_CONFIGURATION,                       //Attribute Type
    //SIZE_OF_ATTRIBUTE_VALUE_2,
    (
        //GATT_DESCRIPTORS_SERVER_CHARACTERISTIC_CONFIGURATION_BROADCAST |
        0x00
    ),                                                                                              //Attribute Value,
    0x00                                                                                            //Attribute Value,
};


const Uint8 ATT_HDL_CPS_CHARACTERISTIC_CYCLING_POWER_FEATURE[] =
{
    0x00, (_CPS_HDL_OFFSET_BASE_ + 5),                                                              //Handle
    GATT_DECLARATIONS, GATT_DECLARATIONS_CHARACTERISTIC,                                            //Attribute Type
    SIZE_OF_ATTRIBUTE_VALUE_5,
    //0x11,
    (
        //Characteristic Properties
        //GATT_DECLARATIONS_PROPERTIES_BROADCAST |
        GATT_DECLARATIONS_PROPERTIES_READ |
        //GATT_DECLARATIONS_PROPERTIES_WRITE_WITHOUT_RESPONSE |
        //GATT_DECLARATIONS_PROPERTIES_WRITE |
        //GATT_DECLARATIONS_PROPERTIES_NOTIFY |
        //GATT_DECLARATIONS_PROPERTIES_INDICATE |
        //GATT_DECLARATIONS_PROPERTIES_AUTHENTICATED_SIGNED_WRITES |
        //GATT_DECLARATIONS_PROPERTIES_EXTENDED_PROPERTIES |
        0x00
    ),
    (_CPS_HDL_OFFSET_BASE_ + 6), 0x00,
    GATT_SPEC_CHARACTERISTIC_CYCLING_POWER_FEATURE, GATT_SPEC_CHARACTERISTIC                        //Attribute Value
};


const Uint8 ATT_HDL_CPS_CYCLING_POWER_FEATURE_INIT[] =
{
    0x00, (_CPS_HDL_OFFSET_BASE_ + 6),                                                              //Handle
    GATT_SPEC_CHARACTERISTIC, GATT_SPEC_CHARACTERISTIC_CYCLING_POWER_FEATURE,                       //Attribute Type
    SIZE_OF_ATTRIBUTE_VALUE_4,
    (
        CPS_FEAT_PEDAL_POWER_BALANCE_SUPPORTED |
        CPS_FEAT_ACCUMULATED_TORQUE_SUPPORTED |
        CPS_FEAT_WHEEL_REVOLUTION_DATA_SUPPORTED |
        CPS_FEAT_CRANK_REVOLUTION_DATA_SUPPORTED |
        CPS_FEAT_EXTREME_MAGNITUDES_SUPPORTED |
        CPS_FEAT_EXTREME_ANGLES_SUPPORTED |
        CPS_FEAT_TOP_AND_BOTTOM_DEAD_SPOT_ANGLES_SUPPORTED |
        CPS_FEAT_ACCUMULATED_ENERGY_SUPPORTED |
        0x00,
    ),
    (
        CPS_FEAT_OFFSET_COMPENSATION_INDICATOR_SUPPORTED |
        CPS_FEAT_OFFSET_COMPENSATION_SUPPORTED |
        CPS_FEAT_CP_MEASUREMENT_CHARA_CONTENT_MASKING_SUPPORTED |
        CPS_FEAT_MULTIPLE_SENSOR_LOCATIONS_SUPPORTED |
        CPS_FEAT_CRANK_LENGTH_ADJUSTMENT_SUPPORTED |
        CPS_FEAT_CHAIN_LENGTH_ADJUSTMENT_SUPPORTED |
        CPS_FEAT_CHAIN_WEIGHT_ADJUSTMENT_SUPPORTED |
        CPS_FEAT_SPAN_LENGTH_ADJUSTMENT_SUPPORTED |
        0x00,
    ),
    (
        CPS_FEAT_SENSOR_MEASUREMENT_CONTEXT_TORQUE_BASED |
        CPS_FEAT_INSTANTANEOUS_MEASUREMENT_DIRECTION_SUPPORTED |
        CPS_FEAT_FACTORY_CALIBRATION_DATE_SUPPORTED |
        0x00,
    ),
    0x00,
};


const Uint8 ATT_HDL_CPS_CHARACTERISTIC_SENSOR_LOCATION[] =
{
    0x00, (_CPS_HDL_OFFSET_BASE_ + 7),                                                              //Handle
    GATT_DECLARATIONS, GATT_DECLARATIONS_CHARACTERISTIC,                                            //Attribute Type
    SIZE_OF_ATTRIBUTE_VALUE_5,
    //0x11,
    (
        //Characteristic Properties
        //GATT_DECLARATIONS_PROPERTIES_BROADCAST |
        GATT_DECLARATIONS_PROPERTIES_READ |
        //GATT_DECLARATIONS_PROPERTIES_WRITE_WITHOUT_RESPONSE |
        //GATT_DECLARATIONS_PROPERTIES_WRITE |
        //GATT_DECLARATIONS_PROPERTIES_NOTIFY |
        //GATT_DECLARATIONS_PROPERTIES_INDICATE |
        //GATT_DECLARATIONS_PROPERTIES_AUTHENTICATED_SIGNED_WRITES |
        //GATT_DECLARATIONS_PROPERTIES_EXTENDED_PROPERTIES |
        0x00
    ),
    (_CPS_HDL_OFFSET_BASE_ + 8), 0x00,
    GATT_SPEC_CHARACTERISTIC_SENSOR_LOCATION, GATT_SPEC_CHARACTERISTIC                              //Attribute Value
};


const Uint8 ATT_HDL_CPS_SENSOR_LOCATION_INIT[] =
{
    0x00, (_CPS_HDL_OFFSET_BASE_ + 8),                                                              //Handle
    GATT_SPEC_CHARACTERISTIC, GATT_SPEC_CHARACTERISTIC_SENSOR_LOCATION,                             //Attribute Type
    SIZE_OF_ATTRIBUTE_VALUE_1,
    0x00,                                                                                           //Sensor Location
};


Uint8 att_HDL_CPS_SENSOR_LOCATION[] =
{
    //0x00, (_CPS_HDL_OFFSET_BASE_+8),                                                              //Handle
    //GATT_SPEC_CHARACTERISTIC, GATT_SPEC_CHARACTERISTIC_SENSOR_LOCATION,                           //Attribute Type
    //SIZE_OF_ATTRIBUTE_VALUE_1,
    0x00,                                                                                           //Sensor Location
};


const Uint8 ATT_HDL_CPS_CHARACTERISTIC_CYCLING_POWER_VECTOR[] =
{
    0x00, (_CPS_HDL_OFFSET_BASE_ + 9),
    GATT_DECLARATIONS, GATT_DECLARATIONS_CHARACTERISTIC,
    SIZE_OF_ATTRIBUTE_VALUE_5,
    (
        //Characteristic Properties
        //GATT_DECLARATIONS_PROPERTIES_BROADCAST |
        //GATT_DECLARATIONS_PROPERTIES_READ |
        //GATT_DECLARATIONS_PROPERTIES_WRITE_WITHOUT_RESPONSE |
        //GATT_DECLARATIONS_PROPERTIES_WRITE |
        GATT_DECLARATIONS_PROPERTIES_NOTIFY |
        //GATT_DECLARATIONS_PROPERTIES_INDICATE |
        //GATT_DECLARATIONS_PROPERTIES_AUTHENTICATED_SIGNED_WRITES |
        //GATT_DECLARATIONS_PROPERTIES_EXTENDED_PROPERTIES |
        0x00
    ),
    (_CPS_HDL_OFFSET_BASE_ + 10), 0x00,
    GATT_SPEC_CHARACTERISTIC_CYCLING_POWER_VECTOR, GATT_SPEC_CHARACTERISTIC                         //Attribute Value
};


const Uint8 ATT_HDL_CPS_CYCLING_POWER_VECTOR_INIT[] =
{
    0x00, (_CPS_HDL_OFFSET_BASE_ + 10),                                                              //Handle
    GATT_SPEC_CHARACTERISTIC, GATT_SPEC_CHARACTERISTIC_CYCLING_POWER_VECTOR,                        //Attribute Type
    SIZE_OF_ATTRIBUTE_VALUE_11,
    0x00,                                                                                           //Flags
    0x00, 0x00,                                                                                     //Crank Revolution Data - Cumulative Crank Revolutions
    0x00, 0x00,                                                                                     //Crank Revolution Data - Last Crank Event Time
    0x00, 0x00,                                                                                     //First Crank Measurement Angle
    0x00, 0x00,                                                                                     //Instantaneous Force Magnitude Array
    0x00, 0x00,                                                                                     //Instantaneous Torque Magnitude Array
};


Uint8 att_HDL_CPS_CYCLING_POWER_VECTOR[] =
{
    //0x00, (_CPS_HDL_OFFSET_BASE_+10),                                                              //Handle
    //GATT_SPEC_CHARACTERISTIC, GATT_SPEC_CHARACTERISTIC_CYCLING_POWER_VECTOR,                      //Attribute Type
    //SIZE_OF_ATTRIBUTE_VALUE_11,
    0x00,                                                                                           //Flags
    0x00, 0x00,                                                                                     //Crank Revolution Data - Cumulative Crank Revolutions
    0x00, 0x00,                                                                                     //Crank Revolution Data - Last Crank Event Time
    0x00, 0x00,                                                                                     //First Crank Measurement Angle
    0x00, 0x00,                                                                                     //Instantaneous Force Magnitude Array
    0x00, 0x00,                                                                                     //Instantaneous Torque Magnitude Array
};


const Uint8 ATT_HDL_CPS_CYCLING_POWER_VECTOR_CLIENT_CHARACTERISTIC_CONFIGURATION_INIT[] =
{
    0x00, (_CPS_HDL_OFFSET_BASE_ + 11),                                                             //Handle
    GATT_DESCRIPTORS, GATT_DESCRIPTORS_CLIENT_CHARACTERISTIC_CONFIGURATION,                         //Attribute Type
    SIZE_OF_ATTRIBUTE_VALUE_2,
    0x00,                                                                                           //Attribute Value
    0x00                                                                                            //Attribute Value,
};


Uint8 att_HDL_CPS_CYCLING_POWER_VECTOR_CLIENT_CHARACTERISTIC_CONFIGURATION[] =
{
    //0x00, (_CPS_HDL_OFFSET_BASE_+11),                                                             //Handle
    //GATT_DESCRIPTORS, GATT_DESCRIPTORS_CLIENT_CHARACTERISTIC_CONFIGURATION,                       //Attribute Type
    //SIZE_OF_ATTRIBUTE_VALUE_2,
    (
        //GATT_DESCRIPTORS_CLIENT_CHARACTERISTIC_CONFIGURATION_NOTIFICATION |
        //GATT_DESCRIPTORS_CLIENT_CHARACTERISTIC_CONFIGURATION_INDICATION |
        0x00
    ),                                                                                              //Attribute Value,
    0x00                                                                                            //Attribute Value,
};


const Uint8 ATT_HDL_CPS_CHARACTERISTIC_CYCLING_POWER_CONTROL_POINT[] =
{
    0x00, (_CPS_HDL_OFFSET_BASE_ + 12),                                                             //Handle
    GATT_DECLARATIONS, GATT_DECLARATIONS_CHARACTERISTIC,                                            //Attribute Type
    SIZE_OF_ATTRIBUTE_VALUE_5,
    //0x11,
    (
        //Characteristic Properties
        //GATT_DECLARATIONS_PROPERTIES_BROADCAST |
        //GATT_DECLARATIONS_PROPERTIES_READ |
        //GATT_DECLARATIONS_PROPERTIES_WRITE_WITHOUT_RESPONSE |
        GATT_DECLARATIONS_PROPERTIES_WRITE |
        //GATT_DECLARATIONS_PROPERTIES_NOTIFY |
        GATT_DECLARATIONS_PROPERTIES_INDICATE |
        //GATT_DECLARATIONS_PROPERTIES_AUTHENTICATED_SIGNED_WRITES |
        //GATT_DECLARATIONS_PROPERTIES_EXTENDED_PROPERTIES |
        0x00
    ),
    (_CPS_HDL_OFFSET_BASE_ + 13), 0x00,
    GATT_SPEC_CHARACTERISTIC_CYCLING_POWER_CONTROL_POINT, GATT_SPEC_CHARACTERISTIC                  //Attribute Value
};


const Uint8 ATT_HDL_CPS_CYCLING_POWER_CONTROL_POINT_INIT[] =
{
    0x00, (_CPS_HDL_OFFSET_BASE_ + 13),                                                             //Handle
    GATT_SPEC_CHARACTERISTIC, GATT_SPEC_CHARACTERISTIC_CYCLING_POWER_CONTROL_POINT,                 //Attribute Type
    SIZE_OF_ATTRIBUTE_VALUE_10,
    0x00,                                                                                           //Op Code
    //0x00,                                                                                           //Parameter Value
    0x00,                                                                                           //Request Op Code
    0x00,                                                                                           //Response Value
    0x00, 0x00,                                                                                     //UTC Time: Year, Max. Response Parameter
    0x00,                                                                                           //UTC Time: Month
    0x00,                                                                                           //UTC Time: Day
    0x00,                                                                                           //UTC Time: Hours
    0x00,                                                                                           //UTC Time: Minutes
    0x00,                                                                                           //UTC Time: Seconds
};


Uint8 att_HDL_CPS_CYCLING_POWER_CONTROL_POINT[] =
{
    //0x00, (_CPS_HDL_OFFSET_BASE_+13),                                                             //Handle
    //GATT_SPEC_CHARACTERISTIC, GATT_SPEC_CHARACTERISTIC_CYCLING_POWER_CONTROL_POINT,               //Attribute Type
    //SIZE_OF_ATTRIBUTE_VALUE_10,
    0x00,                                                                                           //Op Code
    //0x00,                                                                                           //Parameter Value
    0x00,                                                                                           //Request Op Code
    0x00,                                                                                           //Response Value
    0x00, 0x00,                                                                                     //UTC Time: Year, Max. Response Parameter
    0x00,                                                                                           //UTC Time: Month
    0x00,                                                                                           //UTC Time: Day
    0x00,                                                                                           //UTC Time: Hours
    0x00,                                                                                           //UTC Time: Minutes
    0x00,                                                                                           //UTC Time: Seconds
};


const Uint8 ATT_HDL_CPS_CYCLING_POWER_CONTROL_POINT_CLIENT_CHARACTERISTIC_CONFIGURATION_INIT[] =
{
    0x00, (_CPS_HDL_OFFSET_BASE_ + 14),                                                             //Handle
    GATT_DESCRIPTORS, GATT_DESCRIPTORS_CLIENT_CHARACTERISTIC_CONFIGURATION,                         //Attribute Type
    SIZE_OF_ATTRIBUTE_VALUE_2,
    0x00,                                                                                           //Attribute Value
    0x00                                                                                            //Attribute Value,
};


Uint8 att_HDL_CPS_CYCLING_POWER_CONTROL_POINT_CLIENT_CHARACTERISTIC_CONFIGURATION[] =
{
    //0x00, (_CPS_HDL_OFFSET_BASE_+14),                                                             //Handle
    //GATT_DESCRIPTORS, GATT_DESCRIPTORS_CLIENT_CHARACTERISTIC_CONFIGURATION,                       //Attribute Type
    //SIZE_OF_ATTRIBUTE_VALUE_2,
    (
        //GATT_DESCRIPTORS_CLIENT_CHARACTERISTIC_CONFIGURATION_NOTIFICATION |
        //GATT_DESCRIPTORS_CLIENT_CHARACTERISTIC_CONFIGURATION_INDICATION |
        0x00
    ),                                                                                              //Attribute Value,
    0x00                                                                                            //Attribute Value,
};
#endif

//
#ifdef _PROFILE_CSTM_UDF01S_                                                 //
const Uint8 ATT_HDL_UDF01S_PRIMARY_SERVICE[] =                               //
{
    //
    0x00, _UDF01S_HDL_OFFSET_BASE_,                                          //0x00 0x27                   //Handle
    GATT_DECLARATIONS, GATT_DECLARATIONS_PRIMARY_SERVICE,                    //0x28 0x00                  //Attribute Type (0x28, 0x00)
    SIZE_OF_ATTRIBUTE_VALUE_16,                                              //0x10
    //GATT_SPEC_CSTM_SERVICES_UDF01S, GATT_SPEC_CSTM_SERVICES                //                   //Attribute Value (0x01, 0xF9)
    0xFF, 0xEE, 0xDD, 0xCC,                                                  //0xFF, 0xEE, 0xDD, 0xCC,
    0xBB, 0xAA, 0x99, 0x88,                                                  //0xBB, 0xAA, 0x99, 0x88,
    0x77, 0x66, 0x55, 0x44,                                                  //0x77, 0x66, 0x55, 0x44,
    0x33, 0x22, 0x11, 0x00                                                   //0x33, 0x22, 0x11, 0x00
};                                                                           //
//
//
const Uint8 ATT_HDL_UDF01S_CHARACTERISTIC_UDATR01[] =                        //
{
    //
    0x00, (_UDF01S_HDL_OFFSET_BASE_ + 0x01),                                 //0x00 0x28
    GATT_DECLARATIONS, GATT_DECLARATIONS_CHARACTERISTIC,                     //0x28 0x03                   //Attribute Type (0x28, 0x03)
    SIZE_OF_ATTRIBUTE_VALUE_19,                                              //0x13
    //0x11,                                                                  //
    (                                                                        //
        //Characteristic Properties                                          //
        //GATT_DECLARATIONS_PROPERTIES_BROADCAST |                           //                   //Read: Mandatory
        GATT_DECLARATIONS_PROPERTIES_READ |                                  //0x02                   //Notify: Optional
        //GATT_DECLARATIONS_PROPERTIES_WRITE_WITHOUT_RESPONSE |              //
        //GATT_DECLARATIONS_PROPERTIES_WRITE |                               //
        //GATT_DECLARATIONS_PROPERTIES_NOTIFY |                              //
        //GATT_DECLARATIONS_PROPERTIES_INDICATE |                            //
        //GATT_DECLARATIONS_PROPERTIES_AUTHENTICATED_SIGNED_WRITES |         //
        //GATT_DECLARATIONS_PROPERTIES_EXTENDED_PROPERTIES |                 //
        0x00                                                                 //
    ),                                                                       //
    (_UDF01S_HDL_OFFSET_BASE_ + 0x02), 0x00,                                 //0x29 0x00
    0x1F, 0x1E, 0x1D, 0x1C,                                                  //0x1F, 0x1E, 0x1D, 0x1C,                  //128 bits Attribute Type (UUID)
    0x1B, 0x1A, 0x19, 0x18,                                                  //0x1B, 0x1A, 0x19, 0x18,
    0x17, 0x16, 0x15, 0x14,                                                  //0x17, 0x16, 0x15, 0x14,
    0x13, 0x12, 0x11, 0x10                                                   //0x13, 0x12, 0x11, 0x10
};                                                                           //
//
//
const Uint8 ATT_HDL_UDF01S_UDATR01[] =                                       //
{
    //
    0x00, (_UDF01S_HDL_OFFSET_BASE_ + 0x02),                                 //0x00 0x29
    //GATT_SPEC_CSTM_CHARACTERISTIC, GATT_SPEC_CSTM_CHARACTERISTIC_UDATR01,  //                       //Attribute Type (128 bits)
    0x10, 0x11, 0x12, 0x13,                                                  //0x10, 0x11, 0x12, 0x13,
    0x14, 0x15, 0x16, 0x17,                                                  //0x14, 0x15, 0x16, 0x17,
    0x18, 0x19, 0x1A, 0x1B,                                                  //0x18, 0x19, 0x1A, 0x1B,
    0x1C, 0x1D, 0x1E, 0x1F,                                                  //0x1C, 0x1D, 0x1E, 0x1F,
    SIZE_OF_ATTRIBUTE_VALUE_20,                                              //0x14
    0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,                          //0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,       //Send "Read Data" to Central
    0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F,                          //0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F,
    0x10, 0x11, 0x12, 0x13                                                   //0x10, 0x11, 0x12, 0x13
};                                                                           //
//
//
const Uint8 ATT_HDL_UDF01S_CHARACTERISTIC_UDATN01[] =                        //
{
    //
    0x00, (_UDF01S_HDL_OFFSET_BASE_ + 0x03),                                 //0x00 0x2A                       //Handle
    GATT_DECLARATIONS, GATT_DECLARATIONS_CHARACTERISTIC,                     //0x28 0x03                       //Attribute Type (0x28, 0x03)
    SIZE_OF_ATTRIBUTE_VALUE_5,                                               //0x05
    //0x11,                                                                  //
    (                                                                        //
        //Characteristic Properties                                          //
        //GATT_DECLARATIONS_PROPERTIES_BROADCAST |                           //                       //Read: Mandatory
        //GATT_DECLARATIONS_PROPERTIES_READ |                                //                       //Notify: Optional
        //GATT_DECLARATIONS_PROPERTIES_WRITE_WITHOUT_RESPONSE |              //
        //GATT_DECLARATIONS_PROPERTIES_WRITE |                               //
        GATT_DECLARATIONS_PROPERTIES_NOTIFY |                                //0x10
        //GATT_DECLARATIONS_PROPERTIES_INDICATE |                            //
        //GATT_DECLARATIONS_PROPERTIES_AUTHENTICATED_SIGNED_WRITES |         //
        //GATT_DECLARATIONS_PROPERTIES_EXTENDED_PROPERTIES |                 //
        0x00                                                                 //0x00
    ),                                                                       //
    (_UDF01S_HDL_OFFSET_BASE_ + 0x04), 0x00,                                 //0x2b 0x00
    GATT_SPEC_CSTM_CHARACTERISTIC_UDATN01, GATT_SPEC_CSTM_CHARACTERISTIC     //0x02 0xFA                      //Attribute Value (0x02, 0xFA)
};


const Uint8 ATT_HDL_UDF01S_UDATN01_INIT[] =
{
    0x00, (_UDF01S_HDL_OFFSET_BASE_ + 0x04),                                                            //Handle
    GATT_SPEC_CSTM_CHARACTERISTIC, GATT_SPEC_CSTM_CHARACTERISTIC_UDATN01,                               //Attribute Type (0x02, 0xFA)
#if (BLE_DATA_LENGTH_EXTENSION_SUPPORT == ENABLE_)
    SIZE_OF_ATTRIBUTE_VALUE_244,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00,
#else
    SIZE_OF_ATTRIBUTE_VALUE_20,
    0x10, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
    0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F,
    0x10, 0x11, 0x12, 0x13
#endif
};


Uint8 att_HDL_UDF01S_UDATN01[] =
{
    //0x00, (_UDF01S_HDL_OFFSET_BASE_+0x04),                                                            //Handle
    //GATT_SPEC_CSTM_CHARACTERISTIC, GATT_SPEC_CSTM_CHARACTERISTIC_UDATN01,                             //Attribute Type
#if (BLE_DATA_LENGTH_EXTENSION_SUPPORT == ENABLE_)
    //SIZE_OF_ATTRIBUTE_VALUE_244,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00
#else
    //SIZE_OF_ATTRIBUTE_VALUE_20,
    0x10, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
    0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F,
    0x10, 0x11, 0x12, 0x13
#endif
};


const Uint8 ATT_HDL_UDF01S_UDATN01_CLIENT_CHARACTERISTIC_CONFIGURATION_INIT[] =                     //Mandatory if the Battery Level characteristic properties supports notification
{
    //
    0x00, (_UDF01S_HDL_OFFSET_BASE_ + 0x05),                                        //               //Handle
    GATT_DESCRIPTORS, GATT_DESCRIPTORS_CLIENT_CHARACTERISTIC_CONFIGURATION,         //               //Attribute Type (0x29, 0x02)
    SIZE_OF_ATTRIBUTE_VALUE_2,                                                      //
    0x00,                                                                           //               //Attribute Value
    0x00                                                                            //               //Attribute Value,
};


Uint8 att_HDL_UDF01S_UDATN01_MEASUREMENT_CLIENT_CHARACTERISTIC_CONFIGURATION[] =
{
    //0x00, (_UDF01S_HDL_OFFSET_BASE_+0x05),                                                        //Handle
    //GATT_DESCRIPTORS, GATT_DESCRIPTORS_CLIENT_CHARACTERISTIC_CONFIGURATION,         //              //Attribute Type
    //SIZE_OF_ATTRIBUTE_VALUE_2,                                                      //
    (                                                                                 //
        //GATT_DESCRIPTORS_CLIENT_CHARACTERISTIC_CONFIGURATION_NOTIFICATION |         //
        //GATT_DESCRIPTORS_CLIENT_CHARACTERISTIC_CONFIGURATION_INDICATION |           //
        0x00                                                                          //
    ),                                                                                //              //Attribute Value, R/W: Mandatory
    0x00                                                                              //              //Attribute Value,
};


const Uint8 ATT_HDL_UDF01S_CHARACTERISTIC_UDATRW01[] =
{
    0x00, (_UDF01S_HDL_OFFSET_BASE_ + 0x06),                                 //                      //Handle
    GATT_DECLARATIONS, GATT_DECLARATIONS_CHARACTERISTIC,                     //                      //Attribute Type
    SIZE_OF_ATTRIBUTE_VALUE_19,                                              //
    //0x11,                                                                  //
    (                                                                        //
        //Characteristic Properties                                          //
        //GATT_DECLARATIONS_PROPERTIES_BROADCAST |                           //                      //Read: Mandatory
        GATT_DECLARATIONS_PROPERTIES_READ |                                  //                      //Notify: Optional
        //GATT_DECLARATIONS_PROPERTIES_WRITE_WITHOUT_RESPONSE |              //
        GATT_DECLARATIONS_PROPERTIES_WRITE |                                 //
        //GATT_DECLARATIONS_PROPERTIES_NOTIFY |                              //
        //GATT_DECLARATIONS_PROPERTIES_INDICATE |                            //
        //GATT_DECLARATIONS_PROPERTIES_AUTHENTICATED_SIGNED_WRITES |         //
        //GATT_DECLARATIONS_PROPERTIES_EXTENDED_PROPERTIES |                 //
        0x00                                                                 //
    ),                                                                       //
    (_UDF01S_HDL_OFFSET_BASE_ + 0x07), 0x00,                                 //                     //128 bits Type (UUID)
    0x5F, 0x5E, 0x5D, 0x5C,                                                  //
    0x5B, 0x5A, 0x59, 0x58,                                                  //
    0x57, 0x56, 0x55, 0x54,                                                  //
    0x53, 0x52, 0x51, 0x50                                                   //
};


const Uint8 ATT_HDL_UDF01S_UDATRW01_INIT[] =
{
    0x00, (_UDF01S_HDL_OFFSET_BASE_ + 0x07),                                                        //Handle
    0x50, 0x51, 0x52, 0x53,
    0x54, 0x55, 0x56, 0x57,
    0x58, 0x59, 0x5A, 0x5B,
    0x5C, 0x5D, 0x5E, 0x5F,
#if (BLE_DATA_LENGTH_EXTENSION_SUPPORT == ENABLE_)
    SIZE_OF_ATTRIBUTE_VALUE_244,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00
#else
    SIZE_OF_ATTRIBUTE_VALUE_20,
    0x20, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
    0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F,
    0x10, 0x11, 0x12, 0x13
#endif
};


Uint8 att_HDL_UDF01S_UDATRW01[] =
{
    //0x00, (_UDF01S_HDL_OFFSET_BASE_+0x07),                                                        //Handle
    //GATT_SPEC_CSTM_CHARACTERISTIC, GATT_SPEC_CSTM_CHARACTERISTIC_UDATRW01,                        //Attribute Type
#if (BLE_DATA_LENGTH_EXTENSION_SUPPORT == ENABLE_)
    //SIZE_OF_ATTRIBUTE_VALUE_244,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00
#else
    //SIZE_OF_ATTRIBUTE_VALUE_45,
    0x20, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,                                                 //Central "Write Data" to this memory
    0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F,
    0x10, 0x11, 0x12, 0x13
#endif
};
#endif
//////////////////////////////////////////////////////////////////////////////

















const Uint8 ATT_HDL_UDF_PRIMARY_SERVICE[] =                               //
{
    //
    0x00, (_UDF01S_HDL_OFFSET_BASE_ + 0x08),                                        //0x00 0x27                   //Handle
    GATT_DECLARATIONS, GATT_DECLARATIONS_PRIMARY_SERVICE,                    //0x28 0x00                  //Attribute Type (0x28, 0x00)
    SIZE_OF_ATTRIBUTE_VALUE_16,                                              //0x10
    //GATT_SPEC_CSTM_SERVICES_UDF01S, GATT_SPEC_CSTM_SERVICES                //                   //Attribute Value (0x01, 0xF9)
    0x23, 0xd1, 0xbc, 0xea,
    0x5f, 0x78, 0x23, 0x15,
    0xde, 0xef, 0x12, 0x12,
    0x23, 0x15, 0x00, 0x00,
};                                                                           //
//
//
const Uint8 ATT_HDL_UDF_CHARACTERISTIC_UDATR01[] =                        //
{
    //
    0x00, (_UDF01S_HDL_OFFSET_BASE_ + 0x09),                                 //0x00 0x28
    GATT_DECLARATIONS, GATT_DECLARATIONS_CHARACTERISTIC,                     //0x28 0x03                   //Attribute Type (0x28, 0x03)
    SIZE_OF_ATTRIBUTE_VALUE_19,                                              //0x13
    //0x11,                                                                  //
    (                                                                        //
        //Characteristic Properties                                          //
        //GATT_DECLARATIONS_PROPERTIES_BROADCAST |                           //                   //Read: Mandatory
        GATT_DECLARATIONS_PROPERTIES_READ |                                  //0x02                   //Notify: Optional
        //GATT_DECLARATIONS_PROPERTIES_WRITE_WITHOUT_RESPONSE |              //
        //GATT_DECLARATIONS_PROPERTIES_WRITE |                               //
        GATT_DECLARATIONS_PROPERTIES_NOTIFY |                              //
        //GATT_DECLARATIONS_PROPERTIES_INDICATE |                            //
        //GATT_DECLARATIONS_PROPERTIES_AUTHENTICATED_SIGNED_WRITES |         //
        //GATT_DECLARATIONS_PROPERTIES_EXTENDED_PROPERTIES |                 //
        0x00                                                                 //
    ),                                                                       //
    (_UDF01S_HDL_OFFSET_BASE_ + 0x0A), 0x00,                                 //0x29 0x00
    0x23, 0xd1, 0xbc, 0xea,
    0x5f, 0x78, 0x23, 0x15,
    0xde, 0xef, 0x12, 0x12,
    0x24, 0x15, 0x00, 0x00,
};                                                                           //
//
//
const Uint8 ATT_HDL_UDF_UDATR01[] =                                       //
{
    //
    0x00, (_UDF01S_HDL_OFFSET_BASE_ + 0x0A),                                 //0x00 0x29
    //GATT_SPEC_CSTM_CHARACTERISTIC, GATT_SPEC_CSTM_CHARACTERISTIC_UDATR01,  //                       //Attribute Type (128 bits)
    0x23, 0xd1, 0xbc, 0xea,
    0x5f, 0x78, 0x23, 0x15,
    0xde, 0xef, 0x12, 0x12,
    0x24, 0x15, 0x00, 0x00,
    SIZE_OF_ATTRIBUTE_VALUE_1,                                              //0x14
    0xb4,
};                                                                           //
//
//
const Uint8 ATT_HDL_UDF_CHARACTERISTIC_UDATN01[] =                        //
{
    //
    0x00, (_UDF01S_HDL_OFFSET_BASE_ + 0x0B),                                 //0x00 0x2A                       //Handle
    GATT_DECLARATIONS, GATT_DECLARATIONS_CHARACTERISTIC,                     //0x28 0x03                       //Attribute Type (0x28, 0x03)
    SIZE_OF_ATTRIBUTE_VALUE_5,                                               //0x05
    //0x11,                                                                  //
    (                                                                        //
        //Characteristic Properties                                          //
        //GATT_DECLARATIONS_PROPERTIES_BROADCAST |                           //                       //Read: Mandatory
        GATT_DECLARATIONS_PROPERTIES_READ |                                //                       //Notify: Optional
        //GATT_DECLARATIONS_PROPERTIES_WRITE_WITHOUT_RESPONSE |              //
//        GATT_DECLARATIONS_PROPERTIES_WRITE |                               //
//        GATT_DECLARATIONS_PROPERTIES_NOTIFY |                                //0x10
        //GATT_DECLARATIONS_PROPERTIES_INDICATE |                            //
        //GATT_DECLARATIONS_PROPERTIES_AUTHENTICATED_SIGNED_WRITES |         //
        //GATT_DECLARATIONS_PROPERTIES_EXTENDED_PROPERTIES |                 //
        0x00                                                                 //0x00
    ),                                                                       //
    (_UDF01S_HDL_OFFSET_BASE_ + 0x0C), 0x00,                                 //0x2b 0x00
    GATT_SPEC_CSTM_CHARACTERISTIC_UDATN01, GATT_SPEC_CSTM_CHARACTERISTIC     //0x02 0xFA                      //Attribute Value (0x02, 0xFA)
};


const Uint8 ATT_HDL_UDF_UDATN01_INIT[] =
{
    0x00, (_UDF01S_HDL_OFFSET_BASE_ + 0x0C),                                                            //Handle
    GATT_SPEC_CSTM_CHARACTERISTIC, GATT_SPEC_CSTM_CHARACTERISTIC_UDATN01,                               //Attribute Type (0x02, 0xFA)
    SIZE_OF_ATTRIBUTE_VALUE_244,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00,
};


Uint8 att_HDL_UDF_UDATN01[] =
{
    //0x00, (_UDF01S_HDL_OFFSET_BASE_+0x04),                                                            //Handle
    //GATT_SPEC_CSTM_CHARACTERISTIC, GATT_SPEC_CSTM_CHARACTERISTIC_UDATN01,                             //Attribute Type
    //SIZE_OF_ATTRIBUTE_VALUE_244,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00

};


const Uint8 ATT_HDL_UDF_UDATN01_CLIENT_CHARACTERISTIC_CONFIGURATION_INIT[] =                     //Mandatory if the Battery Level characteristic properties supports notification
{
    //
    0x00, (_UDF01S_HDL_OFFSET_BASE_ + 0x0D),                                        //               //Handle
    GATT_DESCRIPTORS, GATT_DESCRIPTORS_CLIENT_CHARACTERISTIC_CONFIGURATION,         //               //Attribute Type (0x29, 0x02)
    SIZE_OF_ATTRIBUTE_VALUE_2,                                                      //
    0x00,                                                                           //               //Attribute Value
    0x00                                                                            //               //Attribute Value,
};


Uint8 att_HDL_UDF_UDATN01_MEASUREMENT_CLIENT_CHARACTERISTIC_CONFIGURATION[] =
{
    //0x00, (_UDF01S_HDL_OFFSET_BASE_+0x05),                                                        //Handle
    //GATT_DESCRIPTORS, GATT_DESCRIPTORS_CLIENT_CHARACTERISTIC_CONFIGURATION,         //              //Attribute Type
    //SIZE_OF_ATTRIBUTE_VALUE_2,                                                      //
    (                                                                                 //
        //GATT_DESCRIPTORS_CLIENT_CHARACTERISTIC_CONFIGURATION_NOTIFICATION |         //
        //GATT_DESCRIPTORS_CLIENT_CHARACTERISTIC_CONFIGURATION_INDICATION |           //
        0x00                                                                          //
    ),                                                                                //              //Attribute Value, R/W: Mandatory
    0x00                                                                              //              //Attribute Value,
};


const Uint8 ATT_HDL_UDF_CHARACTERISTIC_UDATRW01[] =
{
    0x00, (_UDF01S_HDL_OFFSET_BASE_ + 0x0E),                                 //                      //Handle
    GATT_DECLARATIONS, GATT_DECLARATIONS_CHARACTERISTIC,                     //                      //Attribute Type
    SIZE_OF_ATTRIBUTE_VALUE_19,                                              //
    //0x11,                                                                  //
    (                                                                        //
        //Characteristic Properties                                          //
        //GATT_DECLARATIONS_PROPERTIES_BROADCAST |                           //                      //Read: Mandatory
        GATT_DECLARATIONS_PROPERTIES_READ |                                  //                      //Notify: Optional
        //GATT_DECLARATIONS_PROPERTIES_WRITE_WITHOUT_RESPONSE |              //
        GATT_DECLARATIONS_PROPERTIES_WRITE |                                 //
        //GATT_DECLARATIONS_PROPERTIES_NOTIFY |                              //
        //GATT_DECLARATIONS_PROPERTIES_INDICATE |                            //
        //GATT_DECLARATIONS_PROPERTIES_AUTHENTICATED_SIGNED_WRITES |         //
        //GATT_DECLARATIONS_PROPERTIES_EXTENDED_PROPERTIES |                 //
        0x00                                                                 //
    ),                                                                       //
    (_UDF01S_HDL_OFFSET_BASE_ + 0x0F), 0x00,                                 //                     //128 bits Type (UUID)
    0x23, 0xd1, 0xbc, 0xea,
    0x5f, 0x78, 0x23, 0x15,
    0xde, 0xef, 0x12, 0x12,
    0x25, 0x15, 0x00, 0x00,
};


const Uint8 ATT_HDL_UDF_UDATRW01_INIT[] =
{
    0x00, (_UDF01S_HDL_OFFSET_BASE_ + 0x0F),                                                        //Handle
    0x23, 0xd1, 0xbc, 0xea,
    0x5f, 0x78, 0x23, 0x15,
    0xde, 0xef, 0x12, 0x12,
    0x25, 0x15, 0x00, 0x00,
    SIZE_OF_ATTRIBUTE_VALUE_1,
    0x00,
};


Uint8 att_HDL_UDF_UDATRW01[] =
{
    //0x00, (_UDF01S_HDL_OFFSET_BASE_+0x07),                                                        //Handle
    //GATT_SPEC_CSTM_CHARACTERISTIC, GATT_SPEC_CSTM_CHARACTERISTIC_UDATRW01,                        //Attribute Type
    //SIZE_OF_ATTRIBUTE_VALUE_244,
    0x00,
};


//////////////////////////////////////////////////////////////////////////////
#ifdef _PROFILE_CSTM_UDF02S_
const Uint8 ATT_HDL_UDF02S_PRIMARY_SERVICE[] =
{
    0x00, _UDF02S_HDL_OFFSET_BASE_,                                                                 //Handle
    GATT_DECLARATIONS, GATT_DECLARATIONS_PRIMARY_SERVICE,                                           //Attribute Type
    SIZE_OF_ATTRIBUTE_VALUE_2,
    GATT_SPEC_CSTM_SERVICES_UDF02S, GATT_SPEC_CSTM_SERVICES                                         //Attribute Value
};


const Uint8 ATT_HDL_UDF02S_CHARACTERISTIC_UDATR02[] =
{
    0x00, (_UDF02S_HDL_OFFSET_BASE_ + 0x01),
    GATT_DECLARATIONS, GATT_DECLARATIONS_CHARACTERISTIC,                                            //Attribute Type
    SIZE_OF_ATTRIBUTE_VALUE_5,
    //0x11,
    (
        //Characteristic Properties
        //GATT_DECLARATIONS_PROPERTIES_BROADCAST |                                                  //Read: Mandatory
        GATT_DECLARATIONS_PROPERTIES_READ |                                                         //Notify: Optional
        //GATT_DECLARATIONS_PROPERTIES_WRITE_WITHOUT_RESPONSE |
        //GATT_DECLARATIONS_PROPERTIES_WRITE |
        //GATT_DECLARATIONS_PROPERTIES_NOTIFY |
        //GATT_DECLARATIONS_PROPERTIES_INDICATE |
        //GATT_DECLARATIONS_PROPERTIES_AUTHENTICATED_SIGNED_WRITES |
        //GATT_DECLARATIONS_PROPERTIES_EXTENDED_PROPERTIES |
        0x00
    ),
    (_UDF02S_HDL_OFFSET_BASE_ + 0x02), 0x00,
    GATT_SPEC_CSTM_CHARACTERISTIC_UDATR02, GATT_SPEC_CSTM_CHARACTERISTIC                            //Attribute Value
};


const Uint8 ATT_HDL_UDF02S_UDATR02[] =
{
    0x00, (_UDF02S_HDL_OFFSET_BASE_ + 0x02),
    GATT_SPEC_CSTM_CHARACTERISTIC, GATT_SPEC_CSTM_CHARACTERISTIC_UDATR02,                           //Attribute Type
    SIZE_OF_ATTRIBUTE_VALUE_20,
    0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
    0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F,
    0x10, 0x11, 0x12, 0x13
};


const Uint8 ATT_HDL_UDF02S_CHARACTERISTIC_UDATN02[] =
{
    0x00, (_UDF02S_HDL_OFFSET_BASE_ + 0x03),                                                        //Handle
    GATT_DECLARATIONS, GATT_DECLARATIONS_CHARACTERISTIC,                                            //Attribute Type
    SIZE_OF_ATTRIBUTE_VALUE_5,
    //0x11,
    (
        //Characteristic Properties
        //GATT_DECLARATIONS_PROPERTIES_BROADCAST |                                                  //Read: Mandatory
        //GATT_DECLARATIONS_PROPERTIES_READ |                                                       //Notify: Optional
        //GATT_DECLARATIONS_PROPERTIES_WRITE_WITHOUT_RESPONSE |
        //GATT_DECLARATIONS_PROPERTIES_WRITE |
        GATT_DECLARATIONS_PROPERTIES_NOTIFY |
        //GATT_DECLARATIONS_PROPERTIES_INDICATE |
        //GATT_DECLARATIONS_PROPERTIES_AUTHENTICATED_SIGNED_WRITES |
        //GATT_DECLARATIONS_PROPERTIES_EXTENDED_PROPERTIES |
        0x00
    ),
    (_UDF02S_HDL_OFFSET_BASE_ + 0x04), 0x00,
    GATT_SPEC_CSTM_CHARACTERISTIC_UDATN02, GATT_SPEC_CSTM_CHARACTERISTIC                            //Attribute Value
};


const Uint8 ATT_HDL_UDF02S_UDATN02_INIT[] =
{
    0x00, (_UDF02S_HDL_OFFSET_BASE_ + 0x04),                                                        //Handle
    GATT_SPEC_CSTM_CHARACTERISTIC, GATT_SPEC_CSTM_CHARACTERISTIC_UDATN02,                           //Attribute Type
    SIZE_OF_ATTRIBUTE_VALUE_20,
    0x10, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
    0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F,
    0x10, 0x11, 0x12, 0x13
};


Uint8 att_HDL_UDF02S_UDATN02[] =
{
    //0x00, (_UDF02S_HDL_OFFSET_BASE_+0x04),                                                        //Handle
    //GATT_SPEC_CSTM_CHARACTERISTIC, GATT_SPEC_CSTM_CHARACTERISTIC_UDATN02,                         //Attribute Type
    //SIZE_OF_ATTRIBUTE_VALUE_20,
    0x10, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
    0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F,
    0x10, 0x11, 0x12, 0x13
};


const Uint8 ATT_HDL_UDF02S_UDATN02_CLIENT_CHARACTERISTIC_CONFIGURATION_INIT[] =                     //Mandatory if the Battery Level characteristic properties supports notification
{
    0x00, (_UDF02S_HDL_OFFSET_BASE_ + 0x05),                                                        //Handle
    GATT_DESCRIPTORS, GATT_DESCRIPTORS_CLIENT_CHARACTERISTIC_CONFIGURATION,                         //Attribute Type
    SIZE_OF_ATTRIBUTE_VALUE_2,
    0x00,                                                                                           //Attribute Value
    0x00                                                                                            //Attribute Value,
};


Uint8 att_HDL_UDF02S_UDATN02_MEASUREMENT_CLIENT_CHARACTERISTIC_CONFIGURATION[] =
{
    //0x00, (_UDF02S_HDL_OFFSET_BASE_+0x05),                                                        //Handle
    //GATT_DESCRIPTORS, GATT_DESCRIPTORS_CLIENT_CHARACTERISTIC_CONFIGURATION,                       //Attribute Type
    //SIZE_OF_ATTRIBUTE_VALUE_2,
    (
        //GATT_DESCRIPTORS_CLIENT_CHARACTERISTIC_CONFIGURATION_NOTIFICATION |
        //GATT_DESCRIPTORS_CLIENT_CHARACTERISTIC_CONFIGURATION_INDICATION |
        0x00
    ),                                                                                              //Attribute Value, R/W: Mandatory
    0x00                                                                                            //Attribute Value,
};


const Uint8 ATT_HDL_UDF02S_CHARACTERISTIC_UDATRW02[] =
{
    0x00, (_UDF02S_HDL_OFFSET_BASE_ + 0x06),                                                        //Handle
    GATT_DECLARATIONS, GATT_DECLARATIONS_CHARACTERISTIC,                                            //Attribute Type
    SIZE_OF_ATTRIBUTE_VALUE_5,
    //0x11,
    (
        //Characteristic Properties
        //GATT_DECLARATIONS_PROPERTIES_BROADCAST |                                                  //Read: Mandatory
        GATT_DECLARATIONS_PROPERTIES_READ |                                                         //Notify: Optional
        //GATT_DECLARATIONS_PROPERTIES_WRITE_WITHOUT_RESPONSE |
        GATT_DECLARATIONS_PROPERTIES_WRITE |
        //GATT_DECLARATIONS_PROPERTIES_NOTIFY |
        //GATT_DECLARATIONS_PROPERTIES_INDICATE |
        //GATT_DECLARATIONS_PROPERTIES_AUTHENTICATED_SIGNED_WRITES |
        //GATT_DECLARATIONS_PROPERTIES_EXTENDED_PROPERTIES |
        0x00
    ),
    (_UDF02S_HDL_OFFSET_BASE_ + 0x07), 0x00,
    GATT_SPEC_CSTM_CHARACTERISTIC_UDATRW02, GATT_SPEC_CSTM_CHARACTERISTIC                           //Attribute Value
};


const Uint8 ATT_HDL_UDF02S_UDATRW02_INIT[] =
{
    0x00, (_UDF02S_HDL_OFFSET_BASE_ + 0x07),                                                        //Handle
    GATT_SPEC_CSTM_CHARACTERISTIC, GATT_SPEC_CSTM_CHARACTERISTIC_UDATRW02,                          //Attribute Type
    SIZE_OF_ATTRIBUTE_VALUE_20,
    0x20, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
    0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F,
    0x10, 0x11, 0x12, 0x13
};


Uint8 att_HDL_UDF02S_UDATRW02[] =
{
    //0x00, (_UDF02S_HDL_OFFSET_BASE_+0x07),                                                        //Handle
    //GATT_SPEC_CSTM_CHARACTERISTIC, GATT_SPEC_CSTM_CHARACTERISTIC_UDATRW02,                        //Attribute Type
    //SIZE_OF_ATTRIBUTE_VALUE_20,
    0x20, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
    0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F,
    0x10, 0x11, 0x12, 0x13
};

#endif


#ifdef _PROFILE_CSTM_UDF03S_
const Uint8 ATT_HDL_UDF03S_PRIMARY_SERVICE[] =
{
    0x00, _UDF03S_HDL_OFFSET_BASE_,                                                                 //Handle
    GATT_DECLARATIONS, GATT_DECLARATIONS_PRIMARY_SERVICE,                                           //Attribute Type
    SIZE_OF_ATTRIBUTE_VALUE_2,
    GATT_SPEC_CSTM_SERVICES_UDF03S, GATT_SPEC_CSTM_SERVICES                                         //Attribute Value
};


const Uint8 ATT_HDL_UDF03S_CHARACTERISTIC_UDATR03[] =
{
    0x00, (_UDF03S_HDL_OFFSET_BASE_ + 0x01),
    GATT_DECLARATIONS, GATT_DECLARATIONS_CHARACTERISTIC,                                            //Attribute Type
    SIZE_OF_ATTRIBUTE_VALUE_5,
    //0x11,
    (
        //Characteristic Properties
        //GATT_DECLARATIONS_PROPERTIES_BROADCAST |                                                  //Read: Mandatory
        GATT_DECLARATIONS_PROPERTIES_READ |                                                         //Notify: Optional
        //GATT_DECLARATIONS_PROPERTIES_WRITE_WITHOUT_RESPONSE |
        //GATT_DECLARATIONS_PROPERTIES_WRITE |
        //GATT_DECLARATIONS_PROPERTIES_NOTIFY |
        //GATT_DECLARATIONS_PROPERTIES_INDICATE |
        //GATT_DECLARATIONS_PROPERTIES_AUTHENTICATED_SIGNED_WRITES |
        //GATT_DECLARATIONS_PROPERTIES_EXTENDED_PROPERTIES |
        0x00
    ),
    (_UDF03S_HDL_OFFSET_BASE_ + 0x02), 0x00,
    GATT_SPEC_CSTM_CHARACTERISTIC_UDATR03, GATT_SPEC_CSTM_CHARACTERISTIC                            //Attribute Value
};


const Uint8 ATT_HDL_UDF03S_UDATR03[] =
{
    0x00, (_UDF03S_HDL_OFFSET_BASE_ + 0x02),
    GATT_SPEC_CSTM_CHARACTERISTIC, GATT_SPEC_CSTM_CHARACTERISTIC_UDATR03,                           //Attribute Type
    SIZE_OF_ATTRIBUTE_VALUE_20,
    0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
    0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F,
    0x10, 0x11, 0x12, 0x13
};


const Uint8 ATT_HDL_UDF03S_CHARACTERISTIC_UDATN03[] =
{
    0x00, (_UDF03S_HDL_OFFSET_BASE_ + 0x03),                                                         //Handle
    GATT_DECLARATIONS, GATT_DECLARATIONS_CHARACTERISTIC,                                             //Attribute Type
    SIZE_OF_ATTRIBUTE_VALUE_5,
    //0x11,
    (
        //Characteristic Properties
        //GATT_DECLARATIONS_PROPERTIES_BROADCAST |                                                   //Read: Mandatory
        //GATT_DECLARATIONS_PROPERTIES_READ |                                                        //Notify: Optional
        //GATT_DECLARATIONS_PROPERTIES_WRITE_WITHOUT_RESPONSE |
        //GATT_DECLARATIONS_PROPERTIES_WRITE |
        GATT_DECLARATIONS_PROPERTIES_NOTIFY |
        //GATT_DECLARATIONS_PROPERTIES_INDICATE |
        //GATT_DECLARATIONS_PROPERTIES_AUTHENTICATED_SIGNED_WRITES |
        //GATT_DECLARATIONS_PROPERTIES_EXTENDED_PROPERTIES |
        0x00
    ),
    (_UDF03S_HDL_OFFSET_BASE_ + 0x04), 0x00,
    GATT_SPEC_CSTM_CHARACTERISTIC_UDATN03, GATT_SPEC_CSTM_CHARACTERISTIC                            //Attribute Value
};


const Uint8 ATT_HDL_UDF03S_UDATN03_INIT[] =
{
    0x00, (_UDF03S_HDL_OFFSET_BASE_ + 0x04),                                                        //Handle
    GATT_SPEC_CSTM_CHARACTERISTIC, GATT_SPEC_CSTM_CHARACTERISTIC_UDATN03,                           //Attribute Type
    SIZE_OF_ATTRIBUTE_VALUE_20,
    0x10, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
    0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F,
    0x10, 0x11, 0x12, 0x13
};


Uint8 att_HDL_UDF03S_UDATN03[] =
{
    //0x00, (_UDF03S_HDL_OFFSET_BASE_+0x04),                                                        //Handle
    //GATT_SPEC_CSTM_CHARACTERISTIC, GATT_SPEC_CSTM_CHARACTERISTIC_UDATN03,                         //Attribute Type
    //SIZE_OF_ATTRIBUTE_VALUE_20,
    0x10, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
    0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F,
    0x10, 0x11, 0x12, 0x13
};


const Uint8 ATT_HDL_UDF03S_UDATN03_CLIENT_CHARACTERISTIC_CONFIGURATION_INIT[] =                     //Mandatory if the Battery Level characteristic properties supports notification
{
    0x00, (_UDF03S_HDL_OFFSET_BASE_ + 0x05),                                                        //Handle
    GATT_DESCRIPTORS, GATT_DESCRIPTORS_CLIENT_CHARACTERISTIC_CONFIGURATION,                         //Attribute Type
    SIZE_OF_ATTRIBUTE_VALUE_2,
    0x00,                                                                                           //Attribute Value
    0x00                                                                                            //Attribute Value,
};


Uint8 att_HDL_UDF03S_UDATN03_MEASUREMENT_CLIENT_CHARACTERISTIC_CONFIGURATION[] =
{
    //0x00, (_UDF03S_HDL_OFFSET_BASE_+0x05),                                                        //Handle
    //GATT_DESCRIPTORS, GATT_DESCRIPTORS_CLIENT_CHARACTERISTIC_CONFIGURATION,                       //Attribute Type
    //SIZE_OF_ATTRIBUTE_VALUE_2,
    (
        //GATT_DESCRIPTORS_CLIENT_CHARACTERISTIC_CONFIGURATION_NOTIFICATION |
        //GATT_DESCRIPTORS_CLIENT_CHARACTERISTIC_CONFIGURATION_INDICATION |
        0x00
    ),                                                                                              //Attribute Value, R/W: Mandatory
    0x00                                                                                            //Attribute Value,
};


const Uint8 ATT_HDL_UDF03S_CHARACTERISTIC_UDATRW03[] =
{
    0x00, (_UDF03S_HDL_OFFSET_BASE_ + 0x06),                                                        //Handle
    GATT_DECLARATIONS, GATT_DECLARATIONS_CHARACTERISTIC,                                            //Attribute Type
    SIZE_OF_ATTRIBUTE_VALUE_5,
    //0x11,
    (
        //Characteristic Properties
        //GATT_DECLARATIONS_PROPERTIES_BROADCAST |                                                  //Read: Mandatory
        GATT_DECLARATIONS_PROPERTIES_READ |                                                         //Notify: Optional
        //GATT_DECLARATIONS_PROPERTIES_WRITE_WITHOUT_RESPONSE |
        GATT_DECLARATIONS_PROPERTIES_WRITE |
        //GATT_DECLARATIONS_PROPERTIES_NOTIFY |
        //GATT_DECLARATIONS_PROPERTIES_INDICATE |
        //GATT_DECLARATIONS_PROPERTIES_AUTHENTICATED_SIGNED_WRITES |
        //GATT_DECLARATIONS_PROPERTIES_EXTENDED_PROPERTIES |
        0x00
    ),
    (_UDF03S_HDL_OFFSET_BASE_ + 0x07), 0x00,
    GATT_SPEC_CSTM_CHARACTERISTIC_UDATRW03, GATT_SPEC_CSTM_CHARACTERISTIC                          //Attribute Value
};


const Uint8 ATT_HDL_UDF03S_UDATRW03_INIT[] =
{
    0x00, (_UDF03S_HDL_OFFSET_BASE_ + 0x07),                                                       //Handle
    GATT_SPEC_CSTM_CHARACTERISTIC, GATT_SPEC_CSTM_CHARACTERISTIC_UDATRW03,                         //Attribute Type
    SIZE_OF_ATTRIBUTE_VALUE_20,
    0x20, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
    0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F,
    0x10, 0x11, 0x12, 0x13
};


Uint8 att_HDL_UDF03S_UDATRW03[] =
{
    //0x00, (_UDF03S_HDL_OFFSET_BASE_+0x07),                                                       //Handle
    //GATT_SPEC_CSTM_CHARACTERISTIC, GATT_SPEC_CSTM_CHARACTERISTIC_UDATRW03,                       //Attribute Type
    //SIZE_OF_ATTRIBUTE_VALUE_20,
    0x20, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
    0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F,
    0x10, 0x11, 0x12, 0x13
};
#endif


#ifdef _PROFILE_OTA_
const Uint8 ATT_HDL_OTA_PRIMARY_SERVICE[] =
{
    0x00, _OTA_HDL_OFFSET_BASE_,
    GATT_DECLARATIONS, GATT_DECLARATIONS_PRIMARY_SERVICE,
    SIZE_OF_ATTRIBUTE_VALUE_2,
    GATT_SPEC_OTA_SERVICES_OTA, GATT_SPEC_OTA_SERVICES
};


const Uint8 ATT_HDL_OTA_CHARACTERISTIC_OTA_DATA[] =
{
    0x00, (_OTA_HDL_OFFSET_BASE_ + 1),
    GATT_DECLARATIONS, GATT_DECLARATIONS_CHARACTERISTIC,
    SIZE_OF_ATTRIBUTE_VALUE_5,
    (
        //GATT_DECLARATIONS_PROPERTIES_BROADCAST |
        //GATT_DECLARATIONS_PROPERTIES_READ |
        GATT_DECLARATIONS_PROPERTIES_WRITE_WITHOUT_RESPONSE |
        //GATT_DECLARATIONS_PROPERTIES_WRITE |
        GATT_DECLARATIONS_PROPERTIES_NOTIFY |
        //GATT_DECLARATIONS_PROPERTIES_INDICATE |
        //GATT_DECLARATIONS_PROPERTIES_AUTHENTICATED_SIGNED_WRITES |
        //GATT_DECLARATIONS_PROPERTIES_EXTENDED_PROPERTIES |
        0x00
    ),
    (_OTA_HDL_OFFSET_BASE_ + 2), 0x00,
    GATT_SPEC_OTA_CHARACTERISTIC_OTA_DATA, GATT_SPEC_OTA_CHARACTERISTIC
};


const Uint8 ATT_HDL_OTA_OTA_DATA_INIT[] =
{
    0x00, (_OTA_HDL_OFFSET_BASE_ + 2),                                                           //Handle
    GATT_SPEC_OTA_CHARACTERISTIC, GATT_SPEC_OTA_CHARACTERISTIC_OTA_DATA,                         //Attribute Type
    SIZE_OF_ATTRIBUTE_VALUE_244,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00,
};


Uint8 att_HDL_OTA_OTA_DATA[] =
{
    //0x01, (_OTA_HDL_OFFSET_BASE_+2),                                                             //Handle
    //GATT_SPEC_OTA_CHARACTERISTIC, GATT_SPEC_OTA_CHARACTERISTIC_OTA_DATA,                         //Attribute Type
    //SIZE_OF_ATTRIBUTE_VALUE_244,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00,
};


const Uint8 ATT_HDL_OTA_OTA_DATA_CLIENT_CHARACTERISTIC_CONFIGURATION_INIT[] =
{
    0x00, (_OTA_HDL_OFFSET_BASE_ + 3),                                                              //Handle
    GATT_DESCRIPTORS, GATT_DESCRIPTORS_CLIENT_CHARACTERISTIC_CONFIGURATION,                         //Attribute Type
    SIZE_OF_ATTRIBUTE_VALUE_2,
    0x00,                                                                                           //Attribute Value
    0x00                                                                                            //Attribute Value,
};


Uint8 att_HDL_OTA_OTA_DATA_CLIENT_CHARACTERISTIC_CONFIGURATION[] =
{
    //0x01, (_OTA_HDL_OFFSET_BASE_+3),                                                              //Handle
    //GATT_DESCRIPTORS, GATT_DESCRIPTORS_CLIENT_CHARACTERISTIC_CONFIGURATION,                       //Attribute Type
    //SIZE_OF_ATTRIBUTE_VALUE_2,
    (
        //GATT_DESCRIPTORS_CLIENT_CHARACTERISTIC_CONFIGURATION_NOTIFICATION |
        //GATT_DESCRIPTORS_CLIENT_CHARACTERISTIC_CONFIGURATION_INDICATION |
        0x00
    ),                                                                                              //Attribute Value
    0x00                                                                                            //Attribute Value,
};


const Uint8 ATT_HDL_OTA_OTA_DATA_DESCRIPTION[] =
{
    0x00, (_OTA_HDL_OFFSET_BASE_ + 4),
    GATT_DESCRIPTORS, GATT_DESCRIPTORS_CHARACTERISTIC_USER_DESCRIPTION,                             //Attribute Type
    (SIZE_OF_ATTRIBUTE_VALUE_0 + 8),
    0x4F, 0x54, 0x41, 0x5F, 0x44, 0x41, 0x54, 0x41
};


const Uint8 ATT_HDL_OTA_CHARACTERISTIC_OTA_CMD_CONTROL_POINT[] =
{
    0x00, (_OTA_HDL_OFFSET_BASE_ + 5),                                                              //Handle
    GATT_DECLARATIONS, GATT_DECLARATIONS_CHARACTERISTIC,                                            //Attribute Type
    SIZE_OF_ATTRIBUTE_VALUE_5,
    //0x11,
    (
        //Characteristic Properties
        //GATT_DECLARATIONS_PROPERTIES_BROADCAST |
        //GATT_DECLARATIONS_PROPERTIES_READ |
        //GATT_DECLARATIONS_PROPERTIES_WRITE_WITHOUT_RESPONSE |
        GATT_DECLARATIONS_PROPERTIES_WRITE |
        //GATT_DECLARATIONS_PROPERTIES_NOTIFY |
        GATT_DECLARATIONS_PROPERTIES_INDICATE |
        //GATT_DECLARATIONS_PROPERTIES_AUTHENTICATED_SIGNED_WRITES |
        //GATT_DECLARATIONS_PROPERTIES_EXTENDED_PROPERTIES |
        0x00
    ),
    (_OTA_HDL_OFFSET_BASE_ + 6), 0x00,
    GATT_SPEC_OTA_CHARACTERISTIC_OTA_CMD, GATT_SPEC_OTA_CHARACTERISTIC
};


const Uint8 ATT_HDL_OTA_OTA_CMD_CONTROL_POINT_INIT[] =
{
    0x00, (_OTA_HDL_OFFSET_BASE_ + 6),
    GATT_SPEC_OTA_CHARACTERISTIC, GATT_SPEC_OTA_CHARACTERISTIC_OTA_CMD,
    SIZE_OF_ATTRIBUTE_VALUE_17,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
};


Uint8 att_HDL_OTA_OTA_CMD_CONTROL_POINT[] =
{
    //0x01, (_OTA_HDL_OFFSET_BASE_+6),
    //GATT_SPEC_OTA_CHARACTERISTIC, GATT_SPEC_OTA_CHARACTERISTIC_OTA_CMD,
    //SIZE_OF_ATTRIBUTE_VALUE_17,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
};


const Uint8 ATT_HDL_OTA_OTA_CMD_CONTROL_POINT_CLIENT_CHARACTERISTIC_CONFIGURATION_INIT[] =
{
    0x00, (_OTA_HDL_OFFSET_BASE_ + 7),
    GATT_DESCRIPTORS, GATT_DESCRIPTORS_CLIENT_CHARACTERISTIC_CONFIGURATION,
    SIZE_OF_ATTRIBUTE_VALUE_2,
    0x00,
    0x00
};


Uint8 att_HDL_OTA_OTA_CMD_CONTROL_POINT_CLIENT_CHARACTERISTIC_CONFIGURATION[] =
{
    //0x01, (_OTA_HDL_OFFSET_BASE_+7),
    //GATT_DESCRIPTORS, GATT_DESCRIPTORS_CLIENT_CHARACTERISTIC_CONFIGURATION,
    //SIZE_OF_ATTRIBUTE_VALUE_2,
    (
        //GATT_DESCRIPTORS_CLIENT_CHARACTERISTIC_CONFIGURATION_NOTIFICATION |
        //GATT_DESCRIPTORS_CLIENT_CHARACTERISTIC_CONFIGURATION_INDICATION |
        0x00
    ),
    0x00
};


const Uint8 ATT_HDL_OTA_OTA_CMD_USER_DESCRIPTION[] =
{
    0x00, (_OTA_HDL_OFFSET_BASE_ + 8),
    GATT_DESCRIPTORS, GATT_DESCRIPTORS_CHARACTERISTIC_USER_DESCRIPTION,                             //Attribute Type
    SIZE_OF_ATTRIBUTE_VALUE_7,
    0x4F, 0x54, 0x41, 0x5F, 0x43, 0x4D, 0x44
};
#endif




#ifdef _PROFILE_CSTM_DATAEXCHANGE_S_

const Uint8 ATT_HDL_DATAEXCHANGES_PRIMARY_SERVICE[] =
{
    0x00, _DATAEXCHANGE_HDL_OFFSET_BASE_,                                                                 //Handle
    GATT_DECLARATIONS, GATT_DECLARATIONS_PRIMARY_SERVICE,                                           //Attribute Type (0x28, 0x00)
    SIZE_OF_ATTRIBUTE_VALUE_16,
    0x00, 0x00, 0x45, 0x53,                                                                         //128 bits Attribute Type (UUID)
    0x49, 0x57, 0x50, 0x4C,
    0x41, 0x00, 0x00, 0x00,
    0x01, 0x53, 0x00, 0x00
};


const Uint8 ATT_HDL_DATAEXCHANGES_CHARACTERISTIC_WRITE[] =
{
    0x00, (_DATAEXCHANGE_HDL_OFFSET_BASE_ + 0x01),
    GATT_DECLARATIONS, GATT_DECLARATIONS_CHARACTERISTIC,                                            //Attribute Type (0x28, 0x03)
    SIZE_OF_ATTRIBUTE_VALUE_19,
    //0x11,
    (
        //Characteristic Properties
        //GATT_DECLARATIONS_PROPERTIES_BROADCAST |                                                  //Read: Mandatory
        //GATT_DECLARATIONS_PROPERTIES_READ |                                                         //Notify: Optional
        GATT_DECLARATIONS_PROPERTIES_WRITE_WITHOUT_RESPONSE |
        GATT_DECLARATIONS_PROPERTIES_WRITE |
        //GATT_DECLARATIONS_PROPERTIES_NOTIFY |
        //GATT_DECLARATIONS_PROPERTIES_INDICATE |
        //GATT_DECLARATIONS_PROPERTIES_AUTHENTICATED_SIGNED_WRITES |
        //GATT_DECLARATIONS_PROPERTIES_EXTENDED_PROPERTIES |
        0x00
    ),
    (_DATAEXCHANGE_HDL_OFFSET_BASE_ + 0x02), 0x00,
    0x00, 0x00, 0x45, 0x53,                                                                         //128 bits Attribute Type (UUID)
    0x49, 0x57, 0x50, 0x4C,
    0x41, 0x00, 0x00, 0x00,
    0x02, 0x53, 0x00, 0x00
};

const Uint8 ATT_HDL_DATAEXCHANGES_WRITE[] =
{
    0x00, (_DATAEXCHANGE_HDL_OFFSET_BASE_ + 0x02),
    //GATT_SPEC_CSTM_CHARACTERISTIC, GATT_SPEC_CSTM_CHARACTERISTIC_UDATR01,                             //Attribute Type (128 bits)
    0x00, 0x00, 0x53, 0x02,
    0x00, 0x00, 0x00, 0x41,
    0x4C, 0x50, 0x57, 0x49,
    0x53, 0x45, 0x00, 0x00,
    SIZE_OF_ATTRIBUTE_VALUE_20,
    0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,                                                     //Send "Read Data" to Central
    0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F,
    0x10, 0x11, 0x12, 0x13
};

Uint8 att_HDL_DATAEXCHANGES_WRITE[] =
{
    //0x00, (_UDF01S_HDL_OFFSET_BASE_+0x07),                                                        //Handle
    //GATT_SPEC_CSTM_CHARACTERISTIC, GATT_SPEC_CSTM_CHARACTERISTIC_UDATRW01,                        //Attribute Type
    //SIZE_OF_ATTRIBUTE_VALUE_46,
    0x20, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,                                                 //Central "Write Data" to this memory
    0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F,
    0x10, 0x11, 0x12, 0x13, //0xFF
//        0x20, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
//        0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F,
//        0x10, 0x11, 0x12, 0x13,
//        0x20, 0x01, 0x02, 0x03, 0x04, 0x05, 0xFF
};

const Uint8 ATT_HDL_DATAEXCHANGES_CHARACTERISTICR_READ[] =
{
    0x00, (_DATAEXCHANGE_HDL_OFFSET_BASE_ + 0x03),                                                            //Handle
    GATT_DECLARATIONS, GATT_DECLARATIONS_CHARACTERISTIC,                                                //Attribute Type (0x28, 0x03)
    SIZE_OF_ATTRIBUTE_VALUE_19,
    //0x11,
    (
        //Characteristic Properties
        //GATT_DECLARATIONS_PROPERTIES_BROADCAST |                                                      //Read: Mandatory
        //GATT_DECLARATIONS_PROPERTIES_READ |                                                           //Notify: Optional
        //GATT_DECLARATIONS_PROPERTIES_WRITE_WITHOUT_RESPONSE |
        //GATT_DECLARATIONS_PROPERTIES_WRITE |
        GATT_DECLARATIONS_PROPERTIES_NOTIFY |
        GATT_DECLARATIONS_PROPERTIES_INDICATE |
        //GATT_DECLARATIONS_PROPERTIES_AUTHENTICATED_SIGNED_WRITES |
        //GATT_DECLARATIONS_PROPERTIES_EXTENDED_PROPERTIES |
        0x00
    ),
    (_DATAEXCHANGE_HDL_OFFSET_BASE_ + 0x04), 0x00,
    0x00, 0x00, 0x45, 0x53,                                                                         //128 bits Attribute Type (UUID)
    0x49, 0x57, 0x50, 0x4C,
    0x41, 0x00, 0x00, 0x00,
    0x03, 0x53, 0x00, 0x00
};


const Uint8 ATT_HDL_DATAEXCHANGES_READINIT[] =
{
//    0x00, (_DATAEXCHANGE_HDL_OFFSET_BASE_+0x04),                                                              //Handle
//    GATT_SPEC_CSTM_CHARACTERISTIC, GATT_SPEC_CSTM_CHARACTERISTIC_DATAEXCHANGE,                               //Attribute Type (0x02, 0xFA)
    0x00, (_DATAEXCHANGE_HDL_OFFSET_BASE_ + 0x04),
    //GATT_SPEC_CSTM_CHARACTERISTIC, GATT_SPEC_CSTM_CHARACTERISTIC_UDATR01,                             //Attribute Type (128 bits)
    0x00, 0x00, 0x53, 0x03,
    0x00, 0x00, 0x00, 0x41,
    0x4C, 0x50, 0x57, 0x49,
    0x53, 0x45, 0x00, 0x00,

    SIZE_OF_ATTRIBUTE_VALUE_20,
    0x10, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
    0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F,
    0x10, 0x11, 0x12, 0x13
};


Uint8 att_HDL_DATAEXCHANGES_READINIT[] =
{
    //0x00, (_UDF01S_HDL_OFFSET_BASE_+0x04),                                                            //Handle
    //GATT_SPEC_CSTM_CHARACTERISTIC, GATT_SPEC_CSTM_CHARACTERISTIC_UDATN01,                             //Attribute Type
    //SIZE_OF_ATTRIBUTE_VALUE_20,
    0x10, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
    0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F,
    0x10, 0x11, 0x12, 0x13
};


const Uint8 ATT_HDL_DATAEXCHANGES_READINIT_CLIENT_CHARACTERISTIC_CONFIGURATION_INIT[] =                     //Mandatory if the Battery Level characteristic properties supports notification
{
    0x00, (_DATAEXCHANGE_HDL_OFFSET_BASE_ + 0x05),                                                        //Handle
    GATT_DESCRIPTORS, GATT_DESCRIPTORS_CLIENT_CHARACTERISTIC_CONFIGURATION,                         //Attribute Type (0x29, 0x02)
    SIZE_OF_ATTRIBUTE_VALUE_2,
    0x00,                                                                                           //Attribute Value
    0x00                                                                                            //Attribute Value,
};


Uint8 att_HDL_DATAEXCHANGES_READINIT_MEASUREMENT_CLIENT_CHARACTERISTIC_CONFIGURATION[] =
{
    //0x00, (_UDF01S_HDL_OFFSET_BASE_+0x05),                                                        //Handle
    //GATT_DESCRIPTORS, GATT_DESCRIPTORS_CLIENT_CHARACTERISTIC_CONFIGURATION,                       //Attribute Type
    //SIZE_OF_ATTRIBUTE_VALUE_2,
    (
        //GATT_DESCRIPTORS_CLIENT_CHARACTERISTIC_CONFIGURATION_NOTIFICATION |
        //GATT_DESCRIPTORS_CLIENT_CHARACTERISTIC_CONFIGURATION_INDICATION |
        0x00
    ),                                                                                              //Attribute Value, R/W: Mandatory
    0x00                                                                                            //Attribute Value,
};



#endif



const Uint8 ATT_PERMISSION_NULL[] =                     //Bluetooth Spec, Ver4.0 [Vol 3] page 535 of 656
{
    (
        //ATT_PERMISSION_READ |
        //ATT_PERMISSION_WRITE |
        //ATT_PERMISSION_ENC_READ |
        //ATT_PERMISSION_ENC_WRITE |
        //ATT_PERMISSION_AUTHE_READ |
        //ATT_PERMISSION_AUTHE_WRITE |
        //ATT_PERMISSION_AUTHO_READ |
        //ATT_PERMISSION_AUTHO_WRITE |
        0x00
    ),
    (
        ATT_TYPE_FORMAT_16UUID |
        //ATT_TYPE_FORMAT_128UUID |
        0x00
    ),
};


const Uint8 ATT_PERMISSION_PRIMARY_SERVICE[] =                     //Bluetooth Spec, Ver4.0 [Vol 3] page 535 of 656
{
    (
        ATT_PERMISSION_READ |
        //ATT_PERMISSION_WRITE |
        //ATT_PERMISSION_ENC_READ |
        //ATT_PERMISSION_ENC_WRITE |
        //ATT_PERMISSION_AUTHE_READ |
        //ATT_PERMISSION_AUTHE_WRITE |
        //ATT_PERMISSION_AUTHO_READ |
        //ATT_PERMISSION_AUTHO_WRITE |
        0x00
    ),
    (
        ATT_TYPE_FORMAT_16UUID |
        //ATT_TYPE_FORMAT_128UUID |
        0x00
    ),
};


const Uint8 ATT_PERMISSION_SECONDARY_SERVICE[] =                   //Bluetooth Spec, Ver4.0 [Vol 3] page 535 of 656
{
    (
        ATT_PERMISSION_READ |
        //ATT_PERMISSION_WRITE |
        //ATT_PERMISSION_ENC_READ |
        //ATT_PERMISSION_ENC_WRITE |
        //ATT_PERMISSION_AUTHE_READ |
        //ATT_PERMISSION_AUTHE_WRITE |
        //ATT_PERMISSION_AUTHO_READ |
        //ATT_PERMISSION_AUTHO_WRITE |
        0x00
    ),
    (
        ATT_TYPE_FORMAT_16UUID |
        //ATT_TYPE_FORMAT_128UUID |
        0x00
    ),
};


const Uint8 ATT_PERMISSION_INCLUDE[] =                             //Bluetooth Spec, Ver4.0 [Vol 3] page 536 of 656
{
    (
        ATT_PERMISSION_READ |
        //ATT_PERMISSION_WRITE |
        //ATT_PERMISSION_ENC_READ |
        //ATT_PERMISSION_ENC_WRITE |
        //ATT_PERMISSION_AUTHE_READ |
        //ATT_PERMISSION_AUTHE_WRITE |
        //ATT_PERMISSION_AUTHO_READ |
        //ATT_PERMISSION_AUTHO_WRITE |
        0x00
    ),
    (
        ATT_TYPE_FORMAT_16UUID |
        //ATT_TYPE_FORMAT_128UUID |
        0x00
    ),
};


const Uint8 ATT_PERMISSION_CHARACTERISTIC[] =                      //Bluetooth Spec, Ver4.0 [Vol 3] page 537 of 656
{
    (
        ATT_PERMISSION_READ |
        //ATT_PERMISSION_WRITE |
        //ATT_PERMISSION_ENC_READ |
        //ATT_PERMISSION_ENC_WRITE |
        //ATT_PERMISSION_AUTHE_READ |
        //ATT_PERMISSION_AUTHE_WRITE |
        //ATT_PERMISSION_AUTHO_READ |
        //ATT_PERMISSION_AUTHO_WRITE |
        0x00
    ),
    (
        ATT_TYPE_FORMAT_16UUID |
        //ATT_TYPE_FORMAT_128UUID |
        0x00
    ),
};


const Uint8 ATT_PERMISSION_CHARACTERISTIC_AGGREGATE_FORMAT[] =     //Bluetooth Spec, Ver4.0 [Vol 3] page 547 of 656
{
    (
        ATT_PERMISSION_READ |
        //ATT_PERMISSION_WRITE |
        //ATT_PERMISSION_ENC_READ |
        //ATT_PERMISSION_ENC_WRITE |
        //ATT_PERMISSION_AUTHE_READ |
        //ATT_PERMISSION_AUTHE_WRITE |
        //ATT_PERMISSION_AUTHO_READ |
        //ATT_PERMISSION_AUTHO_WRITE |
        0x00
    ),
    (
        ATT_TYPE_FORMAT_16UUID |
        //ATT_TYPE_FORMAT_128UUID |
        0x00
    ),
};


const Uint8 ATT_PERMISSION_CHARACTERISTIC_EXTENDED_PROPERTIES[] =  //Bluetooth Spec, Ver4.0 [Vol 3] page 540 of 656
{
    (
        ATT_PERMISSION_READ |
        //ATT_PERMISSION_WRITE |
        //ATT_PERMISSION_ENC_READ |
        //ATT_PERMISSION_ENC_WRITE |
        //ATT_PERMISSION_AUTHE_READ |
        //ATT_PERMISSION_AUTHE_WRITE |
        //ATT_PERMISSION_AUTHO_READ |
        //ATT_PERMISSION_AUTHO_WRITE |
        0x00
    ),
    (
        ATT_TYPE_FORMAT_16UUID |
        //ATT_TYPE_FORMAT_128UUID |
        0x00
    ),
};


const Uint8 ATT_PERMISSION_CHARACTERISTIC_PRESENTATION_FORMAT[] =  //Bluetooth Spec, Ver4.0 [Vol 3] page 543 of 656
{
    (
        ATT_PERMISSION_READ |
        //ATT_PERMISSION_WRITE |
        //ATT_PERMISSION_ENC_READ |
        //ATT_PERMISSION_ENC_WRITE |
        //ATT_PERMISSION_AUTHE_READ |
        //ATT_PERMISSION_AUTHE_WRITE |
        //ATT_PERMISSION_AUTHO_READ |
        //ATT_PERMISSION_AUTHO_WRITE |
        0x00
    ),
    (
        ATT_TYPE_FORMAT_16UUID |
        //ATT_TYPE_FORMAT_128UUID |
        0x00
    ),
};


const Uint8 ATT_PERMISSION_HDL_GAP_DEVICE_NAME_INIT[] =
{
    (
        ATT_PERMISSION_READ |
        ATT_PERMISSION_WRITE |
        //ATT_PERMISSION_ENC_READ |
        //ATT_PERMISSION_ENC_WRITE |
        //ATT_PERMISSION_AUTHE_READ |
        //ATT_PERMISSION_AUTHE_WRITE |
        //ATT_PERMISSION_AUTHO_READ |
        //ATT_PERMISSION_AUTHO_WRITE |
        0x00
    ),
    (
        ATT_TYPE_FORMAT_16UUID |
        //ATT_TYPE_FORMAT_128UUID |
        0x00
    ),
};


const Uint8 ATT_PERMISSION_HDL_GAP_APPEARANCE[] =
{
    (
        ATT_PERMISSION_READ |
        //ATT_PERMISSION_WRITE |
        //ATT_PERMISSION_ENC_READ |
        //ATT_PERMISSION_ENC_WRITE |
        //ATT_PERMISSION_AUTHE_READ |
        //ATT_PERMISSION_AUTHE_WRITE |
        //ATT_PERMISSION_AUTHO_READ |
        //ATT_PERMISSION_AUTHO_WRITE |
        0x00
    ),
    (
        ATT_TYPE_FORMAT_16UUID |
        //ATT_TYPE_FORMAT_128UUID |
        0x00
    ),
};


const Uint8 ATT_PERMISSION_HDL_GAP_RECONNECTION_ADDRESS_INIT[] =
{
    (
        //ATT_PERMISSION_READ |
        ATT_PERMISSION_WRITE |
        //ATT_PERMISSION_ENC_READ |
        //ATT_PERMISSION_ENC_WRITE |
        //ATT_PERMISSION_AUTHE_READ |
        //ATT_PERMISSION_AUTHE_WRITE |
        //ATT_PERMISSION_AUTHO_READ |
        //ATT_PERMISSION_AUTHO_WRITE |
        0x00
    ),
    (
        ATT_TYPE_FORMAT_16UUID |
        //ATT_TYPE_FORMAT_128UUID |
        0x00
    ),
};


const Uint8 ATT_PERMISSION_HDL_GAP_PERIPHERAL_PRIVACY_FLAG[] =
{
    (
        ATT_PERMISSION_READ |
        //ATT_PERMISSION_WRITE |
        //ATT_PERMISSION_ENC_READ |
        //ATT_PERMISSION_ENC_WRITE |
        //ATT_PERMISSION_AUTHE_READ |
        //ATT_PERMISSION_AUTHE_WRITE |
        //ATT_PERMISSION_AUTHO_READ |
        //ATT_PERMISSION_AUTHO_WRITE |
        0x00
    ),
    (
        ATT_TYPE_FORMAT_16UUID |
        //ATT_TYPE_FORMAT_128UUID |
        0x00
    ),
};


const Uint8 ATT_PERMISSION_HDL_GAP_PERIPHERAL_PREFERRED_CONNECTION_PARAMETERS[] =
{
    (
        ATT_PERMISSION_READ |
        //ATT_PERMISSION_WRITE |
        //ATT_PERMISSION_ENC_READ |
        //ATT_PERMISSION_ENC_WRITE |
        //ATT_PERMISSION_AUTHE_READ |
        //ATT_PERMISSION_AUTHE_WRITE |
        //ATT_PERMISSION_AUTHO_READ |
        //ATT_PERMISSION_AUTHO_WRITE |
        0x00
    ),
    (
        ATT_TYPE_FORMAT_16UUID |
        //ATT_TYPE_FORMAT_128UUID |
        0x00
    ),
};


const Uint8 ATT_PERMISSION_HDL_GATT_SERVICE_CHANGED_INIT[] =
{
    (
        ATT_PERMISSION_READ |
        //ATT_PERMISSION_WRITE |
        //ATT_PERMISSION_ENC_READ |
        //ATT_PERMISSION_ENC_WRITE |
        //ATT_PERMISSION_AUTHE_READ |
        //ATT_PERMISSION_AUTHE_WRITE |
        //ATT_PERMISSION_AUTHO_READ |
        //ATT_PERMISSION_AUTHO_WRITE |
        0x00
    ),
    (
        ATT_TYPE_FORMAT_16UUID |
        //ATT_TYPE_FORMAT_128UUID |
        0x00
    ),
};


const Uint8 ATT_PERMISSION_HDL_GATT_CLIENT_CHARACTERISTIC_CONFIGURATION_INIT[] =
{
    (
        ATT_PERMISSION_READ |
        ATT_PERMISSION_WRITE |
        //ATT_PERMISSION_ENC_READ |
        //ATT_PERMISSION_ENC_WRITE |
        //ATT_PERMISSION_AUTHE_READ |
        //ATT_PERMISSION_AUTHE_WRITE |
        //ATT_PERMISSION_AUTHO_READ |
        //ATT_PERMISSION_AUTHO_WRITE |
        0x00
    ),
    (
        ATT_TYPE_FORMAT_16UUID |
        //ATT_TYPE_FORMAT_128UUID |
        0x00
    ),
};


const Uint8 ATT_PERMISSION_HDL_DIS_SERIAL_NUMBER_STRING[] =
{
    (
        ATT_PERMISSION_READ |
        //ATT_PERMISSION_WRITE |
        //ATT_PERMISSION_ENC_READ |
        //ATT_PERMISSION_ENC_WRITE |
        //ATT_PERMISSION_AUTHE_READ |
        //ATT_PERMISSION_AUTHE_WRITE |
        //ATT_PERMISSION_AUTHO_READ |
        //ATT_PERMISSION_AUTHO_WRITE |
        0x00
    ),
    (
        ATT_TYPE_FORMAT_16UUID |
        //ATT_TYPE_FORMAT_128UUID |
        //ATT_TYPE_FORMAT_ATT_CHR_ACSS_USER |
        0x00
    ),
};


const Uint8 ATT_PERMISSION_HDL_DIS_MANUFACTURER_NAME_STRING[] =
{
    (
        ATT_PERMISSION_READ |
        //ATT_PERMISSION_WRITE |
        //ATT_PERMISSION_ENC_READ |
        //ATT_PERMISSION_ENC_WRITE |
        //ATT_PERMISSION_AUTHE_READ |
        //ATT_PERMISSION_AUTHE_WRITE |
        //ATT_PERMISSION_AUTHO_READ |
        //ATT_PERMISSION_AUTHO_WRITE |
        0x00
    ),
    (
        ATT_TYPE_FORMAT_16UUID |
        //ATT_TYPE_FORMAT_128UUID |
        0x00
    ),
};


const Uint8 ATT_PERMISSION_HDL_DIS_SYSTEM_ID[] =
{
    (
        ATT_PERMISSION_READ |
        //ATT_PERMISSION_WRITE |
        //ATT_PERMISSION_ENC_READ |
        //ATT_PERMISSION_ENC_WRITE |
        //ATT_PERMISSION_AUTHE_READ |
        //ATT_PERMISSION_AUTHE_WRITE |
        //ATT_PERMISSION_AUTHO_READ |
        //ATT_PERMISSION_AUTHO_WRITE |
        0x00
    ),
    (
        ATT_TYPE_FORMAT_16UUID |
        //ATT_TYPE_FORMAT_128UUID |
        0x00
    ),
};


const Uint8 ATT_PERMISSION_HDL_DIS_FIRMWARE_REVISION_STRING[] =
{
    (
        ATT_PERMISSION_READ |
        //ATT_PERMISSION_WRITE |
        //ATT_PERMISSION_ENC_READ |
        //ATT_PERMISSION_ENC_WRITE |
        //ATT_PERMISSION_AUTHE_READ |
        //ATT_PERMISSION_AUTHE_WRITE |
        //ATT_PERMISSION_AUTHO_READ |
        //ATT_PERMISSION_AUTHO_WRITE |
        0x00
    ),
    (
        ATT_TYPE_FORMAT_16UUID |
        //ATT_TYPE_FORMAT_128UUID |
        0x00
    ),
};


const Uint8 ATT_PERMISSION_HDL_DIS_MODEL_NUMBER_STRING[] =
{
    (
        ATT_PERMISSION_READ |
        //ATT_PERMISSION_WRITE |
        //ATT_PERMISSION_ENC_READ |
        //ATT_PERMISSION_ENC_WRITE |
        //ATT_PERMISSION_AUTHE_READ |
        //ATT_PERMISSION_AUTHE_WRITE |
        //ATT_PERMISSION_AUTHO_READ |
        //ATT_PERMISSION_AUTHO_WRITE |
        0x00
    ),
    (
        ATT_TYPE_FORMAT_16UUID |
        //ATT_TYPE_FORMAT_128UUID |
        0x00
    ),
};


const Uint8 ATT_PERMISSION_HDL_DIS_HARDWARE_REVISION_STRING[] =
{
    (
        ATT_PERMISSION_READ |
        //ATT_PERMISSION_WRITE |
        //ATT_PERMISSION_ENC_READ |
        //ATT_PERMISSION_ENC_WRITE |
        //ATT_PERMISSION_AUTHE_READ |
        //ATT_PERMISSION_AUTHE_WRITE |
        //ATT_PERMISSION_AUTHO_READ |
        //ATT_PERMISSION_AUTHO_WRITE |
        0x00
    ),
    (
        ATT_TYPE_FORMAT_16UUID |
        //ATT_TYPE_FORMAT_128UUID |
        0x00
    ),
};


const Uint8 ATT_PERMISSION_HDL_DIS_SOFTWARE_REVISION_STRING[] =
{
    (
        ATT_PERMISSION_READ |
        //ATT_PERMISSION_WRITE |
        //ATT_PERMISSION_ENC_READ |
        //ATT_PERMISSION_ENC_WRITE |
        //ATT_PERMISSION_AUTHE_READ |
        //ATT_PERMISSION_AUTHE_WRITE |
        //ATT_PERMISSION_AUTHO_READ |
        //ATT_PERMISSION_AUTHO_WRITE |
        0x00
    ),
    (
        ATT_TYPE_FORMAT_16UUID |
        //ATT_TYPE_FORMAT_128UUID |
        0x00
    ),
};


const Uint8 ATT_PERMISSION_HDL_DIS_PNP_ID[] =
{
    (
        ATT_PERMISSION_READ |
        //ATT_PERMISSION_WRITE |
        //ATT_PERMISSION_ENC_READ |
        //ATT_PERMISSION_ENC_WRITE |
        //ATT_PERMISSION_AUTHE_READ |
        //ATT_PERMISSION_AUTHE_WRITE |
        //ATT_PERMISSION_AUTHO_READ |
        //ATT_PERMISSION_AUTHO_WRITE |
        0x00
    ),
    (
        ATT_TYPE_FORMAT_16UUID |
        //ATT_TYPE_FORMAT_128UUID |
        0x00
    ),
};


#ifdef _PROFILE_HRP_
const Uint8 ATT_PERMISSION_HDL_HRS_HEART_RATE_MEASUREMENT_INIT[] =
{
    (
        //ATT_PERMISSION_READ |
        //ATT_PERMISSION_WRITE |
        //ATT_PERMISSION_ENC_READ |
        //ATT_PERMISSION_ENC_WRITE |
        //ATT_PERMISSION_AUTHE_READ |
        //ATT_PERMISSION_AUTHE_WRITE |
        //ATT_PERMISSION_AUTHO_READ |
        //ATT_PERMISSION_AUTHO_WRITE |
        0x00
    ),
    (
        ATT_TYPE_FORMAT_16UUID |
        //ATT_TYPE_FORMAT_128UUID |
        0x00
    ),
};


const Uint8 ATT_PERMISSION_HDL_HRS_CLIENT_CHARACTERISTIC_CONFIGURATION_INIT[] =
{
    (
        ATT_PERMISSION_READ |
        ATT_PERMISSION_WRITE |
        //ATT_PERMISSION_ENC_READ |
        //ATT_PERMISSION_ENC_WRITE |
        //ATT_PERMISSION_AUTHE_READ |
        //ATT_PERMISSION_AUTHE_WRITE |
        //ATT_PERMISSION_AUTHO_READ |
        //ATT_PERMISSION_AUTHO_WRITE |
        0x00
    ),
    (
        ATT_TYPE_FORMAT_16UUID |
        //ATT_TYPE_FORMAT_128UUID |
        0x00
    ),
};


const Uint8 ATT_PERMISSION_HDL_HRS_BODY_SENSOR_LOCATION_INIT[] =
{
    (
        ATT_PERMISSION_READ |
        //ATT_PERMISSION_WRITE |
        //ATT_PERMISSION_ENC_READ |
        //ATT_PERMISSION_ENC_WRITE |
        //ATT_PERMISSION_AUTHE_READ |
        //ATT_PERMISSION_AUTHE_WRITE |
        //ATT_PERMISSION_AUTHO_READ |
        //ATT_PERMISSION_AUTHO_WRITE |
        0x00
    ),
    (
        ATT_TYPE_FORMAT_16UUID |
        //ATT_TYPE_FORMAT_128UUID |
        0x00
    ),
};
#endif


#ifdef _PROFILE_BAS_
const Uint8 ATT_PERMISSION_HDL_BAS_BATTERY_LEVEL_INIT[] =
{
    (
        ATT_PERMISSION_READ |
        //ATT_PERMISSION_WRITE |
        //ATT_PERMISSION_ENC_READ |
        //ATT_PERMISSION_ENC_WRITE |
        //ATT_PERMISSION_AUTHE_READ |
        //ATT_PERMISSION_AUTHE_WRITE |
        //ATT_PERMISSION_AUTHO_READ |
        //ATT_PERMISSION_AUTHO_WRITE |
        0x00
    ),
    (
        ATT_TYPE_FORMAT_16UUID |
        //ATT_TYPE_FORMAT_128UUID |
        0x00
    ),
};


const Uint8 ATT_PERMISSION_HDL_BAS_CLIENT_CHARACTERISTIC_CONFIGURATION_INIT[] =
{
    (
        ATT_PERMISSION_READ |
        ATT_PERMISSION_WRITE |
        //ATT_PERMISSION_ENC_READ |
        //ATT_PERMISSION_ENC_WRITE |
        //ATT_PERMISSION_AUTHE_READ |
        //ATT_PERMISSION_AUTHE_WRITE |
        //ATT_PERMISSION_AUTHO_READ |
        //ATT_PERMISSION_AUTHO_WRITE |
        0x00
    ),
    (
        ATT_TYPE_FORMAT_16UUID |
        //ATT_TYPE_FORMAT_128UUID |
        0x00
    ),
};


const Uint8 ATT_PERMISSION_HDL_BAS_BATTERY_POWER_STATE[] =
{
    (
        ATT_PERMISSION_READ |
        //ATT_PERMISSION_WRITE |
        //ATT_PERMISSION_ENC_READ |
        //ATT_PERMISSION_ENC_WRITE |
        //ATT_PERMISSION_AUTHE_READ |
        //ATT_PERMISSION_AUTHE_WRITE |
        //ATT_PERMISSION_AUTHO_READ |
        //ATT_PERMISSION_AUTHO_WRITE |
        0x00
    ),
    (
        ATT_TYPE_FORMAT_16UUID |
        //ATT_TYPE_FORMAT_128UUID |
        0x00
    ),
};
#endif


#ifdef _PROFILE_HOGP_
const Uint8 ATT_PERMISSION_HDL_HIDS_PROTOCOL_MODE_INIT[] =
{
    (
        //ATT_PERMISSION_READ |
        //ATT_PERMISSION_WRITE |
        //ATT_PERMISSION_ENC_READ |
        //ATT_PERMISSION_ENC_WRITE |
        ATT_PERMISSION_AUTHE_READ |
        ATT_PERMISSION_AUTHE_WRITE |
        //ATT_PERMISSION_AUTHO_READ |
        //ATT_PERMISSION_AUTHO_WRITE |
        0x00
    ),
    (
        ATT_TYPE_FORMAT_16UUID |
        //ATT_TYPE_FORMAT_128UUID |
        0x00
    ),
};


const Uint8 ATT_PERMISSION_HDL_HIDS_REPORT_INIT[] =
{
    (
        //ATT_PERMISSION_READ |
        //ATT_PERMISSION_WRITE |
        //ATT_PERMISSION_ENC_READ |
        //ATT_PERMISSION_ENC_WRITE |
        ATT_PERMISSION_AUTHE_READ |
        ATT_PERMISSION_AUTHE_WRITE |
        //ATT_PERMISSION_AUTHO_READ |
        //ATT_PERMISSION_AUTHO_WRITE |
        0x00
    ),
    (
        ATT_TYPE_FORMAT_16UUID |
        //ATT_TYPE_FORMAT_128UUID |
        0x00
    ),
};


const Uint8 ATT_PERMISSION_HDL_HIDS_REPORT_CLIENT_CHARACTERISTIC_CONFIGURATION_INIT[] =
{
    (
        ATT_PERMISSION_READ |
        //ATT_PERMISSION_WRITE |
        //ATT_PERMISSION_ENC_READ |
        //ATT_PERMISSION_ENC_WRITE |
        //ATT_PERMISSION_AUTHE_READ |
        ATT_PERMISSION_AUTHE_WRITE |
        //ATT_PERMISSION_AUTHO_READ |
        //ATT_PERMISSION_AUTHO_WRITE |
        0x00
    ),
    (
        ATT_TYPE_FORMAT_16UUID |
        //ATT_TYPE_FORMAT_128UUID |
        0x00
    ),
};


const Uint8 ATT_PERMISSION_HDL_HIDS_REPORT_REPORT_REFERENCE[] =
{
    (
        //ATT_PERMISSION_READ |
        //ATT_PERMISSION_WRITE |
        //ATT_PERMISSION_ENC_READ |
        //ATT_PERMISSION_ENC_WRITE |
        ATT_PERMISSION_AUTHE_READ |
        //ATT_PERMISSION_AUTHE_WRITE |
        //ATT_PERMISSION_AUTHO_READ |
        //ATT_PERMISSION_AUTHO_WRITE |
        0x00
    ),
    (
        ATT_TYPE_FORMAT_16UUID |
        //ATT_TYPE_FORMAT_128UUID |
        0x00
    ),
};


const Uint8 ATT_PERMISSION_HDL_HIDS_REPORT_MAP[] =
{
    (
        //ATT_PERMISSION_READ |
        //ATT_PERMISSION_WRITE |
        //ATT_PERMISSION_ENC_READ |
        //ATT_PERMISSION_ENC_WRITE |
        ATT_PERMISSION_AUTHE_READ |
        //ATT_PERMISSION_AUTHE_WRITE |
        //ATT_PERMISSION_AUTHO_READ |
        //ATT_PERMISSION_AUTHO_WRITE |
        0x00
    ),
    (
        ATT_TYPE_FORMAT_16UUID |
        //ATT_TYPE_FORMAT_128UUID |
        0x00
    ),
};


const Uint8 ATT_PERMISSION_HDL_HIDS_BOOT_MOUSE_INPUT_REPORT_INIT[] =
{
    (
        //ATT_PERMISSION_READ |
        //ATT_PERMISSION_WRITE |
        //ATT_PERMISSION_ENC_READ |
        //ATT_PERMISSION_ENC_WRITE |
        ATT_PERMISSION_AUTHE_READ |
        ATT_PERMISSION_AUTHE_WRITE |
        //ATT_PERMISSION_AUTHO_READ |
        //ATT_PERMISSION_AUTHO_WRITE |
        0x00
    ),
    (
        ATT_TYPE_FORMAT_16UUID |
        //ATT_TYPE_FORMAT_128UUID |
        0x00
    ),
};


const Uint8 ATT_PERMISSION_HDL_HIDS_BOOT_MOUSE_INPUT_REPORT_CLIENT_CHRCT_CONFIG_INIT[] =
{
    (
        ATT_PERMISSION_READ |
        //ATT_PERMISSION_WRITE |
        //ATT_PERMISSION_ENC_READ |
        //ATT_PERMISSION_ENC_WRITE |
        //ATT_PERMISSION_AUTHE_READ |
        ATT_PERMISSION_AUTHE_WRITE |
        //ATT_PERMISSION_AUTHO_READ |
        //ATT_PERMISSION_AUTHO_WRITE |
        0x00
    ),
    (
        ATT_TYPE_FORMAT_16UUID |
        //ATT_TYPE_FORMAT_128UUID |
        0x00
    ),
};


const Uint8 ATT_PERMISSION_HDL_HIDS_BOOT_KEYBOARD_INPUT_REPORT_INIT[] =
{
    (
        //ATT_PERMISSION_READ |
        //ATT_PERMISSION_WRITE |
        //ATT_PERMISSION_ENC_READ |
        //ATT_PERMISSION_ENC_WRITE |
        ATT_PERMISSION_AUTHE_READ |
        ATT_PERMISSION_AUTHE_WRITE |
        //ATT_PERMISSION_AUTHO_READ |
        //ATT_PERMISSION_AUTHO_WRITE |
        0x00
    ),
    (
        ATT_TYPE_FORMAT_16UUID |
        //ATT_TYPE_FORMAT_128UUID |
        0x00
    ),
};


const Uint8 ATT_PERMISSION_HDL_HIDS_BOOT_KEYBOARD_INPUT_REPORT_CLIENT_CHRCT_CONFIG_INIT[] =
{
    (
        ATT_PERMISSION_READ |
        //ATT_PERMISSION_WRITE |
        //ATT_PERMISSION_ENC_READ |
        //ATT_PERMISSION_ENC_WRITE |
        //ATT_PERMISSION_AUTHE_READ |
        ATT_PERMISSION_AUTHE_WRITE |
        //ATT_PERMISSION_AUTHO_READ |
        //ATT_PERMISSION_AUTHO_WRITE |
        0x00
    ),
    (
        ATT_TYPE_FORMAT_16UUID |
        //ATT_TYPE_FORMAT_128UUID |
        0x00
    ),
};


const Uint8 ATT_PERMISSION_HDL_HIDS_BOOT_KEYBOARD_OUTPUT_REPORT_INIT[] =
{
    (
        //ATT_PERMISSION_READ |
        //ATT_PERMISSION_WRITE |
        //ATT_PERMISSION_ENC_READ |
        //ATT_PERMISSION_ENC_WRITE |
        ATT_PERMISSION_AUTHE_READ |
        ATT_PERMISSION_AUTHE_WRITE |
        //ATT_PERMISSION_AUTHO_READ |
        //ATT_PERMISSION_AUTHO_WRITE |
        0x00
    ),
    (
        ATT_TYPE_FORMAT_16UUID |
        //ATT_TYPE_FORMAT_128UUID |
        0x00
    ),
};


const Uint8 ATT_PERMISSION_HDL_HIDS_HID_INFORMATION[] =
{
    (
        //ATT_PERMISSION_READ |
        //ATT_PERMISSION_WRITE |
        //ATT_PERMISSION_ENC_READ |
        //ATT_PERMISSION_ENC_WRITE |
        ATT_PERMISSION_AUTHE_READ |
        //ATT_PERMISSION_AUTHE_WRITE |
        //ATT_PERMISSION_AUTHO_READ |
        //ATT_PERMISSION_AUTHO_WRITE |
        0x00
    ),
    (
        ATT_TYPE_FORMAT_16UUID |
        //ATT_TYPE_FORMAT_128UUID |
        0x00
    ),
};


const Uint8 ATT_PERMISSION_HDL_HIDS_HID_CONTROL_POINT_INIT[] =
{
    (
        //ATT_PERMISSION_READ |
        //ATT_PERMISSION_WRITE |
        //ATT_PERMISSION_ENC_READ |
        //ATT_PERMISSION_ENC_WRITE |
        //ATT_PERMISSION_AUTHE_READ |
        ATT_PERMISSION_AUTHE_WRITE |
        //ATT_PERMISSION_AUTHO_READ |
        //ATT_PERMISSION_AUTHO_WRITE |
        0x00
    ),
    (
        ATT_TYPE_FORMAT_16UUID |
        //ATT_TYPE_FORMAT_128UUID |
        0x00
    ),
};


const Uint8 ATT_PERMISSION_HDL_HIDS_REPORT1_INIT[] =
{
    (
        //ATT_PERMISSION_READ |
        //ATT_PERMISSION_WRITE |
        //ATT_PERMISSION_ENC_READ |
        //ATT_PERMISSION_ENC_WRITE |
        ATT_PERMISSION_AUTHE_READ |
        ATT_PERMISSION_AUTHE_WRITE |
        //ATT_PERMISSION_AUTHO_READ |
        //ATT_PERMISSION_AUTHO_WRITE |
        0x00
    ),
    (
        ATT_TYPE_FORMAT_16UUID |
        //ATT_TYPE_FORMAT_128UUID |
        0x00
    ),
};


const Uint8 ATT_PERMISSION_HDL_HIDS_REPORT1_REPORT_REFERENCE[] =
{
    (
        //ATT_PERMISSION_READ |
        //ATT_PERMISSION_WRITE |
        //ATT_PERMISSION_ENC_READ |
        //ATT_PERMISSION_ENC_WRITE |
        ATT_PERMISSION_AUTHE_READ |
        //ATT_PERMISSION_AUTHE_WRITE |
        //ATT_PERMISSION_AUTHO_READ |
        //ATT_PERMISSION_AUTHO_WRITE |
        0x00
    ),
    (
        ATT_TYPE_FORMAT_16UUID |
        //ATT_TYPE_FORMAT_128UUID |
        0x00
    ),
};


const Uint8 ATT_PERMISSION_HDL_HIDS_REPORT1_USER_DESCRIPTION[] =
{
    (
        //ATT_PERMISSION_READ |
        //ATT_PERMISSION_WRITE |
        //ATT_PERMISSION_ENC_READ |
        //ATT_PERMISSION_ENC_WRITE |
        ATT_PERMISSION_AUTHE_READ |
        //ATT_PERMISSION_AUTHE_WRITE |
        //ATT_PERMISSION_AUTHO_READ |
        //ATT_PERMISSION_AUTHO_WRITE |
        0x00
    ),
    (
        ATT_TYPE_FORMAT_16UUID |
        //ATT_TYPE_FORMAT_128UUID |
        0x00
    ),
};
#endif


#ifdef _PROFILE_SCPP_
const Uint8 ATT_PERMISSION_HDL_SCPS_SCAN_INTERVAL_WINDOW_INIT[] =
{
    (
        //ATT_PERMISSION_READ |
        //ATT_PERMISSION_WRITE |
        //ATT_PERMISSION_ENC_READ |
        //ATT_PERMISSION_ENC_WRITE |
        //ATT_PERMISSION_AUTHE_READ |
        ATT_PERMISSION_AUTHE_WRITE |
        //ATT_PERMISSION_AUTHO_READ |
        //ATT_PERMISSION_AUTHO_WRITE |
        0x00
    ),
    (
        ATT_TYPE_FORMAT_16UUID |
        //ATT_TYPE_FORMAT_128UUID |
        0x00
    ),
};


const Uint8 ATT_PERMISSION_HDL_SCPS_SCAN_REFRESH_INIT[] =
{
    (
        //ATT_PERMISSION_READ |
        //ATT_PERMISSION_WRITE |
        //ATT_PERMISSION_ENC_READ |
        //ATT_PERMISSION_ENC_WRITE |
        //ATT_PERMISSION_AUTHE_READ |
        //ATT_PERMISSION_AUTHE_WRITE |
        //ATT_PERMISSION_AUTHO_READ |
        //ATT_PERMISSION_AUTHO_WRITE |
        0x00
    ),
    (
        ATT_TYPE_FORMAT_16UUID |
        //ATT_TYPE_FORMAT_128UUID |
        0x00
    ),
};


const Uint8 ATT_PERMISSION_HDL_SCPS_SCAN_REFRESH_CLIENT_CHARACTERISTIC_CONFIGURATION_INIT[] =
{
    (
        ATT_PERMISSION_READ |
        //ATT_PERMISSION_WRITE |
        //ATT_PERMISSION_ENC_READ |
        //ATT_PERMISSION_ENC_WRITE |
        //ATT_PERMISSION_AUTHE_READ |
        ATT_PERMISSION_AUTHE_WRITE |
        //ATT_PERMISSION_AUTHO_READ |
        //ATT_PERMISSION_AUTHO_WRITE |
        0x00
    ),
    (
        ATT_TYPE_FORMAT_16UUID |
        //ATT_TYPE_FORMAT_128UUID |
        0x00
    ),
};
#endif


#ifdef _PROFILE_IAS_
const Uint8 ATT_PERMISSION_HDL_IAS_ALERT_LEVEL_INIT[] =
{
    (
        //ATT_PERMISSION_READ |
        ATT_PERMISSION_WRITE |
        //ATT_PERMISSION_ENC_READ |
        //ATT_PERMISSION_ENC_WRITE |
        //ATT_PERMISSION_AUTHE_READ |
        //ATT_PERMISSION_AUTHE_WRITE |
        //ATT_PERMISSION_AUTHO_READ |
        //ATT_PERMISSION_AUTHO_WRITE |
        0x00
    ),
    (
        ATT_TYPE_FORMAT_16UUID |
        //ATT_TYPE_FORMAT_128UUID |
        0x00
    ),
};
#endif


#ifdef _PROFILE_LLS_
const Uint8 ATT_PERMISSION_HDL_LLS_ALERT_LEVEL_INIT[] =
{
    (
        ATT_PERMISSION_READ |
        ATT_PERMISSION_WRITE |
        //ATT_PERMISSION_ENC_READ |
        //ATT_PERMISSION_ENC_WRITE |
        //ATT_PERMISSION_AUTHE_READ |
        //ATT_PERMISSION_AUTHE_WRITE |
        //ATT_PERMISSION_AUTHO_READ |
        //ATT_PERMISSION_AUTHO_WRITE |
        0x00
    ),
    (
        ATT_TYPE_FORMAT_16UUID |
        //ATT_TYPE_FORMAT_128UUID |
        0x00
    ),
};
#endif


#ifdef _PROFILE_TPS_
const Uint8 ATT_PERMISSION_HDL_TPS_TX_POWER_LEVEL_INIT[] =
{
    (
        ATT_PERMISSION_READ |
        //ATT_PERMISSION_WRITE |
        //ATT_PERMISSION_ENC_READ |
        //ATT_PERMISSION_ENC_WRITE |
        //ATT_PERMISSION_AUTHE_READ |
        //ATT_PERMISSION_AUTHE_WRITE |
        //ATT_PERMISSION_AUTHO_READ |
        //ATT_PERMISSION_AUTHO_WRITE |
        0x00
    ),
    (
        ATT_TYPE_FORMAT_16UUID |
        //ATT_TYPE_FORMAT_128UUID |
        0x00
    ),
};
#endif


#ifdef _PROFILE_BLP_
const Uint8 ATT_PERMISSION_HDL_BLS_BLOOD_PRESSURE_MEASUREMENT_INIT[] =
{
    (
        //ATT_PERMISSION_READ |
        //ATT_PERMISSION_WRITE |
        //ATT_PERMISSION_ENC_READ |
        //ATT_PERMISSION_ENC_WRITE |
        //ATT_PERMISSION_AUTHE_READ |
        //ATT_PERMISSION_AUTHE_WRITE |
        //ATT_PERMISSION_AUTHO_READ |
        //ATT_PERMISSION_AUTHO_WRITE |
        0x00
    ),
    (
        ATT_TYPE_FORMAT_16UUID |
        //ATT_TYPE_FORMAT_128UUID |
        0x00
    ),
};


const Uint8 ATT_PERMISSION_HDL_BLS_BLOOD_PRESSURE_CLIENT_CHARACTERISTIC_CONFIGURATION_INIT[] =
{
    (
        ATT_PERMISSION_READ |
        //ATT_PERMISSION_WRITE |
        //ATT_PERMISSION_ENC_READ |
        //ATT_PERMISSION_ENC_WRITE |
        //ATT_PERMISSION_AUTHE_READ |
        ATT_PERMISSION_AUTHE_WRITE |
        //ATT_PERMISSION_AUTHO_READ |
        //ATT_PERMISSION_AUTHO_WRITE |
        0x00
    ),
    (
        ATT_TYPE_FORMAT_16UUID |
        //ATT_TYPE_FORMAT_128UUID |
        0x00
    ),
};


const Uint8 ATT_PERMISSION_HDL_BLS_BLOOD_PRESSURE_FEATURE_INIT[] =
{
    (
        //ATT_PERMISSION_READ |
        //ATT_PERMISSION_WRITE |
        //ATT_PERMISSION_ENC_READ |
        //ATT_PERMISSION_ENC_WRITE |
        ATT_PERMISSION_AUTHE_READ |
        //ATT_PERMISSION_AUTHE_WRITE |
        //ATT_PERMISSION_AUTHO_READ |
        //ATT_PERMISSION_AUTHO_WRITE |
        0x00
    ),
    (
        ATT_TYPE_FORMAT_16UUID |
        //ATT_TYPE_FORMAT_128UUID |
        0x00
    ),
};
#endif


#ifdef _PROFILE_HTP_
const Uint8 ATT_PERMISSION_HDL_HTS_TEMPERATURE_MEASUREMENT_INIT[] =
{
    (
        //ATT_PERMISSION_READ |
        //ATT_PERMISSION_WRITE |
        //ATT_PERMISSION_ENC_READ |
        //ATT_PERMISSION_ENC_WRITE |
        //ATT_PERMISSION_AUTHE_READ |
        //ATT_PERMISSION_AUTHE_WRITE |
        //ATT_PERMISSION_AUTHO_READ |
        //ATT_PERMISSION_AUTHO_WRITE |
        0x00
    ),
    (
        ATT_TYPE_FORMAT_16UUID |
        //ATT_TYPE_FORMAT_128UUID |
        0x00
    ),
};


const Uint8 ATT_PERMISSION_HDL_HTS_TEMPERATURE_MEASUREMENT_CLIENT_CHARACTERISTIC_CONFIGURATION_INIT[] =
{
    (
        ATT_PERMISSION_READ |
        //ATT_PERMISSION_WRITE |
        //ATT_PERMISSION_ENC_READ |
        //ATT_PERMISSION_ENC_WRITE |
        //ATT_PERMISSION_AUTHE_READ |
        ATT_PERMISSION_AUTHE_WRITE |
        //ATT_PERMISSION_AUTHO_READ |
        //ATT_PERMISSION_AUTHO_WRITE |
        0x00
    ),
    (
        ATT_TYPE_FORMAT_16UUID |
        //ATT_TYPE_FORMAT_128UUID |
        0x00
    ),
};


const Uint8 ATT_PERMISSION_HDL_HTS_TEMPERATURE_TYPE_INIT[] =
{
    (
        //ATT_PERMISSION_READ |
        //ATT_PERMISSION_WRITE |
        //ATT_PERMISSION_ENC_READ |
        //ATT_PERMISSION_ENC_WRITE |
        ATT_PERMISSION_AUTHE_READ |
        //ATT_PERMISSION_AUTHE_WRITE |
        //ATT_PERMISSION_AUTHO_READ |
        //ATT_PERMISSION_AUTHO_WRITE |
        0x00
    ),
    (
        ATT_TYPE_FORMAT_16UUID |
        //ATT_TYPE_FORMAT_128UUID |
        0x00
    ),
};
#endif


#ifdef _PROFILE_RSCP_
const Uint8 ATT_PERMISSION_HDL_RSCS_RSC_MEASUREMENT_INIT[] =
{
    (
        //ATT_PERMISSION_READ |
        //ATT_PERMISSION_WRITE |
        //ATT_PERMISSION_ENC_READ |
        //ATT_PERMISSION_ENC_WRITE |
        //ATT_PERMISSION_AUTHE_READ |
        //ATT_PERMISSION_AUTHE_WRITE |
        //ATT_PERMISSION_AUTHO_READ |
        //ATT_PERMISSION_AUTHO_WRITE |
        0x00
    ),
    (
        ATT_TYPE_FORMAT_16UUID |
        //ATT_TYPE_FORMAT_128UUID |
        0x00
    ),
};


const Uint8 ATT_PERMISSION_HDL_RSCS_RSC_MEASUREMENT_CLIENT_CHARACTERISTIC_CONFIGURATION_INIT[] =
{
    (
        ATT_PERMISSION_READ |
        //ATT_PERMISSION_WRITE |
        //ATT_PERMISSION_ENC_READ |
        //ATT_PERMISSION_ENC_WRITE |
        //ATT_PERMISSION_AUTHE_READ |
        ATT_PERMISSION_AUTHE_WRITE |
        //ATT_PERMISSION_AUTHO_READ |
        //ATT_PERMISSION_AUTHO_WRITE |
        0x00
    ),
    (
        ATT_TYPE_FORMAT_16UUID |
        //ATT_TYPE_FORMAT_128UUID |
        0x00
    ),
};


const Uint8 ATT_PERMISSION_HDL_RSCS_RSC_FEATURE_INIT[] =
{
    (
        //ATT_PERMISSION_READ |
        //ATT_PERMISSION_WRITE |
        //ATT_PERMISSION_ENC_READ |
        //ATT_PERMISSION_ENC_WRITE |
        ATT_PERMISSION_AUTHE_READ |
        //ATT_PERMISSION_AUTHE_WRITE |
        //ATT_PERMISSION_AUTHO_READ |
        //ATT_PERMISSION_AUTHO_WRITE |
        0x00
    ),
    (
        ATT_TYPE_FORMAT_16UUID |
        //ATT_TYPE_FORMAT_128UUID |
        0x00
    ),
};


const Uint8 ATT_PERMISSION_HDL_RSCS_SC_CONTROL_POINT_INIT[] =
{
    (
        //ATT_PERMISSION_READ |
        //ATT_PERMISSION_WRITE |
        //ATT_PERMISSION_ENC_READ |
        //ATT_PERMISSION_ENC_WRITE |
        //ATT_PERMISSION_AUTHE_READ |
        ATT_PERMISSION_AUTHE_WRITE |
        //ATT_PERMISSION_AUTHO_READ |
        //ATT_PERMISSION_AUTHO_WRITE |
        0x00
    ),
    (
        ATT_TYPE_FORMAT_16UUID |
        //ATT_TYPE_FORMAT_128UUID |
        0x00
    ),
};


const Uint8 ATT_PERMISSION_HDL_RSCS_SC_CONTROL_POINT_CLIENT_CHARACTERISTIC_CONFIGURATION_INIT[] =
{
    (
        ATT_PERMISSION_READ |
        //ATT_PERMISSION_WRITE |
        //ATT_PERMISSION_ENC_READ |
        //ATT_PERMISSION_ENC_WRITE |
        //ATT_PERMISSION_AUTHE_READ |
        ATT_PERMISSION_AUTHE_WRITE |
        //ATT_PERMISSION_AUTHO_READ |
        //ATT_PERMISSION_AUTHO_WRITE |
        0x00
    ),
    (
        ATT_TYPE_FORMAT_16UUID |
        //ATT_TYPE_FORMAT_128UUID |
        0x00
    ),
};
#endif  //(#ifdef _PROFILE_RSCP_)


#ifdef _PROFILE_CSCP_
const Uint8 ATT_PERMISSION_HDL_CSCS_CSC_MEASUREMENT_INIT[] =
{
    (
        //ATT_PERMISSION_READ |
        //ATT_PERMISSION_WRITE |
        //ATT_PERMISSION_ENC_READ |
        //ATT_PERMISSION_ENC_WRITE |
        //ATT_PERMISSION_AUTHE_READ |
        //ATT_PERMISSION_AUTHE_WRITE |
        //ATT_PERMISSION_AUTHO_READ |
        //ATT_PERMISSION_AUTHO_WRITE |
        0x00
    ),
    (
        ATT_TYPE_FORMAT_16UUID |
        //ATT_TYPE_FORMAT_128UUID |
        0x00
    ),
};


const Uint8 ATT_PERMISSION_HDL_CSCS_CSC_MEASUREMENT_CLIENT_CHARACTERISTIC_CONFIGURATION_INIT[] =
{
    (
        ATT_PERMISSION_READ |
        //ATT_PERMISSION_WRITE |
        //ATT_PERMISSION_ENC_READ |
        //ATT_PERMISSION_ENC_WRITE |
        //ATT_PERMISSION_AUTHE_READ |
        ATT_PERMISSION_AUTHE_WRITE |
        //ATT_PERMISSION_AUTHO_READ |
        //ATT_PERMISSION_AUTHO_WRITE |
        0x00
    ),
    (
        ATT_TYPE_FORMAT_16UUID |
        //ATT_TYPE_FORMAT_128UUID |
        0x00
    ),
};


const Uint8 ATT_PERMISSION_HDL_CSCS_CSC_FEATURE_INIT[] =
{
    (
        //ATT_PERMISSION_READ |
        //ATT_PERMISSION_WRITE |
        //ATT_PERMISSION_ENC_READ |
        //ATT_PERMISSION_ENC_WRITE |
        ATT_PERMISSION_AUTHE_READ |
        //ATT_PERMISSION_AUTHE_WRITE |
        //ATT_PERMISSION_AUTHO_READ |
        //ATT_PERMISSION_AUTHO_WRITE |
        0x00
    ),
    (
        ATT_TYPE_FORMAT_16UUID |
        //ATT_TYPE_FORMAT_128UUID |
        0x00
    ),
};


const Uint8 ATT_PERMISSION_HDL_CSCS_SC_CONTROL_POINT_INIT[] =
{
    (
        //ATT_PERMISSION_READ |
        //ATT_PERMISSION_WRITE |
        //ATT_PERMISSION_ENC_READ |
        //ATT_PERMISSION_ENC_WRITE |
        //ATT_PERMISSION_AUTHE_READ |
        ATT_PERMISSION_AUTHE_WRITE |
        //ATT_PERMISSION_AUTHO_READ |
        //ATT_PERMISSION_AUTHO_WRITE |
        0x00
    ),
    (
        ATT_TYPE_FORMAT_16UUID |
        //ATT_TYPE_FORMAT_128UUID |
        0x00
    ),
};


const Uint8 ATT_PERMISSION_HDL_CSCS_SC_CONTROL_POINT_CLIENT_CHARACTERISTIC_CONFIGURATION_INIT[] =
{
    (
        ATT_PERMISSION_READ |
        //ATT_PERMISSION_WRITE |
        //ATT_PERMISSION_ENC_READ |
        //ATT_PERMISSION_ENC_WRITE |
        //ATT_PERMISSION_AUTHE_READ |
        ATT_PERMISSION_AUTHE_WRITE |
        //ATT_PERMISSION_AUTHO_READ |
        //ATT_PERMISSION_AUTHO_WRITE |
        0x00
    ),
    (
        ATT_TYPE_FORMAT_16UUID |
        //ATT_TYPE_FORMAT_128UUID |
        0x00
    ),
};
#endif


#ifdef _PROFILE_LNS_
const Uint8 ATT_PERMISSION_HDL_LNS_LOCATION_AND_SPEED_INIT[] =
{
    (
        //ATT_PERMISSION_READ |
        //ATT_PERMISSION_WRITE |
        //ATT_PERMISSION_ENC_READ |
        //ATT_PERMISSION_ENC_WRITE |
        //ATT_PERMISSION_AUTHE_READ |
        //ATT_PERMISSION_AUTHE_WRITE |
        //ATT_PERMISSION_AUTHO_READ |
        //ATT_PERMISSION_AUTHO_WRITE |
        0x00
    ),
    (
        ATT_TYPE_FORMAT_16UUID |
        //ATT_TYPE_FORMAT_128UUID |
        0x00
    ),
};


const Uint8 ATT_PERMISSION_HDL_LNS_LOCATION_AND_SPEED_CLIENT_CHARACTERISTIC_CONFIGURATION_INIT[] =
{
    (
        ATT_PERMISSION_READ |
        //ATT_PERMISSION_WRITE |
        //ATT_PERMISSION_ENC_READ |
        //ATT_PERMISSION_ENC_WRITE |
        //ATT_PERMISSION_AUTHE_READ |
        ATT_PERMISSION_AUTHE_WRITE |
        //ATT_PERMISSION_AUTHO_READ |
        //ATT_PERMISSION_AUTHO_WRITE |
        0x00
    ),
    (
        ATT_TYPE_FORMAT_16UUID |
        //ATT_TYPE_FORMAT_128UUID |
        0x00
    ),
};


const Uint8 ATT_PERMISSION_HDL_LNS_LN_FEATURE_INIT[] =
{
    (
        //ATT_PERMISSION_READ |
        //ATT_PERMISSION_WRITE |
        //ATT_PERMISSION_ENC_READ |
        //ATT_PERMISSION_ENC_WRITE |
        ATT_PERMISSION_AUTHE_READ |
        //ATT_PERMISSION_AUTHE_WRITE |
        //ATT_PERMISSION_AUTHO_READ |
        //ATT_PERMISSION_AUTHO_WRITE |
        0x00
    ),
    (
        ATT_TYPE_FORMAT_16UUID |
        //ATT_TYPE_FORMAT_128UUID |
        0x00
    ),
};


const Uint8 ATT_PERMISSION_HDL_LNS_POSITION_QUALITY_INIT[] =
{
    (
        //ATT_PERMISSION_READ |
        //ATT_PERMISSION_WRITE |
        //ATT_PERMISSION_ENC_READ |
        //ATT_PERMISSION_ENC_WRITE |
        ATT_PERMISSION_AUTHE_READ |
        //ATT_PERMISSION_AUTHE_WRITE |
        //ATT_PERMISSION_AUTHO_READ |
        //ATT_PERMISSION_AUTHO_WRITE |
        0x00
    ),
    (
        ATT_TYPE_FORMAT_16UUID |
        //ATT_TYPE_FORMAT_128UUID |
        0x00
    ),
};


const Uint8 ATT_PERMISSION_HDL_LNS_LN_CONTROL_POINT_INIT[] =
{
    (
        //ATT_PERMISSION_READ |
        //ATT_PERMISSION_WRITE |
        //ATT_PERMISSION_ENC_READ |
        //ATT_PERMISSION_ENC_WRITE |
        //ATT_PERMISSION_AUTHE_READ |
        ATT_PERMISSION_AUTHE_WRITE |
        //ATT_PERMISSION_AUTHO_READ |
        //ATT_PERMISSION_AUTHO_WRITE |
        0x00
    ),
    (
        ATT_TYPE_FORMAT_16UUID |
        //ATT_TYPE_FORMAT_128UUID |
        0x00
    ),
};


const Uint8 ATT_PERMISSION_HDL_LNS_LN_CONTROL_POINT_CLIENT_CHARACTERISTIC_CONFIGURATION_INIT[] =
{
    (
        ATT_PERMISSION_READ |
        //ATT_PERMISSION_WRITE |
        //ATT_PERMISSION_ENC_READ |
        //ATT_PERMISSION_ENC_WRITE |
        //ATT_PERMISSION_AUTHE_READ |
        ATT_PERMISSION_AUTHE_WRITE |
        //ATT_PERMISSION_AUTHO_READ |
        //ATT_PERMISSION_AUTHO_WRITE |
        0x00
    ),
    (
        ATT_TYPE_FORMAT_16UUID |
        //ATT_TYPE_FORMAT_128UUID |
        0x00
    ),
};


const Uint8 ATT_PERMISSION_HDL_LNS_NAVIGATION_INIT[] =
{
    (
        //ATT_PERMISSION_READ |
        //ATT_PERMISSION_WRITE |
        //ATT_PERMISSION_ENC_READ |
        //ATT_PERMISSION_ENC_WRITE |
        //ATT_PERMISSION_AUTHE_READ |
        //ATT_PERMISSION_AUTHE_WRITE |
        //ATT_PERMISSION_AUTHO_READ |
        //ATT_PERMISSION_AUTHO_WRITE |
        0x00
    ),
    (
        ATT_TYPE_FORMAT_16UUID |
        //ATT_TYPE_FORMAT_128UUID |
        0x00
    ),
};


const Uint8 ATT_PERMISSION_HDL_LNS_NAVIGATION_CLIENT_CHARACTERISTIC_CONFIGURATION_INIT[] =
{
    (
        ATT_PERMISSION_READ |
        //ATT_PERMISSION_WRITE |
        //ATT_PERMISSION_ENC_READ |
        //ATT_PERMISSION_ENC_WRITE |
        //ATT_PERMISSION_AUTHE_READ |
        ATT_PERMISSION_AUTHE_WRITE |
        //ATT_PERMISSION_AUTHO_READ |
        //ATT_PERMISSION_AUTHO_WRITE |
        0x00
    ),
    (
        ATT_TYPE_FORMAT_16UUID |
        //ATT_TYPE_FORMAT_128UUID |
        0x00
    ),
};
#endif


#ifdef _PROFILE_GLS_
const Uint8 ATT_PERMISSION_HDL_GLS_GLUCOSE_MEASUREMENT_INIT[] =
{
    (
        //ATT_PERMISSION_READ |
        //ATT_PERMISSION_WRITE |
        //ATT_PERMISSION_ENC_READ |
        //ATT_PERMISSION_ENC_WRITE |
        //ATT_PERMISSION_AUTHE_READ |
        //ATT_PERMISSION_AUTHE_WRITE |
        //ATT_PERMISSION_AUTHO_READ |
        //ATT_PERMISSION_AUTHO_WRITE |
        0x00
    ),
    (
        ATT_TYPE_FORMAT_16UUID |
        //ATT_TYPE_FORMAT_128UUID |
        0x00
    ),
};


const Uint8 ATT_PERMISSION_HDL_GLS_GLUCOSE_MEASUREMENT_CLIENT_CHARACTERISTIC_CONFIGURATION_INIT[] =
{
    (
        ATT_PERMISSION_READ |
        //ATT_PERMISSION_WRITE |
        //ATT_PERMISSION_ENC_READ |
        //ATT_PERMISSION_ENC_WRITE |
        //ATT_PERMISSION_AUTHE_READ |
        ATT_PERMISSION_AUTHE_WRITE |
        //ATT_PERMISSION_AUTHO_READ |
        //ATT_PERMISSION_AUTHO_WRITE |
        0x00
    ),
    (
        ATT_TYPE_FORMAT_16UUID |
        //ATT_TYPE_FORMAT_128UUID |
        0x00
    ),
};


const Uint8 ATT_PERMISSION_HDL_GLS_GLUCOSE_MEASUREMENT_CONTEXT_INIT[] =
{
    (
        //ATT_PERMISSION_READ |
        //ATT_PERMISSION_WRITE |
        //ATT_PERMISSION_ENC_READ |
        //ATT_PERMISSION_ENC_WRITE |
        //ATT_PERMISSION_AUTHE_READ |
        //ATT_PERMISSION_AUTHE_WRITE |
        //ATT_PERMISSION_AUTHO_READ |
        //ATT_PERMISSION_AUTHO_WRITE |
        0x00
    ),
    (
        ATT_TYPE_FORMAT_16UUID |
        //ATT_TYPE_FORMAT_128UUID |
        0x00
    ),
};


const Uint8 ATT_PERMISSION_HDL_GLS_GLUCOSE_MEASUREMENT_CONTEXT_CLIENT_CHARACTERISTIC_CONFIGURATION_INIT[] =
{
    (
        ATT_PERMISSION_READ |
        //ATT_PERMISSION_WRITE |
        //ATT_PERMISSION_ENC_READ |
        //ATT_PERMISSION_ENC_WRITE |
        //ATT_PERMISSION_AUTHE_READ |
        ATT_PERMISSION_AUTHE_WRITE |
        //ATT_PERMISSION_AUTHO_READ |
        //ATT_PERMISSION_AUTHO_WRITE |
        0x00
    ),
    (
        ATT_TYPE_FORMAT_16UUID |
        //ATT_TYPE_FORMAT_128UUID |
        0x00
    ),
};


const Uint8 ATT_PERMISSION_HDL_GLS_GLUCOSE_FEATURE_INIT[] =
{
    (
        ATT_PERMISSION_READ |
        //ATT_PERMISSION_WRITE |
        //ATT_PERMISSION_ENC_READ |
        //ATT_PERMISSION_ENC_WRITE |
        //ATT_PERMISSION_AUTHE_READ |
        //ATT_PERMISSION_AUTHE_WRITE |
        //ATT_PERMISSION_AUTHO_READ |
        //ATT_PERMISSION_AUTHO_WRITE |
        0x00
    ),
    (
        ATT_TYPE_FORMAT_16UUID |
        //ATT_TYPE_FORMAT_128UUID |
        0x00
    ),
};


const Uint8 ATT_PERMISSION_HDL_GLS_RECORD_ACCESS_CONTROL_POINT_INIT[] =
{
    (
        //ATT_PERMISSION_READ |
        //ATT_PERMISSION_WRITE |
        //ATT_PERMISSION_ENC_READ |
        //ATT_PERMISSION_ENC_WRITE |
        //ATT_PERMISSION_AUTHE_READ |
        ATT_PERMISSION_AUTHE_WRITE |
        //ATT_PERMISSION_AUTHO_READ |
        //ATT_PERMISSION_AUTHO_WRITE |
        0x00
    ),
    (
        ATT_TYPE_FORMAT_16UUID |
        //ATT_TYPE_FORMAT_128UUID |
        0x00
    ),
};


const Uint8 ATT_PERMISSION_HDL_GLS_RECORD_ACCESS_CONTROL_POINT_CLIENT_CHARACTERISTIC_CONFIGURATION_INIT[] =
{
    (
        ATT_PERMISSION_READ |
        //ATT_PERMISSION_WRITE |
        //ATT_PERMISSION_ENC_READ |
        //ATT_PERMISSION_ENC_WRITE |
        //ATT_PERMISSION_AUTHE_READ |
        ATT_PERMISSION_AUTHE_WRITE |
        //ATT_PERMISSION_AUTHO_READ |
        //ATT_PERMISSION_AUTHO_WRITE |
        0x00
    ),
    (
        ATT_TYPE_FORMAT_16UUID |
        //ATT_TYPE_FORMAT_128UUID |
        0x00
    ),
};
#endif


#ifdef _PROFILE_CPS_
const Uint8 ATT_PERMISSION_HDL_CPS_CYCLING_POWER_MEASUREMENT_INIT[] =
{
    (
        //ATT_PERMISSION_READ |
        //ATT_PERMISSION_WRITE |
        //ATT_PERMISSION_ENC_READ |
        //ATT_PERMISSION_ENC_WRITE |
        //ATT_PERMISSION_AUTHE_READ |
        //ATT_PERMISSION_AUTHE_WRITE |
        //ATT_PERMISSION_AUTHO_READ |
        //ATT_PERMISSION_AUTHO_WRITE |
        0x00
    ),
    (
        ATT_TYPE_FORMAT_16UUID |
        //ATT_TYPE_FORMAT_128UUID |
        0x00
    ),
};


const Uint8 ATT_PERMISSION_HDL_CPS_CYCLING_POWER_MEASUREMENT_CLIENT_CHARACTERISTIC_CONFIGURATION_INIT[] =
{
    (
        ATT_PERMISSION_READ |
        //ATT_PERMISSION_WRITE |
        //ATT_PERMISSION_ENC_READ |
        //ATT_PERMISSION_ENC_WRITE |
        //ATT_PERMISSION_AUTHE_READ |
        ATT_PERMISSION_AUTHE_WRITE |
        //ATT_PERMISSION_AUTHO_READ |
        //ATT_PERMISSION_AUTHO_WRITE |
        0x00
    ),
    (
        ATT_TYPE_FORMAT_16UUID |
        //ATT_TYPE_FORMAT_128UUID |
        0x00
    ),
};


const Uint8 ATT_PERMISSION_HDL_CPS_CYCLING_POWER_MEASUREMENT_SERVER_CHARACTERISTIC_CONFIGURATION_INIT[] =
{
    (
        ATT_PERMISSION_READ |
        //ATT_PERMISSION_WRITE |
        //ATT_PERMISSION_ENC_READ |
        //ATT_PERMISSION_ENC_WRITE |
        //ATT_PERMISSION_AUTHE_READ |
        ATT_PERMISSION_AUTHE_WRITE |
        //ATT_PERMISSION_AUTHO_READ |
        //ATT_PERMISSION_AUTHO_WRITE |
        0x00
    ),
    (
        ATT_TYPE_FORMAT_16UUID |
        //ATT_TYPE_FORMAT_128UUID |
        0x00
    ),
};


const Uint8 ATT_PERMISSION_HDL_CPS_CYCLING_POWER_FEATURE_INIT[] =
{
    (
        //ATT_PERMISSION_READ |
        //ATT_PERMISSION_WRITE |
        //ATT_PERMISSION_ENC_READ |
        //ATT_PERMISSION_ENC_WRITE |
        ATT_PERMISSION_AUTHE_READ |
        //ATT_PERMISSION_AUTHE_WRITE |
        //ATT_PERMISSION_AUTHO_READ |
        //ATT_PERMISSION_AUTHO_WRITE |
        0x00
    ),
    (
        ATT_TYPE_FORMAT_16UUID |
        //ATT_TYPE_FORMAT_128UUID |
        0x00
    ),
};


const Uint8 ATT_PERMISSION_HDL_CPS_SENSOR_LOCATION_INIT[] =
{
    (
        //ATT_PERMISSION_READ |
        //ATT_PERMISSION_WRITE |
        //ATT_PERMISSION_ENC_READ |
        //ATT_PERMISSION_ENC_WRITE |
        ATT_PERMISSION_AUTHE_READ |
        //ATT_PERMISSION_AUTHE_WRITE |
        //ATT_PERMISSION_AUTHO_READ |
        //ATT_PERMISSION_AUTHO_WRITE |
        0x00
    ),
    (
        ATT_TYPE_FORMAT_16UUID |
        //ATT_TYPE_FORMAT_128UUID |
        0x00
    ),
};


const Uint8 ATT_PERMISSION_HDL_CPS_CYCLING_POWER_VECTOR_INIT[] =
{
    (
        //ATT_PERMISSION_READ |
        //ATT_PERMISSION_WRITE |
        //ATT_PERMISSION_ENC_READ |
        //ATT_PERMISSION_ENC_WRITE |
        //ATT_PERMISSION_AUTHE_READ |
        //ATT_PERMISSION_AUTHE_WRITE |
        //ATT_PERMISSION_AUTHO_READ |
        //ATT_PERMISSION_AUTHO_WRITE |
        0x00
    ),
    (
        ATT_TYPE_FORMAT_16UUID |
        //ATT_TYPE_FORMAT_128UUID |
        0x00
    ),
};


const Uint8 ATT_PERMISSION_HDL_CPS_CYCLING_POWER_VECTOR_CLIENT_CHARACTERISTIC_CONFIGURATION_INIT[] =
{
    (
        ATT_PERMISSION_READ |
        //ATT_PERMISSION_WRITE |
        //ATT_PERMISSION_ENC_READ |
        //ATT_PERMISSION_ENC_WRITE |
        //ATT_PERMISSION_AUTHE_READ |
        ATT_PERMISSION_AUTHE_WRITE |
        //ATT_PERMISSION_AUTHO_READ |
        //ATT_PERMISSION_AUTHO_WRITE |
        0x00
    ),
    (
        ATT_TYPE_FORMAT_16UUID |
        //ATT_TYPE_FORMAT_128UUID |
        0x00
    ),
};


const Uint8 ATT_PERMISSION_HDL_CPS_CYCLING_POWER_CONTROL_POINT_INIT[] =
{
    (
        //ATT_PERMISSION_READ |
        //ATT_PERMISSION_WRITE |
        //ATT_PERMISSION_ENC_READ |
        //ATT_PERMISSION_ENC_WRITE |
        //ATT_PERMISSION_AUTHE_READ |
        ATT_PERMISSION_AUTHE_WRITE |
        //ATT_PERMISSION_AUTHO_READ |
        //ATT_PERMISSION_AUTHO_WRITE |
        0x00
    ),
    (
        ATT_TYPE_FORMAT_16UUID |
        //ATT_TYPE_FORMAT_128UUID |
        0x00
    ),
};


const Uint8 ATT_PERMISSION_HDL_CPS_CYCLING_POWER_CONTROL_POINT_CLIENT_CHARACTERISTIC_CONFIGURATION_INIT[] =
{
    (
        ATT_PERMISSION_READ |
        //ATT_PERMISSION_WRITE |
        //ATT_PERMISSION_ENC_READ |
        //ATT_PERMISSION_ENC_WRITE |
        //ATT_PERMISSION_AUTHE_READ |
        ATT_PERMISSION_AUTHE_WRITE |
        //ATT_PERMISSION_AUTHO_READ |
        //ATT_PERMISSION_AUTHO_WRITE |
        0x00
    ),
    (
        ATT_TYPE_FORMAT_16UUID |
        //ATT_TYPE_FORMAT_128UUID |
        0x00
    ),
};
#endif


#ifdef _PROFILE_CSTM_UDF01S_
const Uint8 ATT_PERMISSION_HDL_UDF01S_UDATR01[] =
{
    (
        ATT_PERMISSION_READ |
        //ATT_PERMISSION_WRITE |
        //ATT_PERMISSION_ENC_READ |
        //ATT_PERMISSION_ENC_WRITE |
        //ATT_PERMISSION_AUTHE_READ |
        //ATT_PERMISSION_AUTHE_WRITE |
        //ATT_PERMISSION_AUTHO_READ |
        //ATT_PERMISSION_AUTHO_WRITE |
        0x00
    ),
    (
        //ATT_TYPE_FORMAT_16UUID |
        ATT_TYPE_FORMAT_128UUID |
        0x00
    ),
};


const Uint8 ATT_PERMISSION_HDL_UDF01S_UDATN01_INIT[] =
{
    (
        ATT_PERMISSION_READ |
        //ATT_PERMISSION_WRITE |
        //ATT_PERMISSION_ENC_READ |
        //ATT_PERMISSION_ENC_WRITE |
        //ATT_PERMISSION_AUTHE_READ |
        //ATT_PERMISSION_AUTHE_WRITE |
        //ATT_PERMISSION_AUTHO_READ |
        //ATT_PERMISSION_AUTHO_WRITE |
        0x00
    ),
    (
        ATT_TYPE_FORMAT_16UUID |
        //ATT_TYPE_FORMAT_128UUID |
        0x00
    ),
};


const Uint8 ATT_PERMISSION_HDL_UDF01S_UDATN01_CLIENT_CHARACTERISTIC_CONFIGURATION_INIT[] =
{
    (
        ATT_PERMISSION_READ |
        ATT_PERMISSION_WRITE |
        //ATT_PERMISSION_ENC_READ |
        //ATT_PERMISSION_ENC_WRITE |
        //ATT_PERMISSION_AUTHE_READ |
        //ATT_PERMISSION_AUTHE_WRITE |
        //ATT_PERMISSION_AUTHO_READ |
        //ATT_PERMISSION_AUTHO_WRITE |
        0x00
    ),
    (
        ATT_TYPE_FORMAT_16UUID |
        //ATT_TYPE_FORMAT_128UUID |
        0x00
    ),
};


const Uint8 ATT_PERMISSION_HDL_UDF01S_UDATRW01_INIT[] =
{
    (
        ATT_PERMISSION_READ |
        ATT_PERMISSION_WRITE |
        //ATT_PERMISSION_ENC_READ |
        //ATT_PERMISSION_ENC_WRITE |
        //ATT_PERMISSION_AUTHE_READ |
        //ATT_PERMISSION_AUTHE_WRITE |
        //ATT_PERMISSION_AUTHO_READ |
        //ATT_PERMISSION_AUTHO_WRITE |
        0x00
    ),
    (
        //ATT_TYPE_FORMAT_16UUID |
        ATT_TYPE_FORMAT_128UUID |
        //ATT_TYPE_FORMAT_DYN_LLENGTH |
        //ATT_TYPE_FORMAT_ATT_CHR_ACSS_USER |
        0x00
    ),
};
#endif









/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
const Uint8 ATT_PERMISSION_HDL_UDF_UDATR01[] =
{
    (
        ATT_PERMISSION_READ |
        //ATT_PERMISSION_WRITE |
        //ATT_PERMISSION_ENC_READ |
        //ATT_PERMISSION_ENC_WRITE |
        //ATT_PERMISSION_AUTHE_READ |
        //ATT_PERMISSION_AUTHE_WRITE |
        //ATT_PERMISSION_AUTHO_READ |
        //ATT_PERMISSION_AUTHO_WRITE |
        0x00
    ),
    (
        //ATT_TYPE_FORMAT_16UUID |
        ATT_TYPE_FORMAT_128UUID |
        0x00
    ),
};


const Uint8 ATT_PERMISSION_HDL_UDF_UDATN01_INIT[] =
{
    (
        ATT_PERMISSION_READ |
        //ATT_PERMISSION_WRITE |
        //ATT_PERMISSION_ENC_READ |
        //ATT_PERMISSION_ENC_WRITE |
        //ATT_PERMISSION_AUTHE_READ |
        //ATT_PERMISSION_AUTHE_WRITE |
        //ATT_PERMISSION_AUTHO_READ |
        //ATT_PERMISSION_AUTHO_WRITE |
        0x00
    ),
    (
        ATT_TYPE_FORMAT_16UUID |
        //ATT_TYPE_FORMAT_128UUID |
        0x00
    ),
};


const Uint8 ATT_PERMISSION_HDL_UDF_UDATN01_CLIENT_CHARACTERISTIC_CONFIGURATION_INIT[] =
{
    (
        ATT_PERMISSION_READ |
        ATT_PERMISSION_WRITE |
        //ATT_PERMISSION_ENC_READ |
        //ATT_PERMISSION_ENC_WRITE |
        //ATT_PERMISSION_AUTHE_READ |
        //ATT_PERMISSION_AUTHE_WRITE |
        //ATT_PERMISSION_AUTHO_READ |
        //ATT_PERMISSION_AUTHO_WRITE |
        0x00
    ),
    (
        ATT_TYPE_FORMAT_16UUID |
        //ATT_TYPE_FORMAT_128UUID |
        0x00
    ),
};


const Uint8 ATT_PERMISSION_HDL_UDF_UDATRW01_INIT[] =
{
    (
        ATT_PERMISSION_READ |
        ATT_PERMISSION_WRITE |
        //ATT_PERMISSION_ENC_READ |
        //ATT_PERMISSION_ENC_WRITE |
        //ATT_PERMISSION_AUTHE_READ |
        //ATT_PERMISSION_AUTHE_WRITE |
        //ATT_PERMISSION_AUTHO_READ |
        //ATT_PERMISSION_AUTHO_WRITE |
        0x00
    ),
    (
        //ATT_TYPE_FORMAT_16UUID |
        ATT_TYPE_FORMAT_128UUID |
        //ATT_TYPE_FORMAT_DYN_LLENGTH |
        //ATT_TYPE_FORMAT_ATT_CHR_ACSS_USER |
        0x00
    ),
};
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////









#ifdef _PROFILE_CSTM_UDF02S_
const Uint8 ATT_PERMISSION_HDL_UDF02S_UDATR02[] =
{
    (
        ATT_PERMISSION_READ |
        //ATT_PERMISSION_WRITE |
        //ATT_PERMISSION_ENC_READ |
        //ATT_PERMISSION_ENC_WRITE |
        //ATT_PERMISSION_AUTHE_READ |
        //ATT_PERMISSION_AUTHE_WRITE |
        //ATT_PERMISSION_AUTHO_READ |
        //ATT_PERMISSION_AUTHO_WRITE |
        0x00
    ),
    (
        ATT_TYPE_FORMAT_16UUID |
        //ATT_TYPE_FORMAT_128UUID |
        0x00
    ),
};


const Uint8 ATT_PERMISSION_HDL_UDF02S_UDATN02_INIT[] =
{
    (
        ATT_PERMISSION_READ |
        //ATT_PERMISSION_WRITE |
        //ATT_PERMISSION_ENC_READ |
        //ATT_PERMISSION_ENC_WRITE |
        //ATT_PERMISSION_AUTHE_READ |
        //ATT_PERMISSION_AUTHE_WRITE |
        //ATT_PERMISSION_AUTHO_READ |
        //ATT_PERMISSION_AUTHO_WRITE |
        0x00
    ),
    (
        ATT_TYPE_FORMAT_16UUID |
        //ATT_TYPE_FORMAT_128UUID |
        0x00
    ),
};


const Uint8 ATT_PERMISSION_HDL_UDF02S_UDATN02_CLIENT_CHARACTERISTIC_CONFIGURATION_INIT[] =
{
    (
        ATT_PERMISSION_READ |
        ATT_PERMISSION_WRITE |
        //ATT_PERMISSION_ENC_READ |
        //ATT_PERMISSION_ENC_WRITE |
        //ATT_PERMISSION_AUTHE_READ |
        //ATT_PERMISSION_AUTHE_WRITE |
        //ATT_PERMISSION_AUTHO_READ |
        //ATT_PERMISSION_AUTHO_WRITE |
        0x00
    ),
    (
        ATT_TYPE_FORMAT_16UUID |
        //ATT_TYPE_FORMAT_128UUID |
        0x00
    ),
};


const Uint8 ATT_PERMISSION_HDL_UDF02S_UDATRW02_INIT[] =
{
    (
        ATT_PERMISSION_READ |
        ATT_PERMISSION_WRITE |
        //ATT_PERMISSION_ENC_READ |
        //ATT_PERMISSION_ENC_WRITE |
        //ATT_PERMISSION_AUTHE_READ |
        //ATT_PERMISSION_AUTHE_WRITE |
        //ATT_PERMISSION_AUTHO_READ |
        //ATT_PERMISSION_AUTHO_WRITE |
        0x00
    ),
    (
        ATT_TYPE_FORMAT_16UUID |
        //ATT_TYPE_FORMAT_128UUID |
        0x00
    ),
};
#endif


#ifdef _PROFILE_CSTM_UDF03S_
const Uint8 ATT_PERMISSION_HDL_UDF03S_UDATR03[] =
{
    (
        ATT_PERMISSION_READ |
        //ATT_PERMISSION_WRITE |
        //ATT_PERMISSION_ENC_READ |
        //ATT_PERMISSION_ENC_WRITE |
        //ATT_PERMISSION_AUTHE_READ |
        //ATT_PERMISSION_AUTHE_WRITE |
        //ATT_PERMISSION_AUTHO_READ |
        //ATT_PERMISSION_AUTHO_WRITE |
        0x00
    ),
    (
        ATT_TYPE_FORMAT_16UUID |
        //ATT_TYPE_FORMAT_128UUID |
        0x00
    ),
};


const Uint8 ATT_PERMISSION_HDL_UDF03S_UDATN03_INIT[] =
{
    (
        ATT_PERMISSION_READ |
        //ATT_PERMISSION_WRITE |
        //ATT_PERMISSION_ENC_READ |
        //ATT_PERMISSION_ENC_WRITE |
        //ATT_PERMISSION_AUTHE_READ |
        //ATT_PERMISSION_AUTHE_WRITE |
        //ATT_PERMISSION_AUTHO_READ |
        //ATT_PERMISSION_AUTHO_WRITE |
        0x00
    ),
    (
        ATT_TYPE_FORMAT_16UUID |
        //ATT_TYPE_FORMAT_128UUID |
        0x00
    ),
};


const Uint8 ATT_PERMISSION_HDL_UDF03S_UDATN03_CLIENT_CHARACTERISTIC_CONFIGURATION_INIT[] =
{
    (
        ATT_PERMISSION_READ |
        ATT_PERMISSION_WRITE |
        //ATT_PERMISSION_ENC_READ |
        //ATT_PERMISSION_ENC_WRITE |
        //ATT_PERMISSION_AUTHE_READ |
        //ATT_PERMISSION_AUTHE_WRITE |
        //ATT_PERMISSION_AUTHO_READ |
        //ATT_PERMISSION_AUTHO_WRITE |
        0x00
    ),
    (
        ATT_TYPE_FORMAT_16UUID |
        //ATT_TYPE_FORMAT_128UUID |
        0x00
    ),
};


const Uint8 ATT_PERMISSION_HDL_UDF03S_UDATRW03_INIT[] =
{
    (
        ATT_PERMISSION_READ |
        ATT_PERMISSION_WRITE |
        //ATT_PERMISSION_ENC_READ |
        //ATT_PERMISSION_ENC_WRITE |
        //ATT_PERMISSION_AUTHE_READ |
        //ATT_PERMISSION_AUTHE_WRITE |
        //ATT_PERMISSION_AUTHO_READ |
        //ATT_PERMISSION_AUTHO_WRITE |
        0x00
    ),
    (
        ATT_TYPE_FORMAT_16UUID |
        //ATT_TYPE_FORMAT_128UUID |
        0x00
    ),
};
#endif


#ifdef _PROFILE_OTA_
const Uint8 ATT_PERMISSION_HDL_OTA_OTA_DATA_INIT[] =
{
    (
        //ATT_PERMISSION_READ |
        //ATT_PERMISSION_WRITE |
        //ATT_PERMISSION_ENC_READ |
        //ATT_PERMISSION_ENC_WRITE |
        //ATT_PERMISSION_AUTHE_READ |
        ATT_PERMISSION_AUTHE_WRITE |
        //ATT_PERMISSION_AUTHO_READ |
        //ATT_PERMISSION_AUTHO_WRITE |
        0x00
    ),
    (
        ATT_TYPE_FORMAT_16UUID |
        //ATT_TYPE_FORMAT_128UUID |
        0x00
    ),
};


const Uint8 ATT_PERMISSION_HDL_OTA_OTA_DATA_CLIENT_CHARACTERISTIC_CONFIGURATION_INIT[] =
{
    (
        ATT_PERMISSION_READ |
        //ATT_PERMISSION_WRITE |
        //ATT_PERMISSION_ENC_READ |
        //ATT_PERMISSION_ENC_WRITE |
        //ATT_PERMISSION_AUTHE_READ |
        ATT_PERMISSION_AUTHE_WRITE |
        //ATT_PERMISSION_AUTHO_READ |
        //ATT_PERMISSION_AUTHO_WRITE |
        0x00
    ),
    (
        ATT_TYPE_FORMAT_16UUID |
        //ATT_TYPE_FORMAT_128UUID |
        0x00
    ),
};


const Uint8 ATT_PERMISSION_HDL_OTA_OTA_DATA_DESCRIPTION[] =
{
    (
        //ATT_PERMISSION_READ |
        //ATT_PERMISSION_WRITE |
        //ATT_PERMISSION_ENC_READ |
        //ATT_PERMISSION_ENC_WRITE |
        ATT_PERMISSION_AUTHE_READ |
        //ATT_PERMISSION_AUTHE_WRITE |
        //ATT_PERMISSION_AUTHO_READ |
        //ATT_PERMISSION_AUTHO_WRITE |
        0x00
    ),
    (
        ATT_TYPE_FORMAT_16UUID |
        //ATT_TYPE_FORMAT_128UUID |
        0x00
    ),
};


const Uint8 ATT_PERMISSION_HDL_OTA_OTA_CMD_CONTROL_POINT_INIT[] =
{
    (
        //ATT_PERMISSION_READ |
        //ATT_PERMISSION_WRITE |
        //ATT_PERMISSION_ENC_READ |
        //ATT_PERMISSION_ENC_WRITE |
        //ATT_PERMISSION_AUTHE_READ |
        ATT_PERMISSION_AUTHE_WRITE |
        //ATT_PERMISSION_AUTHO_READ |
        //ATT_PERMISSION_AUTHO_WRITE |
        0x00
    ),
    (
        ATT_TYPE_FORMAT_16UUID |
        //ATT_TYPE_FORMAT_128UUID |
        0x00
    ),
};


const Uint8 ATT_PERMISSION_HDL_OTA_OTA_CMD_CP_CLIENT_CHARACTERISTIC_CONFIGURATION_INIT[] =
{
    (
        ATT_PERMISSION_READ |
        //ATT_PERMISSION_WRITE |
        //ATT_PERMISSION_ENC_READ |
        //ATT_PERMISSION_ENC_WRITE |
        //ATT_PERMISSION_AUTHE_READ |
        ATT_PERMISSION_AUTHE_WRITE |
        //ATT_PERMISSION_AUTHO_READ |
        //ATT_PERMISSION_AUTHO_WRITE |
        0x00
    ),
    (
        ATT_TYPE_FORMAT_16UUID |
        //ATT_TYPE_FORMAT_128UUID |
        0x00
    ),
};


const Uint8 ATT_PERMISSION_HDL_OTA_OTA_CMD_USER_DESCRIPTION[] =
{
    (
        //ATT_PERMISSION_READ |
        //ATT_PERMISSION_WRITE |
        //ATT_PERMISSION_ENC_READ |
        //ATT_PERMISSION_ENC_WRITE |
        ATT_PERMISSION_AUTHE_READ |
        //ATT_PERMISSION_AUTHE_WRITE |
        //ATT_PERMISSION_AUTHO_READ |
        //ATT_PERMISSION_AUTHO_WRITE |
        0x00
    ),
    (
        ATT_TYPE_FORMAT_16UUID |
        //ATT_TYPE_FORMAT_128UUID |
        0x00
    ),
};
#endif

#ifdef _PROFILE_CSTM_DATAEXCHANGE_S_
const Uint8 ATT_PERMISSION_HDL_DATAEXCHANGE_WRITE[] =
{
    (
        //ATT_PERMISSION_READ |
        ATT_PERMISSION_WRITE |
        //ATT_PERMISSION_ENC_READ |
        //ATT_PERMISSION_ENC_WRITE |
        //ATT_PERMISSION_AUTHE_READ |
        //ATT_PERMISSION_AUTHE_WRITE |
        //ATT_PERMISSION_AUTHO_READ |
        //ATT_PERMISSION_AUTHO_WRITE |
        0x00
    ),
    (
        //ATT_TYPE_FORMAT_16UUID |
        ATT_TYPE_FORMAT_128UUID |
        //ATT_TYPE_FORMAT_DYN_LLENGTH |
        0x00
    ),
};


const Uint8 ATT_PERMISSION_HDL_DATAEXCHANGE_READINT_INIT[] =
{
    (
        ATT_PERMISSION_READ |
        //ATT_PERMISSION_WRITE |
        //ATT_PERMISSION_ENC_READ |
        //ATT_PERMISSION_ENC_WRITE |
        //ATT_PERMISSION_AUTHE_READ |
        //ATT_PERMISSION_AUTHE_WRITE |
        //ATT_PERMISSION_AUTHO_READ |
        //ATT_PERMISSION_AUTHO_WRITE |
        0x00
    ),
    (
        //ATT_TYPE_FORMAT_16UUID |
        ATT_TYPE_FORMAT_128UUID |
        0x00
    ),
};


const Uint8 ATT_PERMISSION_HDL_DATAEXCHANGE_READINT_CLIENT_CHARACTERISTIC_CONFIGURATION_INIT[] =
{
    (
        ATT_PERMISSION_READ |
        ATT_PERMISSION_WRITE |
        //ATT_PERMISSION_ENC_READ |
        //ATT_PERMISSION_ENC_WRITE |
        //ATT_PERMISSION_AUTHE_READ |
        //ATT_PERMISSION_AUTHE_WRITE |
        //ATT_PERMISSION_AUTHO_READ |
        //ATT_PERMISSION_AUTHO_WRITE |
        0x00
    ),
    (
        ATT_TYPE_FORMAT_16UUID |
        //ATT_TYPE_FORMAT_128UUID |
        0x00
    ),
};


#endif


/*!
    \brief  Table combines the content of handles corresponding to each GATT declarations and characteristics.
*/
const Uint8 *ATTRIBUTE_SERVER_PARAM[SIZE_ATTRIBUTE_SERVER] =
{
    &ATT_HDL_INVALID[5],
    &ATT_HDL_GAP_PRIMARY_SERVICE[5],
    &ATT_HDL_GAP_CHARACTERISTIC_DEVICE_NAME[5],
    //&ATT_HDL_GAP_DEVICE_NAME_INIT[5],
    att_HDL_GAP_DEVICE_NAME,
    &ATT_HDL_GAP_CHARACTERISTIC_APPEARANCE[5],
    &ATT_HDL_GAP_APPEARANCE[5],
    &ATT_HDL_GAP_CHARACTERISTIC_RECONNECTION_ADDRESS[5],
    att_HDL_GAP_RECONNECTION_ADDRESS,
    &ATT_HDL_GAP_CHARACTERISTIC_PERIPHERAL_PRIVACY_FLAG[5],
    &ATT_HDL_GAP_PERIPHERAL_PRIVACY_FLAG[5],
    &ATT_HDL_GAP_CHARACTERISTIC_PERIPHERAL_PREFERRED_CONNECTION_PARAMETERS[5],
    &ATT_HDL_GAP_PERIPHERAL_PREFERRED_CONNECTION_PARAMETERS[5],
    &ATT_HDL_GATT_PRIMARY_SERVICE[5],
    &ATT_HDL_GATT_CHARACTERISTIC_SERVICE_CHANGED[5],
    att_HDL_GATT_SERVICE_CHANGED,
    att_HDL_GATT_CLIENT_CHARACTERISTIC_CONFIGURATION,
    &ATT_HDL_DIS_PRIMARY_SERVICE[5],
    &ATT_HDL_DIS_CHARACTERISTIC_SERIAL_NUMBER_STRING[5],
    &ATT_HDL_DIS_SERIAL_NUMBER_STRING[5],
    &ATT_HDL_DIS_SERIAL_NUMBER_STRING_PRESENTATION_FORMAT[5],
    &ATT_HDL_DIS_CHARACTERISTIC_MANUFACTURER_NAME_STRING[5],
    &ATT_HDL_DIS_MANUFACTURER_NAME_STRING[5],
    &ATT_HDL_DIS_MANUFACTURER_NAME_STRING_PRESENTATION_FORMAT[5],
    &ATT_HDL_DIS_CHARACTERISTIC_SYSTEM_ID[5],
    &ATT_HDL_DIS_SYSTEM_ID[5],
    &ATT_HDL_DIS_CHARACTERISTIC_FIRMWARE_REVISION_STRING[5],
    &ATT_HDL_DIS_FIRMWARE_REVISION_STRING[5],
    &ATT_HDL_DIS_FIRMWARE_REVISION_STRING_PRESENTATION_FORMAT[5],
    &ATT_HDL_DIS_CHARACTERISTIC_MODEL_NUMBER_STRING[5],
    &ATT_HDL_DIS_MODEL_NUMBER_STRING[5],
    &ATT_HDL_DIS_MODEL_NUMBER_STRING_PRESENTATION_FORMAT[5],
    &ATT_HDL_DIS_CHARACTERISTIC_HARDWARE_REVISION_STRING[5],
    &ATT_HDL_DIS_HARDWARE_REVISION_STRING[5],
    &ATT_HDL_DIS_HARDWARE_REVISION_STRING_PRESENTATION_FORMAT[5],
    &ATT_HDL_DIS_CHARACTERISTIC_SOFTWARE_REVISION_STRING[5],
    &ATT_HDL_DIS_SOFTWARE_REVISION_STRING[5],
    &ATT_HDL_DIS_SOFTWARE_REVISION_STRING_PRESENTATION_FORMAT[5],
    &ATT_HDL_DIS_CHARACTERISTIC_PNP_ID[5],
    &ATT_HDL_DIS_PNP_ID[5],
#ifdef _PROFILE_HRP_
    &ATT_HDL_HRS_PRIMARY_SERVICE[5],
    &ATT_HDL_HRS_CHARACTERISTIC_HEART_RATE_MEASUREMENT[5],
    att_HDL_HRS_HEART_RATE_MEASUREMENT,
    att_HDL_HRS_CLIENT_CHARACTERISTIC_CONFIGURATION,
    &ATT_HDL_HRS_CHARACTERISTIC_BODY_SENSOR_LOCATION[5],
    att_HDL_HRS_BODY_SENSOR_LOCATION,
#endif
#ifdef _PROFILE_BAS_
    &ATT_HDL_BAS_PRIMARY_SERVICE[5],
    &ATT_HDL_BAS_CHARACTERISTIC_BATTERY_LEVEL[5],
    att_HDL_BAS_BATTERY_LEVEL,
    //&ATT_HDL_BAS_BATTERY_LEVEL_PRESENTATION_FORMAT[5],
    att_HDL_BAS_CLIENT_CHARACTERISTIC_CONFIGURATION,
    &ATT_HDL_BAS_CHARACTERISTIC_BATTERY_POWER_STATE[5],
    att_HDL_BAS_BATTERY_POWER_STATE,
#endif
#ifdef _PROFILE_HOGP_
    &ATT_HDL_HIDS_PRIMARY_SERVICE[5],
    &ATT_HDL_HIDS_CHARACTERISTIC_HID_INFORMATION[5],
    &ATT_HDL_HIDS_HID_INFORMATION[5],
    &ATT_HDL_HIDS_CHARACTERISTIC_HID_CONTROL_POINT[5],
    att_HDL_HIDS_HID_CONTROL_POINT,
    &ATT_HDL_HIDS_CHARACTERISTIC_REPORT_MAP[5],
    &ATT_HDL_HIDS_REPORT_MAP[5],
#ifdef _PROFILE_HOGP_KEYBOARD_
    &ATT_HDL_HIDS_CHARACTERISTIC_REPORT_KBI[5],
    att_HDL_HIDS_REPORT_KBI,
    att_HDL_HIDS_REPORT_KBI_CLIENT_CHARACTERISTIC_CONFIGURATION,
    &ATT_HDL_HIDS_REPORT_KBI_REPORT_REFERENCE[5],
    &ATT_HDL_HIDS_CHARACTERISTIC_REPORT_KBO[5],
    att_HDL_HIDS_REPORT_KBO,
    &ATT_HDL_HIDS_REPORT_KBO_REPORT_REFERENCE[5],
#endif
#ifdef _PROFILE_HOGP_MOUSE_
    &ATT_HDL_HIDS_CHARACTERISTIC_REPORT_MSI[5],
    att_HDL_HIDS_REPORT_MSI,
    att_HDL_HIDS_REPORT_MSI_CLIENT_CHARACTERISTIC_CONFIGURATION,
    &ATT_HDL_HIDS_REPORT_MSI_REPORT_REFERENCE[5],
    &ATT_HDL_HIDS_CHARACTERISTIC_REPORT_MSF[5],
    att_HDL_HIDS_REPORT_MSF,
    &ATT_HDL_HIDS_REPORT_MSF_REPORT_REFERENCE[5],
    &ATT_HDL_HIDS_REPORT_MSF_USER_DESCRIPTION[5],
#endif
#ifdef _PROFILE_HOGP_COMSUMER_
    &ATT_HDL_HIDS_CHARACTERISTIC_REPORT_CSI[5],
    att_HDL_HIDS_REPORT_CSI,
    att_HDL_HIDS_REPORT_CSI_CLIENT_CHARACTERISTIC_CONFIGURATION,
    &ATT_HDL_HIDS_REPORT_CSI_REPORT_REFERENCE[5],
#endif
#ifdef _PROFILE_HOGP_EMC_
    &ATT_HDL_HIDS_CHARACTERISTIC_REPORT_81[5],
    att_HDL_HIDS_REPORT_81,
    att_HDL_HIDS_REPORT_81_CLIENT_CHARACTERISTIC_CONFIGURATION,
    &ATT_HDL_HIDS_REPORT_81_REPORT_REFERENCE[5],
    &ATT_HDL_HIDS_CHARACTERISTIC_REPORT_82[5],
    att_HDL_HIDS_REPORT_82,
    att_HDL_HIDS_REPORT_82_CLIENT_CHARACTERISTIC_CONFIGURATION,
    &ATT_HDL_HIDS_REPORT_82_REPORT_REFERENCE[5],
    &ATT_HDL_HIDS_CHARACTERISTIC_REPORT_5D[5],
    att_HDL_HIDS_REPORT_5D,
    att_HDL_HIDS_REPORT_5D_CLIENT_CHARACTERISTIC_CONFIGURATION,
    &ATT_HDL_HIDS_REPORT_5D_REPORT_REFERENCE[5],
    &ATT_HDL_HIDS_CHARACTERISTIC_REPORT_05[5],
    att_HDL_HIDS_REPORT_05,
    //att_HDL_HIDS_REPORT_05_CLIENT_CHARACTERISTIC_CONFIGURATION,
    &ATT_HDL_HIDS_REPORT_05_REPORT_REFERENCE[5],
    &ATT_HDL_HIDS_CHARACTERISTIC_REPORT_08[5],
    att_HDL_HIDS_REPORT_08,
    //att_HDL_HIDS_REPORT_08_CLIENT_CHARACTERISTIC_CONFIGURATION,
    &ATT_HDL_HIDS_REPORT_08_REPORT_REFERENCE[5],
    &ATT_HDL_HIDS_CHARACTERISTIC_REPORT_09[5],
    att_HDL_HIDS_REPORT_09,
    //att_HDL_HIDS_REPORT_09_CLIENT_CHARACTERISTIC_CONFIGURATION,
    &ATT_HDL_HIDS_REPORT_09_REPORT_REFERENCE[5],
#endif
    &ATT_HDL_HIDS_CHARACTERISTIC_PROTOCOL_MODE[5],
    att_HDL_HIDS_PROTOCOL_MODE,
#ifdef _PROFILE_HOGP_BOOT_
#ifdef _PROFILE_HOGP_KEYBOARD_
    &ATT_HDL_HIDS_CHARACTERISTIC_BOOT_KEYBOARD_INPUT_REPORT[5],
    att_HDL_HIDS_BOOT_KEYBOARD_INPUT_REPORT,
    att_HDL_HIDS_BOOT_KEYBOARD_INPUT_REPORT_CLIENT_CHARACTERISTIC_CONFIGURATION,
    &ATT_HDL_HIDS_CHARACTERISTIC_BOOT_KEYBOARD_OUTPUT_REPORT[5],
    att_HDL_HIDS_BOOT_KEYBOARD_OUTPUT_REPORT,
#endif
#ifdef _PROFILE_HOGP_MOUSE_
    &ATT_HDL_HIDS_CHARACTERISTIC_BOOT_MOUSE_INPUT_REPORT[5],
    att_HDL_HIDS_BOOT_MOUSE_INPUT_REPORT,
    att_HDL_HIDS_BOOT_MOUSE_INPUT_REPORT_CLIENT_CHARACTERISTIC_CONFIGURATION,
#endif
#endif
#endif  //#ifdef _PROFILE_HOGP
#ifdef _PROFILE_SCPP_
    &ATT_HDL_SCPS_PRIMARY_SERVICE[5],
    &ATT_HDL_SCPS_CHARACTERISTIC_SCAN_INTERVAL_WINDOW[5],
    att_HDL_SCPS_SCAN_INTERVAL_WINDOW,
    &ATT_HDL_SCPS_CHARACTERISTIC_SCAN_REFRESH[5],
    att_HDL_SCPS_SCAN_REFRESH,
    att_HDL_SCPS_SCAN_REFRESH_CLIENT_CHARACTERISTIC_CONFIGURATION,
#endif
#ifdef _PROFILE_IAS_
    &ATT_HDL_IAS_PRIMARY_SERVICE[5],
    &ATT_HDL_IAS_CHARACTERISTIC_ALERT_LEVEL[5],
    att_HDL_IAS_ALERT_LEVEL,
#endif
#ifdef _PROFILE_LLS_
    &ATT_HDL_LLS_PRIMARY_SERVICE[5],
    &ATT_HDL_LLS_CHARACTERISTIC_ALERT_LEVEL[5],
    att_HDL_LLS_ALERT_LEVEL,
#endif
#ifdef _PROFILE_TPS_
    &ATT_HDL_TPS_PRIMARY_SERVICE[5],
    &ATT_HDL_TPS_CHARACTERISTIC_TX_POWER_LEVEL[5],
    att_HDL_TPS_TX_POWER_LEVEL,
#endif
#ifdef _PROFILE_BLP_
    &ATT_HDL_BLS_PRIMARY_SERVICE[5],
    &ATT_HDL_BLS_CHARACTERISTIC_BLOOD_PRESSURE_MEASUREMENT[5],
    att_HDL_BLS_BLOOD_PRESSURE_MEASUREMENT,
    att_HDL_BLS_BLOOD_PRESSURE_CLIENT_CHARACTERISTIC_CONFIGURATION,
    &ATT_HDL_BLS_CHARACTERISTIC_BLOOD_PRESSURE_FEATURE[5],
    &ATT_HDL_BLS_BLOOD_PRESSURE_FEATURE_INIT[5],
#endif
#ifdef _PROFILE_HTP_
    &ATT_HDL_HTS_PRIMARY_SERVICE[5],
    &ATT_HDL_HTS_CHARACTERISTIC_TEMPERATURE_MEASUREMENT[5],
    att_HDL_HTS_TEMPERATURE_MEASUREMENT,
    att_HDL_HTS_TEMPERATURE_MEASUREMENT_CLIENT_CHARACTERISTIC_CONFIGURATION,
    &ATT_HDL_HTS_CHARACTERISTIC_TEMPERATURE_TYPE[5],
    &ATT_HDL_HTS_TEMPERATURE_TYPE_INIT[5],
#endif
#ifdef _PROFILE_RSCP_
    &ATT_HDL_RSCS_PRIMARY_SERVICE[5],
    &ATT_HDL_RSCS_CHARACTERISTIC_RSC_MEASUREMENT[5],
    att_HDL_RSCS_RSC_MEASUREMENT,
    att_HDL_RSCS_RSC_MEASUREMENT_CLIENT_CHARACTERISTIC_CONFIGURATION,
    &ATT_HDL_RSCS_CHARACTERISTIC_RSC_FEATURE[5],
    &ATT_HDL_RSCS_RSC_FEATURE_INIT[5],
    &ATT_HDL_RSCS_CHARACTERISTIC_SC_CONTROL_POINT[5],
    att_HDL_RSCS_SC_CONTROL_POINT,
    att_HDL_RSCS_SC_CONTROL_POINT_CLIENT_CHARACTERISTIC_CONFIGURATION,
#endif
#ifdef _PROFILE_CSCP_
    &ATT_HDL_CSCS_PRIMARY_SERVICE[5],
    &ATT_HDL_CSCS_CHARACTERISTIC_CSC_MEASUREMENT[5],
    att_HDL_CSCS_CSC_MEASUREMENT,
    att_HDL_CSCS_CSC_MEASUREMENT_CLIENT_CHARACTERISTIC_CONFIGURATION,
    &ATT_HDL_CSCS_CHARACTERISTIC_CSC_FEATURE[5],
    &ATT_HDL_CSCS_CSC_FEATURE_INIT[5],
    &ATT_HDL_CSCS_CHARACTERISTIC_SC_CONTROL_POINT[5],
    att_HDL_CSCS_SC_CONTROL_POINT,
    att_HDL_CSCS_SC_CONTROL_POINT_CLIENT_CHARACTERISTIC_CONFIGURATION,
#endif
#ifdef _PROFILE_LNS_
    &ATT_HDL_LNS_PRIMARY_SERVICE[5],
    &ATT_HDL_LNS_CHARACTERISTIC_LOCATION_AND_SPEED[5],
    att_HDL_LNS_LOCATION_AND_SPEED,
    att_HDL_LNS_LOCATION_AND_SPEED_CLIENT_CHARACTERISTIC_CONFIGURATION,
    &ATT_HDL_LNS_CHARACTERISTIC_LN_FEATURE[5],
    &ATT_HDL_LNS_LN_FEATURE_INIT[5],
    &ATT_HDL_LNS_CHARACTERISTIC_POSITION_QUALITY[5],
    att_HDL_LNS_POSITION_QUALITY,
    &ATT_HDL_LNS_CHARACTERISTIC_LN_CONTROL_POINT[5],
    att_HDL_LNS_LN_CONTROL_POINT,
    att_HDL_LNS_LN_CONTROL_POINT_CLIENT_CHARACTERISTIC_CONFIGURATION,
    &ATT_HDL_LNS_CHARACTERISTIC_NAVIGATION[5],
    att_HDL_LNS_NAVIGATION,
    att_HDL_LNS_NAVIGATION_CLIENT_CHARACTERISTIC_CONFIGURATION,
#endif
#ifdef _PROFILE_GLS_
    &ATT_HDL_GLS_PRIMARY_SERVICE[5],
    &ATT_HDL_GLS_CHARACTERISTIC_GLUCOSE_MEASUREMENT[5],
    att_HDL_GLS_GLUCOSE_MEASUREMENT,
    att_HDL_GLS_GLUCOSE_MEASUREMENT_CLIENT_CHARACTERISTIC_CONFIGURATION,
    &ATT_HDL_GLS_CHARACTERISTIC_GLUCOSE_MEASUREMENT_CONTEXT[5],
    att_HDL_GLS_GLUCOSE_MEASUREMENT_CONTEXT,
    att_HDL_GLS_GLUCOSE_MEASUREMENT_CONTEXT_CLIENT_CHARACTERISTIC_CONFIGURATION,
    &ATT_HDL_GLS_CHARACTERISTIC_GLUCOSE_FEATURE[5],
    &ATT_HDL_GLS_GLUCOSE_FEATURE_INIT[5],
    &ATT_HDL_GLS_CHARACTERISTIC_RECORD_ACCESS_CONTROL_POINT[5],
    att_HDL_GLS_RECORD_ACCESS_CONTROL_POINT,
    att_HDL_GLS_RECORD_ACCESS_CONTROL_POINT_CLIENT_CHARACTERISTIC_CONFIGURATION,
#endif
#ifdef _PROFILE_CPS_
    &ATT_HDL_CPS_PRIMARY_SERVICE[5],
    &ATT_HDL_CPS_CHARACTERISTIC_CYCLING_POWER_MEASUREMENT[5],
    att_HDL_CPS_CYCLING_POWER_MEASUREMENT,
    att_HDL_CPS_CYCLING_POWER_MEASUREMENT_CLIENT_CHARACTERISTIC_CONFIGURATION,
    att_HDL_CPS_CYCLING_POWER_MEASUREMENT_SERVER_CHARACTERISTIC_CONFIGURATION,
    &ATT_HDL_CPS_CHARACTERISTIC_CYCLING_POWER_FEATURE[5],
    &ATT_HDL_CPS_CYCLING_POWER_FEATURE_INIT[5],
    &ATT_HDL_CPS_CHARACTERISTIC_SENSOR_LOCATION[5],
    att_HDL_CPS_SENSOR_LOCATION,
    &ATT_HDL_CPS_CHARACTERISTIC_CYCLING_POWER_VECTOR[5],
    att_HDL_CPS_CYCLING_POWER_VECTOR,
    att_HDL_CPS_CYCLING_POWER_VECTOR_CLIENT_CHARACTERISTIC_CONFIGURATION,
    &ATT_HDL_CPS_CHARACTERISTIC_CYCLING_POWER_CONTROL_POINT[5],
    att_HDL_CPS_CYCLING_POWER_CONTROL_POINT,
    att_HDL_CPS_CYCLING_POWER_CONTROL_POINT_CLIENT_CHARACTERISTIC_CONFIGURATION,
#endif
#ifdef _PROFILE_CSTM_UDF01S_
    &ATT_HDL_UDF01S_PRIMARY_SERVICE[5],
    &ATT_HDL_UDF01S_CHARACTERISTIC_UDATR01[5],
    &ATT_HDL_UDF01S_UDATR01[5 + 14],
    &ATT_HDL_UDF01S_CHARACTERISTIC_UDATN01[5],
    att_HDL_UDF01S_UDATN01,
    att_HDL_UDF01S_UDATN01_MEASUREMENT_CLIENT_CHARACTERISTIC_CONFIGURATION,
    &ATT_HDL_UDF01S_CHARACTERISTIC_UDATRW01[5],
    att_HDL_UDF01S_UDATRW01,
#endif
    &ATT_HDL_UDF_PRIMARY_SERVICE[5],
    &ATT_HDL_UDF_CHARACTERISTIC_UDATR01[5],
    &ATT_HDL_UDF_UDATR01[5 + 14],
    &ATT_HDL_UDF_CHARACTERISTIC_UDATN01[5],
    att_HDL_UDF_UDATN01,
    att_HDL_UDF_UDATN01_MEASUREMENT_CLIENT_CHARACTERISTIC_CONFIGURATION,
    &ATT_HDL_UDF_CHARACTERISTIC_UDATRW01[5],
    att_HDL_UDF_UDATRW01,

#ifdef _PROFILE_CSTM_UDF02S_
    &ATT_HDL_UDF02S_PRIMARY_SERVICE[5],
    &ATT_HDL_UDF02S_CHARACTERISTIC_UDATR02[5],
    &ATT_HDL_UDF02S_UDATR02[5],
    &ATT_HDL_UDF02S_CHARACTERISTIC_UDATN02[5],
    att_HDL_UDF02S_UDATN02,
    att_HDL_UDF02S_UDATN02_MEASUREMENT_CLIENT_CHARACTERISTIC_CONFIGURATION,
    &ATT_HDL_UDF02S_CHARACTERISTIC_UDATRW02[5],
    att_HDL_UDF02S_UDATRW02,
#endif
#ifdef _PROFILE_CSTM_UDF03S_
    &ATT_HDL_UDF03S_PRIMARY_SERVICE[5],
    &ATT_HDL_UDF03S_CHARACTERISTIC_UDATR03[5],
    &ATT_HDL_UDF03S_UDATR03[5],
    &ATT_HDL_UDF03S_CHARACTERISTIC_UDATN03[5],
    att_HDL_UDF03S_UDATN03,
    att_HDL_UDF03S_UDATN03_MEASUREMENT_CLIENT_CHARACTERISTIC_CONFIGURATION,
    &ATT_HDL_UDF03S_CHARACTERISTIC_UDATRW03[5],
    att_HDL_UDF03S_UDATRW03,
#endif

#ifdef _PROFILE_CSTM_DATAEXCHANGE_S_
    &ATT_HDL_DATAEXCHANGES_PRIMARY_SERVICE[5],
    &ATT_HDL_DATAEXCHANGES_CHARACTERISTIC_WRITE[5],
    att_HDL_DATAEXCHANGES_WRITE,
    &ATT_HDL_DATAEXCHANGES_CHARACTERISTICR_READ[5],
    att_HDL_DATAEXCHANGES_READINIT,
    att_HDL_DATAEXCHANGES_READINIT_MEASUREMENT_CLIENT_CHARACTERISTIC_CONFIGURATION,
#endif

#ifdef _PROFILE_OTA_
    &ATT_HDL_OTA_PRIMARY_SERVICE[5],
    &ATT_HDL_OTA_CHARACTERISTIC_OTA_DATA[5],
    att_HDL_OTA_OTA_DATA,
    att_HDL_OTA_OTA_DATA_CLIENT_CHARACTERISTIC_CONFIGURATION,
    &ATT_HDL_OTA_OTA_DATA_DESCRIPTION[5],
    &ATT_HDL_OTA_CHARACTERISTIC_OTA_CMD_CONTROL_POINT[5],
    att_HDL_OTA_OTA_CMD_CONTROL_POINT,
    att_HDL_OTA_OTA_CMD_CONTROL_POINT_CLIENT_CHARACTERISTIC_CONFIGURATION,
    &ATT_HDL_OTA_OTA_CMD_USER_DESCRIPTION[5],
#endif

};



/*!
    \brief  Table combines the description of handles corresponding to each GATT declarations and characteristics.
*/
//code Uint8 code *ATTRIBUTE_SERVER_PARAM[] =
const Uint8 *ATTRIBUTE_SERVER[SIZE_ATTRIBUTE_SERVER] =
{
    ATT_HDL_INVALID,
    ATT_HDL_GAP_PRIMARY_SERVICE,
    ATT_HDL_GAP_CHARACTERISTIC_DEVICE_NAME,
    ATT_HDL_GAP_DEVICE_NAME_INIT,
    ATT_HDL_GAP_CHARACTERISTIC_APPEARANCE,
    ATT_HDL_GAP_APPEARANCE,
    ATT_HDL_GAP_CHARACTERISTIC_RECONNECTION_ADDRESS,
    ATT_HDL_GAP_RECONNECTION_ADDRESS_INIT,
    ATT_HDL_GAP_CHARACTERISTIC_PERIPHERAL_PRIVACY_FLAG,
    ATT_HDL_GAP_PERIPHERAL_PRIVACY_FLAG,
    ATT_HDL_GAP_CHARACTERISTIC_PERIPHERAL_PREFERRED_CONNECTION_PARAMETERS,
    ATT_HDL_GAP_PERIPHERAL_PREFERRED_CONNECTION_PARAMETERS,
    ATT_HDL_GATT_PRIMARY_SERVICE,
    ATT_HDL_GATT_CHARACTERISTIC_SERVICE_CHANGED,
    ATT_HDL_GATT_SERVICE_CHANGED_INIT,
    ATT_HDL_GATT_CLIENT_CHARACTERISTIC_CONFIGURATION_INIT,
    ATT_HDL_DIS_PRIMARY_SERVICE,
    ATT_HDL_DIS_CHARACTERISTIC_SERIAL_NUMBER_STRING,
    ATT_HDL_DIS_SERIAL_NUMBER_STRING,
    ATT_HDL_DIS_SERIAL_NUMBER_STRING_PRESENTATION_FORMAT,
    ATT_HDL_DIS_CHARACTERISTIC_MANUFACTURER_NAME_STRING,
    ATT_HDL_DIS_MANUFACTURER_NAME_STRING,
    ATT_HDL_DIS_MANUFACTURER_NAME_STRING_PRESENTATION_FORMAT,
    ATT_HDL_DIS_CHARACTERISTIC_SYSTEM_ID,
    ATT_HDL_DIS_SYSTEM_ID,
    ATT_HDL_DIS_CHARACTERISTIC_FIRMWARE_REVISION_STRING,
    ATT_HDL_DIS_FIRMWARE_REVISION_STRING,
    ATT_HDL_DIS_FIRMWARE_REVISION_STRING_PRESENTATION_FORMAT,
    ATT_HDL_DIS_CHARACTERISTIC_MODEL_NUMBER_STRING,
    ATT_HDL_DIS_MODEL_NUMBER_STRING,
    ATT_HDL_DIS_MODEL_NUMBER_STRING_PRESENTATION_FORMAT,
    ATT_HDL_DIS_CHARACTERISTIC_HARDWARE_REVISION_STRING,
    ATT_HDL_DIS_HARDWARE_REVISION_STRING,
    ATT_HDL_DIS_HARDWARE_REVISION_STRING_PRESENTATION_FORMAT,
    ATT_HDL_DIS_CHARACTERISTIC_SOFTWARE_REVISION_STRING,
    ATT_HDL_DIS_SOFTWARE_REVISION_STRING,
    ATT_HDL_DIS_SOFTWARE_REVISION_STRING_PRESENTATION_FORMAT,
    ATT_HDL_DIS_CHARACTERISTIC_PNP_ID,
    ATT_HDL_DIS_PNP_ID,
#ifdef _PROFILE_HRP_
    ATT_HDL_HRS_PRIMARY_SERVICE,
    ATT_HDL_HRS_CHARACTERISTIC_HEART_RATE_MEASUREMENT,
    ATT_HDL_HRS_HEART_RATE_MEASUREMENT_INIT,
    ATT_HDL_HRS_CLIENT_CHARACTERISTIC_CONFIGURATION_INIT,
    ATT_HDL_HRS_CHARACTERISTIC_BODY_SENSOR_LOCATION,
    ATT_HDL_HRS_BODY_SENSOR_LOCATION_INIT,
#endif
#ifdef _PROFILE_BAS_
    ATT_HDL_BAS_PRIMARY_SERVICE,
    ATT_HDL_BAS_CHARACTERISTIC_BATTERY_LEVEL,
    ATT_HDL_BAS_BATTERY_LEVEL_INIT,
    //ATT_HDL_BAS_BATTERY_LEVEL_PRESENTATION_FORMAT,
    ATT_HDL_BAS_CLIENT_CHARACTERISTIC_CONFIGURATION_INIT,
    ATT_HDL_BAS_CHARACTERISTIC_BATTERY_POWER_STATE,
    ATT_HDL_BAS_BATTERY_POWER_STATE_INIT,
#endif
#ifdef _PROFILE_HOGP_
    ATT_HDL_HIDS_PRIMARY_SERVICE,
    ATT_HDL_HIDS_CHARACTERISTIC_HID_INFORMATION,
    ATT_HDL_HIDS_HID_INFORMATION,
    ATT_HDL_HIDS_CHARACTERISTIC_HID_CONTROL_POINT,
    ATT_HDL_HIDS_HID_CONTROL_POINT_INIT,
    ATT_HDL_HIDS_CHARACTERISTIC_REPORT_MAP,
    ATT_HDL_HIDS_REPORT_MAP,
#ifdef _PROFILE_HOGP_KEYBOARD_
    ATT_HDL_HIDS_CHARACTERISTIC_REPORT_KBI,
    ATT_HDL_HIDS_REPORT_KBI_INIT,
    ATT_HDL_HIDS_REPORT_KBI_CLIENT_CHARACTERISTIC_CONFIGURATION_INIT,
    ATT_HDL_HIDS_REPORT_KBI_REPORT_REFERENCE,
    ATT_HDL_HIDS_CHARACTERISTIC_REPORT_KBO,
    ATT_HDL_HIDS_REPORT_KBO_INIT,
    ATT_HDL_HIDS_REPORT_KBO_REPORT_REFERENCE,
#endif
#ifdef _PROFILE_HOGP_MOUSE_
    ATT_HDL_HIDS_CHARACTERISTIC_REPORT_MSI,
    ATT_HDL_HIDS_REPORT_MSI_INIT,
    ATT_HDL_HIDS_REPORT_MSI_CLIENT_CHARACTERISTIC_CONFIGURATION_INIT,
    ATT_HDL_HIDS_REPORT_MSI_REPORT_REFERENCE,
    ATT_HDL_HIDS_CHARACTERISTIC_REPORT_MSF,
    ATT_HDL_HIDS_REPORT_MSF_INIT,
    ATT_HDL_HIDS_REPORT_MSF_REPORT_REFERENCE,
    ATT_HDL_HIDS_REPORT_MSF_USER_DESCRIPTION,
#endif
#ifdef _PROFILE_HOGP_COMSUMER_
    ATT_HDL_HIDS_CHARACTERISTIC_REPORT_CSI,
    ATT_HDL_HIDS_REPORT_CSI_INIT,
    ATT_HDL_HIDS_REPORT_CSI_CLIENT_CHARACTERISTIC_CONFIGURATION_INIT,
    ATT_HDL_HIDS_REPORT_CSI_REPORT_REFERENCE,
#endif
#ifdef _PROFILE_HOGP_EMC_
    ATT_HDL_HIDS_CHARACTERISTIC_REPORT_81,
    ATT_HDL_HIDS_REPORT_81_INIT,
    ATT_HDL_HIDS_REPORT_81_CLIENT_CHARACTERISTIC_CONFIGURATION_INIT,
    ATT_HDL_HIDS_REPORT_81_REPORT_REFERENCE,
    ATT_HDL_HIDS_CHARACTERISTIC_REPORT_82,
    ATT_HDL_HIDS_REPORT_82_INIT,
    ATT_HDL_HIDS_REPORT_82_CLIENT_CHARACTERISTIC_CONFIGURATION_INIT,
    ATT_HDL_HIDS_REPORT_82_REPORT_REFERENCE,
    ATT_HDL_HIDS_CHARACTERISTIC_REPORT_5D,
    ATT_HDL_HIDS_REPORT_5D_INIT,
    ATT_HDL_HIDS_REPORT_5D_CLIENT_CHARACTERISTIC_CONFIGURATION_INIT,
    ATT_HDL_HIDS_REPORT_5D_REPORT_REFERENCE,
    ATT_HDL_HIDS_CHARACTERISTIC_REPORT_05,
    ATT_HDL_HIDS_REPORT_05_INIT,
    //ATT_HDL_HIDS_REPORT_05_CLIENT_CHARACTERISTIC_CONFIGURATION_INIT,
    ATT_HDL_HIDS_REPORT_05_REPORT_REFERENCE,
    ATT_HDL_HIDS_CHARACTERISTIC_REPORT_08,
    ATT_HDL_HIDS_REPORT_08_INIT,
    //ATT_HDL_HIDS_REPORT_08_CLIENT_CHARACTERISTIC_CONFIGURATION_INIT,
    ATT_HDL_HIDS_REPORT_08_REPORT_REFERENCE,
    ATT_HDL_HIDS_CHARACTERISTIC_REPORT_09,
    ATT_HDL_HIDS_REPORT_09_INIT,
    //ATT_HDL_HIDS_REPORT_09_CLIENT_CHARACTERISTIC_CONFIGURATION_INIT,
    ATT_HDL_HIDS_REPORT_09_REPORT_REFERENCE,
#endif
    ATT_HDL_HIDS_CHARACTERISTIC_PROTOCOL_MODE,
    ATT_HDL_HIDS_PROTOCOL_MODE_INIT,
#ifdef _PROFILE_HOGP_BOOT_
#ifdef _PROFILE_HOGP_KEYBOARD_
    ATT_HDL_HIDS_CHARACTERISTIC_BOOT_KEYBOARD_INPUT_REPORT,
    ATT_HDL_HIDS_BOOT_KEYBOARD_INPUT_REPORT_INIT,
    ATT_HDL_HIDS_BOOT_KEYBOARD_INPUT_REPORT_CLIENT_CHARACTERISTIC_CONFIGURATION_INIT,
    ATT_HDL_HIDS_CHARACTERISTIC_BOOT_KEYBOARD_OUTPUT_REPORT,
    ATT_HDL_HIDS_BOOT_KEYBOARD_OUTPUT_REPORT_INIT,
#endif
#ifdef _PROFILE_HOGP_MOUSE_
    ATT_HDL_HIDS_CHARACTERISTIC_BOOT_MOUSE_INPUT_REPORT,
    ATT_HDL_HIDS_BOOT_MOUSE_INPUT_REPORT_INIT,
    ATT_HDL_HIDS_BOOT_MOUSE_INPUT_REPORT_CLIENT_CHARACTERISTIC_CONFIGURATION_INIT,
#endif
#endif
#endif
#ifdef _PROFILE_SCPP_
    ATT_HDL_SCPS_PRIMARY_SERVICE,
    ATT_HDL_SCPS_CHARACTERISTIC_SCAN_INTERVAL_WINDOW,
    ATT_HDL_SCPS_SCAN_INTERVAL_WINDOW_INIT,
    ATT_HDL_SCPS_CHARACTERISTIC_SCAN_REFRESH,
    ATT_HDL_SCPS_SCAN_REFRESH_INIT,
    ATT_HDL_SCPS_SCAN_REFRESH_CLIENT_CHARACTERISTIC_CONFIGURATION_INIT,
#endif
#ifdef _PROFILE_IAS_
    ATT_HDL_IAS_PRIMARY_SERVICE,
    ATT_HDL_IAS_CHARACTERISTIC_ALERT_LEVEL,
    ATT_HDL_IAS_ALERT_LEVEL_INIT,
#endif
#ifdef _PROFILE_LLS_
    ATT_HDL_LLS_PRIMARY_SERVICE,
    ATT_HDL_LLS_CHARACTERISTIC_ALERT_LEVEL,
    ATT_HDL_LLS_ALERT_LEVEL_INIT,
#endif
#ifdef _PROFILE_TPS_
    ATT_HDL_TPS_PRIMARY_SERVICE,
    ATT_HDL_TPS_CHARACTERISTIC_TX_POWER_LEVEL,
    ATT_HDL_TPS_TX_POWER_LEVEL_INIT,
#endif
#ifdef _PROFILE_BLP_
    ATT_HDL_BLS_PRIMARY_SERVICE,
    ATT_HDL_BLS_CHARACTERISTIC_BLOOD_PRESSURE_MEASUREMENT,
    ATT_HDL_BLS_BLOOD_PRESSURE_MEASUREMENT_INIT,
    ATT_HDL_BLS_BLOOD_PRESSURE_CLIENT_CHARACTERISTIC_CONFIGURATION_INIT,
    ATT_HDL_BLS_CHARACTERISTIC_BLOOD_PRESSURE_FEATURE,
    ATT_HDL_BLS_BLOOD_PRESSURE_FEATURE_INIT,
#endif
#ifdef _PROFILE_HTP_
    ATT_HDL_HTS_PRIMARY_SERVICE,
    ATT_HDL_HTS_CHARACTERISTIC_TEMPERATURE_MEASUREMENT,
    ATT_HDL_HTS_TEMPERATURE_MEASUREMENT_INIT,
    ATT_HDL_HTS_TEMPERATURE_MEASUREMENT_CLIENT_CHARACTERISTIC_CONFIGURATION_INIT,
    ATT_HDL_HTS_CHARACTERISTIC_TEMPERATURE_TYPE,
    ATT_HDL_HTS_TEMPERATURE_TYPE_INIT,
#endif
#ifdef _PROFILE_RSCP_
    ATT_HDL_RSCS_PRIMARY_SERVICE,
    ATT_HDL_RSCS_CHARACTERISTIC_RSC_MEASUREMENT,
    ATT_HDL_RSCS_RSC_MEASUREMENT_INIT,
    ATT_HDL_RSCS_RSC_MEASUREMENT_CLIENT_CHARACTERISTIC_CONFIGURATION_INIT,
    ATT_HDL_RSCS_CHARACTERISTIC_RSC_FEATURE,
    ATT_HDL_RSCS_RSC_FEATURE_INIT,
    ATT_HDL_RSCS_CHARACTERISTIC_SC_CONTROL_POINT,
    ATT_HDL_RSCS_SC_CONTROL_POINT_INIT,
    ATT_HDL_RSCS_SC_CONTROL_POINT_CLIENT_CHARACTERISTIC_CONFIGURATION_INIT,
#endif
#ifdef _PROFILE_CSCP_
    ATT_HDL_CSCS_PRIMARY_SERVICE,
    ATT_HDL_CSCS_CHARACTERISTIC_CSC_MEASUREMENT,
    ATT_HDL_CSCS_CSC_MEASUREMENT_INIT,
    ATT_HDL_CSCS_CSC_MEASUREMENT_CLIENT_CHARACTERISTIC_CONFIGURATION_INIT,
    ATT_HDL_CSCS_CHARACTERISTIC_CSC_FEATURE,
    ATT_HDL_CSCS_CSC_FEATURE_INIT,
    ATT_HDL_CSCS_CHARACTERISTIC_SC_CONTROL_POINT,
    ATT_HDL_CSCS_SC_CONTROL_POINT_INIT,
    ATT_HDL_CSCS_SC_CONTROL_POINT_CLIENT_CHARACTERISTIC_CONFIGURATION_INIT,
#endif
#ifdef _PROFILE_LNS_
    ATT_HDL_LNS_PRIMARY_SERVICE,
    ATT_HDL_LNS_CHARACTERISTIC_LOCATION_AND_SPEED,
    ATT_HDL_LNS_LOCATION_AND_SPEED_INIT,
    ATT_HDL_LNS_LOCATION_AND_SPEED_CLIENT_CHARACTERISTIC_CONFIGURATION_INIT,
    ATT_HDL_LNS_CHARACTERISTIC_LN_FEATURE,
    ATT_HDL_LNS_LN_FEATURE_INIT,
    ATT_HDL_LNS_CHARACTERISTIC_POSITION_QUALITY,
    ATT_HDL_LNS_POSITION_QUALITY_INIT,
    ATT_HDL_LNS_CHARACTERISTIC_LN_CONTROL_POINT,
    ATT_HDL_LNS_LN_CONTROL_POINT_INIT,
    ATT_HDL_LNS_LN_CONTROL_POINT_CLIENT_CHARACTERISTIC_CONFIGURATION_INIT,
    ATT_HDL_LNS_CHARACTERISTIC_NAVIGATION,
    ATT_HDL_LNS_NAVIGATION_INIT,
    ATT_HDL_LNS_NAVIGATION_CLIENT_CHARACTERISTIC_CONFIGURATION_INIT,
#endif
#ifdef _PROFILE_GLS_
    ATT_HDL_GLS_PRIMARY_SERVICE,
    ATT_HDL_GLS_CHARACTERISTIC_GLUCOSE_MEASUREMENT,
    ATT_HDL_GLS_GLUCOSE_MEASUREMENT_INIT,
    ATT_HDL_GLS_GLUCOSE_MEASUREMENT_CLIENT_CHARACTERISTIC_CONFIGURATION_INIT,
    ATT_HDL_GLS_CHARACTERISTIC_GLUCOSE_MEASUREMENT_CONTEXT,
    ATT_HDL_GLS_GLUCOSE_MEASUREMENT_CONTEXT_INIT,
    ATT_HDL_GLS_GLUCOSE_MEASUREMENT_CONTEXT_CLIENT_CHARACTERISTIC_CONFIGURATION_INIT,
    ATT_HDL_GLS_CHARACTERISTIC_GLUCOSE_FEATURE,
    ATT_HDL_GLS_GLUCOSE_FEATURE_INIT,
    ATT_HDL_GLS_CHARACTERISTIC_RECORD_ACCESS_CONTROL_POINT,
    ATT_HDL_GLS_RECORD_ACCESS_CONTROL_POINT_INIT,
    ATT_HDL_GLS_RECORD_ACCESS_CONTROL_POINT_CLIENT_CHARACTERISTIC_CONFIGURATION_INIT,
#endif
#ifdef _PROFILE_CPS_
    ATT_HDL_CPS_PRIMARY_SERVICE,
    ATT_HDL_CPS_CHARACTERISTIC_CYCLING_POWER_MEASUREMENT,
    ATT_HDL_CPS_CYCLING_POWER_MEASUREMENT_INIT,
    ATT_HDL_CPS_CYCLING_POWER_MEASUREMENT_CLIENT_CHARACTERISTIC_CONFIGURATION_INIT,
    ATT_HDL_CPS_CYCLING_POWER_MEASUREMENT_SERVER_CHARACTERISTIC_CONFIGURATION_INIT,
    ATT_HDL_CPS_CHARACTERISTIC_CYCLING_POWER_FEATURE,
    ATT_HDL_CPS_CYCLING_POWER_FEATURE_INIT,
    ATT_HDL_CPS_CHARACTERISTIC_SENSOR_LOCATION,
    ATT_HDL_CPS_SENSOR_LOCATION_INIT,
    ATT_HDL_CPS_CHARACTERISTIC_CYCLING_POWER_VECTOR,
    ATT_HDL_CPS_CYCLING_POWER_VECTOR_INIT,
    ATT_HDL_CPS_CYCLING_POWER_VECTOR_CLIENT_CHARACTERISTIC_CONFIGURATION_INIT,
    ATT_HDL_CPS_CHARACTERISTIC_CYCLING_POWER_CONTROL_POINT,
    ATT_HDL_CPS_CYCLING_POWER_CONTROL_POINT_INIT,
    ATT_HDL_CPS_CYCLING_POWER_CONTROL_POINT_CLIENT_CHARACTERISTIC_CONFIGURATION_INIT,
#endif
#ifdef _PROFILE_CSTM_UDF01S_
    ATT_HDL_UDF01S_PRIMARY_SERVICE,
    ATT_HDL_UDF01S_CHARACTERISTIC_UDATR01,
    ATT_HDL_UDF01S_UDATR01,
    ATT_HDL_UDF01S_CHARACTERISTIC_UDATN01,
    ATT_HDL_UDF01S_UDATN01_INIT,
    ATT_HDL_UDF01S_UDATN01_CLIENT_CHARACTERISTIC_CONFIGURATION_INIT,
    ATT_HDL_UDF01S_CHARACTERISTIC_UDATRW01,
    ATT_HDL_UDF01S_UDATRW01_INIT,
#endif
    ATT_HDL_UDF_PRIMARY_SERVICE,
    ATT_HDL_UDF_CHARACTERISTIC_UDATR01,
    ATT_HDL_UDF_UDATR01,
    ATT_HDL_UDF_CHARACTERISTIC_UDATN01,
    ATT_HDL_UDF_UDATN01_INIT,
    ATT_HDL_UDF_UDATN01_CLIENT_CHARACTERISTIC_CONFIGURATION_INIT,
    ATT_HDL_UDF_CHARACTERISTIC_UDATRW01,
    ATT_HDL_UDF_UDATRW01_INIT,
#ifdef _PROFILE_CSTM_UDF02S_
    ATT_HDL_UDF02S_PRIMARY_SERVICE,
    ATT_HDL_UDF02S_CHARACTERISTIC_UDATR02,
    ATT_HDL_UDF02S_UDATR02,
    ATT_HDL_UDF02S_CHARACTERISTIC_UDATN02,
    ATT_HDL_UDF02S_UDATN02_INIT,
    ATT_HDL_UDF02S_UDATN02_CLIENT_CHARACTERISTIC_CONFIGURATION_INIT,
    ATT_HDL_UDF02S_CHARACTERISTIC_UDATRW02,
    ATT_HDL_UDF02S_UDATRW02_INIT,
#endif
#ifdef _PROFILE_CSTM_UDF03S_
    ATT_HDL_UDF03S_PRIMARY_SERVICE,
    ATT_HDL_UDF03S_CHARACTERISTIC_UDATR03,
    ATT_HDL_UDF03S_UDATR03,
    ATT_HDL_UDF03S_CHARACTERISTIC_UDATN03,
    ATT_HDL_UDF03S_UDATN03_INIT,
    ATT_HDL_UDF03S_UDATN03_CLIENT_CHARACTERISTIC_CONFIGURATION_INIT,
    ATT_HDL_UDF03S_CHARACTERISTIC_UDATRW03,
    ATT_HDL_UDF03S_UDATRW03_INIT,
#endif

#ifdef _PROFILE_CSTM_DATAEXCHANGE_S_
    ATT_HDL_DATAEXCHANGES_PRIMARY_SERVICE,
    ATT_HDL_DATAEXCHANGES_CHARACTERISTIC_WRITE,
    ATT_HDL_DATAEXCHANGES_WRITE,
    ATT_HDL_DATAEXCHANGES_CHARACTERISTICR_READ,
    ATT_HDL_DATAEXCHANGES_READINIT,
    ATT_HDL_DATAEXCHANGES_READINIT_CLIENT_CHARACTERISTIC_CONFIGURATION_INIT,
#endif

#ifdef _PROFILE_OTA_
    ATT_HDL_OTA_PRIMARY_SERVICE,
    ATT_HDL_OTA_CHARACTERISTIC_OTA_DATA,
    ATT_HDL_OTA_OTA_DATA_INIT,
    ATT_HDL_OTA_OTA_DATA_CLIENT_CHARACTERISTIC_CONFIGURATION_INIT,
    ATT_HDL_OTA_OTA_DATA_DESCRIPTION,
    ATT_HDL_OTA_CHARACTERISTIC_OTA_CMD_CONTROL_POINT,
    ATT_HDL_OTA_OTA_CMD_CONTROL_POINT_INIT,
    ATT_HDL_OTA_OTA_CMD_CONTROL_POINT_CLIENT_CHARACTERISTIC_CONFIGURATION_INIT,
    ATT_HDL_OTA_OTA_CMD_USER_DESCRIPTION,
#endif


};




/*!
    \brief  Table combines the permission setting corresponding to each GATT declarations and characteristics.
*/
const Uint8 *ATTRIBUTE_SERVER_PERMISSION[SIZE_ATTRIBUTE_SERVER] =
{
    ATT_PERMISSION_NULL,                                                //ATT_HDL_INVALID,
    ATT_PERMISSION_PRIMARY_SERVICE,                                     //ATT_HDL_GAP_PRIMARY_SERVICE,
    ATT_PERMISSION_CHARACTERISTIC,                                      //ATT_HDL_GAP_CHARACTERISTIC_DEVICE_NAME,
    ATT_PERMISSION_HDL_GAP_DEVICE_NAME_INIT,                            //ATT_HDL_GAP_DEVICE_NAME,
    ATT_PERMISSION_CHARACTERISTIC,                                      //ATT_HDL_GAP_CHARACTERISTIC_APPEARANCE,
    ATT_PERMISSION_HDL_GAP_APPEARANCE,                                  //ATT_HDL_GAP_APPEARANCE,
    ATT_PERMISSION_CHARACTERISTIC,                                      //ATT_HDL_GAP_CHARACTERISTIC_RECONNECTION_ADDRESS,
    ATT_PERMISSION_HDL_GAP_RECONNECTION_ADDRESS_INIT,                   //ATT_HDL_GAP_RECONNECTION_ADDRESS_INIT,
    ATT_PERMISSION_CHARACTERISTIC,                                      //ATT_HDL_GAP_CHARACTERISTIC_PERIPHERAL_PRIVACY_FLAG,
    ATT_PERMISSION_HDL_GAP_PERIPHERAL_PRIVACY_FLAG,                     //ATT_HDL_GAP_PERIPHERAL_PRIVACY_FLAG,
    ATT_PERMISSION_CHARACTERISTIC,                                      //ATT_HDL_GAP_CHARACTERISTIC_PERIPHERAL_PREFERRED_CONNECTION_PARAMETERS,
    ATT_PERMISSION_HDL_GAP_PERIPHERAL_PREFERRED_CONNECTION_PARAMETERS,  //ATT_HDL_GAP_PERIPHERAL_PREFERRED_CONNECTION_PARAMETERS,
    ATT_PERMISSION_PRIMARY_SERVICE,                                     //ATT_HDL_GATT_PRIMARY_SERVICE,
    ATT_PERMISSION_CHARACTERISTIC,                                      //ATT_HDL_GATT_CHARACTERISTIC_SERVICE_CHANGED,
    ATT_PERMISSION_HDL_GATT_SERVICE_CHANGED_INIT,                       //ATT_HDL_GATT_SERVICE_CHANGED_INIT,
    ATT_PERMISSION_HDL_GATT_CLIENT_CHARACTERISTIC_CONFIGURATION_INIT,   //ATT_HDL_GATT_CLIENT_CHARACTERISTIC_CONFIGURATION_INIT
    ATT_PERMISSION_PRIMARY_SERVICE,                                     //ATT_HDL_DIS_PRIMARY_SERVICE,
    ATT_PERMISSION_CHARACTERISTIC,                                      //ATT_HDL_DIS_CHARACTERISTIC_SERIAL_NUMBER_STRING,
    ATT_PERMISSION_HDL_DIS_SERIAL_NUMBER_STRING,                        //ATT_HDL_DIS_SERIAL_NUMBER_STRING,
    ATT_PERMISSION_CHARACTERISTIC_PRESENTATION_FORMAT,                  //ATT_HDL_DIS_SERIAL_NUMBER_STRING_PRESENTATION_FORMAT,
    ATT_PERMISSION_CHARACTERISTIC,                                      //ATT_HDL_DIS_CHARACTERISTIC_MANUFACTURER_NAME_STRING,
    ATT_PERMISSION_HDL_DIS_MANUFACTURER_NAME_STRING,                    //ATT_HDL_DIS_MANUFACTURER_NAME_STRING,
    ATT_PERMISSION_CHARACTERISTIC_PRESENTATION_FORMAT,                  //ATT_HDL_DIS_MANUFACTURER_NAME_STRING_PRESENTATION_FORMAT,
    ATT_PERMISSION_CHARACTERISTIC,                                      //ATT_HDL_DIS_CHARACTERISTIC_SYSTEM_ID,
    ATT_PERMISSION_HDL_DIS_SYSTEM_ID,                                   //ATT_HDL_DIS_SYSTEM_ID,
    ATT_PERMISSION_CHARACTERISTIC,                                      //ATT_HDL_DIS_CHARACTERISTIC_FIRMWARE_REVISION_STRING,
    ATT_PERMISSION_HDL_DIS_FIRMWARE_REVISION_STRING,                    //ATT_HDL_DIS_FIRMWARE_REVISION_STRING,
    ATT_PERMISSION_CHARACTERISTIC_PRESENTATION_FORMAT,                  //ATT_HDL_DIS_FIRMWARE_REVISION_STRING_PRESENTATION_FORMAT,
    ATT_PERMISSION_CHARACTERISTIC,                                      //ATT_HDL_DIS_CHARACTERISTIC_MODEL_NUMBER_STRING,
    ATT_PERMISSION_HDL_DIS_MODEL_NUMBER_STRING,                         //ATT_HDL_DIS_MODEL_NUMBER_STRING,
    ATT_PERMISSION_CHARACTERISTIC_PRESENTATION_FORMAT,                  //ATT_HDL_DIS_MODEL_NUMBER_STRING_PRESENTATION_FORMAT,
    ATT_PERMISSION_CHARACTERISTIC,                                      //ATT_HDL_DIS_CHARACTERISTIC_HARDWARE_REVISION_STRING,
    ATT_PERMISSION_HDL_DIS_HARDWARE_REVISION_STRING,                    //ATT_HDL_DIS_HARDWARE_REVISION_STRING,
    ATT_PERMISSION_CHARACTERISTIC_PRESENTATION_FORMAT,                  //ATT_HDL_DIS_HARDWARE_REVISION_STRING_PRESENTATION_FORMAT,
    ATT_PERMISSION_CHARACTERISTIC,                                      //ATT_HDL_DIS_CHARACTERISTIC_SOFTWARE_REVISION_STRING,
    ATT_PERMISSION_HDL_DIS_SOFTWARE_REVISION_STRING,                    //ATT_HDL_DIS_SOFTWARE_REVISION_STRING,
    ATT_PERMISSION_CHARACTERISTIC_PRESENTATION_FORMAT,                  //ATT_HDL_DIS_SOFTWARE_REVISION_STRING_PRESENTATION_FORMAT,
    ATT_PERMISSION_CHARACTERISTIC,                                      //ATT_HDL_DIS_CHARACTERISTIC_PNP_ID,
    ATT_PERMISSION_HDL_DIS_PNP_ID,                                      //ATT_HDL_DIS_PNP_ID,
#ifdef _PROFILE_HRP_
    ATT_PERMISSION_PRIMARY_SERVICE,                                     //ATT_HDL_HRS_PRIMARY_SERVICE,
    ATT_PERMISSION_CHARACTERISTIC,                                      //ATT_HDL_HRS_CHARACTERISTIC_HEART_RATE_MEASUREMENT,
    ATT_PERMISSION_HDL_HRS_HEART_RATE_MEASUREMENT_INIT,                 //ATT_HDL_HRS_HEART_RATE_MEASUREMENT_INIT,
    ATT_PERMISSION_HDL_HRS_CLIENT_CHARACTERISTIC_CONFIGURATION_INIT,    //ATT_HDL_HRS_CLIENT_CHARACTERISTIC_CONFIGURATION_INIT,
    ATT_PERMISSION_CHARACTERISTIC,                                      //ATT_HDL_HRS_CHARACTERISTIC_BODY_SENSOR_LOCATION,
    ATT_PERMISSION_HDL_HRS_BODY_SENSOR_LOCATION_INIT,                   //ATT_HDL_HRS_BODY_SENSOR_LOCATION_INIT,
#endif
#ifdef _PROFILE_BAS_
    ATT_PERMISSION_PRIMARY_SERVICE,                                     //ATT_HDL_BAS_PRIMARY_SERVICE,
    ATT_PERMISSION_CHARACTERISTIC,                                      //ATT_HDL_BAS_CHARACTERISTIC_BATTERY_LEVEL,
    ATT_PERMISSION_HDL_BAS_BATTERY_LEVEL_INIT,                          //ATT_HDL_BAS_BATTERY_LEVEL_INIT,
    //ATT_PERMISSION_CHARACTERISTIC_PRESENTATION_FORMAT,                  //ATT_HDL_BAS_BATTERY_LEVEL_PRESENTATION_FORMAT,
    ATT_PERMISSION_HDL_BAS_CLIENT_CHARACTERISTIC_CONFIGURATION_INIT,    //ATT_HDL_BAS_CLIENT_CHARACTERISTIC_CONFIGURATION_INIT
    ATT_PERMISSION_CHARACTERISTIC,                                      //ATT_HDL_BAS_CHARACTERISTIC_BATTERY_POWER_STATE,
    ATT_PERMISSION_HDL_BAS_BATTERY_POWER_STATE,                         //ATT_HDL_BAS_BATTERY_POWER_STATE_INIT
#endif
#ifdef _PROFILE_HOGP_
    ATT_PERMISSION_PRIMARY_SERVICE,
    ATT_PERMISSION_CHARACTERISTIC,
    ATT_PERMISSION_HDL_HIDS_HID_INFORMATION,
    ATT_PERMISSION_CHARACTERISTIC,
    ATT_PERMISSION_HDL_HIDS_HID_CONTROL_POINT_INIT,
    ATT_PERMISSION_CHARACTERISTIC,
    ATT_PERMISSION_HDL_HIDS_REPORT_MAP,
#ifdef _PROFILE_HOGP_KEYBOARD_
    ATT_PERMISSION_CHARACTERISTIC,
    ATT_PERMISSION_HDL_HIDS_REPORT_INIT,
    ATT_PERMISSION_HDL_HIDS_REPORT_CLIENT_CHARACTERISTIC_CONFIGURATION_INIT,
    ATT_PERMISSION_HDL_HIDS_REPORT_REPORT_REFERENCE,
    ATT_PERMISSION_CHARACTERISTIC,
    ATT_PERMISSION_HDL_HIDS_REPORT1_INIT,
    ATT_PERMISSION_HDL_HIDS_REPORT1_REPORT_REFERENCE,
#endif
#ifdef _PROFILE_HOGP_MOUSE_
    ATT_PERMISSION_CHARACTERISTIC,
    ATT_PERMISSION_HDL_HIDS_REPORT_INIT,
    ATT_PERMISSION_HDL_HIDS_REPORT_CLIENT_CHARACTERISTIC_CONFIGURATION_INIT,
    ATT_PERMISSION_HDL_HIDS_REPORT_REPORT_REFERENCE,
    ATT_PERMISSION_CHARACTERISTIC,
    ATT_PERMISSION_HDL_HIDS_REPORT1_INIT,
    ATT_PERMISSION_HDL_HIDS_REPORT1_REPORT_REFERENCE,
    ATT_PERMISSION_HDL_HIDS_REPORT1_USER_DESCRIPTION,
#endif
#ifdef _PROFILE_HOGP_COMSUMER_
    ATT_PERMISSION_CHARACTERISTIC,
    ATT_PERMISSION_HDL_HIDS_REPORT_INIT,
    ATT_PERMISSION_HDL_HIDS_REPORT_CLIENT_CHARACTERISTIC_CONFIGURATION_INIT,
    ATT_PERMISSION_HDL_HIDS_REPORT_REPORT_REFERENCE,
#endif
#ifdef _PROFILE_HOGP_EMC_
    ATT_PERMISSION_CHARACTERISTIC,
    ATT_PERMISSION_HDL_HIDS_REPORT_INIT,
    ATT_PERMISSION_HDL_HIDS_REPORT_CLIENT_CHARACTERISTIC_CONFIGURATION_INIT,
    ATT_PERMISSION_HDL_HIDS_REPORT_REPORT_REFERENCE,
    ATT_PERMISSION_CHARACTERISTIC,
    ATT_PERMISSION_HDL_HIDS_REPORT_INIT,
    ATT_PERMISSION_HDL_HIDS_REPORT_CLIENT_CHARACTERISTIC_CONFIGURATION_INIT,
    ATT_PERMISSION_HDL_HIDS_REPORT_REPORT_REFERENCE,
    ATT_PERMISSION_CHARACTERISTIC,
    ATT_PERMISSION_HDL_HIDS_REPORT_INIT,
    ATT_PERMISSION_HDL_HIDS_REPORT_CLIENT_CHARACTERISTIC_CONFIGURATION_INIT,
    ATT_PERMISSION_HDL_HIDS_REPORT_REPORT_REFERENCE,
    ATT_PERMISSION_CHARACTERISTIC,
    ATT_PERMISSION_HDL_HIDS_REPORT_INIT,
    //ATT_PERMISSION_HDL_HIDS_REPORT_CLIENT_CHARACTERISTIC_CONFIGURATION_INIT,
    ATT_PERMISSION_HDL_HIDS_REPORT_REPORT_REFERENCE,
    ATT_PERMISSION_CHARACTERISTIC,
    ATT_PERMISSION_HDL_HIDS_REPORT_INIT,
    //ATT_PERMISSION_HDL_HIDS_REPORT_CLIENT_CHARACTERISTIC_CONFIGURATION_INIT,
    ATT_PERMISSION_HDL_HIDS_REPORT_REPORT_REFERENCE,
    ATT_PERMISSION_CHARACTERISTIC,
    ATT_PERMISSION_HDL_HIDS_REPORT_INIT,
    //ATT_PERMISSION_HDL_HIDS_REPORT_CLIENT_CHARACTERISTIC_CONFIGURATION_INIT,
    ATT_PERMISSION_HDL_HIDS_REPORT_REPORT_REFERENCE,
#endif
    ATT_PERMISSION_CHARACTERISTIC,
    ATT_PERMISSION_HDL_HIDS_PROTOCOL_MODE_INIT,
#ifdef _PROFILE_HOGP_BOOT_
#ifdef _PROFILE_HOGP_KEYBOARD_
    ATT_PERMISSION_CHARACTERISTIC,
    ATT_PERMISSION_HDL_HIDS_BOOT_KEYBOARD_INPUT_REPORT_INIT,
    ATT_PERMISSION_HDL_HIDS_BOOT_KEYBOARD_INPUT_REPORT_CLIENT_CHRCT_CONFIG_INIT,
    ATT_PERMISSION_CHARACTERISTIC,
    ATT_PERMISSION_HDL_HIDS_BOOT_KEYBOARD_OUTPUT_REPORT_INIT,
#endif
#ifdef _PROFILE_HOGP_MOUSE_
    ATT_PERMISSION_CHARACTERISTIC,
    ATT_PERMISSION_HDL_HIDS_BOOT_MOUSE_INPUT_REPORT_INIT,
    ATT_PERMISSION_HDL_HIDS_BOOT_MOUSE_INPUT_REPORT_CLIENT_CHRCT_CONFIG_INIT,
#endif
#endif
#endif
#ifdef _PROFILE_SCPP_
    ATT_PERMISSION_PRIMARY_SERVICE,                                     //ATT_HDL_SCPS_PRIMARY_SERVICE,
    ATT_PERMISSION_CHARACTERISTIC,                                      //ATT_HDL_SCPS_CHARACTERISTIC_SCAN_INTERVAL_WINDOW,
    ATT_PERMISSION_HDL_SCPS_SCAN_INTERVAL_WINDOW_INIT,                  //ATT_HDL_SCPS_SCAN_INTERVAL_WINDOW_INIT,
    ATT_PERMISSION_CHARACTERISTIC,                                      //ATT_HDL_SCPS_CHARACTERISTIC_SCAN_REFRESH,
    ATT_PERMISSION_HDL_SCPS_SCAN_REFRESH_INIT,                          //ATT_HDL_SCPS_SCAN_REFRESH_INIT,
    ATT_PERMISSION_HDL_SCPS_SCAN_REFRESH_CLIENT_CHARACTERISTIC_CONFIGURATION_INIT,  //ATT_HDL_SCPS_SCAN_REFRESH_CLIENT_CHARACTERISTIC_CONFIGURATION_INIT
#endif
#ifdef _PROFILE_IAS_
    ATT_PERMISSION_PRIMARY_SERVICE,                                     //ATT_HDL_IAS_PRIMARY_SERVICE,
    ATT_PERMISSION_CHARACTERISTIC,                                      //ATT_HDL_IAS_CHARACTERISTIC_ALERT_LEVEL,
    ATT_PERMISSION_HDL_IAS_ALERT_LEVEL_INIT,                            //ATT_HDL_IAS_ALERT_LEVEL_INIT,
#endif
#ifdef _PROFILE_LLS_
    ATT_PERMISSION_PRIMARY_SERVICE,                                     //ATT_HDL_LLS_PRIMARY_SERVICE,
    ATT_PERMISSION_CHARACTERISTIC,                                      //ATT_HDL_LLS_CHARACTERISTIC_ALERT_LEVEL,
    ATT_PERMISSION_HDL_LLS_ALERT_LEVEL_INIT,                            //ATT_HDL_LLS_ALERT_LEVEL_INIT,
#endif
#ifdef _PROFILE_TPS_
    ATT_PERMISSION_PRIMARY_SERVICE,                                     //ATT_HDL_TPS_PRIMARY_SERVICE,
    ATT_PERMISSION_CHARACTERISTIC,                                      //ATT_HDL_TPS_CHARACTERISTIC_TX_POWER_LEVEL,
    ATT_PERMISSION_HDL_TPS_TX_POWER_LEVEL_INIT,                         //ATT_HDL_TPS_TX_POWER_LEVEL_INIT,
#endif
#ifdef _PROFILE_BLP_
    ATT_PERMISSION_PRIMARY_SERVICE,                                     //ATT_HDL_BLS_PRIMARY_SERVICE,
    ATT_PERMISSION_CHARACTERISTIC,                                      //ATT_HDL_BLS_CHARACTERISTIC_BLOOD_PRESSURE_MEASUREMENT,
    ATT_PERMISSION_HDL_BLS_BLOOD_PRESSURE_MEASUREMENT_INIT,             //ATT_HDL_BLS_BLOOD_PRESSURE_MEASUREMENT_INIT,
    ATT_PERMISSION_HDL_BLS_BLOOD_PRESSURE_CLIENT_CHARACTERISTIC_CONFIGURATION_INIT,     //ATT_HDL_BLS_BLOOD_PRESSURE_CLIENT_CHARACTERISTIC_CONFIGURATION_INIT,
    ATT_PERMISSION_CHARACTERISTIC,                                      //ATT_HDL_BLS_CHARACTERISTIC_BLOOD_PRESSURE_FEATURE,
    ATT_PERMISSION_HDL_BLS_BLOOD_PRESSURE_FEATURE_INIT,                 //ATT_HDL_BLS_BLOOD_PRESSURE_FEATURE_INIT,
#endif
#ifdef _PROFILE_HTP_
    ATT_PERMISSION_PRIMARY_SERVICE,                                     //ATT_HDL_HTS_PRIMARY_SERVICE,
    ATT_PERMISSION_CHARACTERISTIC,                                      //ATT_HDL_HTS_CHARACTERISTIC_TEMPERATURE_MEASUREMENT,
    ATT_PERMISSION_HDL_HTS_TEMPERATURE_MEASUREMENT_INIT,                //ATT_HDL_HTS_TEMPERATURE_MEASUREMENT_INIT,
    ATT_PERMISSION_HDL_HTS_TEMPERATURE_MEASUREMENT_CLIENT_CHARACTERISTIC_CONFIGURATION_INIT,    //ATT_HDL_HTS_TEMPERATURE_MEASUREMENT_CLIENT_CHARACTERISTIC_CONFIGURATION_INIT,
    ATT_PERMISSION_CHARACTERISTIC,                                      //ATT_HDL_HTS_CHARACTERISTIC_TEMPERATURE_TYPE,
    ATT_PERMISSION_HDL_HTS_TEMPERATURE_TYPE_INIT,                       //ATT_HDL_HTS_TEMPERATURE_TYPE_INIT,
#endif
#ifdef _PROFILE_RSCP_
    ATT_PERMISSION_PRIMARY_SERVICE,                                     //ATT_HDL_RSCS_PRIMARY_SERVICE,
    ATT_PERMISSION_CHARACTERISTIC,                                      //ATT_HDL_RSCS_CHARACTERISTIC_RSC_MEASUREMENT,
    ATT_PERMISSION_HDL_RSCS_RSC_MEASUREMENT_INIT,                       //ATT_HDL_RSCS_RSC_MEASUREMENT_INIT,
    ATT_PERMISSION_HDL_RSCS_RSC_MEASUREMENT_CLIENT_CHARACTERISTIC_CONFIGURATION_INIT,    //ATT_HDL_RSCS_RSC_MEASUREMENT_CLIENT_CHARACTERISTIC_CONFIGURATION_INIT,
    ATT_PERMISSION_CHARACTERISTIC,                                      //ATT_HDL_RSCS_CHARACTERISTIC_RSC_FEATURE,
    ATT_PERMISSION_HDL_RSCS_RSC_FEATURE_INIT,                           //ATT_HDL_RSCS_RSC_FEATURE_INIT,
    ATT_PERMISSION_CHARACTERISTIC,                                      //ATT_HDL_RSCS_CHARACTERISTIC_SC_CONTROL_POINT,
    ATT_PERMISSION_HDL_RSCS_SC_CONTROL_POINT_INIT,                      //ATT_HDL_RSCS_SC_CONTROL_POINT_INIT,
    ATT_PERMISSION_HDL_RSCS_SC_CONTROL_POINT_CLIENT_CHARACTERISTIC_CONFIGURATION_INIT,    //ATT_HDL_RSCS_SC_CONTROL_POINT_CLIENT_CHARACTERISTIC_CONFIGURATION_INIT,
#endif
#ifdef _PROFILE_CSCP_
    ATT_PERMISSION_PRIMARY_SERVICE,                                     //ATT_HDL_CSCS_PRIMARY_SERVICE,
    ATT_PERMISSION_CHARACTERISTIC,                                      //ATT_HDL_CSCS_CHARACTERISTIC_CSC_MEASUREMENT,
    ATT_PERMISSION_HDL_CSCS_CSC_MEASUREMENT_INIT,                       //ATT_HDL_CSCS_CSC_MEASUREMENT_INIT,
    ATT_PERMISSION_HDL_CSCS_CSC_MEASUREMENT_CLIENT_CHARACTERISTIC_CONFIGURATION_INIT,    //ATT_HDL_CSCS_CSC_MEASUREMENT_CLIENT_CHARACTERISTIC_CONFIGURATION_INIT,
    ATT_PERMISSION_CHARACTERISTIC,                                      //ATT_HDL_CSCS_CHARACTERISTIC_CSC_FEATURE,
    ATT_PERMISSION_HDL_CSCS_CSC_FEATURE_INIT,                           //ATT_HDL_CSCS_CSC_FEATURE_INIT,
    ATT_PERMISSION_CHARACTERISTIC,                                      //ATT_HDL_CSCS_CHARACTERISTIC_SC_CONTROL_POINT,
    ATT_PERMISSION_HDL_CSCS_SC_CONTROL_POINT_INIT,                      //ATT_HDL_CSCS_SC_CONTROL_POINT_INIT,
    ATT_PERMISSION_HDL_CSCS_SC_CONTROL_POINT_CLIENT_CHARACTERISTIC_CONFIGURATION_INIT,    //ATT_HDL_CSCS_SC_CONTROL_POINT_CLIENT_CHARACTERISTIC_CONFIGURATION_INIT,
#endif
#ifdef _PROFILE_LNS_
    ATT_PERMISSION_PRIMARY_SERVICE,                                     //ATT_HDL_LNS_PRIMARY_SERVICE,
    ATT_PERMISSION_CHARACTERISTIC,                                      //ATT_HDL_LNS_CHARACTERISTIC_LOCATION_AND_SPEED,
    ATT_PERMISSION_HDL_LNS_LOCATION_AND_SPEED_INIT,                     //ATT_HDL_LNS_LOCATION_AND_SPEED,
    ATT_PERMISSION_HDL_LNS_LOCATION_AND_SPEED_CLIENT_CHARACTERISTIC_CONFIGURATION_INIT, //ATT_HDL_LNS_LOCATION_AND_SPEED_CLIENT_CHARACTERISTIC_CONFIGURATION_INIT,
    ATT_PERMISSION_CHARACTERISTIC,                                      //ATT_HDL_LNS_CHARACTERISTIC_LN_FEATURE,
    ATT_PERMISSION_HDL_LNS_LN_FEATURE_INIT,                             //ATT_HDL_LNS_LN_FEATURE,
    ATT_PERMISSION_CHARACTERISTIC,                                      //ATT_HDL_LNS_CHARACTERISTIC_POSITION_QUALITY,
    ATT_PERMISSION_HDL_LNS_POSITION_QUALITY_INIT,                       //ATT_HDL_LNS_POSITION_QUALITY,
    ATT_PERMISSION_CHARACTERISTIC,                                      //ATT_HDL_LNS_CHARACTERISTIC_LN_CONTROL_POINT,
    ATT_PERMISSION_HDL_LNS_LN_CONTROL_POINT_INIT,                       //ATT_HDL_LNS_LN_CONTROL_POINT_INIT,
    ATT_PERMISSION_HDL_LNS_LN_CONTROL_POINT_CLIENT_CHARACTERISTIC_CONFIGURATION_INIT,   //ATT_HDL_LNS_LN_CONTROL_POINT_CLIENT_CHARACTERISTIC_CONFIGURATION_INIT,
    ATT_PERMISSION_CHARACTERISTIC,                                      //ATT_HDL_LNS_CHARACTERISTIC_NAVIGATION,
    ATT_PERMISSION_HDL_LNS_NAVIGATION_INIT,                             //ATT_HDL_LNS_NAVIGATION,
    ATT_PERMISSION_HDL_LNS_NAVIGATION_CLIENT_CHARACTERISTIC_CONFIGURATION_INIT,         //ATT_HDL_LNS_NAVIGATION_CLIENT_CHARACTERISTIC_CONFIGURATION_INIT,
#endif
#ifdef _PROFILE_GLS_
    ATT_PERMISSION_PRIMARY_SERVICE,                                     //ATT_HDL_GLS_PRIMARY_SERVICE,
    ATT_PERMISSION_CHARACTERISTIC,                                      //ATT_HDL_GLS_CHARACTERISTIC_GLUCOSE_MEASUREMENT,
    ATT_PERMISSION_HDL_GLS_GLUCOSE_MEASUREMENT_INIT,                    //ATT_HDL_GLS_GLUCOSE_MEASUREMENT,
    ATT_PERMISSION_HDL_GLS_GLUCOSE_MEASUREMENT_CLIENT_CHARACTERISTIC_CONFIGURATION_INIT,            //ATT_HDL_GLS_GLUCOSE_MEASUREMENT_CLIENT_CHARACTERISTIC_CONFIGURATION_INIT,
    ATT_PERMISSION_CHARACTERISTIC,                                      //ATT_HDL_GLS_CHARACTERISTIC_GLUCOSE_MEASUREMENT_CONTEXT,
    ATT_PERMISSION_HDL_GLS_GLUCOSE_MEASUREMENT_CONTEXT_INIT,            //ATT_HDL_GLS_GLUCOSE_MEASUREMENT_CONTEXT,
    ATT_PERMISSION_HDL_GLS_GLUCOSE_MEASUREMENT_CONTEXT_CLIENT_CHARACTERISTIC_CONFIGURATION_INIT,    //ATT_HDL_GLS_GLUCOSE_MEASUREMENT_CONTEXT_CLIENT_CHARACTERISTIC_CONFIGURATION_INIT,
    ATT_PERMISSION_CHARACTERISTIC,                                      //ATT_HDL_GLS_CHARACTERISTIC_GLUCOSE_FEATURE,
    ATT_PERMISSION_HDL_GLS_GLUCOSE_FEATURE_INIT,                        //ATT_HDL_GLS_GLUCOSE_FEATURE,
    ATT_PERMISSION_CHARACTERISTIC,                                      //ATT_HDL_GLS_CHARACTERISTIC_RECORD_ACCESS_CONTROL_POINT,
    ATT_PERMISSION_HDL_GLS_RECORD_ACCESS_CONTROL_POINT_INIT,            //ATT_HDL_GLS_RECORD_ACCESS_CONTROL_POINT_INIT,
    ATT_PERMISSION_HDL_GLS_RECORD_ACCESS_CONTROL_POINT_CLIENT_CHARACTERISTIC_CONFIGURATION_INIT,    //ATT_HDL_GLS_RECORD_ACCESS_CONTROL_POINT_CLIENT_CHARACTERISTIC_CONFIGURATION_INIT,
#endif
#ifdef _PROFILE_CPS_
    ATT_PERMISSION_PRIMARY_SERVICE,                                     //ATT_HDL_CPS_PRIMARY_SERVICE,
    ATT_PERMISSION_CHARACTERISTIC,                                      //ATT_HDL_CPS_CHARACTERISTIC_CYCLING_POWER_MEASUREMENT,
    ATT_PERMISSION_HDL_CPS_CYCLING_POWER_MEASUREMENT_INIT,              //ATT_HDL_CPS_CYCLING_POWER_MEASUREMENT,
    ATT_PERMISSION_HDL_CPS_CYCLING_POWER_MEASUREMENT_CLIENT_CHARACTERISTIC_CONFIGURATION_INIT,      //ATT_HDL_CPS_CYCLING_POWER_MEASUREMENT_CLIENT_CHARACTERISTIC_CONFIGURATION_INIT,
    ATT_PERMISSION_HDL_CPS_CYCLING_POWER_MEASUREMENT_SERVER_CHARACTERISTIC_CONFIGURATION_INIT,      //ATT_HDL_CPS_CYCLING_POWER_MEASUREMENT_SERVER_CHARACTERISTIC_CONFIGURATION_INIT,
    ATT_PERMISSION_CHARACTERISTIC,                                      //ATT_HDL_CPS_CHARACTERISTIC_CYCLING_POWER_FEATURE,
    ATT_PERMISSION_HDL_CPS_CYCLING_POWER_FEATURE_INIT,                  //ATT_HDL_CPS_CYCLING_POWER_FEATURE,
    ATT_PERMISSION_CHARACTERISTIC,                                      //ATT_HDL_CPS_CHARACTERISTIC_SENSOR_LOCATION,
    ATT_PERMISSION_HDL_CPS_SENSOR_LOCATION_INIT,                        //ATT_HDL_CPS_SENSOR_LOCATION,
    ATT_PERMISSION_CHARACTERISTIC,                                      //ATT_HDL_CPS_CHARACTERISTIC_CYCLING_POWER_VECTOR,
    ATT_PERMISSION_HDL_CPS_CYCLING_POWER_VECTOR_INIT,                   //ATT_HDL_CPS_CYCLING_POWER_VECTOR,
    ATT_PERMISSION_HDL_CPS_CYCLING_POWER_VECTOR_CLIENT_CHARACTERISTIC_CONFIGURATION_INIT,           //ATT_HDL_CPS_CYCLING_POWER_VECTOR_CLIENT_CHARACTERISTIC_CONFIGURATION_INIT,
    ATT_PERMISSION_CHARACTERISTIC,                                      //ATT_HDL_CPS_CHARACTERISTIC_CYCLING_POWER_CONTROL_POINT,
    ATT_PERMISSION_HDL_CPS_CYCLING_POWER_CONTROL_POINT_INIT,            //ATT_HDL_CPS_CYCLING_POWER_CONTROL_POINT_INIT,
    ATT_PERMISSION_HDL_CPS_CYCLING_POWER_CONTROL_POINT_CLIENT_CHARACTERISTIC_CONFIGURATION_INIT,    //ATT_HDL_CPS_CYCLING_POWER_CONTROL_POINT_CLIENT_CHARACTERISTIC_CONFIGURATION_INIT,
#endif
#ifdef _PROFILE_CSTM_UDF01S_
    ATT_PERMISSION_PRIMARY_SERVICE,                                     //ATT_HDL_UDF01S_PRIMARY_SERVICE,
    ATT_PERMISSION_CHARACTERISTIC,                                      //ATT_HDL_UDF01S_CHARACTERISTIC_UDATR01,
    ATT_PERMISSION_HDL_UDF01S_UDATR01,                                  //ATT_HDL_UDF01S_UDATR01,
    ATT_PERMISSION_CHARACTERISTIC,                                      //ATT_HDL_UDF01S_CHARACTERISTIC_UDATN01,
    ATT_PERMISSION_HDL_UDF01S_UDATN01_INIT,                             //ATT_HDL_UDF01S_UDATN01_INIT,
    ATT_PERMISSION_HDL_UDF01S_UDATN01_CLIENT_CHARACTERISTIC_CONFIGURATION_INIT,
    ATT_PERMISSION_CHARACTERISTIC,                                      //ATT_HDL_UDF01S_CHARACTERISTIC_UDATRW01,
    ATT_PERMISSION_HDL_UDF01S_UDATRW01_INIT,                            //ATT_HDL_UDF01S_UDATRW01_INIT,
#endif
    ATT_PERMISSION_PRIMARY_SERVICE,                                     //ATT_HDL_UDF01S_PRIMARY_SERVICE,
    ATT_PERMISSION_CHARACTERISTIC,                                      //ATT_HDL_UDF01S_CHARACTERISTIC_UDATR01,
    ATT_PERMISSION_HDL_UDF_UDATR01,                                  //ATT_HDL_UDF01S_UDATR01,
    ATT_PERMISSION_CHARACTERISTIC,                                      //ATT_HDL_UDF01S_CHARACTERISTIC_UDATN01,
    ATT_PERMISSION_HDL_UDF_UDATN01_INIT,                             //ATT_HDL_UDF01S_UDATN01_INIT,
    ATT_PERMISSION_HDL_UDF_UDATN01_CLIENT_CHARACTERISTIC_CONFIGURATION_INIT,
    ATT_PERMISSION_CHARACTERISTIC,                                      //ATT_HDL_UDF01S_CHARACTERISTIC_UDATRW01,
    ATT_PERMISSION_HDL_UDF_UDATRW01_INIT,                            //ATT_HDL_UDF01S_UDATRW01_INIT,
#ifdef _PROFILE_CSTM_UDF02S_
    ATT_PERMISSION_PRIMARY_SERVICE,                                     //ATT_HDL_UDF01S_PRIMARY_SERVICE,
    ATT_PERMISSION_CHARACTERISTIC,                                      //ATT_HDL_UDF02S_CHARACTERISTIC_UDATR02,
    ATT_PERMISSION_HDL_UDF02S_UDATR02,                                  //ATT_HDL_UDF02S_UDATR02,
    ATT_PERMISSION_CHARACTERISTIC,                                      //ATT_HDL_UDF02S_CHARACTERISTIC_UDATN02,
    ATT_PERMISSION_HDL_UDF02S_UDATN02_INIT,                             //ATT_HDL_UDF02S_UDATN02_INIT,
    ATT_PERMISSION_HDL_UDF02S_UDATN02_CLIENT_CHARACTERISTIC_CONFIGURATION_INIT,
    ATT_PERMISSION_CHARACTERISTIC,                                      //ATT_HDL_UDF02S_CHARACTERISTIC_UDATRW02,
    ATT_PERMISSION_HDL_UDF02S_UDATRW02_INIT,                            //ATT_HDL_UDF02S_UDATRW02_INIT,
#endif
#ifdef _PROFILE_CSTM_UDF03S_
    ATT_PERMISSION_PRIMARY_SERVICE,                                     //ATT_HDL_UDF01S_PRIMARY_SERVICE,
    ATT_PERMISSION_CHARACTERISTIC,                                      //ATT_HDL_UDF03S_CHARACTERISTIC_UDATR03,
    ATT_PERMISSION_HDL_UDF03S_UDATR03,                                  //ATT_HDL_UDF03S_UDATR03,
    ATT_PERMISSION_CHARACTERISTIC,                                      //ATT_HDL_UDF03S_CHARACTERISTIC_UDATN03,
    ATT_PERMISSION_HDL_UDF03S_UDATN03_INIT,                             //ATT_HDL_UDF03S_UDATN03_INIT,
    ATT_PERMISSION_HDL_UDF03S_UDATN03_CLIENT_CHARACTERISTIC_CONFIGURATION_INIT,
    ATT_PERMISSION_CHARACTERISTIC,                                      //ATT_HDL_UDF03S_CHARACTERISTIC_UDATRW03,
    ATT_PERMISSION_HDL_UDF03S_UDATRW03_INIT,                            //ATT_HDL_UDF03S_UDATRW03_INIT,
#endif

#ifdef _PROFILE_CSTM_DATAEXCHANGE_S_
    ATT_PERMISSION_PRIMARY_SERVICE,                                     //ATT_HDL_UDF01S_PRIMARY_SERVICE,
    ATT_PERMISSION_CHARACTERISTIC,                                      //ATT_HDL_UDF01S_CHARACTERISTIC_UDATR01,
    ATT_PERMISSION_HDL_DATAEXCHANGE_WRITE,
    ATT_PERMISSION_CHARACTERISTIC,
    ATT_PERMISSION_HDL_DATAEXCHANGE_READINT_INIT,
    ATT_PERMISSION_HDL_DATAEXCHANGE_READINT_CLIENT_CHARACTERISTIC_CONFIGURATION_INIT,
#endif

#ifdef _PROFILE_OTA_
    ATT_PERMISSION_PRIMARY_SERVICE,                                             //ATT_HDL_OTA_PRIMARY_SERVICE,
    ATT_PERMISSION_CHARACTERISTIC,                                              //ATT_HDL_OTA_CHARACTERISTIC_OTA_DATA,
    ATT_PERMISSION_HDL_OTA_OTA_DATA_INIT,                                       //ATT_HDL_OTA_OTA_DATA_INIT,
    ATT_PERMISSION_HDL_OTA_OTA_DATA_CLIENT_CHARACTERISTIC_CONFIGURATION_INIT,   //ATT_HDL_OTA_OTA_DATA_CLIENT_CHARACTERISTIC_CONFIGURATION_INIT,
    ATT_PERMISSION_HDL_OTA_OTA_DATA_DESCRIPTION,                                //ATT_HDL_OTA_OTA_DATA_DESCRIPTION,
    ATT_PERMISSION_CHARACTERISTIC,                                              //ATT_HDL_OTA_CHARACTERISTIC_OTA_CMD_CONTROL_POINT,
    ATT_PERMISSION_HDL_OTA_OTA_CMD_CONTROL_POINT_INIT,                          //ATT_HDL_OTA_OTA_CMD_CONTROL_POINT_INIT,
    ATT_PERMISSION_HDL_OTA_OTA_CMD_CP_CLIENT_CHARACTERISTIC_CONFIGURATION_INIT, //ATT_HDL_OTA_OTA_CMD_CONTROL_POINT_CLIENT_CHARACTERISTIC_CONFIGURATION_INIT,
    ATT_PERMISSION_HDL_OTA_OTA_CMD_USER_DESCRIPTION,                            //ATT_HDL_OTA_OTA_CMD_USER_DESCRIPTION,
#endif


};


/*!
    \brief  No operation while attribute write related command occurs.
    \param  length length of conents writing to the GATT characteristic.
    \param  srcCMD address of conents writing to the GATT characteristic.
*/
void ATT_HDL_Wr_NULL(uint8_t connID, uint8_t length, uint8_t *srcCMD)
{
}


#ifdef _PROFILE_CSTM_UDF01S_
extern void trspx_receive_data_callback(uint8_t length, uint8_t *data);


/*!
    \brief  Calling fuction while attribute write related to att_HDL_UDF01S_UDATRW01..
    \param  length length of conents writing to the GATT characteristic.
    \param  srcCMD address of conents writing to the GATT characteristic.
*/
void ATT_HDL_Wr_UDF01S_UDATRW01(uint8_t connID, uint8_t length, uint8_t *srcCMD)
{
    //Every time Central WRITE data, "att_HDL_UDF01S_UDATRW01" is updated and callback this function!
    trspx_receive_data_callback(length, srcCMD);
}
#endif  //(#ifdef _PROFILE_CSTM_UDF01S_)

int RF_LED_FLOG = 0;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void ATT_HDL_Wr_UDF_UDATRW01(uint8_t connID, uint8_t length, uint8_t *srcCMD)
{
    //Every time Central WRITE data, "att_HDL_UDF01S_UDATRW01" is updated and callback this function!
    //    trspx_receive_data_callback(length, srcCMD);
    RF_LED_FLOG=(*(uint8_t *)srcCMD);

}

////////////////////////////////////////////////////////////////////////////////////////////////////////////

#ifdef _PROFILE_CSTM_UDF02S_
void ATT_HDL_Wr_UDF02S_UDATRW02(uint8_t connID, uint8_t length, uint8_t *srcCMD)
{

}
#endif  //(#ifdef _PROFILE_CSTM_UDF02S_)



#ifdef _PROFILE_CSTM_UDF03S_
void ATT_HDL_Wr_UDF03S_UDATRW03(uint8_t connID, uint8_t length, uint8_t *srcCMD)
{
}
#endif  //(#ifdef _PROFILE_CSTM_UDF03S_)


#ifdef  _PROFILE_CSTM_DATAEXCHANGE_S_
extern void BleDataExcahngeService_Callback(uint8_t length, uint8_t *data);
void ATT_HDL_Wr_DATAEXCHANGE_WRITE(uint8_t connID, uint8_t length, uint8_t *srcCMD)
{
    //Every time Central WRITE data, "att_HDL_UDF01S_UDATRW01" is updated and callback this function!
    BleDataExcahngeService_Callback(length, srcCMD);
}
#endif  //(#ifdef _PROFILE_CSTM_UDF03S_)

#ifdef _PROFILE_OTA_
extern void BleFota_Cmd(uint8_t length, uint8_t *data);
extern void BleFota_Data(uint8_t length, uint8_t *data);

void ATT_HDL_Wr_OTA_OTA_DATA(uint8_t connID, uint8_t length, uint8_t *srcCMD)
{
    BleFota_Data(length, srcCMD);
}

void ATT_HDL_Wr_OTA_OTA_CMD_CONTROL_POINT(uint8_t connID, uint8_t length, uint8_t *srcCMD)
{
    BleFota_Cmd(length, srcCMD);
}

#endif


/*!
    \brief  Attribute write related command will call the function
            to corresponding GATT characteristic.
    \param  length length of conents writing to the GATT characteristic.
    \param  srcCMD address of conents writing to the GATT characteristic.
*/
void (* const ATT_Write[])(uint8_t connID, uint8_t length, uint8_t *srcCMD) =
{
    ATT_HDL_Wr_NULL,                                                    //ATT_HDL_INVALID,
    ATT_HDL_Wr_NULL,                                                    //ATT_HDL_GAP_PRIMARY_SERVICE,
    ATT_HDL_Wr_NULL,                                                    //ATT_HDL_GAP_CHARACTERISTIC_DEVICE_NAME,
    ATT_HDL_Wr_NULL,                                                    //ATT_HDL_GAP_DEVICE_NAME,
    ATT_HDL_Wr_NULL,                                                    //ATT_HDL_GAP_CHARACTERISTIC_APPEARANCE,
    ATT_HDL_Wr_NULL,                                                    //ATT_HDL_GAP_APPEARANCE,
    ATT_HDL_Wr_NULL,                                                    //ATT_HDL_GAP_CHARACTERISTIC_RECONNECTION_ADDRESS,
    ATT_HDL_Wr_NULL,                                                    //ATT_HDL_GAP_RECONNECTION_ADDRESS_INIT,
    ATT_HDL_Wr_NULL,                                                    //ATT_HDL_GAP_CHARACTERISTIC_PERIPHERAL_PRIVACY_FLAG,
    ATT_HDL_Wr_NULL,                                                    //ATT_HDL_GAP_PERIPHERAL_PRIVACY_FLAG,
    ATT_HDL_Wr_NULL,                                                    //ATT_HDL_GAP_CHARACTERISTIC_PERIPHERAL_PREFERRED_CONNECTION_PARAMETERS,
    ATT_HDL_Wr_NULL,                                                    //ATT_HDL_GAP_PERIPHERAL_PREFERRED_CONNECTION_PARAMETERS,
    ATT_HDL_Wr_NULL,                                                    //ATT_HDL_GATT_PRIMARY_SERVICE,
    ATT_HDL_Wr_NULL,                                                    //ATT_HDL_GATT_CHARACTERISTIC_SERVICE_CHANGED,
    ATT_HDL_Wr_NULL,                                                    //ATT_HDL_GATT_SERVICE_CHANGED_INIT,
    ATT_HDL_Wr_NULL,                                                    //ATT_HDL_GATT_CLIENT_CHARACTERISTIC_CONFIGURATION_INIT
    ATT_HDL_Wr_NULL,                                                    //ATT_HDL_DIS_PRIMARY_SERVICE,
    ATT_HDL_Wr_NULL,                                                    //ATT_HDL_DIS_CHARACTERISTIC_SERIAL_NUMBER_STRING,
    ATT_HDL_Wr_NULL,                                                    //ATT_HDL_DIS_SERIAL_NUMBER_STRING,
    ATT_HDL_Wr_NULL,                                                    //ATT_HDL_DIS_SERIAL_NUMBER_STRING_PRESENTATION_FORMAT,
    ATT_HDL_Wr_NULL,                                                    //ATT_HDL_DIS_CHARACTERISTIC_MANUFACTURER_NAME_STRING,
    ATT_HDL_Wr_NULL,                                                    //ATT_HDL_DIS_MANUFACTURER_NAME_STRING,
    ATT_HDL_Wr_NULL,                                                    //ATT_HDL_DIS_MANUFACTURER_NAME_STRING_PRESENTATION_FORMAT,
    ATT_HDL_Wr_NULL,                                                    //ATT_HDL_DIS_CHARACTERISTIC_SYSTEM_ID,
    ATT_HDL_Wr_NULL,                                                    //ATT_HDL_DIS_SYSTEM_ID,
    ATT_HDL_Wr_NULL,                                                    //ATT_HDL_DIS_CHARACTERISTIC_FIRMWARE_REVISION_STRING,
    ATT_HDL_Wr_NULL,                                                    //ATT_HDL_DIS_FIRMWARE_REVISION_STRING,
    ATT_HDL_Wr_NULL,                                                    //ATT_HDL_DIS_FIRMWARE_REVISION_STRING_PRESENTATION_FORMAT,
    ATT_HDL_Wr_NULL,                                                    //ATT_HDL_DIS_CHARACTERISTIC_MODEL_NUMBER_STRING,
    ATT_HDL_Wr_NULL,                                                    //ATT_HDL_DIS_MODEL_NUMBER_STRING,
    ATT_HDL_Wr_NULL,                                                    //ATT_HDL_DIS_MODEL_NUMBER_STRING_PRESENTATION_FORMAT,
    ATT_HDL_Wr_NULL,                                                    //ATT_HDL_DIS_CHARACTERISTIC_HARDWARE_REVISION_STRING,
    ATT_HDL_Wr_NULL,                                                    //ATT_HDL_DIS_HARDWARE_REVISION_STRING,
    ATT_HDL_Wr_NULL,                                                    //ATT_HDL_DIS_HARDWARE_REVISION_STRING_PRESENTATION_FORMAT,
    ATT_HDL_Wr_NULL,                                                    //ATT_HDL_DIS_CHARACTERISTIC_SOFTWARE_REVISION_STRING,
    ATT_HDL_Wr_NULL,                                                    //ATT_HDL_DIS_SOFTWARE_REVISION_STRING,
    ATT_HDL_Wr_NULL,                                                    //ATT_HDL_DIS_SOFTWARE_REVISION_STRING_PRESENTATION_FORMAT,
    ATT_HDL_Wr_NULL,                                                    //ATT_HDL_DIS_CHARACTERISTIC_PNP_ID,
    ATT_HDL_Wr_NULL,                                                    //ATT_HDL_DIS_PNP_ID,
#ifdef _PROFILE_HRP_
    ATT_HDL_Wr_NULL,                                                    //ATT_HDL_HRS_PRIMARY_SERVICE,
    ATT_HDL_Wr_NULL,                                                    //ATT_HDL_HRS_CHARACTERISTIC_HEART_RATE_MEASUREMENT,
    ATT_HDL_Wr_NULL,                                                    //ATT_HDL_HRS_HEART_RATE_MEASUREMENT_INIT,
    ATT_HDL_Wr_NULL,                                                    //ATT_HDL_HRS_CLIENT_CHARACTERISTIC_CONFIGURATION_INIT,
    ATT_HDL_Wr_NULL,                                                    //ATT_HDL_HRS_CHARACTERISTIC_BODY_SENSOR_LOCATION,
    ATT_HDL_Wr_NULL,                                                    //ATT_HDL_HRS_BODY_SENSOR_LOCATION_INIT,
#endif
#ifdef _PROFILE_BAS_
    ATT_HDL_Wr_NULL,                                                    //ATT_HDL_BAS_PRIMARY_SERVICE,
    ATT_HDL_Wr_NULL,                                                    //ATT_HDL_BAS_CHARACTERISTIC_BATTERY_LEVEL,
    ATT_HDL_Wr_NULL,                                                    //ATT_HDL_BAS_BATTERY_LEVEL_INIT,
    //ATT_PERMISSION_CHARACTERISTIC_PRESENTATION_FORMAT,                  //ATT_HDL_BAS_BATTERY_LEVEL_PRESENTATION_FORMAT,
    ATT_HDL_Wr_NULL,                                                    //ATT_HDL_BAS_CLIENT_CHARACTERISTIC_CONFIGURATION_INIT
    ATT_HDL_Wr_NULL,                                                    //ATT_HDL_BAS_CHARACTERISTIC_BATTERY_POWER_STATE,
    ATT_HDL_Wr_NULL,                                                    //ATT_HDL_BAS_BATTERY_POWER_STATE_INIT
#endif
#ifdef _PROFILE_HOGP_
    ATT_HDL_Wr_NULL,
    ATT_HDL_Wr_NULL,
    ATT_HDL_Wr_NULL,
    ATT_HDL_Wr_NULL,
    ATT_HDL_Wr_NULL,
    ATT_HDL_Wr_NULL,
    ATT_HDL_Wr_NULL,
#ifdef _PROFILE_HOGP_KEYBOARD_
    ATT_HDL_Wr_NULL,
    ATT_HDL_Wr_NULL,
    ATT_HDL_Wr_NULL,
    ATT_HDL_Wr_NULL,
    ATT_HDL_Wr_NULL,
    ATT_HDL_Wr_NULL,
    ATT_HDL_Wr_NULL,
#endif
#ifdef _PROFILE_HOGP_MOUSE_
    ATT_HDL_Wr_NULL,
    ATT_HDL_Wr_NULL,
    ATT_HDL_Wr_NULL,
    ATT_HDL_Wr_NULL,
    ATT_HDL_Wr_NULL,
    ATT_HDL_Wr_NULL,
    ATT_HDL_Wr_NULL,
    ATT_HDL_Wr_NULL,
#endif
#ifdef _PROFILE_HOGP_COMSUMER_
    ATT_HDL_Wr_NULL,
    ATT_HDL_Wr_NULL,
    ATT_HDL_Wr_NULL,
    ATT_HDL_Wr_NULL,
#endif
#ifdef _PROFILE_HOGP_EMC_
    ATT_HDL_Wr_NULL,
    ATT_HDL_Wr_NULL,
    ATT_HDL_Wr_NULL,
    ATT_HDL_Wr_NULL,
    ATT_HDL_Wr_NULL,
    ATT_HDL_Wr_NULL,
    ATT_HDL_Wr_NULL,
    ATT_HDL_Wr_NULL,
    ATT_HDL_Wr_NULL,
    ATT_HDL_Wr_NULL,
    ATT_HDL_Wr_NULL,
    ATT_HDL_Wr_NULL,
    ATT_HDL_Wr_NULL,
    ATT_HDL_Wr_NULL,
    //,
    ATT_HDL_Wr_NULL,
    ATT_HDL_Wr_NULL,
    ATT_HDL_Wr_NULL,
    //,
    ATT_HDL_Wr_NULL,
    ATT_HDL_Wr_NULL,
    ATT_HDL_Wr_NULL,
    //,
    ATT_HDL_Wr_NULL,
#endif
    ATT_HDL_Wr_NULL,
    ATT_HDL_Wr_NULL,
#ifdef _PROFILE_HOGP_BOOT_
#ifdef _PROFILE_HOGP_KEYBOARD_
    ATT_HDL_Wr_NULL,
    ATT_HDL_Wr_NULL,
    ATT_HDL_Wr_NULL,
    ATT_HDL_Wr_NULL,
    ATT_HDL_Wr_NULL,
#endif
#ifdef _PROFILE_HOGP_MOUSE_
    ATT_HDL_Wr_NULL,
    ATT_HDL_Wr_NULL,
    ATT_HDL_Wr_NULL,
#endif
#endif
#endif
#ifdef _PROFILE_SCPP_
    ATT_HDL_Wr_NULL,                                                    //ATT_HDL_SCPS_PRIMARY_SERVICE,
    ATT_HDL_Wr_NULL,                                                    //ATT_HDL_SCPS_CHARACTERISTIC_SCAN_INTERVAL_WINDOW,
    ATT_HDL_Wr_NULL,                                                    //ATT_HDL_SCPS_SCAN_INTERVAL_WINDOW_INIT,
    ATT_HDL_Wr_NULL,                                                    //ATT_HDL_SCPS_CHARACTERISTIC_SCAN_REFRESH,
    ATT_HDL_Wr_NULL,                                                    //ATT_HDL_SCPS_SCAN_REFRESH_INIT,
    ATT_HDL_Wr_NULL,                                                    //ATT_HDL_SCPS_SCAN_REFRESH_CLIENT_CHARACTERISTIC_CONFIGURATION_INIT
#endif
#ifdef _PROFILE_IAS_
    ATT_HDL_Wr_NULL,                                                    //ATT_HDL_IAS_PRIMARY_SERVICE,
    ATT_HDL_Wr_NULL,                                                    //ATT_HDL_IAS_CHARACTERISTIC_ALERT_LEVEL,
    ATT_HDL_Wr_NULL,                                                    //ATT_HDL_IAS_ALERT_LEVEL_INIT,
#endif
#ifdef _PROFILE_LLS_
    ATT_HDL_Wr_NULL,                                                    //ATT_HDL_LLS_PRIMARY_SERVICE,
    ATT_HDL_Wr_NULL,                                                    //ATT_HDL_LLS_CHARACTERISTIC_ALERT_LEVEL,
    ATT_HDL_Wr_NULL,                                                    //ATT_HDL_LLS_ALERT_LEVEL_INIT,
#endif
#ifdef _PROFILE_TPS_
    ATT_HDL_Wr_NULL,                                                    //ATT_HDL_TPS_PRIMARY_SERVICE,
    ATT_HDL_Wr_NULL,                                                    //ATT_HDL_TPS_CHARACTERISTIC_TX_POWER_LEVEL,
    ATT_HDL_Wr_NULL,                                                    //ATT_HDL_TPS_TX_POWER_LEVEL_INIT,
#endif
#ifdef _PROFILE_BLP_
    ATT_HDL_Wr_NULL,                                                    //ATT_HDL_BLS_PRIMARY_SERVICE,
    ATT_HDL_Wr_NULL,                                                    //ATT_HDL_BLS_CHARACTERISTIC_BLOOD_PRESSURE_MEASUREMENT,
    ATT_HDL_Wr_NULL,                                                    //ATT_HDL_BLS_BLOOD_PRESSURE_MEASUREMENT_INIT,
    ATT_HDL_Wr_NULL,                                                    //ATT_HDL_BLS_BLOOD_PRESSURE_CLIENT_CHARACTERISTIC_CONFIGURATION_INIT,
    ATT_HDL_Wr_NULL,                                                    //ATT_HDL_BLS_CHARACTERISTIC_BLOOD_PRESSURE_FEATURE,
    ATT_HDL_Wr_NULL,                                                    //ATT_HDL_BLS_BLOOD_PRESSURE_FEATURE_INIT,
#endif
#ifdef _PROFILE_HTP_
    ATT_HDL_Wr_NULL,                                                    //ATT_HDL_HTS_PRIMARY_SERVICE,
    ATT_HDL_Wr_NULL,                                                    //ATT_HDL_HTS_CHARACTERISTIC_TEMPERATURE_MEASUREMENT,
    ATT_HDL_Wr_NULL,                                                    //ATT_HDL_HTS_TEMPERATURE_MEASUREMENT_INIT,
    ATT_HDL_Wr_NULL,                                                    //ATT_HDL_HTS_TEMPERATURE_MEASUREMENT_CLIENT_CHARACTERISTIC_CONFIGURATION_INIT,
    ATT_HDL_Wr_NULL,                                                    //ATT_HDL_HTS_CHARACTERISTIC_TEMPERATURE_TYPE,
    ATT_HDL_Wr_NULL,                                                    //ATT_HDL_HTS_TEMPERATURE_TYPE_INIT,
#endif
#ifdef _PROFILE_RSCP_
    ATT_HDL_Wr_NULL,                                                    //ATT_HDL_RSCS_PRIMARY_SERVICE,
    ATT_HDL_Wr_NULL,                                                    //ATT_HDL_RSCS_CHARACTERISTIC_RSC_MEASUREMENT,
    ATT_HDL_Wr_NULL,                                                    //ATT_HDL_RSCS_RSC_MEASUREMENT_INIT,
    ATT_HDL_Wr_NULL,                                                    //ATT_HDL_RSCS_RSC_MEASUREMENT_CLIENT_CHARACTERISTIC_CONFIGURATION_INIT,
    ATT_HDL_Wr_NULL,                                                    //ATT_HDL_RSCS_CHARACTERISTIC_RSC_FEATURE,
    ATT_HDL_Wr_NULL,                                                    //ATT_HDL_RSCS_RSC_FEATURE_INIT,
    ATT_HDL_Wr_NULL,                                                    //ATT_HDL_RSCS_CHARACTERISTIC_SC_CONTROL_POINT,
    ATT_HDL_Wr_NULL,                                                    //ATT_HDL_RSCS_SC_CONTROL_POINT_INIT,
    ATT_HDL_Wr_NULL,                                                    //ATT_HDL_RSCS_SC_CONTROL_POINT_CLIENT_CHARACTERISTIC_CONFIGURATION_INIT,
#endif
#ifdef _PROFILE_CSCP_
    ATT_HDL_Wr_NULL,                                                    //ATT_HDL_CSCS_PRIMARY_SERVICE,
    ATT_HDL_Wr_NULL,                                                    //ATT_HDL_CSCS_CHARACTERISTIC_CSC_MEASUREMENT,
    ATT_HDL_Wr_NULL,                                                    //ATT_HDL_CSCS_CSC_MEASUREMENT_INIT,
    ATT_HDL_Wr_NULL,                                                    //ATT_HDL_CSCS_CSC_MEASUREMENT_CLIENT_CHARACTERISTIC_CONFIGURATION_INIT,
    ATT_HDL_Wr_NULL,                                                    //ATT_HDL_CSCS_CHARACTERISTIC_CSC_FEATURE,
    ATT_HDL_Wr_NULL,                                                    //ATT_HDL_CSCS_CSC_FEATURE_INIT,
    ATT_HDL_Wr_NULL,                                                    //ATT_HDL_CSCS_CHARACTERISTIC_SC_CONTROL_POINT,
    ATT_HDL_Wr_NULL,                                                    //ATT_HDL_CSCS_SC_CONTROL_POINT_INIT,
    ATT_HDL_Wr_NULL,                                                    //ATT_HDL_CSCS_SC_CONTROL_POINT_CLIENT_CHARACTERISTIC_CONFIGURATION_INIT,
#endif
#ifdef _PROFILE_LNS_
    ATT_HDL_Wr_NULL,                                                    //ATT_HDL_LNS_PRIMARY_SERVICE,
    ATT_HDL_Wr_NULL,                                                    //ATT_HDL_LNS_CHARACTERISTIC_LOCATION_AND_SPEED,
    ATT_HDL_Wr_NULL,                                                    //ATT_HDL_LNS_LOCATION_AND_SPEED,
    ATT_HDL_Wr_NULL,                                                    //ATT_HDL_LNS_LOCATION_AND_SPEED_CLIENT_CHARACTERISTIC_CONFIGURATION_INIT,
    ATT_HDL_Wr_NULL,                                                    //ATT_HDL_LNS_CHARACTERISTIC_LN_FEATURE,
    ATT_HDL_Wr_NULL,                                                    //ATT_HDL_LNS_LN_FEATURE,
    ATT_HDL_Wr_NULL,                                                    //ATT_HDL_LNS_CHARACTERISTIC_POSITION_QUALITY,
    ATT_HDL_Wr_NULL,                                                    //ATT_HDL_LNS_POSITION_QUALITY,
    ATT_HDL_Wr_NULL,                                                    //ATT_HDL_LNS_CHARACTERISTIC_LN_CONTROL_POINT,
    ATT_HDL_Wr_NULL,                                                    //ATT_HDL_LNS_LN_CONTROL_POINT_INIT,
    ATT_HDL_Wr_NULL,                                                    //ATT_HDL_LNS_LN_CONTROL_POINT_CLIENT_CHARACTERISTIC_CONFIGURATION_INIT,
    ATT_HDL_Wr_NULL,                                                    //ATT_HDL_LNS_CHARACTERISTIC_NAVIGATION,
    ATT_HDL_Wr_NULL,                                                    //ATT_HDL_LNS_NAVIGATION,
    ATT_HDL_Wr_NULL,                                                    //ATT_HDL_LNS_NAVIGATION_CLIENT_CHARACTERISTIC_CONFIGURATION_INIT,
#endif
#ifdef _PROFILE_GLS_
    ATT_HDL_Wr_NULL,                                                    //ATT_HDL_GLS_PRIMARY_SERVICE,
    ATT_HDL_Wr_NULL,                                                    //ATT_HDL_GLS_CHARACTERISTIC_GLUCOSE_MEASUREMENT,
    ATT_HDL_Wr_NULL,                                                    //ATT_HDL_GLS_GLUCOSE_MEASUREMENT,
    ATT_HDL_Wr_NULL,                                                    //ATT_HDL_GLS_GLUCOSE_MEASUREMENT_CLIENT_CHARACTERISTIC_CONFIGURATION_INIT,
    ATT_HDL_Wr_NULL,                                                    //ATT_HDL_GLS_CHARACTERISTIC_GLUCOSE_MEASUREMENT_CONTEXT,
    ATT_HDL_Wr_NULL,                                                    //ATT_HDL_GLS_GLUCOSE_MEASUREMENT_CONTEXT,
    ATT_HDL_Wr_NULL,                                                    //ATT_HDL_GLS_GLUCOSE_MEASUREMENT_CONTEXT_CLIENT_CHARACTERISTIC_CONFIGURATION_INIT,
    ATT_HDL_Wr_NULL,                                                    //ATT_HDL_GLS_CHARACTERISTIC_GLUCOSE_FEATURE,
    ATT_HDL_Wr_NULL,                                                    //ATT_HDL_GLS_GLUCOSE_FEATURE,
    ATT_HDL_Wr_NULL,                                                    //ATT_HDL_GLS_CHARACTERISTIC_RECORD_ACCESS_CONTROL_POINT,
    ATT_HDL_Wr_NULL,                                                    //ATT_HDL_GLS_RECORD_ACCESS_CONTROL_POINT_INIT,
    ATT_HDL_Wr_NULL,                                                    //ATT_HDL_GLS_RECORD_ACCESS_CONTROL_POINT_CLIENT_CHARACTERISTIC_CONFIGURATION_INIT,
#endif
#ifdef _PROFILE_CPS_
    ATT_HDL_Wr_NULL,                                                    //ATT_HDL_CPS_PRIMARY_SERVICE,
    ATT_HDL_Wr_NULL,                                                    //ATT_HDL_CPS_CHARACTERISTIC_CYCLING_POWER_MEASUREMENT,
    ATT_HDL_Wr_NULL,                                                    //ATT_HDL_CPS_CYCLING_POWER_MEASUREMENT,
    ATT_HDL_Wr_NULL,                                                    //ATT_HDL_CPS_CYCLING_POWER_MEASUREMENT_CLIENT_CHARACTERISTIC_CONFIGURATION_INIT,
    ATT_HDL_Wr_NULL,                                                    //ATT_HDL_CPS_CYCLING_POWER_MEASUREMENT_SERVER_CHARACTERISTIC_CONFIGURATION_INIT,
    ATT_HDL_Wr_NULL,                                                    //ATT_HDL_CPS_CHARACTERISTIC_CYCLING_POWER_FEATURE,
    ATT_HDL_Wr_NULL,                                                    //ATT_HDL_CPS_CYCLING_POWER_FEATURE,
    ATT_HDL_Wr_NULL,                                                    //ATT_HDL_CPS_CHARACTERISTIC_SENSOR_LOCATION,
    ATT_HDL_Wr_NULL,                                                    //ATT_HDL_CPS_SENSOR_LOCATION,
    ATT_HDL_Wr_NULL,                                                    //ATT_HDL_CPS_CHARACTERISTIC_CYCLING_POWER_VECTOR,
    ATT_HDL_Wr_NULL,                                                    //ATT_HDL_CPS_CYCLING_POWER_VECTOR,
    ATT_HDL_Wr_NULL,                                                    //ATT_HDL_CPS_CYCLING_POWER_VECTOR_CLIENT_CHARACTERISTIC_CONFIGURATION_INIT,
    ATT_HDL_Wr_NULL,                                                    //ATT_HDL_CPS_CHARACTERISTIC_CYCLING_POWER_CONTROL_POINT,
    ATT_HDL_Wr_NULL,                                                    //ATT_HDL_CPS_CYCLING_POWER_CONTROL_POINT_INIT,
    ATT_HDL_Wr_NULL,                                                    //ATT_HDL_CPS_CYCLING_POWER_CONTROL_POINT_CLIENT_CHARACTERISTIC_CONFIGURATION_INIT,
#endif
#ifdef _PROFILE_CSTM_UDF01S_
    ATT_HDL_Wr_NULL,                                                    //ATT_HDL_UDF01S_PRIMARY_SERVICE,
    ATT_HDL_Wr_NULL,                                                    //ATT_HDL_UDF01S_CHARACTERISTIC_UDATR01,
    ATT_HDL_Wr_NULL,                                                    //ATT_HDL_UDF01S_UDATR01,
    ATT_HDL_Wr_NULL,                                                    //ATT_HDL_UDF01S_CHARACTERISTIC_UDATN01,
    ATT_HDL_Wr_NULL,                                                    //ATT_HDL_UDF01S_UDATN01_INIT,
    ATT_HDL_Wr_NULL,
    ATT_HDL_Wr_NULL,                                                    //ATT_HDL_UDF01S_CHARACTERISTIC_UDATRW01,
    ATT_HDL_Wr_UDF01S_UDATRW01,                                         //ATT_HDL_UDF01S_UDATRW01_INIT,
#endif                                                //ATT_HDL_DIS_PNP_ID,
    ATT_HDL_Wr_NULL,                                                    //ATT_HDL_UDF01S_PRIMARY_SERVICE,
    ATT_HDL_Wr_NULL,                                                    //ATT_HDL_UDF01S_CHARACTERISTIC_UDATR01,
    ATT_HDL_Wr_NULL,                                                    //ATT_HDL_UDF01S_UDATR01,
    ATT_HDL_Wr_NULL,                                                    //ATT_HDL_UDF01S_CHARACTERISTIC_UDATN01,
    ATT_HDL_Wr_NULL,                                                    //ATT_HDL_UDF01S_UDATN01_INIT,
    ATT_HDL_Wr_NULL,
    ATT_HDL_Wr_NULL,                                                    //ATT_HDL_UDF01S_CHARACTERISTIC_UDATRW01,
    ATT_HDL_Wr_UDF_UDATRW01,                                         //ATT_HDL_UDF01S_UDATRW01_INIT,
#ifdef _PROFILE_CSTM_UDF02S_
    ATT_HDL_Wr_NULL,                                                    //ATT_HDL_UDF01S_PRIMARY_SERVICE,
    ATT_HDL_Wr_NULL,                                                    //ATT_HDL_UDF02S_CHARACTERISTIC_UDATR02,
    ATT_HDL_Wr_NULL,                                                    //ATT_HDL_UDF02S_UDATR02,
    ATT_HDL_Wr_NULL,                                                    //ATT_HDL_UDF02S_CHARACTERISTIC_UDATN02,
    ATT_HDL_Wr_NULL,                                                    //ATT_HDL_UDF02S_UDATN02_INIT,
    ATT_HDL_Wr_NULL,
    ATT_HDL_Wr_NULL,                                                    //ATT_HDL_UDF02S_CHARACTERISTIC_UDATRW02,
    ATT_HDL_Wr_UDF02S_UDATRW02,                                         //ATT_HDL_UDF02S_UDATRW02_INIT,
#endif
#ifdef _PROFILE_CSTM_UDF03S_
    ATT_HDL_Wr_NULL,                                                    //ATT_HDL_UDF01S_PRIMARY_SERVICE,
    ATT_HDL_Wr_NULL,                                                    //ATT_HDL_UDF03S_CHARACTERISTIC_UDATR03,
    ATT_HDL_Wr_NULL,                                                    //ATT_HDL_UDF03S_UDATR03,
    ATT_HDL_Wr_NULL,                                                    //ATT_HDL_UDF03S_CHARACTERISTIC_UDATN03,
    ATT_HDL_Wr_NULL,                                                    //ATT_HDL_UDF03S_UDATN03_INIT,
    ATT_HDL_Wr_NULL,
    ATT_HDL_Wr_NULL,                                                    //ATT_HDL_UDF03S_CHARACTERISTIC_UDATRW03,
    ATT_HDL_Wr_UDF03S_UDATRW03,                                         //ATT_HDL_UDF03S_UDATRW03_INIT,
#endif

#ifdef _PROFILE_CSTM_DATAEXCHANGE_S_
    ATT_HDL_Wr_NULL,
    ATT_HDL_Wr_NULL,
    ATT_HDL_Wr_DATAEXCHANGE_WRITE,
    ATT_HDL_Wr_NULL,
    ATT_HDL_Wr_NULL,
    ATT_HDL_Wr_NULL,
#endif


#ifdef _PROFILE_OTA_
    ATT_HDL_Wr_NULL,                                                    //ATT_HDL_OTA_PRIMARY_SERVICE,
    ATT_HDL_Wr_NULL,                                                    //ATT_HDL_OTA_CHARACTERISTIC_OTA_DATA,
    ATT_HDL_Wr_OTA_OTA_DATA,                                  //ATT_HDL_OTA_OTA_DATA_INIT,
    ATT_HDL_Wr_NULL,                                                    //ATT_HDL_OTA_OTA_DATA_CLIENT_CHARACTERISTIC_CONFIGURATION_INIT,
    ATT_HDL_Wr_NULL,                                                    //ATT_HDL_OTA_OTA_DATA_DESCRIPTION,
    ATT_HDL_Wr_NULL,                                                    //ATT_HDL_OTA_CHARACTERISTIC_OTA_CMD_CONTROL_POINT,
    ATT_HDL_Wr_OTA_OTA_CMD_CONTROL_POINT,        //ATT_HDL_OTA_OTA_CMD_CONTROL_POINT_INIT,
    ATT_HDL_Wr_NULL,                                                    //ATT_HDL_OTA_OTA_CMD_CONTROL_POINT_CLIENT_CHARACTERISTIC_CONFIGURATION_INIT,
    ATT_HDL_Wr_NULL,                                                    //ATT_HDL_OTA_OTA_CMD_USER_DESCRIPTION,
#endif


};


void ATT_HDL_Rd_NULL(uint8_t connID, uint8_t *srcCMD)
{
}

#ifdef _PROFILE_HRP_
void ATT_HDL_Rd_HRS_BODY_SENSOR_LOCATION(uint8_t connID, uint8_t *srcCMD)
{
    att_HDL_HRS_BODY_SENSOR_LOCATION[0]++;
    if (att_HDL_HRS_BODY_SENSOR_LOCATION[0] > 6)
    {
        att_HDL_HRS_BODY_SENSOR_LOCATION[0] = 0;
    }
}
#endif

#ifdef _PROFILE_CSTM_UDF01S_
/*!
    \brief  Calling fuction while attribute read related to att_HDL_UDF01S_UDATR01..
    \param  srcCMD address of conents writing to the GATT characteristic.
*/
void ATT_HDL_Rd_UDF01S_UDATR01(uint8_t connID, uint8_t *srcCMD)
{
}


/*!
    \brief  Calling fuction while attribute read related to att_HDL_UDF01S_UDATN01..
    \param  srcCMD address of conents writing to the GATT characteristic.
*/
void ATT_HDL_Rd_UDF01S_UDATN01(uint8_t connID, uint8_t *srcCMD)
{
}


/*!
    \brief  Calling fuction while attribute read related to att_HDL_UDF01S_UDATRW01..
    \param  srcCMD address of conents writing to the GATT characteristic.
*/
void ATT_HDL_Rd_UDF01S_UDATRW01(uint8_t connID, uint8_t *srcCMD)
{
}
#endif  //(#ifdef _PROFILE_CSTM_UDF01S_)
///////////////////////////////////////////////////////////////////////////////////////////////////////////////
void ATT_HDL_Rd_UDF_UDATR01(uint8_t connID, uint8_t *srcCMD)
{
}


/*!
    \brief  Calling fuction while attribute read related to att_HDL_UDF01S_UDATN01..
    \param  srcCMD address of conents writing to the GATT characteristic.
*/
void ATT_HDL_Rd_UDF_UDATN01(uint8_t connID, uint8_t *srcCMD)
{
}


/*!
    \brief  Calling fuction while attribute read related to att_HDL_UDF01S_UDATRW01..
    \param  srcCMD address of conents writing to the GATT characteristic.
*/
void ATT_HDL_Rd_UDF_UDATRW01(uint8_t connID, uint8_t *srcCMD)
{

}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////

#ifdef _PROFILE_CSTM_UDF02S_
void ATT_HDL_Rd_UDF02S_UDATR02(uint8_t connID, uint8_t *srcCMD)
{
}


void ATT_HDL_Rd_UDF02S_UDATN02(uint8_t connID, uint8_t *srcCMD)
{
}


void ATT_HDL_Rd_UDF02S_UDATRW02(uint8_t connID, uint8_t *srcCMD)
{
}
#endif  //(#ifdef _PROFILE_CSTM_UDF02S_)


#ifdef _PROFILE_CSTM_UDF03S_
void ATT_HDL_Rd_UDF03S_UDATR03(uint8_t connID, uint8_t *srcCMD)
{
}


void ATT_HDL_Rd_UDF03S_UDATN03(uint8_t connID, uint8_t *srcCMD)
{
}


void ATT_HDL_Rd_UDF03S_UDATRW03(uint8_t connID, uint8_t *srcCMD)
{
}
#endif  //(#ifdef _PROFILE_CSTM_UDF03S_)


/*!
    \brief  Attribute read related command will call the function
            corresponding to GATT characteristic.
    \param  length length of conents reading from the GATT characteristic.
    \param  srcCMD address of conents reading from the GATT characteristic.
*/
void (* const ATT_Read[])(uint8_t connID, uint8_t *srcCMD) =
{
    ATT_HDL_Rd_NULL,                                                    //ATT_HDL_INVALID,
    ATT_HDL_Rd_NULL,                                                    //ATT_HDL_GAP_PRIMARY_SERVICE,
    ATT_HDL_Rd_NULL,                                                    //ATT_HDL_GAP_CHARACTERISTIC_DEVICE_NAME,
    ATT_HDL_Rd_NULL,                                                    //ATT_HDL_GAP_DEVICE_NAME,
    ATT_HDL_Rd_NULL,                                                    //ATT_HDL_GAP_CHARACTERISTIC_APPEARANCE,
    ATT_HDL_Rd_NULL,                                                    //ATT_HDL_GAP_APPEARANCE,
    ATT_HDL_Rd_NULL,                                                    //ATT_HDL_GAP_CHARACTERISTIC_RECONNECTION_ADDRESS,
    ATT_HDL_Rd_NULL,                                                    //ATT_HDL_GAP_RECONNECTION_ADDRESS_INIT,
    ATT_HDL_Rd_NULL,                                                    //ATT_HDL_GAP_CHARACTERISTIC_PERIPHERAL_PRIVACY_FLAG,
    ATT_HDL_Rd_NULL,                                                    //ATT_HDL_GAP_PERIPHERAL_PRIVACY_FLAG,
    ATT_HDL_Rd_NULL,                                                    //ATT_HDL_GAP_CHARACTERISTIC_PERIPHERAL_PREFERRED_CONNECTION_PARAMETERS,
    ATT_HDL_Rd_NULL,                                                    //ATT_HDL_GAP_PERIPHERAL_PREFERRED_CONNECTION_PARAMETERS,
    ATT_HDL_Rd_NULL,                                                    //ATT_HDL_GATT_PRIMARY_SERVICE,
    ATT_HDL_Rd_NULL,                                                    //ATT_HDL_GATT_CHARACTERISTIC_SERVICE_CHANGED,
    ATT_HDL_Rd_NULL,                                                    //ATT_HDL_GATT_SERVICE_CHANGED_INIT,
    ATT_HDL_Rd_NULL,                                                    //ATT_HDL_GATT_CLIENT_CHARACTERISTIC_CONFIGURATION_INIT
    ATT_HDL_Rd_NULL,                                                    //ATT_HDL_DIS_PRIMARY_SERVICE,
    ATT_HDL_Rd_NULL,                                                    //ATT_HDL_DIS_CHARACTERISTIC_SERIAL_NUMBER_STRING,
    ATT_HDL_Rd_NULL,                                                    //ATT_HDL_DIS_SERIAL_NUMBER_STRING,
    ATT_HDL_Rd_NULL,                                                    //ATT_HDL_DIS_SERIAL_NUMBER_STRING_PRESENTATION_FORMAT,
    ATT_HDL_Rd_NULL,                                                    //ATT_HDL_DIS_CHARACTERISTIC_MANUFACTURER_NAME_STRING,
    ATT_HDL_Rd_NULL,                                                    //ATT_HDL_DIS_MANUFACTURER_NAME_STRING,
    ATT_HDL_Rd_NULL,                                                    //ATT_HDL_DIS_MANUFACTURER_NAME_STRING_PRESENTATION_FORMAT,
    ATT_HDL_Rd_NULL,                                                    //ATT_HDL_DIS_CHARACTERISTIC_SYSTEM_ID,
    ATT_HDL_Rd_NULL,                                                    //ATT_HDL_DIS_SYSTEM_ID,
    ATT_HDL_Rd_NULL,                                                    //ATT_HDL_DIS_CHARACTERISTIC_FIRMWARE_REVISION_STRING,
    ATT_HDL_Rd_NULL,                                                    //ATT_HDL_DIS_FIRMWARE_REVISION_STRING,
    ATT_HDL_Rd_NULL,                                                    //ATT_HDL_DIS_FIRMWARE_REVISION_STRING_PRESENTATION_FORMAT,
    ATT_HDL_Rd_NULL,                                                    //ATT_HDL_DIS_CHARACTERISTIC_MODEL_NUMBER_STRING,
    ATT_HDL_Rd_NULL,                                                    //ATT_HDL_DIS_MODEL_NUMBER_STRING,
    ATT_HDL_Rd_NULL,                                                    //ATT_HDL_DIS_MODEL_NUMBER_STRING_PRESENTATION_FORMAT,
    ATT_HDL_Rd_NULL,                                                    //ATT_HDL_DIS_CHARACTERISTIC_HARDWARE_REVISION_STRING,
    ATT_HDL_Rd_NULL,                                                    //ATT_HDL_DIS_HARDWARE_REVISION_STRING,
    ATT_HDL_Rd_NULL,                                                    //ATT_HDL_DIS_HARDWARE_REVISION_STRING_PRESENTATION_FORMAT,
    ATT_HDL_Rd_NULL,                                                    //ATT_HDL_DIS_CHARACTERISTIC_SOFTWARE_REVISION_STRING,
    ATT_HDL_Rd_NULL,                                                    //ATT_HDL_DIS_SOFTWARE_REVISION_STRING,
    ATT_HDL_Rd_NULL,                                                    //ATT_HDL_DIS_SOFTWARE_REVISION_STRING_PRESENTATION_FORMAT,
    ATT_HDL_Rd_NULL,                                                    //ATT_HDL_DIS_CHARACTERISTIC_PNP_ID,
    ATT_HDL_Rd_NULL,                                                    //ATT_HDL_DIS_PNP_ID,
#ifdef _PROFILE_HRP_
    ATT_HDL_Rd_NULL,                                                    //ATT_HDL_HRS_PRIMARY_SERVICE,
    ATT_HDL_Rd_NULL,                                                    //ATT_HDL_HRS_CHARACTERISTIC_HEART_RATE_MEASUREMENT,
    ATT_HDL_Rd_NULL,                                                    //ATT_HDL_HRS_HEART_RATE_MEASUREMENT_INIT,
    ATT_HDL_Rd_NULL,                                                    //ATT_HDL_HRS_CLIENT_CHARACTERISTIC_CONFIGURATION_INIT,
    ATT_HDL_Rd_NULL,                                                    //ATT_HDL_HRS_CHARACTERISTIC_BODY_SENSOR_LOCATION,
    ATT_HDL_Rd_HRS_BODY_SENSOR_LOCATION,                                //ATT_HDL_HRS_BODY_SENSOR_LOCATION_INIT,
#endif
#ifdef _PROFILE_BAS_
    ATT_HDL_Rd_NULL,                                                    //ATT_HDL_BAS_PRIMARY_SERVICE,
    ATT_HDL_Rd_NULL,                                                    //ATT_HDL_BAS_CHARACTERISTIC_BATTERY_LEVEL,
    ATT_HDL_Rd_NULL,                                                    //ATT_HDL_BAS_BATTERY_LEVEL_INIT,
    //                                                                  //ATT_HDL_BAS_BATTERY_LEVEL_PRESENTATION_FORMAT,
    ATT_HDL_Rd_NULL,                                                    //ATT_HDL_BAS_CLIENT_CHARACTERISTIC_CONFIGURATION_INIT
    ATT_HDL_Rd_NULL,                                                    //ATT_HDL_BAS_CHARACTERISTIC_BATTERY_POWER_STATE,
    ATT_HDL_Rd_NULL,                                                    //ATT_HDL_BAS_BATTERY_POWER_STATE_INIT
#endif
#ifdef _PROFILE_HOGP_
    ATT_HDL_Rd_NULL,
    ATT_HDL_Rd_NULL,
    ATT_HDL_Rd_NULL,
    ATT_HDL_Rd_NULL,
    ATT_HDL_Rd_NULL,
    ATT_HDL_Rd_NULL,
    ATT_HDL_Rd_NULL,
#ifdef _PROFILE_HOGP_KEYBOARD_
    ATT_HDL_Rd_NULL,
    ATT_HDL_Rd_NULL,
    ATT_HDL_Rd_NULL,
    ATT_HDL_Rd_NULL,
    ATT_HDL_Rd_NULL,
    ATT_HDL_Rd_NULL,
    ATT_HDL_Rd_NULL,
#endif
#ifdef _PROFILE_HOGP_MOUSE_
    ATT_HDL_Rd_NULL,
    ATT_HDL_Rd_NULL,
    ATT_HDL_Rd_NULL,
    ATT_HDL_Rd_NULL,
    ATT_HDL_Rd_NULL,
    ATT_HDL_Rd_NULL,
    ATT_HDL_Rd_NULL,
    ATT_HDL_Rd_NULL,
#endif
#ifdef _PROFILE_HOGP_COMSUMER_
    ATT_HDL_Rd_NULL,
    ATT_HDL_Rd_NULL,
    ATT_HDL_Rd_NULL,
    ATT_HDL_Rd_NULL,
#endif
#ifdef _PROFILE_HOGP_EMC_
    ATT_HDL_Rd_NULL,
    ATT_HDL_Rd_NULL,
    ATT_HDL_Rd_NULL,
    ATT_HDL_Rd_NULL,
    ATT_HDL_Rd_NULL,
    ATT_HDL_Rd_NULL,
    ATT_HDL_Rd_NULL,
    ATT_HDL_Rd_NULL,
    ATT_HDL_Rd_NULL,
    ATT_HDL_Rd_NULL,
    ATT_HDL_Rd_NULL,
    ATT_HDL_Rd_NULL,
    ATT_HDL_Rd_NULL,
    ATT_HDL_Rd_NULL,
    //
    ATT_HDL_Rd_NULL,
    ATT_HDL_Rd_NULL,
    ATT_HDL_Rd_NULL,
    //
    ATT_HDL_Rd_NULL,
    ATT_HDL_Rd_NULL,
    ATT_HDL_Rd_NULL,
    //
    ATT_HDL_Rd_NULL,
#endif
    ATT_HDL_Rd_NULL,
    ATT_HDL_Rd_NULL,
#ifdef _PROFILE_HOGP_BOOT_
#ifdef _PROFILE_HOGP_KEYBOARD_
    ATT_HDL_Rd_NULL,
    ATT_HDL_Rd_NULL,
    ATT_HDL_Rd_NULL,
    ATT_HDL_Rd_NULL,
    ATT_HDL_Rd_NULL,
#endif
#ifdef _PROFILE_HOGP_MOUSE_
    ATT_HDL_Rd_NULL,
    ATT_HDL_Rd_NULL,
    ATT_HDL_Rd_NULL,
#endif
#endif
#endif
#ifdef _PROFILE_SCPP_
    ATT_HDL_Rd_NULL,                                                    //ATT_HDL_SCPS_PRIMARY_SERVICE,
    ATT_HDL_Rd_NULL,                                                    //ATT_HDL_SCPS_CHARACTERISTIC_SCAN_INTERVAL_WINDOW,
    ATT_HDL_Rd_NULL,                                                    //ATT_HDL_SCPS_SCAN_INTERVAL_WINDOW_INIT,
    ATT_HDL_Rd_NULL,                                                    //ATT_HDL_SCPS_CHARACTERISTIC_SCAN_REFRESH,
    ATT_HDL_Rd_NULL,                                                    //ATT_HDL_SCPS_SCAN_REFRESH_INIT,
    ATT_HDL_Rd_NULL,                                                    //ATT_HDL_SCPS_SCAN_REFRESH_CLIENT_CHARACTERISTIC_CONFIGURATION_INIT
#endif
#ifdef _PROFILE_IAS_
    ATT_HDL_Rd_NULL,                                                    //ATT_HDL_IAS_PRIMARY_SERVICE,
    ATT_HDL_Rd_NULL,                                                    //ATT_HDL_IAS_CHARACTERISTIC_ALERT_LEVEL,
    ATT_HDL_Rd_NULL,                                                    //ATT_HDL_IAS_ALERT_LEVEL_INIT,
#endif
#ifdef _PROFILE_LLS_
    ATT_HDL_Rd_NULL,                                                    //ATT_HDL_LLS_PRIMARY_SERVICE,
    ATT_HDL_Rd_NULL,                                                    //ATT_HDL_LLS_CHARACTERISTIC_ALERT_LEVEL,
    ATT_HDL_Rd_NULL,                                                    //ATT_HDL_LLS_ALERT_LEVEL_INIT,
#endif
#ifdef _PROFILE_TPS_
    ATT_HDL_Rd_NULL,                                                    //ATT_HDL_TPS_PRIMARY_SERVICE,
    ATT_HDL_Rd_NULL,                                                    //ATT_HDL_TPS_CHARACTERISTIC_TX_POWER_LEVEL,
    ATT_HDL_Rd_NULL,                                                    //ATT_HDL_TPS_TX_POWER_LEVEL_INIT,
#endif
#ifdef _PROFILE_BLP_
    ATT_HDL_Rd_NULL,                                                    //ATT_HDL_BLS_PRIMARY_SERVICE,
    ATT_HDL_Rd_NULL,                                                    //ATT_HDL_BLS_CHARACTERISTIC_BLOOD_PRESSURE_MEASUREMENT,
    ATT_HDL_Rd_NULL,                                                    //ATT_HDL_BLS_BLOOD_PRESSURE_MEASUREMENT_INIT,
    ATT_HDL_Rd_NULL,                                                    //ATT_HDL_BLS_BLOOD_PRESSURE_CLIENT_CHARACTERISTIC_CONFIGURATION_INIT,
    ATT_HDL_Rd_NULL,                                                    //ATT_HDL_BLS_CHARACTERISTIC_BLOOD_PRESSURE_FEATURE,
    ATT_HDL_Rd_NULL,                                                    //ATT_HDL_BLS_BLOOD_PRESSURE_FEATURE_INIT,
#endif
#ifdef _PROFILE_HTP_
    ATT_HDL_Rd_NULL,                                                    //ATT_HDL_HTS_PRIMARY_SERVICE,
    ATT_HDL_Rd_NULL,                                                    //ATT_HDL_HTS_CHARACTERISTIC_TEMPERATURE_MEASUREMENT,
    ATT_HDL_Rd_NULL,                                                    //ATT_HDL_HTS_TEMPERATURE_MEASUREMENT_INIT,
    ATT_HDL_Rd_NULL,                                                    //ATT_HDL_HTS_TEMPERATURE_MEASUREMENT_CLIENT_CHARACTERISTIC_CONFIGURATION_INIT,
    ATT_HDL_Rd_NULL,                                                    //ATT_HDL_HTS_CHARACTERISTIC_TEMPERATURE_TYPE,
    ATT_HDL_Rd_NULL,                                                    //ATT_HDL_HTS_TEMPERATURE_TYPE_INIT,
#endif
#ifdef _PROFILE_RSCP_
    ATT_HDL_Rd_NULL,                                                    //ATT_HDL_RSCS_PRIMARY_SERVICE,
    ATT_HDL_Rd_NULL,                                                    //ATT_HDL_RSCS_CHARACTERISTIC_RSC_MEASUREMENT,
    ATT_HDL_Rd_NULL,                                                    //ATT_HDL_RSCS_RSC_MEASUREMENT_INIT,
    ATT_HDL_Rd_NULL,                                                    //ATT_HDL_RSCS_RSC_MEASUREMENT_CLIENT_CHARACTERISTIC_CONFIGURATION_INIT,
    ATT_HDL_Rd_NULL,                                                    //ATT_HDL_RSCS_CHARACTERISTIC_RSC_FEATURE,
    ATT_HDL_Rd_NULL,                                                    //ATT_HDL_RSCS_RSC_FEATURE_INIT,
    ATT_HDL_Rd_NULL,                                                    //ATT_HDL_RSCS_CHARACTERISTIC_SC_CONTROL_POINT,
    ATT_HDL_Rd_NULL,                                                    //ATT_HDL_RSCS_SC_CONTROL_POINT_INIT,
    ATT_HDL_Rd_NULL,                                                    //ATT_HDL_RSCS_SC_CONTROL_POINT_CLIENT_CHARACTERISTIC_CONFIGURATION_INIT,
#endif
#ifdef _PROFILE_CSCP_
    ATT_HDL_Rd_NULL,                                                    //ATT_HDL_CSCS_PRIMARY_SERVICE,
    ATT_HDL_Rd_NULL,                                                    //ATT_HDL_CSCS_CHARACTERISTIC_CSC_MEASUREMENT,
    ATT_HDL_Rd_NULL,                                                    //ATT_HDL_CSCS_CSC_MEASUREMENT_INIT,
    ATT_HDL_Rd_NULL,                                                    //ATT_HDL_CSCS_CSC_MEASUREMENT_CLIENT_CHARACTERISTIC_CONFIGURATION_INIT,
    ATT_HDL_Rd_NULL,                                                    //ATT_HDL_CSCS_CHARACTERISTIC_CSC_FEATURE,
    ATT_HDL_Rd_NULL,                                                    //ATT_HDL_CSCS_CSC_FEATURE_INIT,
    ATT_HDL_Rd_NULL,                                                    //ATT_HDL_CSCS_CHARACTERISTIC_SC_CONTROL_POINT,
    ATT_HDL_Rd_NULL,                                                    //ATT_HDL_CSCS_SC_CONTROL_POINT_INIT,
    ATT_HDL_Rd_NULL,                                                    //ATT_HDL_CSCS_SC_CONTROL_POINT_CLIENT_CHARACTERISTIC_CONFIGURATION_INIT,
#endif
#ifdef _PROFILE_LNS_
    ATT_HDL_Rd_NULL,                                                    //ATT_HDL_LNS_PRIMARY_SERVICE,
    ATT_HDL_Rd_NULL,                                                    //ATT_HDL_LNS_CHARACTERISTIC_LOCATION_AND_SPEED,
    ATT_HDL_Rd_NULL,                                                    //ATT_HDL_LNS_LOCATION_AND_SPEED,
    ATT_HDL_Rd_NULL,                                                    //ATT_HDL_LNS_LOCATION_AND_SPEED_CLIENT_CHARACTERISTIC_CONFIGURATION_INIT,
    ATT_HDL_Rd_NULL,                                                    //ATT_HDL_LNS_CHARACTERISTIC_LN_FEATURE,
    ATT_HDL_Rd_NULL,                                                    //ATT_HDL_LNS_LN_FEATURE,
    ATT_HDL_Rd_NULL,                                                    //ATT_HDL_LNS_CHARACTERISTIC_POSITION_QUALITY,
    ATT_HDL_Rd_NULL,                                                    //ATT_HDL_LNS_POSITION_QUALITY,
    ATT_HDL_Rd_NULL,                                                    //ATT_HDL_LNS_CHARACTERISTIC_LN_CONTROL_POINT,
    ATT_HDL_Rd_NULL,                                                    //ATT_HDL_LNS_LN_CONTROL_POINT_INIT,
    ATT_HDL_Rd_NULL,                                                    //ATT_HDL_LNS_LN_CONTROL_POINT_CLIENT_CHARACTERISTIC_CONFIGURATION_INIT,
    ATT_HDL_Rd_NULL,                                                    //ATT_HDL_LNS_CHARACTERISTIC_NAVIGATION,
    ATT_HDL_Rd_NULL,                                                    //ATT_HDL_LNS_NAVIGATION,
    ATT_HDL_Rd_NULL,                                                    //ATT_HDL_LNS_NAVIGATION_CLIENT_CHARACTERISTIC_CONFIGURATION_INIT,
#endif
#ifdef _PROFILE_GLS_
    ATT_HDL_Rd_NULL,                                                    //ATT_HDL_GLS_PRIMARY_SERVICE,
    ATT_HDL_Rd_NULL,                                                    //ATT_HDL_GLS_CHARACTERISTIC_GLUCOSE_MEASUREMENT,
    ATT_HDL_Rd_NULL,                                                    //ATT_HDL_GLS_GLUCOSE_MEASUREMENT,
    ATT_HDL_Rd_NULL,                                                    //ATT_HDL_GLS_GLUCOSE_MEASUREMENT_CLIENT_CHARACTERISTIC_CONFIGURATION_INIT,
    ATT_HDL_Rd_NULL,                                                    //ATT_HDL_GLS_CHARACTERISTIC_GLUCOSE_MEASUREMENT_CONTEXT,
    ATT_HDL_Rd_NULL,                                                    //ATT_HDL_GLS_GLUCOSE_MEASUREMENT_CONTEXT,
    ATT_HDL_Rd_NULL,                                                    //ATT_HDL_GLS_GLUCOSE_MEASUREMENT_CONTEXT_CLIENT_CHARACTERISTIC_CONFIGURATION_INIT,
    ATT_HDL_Rd_NULL,                                                    //ATT_HDL_GLS_CHARACTERISTIC_GLUCOSE_FEATURE,
    ATT_HDL_Rd_NULL,                                                    //ATT_HDL_GLS_GLUCOSE_FEATURE,
    ATT_HDL_Rd_NULL,                                                    //ATT_HDL_GLS_CHARACTERISTIC_RECORD_ACCESS_CONTROL_POINT,
    ATT_HDL_Rd_NULL,                                                    //ATT_HDL_GLS_RECORD_ACCESS_CONTROL_POINT_INIT,
    ATT_HDL_Rd_NULL,                                                    //ATT_HDL_GLS_RECORD_ACCESS_CONTROL_POINT_CLIENT_CHARACTERISTIC_CONFIGURATION_INIT,
#endif
#ifdef _PROFILE_CPS_
    ATT_HDL_Rd_NULL,                                                    //ATT_HDL_CPS_PRIMARY_SERVICE,
    ATT_HDL_Rd_NULL,                                                    //ATT_HDL_CPS_CHARACTERISTIC_CYCLING_POWER_MEASUREMENT,
    ATT_HDL_Rd_NULL,                                                    //ATT_HDL_CPS_CYCLING_POWER_MEASUREMENT,
    ATT_HDL_Rd_NULL,                                                    //ATT_HDL_CPS_CYCLING_POWER_MEASUREMENT_CLIENT_CHARACTERISTIC_CONFIGURATION_INIT,
    ATT_HDL_Rd_NULL,                                                    //ATT_HDL_CPS_CYCLING_POWER_MEASUREMENT_SERVER_CHARACTERISTIC_CONFIGURATION_INIT,
    ATT_HDL_Rd_NULL,                                                    //ATT_HDL_CPS_CHARACTERISTIC_CYCLING_POWER_FEATURE,
    ATT_HDL_Rd_NULL,                                                    //ATT_HDL_CPS_CYCLING_POWER_FEATURE,
    ATT_HDL_Rd_NULL,                                                    //ATT_HDL_CPS_CHARACTERISTIC_SENSOR_LOCATION,
    ATT_HDL_Rd_NULL,                                                    //ATT_HDL_CPS_SENSOR_LOCATION,
    ATT_HDL_Rd_NULL,                                                    //ATT_HDL_CPS_CHARACTERISTIC_CYCLING_POWER_VECTOR,
    ATT_HDL_Rd_NULL,                                                    //ATT_HDL_CPS_CYCLING_POWER_VECTOR,
    ATT_HDL_Rd_NULL,                                                    //ATT_HDL_CPS_CYCLING_POWER_VECTOR_CLIENT_CHARACTERISTIC_CONFIGURATION_INIT,
    ATT_HDL_Rd_NULL,                                                    //ATT_HDL_CPS_CHARACTERISTIC_CYCLING_POWER_CONTROL_POINT,
    ATT_HDL_Rd_NULL,                                                    //ATT_HDL_CPS_CYCLING_POWER_CONTROL_POINT_INIT,
    ATT_HDL_Rd_NULL,                                                    //ATT_HDL_CPS_CYCLING_POWER_CONTROL_POINT_CLIENT_CHARACTERISTIC_CONFIGURATION_INIT,
#endif
#ifdef _PROFILE_CSTM_UDF01S_
    ATT_HDL_Rd_NULL,                                                    //ATT_HDL_UDF01S_PRIMARY_SERVICE,
    ATT_HDL_Rd_NULL,                                                    //ATT_HDL_UDF01S_CHARACTERISTIC_UDATR01,
    ATT_HDL_Rd_UDF01S_UDATR01,                                          //ATT_HDL_UDF01S_UDATR01,
    ATT_HDL_Rd_NULL,                                                    //ATT_HDL_UDF01S_CHARACTERISTIC_UDATN01,
    ATT_HDL_Rd_UDF01S_UDATN01,                                          //ATT_HDL_UDF01S_UDATN01_INIT,
    ATT_HDL_Rd_NULL,
    ATT_HDL_Rd_NULL,                                                    //ATT_HDL_UDF01S_CHARACTERISTIC_UDATRW01,
    ATT_HDL_Rd_UDF01S_UDATRW01,                                         //ATT_HDL_UDF01S_UDATRW01_INIT,
#endif
    ATT_HDL_Rd_NULL,                                                    //ATT_HDL_UDF01S_PRIMARY_SERVICE,
    ATT_HDL_Rd_NULL,                                                    //ATT_HDL_UDF01S_CHARACTERISTIC_UDATR01,
    ATT_HDL_Rd_UDF_UDATR01,                                          //ATT_HDL_UDF01S_UDATR01,
    ATT_HDL_Rd_NULL,                                                    //ATT_HDL_UDF01S_CHARACTERISTIC_UDATN01,
    ATT_HDL_Rd_UDF_UDATN01,                                          //ATT_HDL_UDF01S_UDATN01_INIT,
    ATT_HDL_Rd_NULL,
    ATT_HDL_Rd_NULL,                                                    //ATT_HDL_UDF01S_CHARACTERISTIC_UDATRW01,
    ATT_HDL_Rd_UDF_UDATRW01,                                         //ATT_HDL_UDF01S_UDATRW01_INIT,
#ifdef _PROFILE_CSTM_UDF02S_
    ATT_HDL_Rd_NULL,                                                    //ATT_HDL_UDF01S_PRIMARY_SERVICE,
    ATT_HDL_Rd_NULL,                                                    //ATT_HDL_UDF02S_CHARACTERISTIC_UDATR02,
    ATT_HDL_Rd_UDF02S_UDATR02,                                          //ATT_HDL_UDF02S_UDATR02,
    ATT_HDL_Rd_NULL,                                                    //ATT_HDL_UDF02S_CHARACTERISTIC_UDATN02,
    ATT_HDL_Rd_UDF02S_UDATN02,                                          //ATT_HDL_UDF02S_UDATN02_INIT,
    ATT_HDL_Rd_NULL,
    ATT_HDL_Rd_NULL,                                                    //ATT_HDL_UDF02S_CHARACTERISTIC_UDATRW02,
    ATT_HDL_Rd_UDF02S_UDATRW02,                                         //ATT_HDL_UDF02S_UDATRW02_INIT,
#endif
#ifdef _PROFILE_CSTM_UDF03S_
    ATT_HDL_Rd_NULL,                                                    //ATT_HDL_UDF01S_PRIMARY_SERVICE,
    ATT_HDL_Rd_NULL,                                                    //ATT_HDL_UDF03S_CHARACTERISTIC_UDATR03,
    ATT_HDL_Rd_UDF03S_UDATR03,                                          //ATT_HDL_UDF03S_UDATR03,
    ATT_HDL_Rd_NULL,                                                    //ATT_HDL_UDF03S_CHARACTERISTIC_UDATN03,
    ATT_HDL_Rd_UDF03S_UDATN03,                                          //ATT_HDL_UDF03S_UDATN03_INIT,
    ATT_HDL_Rd_NULL,
    ATT_HDL_Rd_NULL,                                                    //ATT_HDL_UDF03S_CHARACTERISTIC_UDATRW03,
    ATT_HDL_Rd_UDF03S_UDATRW03,                                         //ATT_HDL_UDF03S_UDATRW03_INIT,
#endif

#ifdef _PROFILE_CSTM_DATAEXCHANGE_S_
    ATT_HDL_Rd_NULL,
    ATT_HDL_Rd_NULL,
    ATT_HDL_Rd_NULL,
    ATT_HDL_Rd_NULL,
    ATT_HDL_Rd_NULL,
    ATT_HDL_Rd_NULL,
#endif


#ifdef _PROFILE_OTA_
    ATT_HDL_Rd_NULL,                                                    //ATT_HDL_OTA_PRIMARY_SERVICE,
    ATT_HDL_Rd_NULL,                                                    //ATT_HDL_OTA_CHARACTERISTIC_OTA_DATA,
    ATT_HDL_Rd_NULL,                                                    //ATT_HDL_OTA_OTA_DATA_INIT,
    ATT_HDL_Rd_NULL,                                                    //ATT_HDL_OTA_OTA_DATA_CLIENT_CHARACTERISTIC_CONFIGURATION_INIT,
    ATT_HDL_Rd_NULL,                                                    //ATT_HDL_OTA_OTA_DATA_DESCRIPTION,
    ATT_HDL_Rd_NULL,                                                    //ATT_HDL_OTA_CHARACTERISTIC_OTA_CMD_CONTROL_POINT,
    ATT_HDL_Rd_NULL,                                                    //ATT_HDL_OTA_OTA_CMD_CONTROL_POINT_INIT,
    ATT_HDL_Rd_NULL,                                                    //ATT_HDL_OTA_OTA_CMD_CONTROL_POINT_CLIENT_CHARACTERISTIC_CONFIGURATION_INIT,
    ATT_HDL_Rd_NULL,                                                    //ATT_HDL_OTA_OTA_CMD_USER_DESCRIPTION,
#endif

};

Uint8 size_ATTRIBUTE_SERVER = (sizeof(ATTRIBUTE_SERVER) / sizeof(ATTRIBUTE_SERVER[0]));

#ifdef _SMP_ON_
#ifdef _BOND_ON_
const Uint8 *ATTRIBUTE_SERVER_BOND[SIZE_ATTRIBUTE_SERVER_BOND] =
{
    ATT_HDL_GAP_DEVICE_NAME_INIT,
#ifdef _PROFILE_BAS_
    ATT_HDL_BAS_CLIENT_CHARACTERISTIC_CONFIGURATION_INIT,
#endif

#ifdef _PROFILE_HOGP_
#ifdef _PROFILE_HOGP_COMSUMER_
    ATT_HDL_HIDS_REPORT_CSI_CLIENT_CHARACTERISTIC_CONFIGURATION_INIT,
#endif
#ifdef _PROFILE_HOGP_MOUSE_
    ATT_HDL_HIDS_REPORT_MSI_CLIENT_CHARACTERISTIC_CONFIGURATION_INIT,
    ATT_HDL_HIDS_REPORT_MSF_INIT,
#endif
#ifdef _PROFILE_HOGP_KEYBOARD_
    ATT_HDL_HIDS_REPORT_KBI_CLIENT_CHARACTERISTIC_CONFIGURATION_INIT,
#endif
#ifdef _PROFILE_HOGP_BOOT_
#ifdef _PROFILE_HOGP_KEYBOARD_
    ATT_HDL_HIDS_BOOT_KEYBOARD_INPUT_REPORT_CLIENT_CHARACTERISTIC_CONFIGURATION_INIT,
#endif
#ifdef _PROFILE_HOGP_MOUSE_
    ATT_HDL_HIDS_BOOT_MOUSE_INPUT_REPORT_CLIENT_CHARACTERISTIC_CONFIGURATION_INIT,
#endif
#endif
#endif  //#ifdef _PROFILE_HOGP_

#ifdef _PROFILE_GLS_
    ATT_HDL_GLS_GLUCOSE_MEASUREMENT_CLIENT_CHARACTERISTIC_CONFIGURATION_INIT,
    ATT_HDL_GLS_GLUCOSE_MEASUREMENT_CONTEXT_CLIENT_CHARACTERISTIC_CONFIGURATION_INIT,
    ATT_HDL_GLS_RECORD_ACCESS_CONTROL_POINT_CLIENT_CHARACTERISTIC_CONFIGURATION_INIT,
#endif
};
const Uint8 *ATTRIBUTE_SERVER_BOND_PARAM[SIZE_ATTRIBUTE_SERVER_BOND] =
{
    att_HDL_GAP_DEVICE_NAME,
#ifdef _PROFILE_BAS_
    att_HDL_BAS_CLIENT_CHARACTERISTIC_CONFIGURATION,
#endif
#ifdef _PROFILE_HOGP_
#ifdef _PROFILE_HOGP_COMSUMER_
    att_HDL_HIDS_REPORT_CSI_CLIENT_CHARACTERISTIC_CONFIGURATION,
#endif
#ifdef _PROFILE_HOGP_MOUSE_
    att_HDL_HIDS_REPORT_MSI_CLIENT_CHARACTERISTIC_CONFIGURATION,
    att_HDL_HIDS_REPORT_MSF,
#endif
#ifdef _PROFILE_HOGP_KEYBOARD_
    att_HDL_HIDS_REPORT_KBI_CLIENT_CHARACTERISTIC_CONFIGURATION,
#endif
#ifdef _PROFILE_HOGP_BOOT_
#ifdef _PROFILE_HOGP_KEYBOARD_
    att_HDL_HIDS_BOOT_KEYBOARD_INPUT_REPORT_CLIENT_CHARACTERISTIC_CONFIGURATION,
#endif
#ifdef _PROFILE_HOGP_MOUSE_
    att_HDL_HIDS_BOOT_MOUSE_INPUT_REPORT_CLIENT_CHARACTERISTIC_CONFIGURATION,
#endif
#endif
#endif  //#ifdef _PROFILE_HOGP
#ifdef _PROFILE_GLS_
    att_HDL_GLS_GLUCOSE_MEASUREMENT_CLIENT_CHARACTERISTIC_CONFIGURATION,
    att_HDL_GLS_GLUCOSE_MEASUREMENT_CONTEXT_CLIENT_CHARACTERISTIC_CONFIGURATION,
    att_HDL_GLS_RECORD_ACCESS_CONTROL_POINT_CLIENT_CHARACTERISTIC_CONFIGURATION,
#endif
};
Uint8 size_ATTRIBUTE_SERVER_BOND = (sizeof(ATTRIBUTE_SERVER_BOND) / sizeof(ATTRIBUTE_SERVER_BOND[0]));
#endif
#endif

#endif
