/**************************************************************************//**
* @file       ble_serviceDef.h
* @brief      Provide the Definition of BLE Services.
*
* @defgroup ble_service BLE Services Definition
* @{
* @details Here shows the definition in ble_serviceDef.h.
* @ingroup host
* @}
*****************************************************************************/

#ifndef _BLE_SERVICEDEF_H_
#define _BLE_SERVICEDEF_H_

/**
 * @defgroup bleBLP Blood Pressure Service
 * @{
 * @ingroup ble_service
 * @details Blood Pressure service.
 * @}
 * @defgroup bleBLP_feature Blood Pressure Service Feature
 * @{
 * @ingroup bleBLP
 * @details Blood Pressure service feature.
 */
#ifdef _PROFILE_BLP_

//Bluetooth "Blood Pressure Service" Spec. Page 14 of 18
#define BLS_FEAT_BODY_MOVEMENT_DETECTION                            0x01  /**< Body Movement Detection. */
#define BLS_FEAT_CUFF_FIT_DETECTION                                 0x02  /**< Cuff Fit Detection. */
#define BLS_FEAT_IRREGULAR_PULSE_DETECTION                          0x04  /**< Irregular Pulse Detection. */
#define BLS_FEAT_PULSE_RATE_RANGE_DETECTION                         0x08  /**< Pulse Rate Range Detection. */
#define BLS_FEAT_MEASUREMENT_POSITION_DETECTION                     0x10  /**< Measurement Position Detection. */
#define BLS_FEAT_MULTIPLE_BOND                                      0x20  /**< Multiple Bond. */
/** @} */

/**
 * @defgroup bleBLP_measurement Blood Pressure Service Measurement
 * @{
 * @ingroup bleBLP
 * @details Blood Pressure service measurement.
 */
//Bluetooth "Blood Pressure Service" Spec. Page 10 of 18
#define BLS_MEAS_BLOOD_PRESSURE_UNITS_KPA                           0x01  /**< Blood Pressure in Unitsof kPa. */
#define BLS_MEAS_TIME_STAMP                                         0x02  /**< Time Stamp. */
#define BLS_MEAS_PULSE_RATE                                         0x04  /**< Pulse Rate Flag. */
#define BLS_MEAS_USER_ID                                            0x08  /**< User ID Flag. */
#define BLS_MEAS_MEASUREMENT_STATUS                                 0x10  /**< Measurement Status. */
#endif // _PROFILE_BLP_
/** @} */


/**
 * @defgroup bleHTP Health Thermometer Service
 * @{
 * @ingroup ble_service
 * @details Health Thermometer service.
 * @}
 * @defgroup bleHTP_measurement Health Thermometer Service Measurement
 * @{
 * @ingroup bleHTP
 * @details Health Thermometer service measurement.
 */
#ifdef _PROFILE_HTP_
//Bluetooth "Health Thermometer Service" Spec. Page 10 of 18
#define HTS_MEAS_TEMPERATURE_MEASUREMENT_VALUE_FAHRENHEIT           0x01  /**< Temperature is in Fahrenheit units. */
#define HTS_MEAS_TIME_STAMP                                         0x02  /**< Time Stamp is present. */
#define HTS_MEAS_TEMPERATURE_TYPE                                   0x04  /**< Temperature Type is present. */
#endif // _PROFILE_HTP_
/** @} */



/**
 * @def _PROFILE_RSCP_
 * @defgroup bleRSCP Running Speed and Cadence Service
 * @{
 * @ingroup ble_service
 * @details Running Speed and Cadence service.
 * @}
 * @defgroup bleRSCP_feature Running Speed and Cadence Service Feature
 * @{
 * @ingroup bleRSCP
 * @details Running Speed and Cadence service feature.
 */

#ifdef _PROFILE_RSCP_
// Running Speed and Cadence Service Feature
//Bluetooth "Running Speed and Cadence Service" Spec. Page 10 of 19
#define RSCS_FEAT_INSTANTANEOUS_STRIDE_LENGTH_MEASUREMENT_SUPPORTED 0x01  /**< Instantaneous Stride Length Measurement. */
#define RSCS_FEAT_TOTAL_DISTANCE_MEASUREMENT_SUPPORTED              0x02  /**< Total Distance Measurement. */
#define RSCS_FEAT_WALKING_OR_RUNNING_STATUS_SUPPORTED               0x04  /**< Walking or Running Status. */
#define RSCS_FEAT_CALIBRATION_PROCEDURE_NOT_SUPPORTED               0x08  /**< Calibration Procedure. */
#define RSCS_FEAT_MULTIPLE_SENSOR_LOCATIONS_NOT_SUPPORTED           0x10  /**< Multiple Sensor Locations. */
/** @} */


/**
 * @defgroup bleRSCP_measurement Running Speed and Cadence Service Measurement
 * @{
 * @ingroup bleRSCP
 * @details Running Speed and Cadence service measurement.
 */
// Running Speed and Cadence Service measurement
#define RSCS_MEAS_INSTANTANEOUS_STRIDE_LENGTH_PRESENT               0x01  /**< Instantaneous Stride Length is present in the measurement. */
#define RSCS_MEAS_TOTAL_DISTANCE_PRESENT                            0x02  /**< Total Distance is present in the measurement. */
#define RSCS_MEAS_WALKING_OR_RUNNING_STATUS_BITS                    0x04  /**< Running or Walking. */
/** @} */


/**
 * @defgroup bleRSCP_flag Running Speed and Cadence Service Measurement Flag
 * @{
 * @ingroup bleRSCP
 * @details Running Speed and Cadence service measurement flag.
 */
// Running Speed and Cadence Service measurement
#define TAB_RSCS_MEAS_INSTANTANEOUS_FLAGS                                   0x00  /**< The Flag of Low Byte of Instantaneous Measurement. */
#define TAB_RSCS_MEAS_INSTANTANEOUS_SPEED_L                                 0x01  /**< Low Byte of Instantaneous Speed. */
#define TAB_RSCS_MEAS_INSTANTANEOUS_SPEED_H                                 0x02  /**< High Byte of Instantaneous Speed. */
#define TAB_RSCS_MEAS_INSTANTANEOUS_CADENCE                                 0x03  /**< Instantaneous Cadence. */
#define TAB_RSCS_MEAS_INSTANTANEOUS_STRIDE_LENGTH_L                         0x04  /**< Low Byte of Instantaneous Stride Length. */
#define TAB_RSCS_MEAS_INSTANTANEOUS_STRIDE_LENGTH_H                         0x05  /**< High Byte of Instantaneous Stride Length. */
#define TAB_RSCS_MEAS_TOTAL_DISTANCE_0                                      0x06  /**< Total Distance is 4-byte value. Total Distance Byte[0] */
#define TAB_RSCS_MEAS_TOTAL_DISTANCE_1                                      0x07  /**< Total Distance is 4-byte value. Total Distance Byte[1] */
#define TAB_RSCS_MEAS_TOTAL_DISTANCE_2                                      0x08  /**< Total Distance is 4-byte value. Total Distance Byte[2] */
#define TAB_RSCS_MEAS_TOTAL_DISTANCE_3                                      0x09  /**< Total Distance is 4-byte value. Total Distance Byte[3] */
/** @} */


/**
 * @defgroup bleRSCP_controlPoint Running Speed and Cadence Service Control Point
 * @{
 * @ingroup bleRSCP
 * @details Running Speed and Cadence service control point.
 */
#define TAB_RSCS_SC_CONTROL_POINT_OP_CODE                                   0x00  /**< Control point of OP code. */
#define TAB_RSCS_SC_CONTROL_POINT_TOTAL_DISTANCE_B0                         0x01  /**< Control point of total distance of Byte[0]. */
#define TAB_RSCS_SC_CONTROL_POINT_TOTAL_DISTANCE_B1                         0x02  /**< Control point of total distance of Byte[1]. */
#define TAB_RSCS_SC_CONTROL_POINT_TOTAL_DISTANCE_B2                         0x03  /**< Control point of total distance of Byte[2]. */
#define TAB_RSCS_SC_CONTROL_POINT_TOTAL_DISTANCE_B3                         0x04  /**< Control point of total distance of Byte[3]. */
#define TAB_RSCS_SC_CONTROL_POINT_REQUEST_OP_CODE                           0x01  /**< Control point of request OP code. */
#define TAB_RSCS_SC_CONTROL_POINT_RESPONSE_VALUE                            0x02  /**< Control point of response value. */


#define RSCS_SC_CONTROL_POINT_OP_CODE_SET_CUMULATIVE_VALUE                  0x01  /**< Set a cumulative (total) distance. */
#define RSCS_SC_CONTROL_POINT_OP_CODE_START_SENSOR_CALIBRATION              0x02  /**< Start the remote sensor calibration. */
#define RSCS_SC_CONTROL_POINT_OP_CODE_SET_CUMULATIVE_VALUE                  0x01  /**< Start the remote sensor calibration. */
#define RSCS_SC_CONTROL_POINT_OP_CODE_UPDATE_SENSOR_LOCATION                0x03  /**< set a new sensor location. */
#define RSCS_SC_CONTROL_POINT_OP_CODE_REQUEST_SUPPORTED_SENSOR_LOCATIONS    0x04  /**< Read the supported sensor location. */
#define RSCS_SC_CONTROL_POINT_OP_CODE_RESPONSE_CODE                         0x10  /**< Response code. */
/** @} */


/**
 * @defgroup bleRSCP_rsp Running Speed and Cadence Service Response
 * @{
 * @ingroup bleRSCP
 * @details Running Speed and Cadence service response.
 */
#define RSCS_SC_CONTROL_POINT_RESPONSE_VALUE_SUCCESS                        0x01  /**< Success response. */
#define RSCS_SC_CONTROL_POINT_RESPONSE_VALUE_OP_CODE_NOT_SUPPORTED          0x02  /**< OP code not supported response. */
#define RSCS_SC_CONTROL_POINT_RESPONSE_VALUE_INVALID_PARAMETER              0x03  /**< Invalid parameter response. */
#define RSCS_SC_CONTROL_POINT_RESPONSE_VALUE_OPERATION_FAILED               0x04  /**< Operation Failed response. */

#endif // _PROFILE_RSCP_
/** @} */



/**
 * @defgroup bleCSCP Cycling Speed and Cadence Service
 * @{
 * @ingroup ble_service
 * @details Cycling Speed and Cadence service.
 */
#ifdef _PROFILE_CSCP_
// Cycling Speed and Cadence Service Feature
#define CSCS_FEAT_WHEEL_REVOLUTION_DATA_SUPPORTED                   0x01  /**< Wheel Revolution Data. */
#define CSCS_FEAT_CRANK_REVOLUTION_DATA_SUPPORTED                   0x02  /**< Crank Revolution Data. */
#define CSCS_FEAT_MULTIPLE_SENSOR_LOCATIONS_SUPPORTED               0x04  /**< Multiple Sensor Locations. */

// Cycling Speed and Cadence Service measurement
#define CSCS_MEAS_WHEEL_REVOLUTION_DATA_PRESENT                     0x01  /**< Wheel Revolution Data is present in the measurement. */
#define CSCS_MEAS_CRANK_REVOLUTION_DATA_PRESENT                     0x02  /**< Crank Revolution Data is present in the measurement. */
#endif // _PROFILE_CSCP_
/** @} */




/**
 * @defgroup bleLNS Location and Navigation Service
 * @{
 * @ingroup ble_service
 * @details Location and Navigation service.
 */
// Location and Navigation Service Feature
#ifdef _PROFILE_LNS_

#define LNS_LN_FEAT_INSTANTANEOUS_SPEED_SUPPORTED                   0x01  /**< Instaneous Speed. */
#define LNS_LN_FEAT_TOTAL_DISTANCE_SUPPORTED                        0x02  /**< Total Distance Supported bit. */
#define LNS_LN_FEAT_LOCATION_SUPPORTED                              0x04  /**< Location. */
#define LNS_LN_FEAT_ELEVATION_SUPPORTED                             0x08  /**< Elevation. */
#define LNS_LN_FEAT_HEADING_SUPPORTED                               0x10  /**< Heading. */
#define LNS_LN_FEAT_ROLLING_TIME_SUPPORTED                          0x20  /**< Rolling Time. */
#define LNS_LN_FEAT_UTC_TIME_SUPPORTED                              0x40  /**< UTC Time. */
#define LNS_LN_FEAT_REMAINING_DISTANCE_SUPPORTED                    0x80  /**< Remaining Distance. */
#define LNS_LN_FEAT_REMAINING_VERTICAL_DISTANCE_SUPPORTED           0x01  /**< Remaining Vertical Distance. */
#define LNS_LN_FEAT_ESTIMATED_TIME_OF_ARRIVAL_SUPPORTED             0x02  /**< Estimated Time of Arrival. */
#define LNS_LN_FEAT_NUM_OF_BEACONS_IN_SOLUTION_SUPPORTED            0x04  /**< Number of Satellites in Solution. */
#define LNS_LN_FEAT_NUM_OF_BEACONS_IN_VIEW_SUPPORTED                0x08  /**< Number of Satellites in View. */
#define LNS_LN_FEAT_TIME_TO_FIRST_FIX_SUPPORTED                     0x10  /**< Time to First Fix. */
#define LNS_LN_FEAT_ESTIM_HORIZONTAL_POSITION_ERROR_SUPPORTED       0x20  /**< Estimated Horizontal Position Error. */
#define LNS_LN_FEAT_ESTIM_VERTICAL_POSITION_ERROR_SUPPORTED         0x40  /**< Estimated Vertical Position Error. */
#define LNS_LN_FEAT_HORIZONTAL_DILUTION_OF_PRECISION_SUPPORTED      0x80  /**< Horizontal Dilution of Precision. */
#define LNS_LN_FEAT_VERTICAL_DILUTION_OF_PRECISION_SUPPORTED        0x01  /**< Vertical Dilution of Precision. */
#define LNS_LN_FEAT_LOCAT_AND_SPD_CHARCT_CONTENT_MASKING_SUPPORTED  0x02  /**< Location and Speed Characteristic Content Masking. */
#define LNS_LN_FEAT_FIX_RATE_SETTING_SUPPORTED                      0x04  /**< Fix Rate Setting. */
#define LNS_LN_FEAT_ELEVATION_SETTING_SUPPORTED                     0x08  /**< Elevation Setting. */
#define LNS_LN_FEAT_POSITION_STATUS_SUPPORTED                       0x10  /**< Position Status. */

#define TAB_LNS_LOCATION_AND_SPEED_FLAGS_L                          0x00
#define TAB_LNS_LOCATION_AND_SPEED_FLAGS_H                          0x01
#define TAB_LNS_LOCATION_AND_SPEED_INSTANTANEOUS_SPEED_L            0x02
#define TAB_LNS_LOCATION_AND_SPEED_INSTANTANEOUS_SPEED_H            0x03
#define TAB_LNS_LOCATION_AND_SPEED_TOTAL_DISTANCE_0                 0x04
#define TAB_LNS_LOCATION_AND_SPEED_TOTAL_DISTANCE_1                 0x05
#define TAB_LNS_LOCATION_AND_SPEED_TOTAL_DISTANCE_2                 0x06
#define TAB_LNS_LOCATION_AND_SPEED_LOCATION_LATITUDE_0              0x07
#define TAB_LNS_LOCATION_AND_SPEED_LOCATION_LATITUDE_1              0x08
#define TAB_LNS_LOCATION_AND_SPEED_LOCATION_LATITUDE_2              0x09
#define TAB_LNS_LOCATION_AND_SPEED_LOCATION_LATITUDE_3              0x0A
#define TAB_LNS_LOCATION_AND_SPEED_LOCATION_LONGITUDE_0             0x0B
#define TAB_LNS_LOCATION_AND_SPEED_LOCATION_LONGITUDE_1             0x0C
#define TAB_LNS_LOCATION_AND_SPEED_LOCATION_LONGITUDE_2             0x0D
#define TAB_LNS_LOCATION_AND_SPEED_LOCATION_LONGITUDE_3             0x0E
#define TAB_LNS_LOCATION_AND_SPEED_ELEVATION_0                      0x02
#define TAB_LNS_LOCATION_AND_SPEED_ELEVATION_1                      0x03
#define TAB_LNS_LOCATION_AND_SPEED_ELEVATION_2                      0x04
#define TAB_LNS_LOCATION_AND_SPEED_HEADING_L                        0x05
#define TAB_LNS_LOCATION_AND_SPEED_HEADING_H                        0x06
#define TAB_LNS_LOCATION_AND_SPEED_ROLLING_TIME                     0x07
#define TAB_LNS_LOCATION_AND_SPEED_UTC_TIME_YEAR_L                  0x08
#define TAB_LNS_LOCATION_AND_SPEED_UTC_TIME_YEAR_H                  0x09
#define TAB_LNS_LOCATION_AND_SPEED_UTC_TIME_MONTH                   0x0A
#define TAB_LNS_LOCATION_AND_SPEED_UTC_TIME_DAY                     0x0B
#define TAB_LNS_LOCATION_AND_SPEED_UTC_TIME_HOURS                   0x0C
#define TAB_LNS_LOCATION_AND_SPEED_UTC_TIME_MINUTES                 0x0D
#define TAB_LNS_LOCATION_AND_SPEED_UTC_TIME_SECONDS                 0x0E

#define TAB_LNS_NAVIGATION_FLAGS_L                                  0x00
#define TAB_LNS_NAVIGATION_FLAGS_H                                  0x01
#define TAB_LNS_NAVIGATION_BEARING_L                                0x02
#define TAB_LNS_NAVIGATION_BEARING_H                                0x03
#define TAB_LNS_NAVIGATION_HEADING_L                                0x04
#define TAB_LNS_NAVIGATION_HEADING_H                                0x05
#define TAB_LNS_NAVIGATION_REMAINING_DISTANCE_0                     0x06
#define TAB_LNS_NAVIGATION_REMAINING_DISTANCE_1                     0x07
#define TAB_LNS_NAVIGATION_REMAINING_DISTANCE_2                     0x08
#define TAB_LNS_NAVIGATION_REMAINING_VERTICAL_DISTANCE_0            0x09
#define TAB_LNS_NAVIGATION_REMAINING_VERTICAL_DISTANCE_1            0x0A
#define TAB_LNS_NAVIGATION_REMAINING_VERTICAL_DISTANCE_2            0x0B
#define TAB_LNS_NAVIGATION_UTC_TIME_YEAR_L                          0x0C
#define TAB_LNS_NAVIGATION_UTC_TIME_YEAR_H                          0x0D
#define TAB_LNS_NAVIGATION_UTC_TIME_MONTH                           0x0E
#define TAB_LNS_NAVIGATION_UTC_TIME_DAY                             0x0F
#define TAB_LNS_NAVIGATION_UTC_TIME_HOURS                           0x10
#define TAB_LNS_NAVIGATION_UTC_TIME_MINUTES                         0x11
#define TAB_LNS_NAVIGATION_UTC_TIME_SECONDS                         0x12


#define TAB_LNS_LN_CTRL_OP_CODE                                     0x00
#define TAB_LNS_LN_CTRL_PARAM_MASK_LOC_N_SPD_CHARCT_CONTENT_L       0x01
#define TAB_LNS_LN_CTRL_PARAM_MASK_LOC_N_SPD_CHARCT_CONTENT_H       0x02

#define OP_CODE_LNS_LN_CTRL_PNT_SET_CUMULATIVE_VALUE                0x01
#define OP_CODE_LNS_LN_CTRL_PNT_MASK_LOC_N_SPD_CHARCT_CONTENT       0x02
#define OP_CODE_LNS_LN_CTRL_PNT_NAVIGATION_CONTROL                  0x03
#define OP_CODE_LNS_LN_CTRL_PNT_REQUEST_NUMBER_OF_ROUTES            0x04
#define OP_CODE_LNS_LN_CTRL_PNT_REQUEST_NAME_OF_ROUTE               0x05
#define OP_CODE_LNS_LN_CTRL_PNT_SELECT_ROUTE                        0x06
#define OP_CODE_LNS_LN_CTRL_PNT_SET_FIX_RATE                        0x07
#define OP_CODE_LNS_LN_CTRL_PNT_SET_ELEVATION                       0x08
#define OP_CODE_LNS_LN_CTRL_PNT_RESPONSE_CODE                       0x20

#define RSP_LNS_LN_CTRL_PNT_SUCCESS                                 0x01
#define RSP_LNS_LN_CTRL_PNT_OP_CODE_NOT_SUPPORTED                   0x02
#define RSP_LNS_LN_CTRL_PNT_INVALID_PARAMETER                       0x03
#define RSP_LNS_LN_CTRL_PNT_OPERATION_FAILED                        0x04

#define SIZE_PARAM_LNS_LN_CTRL_PNT_SET_CUMULATIVE_VALUE             0x03    //UINT24
#define SIZE_PARAM_LNS_LN_CTRL_PNT_MASK_LOC_N_SPD_CHARCT_CONTENT    0x02    //UINT16
#define SIZE_PARAM_LNS_LN_CTRL_PNT_NAVIGATION_CONTROL               0x01    //IINT8
#define SIZE_PARAM_LNS_LN_CTRL_PNT_REQUEST_NUMBER_OF_ROUTES         0x02    //UINT16
#define SIZE_PARAM_LNS_LN_CTRL_PNT_REQUEST_NAME_OF_ROUTE            0x02    //UINT16
#define SIZE_PARAM_LNS_LN_CTRL_PNT_SELECT_ROUTE                     0x02    //UINT16
#define SIZE_PARAM_LNS_LN_CTRL_PNT_SET_FIX_RATE                     0x01    //UINT8
#define SIZE_PARAM_LNS_LN_CTRL_PNT_SET_ELEVATION                    0x03    //SINT24

#define MASK_LNS_LN_INSTANTANEOUS_SPEED                             0x01
#define MASK_LNS_LN_TOTAL_DISTANCE                                  0x02
#define MASK_LNS_LN_LOCATION                                        0x04
#define MASK_LNS_LN_ELEVATION                                       0x08
#define MASK_LNS_LN_HEADING                                         0x10
#define MASK_LNS_LN_ROLLING_TIME                                    0x20
#define MASK_LNS_LN_UTC_TIME                                        0x40

#endif // _PROFILE_LNS_
/** @} */



/**
 * @defgroup bleGLS Glucose Service
 * @{
 * @ingroup ble_service
 * @details Glucose service.
 */
// Glucose Service Feature
#ifdef _PROFILE_GLS_

#define GLS_FEAT_LOW_BATTERY_DETECTION_DURING_MEAS_SUPPORTED        0x01  /**< Low Battery Detection During Measurement. */
#define GLS_FEAT_SENSOR_MALFUNCTION_DETECTION_SUPPORTED             0x02  /**< Sensor Malfunction Detection. */
#define GLS_FEAT_SENSOR_SAMPLE_SIZE_SUPPORTED                       0x04  /**< Sensor Sample Size. */
#define GLS_FEAT_SENSOR_STRIP_INSERTION_ERROR_DETECTION_SUPPORTED   0x08  /**< Sensor Strip Insertion Error Detection. */
#define GLS_FEAT_SENSOR_STRIP_TYPE_ERROR_DETECTION_SUPPORTED        0x10  /**< Sensor Strip Type Error Detection. */
#define GLS_FEAT_SENSOR_RESULT_HIGHLOW_DETECTION_SUPPORTED          0x20  /**< Sensor Result High-Low Detection. */
#define GLS_FEAT_SENSOR_TEMPERATURE_HIGHLOW_DETECTION_SUPPORTED     0x40  /**< Sensor Temperature High-Low Detection. */
#define GLS_FEAT_SENSOR_READ_INTERRUPT_DETECTION_SUPPORTED          0x80  /**< Sensor Read Interrupt Detection */
#define GLS_FEAT_GENERAL_DEVICE_FAULT_SUPPORTED                     0x01  /**< General Device Fault. */
#define GLS_FEAT_TIME_FAULT_SUPPORTED                               0x02  /**< Time Fault. */
#define GLS_FEAT_MULTIPLE_BOND_SUPPORTED                            0x04  /**< Multiple Bond. */


#define TAB_GLS_GLUCOSE_MEASUREMENT_FLAGS                           0x00
#define TAB_GLS_GLUCOSE_MEASUREMENT_SEQUENCE_NUMBER_L               0x01
#define TAB_GLS_GLUCOSE_MEASUREMENT_SEQUENCE_NUMBER_H               0x02
#define TAB_GLS_GLUCOSE_MEASUREMENT_UTC_TIME_YEAR_L                 0x03
#define TAB_GLS_GLUCOSE_MEASUREMENT_UTC_TIME_YEAR_H                 0x04
#define TAB_GLS_GLUCOSE_MEASUREMENT_UTC_TIME_MONTH                  0x05
#define TAB_GLS_GLUCOSE_MEASUREMENT_UTC_TIME_DAY                    0x06
#define TAB_GLS_GLUCOSE_MEASUREMENT_UTC_TIME_HOURS                  0x07
#define TAB_GLS_GLUCOSE_MEASUREMENT_UTC_TIME_MINUTES                0x08
#define TAB_GLS_GLUCOSE_MEASUREMENT_UTC_TIME_SECONDS                0x09
#define TAB_GLS_GLUCOSE_MEASUREMENT_TIME_OFFSET_L                   0x0A
#define TAB_GLS_GLUCOSE_MEASUREMENT_TIME_OFFSET_H                   0x0B
#define TAB_GLS_GLUCOSE_MEASUREMENT_GLUCOSE_CONCENTRATION_UNITS_L   0x0C
#define TAB_GLS_GLUCOSE_MEASUREMENT_GLUCOSE_CONCENTRATION_UNITS_H   0x0D
#define TAB_GLS_GLUCOSE_MEASUREMENT_TYPE_SAMPLE_LOCATION            0x0E
#define TAB_GLS_GLUCOSE_MEASUREMENT_SENSOR_STATUS_ANNUNCIATION_L    0x0F
#define TAB_GLS_GLUCOSE_MEASUREMENT_SENSOR_STATUS_ANNUNCIATION_H    0x10

#define TAB_GLS_GLUCOSE_MEASUREMENT_CONTEXT_FLAGS                   0x00
#define TAB_GLS_GLUCOSE_MEASUREMENT_CONTEXT_SEQUENCE_NUMBER_L       0x01
#define TAB_GLS_GLUCOSE_MEASUREMENT_CONTEXT_SEQUENCE_NUMBER_H       0x02
#define TAB_GLS_GLUCOSE_MEASUREMENT_CONTEXT_EXTENDED_FLAGS          0x03
#define TAB_GLS_GLUCOSE_MEASUREMENT_CONTEXT_CARBOHYDRATE_ID         0x04
#define TAB_GLS_GLUCOSE_MEASUREMENT_CONTEXT_CARBOHYDRATE_UNITS_OF_KILOGRAMS_L   0x05
#define TAB_GLS_GLUCOSE_MEASUREMENT_CONTEXT_CARBOHYDRATE_UNITS_OF_KILOGRAMS_H   0x06
#define TAB_GLS_GLUCOSE_MEASUREMENT_CONTEXT_MEAL                    0x07
#define TAB_GLS_GLUCOSE_MEASUREMENT_CONTEXT_TESTER_HEALTH           0x08
#define TAB_GLS_GLUCOSE_MEASUREMENT_CONTEXT_EXERCISE_DURATION_L     0x09
#define TAB_GLS_GLUCOSE_MEASUREMENT_CONTEXT_EXERCISE_DURATION_H     0x0A
#define TAB_GLS_GLUCOSE_MEASUREMENT_CONTEXT_EXERCISE_INTENSITY      0x0B
#define TAB_GLS_GLUCOSE_MEASUREMENT_CONTEXT_MEDICATION_ID           0x0C
#define TAB_GLS_GLUCOSE_MEASUREMENT_CONTEXT_MEDICATION_UNITS_L      0x0D
#define TAB_GLS_GLUCOSE_MEASUREMENT_CONTEXT_MEDICATION_UNITS_H      0x0E
#define TAB_GLS_GLUCOSE_MEASUREMENT_CONTEXT_HBA1C_L                 0x0F
#define TAB_GLS_GLUCOSE_MEASUREMENT_CONTEXT_HBA1C_H                 0x10

#define TAB_GLS_RA_CTRL_OP_CODE                                     0x00
#define TAB_GLS_RA_CTRL_OPERATOR                                    0x01
#define TAB_GLS_RA_CTRL_OPERAND                                     0x02
#define TAB_GLS_RA_CTRL_OPERAND_TYPE                                0x02
#define TAB_GLS_RA_CTRL_OPERAND_PARAM                               0x03


#define OP_CODE_GLS_REC_ACCESS_CTRL_PNT_RPT_STORED_REC              0x01
#define OP_CODE_GLS_REC_ACCESS_CTRL_PNT_DEL_STORED_REC              0x02
#define OP_CODE_GLS_REC_ACCESS_CTRL_PNT_ABORT_OPERATION             0x03
#define OP_CODE_GLS_REC_ACCESS_CTRL_PNT_RPT_NUM_OF_STORED_REC       0x04
#define OP_CODE_GLS_REC_ACCESS_CTRL_PNT_NUMBER_OF_STORED_REC_RSP    0x05
#define OP_CODE_GLS_REC_ACCESS_CTRL_PNT_RSP_CODE                    0x06

#define OPERATOR_GLS_RACP_NULL                                      0x00
#define OPERATOR_GLS_RACP_ALL_RECORDS                               0x01
#define OPERATOR_GLS_RACP_LESS_THAN_OR_EQUAL                        0x02
#define OPERATOR_GLS_RACP_GREATER_THAN_OR_EQUAL                     0x03
#define OPERATOR_GLS_RACP_WITHIN_RANGE                              0x04
#define OPERATOR_GLS_RACP_FIRST_RECORD                              0x05
#define OPERATOR_GLS_RACP_LAST_RECORD                               0x06

#define OPERAND_FILTER_TYPE_GLS_RACP_SEQUENCE_NUMBER                0x01
#define OPERAND_FILTER_TYPE_GLS_RACP_USER_FACING_TIME               0x02


#define RSP_GLS_REC_ACCESS_CTRL_PNT_SUCCESS                         0x01
#define RSP_GLS_REC_ACCESS_CTRL_PNT_OP_CODE_NOT_SUPPORTED           0x02
#define RSP_GLS_REC_ACCESS_CTRL_PNT_INVALID_OPERATOR                0x03
#define RSP_GLS_REC_ACCESS_CTRL_PNT_OPERATOR_NOT_SUPPORTED          0x04
#define RSP_GLS_REC_ACCESS_CTRL_PNT_INVALID_OPERAND                 0x05
#define RSP_GLS_REC_ACCESS_CTRL_PNT_NO_RECORDS_FOUND                0x06
#define RSP_GLS_REC_ACCESS_CTRL_PNT_ABORT_UNSUCCESSFUL              0x07
#define RSP_GLS_REC_ACCESS_CTRL_PNT_PROCEDURE_NOT_COMPLETED         0x08
#define RSP_GLS_REC_ACCESS_CTRL_PNT_OPERAND_NOT_SUPPORTED           0x09

#endif // _PROFILE_GLS_
/** @} */



/**
 * @defgroup bleCPS Cycling Power Service
 * @{
 * @ingroup ble_service
 * @details Cycling Power service.
 */
// Cycling Power Service Feature
#ifdef _PROFILE_CPS_
#define CPS_FEAT_PEDAL_POWER_BALANCE_SUPPORTED                      0x01  /**< Pedal Power Balance. */
#define CPS_FEAT_ACCUMULATED_TORQUE_SUPPORTED                       0x02  /**< Accumulated Torque. */
#define CPS_FEAT_WHEEL_REVOLUTION_DATA_SUPPORTED                    0x04  /**< Wheel Revolution Data. */
#define CPS_FEAT_CRANK_REVOLUTION_DATA_SUPPORTED                    0x08  /**< Crank Revolution Data. */
#define CPS_FEAT_EXTREME_MAGNITUDES_SUPPORTED                       0x10  /**< Extreme Magnitudes. */
#define CPS_FEAT_EXTREME_ANGLES_SUPPORTED                           0x20  /**< Extreme Angles. */
#define CPS_FEAT_TOP_AND_BOTTOM_DEAD_SPOT_ANGLES_SUPPORTED          0x40  /**< Top and Bottom Dead Spot Angles. */
#define CPS_FEAT_ACCUMULATED_ENERGY_SUPPORTED                       0x80  /**< Accumulated Energy. */
#define CPS_FEAT_OFFSET_COMPENSATION_INDICATOR_SUPPORTED            0x01  /**< Offset Compensation Inicator. */
#define CPS_FEAT_OFFSET_COMPENSATION_SUPPORTED                      0x02  /**< Offset Compensation. */
#define CPS_FEAT_CP_MEASUREMENT_CHARA_CONTENT_MASKING_SUPPORTED     0x04  /**< CP Measurement Characteristic Masking. */
#define CPS_FEAT_MULTIPLE_SENSOR_LOCATIONS_SUPPORTED                0x08  /**< Multiple Sensor Location. */
#define CPS_FEAT_CRANK_LENGTH_ADJUSTMENT_SUPPORTED                  0x10  /**< Crank Length Adjustment. */
#define CPS_FEAT_CHAIN_LENGTH_ADJUSTMENT_SUPPORTED                  0x20  /**< Chain Length Adjustment. */
#define CPS_FEAT_CHAIN_WEIGHT_ADJUSTMENT_SUPPORTED                  0x40  /**< Chain Weight Adjustment. */
#define CPS_FEAT_SPAN_LENGTH_ADJUSTMENT_SUPPORTED                   0x80  /**< Span Length Adjustment. */
#define CPS_FEAT_SENSOR_MEASUREMENT_CONTEXT_TORQUE_BASED            0x01  /**< Sensor Measurement. */
#define CPS_FEAT_INSTANTANEOUS_MEASUREMENT_DIRECTION_SUPPORTED      0x02  /**< Instantaneous Measurement Direction. */
#define CPS_FEAT_FACTORY_CALIBRATION_DATE_SUPPORTED                 0x04  /**< Factory Calibration Date. */


// Cycling Power Service Measurement Flag Type
#define TAB_CSCS_MEAS_FLAGS                                         0x00  /**< . */
#define TAB_CSCS_MEAS_CUMULATIVE_WHEEL_REVOLUTIONS_0                0x01  /**< The Byte[0] of The Cumulative Wheel Revolutions value. */
#define TAB_CSCS_MEAS_CUMULATIVE_WHEEL_REVOLUTIONS_1                0x02  /**< The Byte[1] of The Cumulative Wheel Revolutions value. */
#define TAB_CSCS_MEAS_CUMULATIVE_WHEEL_REVOLUTIONS_2                0x03  /**< The Byte[2] of The Cumulative Wheel Revolutions value. */
#define TAB_CSCS_MEAS_CUMULATIVE_WHEEL_REVOLUTIONS_3                0x04  /**< The Byte[3] of The Cumulative Wheel Revolutions value. */
#define TAB_CSCS_MEAS_LAST_WHEEL_EVENT_TIME_L                       0x05  /**< The Low Byte of Last Wheel Event Time. */
#define TAB_CSCS_MEAS_LAST_WHEEL_EVENT_TIME_H                       0x06  /**< The High Byte of Last Wheel Event Time. */
#define TAB_CSCS_MEAS_CUMULATIVE_CRANK_REVOLUTIONS_L                0x07  /**< The Low Byte of Cumulative Crank Revolutions value. */
#define TAB_CSCS_MEAS_CUMULATIVE_CRANK_REVOLUTIONS_H                0x08  /**< The High Byte of Cumulative Crank Revolutions value. */
#define TAB_CSCS_MEAS_LAST_CRANK_EVENT_TIME_L                       0x09  /**< The Low Byte of Last Crank Event Time. */
#define TAB_CSCS_MEAS_LAST_CRANK_EVENT_TIME_H                       0x0A  /**< The High Byte of Last Crank Event Time. */


#define TAB_CSCS_SC_CONTROL_POINT_OP_CODE                           0x00
#define TAB_CSCS_SC_CONTROL_POINT_WHEEL_REVOLUTIONS_B0              0x01
#define TAB_CSCS_SC_CONTROL_POINT_WHEEL_REVOLUTIONS_B1              0x02
#define TAB_CSCS_SC_CONTROL_POINT_WHEEL_REVOLUTIONS_B2              0x03
#define TAB_CSCS_SC_CONTROL_POINT_WHEEL_REVOLUTIONS_B3              0x04
#define TAB_CSCS_SC_CONTROL_POINT_REQUEST_OP_CODE                   0x01
#define TAB_CSCS_SC_CONTROL_POINT_RESPONSE_VALUE                    0x02

#define CSCS_SC_CONTROL_POINT_OP_CODE_SET_CUMULATIVE_VALUE          0x01
#define CSCS_SC_CONTROL_POINT_OP_CODE_START_SENSOR_CALIBRATION      0x02
#define CSCS_SC_CONTROL_POINT_OP_CODE_UPDATE_SENSOR_LOCATION        0x03
#define CSCS_SC_CONTROL_POINT_OP_CODE_REQUEST_SUPPORTED_SENSOR_LOCATIONS    0x04
#define CSCS_SC_CONTROL_POINT_OP_CODE_RESPONSE_CODE                 0x10

#define CSCS_SC_CONTROL_POINT_RESPONSE_VALUE_SUCCESS                0x01
#define CSCS_SC_CONTROL_POINT_RESPONSE_VALUE_OP_CODE_NOT_SUPPORTED  0x02
#define CSCS_SC_CONTROL_POINT_RESPONSE_VALUE_INVALID_PARAMETER      0x03
#define CSCS_SC_CONTROL_POINT_RESPONSE_VALUE_OPERATION_FAILED       0x04


#define TAB_CPS_CYCLING_POWER_MEAS_FLAGS_L                                  0x00
#define TAB_CPS_CYCLING_POWER_MEAS_FLAGS_H                                  0x01
#define TAB_CPS_CYCLING_POWER_MEAS_INSTANTANEOUS_POWER_L                    0x02
#define TAB_CPS_CYCLING_POWER_MEAS_INSTANTANEOUS_POWER_H                    0x03

#define TAB_CPS_CYCLING_POWER_MEAS_WHEEL_REV_DATA_CUMULATIVE_WHEEL_REVS_0   0x04
#define TAB_CPS_CYCLING_POWER_MEAS_WHEEL_REV_DATA_CUMULATIVE_WHEEL_REVS_1   0x05
#define TAB_CPS_CYCLING_POWER_MEAS_WHEEL_REV_DATA_CUMULATIVE_WHEEL_REVS_2   0x06
#define TAB_CPS_CYCLING_POWER_MEAS_WHEEL_REV_DATA_CUMULATIVE_WHEEL_REVS_3   0x07
#define TAB_CPS_CYCLING_POWER_MEAS_WHEEL_REV_DATA_LAST_WHEEL_EVENT_TIME_L   0x08
#define TAB_CPS_CYCLING_POWER_MEAS_WHEEL_REV_DATA_LAST_WHEEL_EVENT_TIME_H   0x09
#define TAB_CPS_CYCLING_POWER_MEAS_CRANK_REV_DATA_CUMULATIVE_CRANK_REVS_L   0x0A
#define TAB_CPS_CYCLING_POWER_MEAS_CRANK_REV_DATA_CUMULATIVE_CRANK_REVS_H   0x0B
#define TAB_CPS_CYCLING_POWER_MEAS_CRANK_REV_DATA_LAST_CRANK_EVENT_TIME_L   0x0C
#define TAB_CPS_CYCLING_POWER_MEAS_CRANK_REV_DATA_LAST_CRANK_EVENT_TIME_H   0x0D
#define TAB_CPS_CYCLING_POWER_MEAS_TOP_DEAD_SPOT_ANGLE_L                    0x0E
#define TAB_CPS_CYCLING_POWER_MEAS_TOP_DEAD_SPOT_ANGLE_H                    0x0F
#define TAB_CPS_CYCLING_POWER_MEAS_BOTTOM_DEAD_SPOT_ANGLE_L                 0x10
#define TAB_CPS_CYCLING_POWER_MEAS_BOTTOM_DEAD_SPOT_ANGLE_H                 0x11


#define TAB_CPS_CYCLING_POWER_MEAS_PEDAL_POWER_BALANCE                      0x04
#define TAB_CPS_CYCLING_POWER_MEAS_ACCUMULATED_TORQUE_L                     0x05
#define TAB_CPS_CYCLING_POWER_MEAS_ACCUMULATED_TORQUE_H                     0x06
#define TAB_CPS_CYCLING_POWER_MEAS_EXTRM_FORCE_MAGN_MAX_FORCE_MAGN_L        0x07
#define TAB_CPS_CYCLING_POWER_MEAS_EXTRM_FORCE_MAGN_MAX_FORCE_MAGN_H        0x08
#define TAB_CPS_CYCLING_POWER_MEAS_EXTRM_FORCE_MAGN_MIN_FORCE_MAGN_L        0x09
#define TAB_CPS_CYCLING_POWER_MEAS_EXTRM_FORCE_MAGN_MIN_FORCE_MAGN_H        0x0A
#define TAB_CPS_CYCLING_POWER_MEAS_EXTRM_TORQUE_MAGN_MAX_TORQUE_MAGN_L      0x0B
#define TAB_CPS_CYCLING_POWER_MEAS_EXTRM_TORQUE_MAGN_MAX_TORQUE_MAGN_H      0x0C
#define TAB_CPS_CYCLING_POWER_MEAS_EXTRM_TORQUE_MAGN_MIN_TORQUE_MAGN_L      0x0D
#define TAB_CPS_CYCLING_POWER_MEAS_EXTRM_TORQUE_MAGN_MIN_TORQUE_MAGN_H      0x0E
#define TAB_CPS_CYCLING_POWER_MEAS_EXTRM_ANGLES_MAX_N_MIN_ANGLE_0           0x0F
#define TAB_CPS_CYCLING_POWER_MEAS_EXTRM_ANGLES_MAX_N_MIN_ANGLE_1           0x10
#define TAB_CPS_CYCLING_POWER_MEAS_EXTRM_ANGLES_MAX_N_MIN_ANGLE_2           0x11

#define TAB_CPS_CYCLING_POWER_MEAS_ACCUMULATED_ENERGY_L                     0x12
#define TAB_CPS_CYCLING_POWER_MEAS_ACCUMULATED_ENERGY_H                     0x13

#define TAB_CPS_CYCLING_POWER_VECTOR_FLAGS                                  0x00
#define TAB_CPS_CYCLING_POWER_VECTOR_CRANK_REV_DATA_CUMULATIVE_CRANK_REVS_L 0x01
#define TAB_CPS_CYCLING_POWER_VECTOR_CRANK_REV_DATA_CUMULATIVE_CRANK_REVS_H 0x02
#define TAB_CPS_CYCLING_POWER_VECTOR_CRANK_REV_DATA_LAST_CRANK_EVENT_TIME_L 0x03
#define TAB_CPS_CYCLING_POWER_VECTOR_CRANK_REV_DATA_LAST_CRANK_EVENT_TIME_H 0x04
#define TAB_CPS_CYCLING_POWER_VECTOR_FIRST_CRANK_REVUREMENT_ANGLE_L         0x05
#define TAB_CPS_CYCLING_POWER_VECTOR_FIRST_CRANK_REVUREMENT_ANGLE_H         0x06
#define TAB_CPS_CYCLING_POWER_VECTOR_INSTANTANEOUS_FORCE_MAGNITUDE_ARRAY_L  0x07
#define TAB_CPS_CYCLING_POWER_VECTOR_INSTANTANEOUS_FORCE_MAGNITUDE_ARRAY_H  0x08
#define TAB_CPS_CYCLING_POWER_VECTOR_INSTANTANEOUS_TORQUE_MAGNITUDE_ARRAY_L 0x09
#define TAB_CPS_CYCLING_POWER_VECTOR_INSTANTANEOUS_TORQUE_MAGNITUDE_ARRAY_H 0x0A

#define TAB_CPS_CP_CTRL_OP_CODE                                             0x00
#define TAB_CPS_CP_CTRL_PARAM_MASK_CYCLING_POWER_MEAS_CHARCT_CONTENT_L      0x01
#define TAB_CPS_CP_CTRL_PARAM_MASK_CYCLING_POWER_MEAS_CHARCT_CONTENT_H      0x02

#define OP_CODE_CPS_CP_CTRL_PNT_SET_CUMULATIVE_VALUE                        0x01
#define OP_CODE_CPS_CP_CTRL_PNT_UPDATE_SENSOR_LOCATION                      0x02
#define OP_CODE_CPS_CP_CTRL_PNT_REQ_SUPPRT_SENSOR_LOCATIONS                 0x03
#define OP_CODE_CPS_CP_CTRL_PNT_SET_CRANK_LENGTH                            0x04
#define OP_CODE_CPS_CP_CTRL_PNT_REQ_CRANK_LENGTH                            0x05
#define OP_CODE_CPS_CP_CTRL_PNT_SET_CHAIN_LENGTH                            0x06
#define OP_CODE_CPS_CP_CTRL_PNT_REQ_CHAIN_LENGTH                            0x07
#define OP_CODE_CPS_CP_CTRL_PNT_SET_CHAIN_WEIGHT                            0x08
#define OP_CODE_CPS_CP_CTRL_PNT_REQ_CHAIN_WEIGHT                            0x09
#define OP_CODE_CPS_CP_CTRL_PNT_SET_SPAN_LENGTH                             0x0A
#define OP_CODE_CPS_CP_CTRL_PNT_REQ_SPAN_LENGTH                             0x0B
#define OP_CODE_CPS_CP_CTRL_PNT_START_OFFSET_COMPENSATION                   0x0C
#define OP_CODE_CPS_CP_CTRL_PNT_MASK_CYCLING_POWER_MEAS_CHARCT_CONTENT      0x0D
#define OP_CODE_CPS_CP_CTRL_PNT_REQ_SAMPLING_RATE                           0x0E
#define OP_CODE_CPS_CP_CTRL_PNT_REQ_FACTORY_CALIBRATION_DATE                0x0F
#define OP_CODE_CPS_CP_CTRL_PNT_RESPONSE_CODE                               0x20

#define RSP_CPS_CP_CTRL_PNT_SUCCESS                                         0x01
#define RSP_CPS_CP_CTRL_PNT_OP_CODE_NOT_SUPPORTED                           0x02
#define RSP_CPS_CP_CTRL_PNT_INVALID_PARAMETER                               0x03
#define RSP_CPS_CP_CTRL_PNT_OPERATION_FAILED                                0x04

#define SIZE_PARAM_CPS_CP_CTRL_PNT_SET_CUMULATIVE_VALUE                     0x04    //UINT32
#define SIZE_PARAM_CPS_CP_CTRL_PNT_UPDATE_SENSOR_LOCATION                   0x01    //UINT8
#define SIZE_PARAM_CPS_CP_CTRL_PNT_REQ_SUPPRT_SENSOR_LOCATIONS              0x00
#define SIZE_PARAM_CPS_CP_CTRL_PNT_SET_CRANK_LENGTH                         0x02    //UINT16
#define SIZE_PARAM_CPS_CP_CTRL_PNT_REQ_CRANK_LENGTH                         0x00
#define SIZE_PARAM_CPS_CP_CTRL_PNT_SET_CHAIN_LENGTH                         0x02    //UINT16
#define SIZE_PARAM_CPS_CP_CTRL_PNT_REQ_CHAIN_LENGTH                         0x02    //UINT16
#define SIZE_PARAM_CPS_CP_CTRL_PNT_SET_CHAIN_WEIGHT                         0x02    //UINT16
#define SIZE_PARAM_CPS_CP_CTRL_PNT_REQ_CHAIN_WEIGHT                         0x02    //UINT16
#define SIZE_PARAM_CPS_CP_CTRL_PNT_SET_SPAN_LENGTH                          0x02    //UINT16
#define SIZE_PARAM_CPS_CP_CTRL_PNT_REQ_SPAN_LENGTH                          0x02    //UINT16
#define SIZE_PARAM_CPS_CP_CTRL_PNT_START_OFFSET_COMPENSATION                0x00
#define SIZE_PARAM_CPS_CP_CTRL_PNT_MASK_CYCLING_POWER_MEAS_CHARCT_CONTENT   0x02    //UINT16
#define SIZE_PARAM_CPS_CP_CTRL_PNT_REQ_SAMPLING_RATE                        0x00
#define SIZE_PARAM_CPS_CP_CTRL_PNT_REQ_FACTORY_CALIBRATION_DATE             0x00

#endif // _PROFILE_CPS_
/** @} */


/**
 * @defgroup bleHOGP Human Interface Device over GATT Profile
 * @{
 * @ingroup ble_service
 * @details Human Interface Device over GATT profile.
 */
#ifdef _PROFILE_HOGP_
#ifndef _PROFILE_HOGP_EMC_
#ifdef _PROFILE_HOGP_COMSUMER_
#define HDL_HIDS_REPORT_TAB_CSKEY_L                             0
#define HDL_HIDS_REPORT_TAB_CSKEY_H                             1
#endif

#ifdef _PROFILE_HOGP_MOUSE_
#define HDL_HIDS_REPORT_TAB_KEY_L_R                             0
#define HDL_HIDS_REPORT_TAB_DIR_L_R_L                           1
#define HDL_HIDS_REPORT_TAB_DIR_L_R_H                           2
#define HDL_HIDS_REPORT_TAB_DIR_U_D_L                           3
#define HDL_HIDS_REPORT_TAB_DIR_U_D_H                           4
#define HDL_HIDS_REPORT_TAB_ROL_U_D                             5
#define HDL_HIDS_REPORT_TAB_ROL_L_R_L                           6
#define HDL_HIDS_REPORT_TAB_ROL_L_R_H                           7
#endif

#ifdef _PROFILE_HOGP_KEYBOARD_
#define HDL_HIDS_REPORT_TAB_KEY_CTRL                            0
#define HDL_HIDS_REPORT_TAB_KEY_DATA0                           2
#define HDL_HIDS_REPORT_TAB_KEY_DATA1                           3
#define HDL_HIDS_REPORT_TAB_KEY_DATA2                           4
#define HDL_HIDS_REPORT_TAB_KEY_DATA3                           5
#define HDL_HIDS_REPORT_TAB_KEY_DATA4                           6
#define HDL_HIDS_REPORT_TAB_KEY_DATA5                           7
#ifdef _PROFILE_HOGP_KEYBOARD_PHOTO_
#define HDL_HIDS_REPORT_TAB_KB_KEY_VOL                          8
#endif
#endif
#else       //#ifndef _PROFILE_HOGP_EMC_
#define HDL_HIDS_REPORT_TAB_81KEY                               0
#ifdef _PROFILE_HOGP_MOUSE_
#define HDL_HIDS_REPORT_TAB_KEY_L_R                             0
#define HDL_HIDS_REPORT_TAB_DIR_L_R_L                           1
#define HDL_HIDS_REPORT_TAB_DIR_U_D_L                           2
#define HDL_HIDS_REPORT_TAB_ROL_U_D                             3
#endif

#ifdef _PROFILE_HOGP_KEYBOARD_
#define HDL_HIDS_REPORT_TAB_KEY_CTRL                            0
#define HDL_HIDS_REPORT_TAB_KEY_DATA0                           2
#define HDL_HIDS_REPORT_TAB_KEY_DATA1                           3
#define HDL_HIDS_REPORT_TAB_KEY_DATA2                           4
#define HDL_HIDS_REPORT_TAB_KEY_DATA3                           5
#define HDL_HIDS_REPORT_TAB_KEY_DATA4                           6
#define HDL_HIDS_REPORT_TAB_KEY_DATA5                           7
#endif      //#ifndef _PROFILE_HOGP_EMC_
#endif
#endif
/** @} */



/**
 * @defgroup bleOTA Over The Air Service
 * @{
 * @ingroup ble_service
 * @details Over The Air service.
 */
#ifdef _PROFILE_OTA_
// Over The Air Service Operatione Code Definition
#define OP_CODE_OTA_CMD_REQ_CUSTOMER_VER                                    0x01
#define OP_CODE_OTA_CMD_REQ_PRDCT_VER                                       0x02
#define OP_CODE_OTA_CMD_REQ_FW_VER                                          0x03
#define OP_CODE_OTA_CMD_PROCEDURE_START                                     0x04
#define OP_CODE_OTA_CMD_PROCEDURE_FINISH                                    0x05
#define OP_CODE_OTA_CMD_PROCEDURE_END                                       0x06
#define OP_CODE_OTA_CMD_PROCEDURE_TERMN                                     0x07
#define OP_CODE_OTA_CMD_RESPONSE_CODE                                       0x08
#define TAB_OTA_CP_OP_CODE                                                  0x00
#endif // _PROFILE_OTA_
/** @} */



// DIS
//http://developer.bluetooth.org/gatt/characteristics/Pages/CharacteristicViewer.aspx?u=org.bluetooth.characteristic.pnp_id.xml
#define BLE_GATT_DIS_PNPID_VID_SOURCE_BLUETOOTH_SIG     0x01
#define BLE_GATT_DIS_PNPID_VID_SOURCE_USB_FORUM         0x02




extern const Uint8 ATT_HDL_INVALID[];
extern const Uint8 ATT_HDL_GAP_PRIMARY_SERVICE[];
extern const Uint8 ATT_HDL_GAP_CHARACTERISTIC_DEVICE_NAME[];
extern const Uint8 ATT_HDL_GAP_DEVICE_NAME_INIT[];
extern Uint8 att_HDL_GAP_DEVICE_NAME[];
extern const Uint8 ATT_HDL_GAP_CHARACTERISTIC_APPEARANCE[];
extern const Uint8 ATT_HDL_GAP_APPEARANCE[];
extern const Uint8 ATT_HDL_GAP_CHARACTERISTIC_RECONNECTION_ADDRESS[];
extern const Uint8 ATT_HDL_GAP_RECONNECTION_ADDRESS_INIT[];
extern Uint8 att_HDL_GAP_RECONNECTION_ADDRESS[];
extern const Uint8 ATT_HDL_GAP_CHARACTERISTIC_PERIPHERAL_PRIVACY_FLAG[];
extern const Uint8 ATT_HDL_GAP_PERIPHERAL_PRIVACY_FLAG[];
extern const Uint8 ATT_HDL_GAP_CHARACTERISTIC_PERIPHERAL_PREFERRED_CONNECTION_PARAMETERS[];
extern const Uint8 ATT_HDL_GAP_PERIPHERAL_PREFERRED_CONNECTION_PARAMETERS[];

extern const Uint8 ATT_HDL_GATT_PRIMARY_SERVICE[];
extern const Uint8 ATT_HDL_GATT_CHARACTERISTIC_SERVICE_CHANGED[];
extern const Uint8 ATT_HDL_GATT_SERVICE_CHANGED_INIT[];
extern Uint8 att_HDL_GATT_SERVICE_CHANGED[];


// DIS
extern const Uint8 ATT_HDL_DIS_PRIMARY_SERVICE[];
extern const Uint8 ATT_HDL_DIS_CHARACTERISTIC_SERIAL_NUMBER_STRING[];
extern const Uint8 ATT_HDL_DIS_SERIAL_NUMBER_STRING[];
extern const Uint8 ATT_HDL_DIS_SERIAL_NUMBER_STRING_PRESENTATION_FORMAT[];
extern const Uint8 ATT_HDL_DIS_CHARACTERISTIC_MANUFACTURER_NAME_STRING[];
extern const Uint8 ATT_HDL_DIS_MANUFACTURER_NAME_STRING[];
extern const Uint8 ATT_HDL_DIS_MANUFACTURER_NAME_STRING_PRESENTATION_FORMAT[];
extern const Uint8 ATT_HDL_DIS_CHARACTERISTIC_SYSTEM_ID[];
extern const Uint8 ATT_HDL_DIS_SYSTEM_ID[];
extern const Uint8 ATT_HDL_DIS_CHARACTERISTIC_FIRMWARE_REVISION_STRING[];
extern const Uint8 ATT_HDL_DIS_FIRMWARE_REVISION_STRING[];
extern const Uint8 ATT_HDL_DIS_FIRMWARE_REVISION_STRING_PRESENTATION_FORMAT[];


#ifdef _PROFILE_HRP_
extern const Uint8 ATT_HDL_HRS_PRIMARY_SERVICE[];
extern const Uint8 ATT_HDL_HRS_CHARACTERISTIC_HEART_RATE_MEASUREMENT[];
extern const Uint8 ATT_HDL_HRS_HEART_RATE_MEASUREMENT_INIT[];
extern Uint8 att_HDL_HRS_HEART_RATE_MEASUREMENT[];
extern const Uint8 ATT_HDL_HRS_CLIENT_CHARACTERISTIC_CONFIGURATION_INIT[];
extern Uint8 att_HDL_HRS_CLIENT_CHARACTERISTIC_CONFIGURATION[];
extern const Uint8 ATT_HDL_HRS_CHARACTERISTIC_BODY_SENSOR_LOCATION[];
extern const Uint8 ATT_HDL_HRS_BODY_SENSOR_LOCATION_INIT[];
extern Uint8 att_HDL_HRS_BODY_SENSOR_LOCATION[];
#endif
#ifdef _PROFILE_BAS_
extern const Uint8 ATT_HDL_BAS_PRIMARY_SERVICE[];
extern const Uint8 ATT_HDL_BAS_CHARACTERISTIC_BATTERY_LEVEL[];
extern const Uint8 ATT_HDL_BAS_BATTERY_LEVEL_INIT[];
extern Uint8 att_HDL_BAS_BATTERY_LEVEL[];
extern const Uint8 ATT_HDL_BAS_CLIENT_CHARACTERISTIC_CONFIGURATION_INIT[];
extern Uint8 att_HDL_BAS_CLIENT_CHARACTERISTIC_CONFIGURATION[];
#endif
#ifdef _PROFILE_SCPP_
extern const Uint8 ATT_HDL_SCPS_PRIMARY_SERVICE[];
extern const Uint8 ATT_HDL_SCPS_CHARACTERISTIC_SCAN_INTERVAL_WINDOW[];
extern const Uint8 ATT_HDL_SCPS_SCAN_INTERVAL_WINDOW_INIT[];
extern Uint8 att_HDL_SCPS_SCAN_INTERVAL_WINDOW[];
extern const Uint8 ATT_HDL_SCPS_CHARACTERISTIC_SCAN_REFRESH[];
extern const Uint8 ATT_HDL_SCPS_SCAN_REFRESH_INIT[];
extern Uint8 att_HDL_SCPS_SCAN_REFRESH[];
extern const Uint8 ATT_HDL_SCPS_SCAN_REFRESH_CLIENT_CHARACTERISTIC_CONFIGURATION_INIT[];
extern Uint8 att_HDL_SCPS_SCAN_REFRESH_CLIENT_CHARACTERISTIC_CONFIGURATION[];
#endif
#ifdef _PROFILE_BLP_
extern const Uint8 ATT_HDL_BLS_PRIMARY_SERVICE[];
extern const Uint8 ATT_HDL_BLS_CHARACTERISTIC_BLOOD_PRESSURE_MEASUREMENT[];
extern const Uint8 ATT_HDL_BLS_BLOOD_PRESSURE_MEASUREMENT_INIT[];
extern Uint8 att_HDL_BLS_BLOOD_PRESSURE_MEASUREMENT[];
extern Uint8 idx_ATT_HDL_BLS_BLOOD_PRESSURE_MEASUREMENT_DATABASE[6];
extern const Uint8 ATT_HDL_BLS_BLOOD_PRESSURE_MEASUREMENT_DATABASE[6][16];
extern const Uint8 ATT_HDL_BLS_BLOOD_PRESSURE_CLIENT_CHARACTERISTIC_CONFIGURATION_INIT[];
extern Uint8 att_HDL_BLS_BLOOD_PRESSURE_CLIENT_CHARACTERISTIC_CONFIGURATION[];
extern const Uint8 ATT_HDL_BLS_CHARACTERISTIC_BLOOD_PRESSURE_FEATURE[];
extern const Uint8 ATT_HDL_BLS_BLOOD_PRESSURE_FEATURE_INIT[];
#endif
#ifdef _PROFILE_HTP_
extern const Uint8 ATT_HDL_HTS_PRIMARY_SERVICE[];
extern const Uint8 ATT_HDL_HTS_CHARACTERISTIC_TEMPERATURE_MEASUREMENT[];
extern const Uint8 ATT_HDL_HTS_TEMPERATURE_MEASUREMENT_INIT[];
extern Uint8 att_HDL_HTS_TEMPERATURE_MEASUREMENT[];
extern Uint8 idx_ATT_HDL_HTS_TEMPERATURE_MEASUREMENT_DATABASE[6];
extern const Uint8 ATT_HDL_HTS_TEMPERATURE_MEASUREMENT_DATABASE[6][SIZE_OF_ATTRIBUTE_VALUE_13];
extern const Uint8 ATT_HDL_HTS_TEMPERATURE_MEASUREMENT_CLIENT_CHARACTERISTIC_CONFIGURATION_INIT[];
extern Uint8 att_HDL_HTS_TEMPERATURE_MEASUREMENT_CLIENT_CHARACTERISTIC_CONFIGURATION[];
extern const Uint8 ATT_HDL_HTS_CHARACTERISTIC_TEMPERATURE_TYPE[];
extern const Uint8 ATT_HDL_HTS_TEMPERATURE_TYPE_INIT[];
#endif
#ifdef _PROFILE_RSCP_
extern const Uint8 ATT_HDL_RSCS_PRIMARY_SERVICE[];
extern const Uint8 ATT_HDL_RSCS_CHARACTERISTIC_RSC_MEASUREMENT[];
extern const Uint8 ATT_HDL_RSCS_RSC_MEASUREMENT_INIT[];
extern Uint8 att_HDL_RSCS_RSC_MEASUREMENT[];
extern const Uint8 ATT_HDL_RSCS_RSC_MEASUREMENT_CLIENT_CHARACTERISTIC_CONFIGURATION_INIT[];
extern Uint8 att_HDL_RSCS_RSC_MEASUREMENT_CLIENT_CHARACTERISTIC_CONFIGURATION[];
extern const Uint8 ATT_HDL_RSCS_CHARACTERISTIC_RSC_FEATURE[];
extern const Uint8 ATT_HDL_RSCS_RSC_FEATURE_INIT[];
extern const Uint8 ATT_HDL_RSCS_CHARACTERISTIC_SC_CONTROL_POINT[];
extern const Uint8 ATT_HDL_RSCS_SC_CONTROL_POINT_INIT[];
extern Uint8 att_HDL_RSCS_SC_CONTROL_POINT[];
extern const Uint8 ATT_HDL_RSCS_SC_CONTROL_POINT_CLIENT_CHARACTERISTIC_CONFIGURATION_INIT[];
extern Uint8 att_HDL_RSCS_SC_CONTROL_POINT_CLIENT_CHARACTERISTIC_CONFIGURATION[];
#endif
#ifdef _PROFILE_CSCP_
extern const Uint8 ATT_HDL_CSCS_PRIMARY_SERVICE[];
extern const Uint8 ATT_HDL_CSCS_CHARACTERISTIC_CSC_MEASUREMENT[];
extern const Uint8 ATT_HDL_CSCS_CSC_MEASUREMENT_INIT[];
extern Uint8 att_HDL_CSCS_CSC_MEASUREMENT[];
extern const Uint8 ATT_HDL_CSCS_CSC_MEASUREMENT_CLIENT_CHARACTERISTIC_CONFIGURATION_INIT[];
extern Uint8 att_HDL_CSCS_CSC_MEASUREMENT_CLIENT_CHARACTERISTIC_CONFIGURATION[];
extern const Uint8 ATT_HDL_CSCS_CHARACTERISTIC_CSC_FEATURE[];
extern const Uint8 ATT_HDL_CSCS_CSC_FEATURE_INIT[];
extern const Uint8 ATT_HDL_CSCS_CHARACTERISTIC_SC_CONTROL_POINT[];
extern const Uint8 ATT_HDL_CSCS_SC_CONTROL_POINT_INIT[];
extern Uint8 att_HDL_CSCS_SC_CONTROL_POINT[];
extern const Uint8 ATT_HDL_CSCS_SC_CONTROL_POINT_CLIENT_CHARACTERISTIC_CONFIGURATION_INIT[];
extern Uint8 att_HDL_CSCS_SC_CONTROL_POINT_CLIENT_CHARACTERISTIC_CONFIGURATION[];
#endif

#ifdef _PROFILE_LNS_
extern const Uint8 ATT_HDL_LNS_PRIMARY_SERVICE[];
extern const Uint8 ATT_HDL_LNS_CHARACTERISTIC_LOCATION_AND_SPEED[];
extern const Uint8 ATT_HDL_LNS_LOCATION_AND_SPEED_INIT[];
extern Uint8 att_HDL_LNS_LOCATION_AND_SPEED[];
extern const Uint8 ATT_HDL_LNS_LOCATION_AND_SPEED_CLIENT_CHARACTERISTIC_CONFIGURATION_INIT[];
extern Uint8 att_HDL_LNS_LOCATION_AND_SPEED_CLIENT_CHARACTERISTIC_CONFIGURATION[];
extern const Uint8 ATT_HDL_LNS_CHARACTERISTIC_LN_FEATURE[];
extern const Uint8 ATT_HDL_LNS_LN_FEATURE_INIT[];
extern const Uint8 ATT_HDL_LNS_CHARACTERISTIC_POSITION_QUALITY[];
extern const Uint8 ATT_HDL_LNS_POSITION_QUALITY_INIT[];
extern Uint8 att_HDL_LNS_POSITION_QUALITY[];
extern const Uint8 ATT_HDL_LNS_CHARACTERISTIC_LN_CONTROL_POINT[];
extern const Uint8 ATT_HDL_LNS_LN_CONTROL_POINT_INIT[];
extern Uint8 att_HDL_LNS_LN_CONTROL_POINT[];
extern const Uint8 ATT_HDL_LNS_LN_CONTROL_POINT_CLIENT_CHARACTERISTIC_CONFIGURATION_INIT[];
extern Uint8 att_HDL_LNS_LN_CONTROL_POINT_CLIENT_CHARACTERISTIC_CONFIGURATION[];
extern const Uint8 ATT_HDL_LNS_CHARACTERISTIC_NAVIGATION[];
extern const Uint8 ATT_HDL_LNS_NAVIGATION_INIT[];
extern Uint8 att_HDL_LNS_NAVIGATION[];
extern const Uint8 ATT_HDL_LNS_NAVIGATION_CLIENT_CHARACTERISTIC_CONFIGURATION_INIT[];
extern Uint8 att_HDL_LNS_NAVIGATION_CLIENT_CHARACTERISTIC_CONFIGURATION[];
#endif

#ifdef _PROFILE_GLS_
extern const Uint8 ATT_HDL_GLS_PRIMARY_SERVICE[];
extern const Uint8 ATT_HDL_GLS_CHARACTERISTIC_GLUCOSE_MEASUREMENT[];
extern const Uint8 ATT_HDL_GLS_GLUCOSE_MEASUREMENT_INIT[];
extern Uint8 att_HDL_GLS_GLUCOSE_MEASUREMENT[];
extern const Uint8 ATT_HDL_GLS_GLUCOSE_MEASUREMENT_CLIENT_CHARACTERISTIC_CONFIGURATION_INIT[];
extern Uint8 att_HDL_GLS_GLUCOSE_MEASUREMENT_CLIENT_CHARACTERISTIC_CONFIGURATION[];
extern const Uint8 ATT_HDL_GLS_CHARACTERISTIC_GLUCOSE_MEASUREMENT_CONTEXT[];
extern const Uint8 ATT_HDL_GLS_GLUCOSE_MEASUREMENT_CONTEXT_INIT[];
extern Uint8 att_HDL_GLS_GLUCOSE_MEASUREMENT_CONTEXT[];
extern const Uint8 ATT_HDL_GLS_GLUCOSE_MEASUREMENT_CONTEXT_CLIENT_CHARACTERISTIC_CONFIGURATION_INIT[];
extern Uint8 att_HDL_GLS_GLUCOSE_MEASUREMENT_CONTEXT_CLIENT_CHARACTERISTIC_CONFIGURATION[];
extern const Uint8 ATT_HDL_GLS_CHARACTERISTIC_GLUCOSE_FEATURE[];
extern const Uint8 ATT_HDL_GLS_GLUCOSE_FEATURE_INIT[];
extern const Uint8 ATT_HDL_GLS_CHARACTERISTIC_RECORD_ACCESS_CONTROL_POINT[];
extern const Uint8 ATT_HDL_GLS_RECORD_ACCESS_CONTROL_POINT_INIT[];
extern Uint8 att_HDL_GLS_RECORD_ACCESS_CONTROL_POINT[];
extern const Uint8 ATT_HDL_GLS_RECORD_ACCESS_CONTROL_POINT_CLIENT_CHARACTERISTIC_CONFIGURATION_INIT[];
extern Uint8 att_HDL_GLS_RECORD_ACCESS_CONTROL_POINT_CLIENT_CHARACTERISTIC_CONFIGURATION[];
#endif

#ifdef _PROFILE_CPS_
extern const Uint8 ATT_HDL_CPS_PRIMARY_SERVICE[];
extern const Uint8 ATT_HDL_CPS_CHARACTERISTIC_CYCLING_POWER_MEASUREMENT[];
extern const Uint8 ATT_HDL_CPS_CYCLING_POWER_MEASUREMENT_INIT[];
extern Uint8 att_HDL_CPS_CYCLING_POWER_MEASUREMENT[];
extern const Uint8 ATT_HDL_CPS_CYCLING_POWER_MEASUREMENT_CLIENT_CHARACTERISTIC_CONFIGURATION_INIT[];
extern Uint8 att_HDL_CPS_CYCLING_POWER_MEASUREMENT_CLIENT_CHARACTERISTIC_CONFIGURATION[];
extern const Uint8 ATT_HDL_CPS_CYCLING_POWER_MEASUREMENT_SERVER_CHARACTERISTIC_CONFIGURATION_INIT[];
extern Uint8 att_HDL_CPS_CYCLING_POWER_MEASUREMENT_SERVER_CHARACTERISTIC_CONFIGURATION[];
extern const Uint8 ATT_HDL_CPS_CHARACTERISTIC_CYCLING_POWER_FEATURE[];
extern const Uint8 ATT_HDL_CPS_CYCLING_POWER_FEATURE_INIT[];
extern const Uint8 ATT_HDL_CPS_CHARACTERISTIC_SENSOR_LOCATION[];
extern const Uint8 ATT_HDL_CPS_SENSOR_LOCATION_INIT[];
extern Uint8 att_HDL_CPS_SENSOR_LOCATION[];
extern const Uint8 ATT_HDL_CPS_CHARACTERISTIC_CYCLING_POWER_VECTOR[];
extern const Uint8 ATT_HDL_CPS_CYCLING_POWER_VECTOR_INIT[];
extern Uint8 att_HDL_CPS_CYCLING_POWER_VECTOR[];
extern const Uint8 ATT_HDL_CPS_CYCLING_POWER_VECTOR_CLIENT_CHARACTERISTIC_CONFIGURATION_INIT[];
extern Uint8 att_HDL_CPS_CYCLING_POWER_VECTOR_CLIENT_CHARACTERISTIC_CONFIGURATION[];
extern const Uint8 ATT_HDL_CPS_CHARACTERISTIC_CYCLING_POWER_CONTROL_POINT[];
extern const Uint8 ATT_HDL_CPS_CYCLING_POWER_CONTROL_POINT_INIT[];
extern Uint8 att_HDL_CPS_CYCLING_POWER_CONTROL_POINT[];
extern const Uint8 ATT_HDL_CPS_CYCLING_POWER_CONTROL_POINT_CLIENT_CHARACTERISTIC_CONFIGURATION_INIT[];
extern Uint8 att_HDL_CPS_CYCLING_POWER_CONTROL_POINT_CLIENT_CHARACTERISTIC_CONFIGURATION[];
#endif

#ifdef _PROFILE_CSTM_UDF01S_
extern const Uint8 ATT_HDL_UDF01S_PRIMARY_SERVICE[];
extern const Uint8 ATT_HDL_UDF01S_CHARACTERISTIC_UDATR01[];
extern const Uint8 ATT_HDL_UDF01S_UDATR01[];
extern const Uint8 ATT_HDL_UDF01S_CHARACTERISTIC_UDATN01[];
extern const Uint8 ATT_HDL_UDF01S_UDATN01_INIT[];
extern Uint8 att_HDL_UDF01S_UDATN01[];
extern const Uint8 ATT_HDL_UDF01S_UDATN01_CLIENT_CHARACTERISTIC_CONFIGURATION_INIT[];
extern Uint8 att_HDL_UDF01S_UDATN01_MEASUREMENT_CLIENT_CHARACTERISTIC_CONFIGURATION[];
extern const Uint8 ATT_HDL_UDF01S_CHARACTERISTIC_UDATRW01[];
extern const Uint8 ATT_HDL_UDF01S_UDATRW01_INIT[];
extern Uint8 att_HDL_UDF01S_UDATRW01[];
#endif
#ifdef _PROFILE_CSTM_UDF02S_
extern const Uint8 ATT_HDL_UDF02S_PRIMARY_SERVICE[];
extern const Uint8 ATT_HDL_UDF02S_CHARACTERISTIC_UDATR02[];
extern const Uint8 ATT_HDL_UDF02S_UDATR02[];
extern const Uint8 ATT_HDL_UDF02S_CHARACTERISTIC_UDATN02[];
extern const Uint8 ATT_HDL_UDF02S_UDATN02_INIT[];
extern Uint8 att_HDL_UDF02S_UDATN02[];
extern const Uint8 ATT_HDL_UDF02S_UDATN02_CLIENT_CHARACTERISTIC_CONFIGURATION_INIT[];
extern Uint8 att_HDL_UDF02S_UDATN02_MEASUREMENT_CLIENT_CHARACTERISTIC_CONFIGURATION[];
extern const Uint8 ATT_HDL_UDF02S_CHARACTERISTIC_UDATRW02[];
extern const Uint8 ATT_HDL_UDF02S_UDATRW02_INIT[];
extern Uint8 att_HDL_UDF02S_UDATRW02[];
#endif
#ifdef _PROFILE_CSTM_UDF03S_
extern const Uint8 ATT_HDL_UDF03S_PRIMARY_SERVICE[];
extern const Uint8 ATT_HDL_UDF03S_CHARACTERISTIC_UDATR03[];
extern const Uint8 ATT_HDL_UDF03S_UDATR03[];
extern const Uint8 ATT_HDL_UDF03S_CHARACTERISTIC_UDATN03[];
extern const Uint8 ATT_HDL_UDF03S_UDATN03_INIT[];
extern Uint8 att_HDL_UDF03S_UDATN03[];
extern const Uint8 ATT_HDL_UDF02S_UDATN02_CLIENT_CHARACTERISTIC_CONFIGURATION_INIT[];
extern Uint8 att_HDL_UDF02S_UDATN02_MEASUREMENT_CLIENT_CHARACTERISTIC_CONFIGURATION[];
extern const Uint8 ATT_HDL_UDF03S_CHARACTERISTIC_UDATRW03[];
extern const Uint8 ATT_HDL_UDF03S_UDATRW03_INIT[];
extern Uint8 att_HDL_UDF03S_UDATRW03[];
#endif
#ifdef _PROFILE_ATCMD_
extern const Uint8 ATT_HDL_ATCMD_PRIMARY_SERVICE[];
extern const Uint8 ATT_HDL_ATCMD_CHARACTERISTIC_READ[];
extern const Uint8 ATT_HDL_ATCMD_READ[];
extern const Uint8 ATT_HDL_ATCMD_CHARACTERISTIC_NOTIFY[];
extern const Uint8 ATT_HDL_ATCMD_NOTIFY_INIT[];
extern Uint8 att_HDL_ATCMD_NOTIFY[];
extern const Uint8 ATT_HDL_ATCMD_NOTIFY_CLIENT_CHARACTERISTIC_CONFIGURATION_INIT[];
extern Uint8 att_HDL_ATCMD_NOTIFY_CLIENT_CHARACTERISTIC_CONFIGURATION[];
extern const Uint8 ATT_HDL_ATCMD_CHARACTERISTIC_RW[];
extern const Uint8 ATT_HDL_ATCMD_RW_INIT[];
extern Uint8 att_HDL_ATCMD_UDATRW01[];
#endif
#ifdef _PROFILE_CSTM_DATAEXCHANGE_S_
extern const Uint8 ATT_HDL_DATAEXCHANGES_PRIMARY_SERVICE[];
extern const Uint8 ATT_HDL_DATAEXCHANGES_CHARACTERISTIC_WRITE[];
extern const Uint8 ATT_HDL_DATAEXCHANGES_WRITE[];
extern const Uint8 ATT_HDL_DATAEXCHANGES_CHARACTERISTICR_READ[];
extern const Uint8 ATT_HDL_DATAEXCHANGES_READINIT[];
extern Uint8 att_HDL_DATAEXCHANGES_READINIT[];
extern const Uint8 ATT_HDL_DATAEXCHANGES_READINIT_CLIENT_CHARACTERISTIC_CONFIGURATION_INIT[];
extern Uint8 att_HDL_DATAEXCHANGES_READINIT_MEASUREMENT_CLIENT_CHARACTERISTIC_CONFIGURATION[];
#endif

#ifdef _PROFILE_OTA_
extern const Uint8 ATT_HDL_OTA_PRIMARY_SERVICE[];
extern const Uint8 ATT_HDL_OTA_CHARACTERISTIC_OTA_DATA[];
extern const Uint8 ATT_HDL_OTA_OTA_DATA_INIT[];
extern Uint8 att_HDL_OTA_OTA_DATA[];
extern const Uint8 ATT_HDL_OTA_OTA_DATA_CLIENT_CHARACTERISTIC_CONFIGURATION_INIT[];
extern Uint8 att_HDL_OTA_OTA_DATA_CLIENT_CHARACTERISTIC_CONFIGURATION[];
extern const Uint8 ATT_HDL_OTA_OTA_DATA_DESCRIPTION[];
extern const Uint8 ATT_HDL_OTA_CHARACTERISTIC_OTA_CMD_CONTROL_POINT[];
extern const Uint8 ATT_HDL_OTA_OTA_CMD_CONTROL_POINT_INIT[];
extern Uint8 att_HDL_OTA_OTA_CMD_CONTROL_POINT[];
extern const Uint8 ATT_HDL_OTA_OTA_CMD_CONTROL_POINT_CLIENT_CHARACTERISTIC_CONFIGURATION_INIT[];
extern Uint8 att_HDL_OTA_OTA_CMD_CONTROL_POINT_CLIENT_CHARACTERISTIC_CONFIGURATION[];
extern const Uint8 ATT_HDL_OTA_OTA_CMD_USER_DESCRIPTION[];
#endif


#ifdef _PROFILE_HOGP_
extern const Uint8 ATT_HDL_HIDS_PRIMARY_SERVICE[];
extern const Uint8 ATT_HDL_HIDS_CHARACTERISTIC_HID_INFORMATION[];
extern const Uint8 ATT_HDL_HIDS_HID_INFORMATION[];
extern const Uint8 ATT_HDL_HIDS_CHARACTERISTIC_HID_CONTROL_POINT[];
extern const Uint8 ATT_HDL_HIDS_HID_CONTROL_POINT_INIT[];
extern Uint8 att_HDL_HIDS_HID_CONTROL_POINT[];
extern const Uint8 ATT_HDL_HIDS_CHARACTERISTIC_REPORT_MAP[];
extern const Uint8 ATT_HDL_HIDS_REPORT_MAP[];    //Device Class Definition for Human Interface Devices (HID) Version 1.11, 6.2.2 Report Descriptor
//#ifdef _PROFILE_HOGP_COMSUMER_
extern const Uint8 ATT_HDL_HIDS_CHARACTERISTIC_REPORT_CSI[];
extern const Uint8 ATT_HDL_HIDS_REPORT_CSI_INIT[];
extern Uint8 att_HDL_HIDS_REPORT_CSI[];
extern const Uint8 ATT_HDL_HIDS_REPORT_CSI_CLIENT_CHARACTERISTIC_CONFIGURATION_INIT[];           //Mandatory if the Battery Level characteristic properties supports notification
extern Uint8 att_HDL_HIDS_REPORT_CSI_CLIENT_CHARACTERISTIC_CONFIGURATION[];
extern const Uint8 ATT_HDL_HIDS_REPORT_CSI_REPORT_REFERENCE[];
//#endif      //#ifdef _PROFILE_HOGP_COMSUMER_
//#ifdef _PROFILE_HOGP_MOUSE_
extern const Uint8 ATT_HDL_HIDS_CHARACTERISTIC_REPORT_MSI[];
extern const Uint8 ATT_HDL_HIDS_REPORT_MSI_INIT[];
extern Uint8 att_HDL_HIDS_REPORT_MSI[];
extern const Uint8 ATT_HDL_HIDS_REPORT_MSI_CLIENT_CHARACTERISTIC_CONFIGURATION_INIT[];           //Mandatory if the Battery Level characteristic properties supports notification
extern Uint8 att_HDL_HIDS_REPORT_MSI_CLIENT_CHARACTERISTIC_CONFIGURATION[];
extern const Uint8 ATT_HDL_HIDS_REPORT_MSI_REPORT_REFERENCE[];
extern const Uint8 ATT_HDL_HIDS_CHARACTERISTIC_REPORT_MSF[];
extern const Uint8 ATT_HDL_HIDS_REPORT_MSF_INIT[];
extern Uint8 att_HDL_HIDS_REPORT_MSF[];
extern const Uint8 ATT_HDL_HIDS_REPORT_MSF_REPORT_REFERENCE[];
extern const Uint8 ATT_HDL_HIDS_REPORT_MSF_USER_DESCRIPTION[];
//#endif      //#ifdef _PROFILE_HOGP_MOUSE_
//#ifdef _PROFILE_HOGP_KEYBOARD_
extern const Uint8 ATT_HDL_HIDS_CHARACTERISTIC_REPORT_KBI[];
extern const Uint8 ATT_HDL_HIDS_REPORT_KBI_INIT[];
extern Uint8 att_HDL_HIDS_REPORT_KBI[];
extern const Uint8 ATT_HDL_HIDS_REPORT_KBI_CLIENT_CHARACTERISTIC_CONFIGURATION_INIT[];           //Mandatory if the Battery Level characteristic properties supports notification
extern Uint8 att_HDL_HIDS_REPORT_KBI_CLIENT_CHARACTERISTIC_CONFIGURATION[];
extern const Uint8 ATT_HDL_HIDS_REPORT_KBI_REPORT_REFERENCE[];
//#endif      //#ifdef _PROFILE_HOGP_MOUSE_
#ifdef _PROFILE_HOGP_EMC_
extern const Uint8 ATT_HDL_HIDS_CHARACTERISTIC_REPORT_81[];
extern const Uint8 ATT_HDL_HIDS_REPORT_81_INIT[];
extern Uint8 att_HDL_HIDS_REPORT_81[];
extern const Uint8 ATT_HDL_HIDS_REPORT_81_CLIENT_CHARACTERISTIC_CONFIGURATION_INIT[];
extern Uint8 att_HDL_HIDS_REPORT_81_CLIENT_CHARACTERISTIC_CONFIGURATION[];
extern const Uint8 ATT_HDL_HIDS_REPORT_81_REPORT_REFERENCE[];
extern const Uint8 ATT_HDL_HIDS_CHARACTERISTIC_REPORT_82[];
extern const Uint8 ATT_HDL_HIDS_REPORT_82_INIT[];
extern Uint8 att_HDL_HIDS_REPORT_82[];
extern const Uint8 ATT_HDL_HIDS_REPORT_82_CLIENT_CHARACTERISTIC_CONFIGURATION_INIT[];
extern Uint8 att_HDL_HIDS_REPORT_82_CLIENT_CHARACTERISTIC_CONFIGURATION[];
extern const Uint8 ATT_HDL_HIDS_REPORT_82_REPORT_REFERENCE[];
extern const Uint8 ATT_HDL_HIDS_CHARACTERISTIC_REPORT_5D[];
extern const Uint8 ATT_HDL_HIDS_REPORT_5D_INIT[];
extern Uint8 att_HDL_HIDS_REPORT_5D[];
extern const Uint8 ATT_HDL_HIDS_REPORT_5D_CLIENT_CHARACTERISTIC_CONFIGURATION_INIT[];
extern Uint8 att_HDL_HIDS_REPORT_5D_CLIENT_CHARACTERISTIC_CONFIGURATION[];
extern const Uint8 ATT_HDL_HIDS_REPORT_5D_REPORT_REFERENCE[];
extern const Uint8 ATT_HDL_HIDS_CHARACTERISTIC_REPORT_05[];
extern const Uint8 ATT_HDL_HIDS_REPORT_05_INIT[];
extern Uint8 att_HDL_HIDS_REPORT_05[];
extern const Uint8 ATT_HDL_HIDS_REPORT_05_REPORT_REFERENCE[];
extern const Uint8 ATT_HDL_HIDS_CHARACTERISTIC_REPORT_08[];
extern const Uint8 ATT_HDL_HIDS_REPORT_08_INIT[];
extern Uint8 att_HDL_HIDS_REPORT_08[];
extern const Uint8 ATT_HDL_HIDS_REPORT_08_REPORT_REFERENCE[];
extern const Uint8 ATT_HDL_HIDS_CHARACTERISTIC_REPORT_09[];
extern const Uint8 ATT_HDL_HIDS_REPORT_09_INIT[];
extern Uint8 att_HDL_HIDS_REPORT_09[];
extern const Uint8 ATT_HDL_HIDS_REPORT_09_REPORT_REFERENCE[];
#endif
extern const Uint8 ATT_HDL_HIDS_CHARACTERISTIC_PROTOCOL_MODE[];
extern const Uint8 ATT_HDL_HIDS_PROTOCOL_MODE_INIT[];
extern Uint8 att_HDL_HIDS_PROTOCOL_MODE[];
//#ifdef _PROFILE_HOGP_BOOT_
//#ifdef _PROFILE_HOGP_KEYBOARD_
extern const Uint8 ATT_HDL_HIDS_CHARACTERISTIC_BOOT_KEYBOARD_INPUT_REPORT[];
extern const Uint8 ATT_HDL_HIDS_BOOT_KEYBOARD_INPUT_REPORT_INIT[];
extern Uint8 att_HDL_HIDS_BOOT_KEYBOARD_INPUT_REPORT[];
extern const Uint8 ATT_HDL_HIDS_BOOT_KEYBOARD_INPUT_REPORT_CLIENT_CHARACTERISTIC_CONFIGURATION_INIT[];                        //Mandatory if the Battery Level characteristic properties supports notification
extern Uint8 att_HDL_HIDS_BOOT_KEYBOARD_INPUT_REPORT_CLIENT_CHARACTERISTIC_CONFIGURATION[];
extern const Uint8 ATT_HDL_HIDS_CHARACTERISTIC_BOOT_KEYBOARD_OUTPUT_REPORT[];
extern const Uint8 ATT_HDL_HIDS_BOOT_KEYBOARD_OUTPUT_REPORT_INIT[];
extern Uint8 att_HDL_HIDS_BOOT_KEYBOARD_OUTPUT_REPORT[];
//#endif //#ifdef _PROFILE_HOGP_KEYBOARD_


//#ifdef _PROFILE_HOGP_MOUSE_
extern const Uint8 ATT_HDL_HIDS_CHARACTERISTIC_BOOT_MOUSE_INPUT_REPORT[];
extern const Uint8 ATT_HDL_HIDS_BOOT_MOUSE_INPUT_REPORT_INIT[];
extern Uint8 att_HDL_HIDS_BOOT_MOUSE_INPUT_REPORT[];
extern const Uint8 ATT_HDL_HIDS_BOOT_MOUSE_INPUT_REPORT_CLIENT_CHARACTERISTIC_CONFIGURATION_INIT[];                           //Mandatory if the Battery Level characteristic properties supports notification
extern Uint8 att_HDL_HIDS_BOOT_MOUSE_INPUT_REPORT_CLIENT_CHARACTERISTIC_CONFIGURATION[];
//#endif      //#ifdef _PROFILE_HOGP_MOUSE_
//#endif      //#ifdef _PROFILE_HOGP_BOOT_
#endif
























// ble_service.c
#ifdef _PROFILE_HOGP_
void noti_HIDS_REPORT(void);
#endif

#ifdef _PROFILE_LNS_
void indi_LNS_CP(void);
#endif

#ifdef _PROFILE_GLS_
void indi_GLS_CP(void);
void nodi_GLS_CP(void);
void gen_GLS_meas_rec(void);
#endif

#ifdef _PROFILE_CPS_
void indi_CPS_CP(void);
#endif

#ifdef _PROFILE_OTA_
void noti_OTA_DATA(void);
void indi_OTA_CMD(void);
#endif

#endif // _BLE_SERVICEDEF_H_

