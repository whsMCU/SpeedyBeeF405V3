/*
 * telemetry.h
 *
 *  Created on: 2020. 12. 26.
 *      Author: WANG
 */

#ifndef SRC_COMMON_CORE_TELEMETRY_H_
#define SRC_COMMON_CORE_TELEMETRY_H_

#include "def.h"

#define TX_BUFFER_SIZE 128

void TX_CHR(char ch);
void TX2_CHR(char ch);
int fputc(int ch, FILE *f);
///////////////////////////////////////////////////////////
void SerialCom(void);
void evaluateCommand(void);
void SendTelemetry(void);
void SerialSerialize(uint8_t port,uint8_t a);
void UartSendData(uint8_t port);

typedef enum serialState_t {
  IDLE,
  HEADER_START,
  HEADER_M,
  HEADER_ARROW,
  HEADER_SIZE,
  HEADER_CMD,
} serialState_t;

#define MSP_IDENT                100   //out message         multitype + multiwii version + protocol version + capability variable
#define MSP_STATUS               101   //out message         cycletime & errors_count & sensor present & box activation & current setting number
#define MSP_RAW_IMU              102   //out message         9 DOF
#define MSP_SERVO                103   //out message         8 servos
#define MSP_MOTOR                104   //out message         8 motors
#define MSP_RC                   105   //out message         8 rc chan and more
#define MSP_RAW_GPS              106   //out message         fix, numsat, lat, lon, alt, speed, ground course
#define MSP_COMP_GPS             107   //out message         distance home, direction home
#define MSP_ATTITUDE             108   //out message         2 angles 1 heading
#define MSP_ALTITUDE             109   //out message         altitude, variometer
#define MSP_ANALOG               110   //out message         vbat, powermetersum, rssi if available on RX
#define MSP_RC_TUNING            111   //out message         rc rate, rc expo, rollpitch rate, yaw rate, dyn throttle PID
#define MSP_PID                  112   //out message         P I D coeff (9 are used currently)
#define MSP_BOX                  113   //out message         BOX setup (number is dependant of your setup)
#define MSP_MISC                 114   //out message         powermeter trig
#define MSP_MOTOR_PINS           115   //out message         which pins are in use for motors & servos, for GUI
#define MSP_BOXNAMES             116   //out message         the aux switch names
#define MSP_PIDNAMES             117   //out message         the PID names
#define MSP_WP                   118   //out message         get a WP, WP# is in the payload, returns (WP#, lat, lon, alt, flags) WP#0-home, WP#16-poshold
#define MSP_BOXIDS               119   //out message         get the permanent IDs associated to BOXes
#define MSP_SERVO_CONF           120   //out message         Servo settings
#define MSP_RESET                121
#define MSP_MOBILE               122

////////////////////////////////////////////////////////////////////////////////
#define MSP_RC_RAW               150   //out message         radio channel Flexbot
#define MSP_ARM                  151
#define MSP_DISARM               152
#define MSP_TRIM_UP              153
#define MSP_TRIM_DOWN            154
#define MSP_TRIM_LEFT            155
#define MSP_TRIM_RIGHT           156

#define MSP_TRIM_UP_FAST         157
#define MSP_TRIM_DOWN_FAST       158
#define MSP_TRIM_LEFT_FAST       159
#define MSP_TRIM_RIGHT_FAST      160

#define MSP_READ_TEST_PARAM      189
#define MSP_SET_TEST_PARAM       190

#define MSP_READ_TEST_PARAM      189
#define MSP_HEX_NANO             199
////////////////////////////////////////////////////////////////////////////////

#define MSP_SET_RAW_RC           200   //in message          8 rc chan
#define MSP_SET_RAW_GPS          201   //in message          fix, numsat, lat, lon, alt, speed
#define MSP_SET_PID              202   //in message          P I D coeff (9 are used currently)
#define MSP_SET_BOX              203   //in message          BOX setup (number is dependant of your setup)
#define MSP_SET_RC_TUNING        204   //in message          rc rate, rc expo, rollpitch rate, yaw rate, dyn throttle PID
#define MSP_ACC_CALIBRATION      205   //in message          no param
#define MSP_MAG_CALIBRATION      206   //in message          no param
#define MSP_SET_MISC             207   //in message          powermeter trig + 8 free for future use
#define MSP_RESET_CONF           208   //in message          no param
#define MSP_SET_WP               209   //in message          sets a given WP (WP#,lat, lon, alt, flags)
#define MSP_SELECT_SETTING       210   //in message          Select Setting Number (0-2)
#define MSP_SET_HEAD             211   //in message          define a new heading hold direction
#define MSP_SET_SERVO_CONF       212   //in message          Servo settings
#define MSP_SET_MOTOR            214   //in message          PropBalance function

#define MSP_BIND                 240   //in message          no param

#define MSP_EEPROM_WRITE         250   //in message          no param

#define MSP_DEBUGMSG             253   //out message         debug string buffer
#define MSP_DEBUG                254   //out message         debug1,debug2,debug3,debug4
////////////////////////////////////////////////////////////////////////////////////////////////
#define TELEMERY_ERROR            1
#define TELEMERY_ARMED_MODE       2
#define TELEMERY_HEADFREE_MODE    3
#define TELEMERY_CYCLE_TIME       4
#define TELEMERY_BAT_VOLT         5
#define TELEMERY_TEMPERATURE      6
#define TELEMERY_ANGLE_ROLL       7
#define TELEMERY_ANGLE_PITCH      8
#define TELEMERY_ANGLE_YAW        9
#define TELEMERY_HEADING          10
#define TELEMERY_ACC_ROLL         11
#define TELEMERY_ACC_PITCH        12
#define TELEMERY_ACC_YAW          13
#define TELEMERY_GYRO_ROLL        14
#define TELEMERY_GYRO_PITCH       15
#define TELEMERY_GYRO_YAW         16
#define TELEMERY_MAG_ROLL         17
#define TELEMERY_MAG_PITCH        18
#define TELEMERY_MAG_YAW          19
#define TELEMERY_ARMD_TIME        20
#define TELEMERY_BARO_EST         21
#define TELEMERY_PID_RP_P         22
#define TELEMERY_PID_RP_I         23
#define TELEMERY_PID_RP_D         24
#define TELEMERY_PID_Y_P          25
#define TELEMERY_PID_Y_I          26
#define TELEMERY_PID_Y_D          27
#define TELEMERY_NUM_SATS         28
#define TELEMERY_FIX_TYPE         29
#define TELEMERY_GPS_LAT          30
#define TELEMERY_GPS_LON          31
#define TELEMERY_RADIO_ROLL       32
#define TELEMERY_RADIO_PITCH      33
#define TELEMERY_RADIO_YAW        34
#define TELEMERY_RADIO_THROTTLE   35
#define TELEMERY_RADIO_GEAR       36
#define TELEMERY_RADIO_AUX1       37
#define TELEMERY_MOTOR_1          38
#define TELEMERY_MOTOR_2          39
#define TELEMERY_MOTOR_3          40
#define TELEMERY_MOTOR_4          41

#define TELEMERY_PIDSET_RP_P      50
#define TELEMERY_PIDSET_RP_I      51
#define TELEMERY_PIDSET_RP_D      52
#define TELEMERY_PIDSET_Y_P       53
#define TELEMERY_PIDSET_Y_I       54
#define TELEMERY_PIDSET_Y_D       55
#define TELEMERY_PID_SAVE         56

#endif /* SRC_COMMON_CORE_TELEMETRY_H_ */
