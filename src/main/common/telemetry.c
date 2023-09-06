/*
 * telemery.c
 *
 *  Created on: 2020. 12. 26.
 *      Author: WANG
 */


#include "telemetry.h"
#include "uart.h"
#include "led.h"

#include "barometer.h"
#include "sensors.h"


char Buf[128];

// extern imu_t imu;
// extern pidc_t pid;
// extern rc RC;
// extern rc RC_Raw;

// extern att_t att;
// extern LPS_t lps;
// extern ms5611_t ms5611;
// extern alt_t alt;
// extern flags_t f;
// extern gps_t GPS;
// extern eeror_t Error;
// extern int16_t M_motor[4];
// extern uint32_t time_manual_motor;

// extern PID_PARAM posholdPID_PARAM;
// extern PID_PARAM poshold_ratePID_PARAM;
// extern PID_PARAM navPID_PARAM;
// extern int32_t  GPS_coord[2];

// extern float magBias[3], magScale[3];

// extern uint8_t rcOptions[CHECKBOXITEMS];


// extern uint32_t loopTime;
// extern uint16_t cycleTimeMax;
// extern uint16_t cycleTimeMin;
// extern uint32_t armedTime;
// extern uint16_t calibratingA;

extern int16_t motor[4];

static volatile uint8_t serialHeadTX[UART_MAX_CH],serialTailTX[UART_MAX_CH];
static uint8_t serialBufferTX[TX_BUFFER_SIZE][UART_MAX_CH];
static uint8_t serialBufTx_0[TX_BUFFER_SIZE];
static uint8_t serialBufTx_1[TX_BUFFER_SIZE];
static volatile uint8_t serialHead_0, serialHead_1;
static uint8_t CURRENTPORT=0;

volatile unsigned char command=0;
volatile unsigned char m = 0;
int MSP_TRIM[3]={0, 0, 0};

extern uint8_t Manual_Motor_flag;

uint8_t telemetry_loop_counter = 0;
uint16_t time=0, time1=0, aftertime=0;
uint16_t debug[4]={0,0,0,0};

uint8_t GPS_virtual=0;

////////////////////////////////////////////

uint8_t rx1_buffer[1];
uint8_t rx2_buffer[1];

//////////// MSP //////////////////
#define INBUF_SIZE 128
typedef struct mspPortState_t {
	//    serialPort_t *port;
	uint8_t checksum;
	uint8_t indRX;
	uint8_t inBuf[INBUF_SIZE];
	uint8_t cmdMSP;
	uint8_t offset;
	uint8_t dataSize;
	serialState_t c_state;
} mspPortState_t;

static mspPortState_t ports[2];
static mspPortState_t *currentPortState = &ports[0];

static void serialize8(uint8_t a);
static void serialize16(int16_t a);
static void serialize32(uint32_t a);
static uint8_t read8(void);
static uint16_t read16(void);
static uint32_t read32(void);
static void headSerial(uint8_t err, uint8_t s, uint8_t cmdMSP);
static void headSerialSend(uint8_t s, uint8_t cmdMSP);
static void headSerialResponse(uint8_t err, uint8_t s);
static void headSerialReply(uint8_t s);
static void headSerialError(uint8_t s);
static void tailSerialReply(void);
static void s_struct_partial(uint8_t *cb,uint8_t siz);
static void s_struct(uint8_t *cb,uint8_t siz);


void serialize8(uint8_t a)
{
	SerialSerialize(CURRENTPORT,a);
	currentPortState->checksum ^= (a & 0xFF);
}

void serialize16(int16_t a)
{
	serialize8((a   ) & 0xFF);
	serialize8((a>>8) & 0xFF);
}

void serialize32(uint32_t a)
{
	serialize8((a    ) & 0xFF);
	serialize8((a>> 8) & 0xFF);
	serialize8((a>>16) & 0xFF);
	serialize8((a>>24) & 0xFF);
}

uint8_t read8(void)
{
	return currentPortState->inBuf[currentPortState->indRX++] & 0xff;
}

uint16_t read16(void)
{
	uint16_t t = read8();
	t += (uint16_t)read8() << 8;
	return t;
}

uint32_t read32(void)
{
	uint32_t t = read16();
	t += (uint32_t)read16() << 16;
	return t;
}

void headSerial(uint8_t err, uint8_t s, uint8_t cmdMSP)
{
	serialize8('$');
	serialize8('M');
	serialize8(err ? '!' : '>');
	currentPortState->checksum = 0;               // start calculating a new checksum
	serialize8(s);
	serialize8(cmdMSP);
}

void headSerialSend(uint8_t s, uint8_t cmdMSP)
{
	headSerial(0, s, cmdMSP);
}

void headSerialResponse(uint8_t err, uint8_t s)
{
	serialize8('$');
	serialize8('M');
	serialize8(err ? '!' : '>');
	currentPortState->checksum = 0;               // start calculating a new checksum
	serialize8(s);
	serialize8(currentPortState->cmdMSP);
}

void headSerialReply(uint8_t s)
{
	headSerialResponse(0, s);
}

void headSerialError(uint8_t s)
{
	headSerialResponse(1, s);
}

void tailSerialReply(void)
{
	SerialSerialize(CURRENTPORT,currentPortState->checksum);
	UartSendData(CURRENTPORT);
	//serialize8(currentPortState->checksum);
}

void s_struct_partial(uint8_t *cb,uint8_t siz)
{
	while(siz--) serialize8(*cb++);
}

void s_struct(uint8_t *cb,uint8_t siz)
{
	headSerialReply(siz);
	s_struct_partial(cb,siz);
	tailSerialReply();
}
///////////////////////////////////////////////////

void SerialCom(void)
{
	uint8_t c;
	uint32_t timeMax; // limit max time in this function in case of GPS
	timeMax = micros();
	int i = 1;
#ifndef _USE_HW_CLI
	for(int i = 0; i < 2; i++)
	{
#endif
		currentPortState = &ports[i];
		CURRENTPORT = i;
		while(uartAvailable(CURRENTPORT) > 0)
		{
			c = uartRead(CURRENTPORT);
			if (currentPortState->c_state == IDLE)
			{
				currentPortState->c_state = (c=='$') ? HEADER_START : IDLE;
			} else if (currentPortState->c_state == HEADER_START)
			{
				currentPortState->c_state = (c=='M') ? HEADER_M : IDLE;
			} else if (currentPortState->c_state == HEADER_M)
			{
				currentPortState->c_state = (c=='<') ? HEADER_ARROW : IDLE;
			} else if (currentPortState->c_state == HEADER_ARROW)
			{
				if (c > INBUF_SIZE)
				{  // now we are expecting the payload size
					currentPortState->c_state = IDLE;
					continue;
				}
				currentPortState->dataSize = c;
				currentPortState->offset = 0;
				currentPortState->indRX = 0;
				currentPortState->checksum = 0;
				currentPortState->checksum ^= c;
				currentPortState->c_state = HEADER_SIZE;
			} else if (currentPortState->c_state == HEADER_SIZE)
			{
				currentPortState->cmdMSP = c;
				currentPortState->checksum ^= c;
				currentPortState->c_state = HEADER_CMD;
			} else if (currentPortState->c_state == HEADER_CMD && currentPortState->offset < currentPortState->dataSize)
			{
				currentPortState->checksum ^= c;
				currentPortState->inBuf[currentPortState->offset++] = c;
			} else if (currentPortState->c_state == HEADER_CMD && currentPortState->offset >= currentPortState->dataSize)
			{
				if (currentPortState->checksum == c)
				{
					evaluateCommand();
				}
				currentPortState->c_state = IDLE;
			}
#ifdef GPS_Recive
			if(i == _DEF_UART1||GPS_virtual)
			{
				static uint32_t GPS_last_frame_seen; //Last gps frame seen at this time, used to detect stalled gps communication
				if (GPS_newFrame(c)||GPS_virtual)
				{

					//We had a valid GPS data frame, so signal task scheduler to switch to compute
					if (GPS.GPS_update == 1) GPS.GPS_update = 0; else GPS.GPS_update = 1; //Blink GPS update
					GPS_last_frame_seen = timeMax;
					GPS.GPS_Frame = 1;
				}
				// Check for stalled GPS, if no frames seen for 1.2sec then consider it LOST
				if ((timeMax - GPS_last_frame_seen) > 1200000)
				{
					//No update since 1200ms clear fix...
					f.GPS_FIX = 0;
					GPS.fixquality = 0;
					GPS.satellites = 0;
				}
			}
			if (micros()-timeMax>250) return;  // Limit the maximum execution time of serial decoding to avoid time spike
#endif
		}
#ifndef _USE_HW_CLI
	}
#endif
}

void evaluateCommand(void)
{
	uint8_t i=0;
	uint32_t tmp=0;
	switch(currentPortState->cmdMSP)
	{
		case MSP_ARM:
			//mwArm();
			break;

		case MSP_DISARM:
			//mwDisarm();
			break;

		case MSP_RC_RAW:
			for(i=0; i < 5; i++)
			{
				// RC_Raw.rcCommand[i]  = read8();
			}
			//RC.rcCommand[ROLL]     = map(RC_Raw.rcCommand[ROLL], 0, 250, -20, 20)+ MSP_TRIM[ROLL]; //0~250 left:0, right:250
			// RC.rcCommand[PITCH]    = map(RC_Raw.rcCommand[PITCH], 0, 250, -20, 20)+ MSP_TRIM[PITCH]; //0~250 rear:0, fornt:250
			// RC.rcCommand[YAW]      = map(RC_Raw.rcCommand[YAW], 0, 250, -100, 100); //0~250 left:0, right:250
			// RC.rcCommand[THROTTLE] = map(RC_Raw.rcCommand[THROTTLE], 0, 250, 0, 1800);//0~250
			// RC.rcCommand[AUX1] 	   =  RC_Raw.rcCommand[GEAR];
			break;

		case MSP_RC:
		{  //struct {
		// 	uint16_t roll, pitch, yaw, throttle, gear, aux1;
		// } rc;
		// rc.roll     = RC.rcCommand[ROLL];
		// rc.pitch    = RC.rcCommand[PITCH];
		// rc.yaw      = RC.rcCommand[YAW];
		// rc.throttle = RC.rcCommand[THROTTLE];
		// rc.aux1     = RC.rcCommand[AUX1];
		// rc.gear     = RC.rcCommand[GEAR];
		//s_struct((uint8_t*)&rc, 12);
		break;
		}

		case MSP_STATUS:
		{// struct {
		// 	uint32_t ArmedTime;
		// 	uint32_t cycleTime;
		// 	uint8_t error, flag;
		// } st;
		// st.ArmedTime    = armedTime;
		// st.cycleTime    = loopTime;
		// st.error        = Error.error;
		// if(f.ARMED) tmp |= 1<<BOXARM;
		// if(f.HEADFREE_MODE) tmp |= 1<<BOXHEADFREE;
		// st.flag         = tmp;
		// s_struct((uint8_t*)&st,10);
		break;
		}

		case MSP_ATTITUDE:
			//s_struct((uint8_t*)&att,8);
			break;

		case MSP_ALTITUDE:
		{ //struct {
		// 	int16_t alt;
		// } tmp;
		// tmp.alt = (int16_t) alt.EstAlt;
		// s_struct((uint8_t*)&tmp,2);
		break;
		}

		case MSP_MISC:
		{ //struct {
		// 	uint16_t roll, pitch, yaw, throttle, gear, aux1; //12
		// 	uint32_t ArmedTime; //16
		// 	uint32_t cycleTime; //20
		// 	uint8_t error, flag; //22
		// 	int16_t angle[2];    //26
		// 	int16_t heading;     //28
		// 	int16_t mag_heading; //30
		// 	int16_t alt;  //38
		// 	int16_t VBAT;//32
		// 	int16_t Temp; //34
		// 	int16_t acc[3]; //44
		// 	int16_t gyro[3]; //50
		// 	int16_t mag[3]; //56
		// 	uint8_t a,b; //58
		// 	int32_t c,d; //66
		// 	int16_t e;
		// 	uint16_t f;
		// 	int16_t motor[4];//74
		// 	int16_t debug_t[4];//82
		// } tele;
		// tele.roll     = RC.rcCommand[ROLL];
		// tele.pitch    = RC.rcCommand[PITCH];
		// tele.yaw      = RC.rcCommand[YAW];
		// tele.throttle = RC.rcCommand[THROTTLE];
		// tele.aux1     = RC.rcCommand[AUX1];
		// tele.gear     = RC.rcCommand[GEAR];
		// tele.ArmedTime    = armedTime;
		// tele.cycleTime    = loopTime;
		// tele.error        = Error.error;
		// if(f.ARMED) tmp |= 1<<BOXARM;
		// if(f.HEADFREE_MODE) tmp |= 1<<BOXHEADFREE;
		// if(f.ACRO_MODE) tmp |= 1<<BOXACRO_MODE;
		// if(f.ANGLE_MODE) tmp |= 1<<BOXANGLE_MODE;
		// if(f.CALIBRATE_ACC) tmp |= 1<<BOXCALIBRATE_ACC;
		// if(f.CALIBRATE_MAG) tmp |= 1<<BOXCALIBRATE_MAG;
		// if(f.GPS_HOLD_MODE) tmp |= 1<<BOXGPS_MODE;
		// tele.flag         = tmp;
		// tele.angle[ROLL] = (int16_t) imu.AHRS[ROLL] * 10;
		// tele.angle[PITCH] = (int16_t) imu.AHRS[PITCH] * 10;
		// tele.heading = (int16_t) imu.gyroRaw[YAW];
		// tele.mag_heading = (int16_t) imu.actual_compass_heading;
		// tele.alt = (int16_t) alt.EstAlt;
		// tele.VBAT = (int16_t) 100;//BAT.VBAT;
		// tele.Temp = (int16_t) imu.Temp*10;
		// for(uint8_t axis=0; axis<3;axis++)
		// {
		// 	tele.acc[axis]  = (int16_t) imu.accSmooth[axis];
		// 	tele.gyro[axis] = (int16_t) imu.gyroRaw[axis];
		// 	tele.mag[axis]  = (int16_t) imu.magSmooth[axis];
		// }
		// tele.a     = GPS.fixquality;
		// tele.b     = GPS.satellites;
		// tele.c     = GPS_coord[LAT];
		// tele.d     = GPS_coord[LON];
		// tele.e     = GPS.altitude;
		// tele.f     = GPS.speed;
		// tele.motor[0] = motor[0];
		// tele.motor[1] = motor[1];
		// tele.motor[2] = motor[2];
		// tele.motor[3] = motor[3];
		// tele.debug_t[0] = debug[0];
		// tele.debug_t[1] = debug[1];
		// tele.debug_t[2] = debug[2];
		// tele.debug_t[3] = debug[3];
		// s_struct((uint8_t*)&tele, 86);
		break;
		}

		case MSP_RAW_IMU:
		{ //struct {
		// 	int16_t acc[3];
		// 	int16_t gyro[3];
		// 	int16_t mag[3];
		// } mpu;
		// for(uint8_t axis=0; axis<3;axis++)
		// {
		// 	mpu.acc[axis]  = (int16_t) map(imu.accADC[axis], -32768, 32768, -1000, 1000);
		// 	mpu.gyro[axis] = (int16_t) imu.gyroRaw[axis];
		// 	mpu.mag[axis]  = (int16_t) imu.magRaw[axis];
		// }
		// s_struct((uint8_t*)&mpu,18);
		break;
		}

		case MSP_RAW_GPS:
		{ //struct {
		// 	uint8_t a,b;
		// 	int32_t c,d;
		// 	//	        int16_t e;
		// 	//	        uint16_t f,g;
		// } msp_raw_gps;
		// msp_raw_gps.a     = GPS.fixquality;
		// msp_raw_gps.b     = GPS.satellites;
		// msp_raw_gps.c     = GPS.GPS_coord[LAT];
		// msp_raw_gps.d     = GPS.GPS_coord[LON];
		// //msp_raw_gps.e     = GPS_altitude;
		// //msp_raw_gps.f     = GPS_speed;
		// //msp_raw_gps.g     = GPS_ground_course;
		// s_struct((uint8_t*)&msp_raw_gps,10);
		break;
		}

		case MSP_MOTOR:
			// s_struct((uint8_t*)&motor,8);
			break;

		case MSP_PID:
		{ //struct {
		// 	uint16_t GPS_P[3];
		// 	uint16_t outer_ROLL[2];
		// 	uint16_t inner_ROLL[3];
		// 	uint16_t ROLL_rate[3];

		// 	uint16_t GPS_I[3];
		// 	uint16_t outer_PITCH[2];
		// 	uint16_t inner_PITCH[3];
		// 	uint16_t PITCH_rate[3];

		// 	uint16_t GPS_D[3];
		// 	uint16_t outer_YAW[2];
		// 	uint16_t inner_YAW[3];
		// 	uint16_t YAW_rate[3];
		// } pid_t;


		// pid_t.GPS_P[0]  = (int16_t) (posholdPID_PARAM.kP  * 100);
		// pid_t.GPS_P[1]  = (int16_t) (poshold_ratePID_PARAM.kP  * 100);
		// pid_t.GPS_P[2]  = (int16_t) (navPID_PARAM.kP  * 100);
		// pid_t.GPS_I[0]  = (int16_t) (posholdPID_PARAM.kI * 100);
		// pid_t.GPS_I[1]  = (int16_t) (poshold_ratePID_PARAM.kI * 100);
		// pid_t.GPS_I[2]  = (int16_t) (navPID_PARAM.kI * 100);
		// pid_t.GPS_D[0]  = (int16_t) (poshold_ratePID_PARAM.kD  * 1000);
		// pid_t.GPS_D[1]  = (int16_t) (navPID_PARAM.kD   * 1000);
		// pid_t.GPS_D[2]  = (int16_t) (posholdPID_PARAM.Imax);

		// pid_t.outer_ROLL[0] = (int16_t) (pid.kp1[ROLL] * 10);
		// pid_t.outer_ROLL[1] = (int16_t) (pid.ki1[ROLL] * 10);
		// pid_t.outer_PITCH[0] = (int16_t) (pid.kp1[PITCH] * 10);
		// pid_t.outer_PITCH[1] = (int16_t) (pid.ki1[PITCH] * 10);
		// pid_t.outer_YAW[0] = (int16_t) (pid.kp1[YAW] * 10);
		// pid_t.outer_YAW[1] = (int16_t) (pid.ki1[YAW] * 10);

		// pid_t.inner_ROLL[0] = (int16_t) (pid.kp2[ROLL] * 10);
		// pid_t.inner_ROLL[1] = (int16_t) (pid.ki2[ROLL] * 10);
		// pid_t.inner_ROLL[2] = (int16_t) (pid.kd2[ROLL] * 100);
		// pid_t.inner_PITCH[0] = (int16_t) (pid.kp2[PITCH] * 10);
		// pid_t.inner_PITCH[1] = (int16_t) (pid.ki2[PITCH] * 10);
		// pid_t.inner_PITCH[2] = (int16_t) (pid.kd2[PITCH] * 100);
		// pid_t.inner_YAW[0] = (int16_t) (pid.kp2[YAW] * 10);
		// pid_t.inner_YAW[1] = (int16_t) (pid.ki2[YAW] * 10);
		// pid_t.inner_YAW[2] = (int16_t) (pid.kd2[YAW] * 100);

		// pid_t.ROLL_rate[0]  = (int16_t) (pid.kp_rate[ROLL]  * 10);
		// pid_t.ROLL_rate[1]  = (int16_t) (pid.ki_rate[ROLL]  * 10);
		// pid_t.ROLL_rate[2]  = (int16_t) (pid.kd_rate[ROLL]  * 100);
		// pid_t.PITCH_rate[0] = (int16_t) (pid.kp_rate[PITCH] * 10);
		// pid_t.PITCH_rate[1] = (int16_t) (pid.ki_rate[PITCH] * 10);
		// pid_t.PITCH_rate[2] = (int16_t) (pid.kd_rate[PITCH] * 100);
		// pid_t.YAW_rate[0]   = (int16_t) (pid.kp_rate[YAW]   * 10);
		// pid_t.YAW_rate[1]   = (int16_t) (pid.ki_rate[YAW]   * 10);
		// pid_t.YAW_rate[2]   = (int16_t) (pid.kd_rate[YAW]   * 100);

		// s_struct((uint8_t*)&pid_t,66);

		break;
		}

		case MSP_ANALOG:
		{ //struct {
		// 	uint16_t VBAT;
		// 	uint16_t Temp;
		// } analog;

		// analog.VBAT = 100;//BAT.VBAT;
		// analog.Temp = (imu.Temp*10);

		// s_struct((uint8_t*)&analog,4);
		break;
		}

		// case MSP_SET_PID:
		// 	posholdPID_PARAM.kP   = 0.15f;
		// 	posholdPID_PARAM.kI   = 0;
		// 	posholdPID_PARAM.Imax = 2000;

		// 	poshold_ratePID_PARAM.kP   = 3.4f;
		// 	poshold_ratePID_PARAM.kI   = 0.14f;
		// 	poshold_ratePID_PARAM.kD   = 0.053f;
		// 	poshold_ratePID_PARAM.Imax = 2000;

		// 	navPID_PARAM.kP   = 2.5f;
		// 	navPID_PARAM.kI   = 0.33f;
		// 	navPID_PARAM.kD   = 0.053f;
		// 	navPID_PARAM.Imax = 2000;

		// 	for(i=0; i < 3; i++)
		// 	{
		// 		if(i==0)
		// 		{
		// 			posholdPID_PARAM.kP   = (float) read16();
		// 			posholdPID_PARAM.kP /= 100;

		// 			posholdPID_PARAM.kI = (float) read16();
		// 			posholdPID_PARAM.kI /= 100;

		// 			posholdPID_PARAM.Imax = (float) read16();
		// 			poshold_ratePID_PARAM.Imax = posholdPID_PARAM.Imax;
		// 			navPID_PARAM.Imax = posholdPID_PARAM.Imax;
		// 		}else if(i == 1)
		// 		{
		// 			poshold_ratePID_PARAM.kP = (float) read16();
		// 			poshold_ratePID_PARAM.kP /= 100;

		// 			poshold_ratePID_PARAM.kI = (float) read16();
		// 			poshold_ratePID_PARAM.kI /= 100;

		// 			poshold_ratePID_PARAM.kD = (float) read16();
		// 			poshold_ratePID_PARAM.kD /= 1000;


		// 		}else if(i == 2)
		// 		{
		// 			navPID_PARAM.kP = (float) read16();
		// 			navPID_PARAM.kP /= 100;

		// 			navPID_PARAM.kI = (float) read16();
		// 			navPID_PARAM.kI /= 100;

		// 			navPID_PARAM.kD = (float) read16();
		// 			navPID_PARAM.kD /= 1000;
		// 		}

		// 		pid.kp1[i] = (float) read16();
		// 		pid.kp1[i] /= 10;
		// 		pid.ki1[i] = (float) read16();
		// 		pid.ki1[i] /= 10;
		// 		pid.kp2[i] = (float) read16();
		// 		pid.kp2[i] /= 10;
		// 		pid.ki2[i] = (float) read16();
		// 		pid.ki2[i] /= 10;
		// 		pid.kd2[i] = (float) read16();
		// 		pid.kd2[i] /= 100;

		// 		pid.kp_rate[i] = (float) read16();
		// 		pid.kp_rate[i] /= 10;
		// 		pid.ki_rate[i] = (float) read16();
		// 		pid.ki_rate[i] /= 10;
		// 		pid.kd_rate[i] = (float) read16();
		// 		pid.kd_rate[i] /= 100;
		// 	}
		// 	break;

		// case MSP_SET_MOTOR:
		// 	M_motor[0] = read16();
		// 	M_motor[1] = read16();
		// 	M_motor[2] = read16();
		// 	M_motor[3] = read16();
		// 	Manual_Motor_flag = true;
		// 	time_manual_motor = micros();
		// 	break;

		// case MSP_RESET:
		// 	Error.error = 0;
		// 	ledOff(RGB_RED);
		// 	cycleTimeMax = 0;
		// 	cycleTimeMin = 65535;
		// 	f.mag_reset = 1;
		// 	ledToggle(RGB_GREEN);
		// 	break;

		// case MSP_MOBILE:
		// { struct {
		// 	uint16_t roll, pitch, yaw, throttle, gear, aux1; //12
		// 	//uint32_t ArmedTime; //4
		// 	uint32_t cycleTime; //8
		// 	uint16_t error, flag; //9
		// 	int16_t alt;
		// 	int16_t VBAT;//11
		// 	int16_t Temp; //13
		// 	int16_t angle[2];//17
		// 	int16_t mag_heading;//19
		// 	int16_t motor[4];//74
		// } debug_t;

		// debug_t.roll     = RC.rcCommand[ROLL];
		// debug_t.pitch    = RC.rcCommand[PITCH];
		// debug_t.yaw      = RC.rcCommand[YAW];
		// debug_t.throttle = RC.rcCommand[THROTTLE];
		// debug_t.aux1     = RC.rcCommand[AUX1];
		// debug_t.gear     = RC.rcCommand[GEAR];
		// //debug_t.ArmedTime    = armedTime;
		// debug_t.cycleTime    = loopTime;
		// debug_t.error        = Error.error;
		// if(f.ARMED) tmp |= 1<<BOXARM;
		// if(f.HEADFREE_MODE) tmp |= 1<<BOXHEADFREE;
		// if(f.ACRO_MODE) tmp |= 1<<BOXACRO_MODE;
		// if(f.ANGLE_MODE) tmp |= 1<<BOXANGLE_MODE;
		// if(f.GPS_HOLD_MODE) tmp |= 1<<BOXGPS_MODE;
		// if(f.CALIBRATE_ACC) tmp |= 1<<BOXCALIBRATE_ACC;
		// if(f.CALIBRATE_MAG) tmp |= 1<<BOXCALIBRATE_MAG;
		// debug_t.flag         = tmp;
		// debug_t.alt = (int16_t) alt.EstAlt;
		// debug_t.VBAT = 100;//BAT.VBAT;
		// debug_t.Temp = imu.Temp*10;

		// debug_t.angle[ROLL] = imu.AHRS[ROLL]*10;
		// debug_t.angle[PITCH] = imu.AHRS[PITCH]*10;
		// debug_t.mag_heading = (int16_t)imu.actual_compass_heading*10;

		// debug_t.motor[0] = motor[0];
		// debug_t.motor[1] = motor[1];
		// debug_t.motor[2] = motor[2];
		// debug_t.motor[3] = motor[3];

		// s_struct((uint8_t*)&debug_t, 41);
		// }
		// break;

		// case MSP_ACC_CALIBRATION:
		// 	if(!f.ARMED)
		// 	{
		// 		calibratingA=512;
		// 		f.CALIBRATE_ACC = 1;
		// 	}
		// 	break;

		// case MSP_MAG_CALIBRATION:
		// 	if(!f.ARMED)
		// 	{
		// 		f.CALIBRATE_MAG=!f.CALIBRATE_MAG;
		// 	}
		// 	break;

		// case MSP_TRIM_UP:
		// 	MSP_TRIM[PITCH] += 1;
		// 	break;

		// case MSP_TRIM_DOWN:
		// 	MSP_TRIM[PITCH] -= 1;
		// 	break;

		// case MSP_TRIM_LEFT:
		// 	MSP_TRIM[ROLL] -= 1;
		// 	break;

		// case MSP_TRIM_RIGHT:
		// 	MSP_TRIM[ROLL] += 1;
		// 	break;

		// case TELEMERY_PID_SAVE:
		// 	ledToggle(RGB_BLUE);
		// 	writeFloat(0, posholdPID_PARAM.kP);
		// 	writeFloat(4, posholdPID_PARAM.kI);
		// 	writeFloat(8, poshold_ratePID_PARAM.kP);
		// 	writeFloat(12, poshold_ratePID_PARAM.kI);
		// 	writeFloat(16, poshold_ratePID_PARAM.kD);
		// 	writeFloat(20, navPID_PARAM.kP);
		// 	writeFloat(24, navPID_PARAM.kI);
		// 	writeFloat(28, navPID_PARAM.kD);
		// 	writeFloat(32, posholdPID_PARAM.Imax);
		// 	for(int i = 0; i < 3; i++){
		// 		writeFloat( 36+(4*i), pid.kp1[i]);
		// 		writeFloat( 48+(4*i), pid.ki1[i]);
		// 		writeFloat( 60+(4*i), pid.kp2[i]);
		// 		writeFloat( 72+(4*i), pid.ki2[i]);
		// 		writeFloat( 84+(4*i), pid.kd2[i]);

		// 		writeFloat( 96+(4*i), pid.kp_rate[i]);
		// 		writeFloat(108+(4*i), pid.ki_rate[i]);
		// 		writeFloat(120+(4*i), pid.kd_rate[i]);
		// 	}
		// 	writeFloat(132, magBias[0]);
		// 	writeFloat(136, magBias[1]);
		// 	writeFloat(140, magBias[2]);
		// 	writeFloat(144, magScale[0]);
		// 	writeFloat(148, magScale[1]);
		// 	writeFloat(152, magScale[2]);
		// 	break;

		default:
			//headSerialError();
			//tailSerialReply();
			break;
	}

}

void SerialSerialize(uint8_t port,uint8_t a) {
	uint8_t t = serialHeadTX[port];
	if (++t >= TX_BUFFER_SIZE) t = 0;
	serialBufferTX[t][port] = a;
	serialHeadTX[port] = t;
}

void UartSendData(uint8_t port)
{
	uint8_t t = serialTailTX[port];
	switch(port){
		case _DEF_UART1:
			while (serialHeadTX[port] != t)
			{
				if (++t >= TX_BUFFER_SIZE) t = 0;
				serialBufTx_0[serialHead_0++] = serialBufferTX[t][port];
			}
			serialTailTX[port] = t;
			uartWriteIT(_DEF_UART1, serialBufTx_0, serialHead_0);
			serialHead_0 = 0;
			break;

		case _DEF_UART2:
			while (serialHeadTX[port] != t)
			{
				if (++t >= TX_BUFFER_SIZE) t = 0;
				serialBufTx_1[serialHead_1++] = serialBufferTX[t][port];
			}
			serialTailTX[port] = t;
			uartWriteIT(_DEF_UART2, serialBufTx_1, serialHead_1);
			serialHead_1 = 0;
			break;
	}
}
