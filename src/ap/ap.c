/*
 * ap.c
 *
 *  Created on: Dec 6, 2020
 *      Author: baram
 */


#include "ap.h"
#include "uart.h"
#include "scheduler.h"
#include "tasks.h"

void apInit(void)
{
	cliOpen(_DEF_USB, 57600);
	// Initialize task data as soon as possible. Has to be done before tasksInit(),
    // and any init code that may try to modify task behaviour before tasksInit().
    tasksInitData();

	Sensor_Init();
	Baro_Init();
	tasksInit();
}

void apMain(void)
{
	while(1)
	{
		scheduler();
	}
	// 	if (millis()-pre_time >= 1000)
    // 	{
    //  		pre_time = millis();
    //   		ledToggle(_DEF_LED1);
    // 	}
	// 	if (micros()-pre_time1 >= 312)
    // 	{
    //  		pre_time1 = micros();
	// 		imuUpdate();	//DT 170us
	// 		baroUpdate(micros());
    // 	}
	// 	if (millis()-pre_time2 >= 50)
    // 	{
    //  		pre_time2 = millis();
	// 		baroUpdate(micros());
	// 		calculateEstimatedAltitude(micros());
    // 	}
	// 	cliMain();
	// }
}
