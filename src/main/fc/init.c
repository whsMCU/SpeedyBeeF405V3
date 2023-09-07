/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

#include "build/build_config.h"

#include "common/axis.h"
#include "common/color.h"
#include "common/maths.h"
#include "common/printf_serial.h"

#include "config/config.h"
#include "config/feature.h"

#include "drivers/accgyro/accgyro.h"
#include "drivers/compass/compass.h"
#include "drivers/sensor.h"
#ifdef USE_USB_MSC
#include "drivers/usb_msc.h"
#endif
//#include "drivers/vtx_common.h"
//#include "drivers/vtx_rtc6705.h"
//#include "drivers/vtx_table.h"

#include "fc/board_info.h"
#include "fc/dispatch.h"
#include "fc/init.h"
#include "fc/core.h"
#include "fc/rc_modes.h"
#include "fc/rc_controls.h"
#include "fc/runtime_config.h"
#include "fc/stats.h"
#include "fc/controlrate_profile.h"

#include "scheduler/tasks.h"

//#include "flight/failsafe.h"
#include "flight/imu.h"
#include "flight/mixer_init.h"
#include "flight/mixer.h"
#include "flight/pid.h"
#include "flight/pid_init.h"
#include "flight/position.h"
//#include "flight/servos.h"

//#include "msc/emfat_file.h"
#ifdef USE_PERSISTENT_MSC_RTC
#include "msc/usbd_storage.h"
#endif

//#include "msp/msp.h"
//#include "msp/msp_serial.h"

#include "osd/osd.h"

#include "rx/rx.h"
//#include "rx/spektrum.h"

#include "scheduler/scheduler.h"
#include "scheduler/tasks.h"

#include "sensors/sensors.h"
#include "sensors/acceleration.h"
#include "sensors/barometer.h"
#include "sensors/battery.h"
#include "sensors/adcinternal.h"
#include "sensors/boardalignment.h"
#include "sensors/compass.h"
//#include "sensors/esc_sensor.h"
#include "sensors/gyro.h"
#include "sensors/gyro_init.h"
//#include "sensors/initialisation.h"

#include "telemetry/telemetry.h"

#include "drivers/gps/gps.h"

#include "rx/rx.h"

uint8_t systemState = SYSTEM_STATE_INITIALISING;

static void Param_Config_Init(void);

void init(void)
{
	/////////////// LED //////////////////
		ledOn(ST1);
		for (int i = 0; i < 10; i++)
		{
			ledToggle(ST1);
			#if defined(USE_BEEPER)
				HAL_Delay(25);
				BEEP_ON;
				HAL_Delay(25);
				BEEP_OFF;
			#else
				HAL_Delay(50);
			#endif
		}
		ledOff(ST1);
	////////////////////////////////////////

    // Initialize task data as soon as possible. Has to be done before tasksInit(),
    // and any init code that may try to modify task behaviour before tasksInit().
	tasksInitData();
	Param_Config_Init();

	cliOpen(_DEF_USB, 57600);

	readEEPROM();

	mixerInit(mixerConfig.mixerMode);

	initBoardAlignment(&boardAlignment);

	Sensor_Init();
	Baro_Init();
	compassInit();
	adcInternalInit();

    // Set the targetLooptime based on the detected gyro sampleRateHz and pid_process_denom
    gyroSetTargetLooptime(pidConfig.pid_process_denom);

    // Validate and correct the gyro config or PID loop time if needed
    validateAndFixGyroConfig();

    // Now reset the targetLooptime as it's possible for the validation to change the pid_process_denom
    gyroSetTargetLooptime(pidConfig.pid_process_denom);

    // Finally initialize the gyro filtering
	gyroInitFilters();
	pidInit(currentPidProfile);
	mixerInitProfile();

	imuInit();
    //failsafeInit();

    rxInit();
	gpsInit();

    gyroStartCalibration(false);
    baroStartCalibration();

//    if (featureIsEnabled(FEATURE_TELEMETRY)) {
//        telemetryInit();
//    }

    //setArmingDisabled(ARMING_DISABLED_BOOT_GRACE_TIME);

	batteryInit(); // always needs doing, regardless of features.

	tasksInit();
	MSP_SET_MODE_RANGE(0, 0, 0, 1700, 2100);
	MSP_SET_MODE_RANGE(1, 1, 0, 1700, 2100);
	MSP_SET_MODE_RANGE(2, 6, 1, 1700, 2100);
	MSP_SET_MODE_RANGE(3, 27, 4, 1700, 2100);
	MSP_SET_MODE_RANGE(4, 7, 5, 1700, 2100);
	MSP_SET_MODE_RANGE(5, 26, 0, 1700, 2100);

}

void Param_Config_Init(void)
{
	systemConfig_Init();
	pilotConfig_Init();
	boardConfig_Init();
	boardAlignment_Init(0, 0, 0);
    accelerometerConfig_init();
	gyroConfig_init();
	barometerConfig_Init();
    compassConfig_Init();
    adcConfig_Init();
	voltageSensorADCConfig_Init();
    currentSensorADCConfig_Init();
    dynNotchConfig_Init();
    imuConfig_Init();
	pidConfig_Init();
	pidProfiles_Init();
    rxConfig_Init();
    rxChannelRangeConfigs_Init();
    rxFailsafeChannelConfigs_Init();
	batteryConfig_Init();
	controlRateProfiles_Init();
	mixerConfig_Init();
    throttleCorrectionConfig_Init();
	featureConfig_Init();
	positionConfig_Init();
	rcControlsConfig_Init();
	armingConfig_Init();
	flight3DConfig_Init();
	osdConfig_Init();
	osdElementConfig_Init();
}
