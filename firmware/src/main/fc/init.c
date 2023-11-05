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

#include "hw.h"

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

#include "io/displayport_max7456.h"

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

#include "flight/failsafe.h"
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
#include "drivers/motor.h"
#include "drivers/pwm_output.h"
#include "drivers/osd.h"
#include "drivers/max7456.h"

#include "fc/stats.h"

#include "rx/rx.h"

uint8_t systemState = SYSTEM_STATE_INITIALISING;

static void Param_Config_Init(void);

void init(void)
{
    // Initialize task data as soon as possible. Has to be done before tasksInit(),
    // and any init code that may try to modify task behaviour before tasksInit().
	tasksInitData();
	Param_Config_Init();

	cliOpen(_DEF_USB, 57600);

	readEEPROM();

#if defined(USE_BOARD_INFO)
    initBoardInformation();
#endif

//#if defined(STM32F4) || defined(STM32G4)
//    // F4 has non-8MHz boards
//    // G4 for Betaflight allow 24 or 27MHz oscillator
//    systemClockSetHSEValue(systemConfig.hseMhz * 1000000U);
//#endif

#ifdef USE_OVERCLOCK
    OverclockRebootIfNecessary(systemConfig.cpu_overclock);
#endif

	mixerInit(mixerConfig.mixerMode);

  uint16_t idlePulse = motorConfig.mincommand;
  if (featureIsEnabled(FEATURE_3D)) {
      idlePulse = flight3DConfig.neutral3d;
  }
  if (motorConfig.dev.motorPwmProtocol == PWM_TYPE_BRUSHED) {
      idlePulse = 0; // brushed motors
  }

  /* Motors needs to be initialized soon as posible because hardware initialization
   * may send spurious pulses to esc's causing their early initialization. Also ppm
   * receiver may share timer with motors so motors MUST be initialized here. */
  motorDevInit(&motorConfig.dev, idlePulse, getMotorCount());

	initBoardAlignment(&boardAlignment);

#ifdef USE_ACC
	Sensor_Init();
#endif

#ifdef USE_MAG
	compassInit();
#endif

#ifdef USE_BARO
	Baro_Init();
#endif

#ifdef USE_ADC_INTERNAL
	adcInternalInit();
#endif
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

	/////////////// LED //////////////////
	LED0_ON;
	for (int i = 0; i < 10; i++)
	{
		LED0_TOGGLE;
		#if defined(USE_BEEPER)
		delay(25);
		if (!(beeperConfig.beeper_off_flags & BEEPER_GET_FLAG(BEEPER_SYSTEM_INIT))) {
			BEEP_ON;
		}
			delay(25);
			BEEP_OFF;
		#else
			delay(50);
		#endif
	}
	LED0_OFF;
	////////////////////////////////////////

	imuInit();
    failsafeInit();

    rxInit();

#ifdef USE_GPS
    if (featureIsEnabled(FEATURE_GPS)) {
        gpsInit();
    }
#endif

#ifdef USE_ACC
    if (mixerConfig.mixerMode == MIXER_GIMBAL) {
        accStartCalibration();
    }
#endif
    gyroStartCalibration(false);
#ifdef USE_BARO
    baroStartCalibration();
#endif

	batteryInit(); // always needs doing, regardless of features.

#ifdef USE_PERSISTENT_STATS
    statsInit();
#endif


/*
 * CMS, display devices and OSD
 */
#ifdef USE_CMS
	cmsInit();
#endif

#if (defined(USE_OSD) || (defined(USE_MSP_DISPLAYPORT) && defined(USE_CMS)))
    displayPort_t *osdDisplayPort = NULL;
    osdDisplayPortDevice_e osdDisplayPortDevice = OSD_DISPLAYPORT_DEVICE_NONE;
#endif

#if defined(USE_OSD)
    //The OSD need to be initialised after GYRO to avoid GYRO initialisation failure on some targets

    if (featureIsEnabled(FEATURE_OSD)) {
        osdDisplayPortDevice_e device = osdConfig.displayPortDevice;

        switch(device) {

        case OSD_DISPLAYPORT_DEVICE_AUTO:
            FALLTHROUGH;

#if defined(USE_FRSKYOSD)
        // Test OSD_DISPLAYPORT_DEVICE_FRSKYOSD first, since an FC could
        // have a builtin MAX7456 but also an FRSKYOSD connected to an
        // uart.
        case OSD_DISPLAYPORT_DEVICE_FRSKYOSD:
            osdDisplayPort = frskyOsdDisplayPortInit(vcdProfile()->video_system);
            if (osdDisplayPort || device == OSD_DISPLAYPORT_DEVICE_FRSKYOSD) {
                osdDisplayPortDevice = OSD_DISPLAYPORT_DEVICE_FRSKYOSD;
                break;
            }
            FALLTHROUGH;
#endif

#if defined(USE_MAX7456)
        case OSD_DISPLAYPORT_DEVICE_MAX7456:
            // If there is a max7456 chip for the OSD configured and detected then use it.
            if (max7456DisplayPortInit(&vcdProfile, &osdDisplayPort) || device == OSD_DISPLAYPORT_DEVICE_MAX7456) {
                osdDisplayPortDevice = OSD_DISPLAYPORT_DEVICE_MAX7456;
                break;
            }
            FALLTHROUGH;
#endif

#if defined(USE_CMS) && defined(USE_MSP_DISPLAYPORT) && defined(USE_OSD_OVER_MSP_DISPLAYPORT)
        case OSD_DISPLAYPORT_DEVICE_MSP:
            osdDisplayPort = displayPortMspInit();
            if (osdDisplayPort || device == OSD_DISPLAYPORT_DEVICE_MSP) {
                osdDisplayPortDevice = OSD_DISPLAYPORT_DEVICE_MSP;
                break;
            }
            FALLTHROUGH;
#endif

        // Other device cases can be added here

        case OSD_DISPLAYPORT_DEVICE_NONE:
        default:
            break;
        }

        // osdInit will register with CMS by itself.
        osdInit(osdDisplayPort, osdDisplayPortDevice);

        if (osdDisplayPortDevice == OSD_DISPLAYPORT_DEVICE_NONE) {
            featureDisableImmediate(FEATURE_OSD);
        }
    }
#endif // USE_OSD

#ifdef USE_TELEMETRY
    // Telemetry will initialise displayport and register with CMS by itself.
    if (featureIsEnabled(FEATURE_TELEMETRY)) {
        telemetryInit();
    }
#endif
    setArmingDisabled(ARMING_DISABLED_BOOT_GRACE_TIME);

#ifdef USE_MOTOR
    motorPostInit();
    motorEnable();
#endif

	tasksInit();
	MSP_SET_MODE_RANGE(0,  0, 0, 1700, 2100);
	MSP_SET_MODE_RANGE(1,  1, 1,  900, 2100);
	MSP_SET_MODE_RANGE(2,  6, 2, 1300, 2100);
	MSP_SET_MODE_RANGE(3, 27, 4, 1700, 2100);
	MSP_SET_MODE_RANGE(4,  7, 5, 1700, 2100);
	MSP_SET_MODE_RANGE(5, 13, 5, 1700, 2100);
	MSP_SET_MODE_RANGE(6, 39, 4, 1700, 2100);

}

void Param_Config_Init(void)
{
	systemConfig_Init();
	pilotConfig_Init();
	boardConfig_Init();

	boardAlignment_Init(0, 0, 0);
	failsafeConfig_Init();
	accelerometerConfig_init();
	gyroConfig_init();
	gyroDeviceConfig_Init();
	statsConfig_Init();
	motorConfig_Init();
#ifdef USE_GPS
	gpsConfig_Init();
#endif
	barometerConfig_Init();
#ifdef USE_MAG
	compassConfig_Init();
#endif
	adcConfig_Init();
	voltageSensorADCConfig_Init();
	currentSensorADCConfig_Init();
#ifdef USE_DYN_NOTCH_FILTER
	dynNotchConfig_Init();
#endif
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
#ifdef USE_OSD
	vcdProfile_Init();
	osdConfig_Init();
	osdElementConfig_Init();
	max7456Config_Init();
#if defined(USE_MSP_DISPLAYPORT)
	displayPortProfileMsp_Init();
#endif
#if defined(USE_MAX7456)
	displayPortProfileMax7456_Init();
#endif

#endif
}
