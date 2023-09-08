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

#include "axis.h"

//#include "config/config_reset.h"


#include "config/config.h"
#include "fc/controlrate_profile.h"
#include "fc/rc.h"
#include "fc/rc_controls.h"

controlRateConfig_t *currentControlRateProfile;

controlRateConfig_t controlRateProfiles[CONTROL_RATE_PROFILE_COUNT];

void controlRateProfiles_Init(void)
{
    for (int i = 0; i < CONTROL_RATE_PROFILE_COUNT; i++) {
		controlRateProfiles[i].thrMid8 = 50;
		controlRateProfiles[i].thrExpo8 = 0;
		controlRateProfiles[i].tpa_rate = 65;
		controlRateProfiles[i].tpa_breakpoint = 1350;
		controlRateProfiles[i].rates_type = RATES_TYPE_ACTUAL;
		controlRateProfiles[i].rcRates[FD_ROLL] = 7;
		controlRateProfiles[i].rcRates[FD_PITCH] = 7;
		controlRateProfiles[i].rcRates[FD_YAW] = 7;
		controlRateProfiles[i].rcExpo[FD_ROLL] = 0;
		controlRateProfiles[i].rcExpo[FD_PITCH] = 0;
		controlRateProfiles[i].rcExpo[FD_YAW] = 0;
		controlRateProfiles[i].rates[FD_ROLL] = 67;
		controlRateProfiles[i].rates[FD_PITCH] = 67;
		controlRateProfiles[i].rates[FD_YAW] = 67;
		controlRateProfiles[i].throttle_limit_type = THROTTLE_LIMIT_TYPE_OFF;
		controlRateProfiles[i].throttle_limit_percent = 100;
		controlRateProfiles[i].rate_limit[FD_ROLL] = CONTROL_RATE_CONFIG_RATE_LIMIT_MAX;
		controlRateProfiles[i].rate_limit[FD_PITCH] = CONTROL_RATE_CONFIG_RATE_LIMIT_MAX;
		controlRateProfiles[i].rate_limit[FD_YAW] = CONTROL_RATE_CONFIG_RATE_LIMIT_MAX;
		controlRateProfiles[i].tpaMode = TPA_MODE_D;
		controlRateProfiles[i].profileName[MAX_RATE_PROFILE_NAME_LENGTH + 1] = 0;
		controlRateProfiles[i].quickRatesRcExpo = 0;
		controlRateProfiles[i].levelExpo[FD_ROLL] = 0;
		controlRateProfiles[i].levelExpo[FD_PITCH] = 0;
    }
}

const ratesSettingsLimits_t ratesSettingLimits[RATES_TYPE_COUNT] = {
    [RATES_TYPE_BETAFLIGHT] = { 255, 100, 100 },
    [RATES_TYPE_RACEFLIGHT] = { 200, 255, 100 },
    [RATES_TYPE_KISS]       = { 255,  99, 100 },
    [RATES_TYPE_ACTUAL]     = { 200, 200, 100 },
    [RATES_TYPE_QUICK]      = { 255, 200, 100 },
};

void loadControlRateProfile(void)
{
	controlRateProfiles_Init();
    currentControlRateProfile = &controlRateProfiles[systemConfig.activeRateProfile];
}

void changeControlRateProfile(uint8_t controlRateProfileIndex)
{
    if (controlRateProfileIndex < CONTROL_RATE_PROFILE_COUNT) {
        systemConfig.activeRateProfile = controlRateProfileIndex;
    }

    loadControlRateProfile();
    initRcProcessing();
}

void copyControlRateProfile(const uint8_t dstControlRateProfileIndex, const uint8_t srcControlRateProfileIndex) {
    if ((dstControlRateProfileIndex < CONTROL_RATE_PROFILE_COUNT && srcControlRateProfileIndex < CONTROL_RATE_PROFILE_COUNT)
        && dstControlRateProfileIndex != srcControlRateProfileIndex
    ) {
        memcpy(&controlRateProfiles[dstControlRateProfileIndex], &controlRateProfiles[srcControlRateProfileIndex], sizeof(controlRateConfig_t));
    }
}
