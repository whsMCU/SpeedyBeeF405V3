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

#pragma once

#include "adc.h"


#ifdef USE_ADC_INTERNAL
extern int32_t adcVREFINTCAL;      // ADC value (12-bit) of band gap with Vref = VREFINTCAL_VREF
extern int32_t adcTSCAL1;
extern int32_t adcTSCAL2;
extern int32_t adcTSSlopeK;
#endif

#define ADC_CHANNEL_COUNT_Custem 5
//extern const adcDevice_t adcHardware[];
//extern const adcTagMap_t adcTagMap[ADC_TAG_MAP_COUNT];
extern adcOperatingConfig_t adcOperatingConfig[ADC_CHANNEL_COUNT];
extern volatile uint16_t adcValues[ADC_CHANNEL_COUNT_Custem];

//uint8_t adcChannelByTag(ioTag_t ioTag);
//ADCDevice adcDeviceByInstance(ADC_TypeDef *instance);
//bool adcVerifyPin(ioTag_t tag, ADCDevice device);

// Marshall values in DMA instance/channel based order to adcChannel based order.
// Required for multi DMA instance implementation
void adcGetChannelValues(void);
