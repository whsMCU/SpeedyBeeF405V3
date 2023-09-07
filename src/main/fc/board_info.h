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

#define MAX_MANUFACTURER_ID_LENGTH 4
#define MAX_BOARD_NAME_LENGTH 20
#define SIGNATURE_LENGTH 32

// Warning: This configuration is meant to be applied when loading the initial
// configuration for a generic board, and stay fixed after this, to enable
// identification of the hardware that this is running on.
// Do not modify this parameter group directly, use 'fc/board_info.h' instead.

typedef struct boardConfig_s {
    uint8_t signature[SIGNATURE_LENGTH];
    char manufacturerId[MAX_MANUFACTURER_ID_LENGTH + 1];
    char boardName[MAX_BOARD_NAME_LENGTH + 1];
    uint8_t boardInformationSet;
    uint8_t signatureSet;
} boardConfig_t;

extern boardConfig_t boardConfig;

void boardConfig_Init(void);

void initBoardInformation(void);

const char *getBoardName(void);
const char *getManufacturerId(void);
bool boardInformationIsSet(void);

bool setBoardName(const char *newBoardName);
bool setManufacturerId(const char *newManufacturerId);
bool persistBoardInformation(void);

const uint8_t * getSignature(void);
bool signatureIsSet(void);

bool setSignature(const uint8_t *newSignature);
bool persistSignature(void);
