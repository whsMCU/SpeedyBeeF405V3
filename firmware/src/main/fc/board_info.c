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
#include <string.h>

#include "hw.h"

#if defined(USE_BOARD_INFO)
#include "build/version.h"

#include "fc/board_info.h"

static bool boardInformationSet = false;
static char manufacturerId[MAX_MANUFACTURER_ID_LENGTH + 1];
static char boardName[MAX_BOARD_NAME_LENGTH + 1];
static bool boardInformationWasUpdated = false;

static bool signatureSet = false;
static uint8_t signature[SIGNATURE_LENGTH];

boardConfig_t boardConfig;

void boardConfig_Init(void)
{
    if (boardInformationIsSet()) {
        strncpy(boardConfig.manufacturerId, getManufacturerId(), MAX_MANUFACTURER_ID_LENGTH + 1);
        strncpy(boardConfig.boardName, getBoardName(), MAX_BOARD_NAME_LENGTH + 1);
        boardConfig.boardInformationSet = true;
    } else {
#if !defined(USE_UNIFIED_TARGET)
        strncpy(boardConfig.boardName, targetName, MAX_BOARD_NAME_LENGTH + 1);

#if defined(TARGET_MANUFACTURER_IDENTIFIER)
        strncpy(boardConfig.manufacturerId, TARGET_MANUFACTURER_IDENTIFIER, MAX_MANUFACTURER_ID_LENGTH + 1);
#endif
        boardConfig.boardInformationSet = true;
#else
        boardConfig.boardInformationSet = false;
#endif // USE_UNIFIED_TARGET
    }

#if defined(USE_SIGNATURE)
    if (signatureIsSet()) {
        memcpy(boardConfig.signature, getSignature(), SIGNATURE_LENGTH);
        boardConfig.signatureSet = true;
    } else {
        boardConfig.signatureSet = false;
    }
#endif
}

void initBoardInformation(void)
{
    boardInformationSet = boardConfig.boardInformationSet;
    if (boardInformationSet) {
        strncpy(manufacturerId, boardConfig.manufacturerId, MAX_MANUFACTURER_ID_LENGTH + 1);
        strncpy(boardName, boardConfig.boardName, MAX_BOARD_NAME_LENGTH + 1);
    }

    signatureSet = boardConfig.signatureSet;
    if (signatureSet) {
        memcpy(signature, boardConfig.signature, SIGNATURE_LENGTH);
    }
}

const char *getManufacturerId(void)
{
    return manufacturerId;
}

const char *getBoardName(void)
{
    return boardName;
}

bool boardInformationIsSet(void)
{
    return boardInformationSet;
}

bool setManufacturerId(const char *newManufacturerId)
{
    if (!boardInformationSet || strlen(manufacturerId) == 0) {
        strncpy(manufacturerId, newManufacturerId, MAX_MANUFACTURER_ID_LENGTH + 1);

        boardInformationWasUpdated = true;

        return true;
    } else {
        return false;
    }
}

bool setBoardName(const char *newBoardName)
{
    if (!boardInformationSet || strlen(boardName) == 0) {
        strncpy(boardName, newBoardName, MAX_BOARD_NAME_LENGTH + 1);

        boardInformationWasUpdated = true;

        return true;
    } else {
        return false;
    }
}

bool persistBoardInformation(void)
{
    if (boardInformationWasUpdated) {
        strncpy(boardConfig.manufacturerId, manufacturerId, MAX_MANUFACTURER_ID_LENGTH + 1);
        strncpy(boardConfig.boardName, boardName, MAX_BOARD_NAME_LENGTH + 1);
        boardConfig.boardInformationSet = true;

        initBoardInformation();

        return true;
    } else {
        return false;
    }
}

#if defined(USE_SIGNATURE)
const uint8_t *getSignature(void)
{
    return signature;
}

bool signatureIsSet(void)
{
    return signatureSet;
}

bool setSignature(const uint8_t *newSignature)
{
    if (!signatureSet) {
        memcpy(signature, newSignature, SIGNATURE_LENGTH);

        return true;
    } else {
        return false;
    }
}

bool persistSignature(void)
{
    if (!signatureSet) {
        memcpy(boardConfig.signature, signature, SIGNATURE_LENGTH);
        boardConfig.signatureSet = true;

        initBoardInformation();

        return true;
    } else {
        return false;
    }
}
#endif
#endif // USE_BOARD_INFO
