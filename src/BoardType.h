/*
 * BoardType.h
 *
 *  Created on: 19 Oct 2022
 *      Author: David
 */

#ifndef SRC_BOARDTYPE_H_
#define SRC_BOARDTYPE_H_

#include <RepRapFirmware.h>

// Identify the board and set the parameters, returning true if successful, false if we could not identify the board
bool IdentifyBoard(CanAddress& defaultAddress, bool& doHardwareReset, bool& useAlternateCanPins);

Pin GetLedPin(unsigned int ledNumber);
bool GetLedActiveHigh();
const char *GetBoardTypeName();
const unsigned int GetBoardVersion();

#endif /* SRC_BOARDTYPE_H_ */
