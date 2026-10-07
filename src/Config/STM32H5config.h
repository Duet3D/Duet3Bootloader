/*
 * STM32H5config.h
 *
 *  Created on: 7 Oct 2026
 *      Author: David
 */

#ifndef SRC_CONFIG_STM32H5CONFIG_H_
#define SRC_CONFIG_STM32H5CONFIG_H_

// Configuration data for STM32H5 based boards.
// Currently we have only the NodeTrix board, so we can use fixed assignments.

// Diagnostic LED
constexpr unsigned int NumLedPins = 2;

// Standard assignment of LED pins used by most boards
constexpr Pin LedPins_NodeTrix[NumLedPins] =  { PortAPin(14), PortAPin(13) };
constexpr Pin LedActiveHigh_NodeTrix = false;
constexpr Pin CanResetPin_NodeTrix = PortCPin(7);

constexpr CanParameters CanParameters_NodeTrix =
{
	.instanceNumber = 1,													// FDCAN1 (not FDCAN2)
	.txPin = PortBPin(7),
	.rxPin = PortBPin(8),
	.pinsFunction = GpioPinFunction::AF9
};

#endif /* SRC_CONFIG_STM32H5CONFIG_H_ */
