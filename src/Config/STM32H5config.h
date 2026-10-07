/*
 * STM32H5config.h
 *
 *  Created on: 7 Oct 2026
 *      Author: David
 */

#ifndef SRC_CONFIG_STM32H5CONFIG_H_
#define SRC_CONFIG_STM32H5CONFIG_H_

// Diagnostic LED
constexpr unsigned int NumLedPins = 2;

// Standard assignment of LED pins used by most boards
constexpr Pin LedPins_NodeTrix[NumLedPins] =  { PortAPin(14), PortAPin(13) };
constexpr Pin LedActiveHigh_NodeTrix = false;
constexpr Pin CanResetPin_NodeTrix = PortCPin(7);

#endif /* SRC_CONFIG_STM32H5CONFIG_H_ */
