/*
 * Expansion1_v07.h
 *
 *  Created on: 30 Jun 2019
 *      Author: David
 */

#ifndef SRC_CONFIG_EXPANSION1_V09_H_
#define SRC_CONFIG_EXPANSION1_V09_H_

#include "RepRapFirmware.h"

#define DIAG_SERCOM_NUMBER	3							// which SERCOM device we use for debugging output

constexpr size_t NumAddressBits = 4;
constexpr Pin BoardAddressPins[NumAddressBits] = { PortCPin(11), PortCPin(12), PortCPin(14), PortCPin(15) };

Adc * const CommonAdcDevice = ADC0;						// ADC device used for the board type pin
constexpr Pin BoardTypePin = PortAPin(3);				// Board type pin (not on EXP3HC)
constexpr uint8_t BoardTypeAdcChannel = 1;				// the ADC channel that the board type pin is connected to

// Diagnostic LED
constexpr unsigned int NumLedPins = 2;

constexpr Pin LedPins_EXP3HC[NumLedPins] = { PortCPin(10), PortCPin(7) };
constexpr bool LedActiveHigh_EXP3HC = true;

constexpr Pin LedPins_EXP1HCL[NumLedPins] = { PortAPin(30), PortAPin(31) };
constexpr bool LedActiveHigh_EXP1HCL = false;

constexpr Pin LedPins_DUET3MINI[NumLedPins] = { PortAPin(30), PortAPin(31) };
constexpr bool LedActiveHigh_DUET3MINI = false;

constexpr Pin LedPins_M23CL[NumLedPins] = { PortAPin(12), PortAPin(13) };
constexpr bool LedActiveHigh_M23CL = true;

constexpr Pin LedPins_TOOL1RR[NumLedPins] = { PortAPin(30), PortAPin(31) };
constexpr bool LedActiveHigh_TOOL1RR = false;

constexpr Pin LedPins_F3PTB[NumLedPins] = { PortAPin(30), PortAPin(31) };
constexpr bool LedActiveHigh_F3PTB = false;

constexpr Pin CanResetPin_EXP1HCL_v1 = PortAPin(0);
constexpr Pin CanResetPin_EXP1HCL_v2 = PortAPin(27);	// same as DRIVER_DIR pin
constexpr Pin CanResetPin_M23CL = PortAPin(0);
constexpr Pin CanResetPin_TOOL1RR = PortAPin(10);		// same as DRIVER_DIR pin
constexpr Pin CanResetPin_F3PTB = PortAPin(10);			// same as DRIVER_DIR pin

// Available UART ports
constexpr IRQn Serial0_IRQn = SERCOM3_0_IRQn;

// Interrupt priorities, lower means higher priority. 0-2 can't make RTOS calls.
const uint32_t NvicPriorityUart = 1;
const uint32_t NvicPriorityStep = 2;					// step interrupt is next highest, it can preempt most other interrupts
const uint32_t NvicPriorityPins = 3;					// priority for GPIO pin interrupts
const uint32_t NvicPriorityCan = 4;
const uint32_t NvicPriorityDmac = 5;					// priority for DMA complete interrupts

#endif /* SRC_CONFIG_EXPANSION1_V09_H_ */
