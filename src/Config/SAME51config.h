/*
 * Expansion1_v07.h
 *
 *  Created on: 30 Jun 2019
 *      Author: David
 */

#ifndef SRC_CONFIG_EXPANSION1_V09_H_
#define SRC_CONFIG_EXPANSION1_V09_H_

#include "RepRapFirmware.h"

#define DIAG_SERCOM_NUMBER	3			// which SERCOM device we use for debugging output

constexpr size_t NumAddressBits = 4;
constexpr size_t NumBoardTypeBits = 3;

constexpr Pin GlobalTmc51xxEnablePin = PortBPin(23);
constexpr Pin GlobalTmc51xxCSPin = PortBPin(22);

constexpr Pin StepPins[] = { PortAPin(25), PortAPin(27), PortAPin(1) };

constexpr Pin BoardTypePins[NumBoardTypeBits] = { PortBPin(18), PortCPin(19), PortCPin(16) };
constexpr Pin BoardAddressPins[NumAddressBits] = { PortCPin(11), PortCPin(12), PortCPin(14), PortCPin(15) };

// Diagnostic LED
constexpr Pin LedPins[] = { PortCPin(10) };
constexpr size_t NumLedPins = ARRAY_SIZE(LedPins);
constexpr bool LedActiveHigh = true;

// Available UART ports
constexpr IRQn Serial0_IRQn = SERCOM3_0_IRQn;

// Interrupt priorities, lower means higher priority. 0-2 can't make RTOS calls.
const uint32_t NvicPriorityUart = 1;
const uint32_t NvicPriorityStep = 2;					// step interrupt is next highest, it can preempt most other interrupts
const uint32_t NvicPriorityPins = 3;					// priority for GPIO pin interrupts
const uint32_t NvicPriorityCan = 4;
const uint32_t NvicPriorityDmac = 5;					// priority for DMA complete interrupts

#endif /* SRC_CONFIG_EXPANSION1_V09_H_ */
