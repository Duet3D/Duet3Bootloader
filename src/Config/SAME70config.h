/*
 * SAME70config.h
 *
 *  Created on: 1 Aug 2022
 *      Author: David
 */

#ifndef SRC_CONFIG_SAME70CONFIG_H_
#define SRC_CONFIG_SAME70CONFIG_H_

#include "RepRapFirmware.h"

// Diagnostic LED
constexpr unsigned int NumLedPins = 2;

constexpr Pin LedPins_MB6HC[NumLedPins] = { PortCPin(20), NoPin };
constexpr bool LedActiveHigh_MB6HC[] = { true, true };
constexpr Pin LedPins_MB6XD[NumLedPins] = { PortBPin(6), PortBPin(7) };
constexpr bool LedActiveHigh_MB6XD[] = { false, false };

// Available UART ports
//constexpr IRQn Serial0_IRQn = SERCOM3_0_IRQn;

// Interrupt priorities, lower means higher priority. 0-2 can't make RTOS calls.
const uint32_t NvicPriorityUart = 1;
const uint32_t NvicPriorityStep = 2;					// step interrupt is next highest, it can preempt most other interrupts
const uint32_t NvicPriorityPins = 3;					// priority for GPIO pin interrupts
const uint32_t NvicPriorityCan = 4;
const uint32_t NvicPriorityDmac = 5;					// priority for DMA complete interrupts

#endif /* SRC_CONFIG_SAME70CONFIG_H_ */
