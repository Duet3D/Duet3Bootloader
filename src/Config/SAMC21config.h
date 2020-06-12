/*
 * Tool1_v01.h
 *
 *  Created on: 18 Aug 2019
 *      Author: David
 */

#ifndef SRC_CONFIG_SAMC21CONFIG_H_
#define SRC_CONFIG_SAMC21CONFIG_H_

#include "RepRapFirmware.h"
#include "Hardware/Peripherals.h"

#ifdef SAMMYC21
# define DIAG_SERCOM_NUMBER	5							// which SERCOM device we use for debugging output
#else
# define DIAG_SERCOM_NUMBER	4							// which SERCOM device we use for debugging output
#endif

#ifdef SAMMYC21

constexpr Pin ButtonPins[] = { PortBPin(9) };
constexpr Pin CanStandbyPin = PortAPin(27);

#else

Adc * const CommonAdcDevice = ADC0;						// ADC device used for the board type pin, also for the buttons on the EXP1HCE
constexpr Pin BoardTypePin = PortAPin(5);
constexpr uint8_t BoardTypeAdcChannel = 5;
constexpr Pin ButtonsPin_Exp1HCE = PortAPin(9);
constexpr uint8_t ButtonsAdcChannel_Exp1HCE = 9;

constexpr Pin GlobalTmc22xxEnablePin_Tool1LC = PortBPin(2);
constexpr Pin OutPins_Tool1LC[] = { PortAPin(11), PortAPin(10), PortBPin(11) };
constexpr Pin ButtonPins_Tool1LC[] = { PortBPin(22), PortBPin(23) };

constexpr Pin JumperPin_Exp1XD = PortAPin(27);

#endif

// Diagnostic LED
#ifdef SAMMYC21
constexpr Pin LedPins[] = { PortAPin(28) };
constexpr bool LedActiveHigh = true;
#else
// TODO these now depend on which board we are running on
constexpr Pin LedPins[] = { PortAPin(0), PortAPin(1) };
constexpr bool LedActiveHigh = true;
#endif

// Interrupt priorities, lower means higher priority. 0 can't make RTOS calls.
const uint32_t NvicPriorityUart = 0;
const uint32_t NvicPriorityStep = 1;					// step interrupt is next highest, it can preempt most other interrupts
const uint32_t NvicPriorityPins = 2;					// priority for GPIO pin interrupts
const uint32_t NvicPriorityCan = 3;
const uint32_t NvicPriorityDmac = 3;					// priority for DMA complete interrupts

#endif /* SRC_CONFIG_SAMC21CONFIG_H_ */
