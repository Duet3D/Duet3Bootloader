/*
 * Tool1_v01.h
 *
 *  Created on: 18 Aug 2019
 *      Author: David
 */

#ifndef SRC_CONFIG_SAMC21CONFIG_H_
#define SRC_CONFIG_SAMC21CONFIG_H_

#include "RepRapFirmware.h"

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
constexpr Pin BoardType2Pin = PortAPin(4);				// second board type pin used on some boards
constexpr uint8_t BoardType2AdcChannel = 4;

constexpr Pin GlobalTmc22xxEnablePin_Tool1LC = PortBPin(2);
constexpr Pin OutPins_Tool1LC[] = { PortAPin(11), PortAPin(10), PortBPin(11) };
constexpr Pin ButtonPins_Tool1LC[] = { PortBPin(22), PortBPin(23) };

constexpr Pin CanResetPin_Tool1LC = PortBPin(22);		// first button pin
constexpr Pin CanResetPin_SZP = PortAPin(18);
constexpr Pin CanResetPin_Exp1XD = PortAPin(27);
constexpr Pin CanResetPin_AteCM = NoPin;				// the CAN jumper on the ATECM is connected between AteCmJumperPin and AteCmZeroPin
constexpr Pin CanResetPin_AteIO_v01 = PortAPin(16);
constexpr Pin CanResetPin_AteIO_v02 = PortBPin(11);

constexpr Pin AteCmJumperPin = PortAPin(27);
constexpr Pin AteCmZeroPin = PortBPin(2);

#endif

// Diagnostic LEDs
constexpr unsigned int NumLedPins = 2;

#ifdef SAMMYC21

constexpr Pin LedPins_SAMMYC21[NumLedPins] = { PortAPin(28), NoPin };
constexpr bool LedActiveHigh_SAMMYC21 = true;

#else

constexpr Pin LedPins_Tool1LC_v0[NumLedPins] = { PortAPin(0), PortAPin(1) };
constexpr bool LedActiveHigh_Tool1LC_v0 = true;

constexpr Pin LedPins_Tool1LC_v1[NumLedPins] = { PortAPin(30), PortAPin(31) };
constexpr bool LedActiveHigh_Tool1LC_v1 = false;

constexpr Pin LedPins_SZP[NumLedPins] = { PortAPin(30), PortAPin(31) };
constexpr bool LedActiveHigh_SZP = false;

constexpr Pin LedPins_Exp1XD[NumLedPins] = { PortAPin(19), PortAPin(18) };
constexpr bool LedActiveHigh_Exp1XD = true;

constexpr Pin LedPins_Ate[NumLedPins] = { PortAPin(30), PortAPin(31) };
constexpr bool LedActiveHigh_Ate = false;

constexpr Pin LedPins_Exp1HCE[NumLedPins] = { PortAPin(30), PortAPin(31) };
constexpr bool LedActiveHigh_Exp1HCE = false;

#endif

// Interrupt priorities, lower means higher priority. 0 can't make RTOS calls.
const uint32_t NvicPriorityUart = 0;
const uint32_t NvicPriorityStep = 1;					// step interrupt is next highest, it can preempt most other interrupts
const uint32_t NvicPriorityPins = 2;					// priority for GPIO pin interrupts
const uint32_t NvicPriorityCan = 3;
const uint32_t NvicPriorityDmac = 3;					// priority for DMA complete interrupts

#endif /* SRC_CONFIG_SAMC21CONFIG_H_ */
