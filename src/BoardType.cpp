/*
 * BoardTypeDetection.cpp
 *
 *  Created on: 19 Oct 2022
 *      Author: David
 */

#include "BoardType.h"
#include <CanId.h>

static unsigned int boardTypeIndex = 0;

// Constexpr function to check that a table is in increasing order
inline constexpr bool IsIncreasing(const float *arr, size_t length)
{
	return length < 2 || (arr[1] > arr[0] && IsIncreasing(arr + 1, length - 1));
}

#if SAME5x

# if defined(CAN_IAP)

constexpr const char* BoardTypeNames[] = { "Mini5plus" };
constexpr unsigned int BoardTypeVersions[] = { 0 };
constexpr const Pin *LedPinsTables[] = { LedPins_DUET3MINI };
constexpr bool LedActiveHigh[] = { LedActiveHigh_DUET3MINI };

bool IdentifyBoard(CanAddress& defaultAddress, bool& doHardwareReset, bool& useAlternateCanPins)
{
	defaultAddress = CanId::ExpansionBoardFirmwareUpdateAddress;
	doHardwareReset = false;
	useAlternateCanPins = false;
	return true;
}

# else

#  include <AnalogIn.h>

// Board ID analog pin handling
constexpr uint32_t AdcRange = 1u << AnalogIn::AdcBits;

// Currently we support three boards: EXP3HC, EXP1HCL, and M23CL
constexpr const char* BoardTypeNames[] =
{
	"EXP3HC",
	"M23CL",
	"EXP1HCL",
};

constexpr unsigned int BoardTypeVersions[] =
{
	0,
	0,
	0
};

constexpr const Pin *LedPinsTables[] =
{
	LedPins_EXP3HC,
	LedPins_M23CL,
	LedPins_EXP1HCL,
};

constexpr bool LedActiveHigh[] =
{
	LedActiveHigh_EXP3HC,
	LedActiveHigh_M23CL,
	LedActiveHigh_EXP1HCL,
};

constexpr Pin CanResetPins[] =
{
	NoPin,
	CanResetPin_M23CL,
	CanResetPin_EXP1HCL
};

// This table of floats is only used at compile time, so it shouldn't cause the floating point library to be pulled in
constexpr float BoardTypeFractions[] =
{
	4.7/(4.7 + 60.4),						// M23CL has 4K7 lower resistor, 60.4K upper
	10.0/(1.0 + 10.0),						// EXCP1HCL has 10K lower resistor, 1K upper
};

static_assert(IsIncreasing(BoardTypeFractions, ARRAY_SIZE(BoardTypeFractions)));

// Table of halfway points that we use to decide what board type a reading corresponds to
constexpr uint16_t BoardIdDecisionPoints[] =
{
	(uint16_t)((BoardTypeFractions[0] + BoardTypeFractions[1]) * (AdcRange/2)),
};

static_assert(ARRAY_SIZE(BoardIdDecisionPoints) + 1 == ARRAY_SIZE(BoardTypeFractions));

// Function to read a pin and scan a table of decision points to find the corresponding index
static unsigned int ReadAndQuantise(uint8_t chan, const uint16_t decisionPoints[], size_t numDecisionPoints)
{
	const uint16_t reading = AnalogIn::ReadChannel(CommonAdcDevice, chan);
	unsigned int ainState = 0;
	while (ainState < numDecisionPoints && reading > decisionPoints[ainState])
	{
		++ainState;
	}
	return ainState;
}

// Values of the DID register that correspond to the boards we support
//	ID			Chip		Board
//	0x61810300	SAME51N20A	EXP3HC
//	0x61810301	SAME51N19A	EXP3HC
//	0x61810302	SAME51J19A
//	0x61810303	SAME51J18A
//	0x61810304	SAME51J20A
//	0x61810305	SAME51G19A	EXP1HCL or M23CL
//	0x61810306	SAME51G18A	EXP1HCL or M23CL
// Bits 8-15 (03 in the above) identify the die and revision number, so may be subject to change

constexpr uint32_t DeviceIdMask = 0xFFFF00FF;

enum DeviceId : uint32_t
{
	SAME51N_min = 0x61810300 & DeviceIdMask,
	SAME51N_max = 0x61810301 & DeviceIdMask,
	SAME51J_min = 0x61810302 & DeviceIdMask,
	SAME51J_max = 0x61810304 & DeviceIdMask,
	SAME51G_min = 0x61810305 & DeviceIdMask,
	SAME51G_max = 0x61810306 & DeviceIdMask
};

// Read the board address
uint8_t ReadBoardAddress()
{
	uint8_t rslt = 0;
	for (unsigned int i = 0; i < 4; ++i)
	{
		if (!digitalRead(BoardAddressPins[i]))
		{
			rslt |= 1u << i;
		}
	}
	return rslt;
}

bool IdentifyBoard(CanAddress& defaultAddress, bool& doHardwareReset, bool& useAlternateCanPins)
{
	useAlternateCanPins = false;

	// Determine the board type
	const uint32_t deviceId = REG_DSU_DID & DeviceIdMask;
	if (deviceId >= SAME51N_min && deviceId <= SAME51N_max)
	{
		// If a SAME51N processor, assume EXP3HC
		boardTypeIndex = 0;
		for (Pin p : BoardAddressPins)
		{
			pinMode(p, INPUT_PULLUP);
		}

		// Check whether address switches are set to zero. If so then reset and load new firmware
		const CanAddress switches = ReadBoardAddress();
		doHardwareReset = (switches == 0);
		defaultAddress = (doHardwareReset) ? CanId::ExpansionBoardFirmwareUpdateAddress : switches;
		return true;
	}

	if (deviceId >= SAME51G_min && deviceId <= SAME51G_max)
	{
		// Read the board type pin, which is an analog input fed from a resistor network
		AnalogIn::Init(CommonAdcDevice);
		SetPinFunction(BoardTypePin, GpioPinFunction::B);

		boardTypeIndex = ReadAndQuantise(BoardTypeAdcChannel, BoardIdDecisionPoints, ARRAY_SIZE(BoardIdDecisionPoints)) + 1;

		useAlternateCanPins = true;
		defaultAddress = CanId::Exp1HCEBoardDefaultAddress;		// EXP1HCL and M23CL default address is that same as 1HCE
		pinMode(CanResetPins[boardTypeIndex], INPUT_PULLUP);
		delayMicroseconds(100);
		doHardwareReset = !digitalRead(CanResetPins[boardTypeIndex]);
		AnalogIn::Disable(CommonAdcDevice);						// finished using the ADC
		return true;
	}

	return false;
}

# endif

#elif SAMC21

# ifdef SAMMYC21

constexpr const char* BoardTypeNames[] = { "SAMMYC21" };
constexpr unsigned int BoardTypeVersions[] = { 0 };
constexpr const Pin *LedPinsTables[] = { LedPins_SAMMYC21 };
constexpr bool LedActiveHigh[] = { LedActiveHigh_SAMMYC21 };

bool IdentifyBoard(CanAddress& defaultAddress, bool& doHardwareReset, bool& useAlternateCanPins)
{
	defaultAddress = CanId::SammyC21DefaultAddress;
	useAlternateCanPins = true;
	pinMode(ButtonPins[0], INPUT_PULLUP);
	delayMicroseconds(100);
	doHardwareReset = !digitalRead(ButtonPins[0]);
	return true;
}

# else

#  include <AnalogIn.h>

// Board ID analog pin handling
constexpr uint32_t AdcRange = 1u << AnalogIn::AdcBits;

// Names and board ID pin resistor ratios for the board we support
// The expected ADC readings must be in increasing order, and the board names must be in the corresponding order
// The board type and version as an enumeration
enum class BoardId : unsigned int
{
	// Board types based on the first board detect pin
	tool1lc_v0 = 0,
	exp1hce_v0,
	ate,
	exp1xd_v0,
	tool1lc_v1,

	// ATE board types
	ate_base,
	ate_cm = ate_base,
	ate_io
};

constexpr const char * BoardTypeNames[] =
{
	// Boards selected by just the first board type pin
	"TOOL1LC",			// version 1.0 or earlier
	"EXP1HCE",
	"unknown",			// for ATE we need to look at the second board type pin
	"EXP1XD",
	"TOOL1LC",			// version 1.1 or later

	// ATE board types, distinguished by the second board type pin
	"ATECM",
	"ATEIO",
};

constexpr unsigned int BoardTypeVersions[] =
{
	0,
	0,
	0,
	0,
	1,
	0,
	0
};

constexpr const Pin *LedPinsTables[] =
{
	LedPins_Tool1LC_v0,
	LedPins_Exp1HCE,
	LedPins_Ate,
	LedPins_Exp1XD,
	LedPins_Tool1LC_v1,
	LedPins_Ate,
	LedPins_Ate,
};

constexpr bool LedActiveHigh[] =
{
	LedActiveHigh_Tool1LC_v0,
	LedActiveHigh_Exp1HCE,
	LedActiveHigh_Ate,
	LedActiveHigh_Exp1XD,
	LedActiveHigh_Tool1LC_v1,
	LedActiveHigh_Ate,
	LedActiveHigh_Ate,
};

constexpr Pin CanResetPins[] =
{
	CanResetPin_Tool1LC,
	CanResetPin_Exp1HCE,
	NoPin,
	CanResetPin_Exp1XD,
	CanResetPin_Tool1LC,
	CanResetPin_AteCM,
	CanResetPin_AteIO
};

// This table of floats is only used at compile time, so it shouldn't cause the floating point library to be pulled in
constexpr float BoardTypeFractions[] =
{
	1.0/(1.0 + 10.0),						// TOOL1LC v1.0 has 1K lower resistor, 10K upper
	4.7/(4.7 + 27.0),						// EXP1HCE has 4K7 lower resistor, 27K upper
	3.0/(3.0 + 12.0),						// ATECM and ATEIO have 3K lower, 12K upper
	4.7/(4.7 + 4.7),						// EXP1XD has 4K7 lower resistor, 4K7 upper
	10.0/(1.0 + 10.0),						// TOOL1LC v1.1 has 10K lower resistor, 1K upper
};

static_assert(IsIncreasing(BoardTypeFractions, ARRAY_SIZE(BoardTypeFractions)));

// Table of halfway points that we use to decide what board type a reading corresponds to
constexpr uint16_t BoardIdDecisionPoints[] =
{
	(uint16_t)((BoardTypeFractions[0] + BoardTypeFractions[1]) * (AdcRange/2)),
	(uint16_t)((BoardTypeFractions[1] + BoardTypeFractions[2]) * (AdcRange/2)),
	(uint16_t)((BoardTypeFractions[2] + BoardTypeFractions[3]) * (AdcRange/2)),
	(uint16_t)((BoardTypeFractions[3] + BoardTypeFractions[4]) * (AdcRange/2))
};

static_assert(ARRAY_SIZE(BoardIdDecisionPoints) + 1 == ARRAY_SIZE(BoardTypeFractions));

// This table of floats is only used at compile time, so it shouldn't cause the floating point library to be pulled in
constexpr float BoardType2Fractions[] =
{
	3.0/(3.0 + 12.0),						// ATECM has 3K lower, 12K upper
	12.0/(12.0 + 3.0)						// AREIO has 12K lower 3K upper
};

static_assert(IsIncreasing(BoardType2Fractions, ARRAY_SIZE(BoardType2Fractions)));

// Table of halfway points that we use to decide what board type a reading corresponds to
constexpr uint16_t BoardId2DecisionPoints[] =
{
	(uint16_t)((BoardType2Fractions[0] + BoardType2Fractions[1]) * (AdcRange/2)),
};

static_assert(ARRAY_SIZE(BoardId2DecisionPoints) + 1 == ARRAY_SIZE(BoardType2Fractions));

// Buttons pin ADC handling for EXP1HCE board

constexpr float BothButtonsDownRgnd = (4.7 * 10.0)/(4.7 + 10.0);

constexpr float ButtonsExpectedFractions[] =
{
	BothButtonsDownRgnd/(BothButtonsDownRgnd + 10.0),
	4.7/(4.7 + 10.0),
	10.0/(1.00 + 10.0),
	1.0
};

static_assert(IsIncreasing(ButtonsExpectedFractions, ARRAY_SIZE(ButtonsExpectedFractions)));

// Table of halfway points that we use to decide what board type a reading corresponds to
constexpr uint16_t ButtonsDecisionPoints[] =
{
	(uint16_t)((ButtonsExpectedFractions[0] + ButtonsExpectedFractions[1]) * (AdcRange/2)),
	(uint16_t)((ButtonsExpectedFractions[1] + ButtonsExpectedFractions[2]) * (AdcRange/2)),
	(uint16_t)((ButtonsExpectedFractions[2] + ButtonsExpectedFractions[3]) * (AdcRange/2))
};

static_assert(ARRAY_SIZE(ButtonsDecisionPoints) + 1 == ARRAY_SIZE(ButtonsExpectedFractions));

// Function to read a pin and scan a table of decision points to find the corresponding index
static unsigned int ReadAndQuantise(uint8_t chan, const uint16_t decisionPoints[], size_t numDecisionPoints)
{
	const uint16_t reading = AnalogIn::ReadChannel(CommonAdcDevice, chan);
	unsigned int ainState = 0;
	while (ainState < numDecisionPoints && reading > decisionPoints[ainState])
	{
		++ainState;
	}
	return ainState;
}

bool IdentifyBoard(CanAddress& defaultAddress, bool& doHardwareReset, bool& useAlternateCanPins)
{
	// Read the board type pin, which is an analog input fed from a resistor network
	AnalogIn::Init(CommonAdcDevice);
	SetPinFunction(BoardTypePin, GpioPinFunction::B);

	boardTypeIndex = ReadAndQuantise(BoardTypeAdcChannel, BoardIdDecisionPoints, ARRAY_SIZE(BoardIdDecisionPoints));
	if (boardTypeIndex == (unsigned int)BoardId::ate)
	{
		// We need to read the second board ID pin
		SetPinFunction(BoardType2Pin, GpioPinFunction::B);			// ATE has a second board type pin
		boardTypeIndex = (unsigned int)BoardId::ate_base + ReadAndQuantise(BoardType2AdcChannel, BoardId2DecisionPoints, ARRAY_SIZE(BoardId2DecisionPoints));
	}

	// Set up the hardware and default CAN address as appropriate
	// Determine whether we need to do a hardware reset
	doHardwareReset = false;
	useAlternateCanPins = false;

	const Pin canResetPin = CanResetPins[boardTypeIndex];
	if (canResetPin != NoPin)
	{
		pinMode(canResetPin, INPUT_PULLUP);
		delayMicroseconds(100);
		doHardwareReset = !digitalRead(canResetPin);
	}

	switch ((BoardId)boardTypeIndex)
	{
	case BoardId::tool1lc_v0:
	case BoardId::tool1lc_v1:
		defaultAddress = CanId::ToolBoardDefaultAddress;

		pinMode(OutPins_Tool1LC[0], OUTPUT_LOW);					// V0.6 tool boards don't have pulldown resistors on the outputs, so turn them off
		pinMode(OutPins_Tool1LC[1], OUTPUT_LOW);					// V0.6 tool boards don't have pulldown resistors on the outputs, so turn them off
		pinMode(OutPins_Tool1LC[2], OUTPUT_HIGH);					// this is intended for the hot end fan, so turn it on just as the tool board firmware does
		pinMode(GlobalTmc22xxEnablePin_Tool1LC, OUTPUT_HIGH);
		break;

	case BoardId::exp1xd_v0:
		defaultAddress = CanId::Exp1XDBoardDefaultAddress;
		break;

	case BoardId::exp1hce_v0:
		defaultAddress = CanId::Exp1HCEBoardDefaultAddress;

		SetPinFunction(ButtonsPin_Exp1HCE, GpioPinFunction::B);		// both buttons are on a single analog pin
		{
			const unsigned int buttonState = ReadAndQuantise(ButtonsAdcChannel_Exp1HCE, ButtonsDecisionPoints, ARRAY_SIZE(ButtonsDecisionPoints));
			doHardwareReset = (buttonState == 0);
		}
		break;

	case BoardId::ate_cm:
		useAlternateCanPins = true;
		// ATE CM board has the reset jumper fitted between AteCmZeroPin and AteCmJumperPin
		defaultAddress = CanId::ATECMBoardDefaultAddress;
		pinMode(AteCmZeroPin, OUTPUT_LOW);
		pinMode(AteCmJumperPin, INPUT_PULLUP);
		delayMicroseconds(100);
		doHardwareReset = !digitalRead(AteCmJumperPin);
		pinMode(AteCmZeroPin, INPUT_PULLUP);
		break;

	case BoardId::ate_io:
	case BoardId::ate:
	default:
		useAlternateCanPins = true;
		defaultAddress = CanId::ATEIOBoardDefaultAddress;
		break;
	}

	AnalogIn::Disable(CommonAdcDevice);			// finished using the ADC
	return true;
}

# endif

#elif SAME70

# if defined(MB6HC)
constexpr const char* BoardTypeNames[] = { "MB6HC" };
constexpr unsigned int BoardTypeVersions[] = { 0 };
constexpr const Pin *LedPinsTables[] = { LedPins_MB6HC };
constexpr bool LedActiveHigh[] = { LedActiveHigh_MB6HC };
# elif defined(MB6XD)
constexpr const char* BoardTypeNames[] = { "MB6XD" };
constexpr unsigned int BoardTypeVersions[] = { 0 };
constexpr const Pin *LedPinsTables[] = { LedPins_MB6XD };
constexpr bool LedActiveHigh[] = { LedActiveHigh_MB6XD };
# endif

bool IdentifyBoard(CanAddress& defaultAddress, bool& doHardwareReset, bool& useAlternateCanPins)
{
	defaultAddress = 0;
	doHardwareReset = false;
	useAlternateCanPins = false;
	return true;
}

#else
# error Unsupported board
#endif

static_assert(ARRAY_SIZE(BoardTypeVersions) == ARRAY_SIZE(BoardTypeNames));
static_assert(ARRAY_SIZE(LedPinsTables) == ARRAY_SIZE(BoardTypeNames));
static_assert(ARRAY_SIZE(LedActiveHigh) == ARRAY_SIZE(BoardTypeNames));

Pin GetLedPin(unsigned int ledNumber)
{
	const Pin p = LedPinsTables[boardTypeIndex][ledNumber];
#if defined(DEBUG) && (SAME5x || SAMC21)
	if (p == PortAPin(30) || p == PortAPin(31))
	{
		// Using the SWD pins to drive the LEDs. Don't allow this in a debug build because it prevents debugging.
		return NoPin;
	}
#endif
	return p;
}

bool GetLedActiveHigh()
{
	return LedActiveHigh[boardTypeIndex];
}

const char *GetBoardTypeName()
{
	return BoardTypeNames[boardTypeIndex];
}

const unsigned int GetBoardVersion()
{
	return BoardTypeVersions[boardTypeIndex];
}

// End
