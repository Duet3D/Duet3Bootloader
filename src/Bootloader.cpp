/*
 * Main.cpp
 *
 *  Created on: 3 Aug 2019
 *      Author: David
 */

#include <RepRapFirmware.h>
#include <Hardware/Devices.h>
#include <CAN/CanInterface.h>
#include <Flash.h>
#include <Serial.h>
#include <Config/BoardDef.h>
#include <General/StringRef.h>
#include <CanId.h>
#include <CanMessageBuffer.h>
#include <Duet3Common.h>

#if SAME5x

constexpr uint32_t FlashBlockSize = 0x00010000;							// the erase size we assume for flash (64K)
constexpr const char* BoardTypeName = "EXP3HC";

#elif SAMC21

constexpr uint32_t FlashBlockSize = 0x00004000;							// the erase size we assume for flash (16K)

# ifdef SAMMYC21

constexpr const char* BoardTypeName = "SAMMYC21";

# else

#include <AnalogIn.h>

// Board ID analog pin handling
constexpr uint32_t AdcRange = 1 << 16;			// we use the ADC in 16-bit mode

// Constexpr function to check that a table is in increasing order
inline constexpr bool IsIncreasing(const float *arr, size_t length)
{
	return length < 2 || (arr[1] > arr[0] && IsIncreasing(arr + 1, length - 1));
}

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

	// ATE board types
	ate_base,
	ate_cm = ate_base,
	ate_io
};

constexpr const char * BoardTypeNames[] =
{
	// Boards selected by just the first board type pin
	"TOOL1LC",
	"EXP1HCE",
	"unknown",			// for ATE we need to look at the second board type pin
	"EXP1XD",

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
	0,
	0
};

constexpr const Pin *LedPinsTables[] =
{
	LedPins_Tool1LC,
	LedPins_Exp1HCE,
	LedPins_Ate,
	LedPins_Exp1XD,
	LedPins_Ate,
	LedPins_Ate,
};

constexpr bool LedActiveHigh[] =
{
	LedActiveHigh_Tool1LC,
	LedActiveHigh_Exp1HCE,
	LedActiveHigh_Ate,
	LedActiveHigh_Exp1XD,
	LedActiveHigh_Ate,
	LedActiveHigh_Ate,
};

static_assert(ARRAY_SIZE(BoardTypeVersions) == ARRAY_SIZE(BoardTypeNames));
static_assert(ARRAY_SIZE(LedPinsTables) == ARRAY_SIZE(BoardTypeNames));
static_assert(ARRAY_SIZE(LedActiveHigh) == ARRAY_SIZE(BoardTypeNames));

// This table of floats is only used at compile time, so it shouldn't cause the floating point library to be pulled in
constexpr float BoardTypeFractions[] =
{
	1.0/(1.0 + 10.0),						// TOOL1LC has 1K lower resistor, 10K upper
	4.7/(4.7 + 27.0),						// EXP1HCE has 4K7 lower resistor, 27K upper
	3.0/(3.0 + 12.0),						// ATECM and ATEIO have 3K lower, 12K upper
	4.7/(4.7 + 4.7),						// EXP1XD has 4K7 lower resistor, 4K7 upper
};

static_assert(IsIncreasing(BoardTypeFractions, ARRAY_SIZE(BoardTypeFractions)));

// Table of halfway points that we use to decide what board type a reading corresponds to
constexpr uint16_t BoardIdDecisionPoints[] =
{
	(uint16_t)((BoardTypeFractions[0] + BoardTypeFractions[1]) * (AdcRange/2)),
	(uint16_t)((BoardTypeFractions[1] + BoardTypeFractions[2]) * (AdcRange/2)),
	(uint16_t)((BoardTypeFractions[2] + BoardTypeFractions[3]) * (AdcRange/2))
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
unsigned int ReadAndQuantise(uint8_t chan, const uint16_t decisionPoints[], size_t numDecisionPoints)
{
	const uint16_t reading = AnalogIn::ReadChannel(CommonAdcDevice, chan);
	unsigned int ainState = 0;
	while (ainState < numDecisionPoints && reading > decisionPoints[ainState])
	{
		++ainState;
	}
	return ainState;
}

# endif

#else
# error Unsupported board
#endif

constexpr uint32_t BlockReceiveTimeout = 2000;								// block receive timeout milliseconds
constexpr uint32_t FirmwareFlashStart = FLASH_ADDR + FlashBlockSize;		// the amount of space we reserve for the bootloader

#if SAMC21 && !defined(SAMMYC21)

unsigned int boardTypeIndex;

inline Pin GetLedPin(unsigned int ledNumber)
{
	const Pin p = LedPinsTables[boardTypeIndex][ledNumber];
#ifdef DEBUG
	if (p == PortAPin(30) || p == PortAPin(31))
	{
		// Using the SWD pins to drive the LEDs. Don't allow this in a debug build because it prevents debugging.
		return NoPin;
	}
#endif
	return p;
}

inline bool GetLedActiveHigh()
{
	return LedActiveHigh[boardTypeIndex];
}

#else

inline Pin GetLedPin(unsigned int ledNumber)
{
	return LedPins[ledNumber];
}

inline bool GetLedActiveHigh()
{
	return LedActiveHigh;
}

#endif

static uint8_t blockBuffer[FlashBlockSize];

#if SAME5x
uint8_t ReadBoardAddress();
#endif

bool CheckValidFirmware();
void StartFirmware();

static inline void WriteLed(uint8_t ledNumber, bool turnOn)
{
	if (ledNumber < NumLedPins)
	{
		digitalWrite(GetLedPin(ledNumber), (GetLedActiveHigh()) ? turnOn : !turnOn);
	}
}

// System tick ISR, used for timing functions
extern "C" void SysTick_Handler()
{
	CoreSysTick();
}

void SerialMessage(const char *text)
{
#ifdef DEBUG
# ifdef SAMMYC21
	// Messages go to the USB port, so send them raw
	uart0.print(text);
	uart0.print("\n");
# else
	// Assume a PanelDue is connected, so encapsulate the message
	uart0.print("\n{\"message\":\"");
	uart0.print(text);									// should do json escaping here but for now just be careful what messages we send
	uart0.print("\"}\n");
# endif
	uart0.flush();
	delay(3);											// allow time for the last character to go
#endif
}

void ReportError(const char *text, FirmwareFlashErrorCode err)
{
	SerialMessage(text);

	for (unsigned int i = 0; i < (unsigned int)err; ++i)
	{
		WriteLed(0, true);
		delay(200);
		WriteLed(0, false);
		delay(200);
	}

	delay(1000);
}

[[noreturn]] void Restart()
{
	SCB->AIRCR = (0x5FA << 16) | (1u << 2);				// reset the processor
	for (;;) { }
}

[[noreturn]] void ReportErrorAndRestart(const char *text, FirmwareFlashErrorCode err)
{
	CanInterface::Shutdown();
	ReportError(text, err);
	delay(2000);
	Restart();
}

void RequestFirmwareBlock(uint32_t fileOffset, uint32_t numBytes)
{
	CanMessageBuffer *buf = CanMessageBuffer::Allocate();
	if (buf == nullptr)
	{
		ReportErrorAndRestart("No buffers", FirmwareFlashErrorCode::noBuffer);
	}
	CanMessageFirmwareUpdateRequest * const msg = buf->SetupRequestMessage<CanMessageFirmwareUpdateRequest>(0, CanInterface::GetCanAddress(), CanId::MasterAddress);
#if defined(SAMMYC21) || SAME5x
	SafeStrncpy(msg->boardType, BoardTypeName, sizeof(msg->boardType));
	msg->boardVersion = 0;
#else
	SafeStrncpy(msg->boardType, BoardTypeNames[boardTypeIndex], sizeof(msg->boardType));
	msg->boardVersion = BoardTypeVersions[boardTypeIndex];
#endif
	msg->bootloaderVersion = CanMessageFirmwareUpdateRequest::BootloaderVersion0;
	msg->fileWanted = (uint32_t)FirmwareModule::main;
	msg->fileOffset = fileOffset;
	msg->lengthRequested = numBytes;
	buf->dataLength = msg->GetActualDataLength();
	CanInterface::SendAndFree(buf);
}

// Get a buffer of data from the host
void GetBlock(uint32_t startingOffset, uint32_t& fileSize)
{
	constexpr Pin CanLedNumber = (NumLedPins >= 2) ? 1 : 0;
	WriteLed(CanLedNumber, true);
	delay(25);														// flash the LED briefly to indicate we are requesting a new flash block
	WriteLed(CanLedNumber, false);

	RequestFirmwareBlock(startingOffset, FlashBlockSize);			// ask for 16K or 64K from the starting offset

	CanMessageBuffer *buf = CanMessageBuffer::Allocate();
	if (buf == nullptr)
	{
		ReportErrorAndRestart("No buffers", FirmwareFlashErrorCode::noBuffer);
	}

	uint32_t whenStartedWaiting = millis();
	uint32_t bytesReceived = 0;
	bool done = false;
	do
	{
		const bool ok = CanInterface::GetCanMessage(buf);
		if (ok)
		{
			if (buf->id.MsgType() == CanMessageType::firmwareBlockResponse)
			{
				const CanMessageFirmwareUpdateResponse& response = buf->msg.firmwareUpdateResponse;
				switch (response.err)
				{
				case CanMessageFirmwareUpdateResponse::ErrNoFile:
					ReportErrorAndRestart("Host reported no file", FirmwareFlashErrorCode::noFile);
				case CanMessageFirmwareUpdateResponse::ErrBadOffset:
					ReportErrorAndRestart("Host reported bad offset", FirmwareFlashErrorCode::badOffset);
				case CanMessageFirmwareUpdateResponse::ErrOther:
					ReportErrorAndRestart("Host reported other error", FirmwareFlashErrorCode::hostOther);
				case CanMessageFirmwareUpdateResponse::ErrNone:
					if (response.fileOffset >= startingOffset && response.fileOffset <= startingOffset + bytesReceived)
					{
						const uint32_t bufferOffset = response.fileOffset - startingOffset;
						const uint32_t bytesToCopy = min<uint32_t>(sizeof(blockBuffer) - bufferOffset, response.dataLength);
						memcpy(blockBuffer + bufferOffset, response.data, bytesToCopy);
						if (response.fileOffset + bytesToCopy > startingOffset + bytesReceived)
						{
							bytesReceived = response.fileOffset - startingOffset + bytesToCopy;
						}
						if (bytesReceived == FlashBlockSize || bytesReceived >= response.fileLength - startingOffset)
						{
							// Reached the end of the file
							memset(blockBuffer + bytesReceived, 0xFF, sizeof(blockBuffer) - bytesReceived);
							fileSize = response.fileLength;
							done = true;
						}
					}
					whenStartedWaiting = millis();
				}
			}
		}
		else if (millis() - whenStartedWaiting > BlockReceiveTimeout)
		{
			if (bytesReceived == 0)
			{
				ReportErrorAndRestart("Block receive timeout", FirmwareFlashErrorCode::blockReceiveTimeout);
			}
			RequestFirmwareBlock(startingOffset + bytesReceived, FlashBlockSize - bytesReceived);			// ask for 64K from the starting offset
			whenStartedWaiting = millis();
		}
	} while (!done);

	CanMessageBuffer::Free(buf);
}

// Clock configuration:
// SAME5x:
//	XOSC1 = 12MHz or 25MHz crystal oscillator
//	FDPLL0 = takes XOSC1 divide by 4 (3MHz), multiplies by 40 to get 120MHz main clock
//	FDPLL1 = takes XOSC1 divide by 4 (3MHz), multiplies by 12 to get 48MHz CAN clock
//	GCLK0 = takes FDPLL0 output, no divisor, giving 120MHz main clock used by CPU
//	GCLK1 = takes FDPLL0 output, divided by 2 to get 60MHz clock used by most peripherals
//	GCLK2 = takes FDPLL1 output, no divisor, giving 48MHz CAN clock
// SAMC21:
//	XOSC1 = 12MHz or 25MHz crystal oscillator (16MHz on Sammy-C21 board)
//	FDPLL = takes XOSC1 divide by 6 (8 for Sammy-C21) (2MHz), multiplied by 24 to get 48MHz main clock
//	GCLK0 = takes FDPLL output, no divisor, giving 48MHz main clock used by CPU and most peripherals
void AppMain()
{
	// Initialise systick (needed for delay calls to work)
	SysTick->LOAD = ((SystemCoreClockFreq/1000) - 1) << SysTick_LOAD_RELOAD_Pos;
	SysTick->CTRL = (1 << SysTick_CTRL_ENABLE_Pos) | (1 << SysTick_CTRL_TICKINT_Pos) | (1 << SysTick_CTRL_CLKSOURCE_Pos);
	NVIC_EnableIRQ(SysTick_IRQn);

	DeviceInit();

#if SAME5x

	for (Pin p : BoardAddressPins)
	{
		pinMode(p, INPUT_PULLUP);
	}

	// Check whether address switches are set to zero. If so then reset and load new firmware
	const CanAddress switches = ReadBoardAddress();
	const bool doHardwareReset = (switches == 0);
	const CanAddress defaultAddress = (doHardwareReset) ? CanId::ExpansionBoardFirmwareUpdateAddress : switches;
	const bool useAlternatCanPins = false;

#elif SAMC21

	// Establish the board type and initialise pins

# ifdef SAMMYC21
	pinMode(ButtonPins[0], INPUT_PULLUP);
	const CanAddress defaultAddress = CanId::SammyC21DefaultAddress;
	const bool doHardwareReset = !digitalRead(ButtonPins[0]);
	const bool useAlternatCanPins = true;
# else

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
	bool doHardwareReset = false, useAlternatCanPins = false;
	CanAddress defaultAddress;

	switch ((BoardId)boardTypeIndex)
	{
	case BoardId::tool1lc_v0:
		defaultAddress = CanId::ToolBoardDefaultAddress;

		pinMode(OutPins_Tool1LC[0], OUTPUT_LOW);					// V0.6 tool boards don't have pulldown resistors on the outputs, so turn them off
		pinMode(OutPins_Tool1LC[1], OUTPUT_LOW);					// V0.6 tool boards don't have pulldown resistors on the outputs, so turn them off
		pinMode(OutPins_Tool1LC[2], OUTPUT_HIGH);					// this is intended for the hot end fan, so turn it on just as the tool board firmware does
		pinMode(GlobalTmc22xxEnablePin_Tool1LC, OUTPUT_HIGH);

		pinMode(ButtonPins_Tool1LC[0], INPUT_PULLUP);
		pinMode(ButtonPins_Tool1LC[1], INPUT_PULLUP);


		// If both button pins are pressed, reset and load new firmware
		delayMicroseconds(100);
		doHardwareReset = !digitalRead(ButtonPins_Tool1LC[0]) && !digitalRead(ButtonPins_Tool1LC[1]);
		break;

	case BoardId::exp1xd_v0:
		defaultAddress = CanId::Exp1XDBoardDefaultAddress;

		pinMode(JumperPin_Exp1XD, INPUT_PULLUP);
		delayMicroseconds(100);
		doHardwareReset = !digitalRead(JumperPin_Exp1XD);
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
		useAlternatCanPins = true;
		// ATE CM board has a reset jumper fitted between AteCmZeroPin and AteCmJumperPin
		defaultAddress = CanId::ATECMBoardDefaultAddress;
		pinMode(AteCmZeroPin, OUTPUT_LOW);
		pinMode(AteCmJumperPin, INPUT_PULLUP);
		delayMicroseconds(100);
		doHardwareReset = !digitalRead(AteCmJumperPin);
		pinMode(AteCmZeroPin, INPUT_PULLUP);
		break;

	case BoardId::ate_io:
		useAlternatCanPins = true;
		defaultAddress = CanId::ATEIOBoardDefaultAddress;
		pinMode(AteIoJumperPin, INPUT_PULLUP);
		delayMicroseconds(100);
		doHardwareReset = !digitalRead(AteIoJumperPin);
		break;

	default:									// should never be used
		defaultAddress = CanId::ATEIOBoardDefaultAddress;
		break;
	}

	AnalogIn::Disable(CommonAdcDevice);			// finished using the ADC

# endif
#else
# error Unsupported processor
#endif

	for (unsigned int ledNumber = 0; ledNumber < NumLedPins; ++ledNumber)
	{
		pinMode(GetLedPin(ledNumber), (GetLedActiveHigh()) ? OUTPUT_LOW : OUTPUT_HIGH);
	}

#ifdef DEBUG
	uart0.begin(57600);
#endif

	if (!doHardwareReset && CheckValidFirmware())
	{
		// Relocate the vector table and jump into the firmware. If it returns then we execute the bootloader.
		StartFirmware();
	}

	// If we get here then we are staying in the bootloader
	// Initialise the CAN subsystem
	if (!Flash::Init())
	{
		ReportErrorAndRestart("Failed to initialize flash controller", FirmwareFlashErrorCode::flashInitFailed);
	}

	CanInterface::Init(defaultAddress, doHardwareReset, useAlternatCanPins);

	// Loop requesting firmware from the main board and handling any firmware that it sends to us
	uint32_t bufferStartOffset = 0;
	uint32_t roundedUpLength;
	for (;;)
	{
		uint32_t fileSize;
		GetBlock(bufferStartOffset, fileSize);
		if (bufferStartOffset == 0)
		{
			// First block received, so unlock and erase the firmware
			roundedUpLength = ((fileSize + (FlashBlockSize - 1))/FlashBlockSize) * FlashBlockSize;
			SerialMessage("Unlocking flash");
			if (!Flash::Unlock(FirmwareFlashStart, roundedUpLength))
			{
				ReportErrorAndRestart("Failed to unlock flash", FirmwareFlashErrorCode::unlockFailed);
			}
			SerialMessage("Erasing flash");
			if (!Flash::Erase(FirmwareFlashStart, roundedUpLength))
			{
				ReportErrorAndRestart("Failed to erase flash", FirmwareFlashErrorCode::eraseFailed);
			}
		}

		// If we have both red and green LEDs, the green one indicates CAN activity. use the red one to indicate writing to flash.
		if (NumLedPins == 2)
		{
			WriteLed(0, true);
		}

		if (!Flash::Write(FirmwareFlashStart + bufferStartOffset, FlashBlockSize, blockBuffer))
		{
			ReportErrorAndRestart("Failed to write flash", FirmwareFlashErrorCode::writeFailed);
		}

		if (NumLedPins == 2)
		{
			WriteLed(0, false);
		}

		bufferStartOffset += FlashBlockSize;
		if (bufferStartOffset >= fileSize)
		{
			break;
		}
	}

	// If we get here, firmware update is complete
	if (!Flash::Lock(FirmwareFlashStart, roundedUpLength))
	{
		ReportErrorAndRestart("Failed to lock flash", FirmwareFlashErrorCode::lockFailed);
	}

	CanInterface::Shutdown();

	delay(2);

	NVIC_DisableIRQ(CAN0_IRQn);
	NVIC_DisableIRQ(CAN1_IRQn);
	CAN0->IR.reg = 0xFFFFFFFF;			// clear all interrupt sources for when the device gets enabled by the main firmware
	CAN0->ILE.reg = 0;
	CAN1->IR.reg = 0xFFFFFFFF;			// clear all interrupt sources for when the device gets enabled by the main firmware
	CAN1->ILE.reg = 0;

	SerialMessage("Finished firmware update");
	delay(1000);

	if (!CheckValidFirmware())
	{
		Restart();
	}

	StartFirmware();
}

#if SAME5x

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

#endif

// Check that valid firmware is installed. If not, report the error and return false.
bool CheckValidFirmware()
{
	const DeviceVectors * const vectors = reinterpret_cast<const DeviceVectors*>(FirmwareFlashStart);
	if (   reinterpret_cast<uint32_t>(vectors->pfnReset_Handler) < FirmwareFlashStart
		|| reinterpret_cast<uint32_t>(vectors->pfnReset_Handler) >= FLASH_ADDR + FLASH_SIZE
		|| reinterpret_cast<uint32_t>(vectors->pvStack) < HSRAM_ADDR
		|| reinterpret_cast<uint32_t>(vectors->pvStack) > HSRAM_ADDR + HSRAM_SIZE
		|| reinterpret_cast<uint32_t>(vectors->pvReservedM9) < FirmwareFlashStart
		|| reinterpret_cast<uint32_t>(vectors->pvReservedM9) > FirmwareFlashStart + FLASH_SIZE - 4
	   )
	{
		ReportError("Invalid firmware", FirmwareFlashErrorCode::invalidFirmware);
		return false;
	}

	// Fetch the CRC-32 from the file
	const uint32_t *crcAddr = (const uint32_t*)(vectors->pvReservedM9);
	const uint32_t storedCRC = *crcAddr;

	// Compute the CRC-32 of the file
#if SAME5x
	DMAC->CRCCTRL.reg = DMAC_CRCCTRL_CRCBEATSIZE_WORD | DMAC_CRCCTRL_CRCSRC_DISABLE | DMAC_CRCCTRL_CRCPOLY_CRC32;	// disable the CRC unit
#elif SAMC21
	DMAC->CTRL.bit.CRCENABLE = 0;
#else
# error Unsupported processor
#endif
	DMAC->CRCCHKSUM.reg = 0xFFFFFFFF;
	DMAC->CRCCTRL.reg = DMAC_CRCCTRL_CRCBEATSIZE_WORD | DMAC_CRCCTRL_CRCSRC_IO | DMAC_CRCCTRL_CRCPOLY_CRC32;
#if SAMC21
	DMAC->CTRL.bit.CRCENABLE = 1;
#endif
	for (const uint32_t *p = reinterpret_cast<const uint32_t*>(FirmwareFlashStart); p < crcAddr; ++p)
	{
		DMAC->CRCDATAIN.reg = *p;
		asm volatile("nop");
		asm volatile("nop");
	}

	DMAC->CRCSTATUS.reg = DMAC_CRCSTATUS_CRCBUSY;
	asm volatile("nop");
	const uint32_t actualCRC = DMAC->CRCCHKSUM.reg;
	if (actualCRC == storedCRC)
	{
		return true;
	}

	String<100> message;
	message.printf("CRC error: stored %08" PRIx32 ", actual %" PRIx32, storedCRC, actualCRC);
	ReportError(message.c_str(), FirmwareFlashErrorCode::badCRC);
	return false;
}

// Execute the main firmware CAN1
[[noreturn]] void StartFirmware()
{
#ifdef DEBUG
	SerialMessage("Bootloader transferring control to main firmware");
	uart0.end();										// disable serial port so that main firmware can initialise it
#endif

	// Disable all IRQs
	SysTick->CTRL = (1 << SysTick_CTRL_CLKSOURCE_Pos);	// disable the system tick exception
	__disable_irq();

#if SAME5x

	for (size_t i = 0; i < 8; i++)
	{
		NVIC->ICER[i] = 0xFFFFFFFF;						// Disable IRQs
		NVIC->ICPR[i] = 0xFFFFFFFF;						// Clear pending IRQs
	}

	// Reset the generic clock generator. This sets all clock generators to default values and the CPU clock to the 48MHz DFLL output.
	GCLK->CTRLA.reg = GCLK_CTRLA_SWRST;
	while ((GCLK->CTRLA.reg & GCLK_CTRLA_SWRST) != 0) { }

	// Disable DPLL0 and DPLL1 so hat they can be reprogrammed by the main firmware
	OSCCTRL->Dpll[0].DPLLCTRLA.bit.ENABLE = 0;
	while (OSCCTRL->Dpll[0].DPLLSYNCBUSY.bit.ENABLE) { }
	OSCCTRL->Dpll[1].DPLLCTRLA.bit.ENABLE = 0;
	while (OSCCTRL->Dpll[1].DPLLSYNCBUSY.bit.ENABLE) { }

#elif SAMC21

	NVIC->ICER[0] = 0xFFFFFFFF;							// Disable IRQs
	NVIC->ICPR[0] = 0xFFFFFFFF;							// Clear pending IRQs

	// Switch back to the OSC48M clock divided to 4MHz
# if 1
	// 2020-06-03: on the SammyC21 board the software reset of GCLK never completed, so reset it manually
	OSCCTRL->OSC48MCTRL.reg = OSCCTRL_OSC48MCTRL_ENABLE;				// make sure OSC48M is enabled, clear the on-demand bit
	while ((OSCCTRL->STATUS.reg & OSCCTRL_STATUS_OSC48MRDY) == 0) { }	// wait for it to become ready
	GCLK->GENCTRL[0].reg = 0x00000106;									// this is the reset default
	OSCCTRL->OSC48MCTRL.reg = OSCCTRL_OSC48MCTRL_ENABLE | OSCCTRL_OSC48MCTRL_ONDEMAND;		// back to reset default
# else
	// The following code works on Duet3D boards, but it hangs on the SammyC21 with device ID 0x11010405 (SAMC21G18A die revision E)
	GCLK->CTRLA.reg = GCLK_CTRLA_SWRST;
	while ((GCLK->CTRLA.reg & GCLK_CTRLA_SWRST) != 0) { }
# endif

	// Disable the DPLL so that it can be reprogrammed by the main firmware
	OSCCTRL->DPLLCTRLA.bit.ENABLE = 0;
	while (OSCCTRL->DPLLSYNCBUSY.bit.ENABLE) { }

#else
# error Unsupported processor
#endif

	for (size_t i = 0; i < NumLedPins; ++i)
	{
		WriteLed(i, false);								// turn all LEDs off
	}

//	hri_wdt_write_CLEAR_reg(WDT, WDT_CLEAR_CLEAR_KEY);	// reset the watchdog timer

	// Modify vector table location
	__DSB();
	__ISB();
	SCB->VTOR = FirmwareFlashStart & SCB_VTOR_TBLOFF_Msk;
	__DSB();
	__ISB();

	__asm volatile ("mov r3, %0" : : "r" (FirmwareFlashStart) : "r3");

	__asm volatile ("ldr r1, [r3]");
	__asm volatile ("msr msp, r1");
	__asm volatile ("mov sp, r1");

	__asm volatile ("isb");
	__enable_irq();

	__asm volatile ("ldr r1, [r3, #4]");
#if SAME5x
	__asm volatile ("orr r1, r1, #1");
#elif SAMC21
	__asm volatile ("movs r2, #1");
	__asm volatile ("orr r1, r1, r2");
#else
# error Unsupported processor
#endif
	__asm volatile ("bx r1");

	// This point is unreachable, but gcc doesn't seem to know that
	for (;;) { }
}

// Function needed by CoreNG. It doesn't need to do anything in this instance.
void OutOfMemoryHandler() { }

// End
