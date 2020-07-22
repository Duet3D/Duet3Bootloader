/*
 * Main.cpp
 *
 *  Created on: 3 Aug 2019
 *      Author: David
 */

#include <RepRapFirmware.h>
#include <CAN/CanInterface.h>
#include <Config/peripheral_clk_config.h>
#include <Hardware/Io.h>
#include <Hardware/Flash.h>
#include <Hardware/Serial.h>
#include <Hardware/CanDriver.h>
#include <Config/BoardDef.h>
#include <General/StringRef.h>
#include <CanId.h>
#include <CanMessageBuffer.h>

#if SAME5x

constexpr uint32_t FlashBlockSize = 0x00010000;							// the block size we assume for flash
constexpr uint32_t BlockReceiveTimeout = 2000;							// block receive timeout milliseconds
const uint32_t FirmwareFlashStart = FLASH_ADDR + FlashBlockSize;		// we reserve 64K for the bootloader
constexpr const char* BoardTypeName = "EXP3HC";

#elif SAMC21

constexpr uint32_t FlashBlockSize = 0x00004000;							// the block size we assume for flash
constexpr uint32_t BlockReceiveTimeout = 2000;							// block receive timeout milliseconds
const uint32_t FirmwareFlashStart = FLASH_ADDR + FlashBlockSize;		// we reserve 16K for the bootloader

# ifdef SAMMYC21

constexpr const char* BoardTypeName = "SAMMYC21";

# else

#include <Hardware/SimpleAnalogIn.h>

// Board ID analog pin handling
constexpr uint32_t AdcRange = 1 << 16;			// we use the ADC in 16-bit mode

// Names and board ID pin resistor ratios for the board we support
// The expected ADC readings must be in increasing order, and the board names must be in the corresponding order
constexpr const char * BoardTypeNames[] =
{
	"TOOL1LC",
	"EXP1HCE",
	"EXP1XD"
};

constexpr unsigned int BoardTypeVersions[] =
{
	0,
	0,
	0
};

// The board type and version as an enumeration
enum class BoardId : unsigned int
{
	tool1lc_v0,
	exp1hce_v0,
	exp1xd_v0
};

constexpr const Pin *LedPinsTables[] =
{
	LedPins_Tool1LC,
	LedPins_Exp1HCE,
	LedPins_Exp1XD
};

constexpr bool LedActiveHigh[] =
{
	LedActiveHigh_Tool1LC,
	LedActiveHigh_Exp1HCE,
	LedActiveHigh_Exp1XD
};

// This table of floats is only used at compile time, so it shouldn't cause the floating point library to be pulled in
constexpr float BoardTypeFractions[] =
{
	1.0/(1.0 + 10.0),						// TOOL1LC has 1K lower resistor, 10K upper
	4.7/(4.7 + 27.0),						// EXP1HCE has 4K7 lower resistor, 27K upper
	4.7/(4.7 + 4.7),						// EXP1XD has 4K7 lower resistor, 4K7 upper
};

static_assert(ARRAY_SIZE(BoardTypeNames) == ARRAY_SIZE(BoardTypeFractions));
static_assert(ARRAY_SIZE(BoardTypeVersions) == ARRAY_SIZE(BoardTypeFractions));
static_assert(ARRAY_SIZE(LedPinsTables) == ARRAY_SIZE(BoardTypeFractions));
static_assert(ARRAY_SIZE(LedActiveHigh) == ARRAY_SIZE(BoardTypeFractions));

inline constexpr bool IsIncreasing(const float *arr, size_t length)
{
	return length < 2 || (arr[1] > arr[0] && IsIncreasing(arr + 1, length - 1));
}

static_assert(IsIncreasing(BoardTypeFractions, ARRAY_SIZE(BoardTypeFractions)));

// Table of halfway points that we use to decide what board type a reading corresponds to
constexpr uint16_t BoardIdDecisionPoints[] =
{
	(uint16_t)((BoardTypeFractions[0] + BoardTypeFractions[1]) * (AdcRange/2)),
	(uint16_t)((BoardTypeFractions[1] + BoardTypeFractions[2]) * (AdcRange/2))
};

static_assert(ARRAY_SIZE(BoardIdDecisionPoints) + 1 == ARRAY_SIZE(BoardTypeFractions));

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

# endif

#else
# error Unsupported board
#endif

// Error codes, presented as a number of flashes of the DIAG LED
enum class ErrorCode : unsigned int
{
	invalidFirmware = 2,
	badCRC = 3,
	blockReceiveTimeout = 4,
	noFile = 5,
	badOffset = 6,
	hostOther = 7,
	noBuffer = 8,
	flashInitFailed = 9,
	unlockFailed = 10,
	eraseFailed = 11,
	writeFailed = 12,
	lockFailed = 13
};

#if SAMC21 && !defined(SAMMYC21)

unsigned int boardTypeIndex;

inline Pin GetLedPin(unsigned int ledNumber)
{
	return LedPinsTables[boardTypeIndex][ledNumber];
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

volatile static uint32_t tickCount = 0;

inline uint32_t millis()
{
	return tickCount;
}

void delay(uint32_t ticks)
{
	const uint32_t start = millis();
	while (millis() - start < ticks) { }
}

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
	++tickCount;
}

void SerialMessage(const char *text)
{
#ifdef SAMMYC21
	// Messages go to the USB port, so send them raw
	Serial::Send(text);
	Serial::Send("\n");
#else
	// Assume a PanelDue is connected, so encapsulate the message
	Serial::Send("\n{\"message\":\"");
	Serial::Send(text);									// should do json escaping here but for now just be careful what messages we send
	Serial::Send("\"}\n");
#endif
	delay(3);											// allow time for the last character to go
}

void ReportError(const char *text, ErrorCode err)
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

[[noreturn]] void ReportErrorAndRestart(const char *text, ErrorCode err)
{
	ReportError(text, err);
	delay(2000);
	Restart();
}

void RequestFirmwareBlock(uint32_t fileOffset, uint32_t numBytes)
{
	CanMessageBuffer *buf = CanMessageBuffer::Allocate();
	if (buf == nullptr)
	{
		ReportErrorAndRestart("No buffers", ErrorCode::noBuffer);
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
	msg->fileOffset = fileOffset;
	msg->lengthRequested = numBytes;
	buf->dataLength = msg->GetActualDataLength();
	CanInterface::Send(buf);
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
		ReportErrorAndRestart("No buffers", ErrorCode::noBuffer);
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
					ReportErrorAndRestart("Host reported no file", ErrorCode::noFile);
				case CanMessageFirmwareUpdateResponse::ErrBadOffset:
					ReportErrorAndRestart("Host reported bad offset", ErrorCode::badOffset);
				case CanMessageFirmwareUpdateResponse::ErrOther:
					ReportErrorAndRestart("Host reported other error", ErrorCode::hostOther);
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
				ReportErrorAndRestart("Block receive timeout", ErrorCode::blockReceiveTimeout);
			}
			RequestFirmwareBlock(startingOffset + bytesReceived, FlashBlockSize - bytesReceived);			// ask for 64K from the starting offset
			whenStartedWaiting = millis();
		}
	} while (!done);

	CanMessageBuffer::Free(buf);
}

// Clock configuration:
// ATSAME51:
//	XOSC1 = 12MHz crystal oscillator
//	FDPLL0 = takes XOSC1 divide by 4 (3MHz), multiplies by 40 to get 120MHz main clock
//	FDPLL1 = takes XOSC1 divide by 4 (3MHz), multiplies by 12 to get 48MHz CAN clock
//	GCLK0 = takes FDPLL0 output, no divisor, giving 120MHz main clock used by CPU
//	GCLK1 = takes FDPLL0 output, divided by 2 to get 60MHz clock used by most peripherals
//	GCLK2 = takes FDPLL1 output, no divisor, giving 48MHz CAN clock
// ATSAMC21:
//	XOSC1 = 12MHz crystal oscillator (16MHz on Sammy-C21 board)
//	FDPLL = takes XOSC1 divide by 6 (8 for Sammy-C21) (2MHz), multiplied by 24 to get 48MHz main clock
//	GCLK0 = takes FDPLL output, no divisor, giving 48MHz main clock used by CPU and most peripherals
extern "C" int main()
{
	atmel_start_init();
	SystemCoreClock = CONF_CPU_FREQUENCY;

#if SAME5x
	SystemPeripheralClock = CONF_CPU_FREQUENCY/2;
#elif SAMC21
	SystemPeripheralClock = CONF_CPU_FREQUENCY;
#endif

	// Initialise systick (needed for delay calls) and serial
	SysTick->LOAD = ((CONF_CPU_FREQUENCY/1000) - 1) << SysTick_LOAD_RELOAD_Pos;
	SysTick->CTRL = (1 << SysTick_CTRL_ENABLE_Pos) | (1 << SysTick_CTRL_TICKINT_Pos) | (1 << SysTick_CTRL_CLKSOURCE_Pos);
	Serial::Init();

#if SAME5x

	for (Pin p : BoardAddressPins)
	{
		SetPinMode(p, INPUT_PULLUP);
	}

#elif SAMC21

	// Establish the board type and initialise pins

# ifdef SAMMYC21
	SetPinMode(ButtonPins[0], INPUT_PULLUP);
# else

	// Read the board type pin, which is an analog input fed from a resistor network
	SimpleAnalogIn::Init(CommonAdcDevice);
	gpio_set_pin_function(BoardTypePin, GPIO_PIN_FUNCTION_B);

	const uint16_t reading = SimpleAnalogIn::ReadChannel(CommonAdcDevice, BoardTypeAdcChannel);

	boardTypeIndex = 0;
	while (boardTypeIndex < ARRAY_SIZE(BoardIdDecisionPoints) && reading > BoardIdDecisionPoints[boardTypeIndex])
	{
		++boardTypeIndex;
	}

	switch ((BoardId)boardTypeIndex)
	{
	case BoardId::tool1lc_v0:
		SetPinMode(OutPins_Tool1LC[0], OUTPUT_LOW);					// V0.6 tool boards don't have pulldown resistors on the outputs, so turn them off
		SetPinMode(OutPins_Tool1LC[1], OUTPUT_LOW);					// V0.6 tool boards don't have pulldown resistors on the outputs, so turn them off
		SetPinMode(OutPins_Tool1LC[2], OUTPUT_HIGH);				// this is intended for the hot end fan, so turn it on just as the tool board firmware does

		SetPinMode(ButtonPins_Tool1LC[0], INPUT_PULLUP);
		SetPinMode(ButtonPins_Tool1LC[1], INPUT_PULLUP);

		SetPinMode(GlobalTmc22xxEnablePin_Tool1LC, OUTPUT_HIGH);
		break;

	case BoardId::exp1xd_v0:
		SetPinMode(JumperPin_Exp1XD, INPUT_PULLUP);
		break;

	case BoardId::exp1hce_v0:
		//TODO the buttons are read via analog inputs
		break;
	}

#endif

#else
# error Unsupported processor
#endif

	for (unsigned int ledNumber = 0; ledNumber < NumLedPins; ++ledNumber)
	{
		SetPinMode(GetLedPin(ledNumber), (GetLedActiveHigh()) ? OUTPUT_LOW : OUTPUT_HIGH);
	}

#if SAME5x

	// Check whether address switches are set to zero. If so then reset and load new firmware
	const CanAddress switches = ReadBoardAddress();
	const bool doHardwareReset = (switches == 0);
	const CanAddress defaultAddress = (doHardwareReset) ? CanId::ExpansionBoardFirmwareUpdateAddress : switches;

#elif SAMC21

# ifdef SAMMYC21
	const CanAddress defaultAddress = CanId::SammyC21DefaultAddress;
	const bool doHardwareReset = !digitalRead(ButtonPins[0]);
# else

	bool doHardwareReset;
	CanAddress defaultAddress;

	switch ((BoardId)boardTypeIndex)
	{
	case BoardId::tool1lc_v0:
	default:
		// If both button pins are pressed, reset and load new firmware
		defaultAddress = CanId::ToolBoardDefaultAddress;
		doHardwareReset = !digitalRead(ButtonPins_Tool1LC[0]) && !digitalRead(ButtonPins_Tool1LC[1]);
		break;

	case BoardId::exp1xd_v0:
		defaultAddress = CanId::Exp1XDBoardDefaultAddress;
		doHardwareReset = !digitalRead(JumperPin_Exp1XD);
		break;

	case BoardId::exp1hce_v0:
		defaultAddress = CanId::Exp1HCEBoardDefaultAddress;
		{
			gpio_set_pin_function(ButtonsPin_Exp1HCE, GPIO_PIN_FUNCTION_B);
			const uint16_t reading = SimpleAnalogIn::ReadChannel(CommonAdcDevice, ButtonsAdcChannel_Exp1HCE);
			unsigned int buttonState = 0;
			while (buttonState < ARRAY_SIZE(ButtonsDecisionPoints) && reading > ButtonsDecisionPoints[buttonState])
			{
				++buttonState;
			}
			doHardwareReset = (buttonState == 0);
		}
		break;
	}

	SimpleAnalogIn::Disable(CommonAdcDevice);			// finished using the ADC

# endif
#endif


	if (!doHardwareReset && CheckValidFirmware())
	{
		// Relocate the vector table and jump into the firmware. If it returns then we execute the bootloader.
		StartFirmware();
	}

	// If we get here then we are staying in the bootloader
	// Initialise the CAN subsystem and systick.
	// We don't use the DMAC so no need to initialise that.
	// Serial port is initialised for diagnostic info.
	Serial::Init();
	if (!Flash::Init())
	{
		ReportErrorAndRestart("Failed to initialize flash controller", ErrorCode::flashInitFailed);
	}
	CanInterface::Init(defaultAddress, doHardwareReset);

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
				ReportErrorAndRestart("Failed to unlock flash", ErrorCode::unlockFailed);
			}
			SerialMessage("Erasing flash");
			if (!Flash::Erase(FirmwareFlashStart, roundedUpLength))
			{
				ReportErrorAndRestart("Failed to erase flash", ErrorCode::eraseFailed);
			}
		}
		if (!Flash::Write(FirmwareFlashStart + bufferStartOffset, FlashBlockSize, blockBuffer))
		{
			ReportErrorAndRestart("Failed to write flash", ErrorCode::writeFailed);
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
		ReportErrorAndRestart("Failed to lock flash", ErrorCode::lockFailed);
	}

	CanInterface::Disable();

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
			rslt |= 1 << i;
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
		ReportError("Invalid firmware", ErrorCode::invalidFirmware);
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
	ReportError(message.c_str(), ErrorCode::badCRC);
	return false;
}

// Execute the main firmware CAN1
[[noreturn]] void StartFirmware()
{
	SerialMessage("Bootloader transferring control to main firmware");
	delay(3);											// allow last 2 characters to go
	Serial::Disable();									// disable serial port so that main firmware can initialise it

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

// End
