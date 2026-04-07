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

#if SAME70
# include <AsyncSerial.h>
#else
# include <Serial.h>
#endif

#include <Config/BoardDef.h>
#include <General/StringRef.h>
#include <CanId.h>
#include <CanMessageBuffer.h>
#include <Duet3Common.h>
#include "BoardType.h"

#ifdef DEBUG
# include <UART/UartParameters.h>
# include <AsyncSerial.h>

AsyncSerial *uart0;

#endif

#if SAME5x

#include <same51.h>

constexpr uint32_t FlashBlockWriteSize = 0x00004000;					// the block write size we use for flash (16K)
constexpr uint32_t FlashBlockEraseSize = 0x00004000;					// the block erase size we use for flash (16K)

# if defined(CAN_IAP)

struct UF2_Block
{
	// 32 byte header
	uint32_t magicStart0;
	uint32_t magicStart1;
	uint32_t flags;
	uint32_t targetAddr;
	uint32_t payloadSize;
	uint32_t blockNo;
	uint32_t numBlocks;
	uint32_t fileSize;		// or familyID
	uint32_t data[476/4];
	uint32_t magicEnd;

	static constexpr uint32_t MagicStart0Val = 0x0A324655;
	static constexpr uint32_t MagicStart1Val = 0x9E5D5157;
	static constexpr uint32_t MagicEndVal = 0x0AB16F30;
};

# ifdef DEBUG
constexpr UartParameters Serial0Params =
{
	.sercomNumber = 2,
	.rxPin = NoPin,
	.txPin = PortBPin(25),
	.pinFunction = GpioPinFunction::D,
	.dataInPad = 1,
	.dataOutPad = 0,
	.numRxSlots = 16,
	.numTxSlots = 64

};
# endif

#else

# ifdef DEBUG
constexpr UartParameters Serial0Params =
{
	.sercomNumber = 3,
	.rxPin = NoPin,
	.txPin = PortBPin(20),
	.pinFunction = GpioPinFunction::C,
	.dataInPad = 3,
	.dataOutPad = 0,
	.numRxSlots = 16,
	.numTxSlots = 64

};
# endif

# endif

#elif SAMC21

constexpr uint32_t FlashBlockWriteSize = 0x00004000;							// the block write size we use for flash (16K)
constexpr uint32_t FlashBlockEraseSize = 0x00004000;							// the block erase size we use for flash (16K)

# ifdef DEBUG
constexpr UartParameters Serial0Params =
{
	.sercomNumber = 4,
	.rxPin = NoPin,
	.txPin = PortAPin(12),
	.pinFunction = GpioPinFunction::D,
	.dataInPad = 3,
	.dataOutPad = 0,
	.numRxSlots = 16,
	.numTxSlots = 64

};
# endif

#elif SAME70

# define pvReservedM9	pfnReserved1_Handler
# define FLASH_ADDR		IFLASH_ADDR
# define FLASH_SIZE		IFLASH_SIZE
# define HSRAM_ADDR		IRAM_ADDR
# define HSRAM_SIZE		IRAM_SIZE

// We program the flash in 64kb blocks
constexpr uint32_t FlashBlockWriteSize = 0x00010000;
constexpr uint32_t FlashBlockEraseSize = 0x00020000;

# ifdef DEBUG
constexpr UartParameters Serial0Params =
{
	.uartOrUsartInstance = 2,
	.rxPin = NoPin,
	.txPin = PortDPin(26),
	.pinFunction = GpioPinFunction::C,
	.numRxSlots = 16,
	.numTxSlots = 64
};
# endif

#else
# error Unsupported board
#endif

constexpr uint32_t BlockReceiveTimeout = 2000;								// block receive timeout milliseconds

#if SAME70
constexpr uint32_t FirmwareFlashStart = FLASH_ADDR;							// no bootloader on SAME70
#elif SAMC21
constexpr uint32_t FirmwareFlashStart = 0x04000;							// 16K bootloader on SAMC21
#elif SAME5x && defined(CAN_IAP)
constexpr uint32_t FirmwareFlashStart = FLASH_ADDR + 0x4000;				// the amount of space we reserve for the USB bootloader on the Duet 3 Mini and for the CAN bootloader on the INDX
#elif SAME5x
constexpr uint32_t FirmwareFlashStart1 = FLASH_ADDR + 0x04000;				// the amount of space we reserve for the CAN bootloader on the SAME5x-based expansion boards except INDX
constexpr uint32_t FirmwareFlashStart2 = FLASH_ADDR + 0x10000;				// the amount of space we reserve for the CAN bootloader on the SAME5x-based expansion boards except INDX
#endif

#if SAME70

// Erase at least 'length' bytes of flash
// There are two 8K sectors, then one 112K sector, then the rest are 128K sectors. We can only erase whole sectors.
bool EraseFlash(uint32_t length) noexcept
{
	uint32_t offset = 0;
	while (offset < length)
	{
		const uint32_t next = (offset < 0x00004000) ? offset + 0x00002000	// we are in one of the 8K sectors
						: (offset < 0x00020000) ? 0x00020000				// we are in the 112k sector
							: offset + 0x00020000;							// we are in one of the 128k sectors
		if (!Flash::EraseSector(offset + IFLASH_ADDR))
		{
			return false;
		}
		offset = next;
	}

	return true;
}

#endif

alignas(4) static uint8_t blockBuffer[FlashBlockWriteSize];

#if !defined(CAN_IAP)

// Forward declarations
bool CheckValidFirmware(uint32_t startAddress, bool doReportError);
[[noreturn]] void StartFirmware(uint32_t startAddress);

#endif

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
#if defined(CAN_IAP)
	// The watchdog is enabled, so we need to reset that too
	WatchdogReset();
# if SAME70
	WatchdogResetSecondary();
# endif
#endif
}

void SerialMessage(const char *text)
{
#ifdef DEBUG
# ifdef SAMMYC21
	// Messages go to the USB port, so send them raw
	uart0->print(text);
	uart0->print("\n");
# else
	// Assume a PanelDue is connected, so encapsulate the message
	uart0->print("\n{\"message\":\"");
	uart0->print(text);									// should do json escaping here but for now just be careful what messages we send
	uart0->print("\"}\n");
# endif
	uart0->flush();
	delay(3);											// allow time for the last character to go
#endif
}

// Flash the red LED the specified number of times
void FlashLed(unsigned int numFlashes)
{
	for (unsigned int i = 0; i < numFlashes; ++i)
	{
		WriteLed(0, true);
		delay(200);
		WriteLed(0, false);
		delay(200);
	}
}

// Report an error via the serial port (if enabled) and by flashing the red LED
void ReportError(const char *text, FirmwareFlashErrorCode err)
{
	SerialMessage(text);
	FlashLed((unsigned int)err);
	delay(1000);
}

// Report an error and start from the beginning again
[[noreturn]] void ReportErrorAndRestart(const char *text, FirmwareFlashErrorCode err)
{
	CanInterface::Shutdown();
	ReportError(text, err);
	delay(2000);
	ResetProcessor();
}

// Make sure there are no received messages
void FlushCanMessages()
{
	CanMessageBuffer buf;
	while (CanInterface::GetCanMessage(&buf)) { }
}

void RequestFirmwareBlock(uint32_t fileOffset, uint32_t numBytes, CanMessageBuffer& buf)
{
	CanMessageFirmwareUpdateRequest * const msg = buf.SetupRequestMessageNoRid<CanMessageFirmwareUpdateRequest>(CanInterface::GetCanAddress(), CanId::MasterAddress);
	SafeStrncpy(msg->boardType, GetBoardTypeName(), sizeof(msg->boardType));
	msg->boardVersion = GetBoardVersion();
	msg->bootloaderVersion = CanMessageFirmwareUpdateRequest::BootloaderVersion0;
#if defined(CAN_IAP) && SAME5x
	msg->uf2Format = true;											// firmware files for Duet 3 Mini are shipped in .uf2 format
#else
	msg->uf2Format = false;
#endif
	msg->fileWanted = (uint32_t)FirmwareModule::main;
	msg->fileOffset = fileOffset;
	msg->lengthRequested = numBytes;
	buf.dataLength = msg->GetActualDataLength();
	CanInterface::Send(&buf);
}

// Get a buffer of data from the host
void GetBlock(uint32_t startingOffset, uint32_t& fileSize)
{
	constexpr Pin CanLedNumber = (NumLedPins >= 2) ? 1 : 0;
	WriteLed(CanLedNumber, true);
	delay(25);														// flash the LED briefly to indicate we are requesting a new flash block
	WriteLed(CanLedNumber, false);

	FlushCanMessages();												// flush the receive buffer in case it's full of time sync messages

	CanMessageBuffer buf;
	RequestFirmwareBlock(startingOffset, FlashBlockWriteSize, buf);	// ask for 16K or 64K from the starting offset

	uint32_t whenStartedWaiting = millis();
	uint32_t bytesReceived = 0;
	bool done = false;
	do
	{
		const bool ok = CanInterface::GetCanMessage(&buf);
		if (ok)
		{
			if (buf.id.MsgType() == CanMessageType::firmwareBlockResponse)
			{
				const CanMessageFirmwareUpdateResponse& response = buf.msg.firmwareUpdateResponse;
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
						if (bytesReceived == FlashBlockWriteSize || bytesReceived >= response.fileLength - startingOffset)
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
			RequestFirmwareBlock(startingOffset + bytesReceived, FlashBlockWriteSize - bytesReceived, buf);		// ask for 16K or 64K from the starting offset
			whenStartedWaiting = millis();
		}
	} while (!done);
}

bool LookForClockMessages() noexcept
{
	FlushCanMessages();												// make sure there are no old messages in the buffer

	constexpr uint32_t millsecondsAllowed = 2500;					// how long we allow to receive three time sync messages (we should receive about four every second)
	const uint32_t whenStartedWaiting = millis();
	FlashLed(1);													// flash the LED once to indicate that we are trying a new bit rate
	unsigned int numTimeSyncMessagesReceived = 0;
	do
	{
		CanMessageBuffer buf;
		const bool ok = CanInterface::GetCanMessage(&buf);
		if (ok && buf.id.MsgType() == CanMessageType::timeSync)
		{
			++numTimeSyncMessagesReceived;
			if (numTimeSyncMessagesReceived == 3) { return true; }
		}
	} while (millis() - whenStartedWaiting < millsecondsAllowed);
	return false;
}

// Set the bit rate and listed for CAN time sync messages. Return true if we heard enough of them,.
bool TryBitRate(uint32_t bitRate)
{
	CanTiming timing;
	timing.SetDefaults(bitRate);
	CanInterface::SetLocalCanTiming(timing);
	if (LookForClockMessages())
	{
#if !defined(CAN_IAP)
		// If we get here then we've seen a time sync message that is probably at a bit rate different from the original
		(void)CanInterface::StoreLocalCanTiming(timing);			// store the new timing in NVRAM. CAUTION: this allocates a 512-byte buffer on the stack!
		delay(100);													// see if a delay at this point helps
#endif
		return true;
	}
	return false;
}

// Establish the bit rate in use.
// On entry the standard bit rate as read from flash memory has been set.
// On return we are using the same bit rate as the master.
void FindBitRate()
{
#if !defined(CAN_IAP)
	// Try the bit rate stored in NVM first
	if (LookForClockMessages()) { return; }
#endif

	// Try the standard bit rates
	if (TryBitRate(CanTiming::DefaultCanBitRate)) { return; }
	if (TryBitRate(CanTiming::DefaultCanBitRate/2)) { return; }
	if (TryBitRate(CanTiming::DefaultCanBitRate/4)) { return; }

	ReportErrorAndRestart("No time sync message seen", FirmwareFlashErrorCode::noTimeSyncMessageSeen);
}

// Request data from the master and program the flash memory. Return the address at which the firmware was loaded.
uint32_t ProgramFlash()
{
	if (!Flash::Init())
	{
		ReportErrorAndRestart("Failed to initialize flash controller", FirmwareFlashErrorCode::flashInitFailed);
	}

	// Loop requesting firmware from the main board and handling any firmware that it sends to us
	uint32_t bufferStartOffset = 0;
	uint32_t roundedUpLength;
#if SAME5x && !defined(CAN_IAP)
	uint32_t startAddress;												// we have two possible start addresses on the SAME5x
#else
	constexpr uint32_t startAddress = FirmwareFlashStart;
#endif

	for (;;)
	{
		uint32_t fileSize;
		GetBlock(bufferStartOffset, fileSize);
		if (bufferStartOffset == 0)
		{
#if SAME5x && !defined(CAN_IAP)
			// Find the address of the CRC at the end of the file, and compare it with the file size
			static_assert(FirmwareFlashStart1 < FirmwareFlashStart2);	// the following code assumes this
			const uint32_t crcAddr = reinterpret_cast<const uint32_t*>(blockBuffer)[7];
			if (crcAddr - FirmwareFlashStart1 < fileSize)				// should have crcAddr - FirmwareFlashStart1 == fileSize - 4 if the firmware starts at this address
			{
				startAddress = FirmwareFlashStart1;
			}
			else
			{
				startAddress = FirmwareFlashStart2;
			}
#endif
			// First block received, so unlock and erase the firmware
			const uint32_t firmwareSize =
#if defined(CAN_IAP) && SAME5x
										fileSize/2;			// using UF2 format with 256 data bytes per 512b block
#else
										fileSize;			// using binary format
#endif
			roundedUpLength = ((firmwareSize + (FlashBlockEraseSize - 1))/FlashBlockEraseSize) * FlashBlockEraseSize;
			SerialMessage("Unlocking flash");
			if (!Flash::Unlock(startAddress, roundedUpLength))
			{
				ReportErrorAndRestart("Failed to unlock flash", FirmwareFlashErrorCode::unlockFailed);
			}

			SerialMessage("Erasing flash");
#if SAME70
			if (!EraseFlash(roundedUpLength))
#else
			if (!Flash::Erase(startAddress, roundedUpLength))
#endif
			{
				ReportErrorAndRestart("Failed to erase flash", FirmwareFlashErrorCode::eraseFailed);
			}
		}

		// If we have both red and green LEDs, the green one indicates CAN activity. Use the red one to indicate writing to flash.
		if (NumLedPins == 2)
		{
			WriteLed(0, true);
		}

		SerialMessage("Writing flash");
#if defined(CAN_IAP) && SAME5x
		// The file being fetched is in .uf2 format, so extract the data from the buffer and write it
		// On the SAME5x we fetch 64kb at a time, so we have up to 128 blocks in the buffer
		for (unsigned int block = 0; block < FlashBlockWriteSize/512 && bufferStartOffset + (512 * (block + 1)) <= fileSize; ++block)
		{
			const UF2_Block *const currentBlock = reinterpret_cast<const UF2_Block*>(blockBuffer + (512 * block));
			if (   currentBlock->magicStart0 == UF2_Block::MagicStart0Val
				&& currentBlock->magicStart1 == UF2_Block::MagicStart1Val
				&& currentBlock->magicEnd == UF2_Block::MagicEndVal
				&& currentBlock->payloadSize <= 256
			   )
			{
				const uint32_t firmwareOffset = startAddress + (bufferStartOffset/2) + (block * 256);
				if (!Flash::Write(firmwareOffset, 256, currentBlock->data))
				{
					ReportErrorAndRestart("Failed to write flash", FirmwareFlashErrorCode::writeFailed);
				}
			}
			else
			{
				ReportErrorAndRestart("bad UF2 file", FirmwareFlashErrorCode::invalidFirmware);
			}
		}
#else
		// The file being fetched is in binary format, so we can write it directly
		if (!Flash::Write(startAddress + bufferStartOffset, FlashBlockWriteSize, reinterpret_cast<uint32_t*>(blockBuffer)))
		{
			ReportErrorAndRestart("Failed to write flash", FirmwareFlashErrorCode::writeFailed);
		}
#endif
		if (NumLedPins == 2)
		{
			WriteLed(0, false);
		}

		bufferStartOffset += FlashBlockWriteSize;
		if (bufferStartOffset >= fileSize)
		{
			break;
		}
	}

	// If we get here, firmware update is complete
	SerialMessage("Locking flash");
	if (!Flash::Lock(startAddress, roundedUpLength))
	{
		ReportErrorAndRestart("Failed to lock flash", FirmwareFlashErrorCode::lockFailed);
	}
	Flash::Deinit();
	return startAddress;
}

// Clock configuration:
// SAME5x:
//	XOSCn (n=0 on expansion boards, 1 on Duet 3 Mini) = 12MHz or 25MHz crystal oscillator
//  DPLL0 120MHz locked to XOSCn
//  DPLL1 96MHz locked to XOSCn
//  DFLL48M no longer used because it has high jitter
//  GCLK0 120MHz from DPLL0, for CPU and fast peripherals
//  GCLK1 XOSCn divided by (32 * XOSCn_frequency_MHz) to give 31250Hz for SERCOM slow clock
//  GCLK2 XOSCn direct, used by Ethernet PHY on Duet 3 Mini
//  GCLK3: DPLL0 divided by 2, 60MHz for peripherals that need slower than 120MHz
//  GCLK4: DPLL1 divided by 2, 48MHz for CAN and step timer
//  GCLK5: For use by the application, e.g. TMC clock on EXP1HCL/M23CL, LDC1612 clock on TOOL1RR and SZP
//  GCLK6: DPLL0 divided by 120 to give 1MHz, for EIC deglitching
//  GCLK7: DPLL1 direct to give 96MHz for SDHC interface on Duet 3 Mini
// SAMC21:
//	XOSC1 12MHz or 25MHz crystal oscillator (16MHz on Sammy-C21 board)
//	FDPLL 48MHz locked to XOSC1
//	GCLK0 48MHz from FDPLL, used by CPU, CAN and most peripherals
//  GCLK1 31250Hz (1MHz divided by 32) for e.g. SERCOM slow clock
//  GCLK2 1MHz
void AppMain()
{
	// Initialise systick (needed for delay calls to work)
	SysTick->LOAD = ((SystemCoreClockFreq/1000) - 1u) << SysTick_LOAD_RELOAD_Pos;
	SysTick->CTRL = (1u << SysTick_CTRL_ENABLE_Pos) | (1u << SysTick_CTRL_TICKINT_Pos) | (1 << SysTick_CTRL_CLKSOURCE_Pos);
	NVIC_SetPriority(SysTick_IRQn, (1UL << __NVIC_PRIO_BITS) - 1UL);	// set priority for Systick Interrupt

#ifdef DEBUG				// we don't need to call CoreInit in non-debug builds because we don't use DMA, EXINTs or the random number generator
	CoreInit();
#endif
	DeviceInit();

#ifdef DEBUG
	uart0 = new AsyncSerial(Serial0Params);
#endif

	// Establish the board type and initialise pins
	CanAddress defaultAddress;
	bool doHardwareReset;
	unsigned int whichCanPort;
	bool useLaterCanPins;
	if (!IdentifyBoard(defaultAddress, doHardwareReset, whichCanPort, useLaterCanPins))
	{
		ReportErrorAndRestart("Unknown board", FirmwareFlashErrorCode::unknownBoard);
	}

	for (unsigned int ledNumber = 0; ledNumber < NumLedPins; ++ledNumber)
	{
		SetPinMode(GetLedPin(ledNumber), (GetLedActiveHigh()) ? OUTPUT_LOW : OUTPUT_HIGH);
	}

#ifdef DEBUG
	uart0->begin(57600);
# if defined(CAN_IAP)
	SerialMessage("CAN IAP running");
# else
	SerialMessage("Bootloader running");
# endif
#endif

#if !defined(CAN_IAP)
	if (!doHardwareReset)
	{
# if SAME5x
		if (CheckValidFirmware(FirmwareFlashStart1, false))
		{
			StartFirmware(FirmwareFlashStart1);
		}
		if (CheckValidFirmware(FirmwareFlashStart2, true))
		{
			StartFirmware(FirmwareFlashStart2);
		}
# else
		if (CheckValidFirmware(FirmwareFlashStart, true))
		{
			// Relocate the vector table and jump into the firmware. If it returns then we execute the bootloader.
			StartFirmware(FirmwareFlashStart);
		}
# endif
	}
#endif

	// If we get here then we are staying in the bootloader
	CanInterface::Init(defaultAddress, doHardwareReset, whichCanPort, useLaterCanPins);		// initialise CAN subsystem
	FindBitRate();																			// establish the bit rate by listening for clock messages at the standard speeds
	const uint32_t startAddress = ProgramFlash();											// fetch and the firmware file and program it into flash, return the start address
	CanInterface::Shutdown();

	delay(2);

#if defined(CAN_IAP)
	(void)startAddress;
	SerialMessage("Finished firmware update");
	delay(1000);

	ResetProcessor();
#else
	NVIC_DisableIRQ(CAN0_IRQn);
	NVIC_DisableIRQ(CAN1_IRQn);
	CAN0->IR.reg = 0xFFFFFFFF;			// clear all interrupt sources for when the device gets enabled by the main firmware
	CAN0->ILE.reg = 0;
	CAN1->IR.reg = 0xFFFFFFFF;			// clear all interrupt sources for when the device gets enabled by the main firmware
	CAN1->ILE.reg = 0;

	SerialMessage("Finished firmware update");
	delay(1000);

	if (!CheckValidFirmware(startAddress, true))
	{
		ResetProcessor();
	}

	StartFirmware(startAddress);
#endif
}

#if !defined(CAN_IAP)

// Compute the CRC32 of a dword-aligned block of memory
// This assumes the caller has exclusive use of the DMAC
uint32_t ComputeCRC32(const uint32_t *start, const uint32_t *end)
{
# if SAME5x
	DMAC->CRCCTRL.reg = DMAC_CRCCTRL_CRCBEATSIZE_WORD | DMAC_CRCCTRL_CRCSRC_DISABLE | DMAC_CRCCTRL_CRCPOLY_CRC32;	// disable the CRC unit
# elif SAMC21
	DMAC->CTRL.bit.CRCENABLE = 0;
# else
#  error Unsupported processor
# endif
	DMAC->CRCCHKSUM.reg = 0xFFFFFFFF;
	DMAC->CRCCTRL.reg = DMAC_CRCCTRL_CRCBEATSIZE_WORD | DMAC_CRCCTRL_CRCSRC_IO | DMAC_CRCCTRL_CRCPOLY_CRC32;
# if SAMC21
	DMAC->CTRL.bit.CRCENABLE = 1;
# endif
	while (start < end)
	{
		DMAC->CRCDATAIN.reg = *start++;
		asm volatile("nop");
		asm volatile("nop");
	}

	DMAC->CRCSTATUS.reg = DMAC_CRCSTATUS_CRCBUSY;
	asm volatile("nop");
	return DMAC->CRCCHKSUM.reg;
}

// Check whether valid firmware is installed at the specified address. If not, report the error if asked to, then return false.
bool CheckValidFirmware(uint32_t startAddress, bool doReportError)
{
	const DeviceVectors * const vectors = reinterpret_cast<const DeviceVectors*>(startAddress);
	if (   reinterpret_cast<uint32_t>(vectors->pfnReset_Handler) < startAddress
		|| reinterpret_cast<uint32_t>(vectors->pfnReset_Handler) >= FLASH_ADDR + FLASH_SIZE
		|| reinterpret_cast<uint32_t>(vectors->pvStack) < HSRAM_ADDR
		|| reinterpret_cast<uint32_t>(vectors->pvStack) > HSRAM_ADDR + HSRAM_SIZE
		|| reinterpret_cast<uint32_t>(vectors->pvReservedM9) < startAddress
		|| reinterpret_cast<uint32_t>(vectors->pvReservedM9) > startAddress + FLASH_SIZE - 4
	   )
	{
		if (doReportError)
		{
			ReportError("Invalid firmware", FirmwareFlashErrorCode::invalidFirmware);
		}
		return false;
	}

	// Fetch the CRC-32 from the file
	const uint32_t *crcAddr = (const uint32_t*)(vectors->pvReservedM9);
	const uint32_t storedCRC = *crcAddr;

	// Compute the CRC-32 of the firmware
	const uint32_t actualCRC = ComputeCRC32(reinterpret_cast<const uint32_t*>(startAddress), crcAddr);

	if (actualCRC == storedCRC)
	{
		return true;
	}

	if (doReportError)
	{
		// Don't use printf/sprintf etc. here, it makes the build too large
		ReportError("CRC error", FirmwareFlashErrorCode::badCRC);
	}
	return false;
}

// Execute the main firmware
[[noreturn]] void StartFirmware(uint32_t startAddress)
{
	// Turn all LEDs off. Must be done before resetting the clocks, otherwise it doesn't work.
	for (size_t i = 0; i < NumLedPins; ++i)
	{
		WriteLed(i, false);
	}

# ifdef DEBUG
	SerialMessage("Bootloader transferring control to main firmware");
	uart0->end();										// disable serial port so that main firmware can initialise it
# endif

	// Disable all IRQs
	SysTick->CTRL = (1u << SysTick_CTRL_CLKSOURCE_Pos);	// disable the system tick exception
	__disable_irq();

# if SAME5x

	for (size_t i = 0; i < 8; i++)
	{
		NVIC->ICER[i] = 0xFFFFFFFF;						// Disable IRQs
		NVIC->ICPR[i] = 0xFFFFFFFF;						// Clear pending IRQs
	}

	// Reset the generic clock generator. This sets all clock generators to default values and the CPU clock to the 48MHz DFLL output.
	GCLK->CTRLA.reg = GCLK_CTRLA_SWRST;
	while ((GCLK->CTRLA.reg & GCLK_CTRLA_SWRST) != 0) { }

	// Disable DPLL0 and DPLL1 so that they can be reprogrammed by the main firmware
	OSCCTRL->Dpll[0].DPLLCTRLA.bit.ENABLE = 0;
	while (OSCCTRL->Dpll[0].DPLLSYNCBUSY.bit.ENABLE) { }
	OSCCTRL->Dpll[1].DPLLCTRLA.bit.ENABLE = 0;
	while (OSCCTRL->Dpll[1].DPLLSYNCBUSY.bit.ENABLE) { }

# elif SAMC21

	NVIC->ICER[0] = 0xFFFFFFFF;							// Disable IRQs
	NVIC->ICPR[0] = 0xFFFFFFFF;							// Clear pending IRQs

	// Switch back to the OSC48M clock divided to 4MHz
#  if 1
	// 2020-06-03: on the SammyC21 board the software reset of GCLK never completed, so reset it manually
	OSCCTRL->OSC48MCTRL.reg = OSCCTRL_OSC48MCTRL_ENABLE;				// make sure OSC48M is enabled, clear the on-demand bit
	while ((OSCCTRL->STATUS.reg & OSCCTRL_STATUS_OSC48MRDY) == 0) { }	// wait for it to become ready
	GCLK->GENCTRL[0].reg = 0x00000106;									// this is the reset default
	OSCCTRL->OSC48MCTRL.reg = OSCCTRL_OSC48MCTRL_ENABLE | OSCCTRL_OSC48MCTRL_ONDEMAND;		// back to reset default
#  else
	// The following code works on Duet3D boards, but it hangs on the SammyC21 with device ID 0x11010405 (SAMC21G18A die revision E)
	GCLK->CTRLA.reg = GCLK_CTRLA_SWRST;
	while ((GCLK->CTRLA.reg & GCLK_CTRLA_SWRST) != 0) { }
#  endif

	// Disable the DPLL so that it can be reprogrammed by the main firmware
	OSCCTRL->DPLLCTRLA.bit.ENABLE = 0;
	while (OSCCTRL->DPLLSYNCBUSY.bit.ENABLE) { }
# else
#  error Unsupported processor
# endif

//	hri_wdt_write_CLEAR_reg(WDT, WDT_CLEAR_CLEAR_KEY);	// reset the watchdog timer

	// Modify vector table location
	__DSB();
	__ISB();
	SCB->VTOR = startAddress & SCB_VTOR_TBLOFF_Msk;
	__DSB();
	__ISB();

	__asm volatile ("mov r3, %0" : : "r" (startAddress) : "r3");

	__asm volatile ("ldr r1, [r3]");
	__asm volatile ("msr msp, r1");
	__asm volatile ("mov sp, r1");

	__asm volatile ("isb");
	__enable_irq();

	__asm volatile ("ldr r1, [r3, #4]");
# if SAME5x
	__asm volatile ("orr r1, r1, #1");
# elif SAMC21
	__asm volatile ("movs r2, #1");
	__asm volatile ("orr r1, r1, r2");
# else
#  error Unsupported processor
# endif
	__asm volatile ("bx r1");

	// This point is unreachable, but gcc doesn't seem to know that
	for (;;) { }
}

#endif

// Function needed by CoreNG
[[noreturn]] void OutOfMemoryHandler()
{
	ReportErrorAndRestart("Out of memory", FirmwareFlashErrorCode::noMemory);
}

#if SAME70

// Dummy assertion handler, called by the Cache module in CoreN2G
extern "C" [[noreturn]] void vAssertCalled(uint32_t line, const char *file) noexcept
{
	ReportErrorAndRestart("vAssert called", FirmwareFlashErrorCode::vAssertCalled);
}

#endif

// End
