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

#if SAME5x

#include <same51.h>

constexpr uint32_t FlashBlockWriteSize = 0x00010000;					// the block write size we use for flash (64K)
constexpr uint32_t FlashBlockEraseSize = 0x00010000;					// the block erase size we use for flash (64K)

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

# else

# endif

#elif SAMC21

constexpr uint32_t FlashBlockWriteSize = 0x00004000;							// the block write size we use for flash (16K)
constexpr uint32_t FlashBlockEraseSize = 0x00004000;							// the block erase size we use for flash (16K)

#elif SAME70

# define pvReservedM9	pfnReserved1_Handler
# define FLASH_ADDR		IFLASH_ADDR
# define FLASH_SIZE		IFLASH_SIZE
# define HSRAM_ADDR		IRAM_ADDR
# define HSRAM_SIZE		IRAM_SIZE

// We program the flash in 64kb blocks
constexpr uint32_t FlashBlockWriteSize = 0x00010000;
constexpr uint32_t FlashBlockEraseSize = 0x00020000;

#else
# error Unsupported board
#endif

constexpr uint32_t BlockReceiveTimeout = 2000;								// block receive timeout milliseconds

#if SAME70
constexpr uint32_t FirmwareFlashStart = FLASH_ADDR;							// no bootloader on SAME70
#elif SAME5x && defined(CAN_IAP)
constexpr uint32_t FirmwareFlashStart = FLASH_ADDR + 0x4000;				// the amount of space we reserve for the USB bootloader on the Duet 3 Mini
#else
constexpr uint32_t FirmwareFlashStart = FLASH_ADDR + FlashBlockWriteSize;	// the amount of space we reserve for the bootloader on the SAME5x-based expansion boards
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
bool CheckValidFirmware();
[[noreturn]] void StartFirmware();

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

[[noreturn]] void ReportErrorAndRestart(const char *text, FirmwareFlashErrorCode err)
{
	CanInterface::Shutdown();
	ReportError(text, err);
	delay(2000);
	ResetProcessor();
}

void RequestFirmwareBlock(uint32_t fileOffset, uint32_t numBytes, CanMessageBuffer& buf)
{
	CanMessageFirmwareUpdateRequest * const msg = buf.SetupRequestMessage<CanMessageFirmwareUpdateRequest>(0, CanInterface::GetCanAddress(), CanId::MasterAddress);
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

	CanMessageBuffer buf(nullptr);
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
	SysTick->LOAD = ((SystemCoreClockFreq/1000) - 1u) << SysTick_LOAD_RELOAD_Pos;
	SysTick->CTRL = (1u << SysTick_CTRL_ENABLE_Pos) | (1u << SysTick_CTRL_TICKINT_Pos) | (1 << SysTick_CTRL_CLKSOURCE_Pos);
	NVIC_SetPriority(SysTick_IRQn, (1UL << __NVIC_PRIO_BITS) - 1UL);	// set Priority for Systick Interrupt

#if 0				// we don't need to call CoreInit because we don't use DMA, EXINTs or the random number generator
	CoreInit();
#endif
	DeviceInit();

	// Establish the board type and initialise pins
	CanAddress defaultAddress;
	bool doHardwareReset;
	bool useAlternateCanPins;
	if (!IdentifyBoard(defaultAddress, doHardwareReset, useAlternateCanPins))
	{
		ReportErrorAndRestart("Unknown board", FirmwareFlashErrorCode::unknownBoard);
	}

	for (unsigned int ledNumber = 0; ledNumber < NumLedPins; ++ledNumber)
	{
		pinMode(GetLedPin(ledNumber), (GetLedActiveHigh()) ? OUTPUT_LOW : OUTPUT_HIGH);
	}

#ifdef DEBUG
	uart0.begin(57600);
# if defined(CAN_IAP)
	SerialMessage("CAN IAP running");
# else
	SerialMessage("Bootloader running");
# endif
#endif

#if !defined(CAN_IAP)
	if (!doHardwareReset && CheckValidFirmware())
	{
		// Relocate the vector table and jump into the firmware. If it returns then we execute the bootloader.
		StartFirmware();
	}
#endif

	// If we get here then we are staying in the bootloader
	// Initialise the CAN subsystem
	if (!Flash::Init())
	{
		ReportErrorAndRestart("Failed to initialize flash controller", FirmwareFlashErrorCode::flashInitFailed);
	}

	CanInterface::Init(defaultAddress, doHardwareReset, useAlternateCanPins);

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
			const uint32_t firmwareSize =
#if defined(CAN_IAP) && SAME5x
										fileSize/2;			// using UF2 format with 256 data bytes per 512b block
#else
										fileSize;			// using binary format
#endif
			roundedUpLength = ((firmwareSize + (FlashBlockEraseSize - 1))/FlashBlockEraseSize) * FlashBlockEraseSize;
			SerialMessage("Unlocking flash");
			if (!Flash::Unlock(FirmwareFlashStart, roundedUpLength))
			{
				ReportErrorAndRestart("Failed to unlock flash", FirmwareFlashErrorCode::unlockFailed);
			}

			SerialMessage("Erasing flash");
#if SAME70
			if (!EraseFlash(roundedUpLength))
#else
			if (!Flash::Erase(FirmwareFlashStart, roundedUpLength))
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
				const uint32_t firmwareOffset = FirmwareFlashStart + (bufferStartOffset/2) + (block * 256);
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
		if (!Flash::Write(FirmwareFlashStart + bufferStartOffset, FlashBlockWriteSize, reinterpret_cast<uint32_t*>(blockBuffer)))
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
	if (!Flash::Lock(FirmwareFlashStart, roundedUpLength))
	{
		ReportErrorAndRestart("Failed to lock flash", FirmwareFlashErrorCode::lockFailed);
	}

	CanInterface::Shutdown();

	delay(2);

#if defined(CAN_IAP)
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

	if (!CheckValidFirmware())
	{
		ResetProcessor();
	}

	StartFirmware();
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
	const uint32_t actualCRC = ComputeCRC32(reinterpret_cast<const uint32_t*>(FirmwareFlashStart), crcAddr);

	if (actualCRC == storedCRC)
	{
		return true;
	}

	String<100> message;
	message.printf("CRC error: stored %08" PRIx32 ", actual %" PRIx32, storedCRC, actualCRC);
	ReportError(message.c_str(), FirmwareFlashErrorCode::badCRC);
	return false;
}

// Execute the main firmware
[[noreturn]] void StartFirmware()
{
# ifdef DEBUG
	SerialMessage("Bootloader transferring control to main firmware");
	uart0.end();										// disable serial port so that main firmware can initialise it
# endif

	// Disable all IRQs
	SysTick->CTRL = (1 << SysTick_CTRL_CLKSOURCE_Pos);	// disable the system tick exception
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

	// Disable DPLL0 and DPLL1 so hat they can be reprogrammed by the main firmware
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
