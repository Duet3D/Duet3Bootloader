/*
 * Main.cpp
 *
 *  Created on: 3 Aug 2019
 *      Author: David
 */

#include "RepRapFirmware.h"
#include "CAN/CanInterface.h"
#include <Config/peripheral_clk_config.h>
#include "Hardware/Io.h"
#include "Hardware/Flash.h"
#include "Hardware/Serial.h"
#include "Config/BoardDef.h"
#include <General/StringRef.h>
#include <CanId.h>
#include <CanMessageBuffer.h>
#include <core_cm4.h>

constexpr uint32_t FlashBlockSize = 0x00010000;							// the block size we assume for flash
constexpr uint32_t BlockReceiveTimeout = 2000;							// block receive timeout milliseconds
const uint32_t FirmwareFlashStart = FLASH_ADDR + FlashBlockSize;		// we reserve 64K for the bootloader
constexpr const char* BoardTypeName = "DUET3EXP3HC";

// Error codes, presented as a number of flashes of the DIAG LED
enum class ErrorCode : unsigned int
{
	invalidFirmware = 2,
	blockReceiveTimeout = 3,
	noFile = 4,
	badOffset = 5,
	hostOther = 6,
	noBuffer = 7,
	flashInitFailed = 8,
	unlockFailed = 9,
	eraseFailed = 10,
	writeFailed = 11,
	lockFailed = 12
};

static CanAddress boardAddress;
static uint8_t blockBuffer[FlashBlockSize];

uint8_t ReadBoardId();
uint8_t ReadBoardType();

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

// System tick ISR, used for timing functions
extern "C" void SysTick_Handler()
{
	++tickCount;
}

void SerialMessage(const char *text)
{
	Serial::Send("\n{\"message\":\"");
	Serial::Send(text);									// should do json escaping here but for now just be careful what messages we send
	Serial::Send("\"}\n");
	delay(3);											// allow time for the last character to go
}

void ReportError(const char *text, ErrorCode err)
{
	SerialMessage(text);

	for (unsigned int i = 0; i < (unsigned int)err; ++i)
	{
		digitalWrite(DiagLedPin, true);
		delay(200);
		digitalWrite(DiagLedPin, false);
		delay(200);
	}

	delay(1000);
}

[[noreturn]] void ReportErrorAndRestart(const char *text, ErrorCode err)
{
	ReportError(text, err);
	SCB->AIRCR = (0x5FA << 16) | (1u << 2);				// reset the processor
	for (;;) { }
}

void RequestFirmwareBlock(uint32_t fileOffset, uint32_t numBytes)
{
	CanMessageBuffer *buf = CanMessageBuffer::Allocate();
	if (buf == nullptr)
	{
		ReportErrorAndRestart("No buffers", ErrorCode::noBuffer);
	}
	CanMessageFirmwareUpdateRequest *msg = buf->SetupRequestMessage<CanMessageFirmwareUpdateRequest>(boardAddress, 0);
	SafeStrncpy(msg->boardType, BoardTypeName, sizeof(msg->boardType));
	msg->boardVersion = ReadBoardType();
	msg->bootloaderVersion = CanMessageFirmwareUpdateRequest::BootloaderVersion0;
	msg->fileOffset = fileOffset;
	msg->lengthRequested = numBytes;
	buf->dataLength = msg->GetActualDataLength();
	CanInterface::Send(buf);
}

// Get a buffer of data from the host
void GetBlock(uint32_t startingOffset, uint32_t& fileSize)
{
	RequestFirmwareBlock(startingOffset, FlashBlockSize);			// ask for 64K from the starting offset

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
			if (buf->id.MsgType() == CanMessageType::FirmwareBlockResponse)
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

extern "C" int main()
{
	atmel_start_init();
	SystemCoreClock = CONF_CPU_FREQUENCY;
	SystemPeripheralClock = SystemCoreClock/2;

	SetPinMode(DiagLedPin, OUTPUT_LOW);
	for (Pin p : BoardAddressPins)
	{
		SetPinMode(p, INPUT_PULLUP);
	}

	// Initialise systick and serial, needed if we detect that the firmware is invalid
	SysTick->LOAD = ((SystemCoreClock/1000) - 1) << SysTick_LOAD_RELOAD_Pos;
	SysTick->CTRL = (1 << SysTick_CTRL_ENABLE_Pos) | (1 << SysTick_CTRL_TICKINT_Pos) | (1 << SysTick_CTRL_CLKSOURCE_Pos);
	Serial::Init();

	// Check whether address switches are set to zero. If so then we stay in the bootloader
	boardAddress = ReadBoardId();
	if (boardAddress != 0)
	{
		// Check whether there is valid firmware installed, if not then stay in the bootloader
		if (CheckValidFirmware())
		{
			// Relocate the vector table and jump into the firmware. If it returns then we execute the bootloader.
			StartFirmware();
		}
		else
		{
			ReportError("Invalid firmware", ErrorCode::invalidFirmware);
		}
	}

	// If we get here then we are staying in the bootloader
	// Initialise the CAN subsystem and systick.
	// We don't use the DMAC so no need to initialise that.
	// Serial port is initialised for diagnostic info.
	DeviceInit();
	Serial::Init();
	if (!Flash::Init())
	{
		ReportErrorAndRestart("Failed to initialize flash controller", ErrorCode::flashInitFailed);
	}
	CanInterface::Init((boardAddress == 0) ? CanId::FirmwareUpdateAddress : boardAddress);

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

	can_async_disable(&CAN_0);			// disable CAN to prevent it receiving packets into RAM
	delay(2);
	CAN1->IR.reg = 0xFFFFFFFF;			// clear all interrupt sources for when the device gets enabled by the main firmware

	SerialMessage("Finished firmware update");
	delay(1000);

	if (!CheckValidFirmware())
	{
		ReportErrorAndRestart("Invalid firmware", ErrorCode::invalidFirmware);
	}

	StartFirmware();
}

// Read the board address
uint8_t ReadBoardId()
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

uint8_t ReadBoardType()
{
	//TODO
	return 0;
}

// Check that valid firmware is installed
bool CheckValidFirmware()
{
	const DeviceVectors * const vectors = reinterpret_cast<const DeviceVectors*>(FirmwareFlashStart);
	if (   reinterpret_cast<uint32_t>(vectors->pfnReset_Handler) < FirmwareFlashStart
		|| reinterpret_cast<uint32_t>(vectors->pfnReset_Handler) >= FLASH_ADDR + FLASH_SIZE
		|| reinterpret_cast<uint32_t>(vectors->pvStack) < HSRAM_ADDR
		|| reinterpret_cast<uint32_t>(vectors->pvStack) > HSRAM_ADDR + HSRAM_SIZE
	   )
	{
		return false;
	}
	//TODO CRC
	return true;
}

// Execute the main firmware
[[noreturn]] void StartFirmware()
{
	SerialMessage("Bootloader transferring control to main firmware");
	delay(3);											// allow last 2 characters to go
	Serial::Disable();									// disable serial port so that main firmware can initialise it

	// Disable all IRQs
	SysTick->CTRL = (1 << SysTick_CTRL_CLKSOURCE_Pos);	// disable the system tick exception
	__disable_irq();
	for (size_t i = 0; i < 8; i++)
	{
		NVIC->ICER[i] = 0xFFFFFFFF;						// Disable IRQs
		NVIC->ICPR[i] = 0xFFFFFFFF;						// Clear pending IRQs
	}

	digitalWrite(DiagLedPin, false);					// turn the DIAG LED off

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
	__asm volatile ("orr r1, r1, #1");
	__asm volatile ("bx r1");

	// This point is unreachable, but gcc doesn't seem to know that
	for (;;) { }
}

// End
