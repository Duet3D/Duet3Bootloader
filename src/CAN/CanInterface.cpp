/*
 * Can.cpp
 *
 *  Created on: 17 Sep 2018
 *      Author: David
 */

#include "CanInterface.h"

#include <CanSettings.h>
#include <CanMessageFormats.h>
#include <CanMessageBuffer.h>

#define SUPPORT_CAN		1		// needed by CanDevice.h
#include <CanDevice.h>

#if SAME5x || SAMC21
# include <hpl_user_area.h>
#endif

static CanDevice *can0dev = nullptr;

#if !defined(CAN_IAP)

# if SAME5x
constexpr uint32_t CanUserAreaDataOffset = CanUserAreaDataOffset_SAME5x;
# elif SAMC21
constexpr uint32_t CanUserAreaDataOffset = CanUserAreaDataOffset_SAMC21;
# endif

static CanUserAreaData canConfigData;

#endif

static CanAddress boardAddress;

constexpr CanDevice::Config Can0Config =
{
	.dataSize = 64,									// must be one of: 8, 12, 16, 20, 24, 32, 48, 64
#if STM32H5											// STM32H5 has reduced CAN functionality
	.numTxBuffers = 0,
	.txFifoSize = 3,
	.numRxBuffers = 0,
	.rxFifo0Size = 3,
	.rxFifo1Size = 3,
	.numShortFilterElements = 0,
	.numExtendedFilterElements = 2,
	.txEventFifoSize = 3
#else
	.numTxBuffers = 2,
	.txFifoSize = 4,
	.numRxBuffers = 0,
	.rxFifo0Size = 16,
	.rxFifo1Size = 16,
	.numShortFilterElements = 0,
	.numExtendedFilterElements = 2,
	.txEventFifoSize = 2
#endif
};

static_assert(Can0Config.IsValid());

// CAN buffer memory must be in the first 64Kb of RAM (SAME5x) or in non-cached RAM (SAME70), so put it in its own segment
static uint32_t can0Memory[Can0Config.GetMemorySize()] __attribute__ ((section (".CanMessage")));

// Initialise the CAN interface
void CanInterface::Init(CanAddress defaultBoardAddress, bool doHardwareReset, const CanParameters& params)
{
#if !defined(CAN_IAP)
	// Read the CAN timing data from the top part of the NVM User Row
	canConfigData = *reinterpret_cast<CanUserAreaData*>(NVMCTRL_USER + CanUserAreaDataOffset);

	if (doHardwareReset)
	{
		canConfigData.Clear();
		_user_area_write(reinterpret_cast<void*>(NVMCTRL_USER), CanUserAreaDataOffset, reinterpret_cast<const uint8_t*>(&canConfigData), sizeof(canConfigData));
	}
#endif

	CanTiming timing;

#if defined(CAN_IAP)
	timing.SetDefaults(CanTiming::DefaultCanBitRate);
#else
	canConfigData.GetTiming(timing);
#endif

	// Set up the CAN pins
#if SAME70
	SetPinFunction(params.txPin, params.txPinFunction);
	SetPinFunction(params.rxPin, params.rxPinFunction);
#else
	SetPinFunction(params.txPin, params.pinsFunction);
	SetPinFunction(params.rxPin, params.pinsFunction);
#endif

	// Initialise the CAN hardware, using the timing data if it was valid
	can0dev = CanDevice::Init(0,
# if STM32
								params.instanceNumber - 1,				// STM numbers instances from 1, our driver numbers them from zero
# else
								params.instanceNumber,
# endif
								Can0Config, can0Memory, timing, nullptr);

#ifdef SAMMYC21
	SetPinMode(CanStandbyPin, OUTPUT_LOW);						// take the CAN drivers out of standby
#endif

#if defined(CAN_IAP)
	boardAddress = defaultBoardAddress;
#else
	boardAddress = canConfigData.GetCanAddress(defaultBoardAddress);
#endif

	// Set up a CAN receive filter to receive all messages addressed to us in FIFO 0
	can0dev->SetExtendedFilterElement(0, CanDevice::RxBufferNumber::fifo0,
										(uint32_t)boardAddress << CanId::DstAddressShift,
										CanId::BoardAddressMask << CanId::DstAddressShift);
	// Set up a CAN receive filter to receive clock messages
	can0dev->SetExtendedFilterElement(1, CanDevice::RxBufferNumber::fifo0,
										((uint32_t)CanId::BroadcastAddress << CanId::DstAddressShift) | ((uint32_t)CanMessageType::timeSync << CanId::MessageTypeShift),
										(CanId::BoardAddressMask << CanId::DstAddressShift) | (CanId::MessageTypeMask << CanId::MessageTypeShift));

	can0dev->Enable();
}

// Close down the CAN interface
void CanInterface::Shutdown()
{
	if (can0dev != nullptr)
	{
		can0dev->DeInit();
	}
}

CanAddress CanInterface::GetCanAddress()
{
	return boardAddress;
}

// Get a received CAN message if there is one
bool CanInterface::GetCanMessage(CanMessageBuffer *buf)
{
	return can0dev->ReceiveMessage(CanDevice::RxBufferNumber::fifo0, 0, buf);
}

// Send a CAN message and free the buffer
void CanInterface::Send(CanMessageBuffer *buf)
{
	(void)can0dev->SendMessage(CanDevice::TxBufferNumber::fifo, 1000, buf);
}

void CanInterface::GetLocalCanTiming(CanTiming& timing) noexcept
{
	can0dev->GetLocalCanTiming(timing);
}

void CanInterface::SetLocalCanTiming(const CanTiming& timing) noexcept
{
	can0dev->ChangeLocalCanTiming(timing);
}

#if !defined(CAN_IAP)

bool CanInterface::StoreLocalCanTiming(const CanTiming& timing) noexcept
{
	canConfigData.SetTiming(timing);
#if RP2040
	NonVolatileMemory mem(NvmPage::common);
	mem.SetCanSettings(canConfigData);
	mem.EnsureWritten();
	return true;
#elif SAMC21 || SAME5x
	return _user_area_write(reinterpret_cast<void*>(NVMCTRL_USER), CanUserAreaDataOffset, reinterpret_cast<const uint8_t*>(&canConfigData), sizeof(canConfigData)) == 0;
#endif
}

#endif

// End
