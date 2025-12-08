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

#if !SAME70
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
	.numTxBuffers = 2,
	.txFifoSize = 4,
	.numRxBuffers = 0,
	.rxFifo0Size = 16,
	.rxFifo1Size = 16,
	.numShortFilterElements = 0,
	.numExtendedFilterElements = 2,
	.txEventFifoSize = 2
};

static_assert(Can0Config.IsValid());

// CAN buffer memory must be in the first 64Kb of RAM (SAME5x) or in non-cached RAM (SAME70), so put it in its own segment
static uint32_t can0Memory[Can0Config.GetMemorySize()] __attribute__ ((section (".CanMessage")));

// Initialise the CAN interface
void CanInterface::Init(CanAddress defaultBoardAddress, bool doHardwareReset, bool useAlternatePins)
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
#if SAME5x
# if defined(CAN_IAP)
	// Duet 3 Mini uses PB14 and PB15, CAN 1
	SetPinFunction(PortBPin(15), GpioPinFunction::H);
	SetPinFunction(PortBPin(14), GpioPinFunction::H);
	constexpr unsigned int whichPort = 1;
# else
	unsigned int whichPort;
	if (useAlternatePins)
	{
		SetPinFunction(PortAPin(23), GpioPinFunction::I);
		SetPinFunction(PortAPin(22), GpioPinFunction::I);
		whichPort = 0;											// use CAN0
	}
	else
	{
		SetPinFunction(PortBPin(13), GpioPinFunction::H);
		SetPinFunction(PortBPin(12), GpioPinFunction::H);
		whichPort = 1;											// use CAN1
	}
# endif
#elif SAMC21
	if (useAlternatePins)
	{
		SetPinFunction(PortBPin(23), GpioPinFunction::G);
		SetPinFunction(PortBPin(22), GpioPinFunction::G);
	}
	else
	{
		SetPinFunction(PortAPin(25), GpioPinFunction::G);
		SetPinFunction(PortAPin(24), GpioPinFunction::G);
	}
	constexpr unsigned int whichPort = 0;						// we always use CAN0 on the SAMC21
#elif SAME70
	constexpr unsigned int whichPort = 1;						// we always use MCAN1 for Can-FD on the SAME70
	SetPinFunction(PortDPin(12), GpioPinFunction::B);
	SetPinFunction(PortCPin(12), GpioPinFunction::C);
#endif

	// Initialise the CAN hardware, using the timing data if it was valid
	can0dev = CanDevice::Init(0, whichPort, Can0Config, can0Memory, timing, nullptr);

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
	can0dev->SetLocalCanTiming(timing);
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
