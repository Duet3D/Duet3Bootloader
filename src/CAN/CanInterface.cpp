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
#include <hpl_user_area.h>

static CanDevice *can0dev = nullptr;
static CanUserAreaData canConfigData;
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
	.numExtendedFilterElements = 3,
	.txEventFifoSize = 2
};

static_assert(Can0Config.IsValid());

// CAN buffer memory must be in the first 64Kb of RAM (SAME5x) or in non-cached RAM (SAME70), so put it in its own segment
static uint32_t can0Memory[Can0Config.GetMemorySize()] __attribute__ ((section (".CanMessage")));

// Initialise the CAN interface
void CanInterface::Init(CanAddress defaultBoardAddress, bool doHardwareReset, bool useAlternatePins)
{
	// Read the CAN timing data from the top part of the NVM User Row
#if SAME5x
	const uint32_t CanUserAreaDataOffset = 512 - sizeof(CanUserAreaData);
#elif SAMC21
	const uint32_t CanUserAreaDataOffset = 256 - sizeof(CanUserAreaData);
#endif

	canConfigData = *reinterpret_cast<CanUserAreaData*>(NVMCTRL_USER + CanUserAreaDataOffset);

	if (doHardwareReset)
	{
		canConfigData.Clear();
		_user_area_write(reinterpret_cast<void*>(NVMCTRL_USER), CanUserAreaDataOffset, reinterpret_cast<const uint8_t*>(&canConfigData), sizeof(canConfigData));
	}
	CanTiming timing;
	canConfigData.GetTiming(timing);

	// Set up the CAN pins
#if SAME5x
	// We don't support alternate pins for the SAME5x yet
	SetPinFunction(PortBPin(13), GpioPinFunction::H);
	SetPinFunction(PortBPin(12), GpioPinFunction::H);
	const unsigned int whichPort = 1;							// we use CAN1 on the SAME5x
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
	const unsigned int whichPort = 0;							// we use CAN0 on the SAMC21
#endif

	// Initialise the CAN hardware, using the timing data if it was valid
	can0dev = CanDevice::Init(0, whichPort, Can0Config, can0Memory, timing);

#ifdef SAMMYC21
	pinMode(CanStandbyPin, OUTPUT_LOW);							// take the CAN drivers out of standby
#endif

	boardAddress = canConfigData.GetCanAddress(defaultBoardAddress);

	// Set up a CAN receive filter to receive all messages addressed to us in FIFO 0
	can0dev->SetExtendedFilterElement(0, CanDevice::RxBufferNumber::fifo0,
										(uint32_t)boardAddress << CanId::DstAddressShift,
										CanId::BoardAddressMask << CanId::DstAddressShift);
	// We ignore broadcast messages so no need to set up a filter for them
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
	can0dev->SendMessage(CanDevice::TxBufferNumber::fifo, 1000, buf);
}

// End
