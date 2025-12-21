/*
 * Can.h
 *
 *  Created on: 17 Sep 2018
 *      Author: David
 */

#ifndef SRC_CAN_CANINTERFACE_H_
#define SRC_CAN_CANINTERFACE_H_

#include "RepRapFirmware.h"
#include <CanId.h>

class CanMessageBuffer;
class CanTiming;

namespace CanInterface
{
	void Init(CanAddress defaultBoardAddress, bool doHardwareReset, unsigned int whichPort, bool useLaterPins) noexcept;
	void Shutdown() noexcept;

	CanAddress GetCanAddress() noexcept;
	void Send(CanMessageBuffer *buf) noexcept;
	bool GetCanMessage(CanMessageBuffer *buf) noexcept;

	void GetLocalCanTiming(CanTiming& timing) noexcept;
	void SetLocalCanTiming(const CanTiming& timing) noexcept;

#if !defined(CAN_IAP)
	bool StoreLocalCanTiming(const CanTiming& timing) noexcept;
#endif
}

#endif /* SRC_CAN_CANINTERFACE_H_ */
