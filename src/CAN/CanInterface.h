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

namespace CanInterface
{
	void Init(CanAddress defaultBoardAddress, bool doHardwareReset, bool useAlternatePins);
	void Shutdown();

	CanAddress GetCanAddress();
	void SendAndFree(CanMessageBuffer *buf);
	bool GetCanMessage(CanMessageBuffer *buf);
}

#endif /* SRC_CAN_CANINTERFACE_H_ */
