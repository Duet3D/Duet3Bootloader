/*
 * Can.cpp
 *
 *  Created on: 17 Sep 2018
 *      Author: David
 */

#include <CAN/CanInterface.h>
#include "CanMessageFormats.h"
#include "CanMessageBuffer.h"

const unsigned int NumCanBuffers = 4;

static CanAddress boardAddress;

class CanMessageQueue
{
public:
	CanMessageQueue();
	void AddMessage(CanMessageBuffer *buf);
	CanMessageBuffer *GetMessage();

private:
	CanMessageBuffer *pendingMessages;
	CanMessageBuffer *lastPendingMessage;			// only valid when pendingMessages != nullptr
};

CanMessageQueue::CanMessageQueue() : pendingMessages(nullptr) { }

void CanMessageQueue::AddMessage(CanMessageBuffer *buf)
{
	buf->next = nullptr;
	{
		if (pendingMessages == nullptr)
		{
			pendingMessages = lastPendingMessage = buf;
		}
		else
		{
			lastPendingMessage->next = buf;
		}
	}
}

// Fetch a message from the queue, or return nullptr if there are no messages
CanMessageBuffer *CanMessageQueue::GetMessage()
{
	CanMessageBuffer *buf;
	{
		buf = pendingMessages;
		if (buf != nullptr)
		{
			pendingMessages = buf->next;
		}
	}
	return buf;
}

static CanMessageQueue PendingMoves;
static CanMessageQueue PendingCommands;

extern "C" struct can_async_descriptor CAN_0;

#if 0
extern "C" void CAN_0_tx_callback(struct can_async_descriptor *const descr)
{
	//TODO
}

extern "C" void CAN_0_rx_callback(struct can_async_descriptor *const descr)
{
	//TODO
}
#endif

// Get a received CAN message if there is one
bool CanInterface::GetCanMessage(CanMessageBuffer *buf)
{
	can_message msg;										// descriptor for the message
	msg.data = reinterpret_cast<uint8_t*>(&(buf->msg));		// set up where we want the message data to be stored
	const int32_t rslt = can_async_read(&CAN_0, &msg);		// fetch the message
	if (rslt == ERR_NONE)
	{
		buf->dataLength = msg.len;
		buf->id.SetReceivedId(msg.id);
		return true;
	}
	return false;
}

void CanInterface::Init(CanAddress pBoardAddress)
{
	boardAddress = (pBoardAddress == CanId::MasterAddress) ? CanId::FirmwareUpdateAddress : pBoardAddress;
	CanMessageBuffer::Init(NumCanBuffers);

//	can_async_register_callback(&CAN_0, CAN_ASYNC_RX_CB, (FUNC_PTR)CAN_0_rx_callback);
//	can_async_register_callback(&CAN_0, CAN_ASYNC_TX_CB, (FUNC_PTR)CAN_0_tx_callback);

	// Set up CAN receiver filtering
	can_filter filter;

	// First a filter for our own ID
	filter.id = (uint32_t)boardAddress << CanId::DstAddressShift;
	filter.mask = CanId::BoardAddressMask << CanId::DstAddressShift;
	can_async_set_filter(&CAN_0, 0, CAN_FMT_EXTID, &filter);

	// We ignore broadcast messages so on need to set up a filter for them

	can_async_enable(&CAN_0);
}

CanAddress CanInterface::GetCanAddress()
{
	return boardAddress;
}

// Send a CAN message and free the buffer
void CanInterface::Send(CanMessageBuffer *buf)
{
	can_message msg;
	msg.id = buf->id.GetWholeId();
	msg.type = CAN_TYPE_DATA;
	msg.data = buf->msg.raw;
	msg.len = buf->dataLength;
	msg.fmt = CAN_FMT_EXTID;
	int32_t err;
	do
	{
		err = can_async_write(&CAN_0, &msg);
	} while (err != ERR_NONE);
	CanMessageBuffer::Free(buf);
}

// End
