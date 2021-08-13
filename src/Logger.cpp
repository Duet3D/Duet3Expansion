/*
 * Logger.cpp
 *
 *  Created on: 17 Sep 2017
 *      Author: David
 */

#include "Logger.h"

#if SUPPORT_CAN_LOGGING

# include <CAN/CanInterface.h>
# include <CanMessageBuffer.h>
# include <CanMessageFormats.h>

const int CAN_MSG_MAX_CHARS = 50;

void Logger::LogMessage(time_t time, const StringRef& message, LogLevel type) noexcept
{
	int startPointer = 0;
	int messageLength = message.strlen();
	do {
		// Set up a CAN message
		CanMessageBuffer buf(nullptr);
		CanMessageLogMessage& msg = *(buf.SetupStatusMessage<CanMessageLogMessage>(CanInterface::GetCanAddress(), CanInterface::GetCurrentMasterAddress()));

		// Populate the message fields
		msg.time = time;
		msg.type = type.ToBaseType();
		msg.lastPacket = startPointer + CAN_MSG_MAX_CHARS >= messageLength;
		msg.packetLength = msg.lastPacket ? messageLength - startPointer : CAN_MSG_MAX_CHARS;

		// TODO: Can we use memcpy here?
		for (int i=0; i<msg.packetLength; i++)
		{
			msg.message[i] = message.c_str()[startPointer + i];
		}

		// Send the CAN message
		buf.dataLength = msg.GetActualDataLength();
		CanInterface::Send(&buf);

		// Up the current start pointer
		startPointer += CAN_MSG_MAX_CHARS;
	} while (startPointer < messageLength);
}

#endif	/* SUPPORT_CAN_LOGGING */

// End
