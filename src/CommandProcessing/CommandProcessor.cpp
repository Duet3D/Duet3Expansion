/*
 * CommandProcessor.cpp
 *
 *  Created on: 26 Jul 2019
 *      Author: David
 */

#include "CommandProcessor.h"
#include <CAN/CanInterface.h>
#include "CanMessageBuffer.h"
#include "GCodes/GCodeResult.h"
#include "Heating/Heat.h"
#include "CanMessageGenericParser.h"

GCodeResult ProcessM950(const CanMessageGeneric& msg, const StringRef& reply)
{
	CanMessageGenericParser parser(msg, M950Params);
	uint16_t deviceNumber;
	if (parser.GetUintParam('F', deviceNumber))
	{
		//TODO configure fan
		reply.copy("Fan configuration not implemented");
		return GCodeResult::error;
	}
	if (parser.GetUintParam('H', deviceNumber))
	{
		//TODO configure servo
		reply.copy("heater configuration not implemented");
		return GCodeResult::error;
	}
	if (parser.GetUintParam('P', deviceNumber))
	{
		//TODO configure gpio
		reply.copy("GPIO configuration not implemented");
		return GCodeResult::error;
	}
	if (parser.GetUintParam('S', deviceNumber))
	{
		//TODO configure servo
		reply.copy("GPIO configuration not implemented");
		return GCodeResult::error;
	}
	reply.copy("Missing FPSH parameter");
	return GCodeResult::error;
}

void CommandProcessor::Spin()
{
	CanMessageBuffer *buf = CanInterface::GetCanCommand();
	if (buf != nullptr)
	{
		String<MaxCanReplyLength> reply;
		GCodeResult rslt;

		const CanMessageType id = buf->id.MsgType();
		switch (id)
		{
//		case CanMessageType::m307:
//			rslt = Heat::ProcessM307(buf->msg.generic, reply.GetRef());
//			break;

		case CanMessageType::m308:
			rslt = Heat::ProcessM308(buf->msg.generic, reply.GetRef());
			break;

		case CanMessageType::m950:
			rslt = ProcessM950(buf->msg.generic, reply.GetRef());
			break;

		default:
			reply.printf("Unknown message type %04x", (unsigned int)buf->id.MsgType());
			rslt = GCodeResult::error;
			break;
		}

		// Re-use the message buffer to send the reply
		CanMessageStandardReply *msg = buf->SetupResponseMessage<CanMessageStandardReply>(CanInterface::GetCanAddress(), buf->id.Src());
		msg->requestId = (uint16_t)id;
		msg->resultCode = (uint16_t)rslt;
		const size_t textLength = min<size_t>(reply.strlen() + 1, sizeof(buf->msg.standardReply.text));
		buf->dataLength = textLength + 2;
		memcpy(msg->text, reply.c_str(), textLength);
		CanInterface::Send(buf);
	}
}

// End
