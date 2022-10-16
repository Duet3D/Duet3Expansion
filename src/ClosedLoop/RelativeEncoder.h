/*
 * RelativeEncoder.h
 *
 *  Created on: 6 Sep 2021
 *      Author: Louis
 */

/*
 * To use the RelativeEncoder class, define:
 *
 *  - int32_t GetRelativePosition() to return a value representative of the distance moved relative to the start position
 *
 *  - Plus all the virtual functions required by the Encoder class
 */

#ifndef SRC_CLOSEDLOOP_RELATIVEENCODER_H_
# define SRC_CLOSEDLOOP_RELATIVEENCODER_H_

# include "Encoder.h"

# if SUPPORT_CLOSED_LOOP

class RelativeEncoder : public Encoder
{
public:
	// Constructors
	RelativeEncoder() noexcept : offset(0) {}

	// Overridden virtual functions

	// Return true if this is an absolute encoder
	bool IsAbsolute() const noexcept override { return false; }

	// Get the current reading
	int32_t GetReading() noexcept override
	{
		bool error;	// TODO: How to handle error?
		return GetRelativePosition(error) + offset;
	}

	// Offset management
	void SetOffset(int32_t newOffset) noexcept { offset += newOffset; }
	void ClearOffset() noexcept { offset = 0; }

	// Constants
	EncoderPositioningType GetPositioningType() const noexcept override { return EncoderPositioningType::relative; }

protected:
	// Get the relative position since the start
	virtual int32_t GetRelativePosition(bool& error) noexcept = 0;

	// For calculating the raw reading
	int32_t offset;
};

# endif

#endif /* SRC_CLOSEDLOOP_RELATIVEENCODER_H_ */
