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
	RelativeEncoder(float p_countsPerStep) noexcept : Encoder(p_countsPerStep) {}

	// Overridden virtual functions

	// Return true if this is an absolute encoder
	bool IsAbsolute() const noexcept override { return false; }

	// Get the current reading
	int32_t GetReading(bool& err) noexcept override
	{
		return GetRelativePosition(err) + offset;
	}

protected:
	// Get the relative position since the start
	virtual int32_t GetRelativePosition(bool& error) noexcept = 0;
};

# endif

#endif /* SRC_CLOSEDLOOP_RELATIVEENCODER_H_ */
