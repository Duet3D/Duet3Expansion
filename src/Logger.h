/*
 * Logger.h
 *
 *  Created on: 21 Jul 2021
 *      Author: Louis
 */

#ifndef SRC_LOGGER_H_
# define SRC_LOGGER_H_

# include <Duet3Common.h>
# include <RepRapFirmware.h>

# if SUPPORT_CAN_LOGGING

#  include <ctime>

namespace Logger
{
	void LogMessage(time_t time, const StringRef& message, LogLevel type) noexcept;
};

# endif /* SUPPORT_CAN_LOGGING */
#endif /* SRC_LOGGER_H_ */
