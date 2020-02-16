/****************************************************************************************************

RepRapFirmware - Configuration

This is where all machine-independent configuration and other definitions are set up. Nothing that
depends on any particular RepRap, RepRap component, or RepRap controller should go in here. Define
machine-dependent things in Platform.h

-----------------------------------------------------------------------------------------------------

Version 0.1

18 November 2012

Adrian Bowyer
RepRap Professional Ltd
http://reprappro.com

Licence: GPL

****************************************************************************************************/

#ifndef CONFIGURATION_H
#define CONFIGURATION_H

#include <cstddef>			// for size_t

// Generic constants
constexpr float ABS_ZERO = -273.15;						// Celsius
constexpr float NEARLY_ABS_ZERO = -273.0;				// Celsius
constexpr float ROOM_TEMPERATURE = 21.0;				// Celsius

// Timeouts
constexpr uint32_t FanCheckInterval = 500;				// Milliseconds
constexpr uint32_t OpenLoadTimeout = 500;				// Milliseconds
constexpr uint32_t MinimumWarningInterval = 4000;		// Milliseconds, must be at least as long as FanCheckInterval
constexpr uint32_t LogFlushInterval = 15000;			// Milliseconds
constexpr uint32_t DriverCoolingTimeout = 4000;			// Milliseconds
constexpr float DefaultMessageTimeout = 10.0;			// How long a message is displayed by default, in seconds

constexpr uint32_t MinimumOpenLoadFullStepsPerSec = 20;

// FanCheckInterval must be lower than MinimumWarningInterval to avoid giving driver over temperature warnings too soon when thermostatic control of electronics cooling fans is used
static_assert(FanCheckInterval < MinimumWarningInterval, "FanCheckInterval too large");

// Comms defaults
constexpr unsigned int MAIN_BAUD_RATE = 115200;			// Default communication speed of the USB if needed
constexpr unsigned int AUX_BAUD_RATE = 57600;			// Ditto - for auxiliary UART device
constexpr unsigned int AUX2_BAUD_RATE = 115200;			// Ditto - for second auxiliary UART device
constexpr uint32_t SERIAL_MAIN_TIMEOUT = 1000;			// timeout in ms for sending data to the main serial/USB port

// Heater values
constexpr uint32_t HeatSampleIntervalMillis = 250;		// interval between taking temperature samples
constexpr float HeatPwmAverageTime = 5.0;				// Seconds

constexpr float TEMPERATURE_CLOSE_ENOUGH = 1.0;			// Celsius
constexpr float TEMPERATURE_LOW_SO_DONT_CARE = 40.0;	// Celsius
constexpr float HOT_ENOUGH_TO_EXTRUDE = 160.0;			// Celsius
constexpr float HOT_ENOUGH_TO_RETRACT = 90.0;			// Celsius

constexpr uint8_t MaxBadTemperatureCount = 4;			// Number of bad temperature samples permitted before a heater fault is reported
constexpr float BadLowTemperature = -10.0;				// Celsius
constexpr float DefaultHotEndTemperatureLimit = 290.0;	// Celsius - E3D say to tighten the hot end at 285C
constexpr float DefaultBedTemperatureLimit = 125.0;		// Celsius
constexpr float DefaultAllowedOverTemperature = 5.0;
constexpr float DefaultHotEndFanTemperature = 45.0;		// Temperature at which a thermostatic hot end fan comes on
constexpr float ThermostatHysteresis = 1.0;				// How much hysteresis we use to prevent noise turning fans on/off too often
constexpr float BadErrorTemperature = 2000.0;			// Must exceed any reasonable 5temperature limit including DEFAULT_TEMPERATURE_LIMIT
constexpr uint32_t DefaultHeaterFaultTimeout = 10 * 60 * 1000;	// How long we wait (in milliseconds) for user intervention after a heater fault before shutting down

// Default thermistor parameters
constexpr float DefaultR25 = 100000.0;
constexpr float DefaultBeta = 4388.0;
constexpr float DefaultShc = 0.0;

// Heating model default parameters. For the chamber heater, we use the same values as for the bed heater.
// These parameters are about right for an E3Dv6 hot end with 30W heater.
constexpr float DefaultHotEndHeaterGain = 340.0;
constexpr float DefaultHotEndHeaterTimeConstant = 140.0;
constexpr float DefaultHotEndHeaterDeadTime = 5.5;

constexpr unsigned int FirstExtraHeaterProtection = 100;	// Index of the first extra heater protection item

// These parameters are about right for a typical PCB bed heater that maxes out at 110C
constexpr float DefaultBedHeaterGain = 90.0;
constexpr float DefaultBedHeaterTimeConstant = 700.0;
constexpr float DefaultBedHeaterDeadTime = 10.0;

// Parameters used to detect heating errors
constexpr float DefaultMaxHeatingFaultTime = 5.0;			// How many seconds we allow a heating fault to persist
constexpr float AllowedTemperatureDerivativeNoise = 0.25;	// How much fluctuation in the averaged temperature derivative we allow
constexpr float MaxAmbientTemperature = 45.0;				// We expect heaters to cool to this temperature or lower when switched off
constexpr float NormalAmbientTemperature = 25.0;			// The ambient temperature we assume - allow for the printer heating its surroundings a little
constexpr float DefaultMaxTempExcursion = 15.0;				// How much error we tolerate when maintaining temperature before deciding that a heater fault has occurred
constexpr float MinimumConnectedTemperature = -5.0;			// Temperatures below this we treat as a disconnected thermistor

static_assert(DefaultMaxTempExcursion > TEMPERATURE_CLOSE_ENOUGH, "DefaultMaxTempExcursion is too low");

// PWM frequencies
constexpr PwmFrequency MaxHeaterPwmFrequency = 1000;	// maximum supported heater PWM frequency, to avoid overheating the mosfets
constexpr PwmFrequency SlowHeaterPwmFreq = 10;			// slow PWM frequency for bed and chamber heaters, compatible with DC/AC SSRs
constexpr PwmFrequency NormalHeaterPwmFreq = 250;		// normal PWM frequency used for hot ends
constexpr PwmFrequency DefaultFanPwmFreq = 250;			// increase to 25kHz using M106 command to meet Intel 4-wire PWM fan specification
constexpr PwmFrequency DefaultPinWritePwmFreq = 500;	// default PWM frequency for M42 pin writes and extrusion ancillary PWM
constexpr PwmFrequency ServoRefreshFrequency = 50;

// String lengths. Keeping the number of distinct lengths small will reduce flash memory usage.
constexpr size_t FormatStringLength = 256;
constexpr size_t MaxMessageLength = 256;
constexpr size_t MaxTitleLength = 61;
constexpr size_t StringLength20 = 20;
constexpr size_t StringLength50 = 50;
constexpr size_t StringLength100 = 50;

constexpr size_t GCODE_LENGTH = 161;					// maximum number of non-comment characters in a line of GCode including the null terminator
constexpr size_t SHORT_GCODE_LENGTH = 61;				// maximum length of a GCode that we can queue to synchronise it to a move

// Output buffer length and number of buffers
// When using RTOS, it is best if it is possible to fit an HTTP response header in a single buffer. Our headers are currently about 230 bytes long.
// A note on reserved buffers: the worst case is when a GCode with a long response is processed. After string the response, there must be enough buffer space
// for the HTTP responder to return a status response. Otherwise DWC never gets to know that it needs to make a rr_reply call and the system deadlocks.
constexpr size_t OUTPUT_BUFFER_SIZE = 256;				// How many bytes does each OutputBuffer hold?
constexpr size_t OUTPUT_BUFFER_COUNT = 20;				// How many OutputBuffer instances do we have?
constexpr size_t RESERVED_OUTPUT_BUFFERS = 4;			// Number of reserved output buffers after long responses, enough to hold a status response

const size_t maxQueuedCodes = 16;						// How many codes can be queued?

// Move system
constexpr float DefaultFeedRate = 3000.0;				// The initial requested feed rate after resetting the printer, in mm/min
constexpr float DefaultG0FeedRate = 18000;				// The initial feed rate for G0 commands after resetting the printer, in mm/min
constexpr float DefaultRetractSpeed = 1000.0;			// The default firmware retraction and un-retraction speed, in mm
constexpr float DefaultRetractLength = 2.0;
constexpr float MinimumMovementSpeed = 0.5;				// The minimum movement speed (extruding moves will go slower than this if the extrusion rate demands it)
constexpr uint32_t ProbingSpeedReductionFactor = 3;		// The factor by which we reduce the Z probing speed when we get a 'near' indication
constexpr float ZProbeMaxAcceleration = 250.0;			// Maximum Z acceleration to use at the start of a probing move

constexpr float DefaultArcSegmentLength = 0.2;			// G2 and G3 arc movement commands get split into segments this long

constexpr uint32_t DefaultIdleTimeout = 30000;			// Milliseconds
constexpr float DefaultIdleCurrentFactor = 0.3;			// Proportion of normal motor current that we use for idle hold

constexpr float DefaultNonlinearExtrusionLimit = 0.2;	// Maximum additional commanded extrusion to compensate for nonlinearity
constexpr size_t NumRestorePoints = 4;					// Number of restore points, must be at least 3

// Triggers
constexpr unsigned int MaxTriggers = 10;				// Must be <= 32 because we store a bitmap of pending triggers in a uint32_t

// Default nozzle and filament values
constexpr float NOZZLE_DIAMETER = 0.5;					// Millimetres
constexpr float FILAMENT_WIDTH = 1.75;					// Millimetres

constexpr unsigned int MaxStackDepth = 5;				// Maximum depth of stack

#endif
