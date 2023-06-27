/*
 * Devices.cpp
 *
 *  Created on: 28 Jul 2020
 *      Author: David
 */

#include <Hardware/Devices.h>

#if SAME5x

#include <AnalogIn.h>
#include <AnalogOut.h>
#include <Platform/TaskPriorities.h>
#include <Platform/Platform.h>

// Analog input support
constexpr size_t AnalogInTaskStackWords = 300;
static Task<AnalogInTaskStackWords> analogInTask;

#if defined(EXP3HC)

void SerialPortInit(AsyncSerial*) noexcept
{
	SetPinFunction(PortBPin(20), GpioPinFunction::C);		// TxD
# if 0	// we don't use the receiver, but if we did we would need to do this:
	SetPinFunction(PortBPin(21), GpioPinFunction::C);		// RxD
# endif
}

void SerialPortDeinit(AsyncSerial*) noexcept
{
	pinMode(PortBPin(20), INPUT_PULLUP);
# if 0	// we don't use the receiver, but if we did we would need to do this:
	pinMode(PortBPin(21), INPUT_PULLUP);					// RxD
# endif
}

AsyncSerial uart0(3, 3, 512, 512, SerialPortInit, SerialPortDeinit);

extern "C" void SERCOM3_0_Handler()
{
	uart0.Interrupt0();
}

extern "C" void SERCOM3_2_Handler()
{
	uart0.Interrupt2();
}

extern "C" void SERCOM3_3_Handler()
{
	uart0.Interrupt3();
}

#elif defined(EXP1HCL) || defined(M23CL)

// Set up an optional serial port on the IO1 port via SERCOM2
void SerialPortInit(AsyncSerial*) noexcept
{
	SetPinFunction(PortAPin(12), GpioPinFunction::C);		// TxD
# if 0	// we don't use the receiver, but if we did we would need to do this:
	SetPinFunction(PortAPin(13), GpioPinFunction::C);		// RxD
# endif
}

void SerialPortDeinit(AsyncSerial*) noexcept
{
	pinMode(PortAPin(12), INPUT_PULLUP);
# if 0	// we don't use the receiver, but if we did we would need to do this:
	pinMode(PortAPin(13), INPUT_PULLUP);					// RxD
# endif
}

# if !SUPPORT_I2C_SENSORS

// Sercom2 on port io1 can be used as a UART if it is not being used for I2C
AsyncSerial uart0(2, 1, 512, 512, SerialPortInit, SerialPortDeinit);

extern "C" void SERCOM2_0_Handler()
{
	uart0.Interrupt0();
}

extern "C" void SERCOM2_2_Handler()
{
	uart0.Interrupt2();
}

extern "C" void SERCOM2_3_Handler()
{
	uart0.Interrupt3();
}

# endif

#endif

void DeviceInit() noexcept
{
#if defined(EXP1HCL) || defined(M23CL)
	pinMode(ClockGenPin, OUTPUT_LOW);			// default the TMC clock to its internal clock until we program the clock generator
#endif
	AnalogIn::Init(NvicPriorityAdc);
	AnalogOut::Init();
	analogInTask.Create(AnalogIn::TaskLoop, "AIN", nullptr, TaskPriority::AinPriority);

#if (defined(EXP1HCL) || defined(M23CL)) && USE_SERIAL_DEBUG
	Platform::SetInterruptPriority(SERCOM2_0_IRQn, 4, NvicPriorityUart);
#endif
}

#endif

// End
