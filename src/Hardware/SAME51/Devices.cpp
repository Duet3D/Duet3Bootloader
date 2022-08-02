/*
 * Devices.cpp
 *
 *  Created on: 28 Jul 2020
 *      Author: David
 */

#include <Hardware/Devices.h>

#if SAME5x

#include <Version.h>

#if defined(CAN_IAP)
extern const char VersionText[] = "Duet 3 Mini CAN IAP version " VERSION_TEXT;
#else
extern const char VersionText[] = "SAME5x bootloader version " VERSION_TEXT;
#endif

#ifdef DEBUG

# if defined(CAN_IAP)

// Duet 3 Mini uses SERCOM 2
void SerialPortInit(AsyncSerial*) noexcept
{
	SetPinFunction(PortBPin(25), GpioPinFunction::D);		// TxD
}

void SerialPortDeinit(AsyncSerial*) noexcept
{
	pinMode(PortBPin(25), INPUT_PULLUP);
}

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

#else

// EXP3HC and EXP1HCL use SERCOM 3
void SerialPortInit(AsyncSerial*) noexcept
{
	SetPinFunction(PortBPin(20), GpioPinFunction::C);		// TxD
}

void SerialPortDeinit(AsyncSerial*) noexcept
{
	pinMode(PortBPin(20), INPUT_PULLUP);
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

# endif
#endif

void DeviceInit() noexcept
{
}

#endif

// End
