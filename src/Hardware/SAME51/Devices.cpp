/*
 * Devices.cpp
 *
 *  Created on: 28 Jul 2020
 *      Author: David
 */

#include <Hardware/Devices.h>

#if SAME5x

#include <Version.h>

extern const char VersionText[] = "SAME5x bootloader version " VERSION_TEXT;

#ifdef DEBUG

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

#endif

void DeviceInit() noexcept
{
}

#endif

// End
