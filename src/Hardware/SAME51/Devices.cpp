/*
 * Devices.cpp
 *
 *  Created on: 28 Jul 2020
 *      Author: David
 */

#include <Hardware/Devices.h>

#if SAME5x

#ifdef DEBUG

void SerialPortInit(Uart*) noexcept
{
	SetPinFunction(PortBPin(20), GpioPinFunction::C);		// TxD
}

void SerialPortDeinit(Uart*) noexcept
{
	pinMode(PortBPin(20), INPUT_PULLUP);
}

Uart uart0(3, 3, 512, 512, SerialPortInit, SerialPortDeinit);

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
