/*
 * Devices.cpp
 *
 *  Created on: 28 Jul 2020
 *      Author: David
 */

#include <Hardware/Devices.h>

#if SAME5x

#ifdef DEBUG

Uart uart0(3, 3, 512, 512);

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
#ifdef DEBUG
	SetPinFunction(PortBPin(20), GpioPinFunction::C);		// TxD
# if 0	// we don't use the receiver, but if we did we would need to do this:
	SetPinFunction(PortBPin(21), GpioPinFunction::C);		// RxD
# endif
#endif
}

#endif

// End
