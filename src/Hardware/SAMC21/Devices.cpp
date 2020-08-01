/*
 * Devices.cpp
 *
 *  Created on: 28 Jul 2020
 *      Author: David
 */

#include <Hardware/Devices.h>

#if SAMC21

#include <RepRapFirmware.h>
#include <AnalogIn.h>

#ifdef DEBUG

# ifdef SAMMYC21

Uart uart0(5, 3, 512, 512);

extern "C" void SERCOM5_Handler()
{
	uart0.Interrupt();
}

# else

Uart uart0(4, 3, 512, 512);

extern "C" void SERCOM4_Handler()
{
	uart0.Interrupt();
}

# endif

#endif

void DeviceInit() noexcept
{
#ifdef DEBUG
# ifdef SAMMYC21
	gpio_set_pin_function(PortBPin(2), PINMUX_PB02D_SERCOM5_PAD0);		// TxD
# else
	gpio_set_pin_function(PortAPin(12), PINMUX_PA12D_SERCOM4_PAD0);		// TxD
# endif
#endif

#ifndef SAMMYC21
	AnalogIn::Init(CommonAdcDevice);
#endif
}

#endif

// End
