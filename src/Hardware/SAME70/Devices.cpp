/*
 * Devices.cpp
 *
 *  Created on: 28 Jul 2020
 *      Author: David
 */

#include <Hardware/Devices.h>

#if SAME70

#include <Version.h>

#if defined(MB6HC)
extern const char VersionText[] = "Duet 3 MB6HC CAN IAP version " VERSION_TEXT;
#elif defined(MB6XD)
extern const char VersionText[] = "Duet 3 MB6XD CAN IAP version " VERSION_TEXT;
#endif

#ifdef DEBUG

void SerialPortInit(AsyncSerial*) noexcept
{
	SetPinFunction(PortDPin(26), GpioPinFunction::C);		// TxD
}

void SerialPortDeinit(AsyncSerial*) noexcept
{
	pinMode(PortDPin(26), INPUT_PULLUP);
}

AsyncSerial uart0(UART2, UART2_IRQn, ID_UART2, 512, 512, SerialPortInit, SerialPortDeinit);

void UART2_Handler(void) noexcept
{
	uart0.IrqHandler();
}

#endif

void DeviceInit() noexcept
{
}

#endif

// End
