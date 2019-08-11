/*
 * Io.h
 *
 *  Created on: 6 Aug 2019
 *      Author: David
 */

#ifndef SRC_HARDWARE_IO_H_
#define SRC_HARDWARE_IO_H_

#include "RepRapFirmware.h"

// Pin mode enumeration. Would ideally be a C++ scoped enum, but we need to use it from C library functions.
enum PinMode
{
	PIN_MODE_NOT_CONFIGURED = -1,	// used in Platform class to record that the mode for a pin has not been set yet
	INPUT = 0,						// pin is a digital input
	INPUT_PULLUP,					// pin is a digital input with pullup enabled
	INPUT_PULLDOWN,					// pin is a digital input with pulldown enabled
	OUTPUT_LOW,						// pin is an output with initial state LOW
	OUTPUT_HIGH,					// pin is an output with initial state HIGH
	AIN,							// pin is an analog input, digital input buffer is disabled if possible
	SPECIAL,						// pin is used for the special function defined for it in the variant.cpp file
	OUTPUT_PWM_LOW,					// PWM output mode, initially low
	OUTPUT_PWM_HIGH,				// PWM output mode, initially high
	OUTPUT_LOW_OPEN_DRAIN,			// used in SX1509B expansion driver to put the pin in open drain output mode
	OUTPUT_HIGH_OPEN_DRAIN,			// used in SX1509B expansion driver to put the pin in open drain output mode
	OUTPUT_PWM_OPEN_DRAIN			// used in SX1509B expansion driver to put the pin in PWM output mode
};

inline bool digitalRead(Pin p)
{
	return gpio_get_pin_level(p);
}

inline void digitalWrite(Pin p, bool high)
{
	gpio_set_pin_level(p, high);
}

inline void fastDigitalWriteHigh(Pin p)
{
	gpio_set_pin_level(p, true);
}

inline void fastDigitalWriteLow(Pin p)
{
	gpio_set_pin_level(p, false);
}

void SetPinMode(Pin pin, PinMode mode);

#endif /* SRC_HARDWARE_IO_H_ */
