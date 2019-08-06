/*
 * Io.h
 *
 *  Created on: 6 Aug 2019
 *      Author: David
 */

#ifndef SRC_HARDWARE_IO_H_
#define SRC_HARDWARE_IO_H_

#include "RepRapFirmware.h"

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

#endif /* SRC_HARDWARE_IO_H_ */
