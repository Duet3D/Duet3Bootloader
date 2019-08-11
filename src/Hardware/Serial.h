/*
 * Serial.h
 *
 *  Created on: 9 Aug 2019
 *      Author: David
 */

#ifndef SRC_HARDWARE_SERIAL_H_
#define SRC_HARDWARE_SERIAL_H_

#include "RepRapFirmware.h"

namespace Serial
{
	void Init();
	void Send(char c);
	void Send(const char *s);
}

#endif /* SRC_HARDWARE_SERIAL_H_ */
