/*
 * SimpleAnalogIn.h
 *
 *  Created on: 12 Jun 2020
 *      Author: David
 */

#ifndef SRC_HARDWARE_SIMPLEANALOGIN_H_
#define SRC_HARDWARE_SIMPLEANALOGIN_H_

#include <RepRapFirmware.h>

namespace SimpleAnalogIn
{
	void Init(Adc * device);
	void Disable(Adc * device);
	uint16_t ReadChannel(Adc * device, uint8_t channel);
}

#endif /* SRC_HARDWARE_SIMPLEANALOGIN_H_ */
