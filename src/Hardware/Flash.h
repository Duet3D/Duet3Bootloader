/*
 * Flash.h
 *
 *  Created on: 8 Aug 2019
 *      Author: David
 */

#ifndef SRC_HARDWARE_FLASH_H_
#define SRC_HARDWARE_FLASH_H_

#include "RepRapFirmware.h"

namespace Flash
{
	void Init();
	bool UnlockAndErase(uint32_t start, uint32_t length);
	bool Lock(uint32_t start, uint32_t length);
	bool Write(uint32_t start, uint32_t length, uint8_t *data);
}

#endif /* SRC_HARDWARE_FLASH_H_ */
