/*
 * Flash.cpp
 *
 *  Created on: 8 Aug 2019
 *      Author: David
 */

#include "Flash.h"

#include <hal/include/hal_flash.h>

namespace Flash
{
	static flash_descriptor flash;
}

void Flash::Init()
{
	flash_init(&flash, NVIC);
}

bool Flash::UnlockAndErase(uint32_t start, uint32_t length)
{
	const uint32_t pageSize = flash_get_page_size(&flash);
	return flash_unlock(&flash, start, length/pageSize) == 0 && flash_erase(&flash, start, length/pageSize) == 0;
}

bool Flash::Lock(uint32_t start, uint32_t length)
{
	const uint32_t pageSize = flash_get_page_size(&flash);
	return flash_lock(&flash, start, length/pageSize) == 0;
}

bool Flash::Write(uint32_t start, uint32_t length, uint8_t *data)
{
	return flash_write(&flash, start, data, length) == 0;
}

// End
