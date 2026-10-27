/*
 * NonVolatileMemory.cpp
 *
 *  Created on: 24 Aug 2020
 *      Author: David
 */

#include "NonVolatileMemory.h"

#if SAM4E || SAM4S || SAME70
# include <Cache.h>
# include <flash_efc.h>
#elif SAMC21
# include <Flash.h>
constexpr uint32_t RWW_ADDR = FLASH_ADDR + 0x00400000;
#elif STM32
# include <NVMEmulation.h>
#elif RPXXXX
# include <hardware/flash.h>
// We allocate one sector for each type of non-volatile memory page. We store the page within the sector using wear levelling.
constexpr uint32_t FlashSectorSize = 4096;									// the flash chip has 4K sectors
constexpr uint32_t FlashSize = 2 * 1024 * 1024;								// the flash chip size in bytes (2Mbytes = 16Mbits)
constexpr uint32_t NvmPage0Offset = FlashSize - FlashSectorSize;			// the offset into flash memory where we store page 0 of the non-volatile data
constexpr uint32_t NvmPage0Addr = XIP_BASE + FlashSize - FlashSectorSize;	// the address where we can read page 0 of the non-volatile data
#endif

NonVolatileMemory::NonVolatileMemory(NvmPage whichPage) noexcept : state(NvmState::notRead), page(whichPage)
{
}

void NonVolatileMemory::EnsureRead() noexcept
{
	if (state == NvmState::notRead)
	{
#if SAME5x
		memcpyu32(reinterpret_cast<uint32_t *>(&buffer), reinterpret_cast<const uint32_t *>(SEEPROM_ADDR + (512 * (unsigned int)page)), sizeof(buffer)/sizeof(uint32_t));
#elif SAMC21
		memcpyu32(reinterpret_cast<uint32_t *>(&buffer), reinterpret_cast<const uint32_t *>(RWW_ADDR + (512 * (unsigned int)page)), sizeof(buffer)/sizeof(uint32_t));
#elif STM32
		NVMEmulationRead((uint8_t)page, &buffer, sizeof(buffer));
#elif RPXXXX
		//TODO don't just read the first page in the sector, search for the most recent one written
		memcpyu32(reinterpret_cast<uint32_t *>(&buffer), reinterpret_cast<const uint32_t *>(NvmPage0Addr - (FlashSectorSize * (unsigned int)page)), sizeof(buffer)/sizeof(uint32_t));
#else
# error Unsupported processor
#endif
		if (buffer.commonPage.magic != GetMagicValue())
		{
			memset(&buffer, 0xFF, sizeof(buffer));
			buffer.commonPage.magic = GetMagicValue();
			SetDirty(true);
		}
		else
		{
			state = NvmState::clean;
		}
	}
}

void NonVolatileMemory::SetDirty(bool eraseNeeded) noexcept
{
	if (eraseNeeded && state < NvmState::eraseAndWriteNeeded)
	{
		state = NvmState::eraseAndWriteNeeded;
	}
	else if (state < NvmState::writeNeeded)
	{
		state = NvmState::writeNeeded;
	}
}

void NonVolatileMemory::EnsureWritten() noexcept
{
#if SAME5x
	if (state >= NvmState::writeNeeded)
	{
		// No need to erase on the SAME5x because the EEPROM emulation manages it
        while (NVMCTRL->SEESTAT.bit.BUSY) { }
        memcpyu32(reinterpret_cast<uint32_t*>(SEEPROM_ADDR + (512 * (unsigned int)page)), reinterpret_cast<const uint32_t*>(&buffer), sizeof(buffer)/sizeof(uint32_t));
		state = NvmState::clean;
        while (NVMCTRL->SEESTAT.bit.BUSY) { }
	}
#elif SAMC21
	if (state == NvmState::eraseAndWriteNeeded)
	{
		Flash::RwwErase(RWW_ADDR + (512 * (unsigned int)page), 512);
		state = NvmState::writeNeeded;
	}

	if (state == NvmState::writeNeeded)
	{
		Flash::RwwWrite(RWW_ADDR + (512 * (unsigned int)page), 512, (uint8_t*)&buffer);
		state = NvmState::clean;
	}
#elif SAM4E || SAME70
	if (state == NvmState::eraseAndWriteNeeded)
	{
		Flash::EraseUserSignature();
		state = NvmState::writeNeeded;
	}
	if (state == NvmState::writeNeeded)
	{
		const bool cacheEnabled = Cache::Disable();
		Flash::WriteUserSignature(reinterpret_cast<const uint32_t*>(&buffer));
		if (cacheEnabled)
		{
			Cache::Enable();
		}
	}
#elif STM32
	if (state == NvmState::eraseAndWriteNeeded)
	{
		NVMEmulationErase((uint8_t)page);
		state = NvmState::writeNeeded;
	}
	if (state == NvmState::writeNeeded)
	{
		NVMEmulationWrite((uint8_t)page, &buffer, sizeof(buffer));
		state = NvmState::clean;
	}
#elif RPXXXX
	if (state == NvmState::eraseAndWriteNeeded)
	{
		DisableCore1Processing();
		IrqDisable();
		flash_range_erase(NvmPage0Offset, FlashSectorSize);
		IrqEnable();
		//TODO allocate a new page in the sector, if there is one, else erase the sector
		state = NvmState::writeNeeded;
		EnableCore1Processing();
	}

	if (state == NvmState::writeNeeded)
	{
		DisableCore1Processing();
		IrqDisable();
		flash_range_program(NvmPage0Offset, (uint8_t *)&buffer, 512);
		IrqEnable();
		state = NvmState::clean;
		EnableCore1Processing();
	}
#else
# error Unsupported processor
#endif
}

#if RPXXXX || STM32H5

bool NonVolatileMemory::GetCanSettings(CanUserAreaData& canSettings) noexcept
{
	EnsureRead();
	canSettings = buffer.commonPage.canSettings;
	return true;
}

void NonVolatileMemory::SetCanSettings(CanUserAreaData& canSettings) noexcept
{
	EnsureRead();
	if (memcmp(&canSettings, &buffer.commonPage.canSettings, sizeof(CanUserAreaData)) != 0)
	{
		buffer.commonPage.canSettings = canSettings;
		SetDirty(true);
	}
}

#endif

// End
