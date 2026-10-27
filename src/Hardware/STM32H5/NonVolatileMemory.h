/*
 * NonVolatileMemory.h
 *
 * Class to manage an area of flash memory that we use to store data that persists over a reset
 *  Created on: 24 Aug 2020
 *      Author: David
 */

#ifndef SRC_HARDWARE_NONVOLATILEMEMORY_H_
#define SRC_HARDWARE_NONVOLATILEMEMORY_H_

#include <RepRapFirmware.h>

#if RPXXXX || STM32H5
# include <CanSettings.h>
#endif

// This class manages nonvolatile settings that are specific to the board, and the software reset data that is stored by the crash handler.
// On most Duets there is a 512-byte User Page that we use for this.
// The SAMC21 and SAME5x processors already store various data in the user page, however both those processor families support EEPROM emulation so we use 512 bytes of that instead.

enum class NvmPage : uint8_t { common = 0, closedLoop };

class NonVolatileMemory
{
public:
	NonVolatileMemory(NvmPage whichPage) noexcept;

	void *operator new(size_t, void *p) noexcept { return p; }			// for placement new

	void EnsureWritten() noexcept;

#if RPXXXX || STM32H5
	bool GetCanSettings(CanUserAreaData& canSettings) noexcept pre(page == NvmPage::common);
	void SetCanSettings(CanUserAreaData& canSettings) noexcept pre(page == NvmPage::common);
#endif

private:
	void EnsureRead() noexcept;
	void SetDirty(bool eraseNeeded) noexcept pre(state != NvmState::notRead);

	// In the following structs, the magic value must always be the first 2 bytes
	struct CommonPage
	{
		static constexpr unsigned int NumberOfResetDataSlots = 3;
		static constexpr unsigned int MaxCalibratedThermistors = 8;

		uint16_t magic;
		uint8_t thermistorLowCalibration[MaxCalibratedThermistors];
		uint8_t thermistorHighCalibration[MaxCalibratedThermistors];
#if RPXXXX || STM32H5
		CanUserAreaData canSettings;
		uint8_t spare[30-16];
#else
		uint8_t spare[30];
#endif
		uint8_t inductiveHeaterParams[8];

		// 56 bytes up to here
		uint8_t resetData[NumberOfResetDataSlots * 152];			// 3 slots of 152 bytes each
	};

	static_assert(sizeof(CommonPage) == 512);

	union NVM
	{
		CommonPage commonPage;
	};

	static_assert(sizeof(NVM) == 512);

	uint32_t GetMagicValue() const noexcept { return 0x41E5 + (unsigned int)page + ((unsigned int)page << 8); }

	enum class NvmState : uint8_t { notRead, clean, writeNeeded, eraseAndWriteNeeded };

	alignas(4) NVM buffer;
	NvmState state;
	NvmPage page;
};

#endif /* SRC_HARDWARE_NONVOLATILEMEMORY_H_ */
