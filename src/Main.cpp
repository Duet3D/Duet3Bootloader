/*
 * Main.cpp
 *
 *  Created on: 3 Aug 2019
 *      Author: David
 */

#include "RepRapFirmware.h"
#include <Config/peripheral_clk_config.h>
#include "Hardware/Io.h"
#include "Config/BoardDef.h"
#include <General/StringRef.h>
#include <CanId.h>

const uint32_t FIRMWARE_FLASH_START = FLASH_ADDR + 0x00010000;			// we reserve 64K for the bootloader

static uint32_t blockBuffer[(65 * 1024)/4];

uint8_t ReadBoardId();
bool CheckValidFirmware();
void StartFirmware();

extern "C" int main()
{
	// Initialize MCU, drivers and middleware
	atmel_start_init();
	SystemCoreClock = CONF_CPU_FREQUENCY;

	// Check whether address switches are set to zero. If so then we stay in the bootloader
	if (ReadBoardId() != 0)
	{
		// Check whether there is valid firmware installed, if not then stay in the bootloader
		if (CheckValidFirmware())
		{
			// Relocate the vector table and jump into the firmware. If it returns then we execute the bootloader.
			StartFirmware();
		}
	}

	// If we get here then we are staying in the bootloader
	// Initialise the CAN subsystem, DMAC and systick
	DeviceInit();
	//TODO initialise systick
	//qq;
	//qq;

	// Loop requesting firmware from the main board and handling any firmware that it sends to us
	for (;;)
	{
		// If nothing received recently, ask the main board for the next block
		//qq;

		// Deal with any received blocks
		//qq;
	}
}

// Read the board address
uint8_t ReadBoardId()
{
	uint8_t rslt = 0;
	for (unsigned int i = 0; i < 4; ++i)
	{
		if (!digitalRead(BoardAddressPins[i]))
		{
			rslt |= 1 << i;
		}
	}
	return rslt;
}

// Check that valid firmware is installed
bool CheckValidFirmware()
{
	//TODO
	return true;
}

// Execute the main firmware
void StartFirmware()
{
	// Disable all IRQs
	SysTick->CTRL  = SysTick_CTRL_CLKSOURCE_Msk;	// disable the system tick exception
	__disable_irq();
	for (size_t i = 0; i < 8; i++)
	{
		NVIC->ICER[i] = 0xFFFFFFFF;					// Disable IRQs
		NVIC->ICPR[i] = 0xFFFFFFFF;					// Clear pending IRQs
	}

	digitalWrite(DiagLedPin, false);				// turn the DIAG LED off

	hri_wdt_write_CLEAR_reg(WDT, WDT_CLEAR_CLEAR_KEY);	// reset the watchdog timer

	// Modify vector table location
	__DSB();
	__ISB();
	SCB->VTOR = ((uint32_t)FIRMWARE_FLASH_START & SCB_VTOR_TBLOFF_Msk);
	__DSB();
	__ISB();

	__enable_irq();

	__asm volatile ("mov r3, %0" : : "r" (FIRMWARE_FLASH_START) : "r3");

	// We are using separate process and handler stacks. Put the process stack 1K bytes below the handler stack.
	__asm volatile ("ldr r1, [r3]");
	__asm volatile ("msr msp, r1");
	__asm volatile ("sub r1, #1024");
	__asm volatile ("mov sp, r1");

	__asm volatile ("isb");
	__asm volatile ("ldr r1, [r3, #4]");
	__asm volatile ("orr r1, r1, #1");
	__asm volatile ("bx r1");
}

// End
