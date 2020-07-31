/*
 * Main.cpp
 *
 *  Created on: 27 Jul 2020
 *      Author: David
 */

#include "CoreIO.h"

#if SAMC21

#include <hpl_div.h>

static void InitClocks();

extern "C" void AppInit() noexcept
{
	InitClocks();

	// All done
	SystemCoreClock = SystemCoreClockFreq;
}

static void InitClocks()
{
	hri_nvmctrl_set_CTRLB_RWS_bf(NVMCTRL, 2);

	// 32kHz oscillators
	const uint16_t calib = hri_osc32kctrl_read_OSCULP32K_CALIB_bf(OSC32KCTRL);
	hri_osc32kctrl_write_OSCULP32K_reg(OSC32KCTRL, OSC32KCTRL_OSCULP32K_CALIB(calib));
	hri_osc32kctrl_write_RTCCTRL_reg(OSC32KCTRL, OSC32KCTRL_RTCCTRL_RTCSEL(OSC32KCTRL_RTCCTRL_RTCSEL_ULP32K_Val));

	// Crystal oscillator
	hri_oscctrl_write_XOSCCTRL_reg(OSCCTRL,
	    	  OSCCTRL_XOSCCTRL_STARTUP(0)
			| (0 << OSCCTRL_XOSCCTRL_AMPGC_Pos)
	        | OSCCTRL_XOSCCTRL_GAIN(3)						// 3 is suitable for 12 and 16MHz
			| (1 << OSCCTRL_XOSCCTRL_RUNSTDBY_Pos)
	        | (0 << OSCCTRL_XOSCCTRL_SWBEN_Pos)
			| (0 << OSCCTRL_XOSCCTRL_CFDEN_Pos)
	        | (1 << OSCCTRL_XOSCCTRL_XTALEN_Pos)
			| (1 << OSCCTRL_XOSCCTRL_ENABLE_Pos));

	hri_oscctrl_write_EVCTRL_reg(OSCCTRL, (0 << OSCCTRL_EVCTRL_CFDEO_Pos));

	while (!hri_oscctrl_get_STATUS_XOSCRDY_bit(OSCCTRL)) { }
	hri_oscctrl_set_XOSCCTRL_AMPGC_bit(OSCCTRL);

	// DPLL
	hri_oscctrl_write_DPLLRATIO_reg(OSCCTRL, OSCCTRL_DPLLRATIO_LDRFRAC(0) | OSCCTRL_DPLLRATIO_LDR(23));
	hri_oscctrl_write_DPLLCTRLB_reg(OSCCTRL,
#ifdef SAMMYC21
			  OSCCTRL_DPLLCTRLB_DIV(3)						// divide 16MHz by 8 to get 2MHz
#else
	    	  OSCCTRL_DPLLCTRLB_DIV(2)						// divide 12MHz by 6 to get 2MHz
#endif
			| (0 << OSCCTRL_DPLLCTRLB_LBYPASS_Pos)
	        | OSCCTRL_DPLLCTRLB_LTIME(0)
			| OSCCTRL_DPLLCTRLB_REFCLK(1)					// reference clock is XOSC
	        | (0 << OSCCTRL_DPLLCTRLB_WUF_Pos)
			| (0 << OSCCTRL_DPLLCTRLB_LPEN_Pos)
	        | OSCCTRL_DPLLCTRLB_FILTER(0));
	hri_oscctrl_write_DPLLPRESC_reg(OSCCTRL, OSCCTRL_DPLLPRESC_PRESC(0));
	hri_oscctrl_write_DPLLCTRLA_reg(OSCCTRL,
			  (0 << OSCCTRL_DPLLCTRLA_RUNSTDBY_Pos)
			| (1 << OSCCTRL_DPLLCTRLA_ENABLE_Pos));
	while (!(hri_oscctrl_get_DPLLSTATUS_LOCK_bit(OSCCTRL) || hri_oscctrl_get_DPLLSTATUS_CLKRDY_bit(OSCCTRL))) { }

	// MCLK
	hri_mclk_write_CPUDIV_reg(MCLK, MCLK_CPUDIV_CPUDIV(1));

	// GCLK 0
	hri_gclk_write_GENCTRL_reg(GCLK, 0,
		GCLK_GENCTRL_DIV(1) | (0 << GCLK_GENCTRL_RUNSTDBY_Pos)
			| (0 << GCLK_GENCTRL_DIVSEL_Pos) | (0 << GCLK_GENCTRL_OE_Pos)
			| (0 << GCLK_GENCTRL_OOV_Pos) | (0 << GCLK_GENCTRL_IDC_Pos)
			| (1 << GCLK_GENCTRL_GENEN_Pos) | 7);

	// GCLK 2
	hri_gclk_write_GENCTRL_reg(GCLK, 2,
		GCLK_GENCTRL_DIV(1) | (0 << GCLK_GENCTRL_RUNSTDBY_Pos)
			| (0 << GCLK_GENCTRL_DIVSEL_Pos) | (0 << GCLK_GENCTRL_OE_Pos)
			| (0 << GCLK_GENCTRL_OOV_Pos) | (0 << GCLK_GENCTRL_IDC_Pos)
			| (1 << GCLK_GENCTRL_GENEN_Pos) | 6);

	// Initialise divide and square root accelerator
	_div_init();
}

#endif

// End
