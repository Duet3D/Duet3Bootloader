/*
 * Serial.cpp - simple serial driver for sending messages to an attached PanelDue
 *
 *  Created on: 9 Aug 2019
 *      Author: David
 */

#include "Serial.h"

#include "SAME5x.h"
#include <peripheral_clk_config.h>
#include <hal_gpio.h>
#include <atmel_start_pins.h>

#include <hri_sercom_e51.h>

constexpr uint32_t DiagBaudRate = 57600;		// the baud rate we use
#define DIAG_SERCOM	SERCOM3						// which SERCOM device we use

// Initialise the serial port so that we can send messages to an attached PanelDue
void Serial::Init()
{
	// The following code assumes SERCOM3
	hri_gclk_write_PCHCTRL_reg(GCLK, SERCOM3_GCLK_ID_CORE, CONF_GCLK_SERCOM3_CORE_SRC | (1 << GCLK_PCHCTRL_CHEN_Pos));
	hri_gclk_write_PCHCTRL_reg(GCLK, SERCOM3_GCLK_ID_SLOW, CONF_GCLK_SERCOM3_SLOW_SRC | (1 << GCLK_PCHCTRL_CHEN_Pos));
	hri_mclk_set_APBBMASK_SERCOM3_bit(MCLK);
	gpio_set_pin_function(PB20, PINMUX_PB20C_SERCOM3_PAD0);					// TxD
	gpio_set_pin_function(PB21, PINMUX_PB21C_SERCOM3_PAD1);					// RxD

	// The remaining code should work with the SERCOM that DIAG_SERCOM is defined as
	const uint32_t ctrla = (1u << SERCOM_USART_CTRLA_DORD_Pos)				// MSB first
						 | (0u << SERCOM_USART_CTRLA_CPOL_Pos)				// use rising clock edge
						 | (0u << SERCOM_USART_CTRLA_CMODE_Pos)				// async mode
						 | (0u << SERCOM_USART_CTRLA_FORM_Pos)				// usart frame, no parity
						 | (0u << SERCOM_USART_CTRLA_SAMPA_Pos)				// sample on clocks 7-8-9
						 | (3u << SERCOM_USART_CTRLA_RXPO_Pos)				// receive data on pad 3
						 | (0u << SERCOM_USART_CTRLA_TXPO_Pos)				// transmit on pad 0
						 | (0u << SERCOM_USART_CTRLA_SAMPR_Pos)				// 16x over sampling, normal baud rate generation
						 | (0u << SERCOM_USART_CTRLA_RXINV_Pos)				// don't invert receive data
						 | (0u << SERCOM_USART_CTRLA_TXINV_Pos)				// don't invert transmitted data
						 | (0u << SERCOM_USART_CTRLA_IBON_Pos)				// don't report buffer overflow early
						 | (0u << SERCOM_USART_CTRLA_RUNSTDBY_Pos)			// don't clock during standby
						 | (1u << SERCOM_USART_CTRLA_MODE_Pos)				// use internal clock
						 | (0u << SERCOM_USART_CTRLA_ENABLE_Pos)			// not enabled
						 | (0u << SERCOM_USART_CTRLA_SWRST_Pos);			// no reset
	if (!hri_sercomusart_is_syncing(DIAG_SERCOM, SERCOM_USART_SYNCBUSY_SWRST))
	{
		const uint32_t mode = ctrla & SERCOM_USART_CTRLA_MODE_Msk;
		if (hri_sercomusart_get_CTRLA_reg(DIAG_SERCOM, SERCOM_USART_CTRLA_ENABLE))
		{
			hri_sercomusart_clear_CTRLA_ENABLE_bit(DIAG_SERCOM);
			hri_sercomusart_wait_for_sync(DIAG_SERCOM, SERCOM_USART_SYNCBUSY_ENABLE);
		}
		hri_sercomusart_write_CTRLA_reg(DIAG_SERCOM, SERCOM_USART_CTRLA_SWRST | mode);
	}
	hri_sercomusart_wait_for_sync(DIAG_SERCOM, SERCOM_USART_SYNCBUSY_SWRST);

	DIAG_SERCOM->USART.CTRLA.reg = ctrla;
	DIAG_SERCOM->USART.CTRLB.reg = SERCOM_USART_CTRLB_TXEN | SERCOM_USART_CTRLB_RXEN;
	DIAG_SERCOM->USART.CTRLC.reg = 0u;
	const uint32_t baudReg = 65536 - (((uint64_t)65536 * 16 * DiagBaudRate)/SystemPeripheralClock);
	DIAG_SERCOM->USART.BAUD.reg = baudReg;
	hri_sercomusart_set_CTRLA_ENABLE_bit(DIAG_SERCOM);
}

// Undo the initialisation, so that when we jump into the main firmware the USART can be initialised again
void Serial::Disable()
{
	hri_sercomusart_clear_CTRLA_ENABLE_bit(DIAG_SERCOM);
	hri_sercomusart_set_CTRLA_SWRST_bit(DIAG_SERCOM);
}

void Serial::Send(char c)
{
	while ((DIAG_SERCOM->USART.INTFLAG.reg & SERCOM_USART_INTFLAG_DRE) == 0) { }
	DIAG_SERCOM->USART.DATA.reg = c;
}

void Serial::Send(const char *s)
{
	char c;
	while ((c = *s++) != 0)
	{
		Send(c);
	}
}

// End
