/*
 * Io.cpp
 *
 *  Created on: 11 Aug 2019
 *      Author: David
 */


#include "Io.h"

/*static*/ void SetPinMode(Pin pin, PinMode mode)
{
	switch (mode)
	{
	case INPUT:
		gpio_set_pin_function(pin,GPIO_PIN_FUNCTION_OFF);
		// The direction must be set before the pullup, otherwise setting the pullup doesn't work
		gpio_set_pin_direction(pin, GPIO_DIRECTION_IN);
		gpio_set_pin_pull_mode(pin, GPIO_PULL_OFF);
		break;

	case INPUT_PULLUP:
		gpio_set_pin_function(pin,GPIO_PIN_FUNCTION_OFF);
		// The direction must be set before the pullup, otherwise setting the pullup doesn't work
		gpio_set_pin_direction(pin, GPIO_DIRECTION_IN);
		gpio_set_pin_pull_mode(pin, GPIO_PULL_UP);
		break;

	case INPUT_PULLDOWN:
		gpio_set_pin_function(pin,GPIO_PIN_FUNCTION_OFF);
		// The direction must be set before the pullup, otherwise setting the pullup doesn't work
		gpio_set_pin_direction(pin, GPIO_DIRECTION_IN);
		gpio_set_pin_pull_mode(pin, GPIO_PULL_DOWN);
		break;

	case OUTPUT_LOW:
		gpio_set_pin_function(pin,GPIO_PIN_FUNCTION_OFF);
		gpio_set_pin_level(pin, false);
		gpio_set_pin_direction(pin, GPIO_DIRECTION_OUT);
		break;

	case OUTPUT_HIGH:
		gpio_set_pin_function(pin,GPIO_PIN_FUNCTION_OFF);
		gpio_set_pin_level(pin, true);
		gpio_set_pin_direction(pin, GPIO_DIRECTION_OUT);
		break;

	case AIN:
		// The SAME70 errata says we must disable the pullup resistor before enabling the AFEC channel
		gpio_set_pin_pull_mode(pin, GPIO_PULL_OFF);
		gpio_set_pin_direction(pin, GPIO_DIRECTION_OFF);		// disable the data input buffer
		gpio_set_pin_function(pin, GPIO_PIN_FUNCTION_B);		// ADC is always on peripheral B
		break;

	default:
		break;
	}
}

// End

