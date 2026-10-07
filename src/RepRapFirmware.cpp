/*
 * RepRapFirmware.cpp
 *
 *  Created on: 31 Jul 2020
 *      Author: David
 */

#include "RepRapFirmware.h"

#include <syscalls.h>

// Define the system stack. The stack doesn't actually live here, instead the linker script uses this section to define the stack start and end symbols.
uint32_t dummySystemStack[SystemStackSize] __attribute__ ((section (".stack")));

// End
