/*
 * Devices.h
 *
 *  Created on: 31 Jul 2020
 *      Author: David
 */

#ifndef SRC_HARDWARE_DEVICES_H_
#define SRC_HARDWARE_DEVICES_H_

#include <CoreIO.h>

#ifdef DEBUG

#include <AsyncSerial.h>
extern AsyncSerial uart0;

#endif

void DeviceInit();

#endif /* SRC_HARDWARE_DEVICES_H_ */
