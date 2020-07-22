/*
 * BoardDef.h
 *
 *  Created on: 30 Jun 2019
 *      Author: David
 */

#ifndef SRC_CONFIG_BOARDDEF_H_
#define SRC_CONFIG_BOARDDEF_H_

constexpr float DefaultThermistorR25 = 100000.0;
constexpr float DefaultThermistorbeta = 4388.0;
constexpr float DefaultThermistorC = 0.0;

#if SAME5x
# include "SAME51config.h"
#endif

#if SAMC21
# include "SAMC21config.h"
#endif

#define SUPPORT_CAN_EXPANSION	1

#endif /* SRC_CONFIG_BOARDDEF_H_ */
