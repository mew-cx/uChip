/*
 * uChip Power management library.
 * Copyright (c) 2019 Itaca Innovation.
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */


#ifndef _UCHIP_POWER_MANAGEMENT_H_INCLUDED
#define _UCHIP_POWER_MANAGEMENT_H_INCLUDED

#include <Arduino.h>

#define UCHIP_BOOST_DISABLED 0
#define UCHIP_BOOST_ENABLED 1
#define SLEEP_IDLE0 0
#define SLEEP_IDLE1 1
#define SLEEP_IDLE2 2
#define SLEEP_DEEP 4
#define SLEEP_ON_EXIT 1
#define NO_SLEEP_ON_EXIT 0
#define VEXT_USB 1
#define VEXT_3V3 0
void uChipEnableBoost(uint8_t enable);
void uChipSetVextValue(uint8_t value);
void uChipSleep(uint8_t sleepMode, uint8_t sleepOnExit);
#endif
