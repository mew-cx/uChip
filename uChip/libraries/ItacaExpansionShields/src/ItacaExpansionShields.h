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


#ifndef _ITACA_EXPANSION_SHIELDS_H_INCLUDED
#define _ITACA_EXPANSION_SHIELDS_H_INCLUDED

#include <Arduino.h>
//////////////////////////////////////////////////
// Motor defines and functions
enum 
{
ITACA_HV_MODE,
ITACA_LV_MODE
}
itaca_voltage_mode_t;
enum
{
ITACA_BYPASS_DE,
ITACA_BYPASS_EN
}
itaca_bypass_mode_t;
void itacaMotorShieldSetBypass(uint8_t enable);
//////////////////////////////////////////////////
// Relay shield defines and functions
enum 
{
ITACA_RELAY_CH_A = 1,
ITACA_RELAY_CH_B = 2,
ITACA_RELAY_CH_C = 3,
ITACA_RELAY_CH_D = 4,
ITACA_RELAY_CH_E = 5,
ITACA_RELAY_CH_F = 6,
ITACA_RELAY_CH_G = 7,
ITACA_RELAY_CH_H = 15,
ITACA_RELAY_CH_I = 12,
ITACA_RELAY_CH_J = 11,
ITACA_RELAY_CH_K = 10,
ITACA_RELAY_CH_L = 9
}
itaca_relay_ch_pinout_t;
enum 
{
ITACA_RELAY_SW_P1 = 13,
ITACA_RELAY_SW_P2 = 14
}
itaca_relay_sw_pinout_t;
enum 
{
ITACA_RELAY_STATUS_DE,
ITACA_RELAY_STATUS_EN
}
itaca_relay_ch_status_t;
enum 
{
ITACA_RELAY_SW_RELEASED,
ITACA_RELAY_SW_PRESSED
}
itaca_relay_sw_status_t;
void itacaRelayShieldInit(void);
void itacaRelayShieldResetAll(void);
void itacaRelayShieldSetAll(void);
void itacaRelayShieldSetCh(uint8_t ch, uint8_t status);
uint8_t itacaRelayShieldGetCh(uint8_t ch);
uint8_t itacaRelayShieldGetSw(uint8_t sw);
//////////////////////////////////////////////////
#endif
