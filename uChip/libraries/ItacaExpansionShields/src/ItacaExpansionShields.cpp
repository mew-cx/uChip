/*
 * Itaca Expansion Shields library.
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

#include "ItacaExpansionShields.h"
#include <Arduino.h>
#include <sam.h>
#ifndef ITACA_MOTOR_SHIELD_BYPASS_PIN
  #define ITACA_MOTOR_SHIELD_BYPASS_PIN 18
#endif

/*
*   itacaMotorShieldEnableBypass: enables or disables the PMOS bypass in the Itaca Motor Shield board.
*   Params:  
*    enable = ITACA_BYPASS_DE:  Bypass disabled.
*    enable = ITACA_BYPASS_EN:  Bypass enabled.
*/
void itacaMotorShieldSetBypass(uint8_t enable)
{
  if (enable == ITACA_BYPASS_EN)
     REG_PORT_OUTSET0 = (1 << ITACA_MOTOR_SHIELD_BYPASS_PIN);
  else
     REG_PORT_OUTCLR0 = (1 << ITACA_MOTOR_SHIELD_BYPASS_PIN);
     // and configure pin as output
  REG_PORT_DIRSET0 = (1 << ITACA_MOTOR_SHIELD_BYPASS_PIN);
}
/*
*   itacaRelayShieldInit: sets all channels as outputs and switches as input
*   Params:  
*    none
*/
void itacaRelayShieldInit(void)
{
	digitalWrite(ITACA_RELAY_CH_A, LOW);
	digitalWrite(ITACA_RELAY_CH_B, LOW);
	digitalWrite(ITACA_RELAY_CH_C, LOW);
	digitalWrite(ITACA_RELAY_CH_D, LOW);
	digitalWrite(ITACA_RELAY_CH_E, LOW);
	digitalWrite(ITACA_RELAY_CH_F, LOW);
	digitalWrite(ITACA_RELAY_CH_G, LOW);
	digitalWrite(ITACA_RELAY_CH_H, LOW);
	digitalWrite(ITACA_RELAY_CH_I, LOW);
	digitalWrite(ITACA_RELAY_CH_J, LOW);
	digitalWrite(ITACA_RELAY_CH_K, LOW);
	digitalWrite(ITACA_RELAY_CH_L, LOW);
	pinMode(ITACA_RELAY_CH_A, OUTPUT);
	pinMode(ITACA_RELAY_CH_B, OUTPUT);
	pinMode(ITACA_RELAY_CH_C, OUTPUT);
	pinMode(ITACA_RELAY_CH_D, OUTPUT);
	pinMode(ITACA_RELAY_CH_E, OUTPUT);
	pinMode(ITACA_RELAY_CH_F, OUTPUT);
	pinMode(ITACA_RELAY_CH_G, OUTPUT);
	pinMode(ITACA_RELAY_CH_H, OUTPUT);
	pinMode(ITACA_RELAY_CH_I, OUTPUT);
	pinMode(ITACA_RELAY_CH_J, OUTPUT);
	pinMode(ITACA_RELAY_CH_K, OUTPUT);
	pinMode(ITACA_RELAY_CH_L, OUTPUT);
	pinMode(ITACA_RELAY_SW_P1, INPUT_PULLUP);
	pinMode(ITACA_RELAY_SW_P2, INPUT_PULLUP);
}
/*
*   itacaRelayShieldResetAll: resets all channels
*   Params:  
*    none
*/
void itacaRelayShieldResetAll(void)
{
	digitalWrite(ITACA_RELAY_CH_A, LOW);
	digitalWrite(ITACA_RELAY_CH_B, LOW);
	digitalWrite(ITACA_RELAY_CH_C, LOW);
	digitalWrite(ITACA_RELAY_CH_D, LOW);
	digitalWrite(ITACA_RELAY_CH_E, LOW);
	digitalWrite(ITACA_RELAY_CH_F, LOW);
	digitalWrite(ITACA_RELAY_CH_G, LOW);
	digitalWrite(ITACA_RELAY_CH_H, LOW);
	digitalWrite(ITACA_RELAY_CH_I, LOW);
	digitalWrite(ITACA_RELAY_CH_J, LOW);
	digitalWrite(ITACA_RELAY_CH_K, LOW);
	digitalWrite(ITACA_RELAY_CH_L, LOW);
}
/*
*   itacaRelayShieldSetAll: sets all channels
*   Params:  
*    none
*/
void itacaRelayShieldSetAll(void)
{
	digitalWrite(ITACA_RELAY_CH_A, HIGH);
	digitalWrite(ITACA_RELAY_CH_B, HIGH);
	digitalWrite(ITACA_RELAY_CH_C, HIGH);
	digitalWrite(ITACA_RELAY_CH_D, HIGH);
	digitalWrite(ITACA_RELAY_CH_E, HIGH);
	digitalWrite(ITACA_RELAY_CH_F, HIGH);
	digitalWrite(ITACA_RELAY_CH_G, HIGH);
	digitalWrite(ITACA_RELAY_CH_H, HIGH);
	digitalWrite(ITACA_RELAY_CH_I, HIGH);
	digitalWrite(ITACA_RELAY_CH_J, HIGH);
	digitalWrite(ITACA_RELAY_CH_K, HIGH);
	digitalWrite(ITACA_RELAY_CH_L, HIGH);
}
/*
*   itacaRelayShieldSetCh: sets the specified channel as selected by status
*   Params:  
*    ch     = channel to be set.
*	 status = status of the channel (ITACA_RELAY_STATUS_DE/ITACA_RELAY_STATUS_EN)
*/
void itacaRelayShieldSetCh(uint8_t ch, uint8_t status)
{
	if(status == ITACA_RELAY_STATUS_EN)
	{
		digitalWrite(ch, HIGH);
	}
	else
	{
		digitalWrite(ch, LOW);
	}
}
/*
*   itacaRelayShieldGetCh: gets the specified channel status
*   Params:  
*    ch     = channel to get status.
*	Returns:
*    status = status of the channel (ITACA_RELAY_STATUS_DE/ITACA_RELAY_STATUS_EN)
*/
uint8_t itacaRelayShieldGetCh(uint8_t ch)
{
	if( digitalRead(ch) == HIGH)
	{
		return ITACA_RELAY_STATUS_EN;
	}
	else
	{
		return ITACA_RELAY_STATUS_DE;
	}
}

/*
*   itacaRelayShieldGetCh: gets the specified switch status
*   Params:  
*    sw     = switch to be read.
*	Returns:
*    status = status of the switch (ITACA_RELAY_SW_RELEASED/ITACA_RELAY_SW_PRESSED)
*/
uint8_t itacaRelayShieldGetSw(uint8_t sw)
{
	if(digitalRead(sw) == LOW)
	{
		return ITACA_RELAY_SW_PRESSED;
	}
	else
	{
		return ITACA_RELAY_SW_RELEASED;
	}
}