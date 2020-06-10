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

#include "uChipPowerManagement.h"
#include <Arduino.h>
#include <sam.h>
#define UCHIP_PIN_BOOST_ENABLE 14
#define UCHIP_PIN_USB_HOST_EN 28
#define UCHIP_PIN_VEXT_SELECT 15
/*
*   uChipEnableBoost: enables the internal boost, if an OTG cable is inserted (ID_PIN low).
*   NOTE: The boost is in any case enabled only when an OTG cable is inserted. At the same time the buck is disabled!
*   Params:  
*    enable = UCHIP_BOOST_DISABLED:  Boost always disabled, regardless the presence of the OTG cable.
*    enable = UCHIP_BOOST_ENABLED:  Boost is enabled in presence of the OTG cable.
*/
void uChipEnableBoost(uint8_t enable)
{
  // set the boost enable feature so that it is enabled only if the correct cable is connected
  PORT->Group[0].PINCFG[UCHIP_PIN_USB_HOST_EN].reg = PORT_PINCFG_PULLEN | PORT_PINCFG_INEN;
  REG_PORT_DIRCLR0 = (1 << UCHIP_PIN_USB_HOST_EN);
  // now set pin state
  if (enable == UCHIP_BOOST_DISABLED)
     REG_PORT_OUTCLR0 = (1 << UCHIP_PIN_BOOST_ENABLE);
  else
     REG_PORT_OUTSET0 = (1 << UCHIP_PIN_BOOST_ENABLE);
  // and configure pin as output
  REG_PORT_DIRSET0 = (1 << UCHIP_PIN_BOOST_ENABLE);
}
/*
*  uChipSetVextValue: sets the output voltage. This works only if uChip is USB powered!
*  Params:
*  value = VEXT_USB: VEXT (Pin 16) is set to the USB voltage (typically 5V +/- 10%. Note the actual voltage will depend on the output current.
*  value = VEXT_3V3: VEXT (pin 16) is set to 3.3V.
*/
void uChipSetVextValue(uint8_t value)
{
  if (value == VEXT_USB)
     REG_PORT_OUTSET0 = (1 << UCHIP_PIN_VEXT_SELECT);
  else
     REG_PORT_OUTCLR0 = (1 << UCHIP_PIN_VEXT_SELECT);
     // and configure pin as output
  REG_PORT_DIRSET0 = (1 << UCHIP_PIN_VEXT_SELECT);
}
/*
*  uChipSleep: Put uChip in a low power mode.
*  Params:
*  sleepMode = SLEEP_IDLE0: The CPU clock domain is stopped
*  sleepMode = SLEEP_IDLE1: The CPU, and AHB clock domains are stopped
*  sleepMode = SLEEP_IDLE2: The CPU, AHB and APB clock domains are stopped
*  sleepMode = SLEEP_DEEP:  All clocks are stopped except those which are kept running if requested by a running module or have the
*                           ONDEMAND bit set to zero. Furthermore the regulator and the RAM are set in low power mode.
*  sleepOnExit = SLEEP_ON_EXIT: When an interrupt occurs, it wakes up uChip, and the interrupt routine (ISR) is serviced. After the ISR is complete,
*                               uChip automatically goes back to sleep.
*  sleepOnExit = NO_SLEEP_ON_EXT: same as SLEEP_ON_EXIT, but when the ISR is complete, uChip does not automatically go into sleep mode again.
*/
void uChipSleep(uint8_t sleepMode, uint8_t sleepOnExit)
{
     switch (sleepMode)
     {
            case SLEEP_IDLE0:
            case SLEEP_IDLE1:
            case SLEEP_IDLE2:
                 if (sleepOnExit)
                    SCB->SCR = SCB_SCR_SLEEPONEXIT_Msk;
                 else
                    SCB->SCR = 0;
 	          PM->SLEEP.bit.IDLE = sleepMode;
                  break;
            case SLEEP_DEEP:
                 if (sleepOnExit)
                    SCB->SCR = SCB_SCR_SLEEPONEXIT_Msk | SCB_SCR_SLEEPDEEP_Msk;
                 else
                    SCB->SCR = SCB_SCR_SLEEPDEEP_Msk;
                 break;
           default:
                   return; // no sleep if you do not specify a valid sleep mode!
     }
     asm ("DSB\n\t"
          "WFI\n\t");
}