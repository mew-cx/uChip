/*
  Copyright (c) 2014-2015 Arduino LLC.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
  See the GNU Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/
/*
 * +------------+------------------+--------+-----------------+--------------------------------------------------------------------------------------------------------
 * + Pin number +  ZERO Board pin  |  PIN   | Label/Name      | Comments (* is for default peripheral in use)
 * +------------+------------------+--------+-----------------+--------------------------------------------------------------------------------------------------------
 * |            | Digital Low      |        |                 |
 * +------------+------------------+--------+-----------------+--------------------------------------------------------------------------------------------------------
 * | 5          | 5                |  PA11  |                 | EIC/EXTINT[11] ADC/AIN[19]           PTC/X[3] *SERCOM0/PAD[3]  SERCOM2/PAD[3]  TCC0/WO[3]  TCC1/WO[1]
 * | 3          | 3                |  PA10  |                 | EIC/EXTINT[10] ADC/AIN[18]           PTC/X[2] *SERCOM0/PAD[2]                  TCC0/WO[2]  TCC1/WO[0]
 * | 8          | 8                |  PA14  | BOOST_ENABLE    | EIC/EXTINT[14]                                 SERCOM2/PAD[2]  SERCOM4/PAD[2]  TC3/WO[0]   TCC0/WO[4]
 * | 4          | ~4               |  PA09  |                 | EIC/EXTINT[9]  ADC/AIN[17]           PTC/X[1]  SERCOM0/PAD[1]  SERCOM2/PAD[1] *TCC0/WO[1]  TCC1/WO[3]
 * | 2          | ~2               |  PA08  |                 | EIC/NMI        ADC/AIN[16]           PTC/X[0]  SERCOM0/PAD[0]  SERCOM2/PAD[0] *TCC0/WO[0]  TCC1/WO[2]
 * | 16         | ~16              |  PA15  | VEXT_SELECT     | EIC/EXTINT[15]                                 SERCOM2/PAD[3]  SERCOM4/PAD[3] *TC3/WO[1]   TCC0/WO[5]
 * +------------+------------------+--------+-----------------+--------------------------------------------------------------------------------------------------------
 * |            | Digital High     |        |                 |
 * +------------+------------------+--------+-----------------+--------------------------------------------------------------------------------------------------------
 * | 0          | ~0               |  PA07  | LED                | EIC/EXTINT[7]  ADC/AIN[7]  AC/AIN[3] PTC/Y[5]  SERCOM0/PAD[3]                 *TCC1/WO[1]
 * | 7          | ~7               |  PA18  |                 | EIC/EXTINT[2]                        PTC/X[6] +SERCOM1/PAD[2]  SERCOM3/PAD[2] *TC3/WO[0]    TCC0/WO[2]
 * | 1          | ~1               |  PA16  |                 | EIC/EXTINT[0]                        PTC/X[4] +SERCOM1/PAD[0]  SERCOM3/PAD[0] *TCC2/WO[0]   TCC0/WO[6]
 * | 9          | ~9               |  PA19  |                 | EIC/EXTINT[3]                        PTC/X[7] +SERCOM1/PAD[3]  SERCOM3/PAD[3]  TC3/WO[1]   *TCC0/WO[3]
 * | 6          | ~6               |  PA17  |                 | EIC/EXTINT[1]                        PTC/X[5] +SERCOM1/PAD[1]  SERCOM3/PAD[1] *TCC2/WO[1]   TCC0/WO[7]
 * +------------+------------------+--------+-----------------+--------------------------------------------------------------------------------------------------------
 * |            | Analog Connector |        |                 |
 * +------------+------------------+--------+-----------------+--------------------------------------------------------------------------------------------------------
 * | 21         | 14               |  PA02  | A5              | EIC/EXTINT[2] *ADC/AIN[0]  DAC/VOUT  PTC/Y[0]
 * | 22         | 15               |  PA04  | A6              | EIC/EXTINT[4] *ADC/AIN[4]  AC/AIN[0] PTC/Y[2]  SERCOM0/PAD[0]                  TCC0/WO[0]
 * | 24         | 15               |  PA05  | A8              | EIC/EXTINT[5] *ADC/AIN[5]  AC/AIN[1] PTC/Y[5]  SERCOM0/PAD[1]                  TCC0/WO[1]
 * +------------+------------------+--------+-----------------+--------------------------------------------------------------------------------------------------------
 * |            | Wire             |        |                 |
 * +------------+------------------+--------+-----------------+--------------------------------------------------------------------------------------------------------
 * | 25         | SDA              |  PA22  | SDA             | EIC/EXTINT[6]                        PTC/X[10] *SERCOM3/PAD[0] SERCOM5/PAD[0] TC4/WO[0] TCC0/WO[4]
 * | 26         | SCL              |  PA23  | SCL             | EIC/EXTINT[7]                        PTC/X[11] *SERCOM3/PAD[1] SERCOM5/PAD[1] TC4/WO[1] TCC0/WO[5]
 * +------------+------------------+--------+-----------------+--------------------------------------------------------------------------------------------------------
 * |            |SPI (Legacy ICSP) |        |                 |
 * +------------+------------------+--------+-----------------+--------------------------------------------------------------------------------------------------------
 * | 31         | 1                |  PA19  | MISO            | EIC/EXTINT[3] 				 *SERCOM1/PAD[3] TC3/WO[1]  
 * |            | 2                |        | 5V0             |
 * | 32         | 4                |  PA16  | MOSI            | EIC/EXTINT[0]                *SERCOM1/PAD[0] TCC2/WO[0] 
 * | 33         | 3                |  PA18  | SCK             | EIC/EXTINT[2]                *SERCOM1/PAD[2] TC3/WO[0]
 * |            | 5                |        | RESET           |
 * |            | 6                |        | GND             |
 * +------------+------------------+--------+-----------------+--------------------------------------------------------------------------------------------------------
 * |            | USB              |        |                 |
 * +------------+------------------+--------+-----------------+--------------------------------------------------------------------------------------------------------
 * | 27         |                  |  PA28  | USB_HOST_ENABLE | EIC/EXTINT[8]
 * | 28         |                  |  PA24  | USB_NEGATIVE    | *USB/DM
 * | 29         |                  |  PA25  | USB_POSITIVE    | *USB/DP
 * +------------+------------------+--------+-----------------+--------------------------------------------------------------------------------------------------------
 * |            |                  |        |                 |
 * +------------+------------------+--------+-----------------+--------------------------------------------------------------------------------------------------------
 * |            |32.768KHz Crystal |        |                 |
 * +------------+------------------+--------+-----------------+--------------------------------------------------------------------------------------------------------
 * |            |                  |  PA00  | XIN32           | EIC/EXTINT[0] SERCOM1/PAD[0] TCC2/WO[0]
 * |            |                  |  PA01  | XOUT32          | EIC/EXTINT[1] SERCOM1/PAD[1] TCC2/WO[1]
 * +------------+------------------+--------+-----------------+--------------------------------------------------------------------------------------------------------
 */


#include "variant.h"

/*
 * Pins descriptions
 */
const PinDescription g_APinDescription[]=
{
  // 0..16 - Digital pins
  // ----------------------
  
  // 0 (LED)
  { PORTA,   7, PIO_TIMER, (PIN_ATTR_DIGITAL|PIN_ATTR_PWM|PIN_ATTR_TIMER), No_ADC_Channel, PWM1_CH1, TCC1_CH1, EXTERNAL_INT_7   }, // TCC1/WO[1]	--LED
  // 1..7 GPIO GENERAL LEFT
  { PORTA,  16, PIO_TIMER, (PIN_ATTR_DIGITAL|PIN_ATTR_PWM|PIN_ATTR_TIMER), No_ADC_Channel, PWM2_CH0, TCC2_CH0, EXTERNAL_INT_0   }, // TCC2/WO[0]	--P1/MOSI
  { PORTA,   8, PIO_TIMER, (PIN_ATTR_DIGITAL|PIN_ATTR_PWM|PIN_ATTR_TIMER), No_ADC_Channel, PWM0_CH0, TCC0_CH0, EXTERNAL_INT_NMI}, // TCC0/WO[0]	--P2/TX0
  { PORTA,  10, PIO_TIMER, (PIN_ATTR_DIGITAL|PIN_ATTR_PWM|PIN_ATTR_TIMER), No_ADC_Channel, PWM1_CH0, TCC1_CH0, EXTERNAL_INT_10  }, // TCC1/WO[0]	--P3/TX1
  { PORTA,   9, PIO_TIMER, (PIN_ATTR_DIGITAL|PIN_ATTR_PWM|PIN_ATTR_TIMER), No_ADC_Channel, PWM0_CH1, TCC0_CH1, EXTERNAL_INT_9   }, // TCC0/WO[1]	--P4/RX0
  { PORTA,  11, PIO_TIMER, (PIN_ATTR_DIGITAL|PIN_ATTR_PWM|PIN_ATTR_TIMER), No_ADC_Channel, PWM1_CH1, TCC1_CH1, EXTERNAL_INT_11  }, // TCC1/WO[1]	--P5/RX1
  { PORTA,  17, PIO_TIMER, (PIN_ATTR_DIGITAL|PIN_ATTR_PWM|PIN_ATTR_TIMER), No_ADC_Channel, PWM2_CH1, TCC2_CH1, EXTERNAL_INT_1   }, // TCC2/WO[1]	--P6/SCK
  { PORTA,  18, PIO_TIMER, (PIN_ATTR_DIGITAL|PIN_ATTR_PWM|PIN_ATTR_TIMER), No_ADC_Channel, PWM3_CH0,  TC3_CH0, EXTERNAL_INT_2   }, // TC3/WO[0]		--P7/SS
  // 8 BOOST ENABLE PIN
  { PORTA, 14, PIO_DIGITAL, (PIN_ATTR_DIGITAL), No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE },// Output Only --BOOST_ENABLE
  // 9..15 GPIO GENERAL RIGHT
  { PORTA,  19, PIO_TIMER, (PIN_ATTR_DIGITAL|PIN_ATTR_PWM|PIN_ATTR_TIMER), No_ADC_Channel, PWM3_CH1,  TC3_CH1, EXTERNAL_INT_3   }, // TC3/WO[1]	--P9/MISO
  { PORTA,  22, PIO_TIMER, (PIN_ATTR_DIGITAL|PIN_ATTR_PWM|PIN_ATTR_TIMER), No_ADC_Channel, PWM4_CH0,  TC4_CH0, EXTERNAL_INT_6   }, // TC4/WO[0]	--P10/SDA
  { PORTA,  23, PIO_TIMER, (PIN_ATTR_DIGITAL|PIN_ATTR_PWM|PIN_ATTR_TIMER), No_ADC_Channel, PWM4_CH1,  TC4_CH1, EXTERNAL_INT_7   }, // TC4/WO[1]	--P11/SCL
  { PORTA,  31, PIO_TIMER, (PIN_ATTR_DIGITAL|PIN_ATTR_PWM|PIN_ATTR_TIMER), No_ADC_Channel, PWM1_CH1,  TCC1_CH1, EXTERNAL_INT_11  }, // TC1/WO[0]	--P12
  { PORTA,  30, PIO_TIMER, (PIN_ATTR_DIGITAL|PIN_ATTR_PWM|PIN_ATTR_TIMER), No_ADC_Channel, PWM1_CH0,  TCC1_CH0, EXTERNAL_INT_10  }, // TC1/WO[1]	--P13
  { PORTA,   2, PIO_DIGITAL, (PIN_ATTR_DIGITAL), No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_2 }, // ADC/AIN[0]	--P14
  { PORTA,   3, PIO_DIGITAL, (PIN_ATTR_DIGITAL), No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_3 }, // ADC/AIN[1]	--P15
  // 16 Vext voltage selector
  { PORTA,  15, PIO_DIGITAL, (PIN_ATTR_DIGITAL), No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE }, // Output Only --5V/3V3_SELECTOR
  
  // 17..25 - Analog pins
  // --------------------
  { PORTA,   6, PIO_ANALOG, PIN_ATTR_ANALOG, ADC_Channel6 , NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_6},		// ADC/AIN[6]	--P1
  { PORTA,   8, PIO_ANALOG, PIN_ATTR_ANALOG, ADC_Channel16, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NMI}, 	// ADC/AIN[16]	--P2
  { PORTA,  10, PIO_ANALOG, PIN_ATTR_ANALOG, ADC_Channel18, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_10},   	// ADC/AIN[18]	--P3
  { PORTA,   9, PIO_ANALOG, PIN_ATTR_ANALOG, ADC_Channel17, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_9},   	// ADC/AIN[17]	--P4
  { PORTA,  11, PIO_ANALOG, PIN_ATTR_ANALOG, ADC_Channel19, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_11}, 	// ADC/AIN[19]	--P5
  { PORTA,   5, PIO_ANALOG, PIN_ATTR_ANALOG, ADC_Channel5,  NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_5 }, 	// ADC/AIN[5]	--P13
  { PORTA,   2, PIO_ANALOG, PIN_ATTR_ANALOG, ADC_Channel0,  NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_2 }, 	// ADC/AIN[0]	--P14
  { PORTA,   3, PIO_ANALOG, PIN_ATTR_ANALOG, ADC_Channel1,  NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_3 }, 	// ADC/AIN[1]	--P15
  { PORTA,   4, PIO_ANALOG, PIN_ATTR_ANALOG, ADC_Channel4,  NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_4 }, 	// ADC/AIN[4]	--P15

  // 26/27 I2C pins (SDA/SCL)
  // ----------------------
  { PORTA, 22, PIO_SERCOM, (PIN_ATTR_DIGITAL), No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_6 }, // SDA: SERCOM3/PAD[0]		--P10
  { PORTA, 23, PIO_SERCOM, (PIN_ATTR_DIGITAL), No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_7 }, // SCL: SERCOM3/PAD[1]		--P11
  // 28..31 - SERCOM/UART 
  // ----------------------(Serial)
  { PORTA,  8, PIO_SERCOM, (PIN_ATTR_DIGITAL), No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NMI }, // TX: SERCOM0/PAD[0]		--P2
  { PORTA,  9, PIO_SERCOM, (PIN_ATTR_DIGITAL), No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_9 }, // RX: SERCOM0/PAD[1]		--p4
  // ----------------------(Serial1)
  { PORTA, 10, PIO_SERCOM_ALT, (PIN_ATTR_DIGITAL), No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_10 }, // TX: SERCOM2/PAD[2]	--P3
  { PORTA, 11, PIO_SERCOM_ALT, (PIN_ATTR_DIGITAL), No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_11 }, // RX: SERCOM2/PAD[3]	--P5
  // 32..35 - SPI pins (ICSP:MISO,SCK,MOSI)
  // ----------------------
  { PORTA, 19, PIO_SERCOM, (PIN_ATTR_DIGITAL), No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_3 }, // MISO: SERCOM1/PAD[3]	--p9
  { PORTA, 16, PIO_SERCOM, (PIN_ATTR_DIGITAL), No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_0 }, // MOSI: SERCOM1/PAD[0]	--p1
  { PORTA, 18, PIO_SERCOM, (PIN_ATTR_DIGITAL), No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_2 }, // SS: SERCOM1/PAD[2]		--p7
  { PORTA, 17, PIO_SERCOM, (PIN_ATTR_DIGITAL), No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_1 }, // SCK: SERCOM1/PAD[1]		--p6

  // 36..38 - USB
  // --------------------
  { PORTA, 28, PIO_COM, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE }, // --USB Host enable
  { PORTA, 24, PIO_COM, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE }, // --USB/DM
  { PORTA, 25, PIO_COM, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE }, // --USB/DP
  // 39 (AREF)
  { PORTA, 3, PIO_ANALOG, PIN_ATTR_ANALOG, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE }, // DAC/VREFP --P15
  // ----------------------
  // 40 - Alternate use of A7 (DAC output)
  { PORTA,  2, PIO_ANALOG, PIN_ATTR_ANALOG, DAC_Channel0, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_2 }, // DAC/VOUT	--P14
} ;

const void* g_apTCInstances[TCC_INST_NUM+TC_INST_NUM]={ TCC0, TCC1, TCC2, TC3, TC4, TC5 } ;

// Multi-serial objects instantiation
SERCOM sercom0( SERCOM0 ) ;
SERCOM sercom1( SERCOM1 ) ;
SERCOM sercom2( SERCOM2 ) ;
SERCOM sercom3( SERCOM3 ) ;

Uart Serial( &sercom0, PIN_SERIAL_RX, PIN_SERIAL_TX, PAD_SERIAL_RX, PAD_SERIAL_TX ) ;
Uart Serial1( &sercom2, PIN_SERIAL1_RX, PIN_SERIAL1_TX, PAD_SERIAL1_RX, PAD_SERIAL1_TX ) ;

void SERCOM0_Handler()
{
  Serial.IrqHandler();
}

void SERCOM2_Handler()
{
  Serial1.IrqHandler();
}

