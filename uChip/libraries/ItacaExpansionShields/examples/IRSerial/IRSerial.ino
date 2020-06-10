/*
  Simple sketch for uChip showing how to implement a Serial over IR (Infrared).

  Demonstrates how to use uChip to implement a Serial comunication over IR with minimum amount of components
  uChip can be powered either via USB or via external voltage VEXT (3.3V - 5V)
  When USB powered the external voltage VEXT is left to default (3.3V)

  This example code is in the public domain, more details can be found on the online guide at

  https://shop.itaca-innovation.com

  created by Antonio Rizzo <info@itaca-innovation.com>
  24 Apr 2020
*/

#include <uChipPowerManagement.h>

// PWM (analogWrite on digital PINs) configuration:
// Default is 0xFFFF which gives a PWM frequency of 730Hz
// This is because analogWrite is feeding directly the 48MHz to the timer counter
// Therefore, in order to obtain 38kHz to match the decoder frequency we need to set the 
// PER_REG = 48MHz/38KHz ~ 1264 = 0x04F0
// The PWM width sets the output power, to reduce cross-talk we set this to a 12.5%
// Increase the CC_REG in order to increas output power
// CC_REG = PER_REG / 8
#define PER_REG 0x04F0
// 12.5%
#define CC_REG  (PER_REG >> 3)

// It is mandatory to use a TCC timer thus any PINs among 1,2,3,4,5,6,12 or 13 on uChip
// Check the pinout of your board in the Variang.c file

// PIN used to generate the modulation frequency
#define PIN_PWM_MOD 1
// PIN used to power the TSOP
#define TSOP_GND_PIN uC5
#define TSOP_VDD_PIN uC6

// Comment/uncomment accordingly to your needs
#define TX
#define RX


void setup() {
  // Disabling the boost
  uChipEnableBoost(UCHIP_BOOST_DISABLED);
  // Setting the external voltage to 3V3
  uChipSetVextValue(VEXT_3V3);
  // Setting up the LED port
  pinMode(LED_BUILTIN, OUTPUT);
  
  // The TSOP is powered using digital pins
  // because the power consumption is very limited.
  // Therefore we need to setup the digital pins accordingly.
  // Set GND for TSOP
  pinMode(TSOP_GND_PIN, OUTPUT);
  digitalWrite(TSOP_GND_PIN, LOW);
  // Set VDD
  pinMode(TSOP_VDD_PIN, OUTPUT);
  // Power the TSOP only if RX feature is necessary
#ifdef RX
  digitalWrite(TSOP_VDD_PIN, HIGH);
#else
  digitalWrite(TSOP_VDD_PIN, LOW);
#endif

#ifdef TX
  // Setting up PWM at 38000 frequency
  setupTimer();
#endif
  // Since we are going at 38k, in order to have at least 10 pulses per bit
  // we use 16 pulses.
  // We need to use the correct bitrate on the serial
  // Bitrate = 38000/16 = 2400bps
  // The Serial uses pins of RX0 = 4, TX0 = 2.
  // When using Serial1 pins assigned are RX1 = 5, TX1 = 3
  Serial.begin(2400);
  SerialUSB.begin(2400);
  // Wait for SerialUSB to open
  while(!SerialUSB);
}


// The main loop is nothing but a passthrough routine
// Change the code to run your code using the serial as
// a common serial.
void loop() {  
  // passthrogh
#ifdef TX
  if(SerialUSB.available())
    Serial.write(SerialUSB.read());  
#endif
#ifdef RX
  if(Serial.available())
    SerialUSB.write(Serial.read()); 
#endif
  }



  

// Wait for synchronization of registers between the clock domains
static __inline__ void syncTCC(Tcc* TCCx) __attribute__((always_inline, unused));
static void syncTCC(Tcc* TCCx) {
  while (TCCx->SYNCBUSY.reg & TCC_SYNCBUSY_MASK);
}

// This function sets up the timer such that the required PWM is generated
// Then signal is delivered by the serial switching HIGH/LOW
void setupTimer(void)
{
  PinDescription pinDesc = g_APinDescription[PIN_PWM_MOD];
  uint8_t tcChannel = GetTCChannelNumber(pinDesc.ulPWMChannel);
  Tcc* TCCx = (Tcc*) GetTC(pinDesc.ulPWMChannel);
  
  analogWrite(PIN_PWM_MOD, 0);
  // Disable TCCx
  TCCx->CTRLA.bit.ENABLE = 0;
  syncTCC(TCCx);
  // Set the initial value
  TCCx->CC[tcChannel].reg = (uint32_t) CC_REG;
  syncTCC(TCCx);
  // Write Period
  TCCx->PER.reg = PER_REG;
  syncTCC(TCCx);
  // Enable TCCx
  TCCx->CTRLA.bit.ENABLE = 1;
  syncTCC(TCCx);
  }
