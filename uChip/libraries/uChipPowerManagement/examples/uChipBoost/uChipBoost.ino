/*
  Simple sketch for uChip showing power management of BOOST

  Demonstrates the use of the power management library to control the BOOST
  feature on uChip.
  uChip MUST BE VEXT POWERED and an USB OTG adapter must be connected to USB port to run this example.

  This example code is in the public domain, more details can be found on the online guide at

  https://shop.itaca-innovation.com

  created by Antonio Rizzo <info@itaca-innovation.com>
  4 Apr 2020

  The code is loaded via USB, however while the USB is connected the BOOST cannot be used.
  Once the program is correctly loaded, disconnect the USB and power uChip prom VEXT [3.3V to 5V]
  We setup uChip such that: 
   *  - The boost feature is initially disabled
   *  - VEXT is set to 3V3 as default
   *  - The boost in the loop turns on/off every 5 seconds
   */
#include <uChipPowerManagement.h>

uint8_t boost_status = UCHIP_BOOST_DISABLED;

void setup() {
  // Disabling the boost since the variable was initialized so.
  uChipEnableBoost(boost_status);
  // Setting the external voltage to 3V3
  uChipSetVextValue(VEXT_3V3);
  // Setting up the LED port
  pinMode(LED_BUILTIN, OUTPUT);
}



void loop() {
  // The loop simply demonstrates how to switch ON/OFF the boost
  // It is possible to observe the 5V on the USB port only if an USB OTG
  // adapter is connected to the USB port
  
  // If the status flag of the boost is ON we turn on the LED
  if(boost_status == UCHIP_BOOST_ENABLED)
  {
      digitalWrite(LED_BUILTIN, HIGH);  
  }
  else
  {
      digitalWrite(LED_BUILTIN, LOW);
  }
  // Add a delay to avoid switching continuously
  delay(5000);
  // Change the status
  if(boost_status == UCHIP_BOOST_DISABLED)
  {
      boost_status = UCHIP_BOOST_ENABLED; 
  }
  else
  {
      boost_status = UCHIP_BOOST_DISABLED;
  }
  // Then write the status into the GPIO
  uChipEnableBoost(boost_status);
}
