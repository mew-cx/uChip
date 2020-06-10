/*
  Simple sketch for uChip showing power management of VEXT

  Demonstrates the use of the power management library to control the VEXT voltage
  uChip MUST BE USB POWERED to run this example.

  This example code is in the public domain, more details can be found on the online guide at

  https://shop.itaca-innovation.com

  created by Antonio Rizzo <info@itaca-innovation.com>
  4 Apr 2020

  We setup uChip such that: 
   *  - The boost feature is disabled 
   *    (by default when USB powered but showed here as an example)
   *  - VEXT is set to the predefined value set with the FLAG
   */
#include <uChipPowerManagement.h>

// Flag indicating the set VEXT voltage
// Uncomment the desired voltage on VEXT
//uint8_t vext_status = VEXT_USB;
uint8_t vext_status = VEXT_3V3;

void setup() {
  // Disabling the boost
  uChipEnableBoost(UCHIP_BOOST_DISABLED);
  // Setting the external voltage
  uChipSetVextValue(vext_status);
  // Setting up the LED port
  pinMode(LED_BUILTIN, OUTPUT);
}

void loop() {
  // The loop simply demonstrates how to switch from a VUSB to 3V3
  // and viceversa
  
  // If the status flag is USB voltage we turn on the LED
  if(vext_status == VEXT_USB)
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
  if(vext_status == VEXT_3V3)
  {
      vext_status = VEXT_USB; 
  }
  else
  {
      vext_status = VEXT_3V3; 
  }
  // Then write the status into the GPIO
  uChipSetVextValue(vext_status);
}
