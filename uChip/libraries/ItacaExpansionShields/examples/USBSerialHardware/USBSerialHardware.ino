/*
  Simple sketch for uChip showing how to use the Hardware Serial port in the microUSB connector.
  
  Demonstrates how to configure uChip to use the Hardware Serial port in the microUSB connector.
  The power can come either from USB or from VEXT.
  To power an external (slave) device connected via to the microUSB port you need an OTG adapter.
  uChip is programmed via USB, after programming connect your device.
  Use an OTG adapter to enable boost capabilities and power uChip via external voltage VEXT (3.3V - 5V).
  IF THE POWER COMES FROM USB DO NOT USE OTG ADAPTER.
 
  This example code is in the public domain, more details can be found on the online guide at

  https://shop.itaca-innovation.com

  created by Antonio Rizzo <info@itaca-innovation.com>
  24 Apr 2020
*/


#include <uChipPowerManagement.h>
#include "wiring_private.h"


// These defines change the use of the microUSB pins.
// If SERIAL_USB_HW is defined, on startup the microUSB will be used as an Hardware serial port
// If SERIAL_USB_HW is not defined, the microUSB will be used as standard virtual COM (Arduino default)
#define SERIAL_USB_HW

// Defines where the power comes from.
// If VEXT_POWER is defined the code assumes that we are powering from VEXT (3.3V - 5V)
// and we wish to enable the BOOST to power external devices connected via OTG adapter.
#define VEXT_POWER

// Define Baud
#define BAUD 38400


#ifdef SERIAL_USB_HW

  // Pins defines are within the variant.h for the specified board.
  // If not included in the variant please define it here.
  // The default pin assignment is based on uChip
  // NB: In case you are using an Arduino M0 use TX --> 33 & RX --> 34
  // NB: In case you are using an Arduino Zero use TX --> 28 & RX --> 29
  #ifndef PIN_SERIAL_USB_HW_TX 
    #define  PIN_SERIAL_USB_HW_TX 37
  #endif
  #ifndef PIN_SERIAL_USB_HW_RX 
    #define  PIN_SERIAL_USB_HW_RX 38
  #endif
  // Telling the Uart constructor which sercom pad it should use (this part is common for all SAMD21 MCU)
  #ifndef PAD_SERIAL_USB_HW_TX 
    #define  PAD_SERIAL_USB_HW_TX UART_TX_PAD_2
  #endif
  #ifndef PAD_SERIAL_USB_HW_RX 
    #define  PAD_SERIAL_USB_HW_RX SERCOM_RX_PAD_3
  #endif

  // Creating the new Serial
  Uart SerialUSB_HW( &sercom3, PIN_SERIAL_USB_HW_RX, PIN_SERIAL_USB_HW_TX, PAD_SERIAL_USB_HW_RX, PAD_SERIAL_USB_HW_TX);
#else
  // In case you want to use the standard SerialUSB which is a virtual COM emulation
  #define SerialUSB_HW SerialUSB
#endif

// Variable for reading the incoming char, we read one char at a time at the moment.
char income;

void setup() {
  // Enabling/Disabling the boost accordingly to the define.
#ifdef VEXT_POWER
  uChipEnableBoost(UCHIP_BOOST_ENABLED);
#else
  uChipEnableBoost(UCHIP_BOOST_DISABLED);
#endif
  // Setting the external voltage to 3V3 (Default)
  // This setting can be changed if we need the VUSB on the VEXT
  uChipSetVextValue(VEXT_3V3);
  // Setting up the LED port
  pinMode(LED_BUILTIN, OUTPUT);
  // Turning off the LED
  digitalWrite(LED_BUILTIN, LOW);

#ifdef SERIAL_USB_HW
  // Initialize serial, this instruction is ok both for
  // the HW serial or the standard SerialUSB
  SerialUSB_HW.begin(BAUD);
  // WARNING! Keep instructions in this order! 
  // pinPeripheral MUST be called AFTER SerialUSB.begin()!
  // When using the usb as HW serial we need to
  // change USB pins function from USB mode to SERCOM
  pinPeripheral(PIN_SERIAL_USB_HW_TX, PIO_SERCOM); 
  pinPeripheral(PIN_SERIAL_USB_HW_RX, PIO_SERCOM);
#endif
}


// The loop does nothing but receiving the chars and looking
// for a special char which allows turning off/on the led
void loop() {
  if (SerialUSB_HW.available()) {      
    income = (SerialUSB_HW.read());
    // Prints on the USB that the char was received
    SerialUSB_HW.println("ACK");
  }
  if(income == 'o')
  {
    // Turn on the LED
    digitalWrite(LED_BUILTIN, HIGH);
  }
  else
  {
    // Turn off the LED if anything else arrives
    digitalWrite(LED_BUILTIN, LOW);
  }
  // Print something on Serial, like the status of the LED
  SerialUSB_HW.print("The LED is ");
  if (digitalRead(LED_BUILTIN) == HIGH)
    SerialUSB_HW.println(" ON");
  else
    SerialUSB_HW.println(" OFF");

  // Add a little delay to prevent the Serial from spamming the output
  delay(200);
}


#ifdef SERIAL_USB_HW
  // Attach the interrupt handler to the SERCOM3
  // This is necessary otherwise nothing works!
  void SERCOM3_Handler()
  {
    SerialUSB_HW.IrqHandler();
  }
#endif
