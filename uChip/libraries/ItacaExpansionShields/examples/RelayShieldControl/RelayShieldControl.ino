/*
  Simple sketch for uChip showing functions that can be used with the RELAY SHIELD EXPANSION BOARD

  Demonstrates the use of the Relay Expansion Boards library for uChip.
  uChip can be powered either via USB or via external voltage VEXT (3.3V - 5V)
  When USB powered the external voltage VEXT is left to default (3.3V)

  This example code is in the public domain, more details can be found on the online guide at

  https://shop.itaca-innovation.com

  created by Antonio Rizzo <info@itaca-innovation.com>
  19 Apr 2020
*/
#include <ItacaExpansionShields.h>

void setup() {
  // Call initialization function of the library
  itacaRelayShieldInit();
}

#define CHECK_PERIOD 10000 // In milliseconds
uint32_t timestamp_ms = 0;

void loop() {

  // After the check period timedout we run the periodic switch of some channels
  if(millis() - timestamp_ms > CHECK_PERIOD)
  {
    timestamp_ms = millis();
    itacaRelayShieldSetCh(ITACA_RELAY_CH_A, !itacaRelayShieldGetCh(ITACA_RELAY_CH_A));
    itacaRelayShieldSetCh(ITACA_RELAY_CH_C, !itacaRelayShieldGetCh(ITACA_RELAY_CH_C));
    itacaRelayShieldSetCh(ITACA_RELAY_CH_E, !itacaRelayShieldGetCh(ITACA_RELAY_CH_E));
    itacaRelayShieldSetCh(ITACA_RELAY_CH_G, !itacaRelayShieldGetCh(ITACA_RELAY_CH_G));
    itacaRelayShieldSetCh(ITACA_RELAY_CH_I, !itacaRelayShieldGetCh(ITACA_RELAY_CH_I));
    itacaRelayShieldSetCh(ITACA_RELAY_CH_K, !itacaRelayShieldGetCh(ITACA_RELAY_CH_K));
  }

  // When pressing the switch P1 we turn all channels off
  if(itacaRelayShieldGetSw(ITACA_RELAY_SW_P1) == ITACA_RELAY_SW_PRESSED)
  {
    // Reset the relay using the reset function
    itacaRelayShieldResetAll();
    // We change the timestamp so that the check period starts from the button press
    timestamp_ms = millis();
  }

  // When pressing the switch P2 we turn all channels off
  if(itacaRelayShieldGetSw(ITACA_RELAY_SW_P2) == ITACA_RELAY_SW_PRESSED)
  {
    // Reset the relay using the set function
    itacaRelayShieldSetAll();
    // We change the timestamp so that the check period starts from the button press
    timestamp_ms = millis();
  }
}
