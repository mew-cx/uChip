/*
  Simple sketch for uChip showing how to make a Sonar acting like parking sensors.

  Demonstrates how to use sonar to make a parking sensor.
  uChip can be powered either via USB or via external voltage VEXT (3.3V - 5V)
  When USB powered the external voltage VEXT is left to default (3.3V)

  This example code is in the public domain, more details can be found on the online guide at

  https://shop.itaca-innovation.com

  created by Antonio Rizzo <info@itaca-innovation.com>
  24 Apr 2020
*/
#include <uChipPowerManagement.h>

// When PASSIVE_BUZZER is defined the analogWrite function
// is used to turn ON/OFF continuously the BUZZER on a digital pin
// Otherwise with ACTIVE_BUFFER the standard digitalWrite is used.
//#define PASSIVE_BUZZER


// Define required to use HC-SR04
#define PIN_TRIGGER 10
#define PIN_ECHO 9
// Define PIN to control the BUZZER for beeping
#define PIN_BUZZER 1

#ifdef PASSIVE_BUZZER
    // Parameters used to change the BUZZER beeping
    // These parameters are used to set how long the PWM
    // should keep the BUZZER on duty.
    // analogWrite is used because passive buzzer requires
    // continuously ON/OFF switching and analogWrite on digital enabled
    // pins applies a PWM with duty cycle proportional to the set value.
    // When using an active BUZZER just apply the digitalWrite command instead
    #define BUZZ_MAX 127
    #define BUZZ_MIN 0
#else
    #define BUZZ_MAX 1
    #define BUZZ_MIN 0
#endif

// Max and min distance in mm for beeping
#define SPACE_MAX 2500
#define SPACE_MIN 200
#define SPACE_DIV 25

// Variable necessary for ultrasonic sensor
uint32_t pulse_duration, space, delay_space;
// Require to calculate the passed time
long millis_old, millis_new;
// Tells the buzzer beeper if it is in HIGH (on) or LOW (off) state
boolean buzz_state = false;

void setup() {
  // Disabling the boost since the variable was initialized so.
  uChipEnableBoost(UCHIP_BOOST_DISABLED);
  // Setting the external voltage to VUSB
  // The HC-SR04 requires voltage higher than 3.6V
  // and beeps are stronger! :)
  uChipSetVextValue(VEXT_USB);
  // Setting up the LED port
  pinMode(LED_BUILTIN, OUTPUT);
  
  
  // Turn led on, it tells us that the micro is working
  digitalWrite(LED_BUILTIN, HIGH);
  // HC-SR04 PINs
  pinMode(PIN_TRIGGER, OUTPUT); // Sets the trigPin as an Output
  pinMode(PIN_ECHO, INPUT); // Sets the echoPin as an Input
  // BUZZER PIN
  pinMode(PIN_BUZZER, OUTPUT);
  // Turn off buzzer
  digitalWrite(PIN_BUZZER, LOW);
  // Save the current millis() value
  millis_old = millis();
}





// Main Loop
void loop() {
  // Call the function that updates the distance measured by sonar
  // This function is called continuously in the loop
  spaceUpdate();
 
  // Under Space max meter start buzzing otherwise stop
  if (space > SPACE_MAX) // Turn off buzzer
 #ifdef PASSIVE_BUZZER
    analogWrite(PIN_BUZZER, 0);
 #else
    digitalWrite(PIN_BUZZER, BUZZ_MIN);
 #endif
  else
  {
    // Under Threshold go continuosly
    if (space < SPACE_MIN) // Set Max
   #ifdef PASSIVE_BUZZER
      analogWrite(PIN_BUZZER, BUZZ_MAX);
   #else
      digitalWrite(PIN_BUZZER, BUZZ_MAX);
   #endif
    else
    // Create a pulsed buzz whose frequency is related to the distance
    // Uses millis() - millis_old to check when it is time to switch
    {
      millis_new = millis();
      delay_space = space/SPACE_DIV;
      if(millis_new - millis_old > delay_space)
      {
        millis_old = millis_new;
        if(buzz_state)
#ifdef PASSIVE_BUZZER
          analogWrite(PIN_BUZZER, BUZZ_MAX);
#else
          digitalWrite(PIN_BUZZER, BUZZ_MAX);
#endif
        else
#ifdef PASSIVE_BUZZER
          analogWrite(PIN_BUZZER, BUZZ_MIN);
#else
          digitalWrite(PIN_BUZZER, BUZZ_MIN);
#endif
        buzz_state = !buzz_state;  
      }
    }
  } 
}


void spaceUpdate()
{
  // Clear PIN_TRIGGER
  digitalWrite(PIN_TRIGGER, LOW);
  delayMicroseconds(2);
  // Set PIN_TRIGGER for 10 micro seconds then clear
  digitalWrite(PIN_TRIGGER, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_TRIGGER, LOW);
  // Reads PIN_ECHO in micro
  pulse_duration = pulseIn(PIN_ECHO, HIGH);
  // Calculate distance in mm s = v*t conversion from m to mm and s to us
  // 340 [m/s] is the sound velocity in air
  // 2 is due to the fact that the time measured is for the sound
  // to both go forward and backward
  // 1000 is the mm/us constant
  space = pulse_duration*340/2/1000;
}
