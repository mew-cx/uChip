/*
  Sketch for uChip showing the features which can be controlled using the Motor Shield Expansion Board

  Demonstrates the use of the Motor Shield Expansion Board library for uChip.
  The Motor Shield can be powered externally either at stadard VEXT voltage (3.3V - 5V)
  or Higher Voltage (up to 12V). Read carefully the manual and schematics before connecting power!

  Read the following code in details to find out how to set your Motor Shield board.

  This example code is in the public domain, more details can be found on the online guide at

  https://shop.itaca-innovation.com

  created by Antonio Rizzo <info@itaca-innovation.com>
  19 Apr 2020
*/

#include <ItacaExpansionShields.h>
#include <uChipPowerManagement.h>
#include <Servo.h>

// Program defines

// Comment/uncomment the right define accordingly to the mode you soldered your motor shield board
//////////////////////////////////
#define MOTOR_SHIELD_MODE ITACA_HV_MODE
//#define MOTOR_SHIELD_MODE ITACA_LV_MODE
//////////////////////////////////


// R614X 2.4GHz radio defines
#define NUM_CH     6
#define NUM_ACQ  2*NUM_CH + 1
// This define is just to remember where the Capture Compare is enabled
//#define PIN_CPPM  9
#define SYNC_VAL  10000
#define MIN_RANGE   750
#define MAX_RANGE  1650
#define MID_RANGE  1210
#define HEADROOM     10

// Servo values common define
#define MIN_SERVO_VALUE 5
#define MAX_SERVO_VALUE 175
// Motor values common define
#define MIN_MOTOR_VALUE 0
#define MAX_MOTOR_VALUE 255

//////////////////////////////////////////////////////////////
// With these defines you can enable/disable the features   //
//////////////////////////////////////////////////////////////
// DEBUG: To print out values uncomment
//#define DEBUG
//#define EMULATE_RX
#define SERVO_1_EN
#define SERVO_2_EN
#define SERVO_3_EN
#define MOTOR_1_EN
#define MOTOR_2_EN
#define MOTOR_FB_EN







#ifdef DEBUG
//  #define PRINT_ALL
//  #define PIN_TEST 12
  #define BAUD_RATE 115200
  static uint8_t debug_analog = 1;
#endif

// Defines used by the R614X 2.4GHz receiver or emulator

#ifndef EMULATE_RX
  // Internal define used to measure the PPM pulses using the timer in interrupt
  #define F_CLK 1
#else
  // Delay applied between each change in the output value
  #define EMULATE_DELAY 2500
#endif

// Variable declaration: volatile because they are changed in the interrupt or by the emulator
static volatile uint16_t cap0[NUM_ACQ];     // Period with PPW capture
static volatile uint16_t cap1[NUM_ACQ];     // Pulse width with PPW capture
static volatile uint16_t nCap;                // Counter used in the interrupt to save array cap0/1
static volatile bool capture_done = false;  // Flag signaling that a whole array of cap0/1 is ready
static uint16_t ch[NUM_CH];                 // Array containing the channels values

// In this section, depending on which feature was enabled, we declare required variables
#ifdef SERVO_1_EN
  #define PIN_SERVO_1   10
  Servo servo_1;
  int servo1_value = 90; 
#endif

#ifdef SERVO_2_EN
  #define PIN_SERVO_2   11
  Servo servo_2;
  int servo2_value = 90; 
#endif

#ifdef SERVO_3_EN
  #define PIN_SERVO_3   15
  Servo servo_3;
  int servo3_value = 90; 
#endif

#ifdef MOTOR_1_EN
  #define PIN_MOTOR_1    1
  int motor1_value = 0; 
#endif

#ifdef MOTOR_2_EN
  #define PIN_MOTOR_2    6
  int motor2_value = 0; 
#endif

#ifdef MOTOR_FB_EN
  #define PIN_FBM_A_H   12
  #define PIN_FBM_A_L   13
  #define PIN_FBM_B_H    2
  #define PIN_FBM_B_L    4
  int motorFB_value = 0;
#endif

// Function declaration to be used later by the main loop.
// These function implementation is below in the code.
int mapValue(int ch_value, int min_range, int max_range, int max_value, int min_value);
int fitChToValue(uint16_t ch_value, uint16_t min_range, uint16_t max_range, uint16_t max_value, uint16_t min_value);

// This variable saves the value we relate to the gas or throttle
int throttle_value = 0;


void setup() {
  // Run the setting routine
  set_uChip();
  // Add delay after each init to prevent absorbtion from undesired sources
#ifdef SERVO_1_EN
  servo_1.write(servo1_value);
  // Attach to the desired PIN
  servo_1.attach(PIN_SERVO_1);
  delay(500);
#endif
#ifdef SERVO_2_EN
  servo_2.write(servo2_value);
  // Allow time for the servo to update its position
  servo_2.attach(PIN_SERVO_2);
  // Allow time for the servo to update its position
  delay(500);
#endif
#ifdef SERVO_3_EN
  servo_3.write(servo3_value);
  // Allow time for the servo to update its position
  servo_3.attach(PIN_SERVO_3);
  // Allow time for the servo to update its position
  delay(500);
#endif
#ifdef MOTOR_1_EN
  pinMode(PIN_MOTOR_1, OUTPUT);
  analogWrite(PIN_MOTOR_1, 0);
#endif  
#ifdef MOTOR_2_EN
  pinMode(PIN_MOTOR_2, OUTPUT);
  analogWrite(PIN_MOTOR_2, 0);
#endif
#ifdef MOTOR_FB_EN
  pinMode(PIN_FBM_A_H, OUTPUT);
  pinMode(PIN_FBM_B_H, OUTPUT);
  digitalWrite(PIN_FBM_A_H, LOW);
  digitalWrite(PIN_FBM_B_H, LOW);
  analogWrite(PIN_FBM_A_L, 0);
  analogWrite(PIN_FBM_B_L, 0);
#endif
#ifdef EMULATE_RX
  for(int i = 0; i<NUM_CH; i++)
  {ch[i] = 1210;}
#else
  set_TTC();
#endif
}

void loop() {
#ifdef EMULATE_RX
  // Run continuosly if we are using the emulator
  if(true) 
#else
  // Run only when there is a new capture done
  if(capture_done)
#endif
  {
#ifdef EMULATE_RX
    // Assign a random value to the channel
    for(int i = 0; i<NUM_CH; i++)
    {
      ch[i] = (uint16_t) random(MIN_RANGE, MAX_RANGE);
    }
#else
    // Stop interrupts and then update the channels
    noInterrupts();
    update_ch();
#endif
#ifndef EMULATE_RX
    capture_done = false;  
    interrupts();
#endif



////////////////////////////////////////////////////////////////////////   
// INFO: CUSTOM SECTION                                               //
//       INSERT HERE THE CODE TO HANDLE CHANNELS!!                    //
//       In ch[i] you will find the channels values (EMULATED or REAL)//
////////////////////////////////////////////////////////////////////////  

    // Translate each channel value into the desired output value
    // Servo values
#ifdef SERVO_1_EN
    // Ch 0 sets the servo direction, with direct connection. In my case I also need to reverse the direction of the servo
    servo1_value = fitChToValue(ch[0], MIN_RANGE + HEADROOM, MAX_RANGE - HEADROOM, MAX_SERVO_VALUE, MIN_SERVO_VALUE);
#endif
#ifdef SERVO_2_EN
    // Ch 4 sets the servo direction, with direct connection. In my case I also need to reverse the direction of the servo
    servo2_value = fitChToValue(ch[4], MIN_RANGE + HEADROOM, MAX_RANGE - HEADROOM, MAX_SERVO_VALUE, MIN_SERVO_VALUE);
#endif
#ifdef SERVO_3_EN
    // Ch 5 sets the servo direction, with direct connection. In my case I also need to reverse the direction of the servo
    servo3_value = fitChToValue(ch[5], MIN_RANGE + HEADROOM, MAX_RANGE - HEADROOM, MAX_SERVO_VALUE, MIN_SERVO_VALUE);
#endif
#ifdef MOTOR_1_EN
    // Ch 3 sets the motor power.
    motor1_value = fitChToValue(ch[3], MIN_RANGE + HEADROOM, MAX_RANGE - HEADROOM, MAX_MOTOR_VALUE, MIN_MOTOR_VALUE);
#endif
#ifdef MOTOR_2_EN
    // Ch 1 sets the motor power.
    motor2_value = fitChToValue(ch[1], MIN_RANGE + HEADROOM, MAX_RANGE - HEADROOM, MAX_MOTOR_VALUE, MIN_MOTOR_VALUE);
#endif

    // Ch[2] sets the motorFB direction and power.
    throttle_value = (int) ch[2];
#ifdef MOTOR_FB_EN    
    if((throttle_value >= MID_RANGE - HEADROOM) &&  (throttle_value <= MID_RANGE + HEADROOM)) // then left everything off
      motorFB_value = 0;
    else
    {
      if(throttle_value > MID_RANGE + HEADROOM)
      {
        motorFB_value = mapValue(throttle_value, MID_RANGE + HEADROOM, MAX_RANGE - HEADROOM, MAX_MOTOR_VALUE, MIN_MOTOR_VALUE);
      }
      else
      { 
        motorFB_value = mapValue(throttle_value, MIN_RANGE + HEADROOM, MID_RANGE - HEADROOM, MAX_MOTOR_VALUE, MIN_MOTOR_VALUE) - MAX_MOTOR_VALUE;
      }
    }
#endif

////////////////////////////////////////////////////////////////////////   
// INFO: END CUSTOM SECTION                                        //
//////////////////////////////////////////////////////////////////////// 


  
#ifdef EMULATE_RX  
    delay(EMULATE_DELAY);
#endif
  // NB: Update must be done when interrupts are enabled!
  //     Otherwise expect noisy behaviour
  update_output_values();
  debug();
  }
}


// WARNING!! The off-time delay is necessary to make sure
// that the H-Bridge are not shorted during switching.
// LOWER THIS VALUE AT YOUR OWN RISK!! 
// THE WORST MIGHT HAPPEN (BURNING THE MOS)!!
#define OFF_DELAY_HV_MODE_MS 20

// Function used to drive the Full Bridge motor
static void motorFBwrite()
{
	static int timestamp = 0;						 
#ifdef MOTOR_FB_EN
  #if (MOTOR_SHIELD_MODE == HV_MODE)
    // A delay is necessary in order to prevent short-circuiting
    // Half of the bridge
    if(millis()-timestamp > OFF_DELAY_HV_MODE_MS)
    {
        timestamp = millis();  
    }
    else
    {
        return;  
    } 
  #endif
  if(motorFB_value == 0)
  {
    digitalWrite(PIN_FBM_A_H, LOW);
    digitalWrite(PIN_FBM_B_H, LOW);
    analogWrite(PIN_FBM_A_L, 0);
    analogWrite(PIN_FBM_B_L, 0);
						
   
    return;
  }
  // We keep the pMOSs always on and apply pwm on the nMOSs side
  if(motorFB_value > 0) // forward
  {
    // First make sure that the other side is off by reading the pin
    if(digitalRead(PIN_FBM_A_H) == LOW)
    {
        digitalWrite(PIN_FBM_B_H, HIGH);
        analogWrite(PIN_FBM_A_L, motorFB_value);
    }
    else
    {
        // Probably didn't passed throught he dead zone..we need to go there first
        digitalWrite(PIN_FBM_A_H, LOW);
        digitalWrite(PIN_FBM_B_H, LOW);
        analogWrite(PIN_FBM_A_L, 0);
        analogWrite(PIN_FBM_B_L, 0);
    }
  }
  else // reverse
  {
    // First make sure that the other side is off by reading the pin
    if(digitalRead(PIN_FBM_B_H) == LOW)
    {
        digitalWrite(PIN_FBM_A_H, HIGH);
        analogWrite(PIN_FBM_B_L, -motorFB_value);
    }
    else
    {
        // Probably didn't passed throught he dead zone..we need to go there first
        digitalWrite(PIN_FBM_A_H, LOW);
        digitalWrite(PIN_FBM_B_H, LOW);
        analogWrite(PIN_FBM_A_L, 0);
        analogWrite(PIN_FBM_B_L, 0);
    }
  }
#endif
}


// Interrupt handler from the capture compare routine
void TC3_Handler()
{
  if (TC3->COUNT16.INTFLAG.bit.MC0) 
  {
    REG_TC3_READREQ = TC_READREQ_RREQ |           // Enable a read request
                      TC_READREQ_ADDR(0x18);      // Offset address of the CC0 register
    while (TC3->COUNT16.STATUS.bit.SYNCBUSY);     // Wait for (read) synchronization
    cap0[nCap] = REG_TC3_COUNT16_CC0;              // Copy the period
    // When we get more than NUM_ACQ acquire, we can analyze the resulting data
    if (++nCap == NUM_ACQ) 
    {
      nCap = 0;
      capture_done = true;
    }
  }
    // Check for match counter 1 (MC1) interrupt
  if (TC3->COUNT16.INTFLAG.bit.MC1)           
  {
    REG_TC3_READREQ = TC_READREQ_RREQ |           // Enable a read request
                      TC_READREQ_ADDR(0x1A);      // Offset address of the CC1 register
    while (TC3->COUNT16.STATUS.bit.SYNCBUSY);     // Wait for (read) synchronization
    cap1[nCap] = REG_TC3_COUNT16_CC1;          // Copy the pulse-width
  }
}


// Static function for setting up uchip at startup
static void set_uChip(void)
{
  // Setting uChip default configuration
  // Turn on bypass pMos, this feature is not necessary in case you have a simple diode
#if (MOTOR_SHIELD_MODE == ITACA_HV_MODE)
  itacaMotorShieldSetBypass(ITACA_BYPASS_DE);
#else
  itacaMotorShieldSetBypass(ITACA_BYPASS_EN);
#endif
  // Disabling the boost since the variable was initialized so.
  uChipEnableBoost(UCHIP_BOOST_DISABLED);
  // Setting the external voltage to 5V in case we power from USB
  uChipSetVextValue(VEXT_USB);
  // Turn led off
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
    // Setting pins for program
#ifdef DEBUG
  #ifdef PIN_TEST
    pinMode(PIN_TEST, OUTPUT);
    // Set initial default value
    analogWrite(PIN_TEST, debug_analog);
  #endif
  SerialUSB.begin(BAUD_RATE);
  while(!SerialUSB);
  // Turn LED on to signal that the serial is open
  digitalWrite(LED_BUILTIN,  HIGH);
#endif
}


#ifndef EMULATE_RX
  // This static function sets up the timer to detect the pulses from the radio
  static void set_TTC(void)
  {
  #ifdef DEBUG
    SerialUSB.println("Setting up the TC...");
    SerialUSB.println("..turning on PM..");
  #endif
    REG_PM_APBCMASK |= PM_APBCMASK_EVSYS;     // Switch on the event system peripheral
  #ifdef DEBUG
    SerialUSB.println("..setting the clock generator source..");
  #endif
    REG_GCLK_GENDIV = GCLK_GENDIV_DIV(3) |    // Divide the 48MHz system clock by 3 = 16MHz
                      GCLK_GENDIV_ID(5);      // Set division on Generic Clock Generator (GCLK) 5
    while (GCLK->STATUS.bit.SYNCBUSY);        // Wait for synchronization
  
    REG_GCLK_GENCTRL = GCLK_GENCTRL_IDC |           // Set the duty cycle to 50/50 HIGH/LOW
                       GCLK_GENCTRL_GENEN |         // Enable GCLK 5
                       GCLK_GENCTRL_SRC_DFLL48M |   // Set the clock source to 48MHz
                       GCLK_GENCTRL_ID(5);          // Set clock source on GCLK 5
    while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization*/
  
    
    REG_GCLK_CLKCTRL = GCLK_CLKCTRL_CLKEN |         // Enable the generic clock...
                       GCLK_CLKCTRL_GEN_GCLK5 |     // ....on GCLK5
                       GCLK_CLKCTRL_ID_TCC2_TC3;    // Feed the GCLK5 to TCC2 and TC3
    while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization
      
  #ifdef DEBUG
    SerialUSB.println("..setting the input pin connected to CPPM..");
  #endif
    // Input on PA19
    PORT->Group[0].PMUX[19/2].bit.PMUXO = MUX_PA19A_EIC_EXTINT3;
    // Set pin to input mode with pull-up resistor enabled
    PORT->Group[0].PINCFG[19].reg=(uint8_t)(PORT_PINCFG_INEN|PORT_PINCFG_PULLEN|PORT_PINCFG_PMUXEN);
  #ifdef DEBUG
    SerialUSB.println("..the pulsewidth corresponds to the LOW time..");
  #endif
    EIC->CONFIG[0].bit.SENSE3 = EIC_CONFIG_SENSE3_HIGH_Val; // None of RISE,FALL or BOTH work for this
    EIC->EVCTRL.bit.EXTINTEO3 = 1;
    
    // The EIC interrupt is only for verifying the input
    //REG_EIC_INTENSET = EIC_INTFLAG_EXTINT3;
    //NVIC_EnableIRQ(EIC_IRQn);
    while (EIC->STATUS.reg & EIC_STATUS_SYNCBUSY);
    EIC->CTRL.bit.ENABLE = 1;
    while (EIC->STATUS.reg & EIC_STATUS_SYNCBUSY);
  
  
  #ifdef DEBUG
    SerialUSB.println("..setting TC3 interrupt capture..");
    SerialUSB.println("..giving power to the peripherial..");
  #endif
  
    REG_EVSYS_USER = EVSYS_USER_CHANNEL(1) |                                // Attach the event user (receiver) to channel 0 (n + 1)
                     EVSYS_USER_USER(EVSYS_ID_USER_TC3_EVU);                // Set the event user (receiver) as timer TC3
  
    REG_EVSYS_CHANNEL = EVSYS_CHANNEL_EDGSEL_NO_EVT_OUTPUT |                // No event edge detection
                        EVSYS_CHANNEL_PATH_ASYNCHRONOUS |                   // Set event path as asynchronous
                        EVSYS_CHANNEL_EVGEN(EVSYS_ID_GEN_EIC_EXTINT_3) |    // Set event generator (sender) as external interrupt 3
                        EVSYS_CHANNEL_CHANNEL(0);                           // Attach the generator (sender) to channel 0
  #ifdef DEBUG
    SerialUSB.println("..enable, set the clock generator and assign the clock..");
  #endif
    REG_TC3_EVCTRL |= TC_EVCTRL_TCEI |              // Enable the TC event input
                      /*TC_EVCTRL_TCINV |*/         // Invert the event input
                      TC_EVCTRL_EVACT_PPW;          // Set up the timer for capture: CC0 period, CC1 pulsewidth
  
    REG_TC3_READREQ = TC_READREQ_RREQ |             // Enable a read request
                      TC_READREQ_ADDR(0x06);        // Offset of the CTRLC register
    while (TC3->COUNT16.STATUS.bit.SYNCBUSY);       // Wait for (read) synchronization
    REG_TC3_CTRLC |= TC_CTRLC_CPTEN1 |              // Enable capture on CC1
                     TC_CTRLC_CPTEN0;               // Enable capture on CC0
    while (TC3->COUNT16.STATUS.bit.SYNCBUSY);       // Wait for (write) synchronization
     
    ///////////////////////////////////////////////////////////////////////////////////////////////////////
  
   
  #ifdef DEBUG
    SerialUSB.println("..setting interrupts for capture mode..");
  #endif
    NVIC_SetPriority(TC3_IRQn, 0);      // Set the Nested Vector Interrupt Controller (NVIC) priority for TC3 to 0 (highest)
    NVIC_EnableIRQ(TC3_IRQn);           // Connect the TC3 timer to the Nested Vector Interrupt Controller (NVIC)
   
    REG_TC3_INTENSET = TC_INTENSET_MC1 |            // Enable compare channel 1 (CC1) interrupts
                       TC_INTENSET_MC0;             // Enable compare channel 0 (CC0) interrupts
  #ifdef DEBUG
    SerialUSB.println("..enable TC..");
  #endif 
  // Enable TCC
    REG_TC3_CTRLA |= TC_CTRLA_PRESCALER_DIV16 |     // Set prescaler to 16, 16MHz/16 = 1MHz
                     TC_CTRLA_ENABLE;               // Enable TC3
    while (TC3->COUNT16.STATUS.bit.SYNCBUSY);       // Wait for synchronization
  #ifdef DEBUG
    SerialUSB.println("..finished TC3 setting procedure.");
  #endif 
  }

  // Static function used to update the channels based on the captured pulses
  static void update_ch(void)
  {
      // Check in the array where there are sync data, the first marks the beginning of recording, the last the ending
      static uint8_t counter;
      counter = 0;
      for(uint8_t i = 0; i<NUM_ACQ; i++)
      {
        if(cap1[i] > SYNC_VAL)
        {
          counter = 0;
        }
        else
        {
          ch[counter++] = cap1[i]/F_CLK;  
        }
        if(counter == NUM_CH)
          break;
      }  
  }
#endif

int mapValue(int ch_value, int min_range, int max_range, int max_value, int min_value)
{
  int value = -1;
  if(ch_value < min_range)
      value = min_value;
  else 
  {
    if(ch_value > max_range)
      value = max_value;
    else
      value = (ch_value - min_range)*(max_value - min_value)/(max_range - min_range) + min_value;
  }
  return value;
}


int fitChToValue(uint16_t ch_value, uint16_t min_range, uint16_t max_range, uint16_t max_value, uint16_t min_value)
{
  int value = -1;
  if(ch_value < min_range)
      value = min_value;
  else 
  {
    if(ch_value > max_range)
      value = max_value;
    else
      value = (ch_value - min_range)*(max_value - min_value)/(max_range - min_range) + min_value;
  }
  return value;
}

static void debug(void)
{
#ifdef DEBUG
  #ifdef SERVO_1_EN
     SerialUSB.print("Servo_1 = ");
     SerialUSB.println(servo1_value);
  #endif
  #ifdef SERVO_2_EN
     SerialUSB.print("Servo_2 = ");
     SerialUSB.println(servo2_value);
  #endif
  #ifdef SERVO_3_EN
     SerialUSB.print("Servo_3 = ");
     SerialUSB.println(servo3_value);
  #endif
  #ifdef MOTOR_1
     SerialUSB.print("Motor_1 = ");
     SerialUSB.println(motor1_value);
  #endif
  #ifdef MOTOR_2
     SerialUSB.print("Motor_2 = ");
     SerialUSB.println(motor2_value);
  #endif
  #ifdef MOTOR_FB_EN
     SerialUSB.print("Motor_FB = ");
     SerialUSB.println(motorFB_value);
  #endif
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  #ifdef PIN_TEST
    if(debug_analog == 0)
      debug_analog++;
    analogWrite(PIN_TEST,debug_analog++);
  #endif
  #ifdef PRINT_ALL
    for(uint16_t i = 0; i<NUM_ACQ; i++)
    {
      SerialUSB.print("#");
      SerialUSB.print(i);
      SerialUSB.print(F("   "));
      SerialUSB.print(cap0[i]/F_CLK);                      // Output the results
      SerialUSB.print(F("   "));
      SerialUSB.println(cap1[i]/F_CLK); 
     
    }
  #endif
    for(uint8_t i = 0; i < NUM_CH; i++)
    {
      SerialUSB.print("Ch #");
      SerialUSB.print(i);
      SerialUSB.print(" = ");
      SerialUSB.println(ch[i]);                      // Output the results
    }
    SerialUSB.println("/////////////////////////////////////////////////////////////////////////////////////");
#endif
}

// Update all outputs accordingly to the selected features
static void update_output_values(void)
{
#ifdef SERVO_1_EN
    servo_1.write(servo1_value); 
#endif

#ifdef SERVO_2_EN
    servo_2.write(servo2_value); 
#endif

#ifdef SERVO_3_EN
    servo_3.write(servo3_value); 
#endif

#ifdef MOTOR_FB_EN
    motorFBwrite();
#endif
#ifdef MOTOR_1_EN
    analogWrite(PIN_MOTOR_1, motor1_value);
#endif
#ifdef MOTOR_2_EN
    analogWrite(PIN_MOTOR_2, motor2_value);
#endif
}
