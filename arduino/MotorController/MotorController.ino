
#include <EnableInterrupt.h>

#include <Wire.h>

#include <PID_v1.h>

// sample timing
unsigned int sampleInterval = 200;
unsigned long previousTime = 0;

// encoder counters
volatile byte encoderCountA = 0;
volatile byte encoderCountB = 0;

// motor settings
double motorPWMA = 0;
double motorPWMB = 0;

// PID parameters
double KpA = 5;
double KiA = 0;
double KdA = 0;

double KpB = KpA;
double KiB = KiA;
double KdB = KdA;

// PID variables
double pidInputA = 0;
double pidOutputA = 0;
double pidSetpointA = 0;

double pidInputB = 0;
double pidOutputB = 0;
double pidSetpointB = 0;

// PID classes
PID pidA( &pidInputA, &pidOutputA, &pidSetpointA, KpA, KiA, KdA, DIRECT );
PID pidB( &pidInputB, &pidOutputB, &pidSetpointB, KpB, KiB, KdB, DIRECT );

#define encapin    A1    // Motor A encoder   sensor  pin
#define encbpin    A0    // Motor B encoder   sensor  pin
#define dirapin    11    // Motor A direction control pin
#define pwmapin     6    // Motor A speed     control pin
#define pwmbpin     5    // Motor B speed     control pin
#define dirbpin    10    // Motor B direction control pin
#define voltpin     7    // Battery monitor   sensor  pins
#define ledpin     13    // LED pin

void encoderA() {
    // increment left encoder counter

}

void encoderB() {
    // increment right encoder counter
}

void setup() {
    
  // set pin modes
  
  // initialize motor speeds to zero

  // initialize motor directions

  // attach encoder interrupts (NB: use EnableInterrupt, not the Arduino built-in interrupts)
  
  // open serial port at 9600 baud

  // set PID sample time and output limits

  // turn the PIDs on (set mode to automatic)

  // record first timestamp in previousTime
}

void loop() {

  // check for serial input (4 bytes: left motor direction, left motor speed, right motor direction, right motor speed)
    
  // if sample period has elapsed
    
    // disable interrupts
    
    // copy encoder counters to PID input
    
    // reset encoder counters

    // re-enable interrupts

    // report encoder counts to serial
    
    // compute pid control update

    // apply pid control update
  
  // set motor directions and speeds
}

