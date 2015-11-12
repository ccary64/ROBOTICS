
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

double lastNonZeroMotorPWMA = 0;
double lastNonZeroMotorPWMB = 0;

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
#define servopin    3    // servo pin

void encoderA() {
  encoderCountA++;
}

void encoderB() {
  encoderCountB++;
}

void setup() {
  // put your setup code here, to run once:

  // set pin modes
  pinMode( encapin, INPUT );
  pinMode( encbpin, INPUT );
  pinMode( dirapin, OUTPUT );
  pinMode( pwmapin, OUTPUT );
  pinMode( dirbpin, OUTPUT );
  pinMode( pwmbpin, OUTPUT );
  pinMode( voltpin, INPUT );
  pinMode( ledpin,  OUTPUT );
  pinMode( servopin, OUTPUT );
  
  // set motor speeds to zero
  analogWrite( pwmapin, 0 );
  analogWrite( pwmbpin, 0 );

  // enable motors
  digitalWrite( dirapin, HIGH );
  digitalWrite( dirbpin, HIGH );

  // attach encoder interrupts
  enableInterrupt( encapin, encoderA, CHANGE );
  enableInterrupt( encbpin, encoderB, CHANGE );
  
  // open serial port
  Serial.begin(9600);

  // set PID parameters
  pidA.SetSampleTime( sampleInterval );
  pidA.SetOutputLimits( -255, 255 );

  pidB.SetSampleTime( sampleInterval );
  pidB.SetOutputLimits( -255, 255 );

  // turn the PIDs on
  pidA.SetMode( AUTOMATIC );
  pidB.SetMode( AUTOMATIC );

  // record first timestamp
  previousTime = millis();
}

void loop() {
  // put your main code here, to run repeatedly:

  // check for serial input
  if ( Serial.available() )
  {
      // read setpoints from serial
      byte cmd[4];
      Serial.readBytes( (char*)cmd, 4 );
      pidSetpointA = cmd[1];
      if ( cmd[0] ) pidSetpointA = -pidSetpointA;
      pidSetpointB = cmd[3];
      if ( cmd[2] ) pidSetpointB = -pidSetpointB;
  }
  
  // check sample interval
  unsigned long currentTime = millis();

  if ( currentTime - previousTime >= sampleInterval )
  {
    // disable interrupts
    noInterrupts();
    
    // copy encoder counters to PID input
    pidInputA = (double)encoderCountA;
    pidInputB = (double)encoderCountB;
    
    // reset encoder counters
    encoderCountA = 0;
    encoderCountB = 0;

    // re-enable interrupts
    interrupts();

    // check motor direction
    if ( lastNonZeroMotorPWMA < 0 ) pidInputA = -pidInputA;
    if ( lastNonZeroMotorPWMB < 0 ) pidInputB = -pidInputB;

    // report encoder counts to serial
    byte data[4];
    data[0] = (pidInputA<0)?1:0;
    data[1] = abs(pidInputA);
    data[2] = (pidInputB<0)?1:0;
    data[3] = abs(pidInputB);
    Serial.write( data, 4 );
    
    // compute pid control update
    pidA.Compute();
    pidB.Compute();

    // apply pid control update
    motorPWMA += pidOutputA;
    motorPWMB += pidOutputB;

    // fix motor settings to PWM limits
    if ( motorPWMA < -255 ) motorPWMA = -255;
    if ( motorPWMA > 255 ) motorPWMA = 255;
    if ( motorPWMB < -255 ) motorPWMB = -255;
    if ( motorPWMB > 255 ) motorPWMB = 255;

    // turn off motors when setpoint is zero
    if ( pidSetpointA == 0 ) motorPWMA = 0;
    if ( pidSetpointB == 0 ) motorPWMB = 0;
    
    if ( motorPWMA != 0 ) lastNonZeroMotorPWMA = motorPWMA;
    if ( motorPWMB != 0 ) lastNonZeroMotorPWMB = motorPWMB;    
    
    // record sample time
    previousTime = currentTime;
  }
  
  // set motors
  digitalWrite( dirapin, (motorPWMA>0)?LOW:HIGH );
  digitalWrite( dirbpin, (motorPWMB>0)?LOW:HIGH );
  analogWrite( pwmapin, abs(motorPWMA) );
  analogWrite( pwmbpin, abs(motorPWMB) );
  
  analogWrite( servopin, 128 );
}

