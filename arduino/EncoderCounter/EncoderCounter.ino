
#include <EnableInterrupt.h>

// sample timing
unsigned int sampleInterval = 200;
unsigned long previousTime = 0;

// encoder counters
volatile char encoderCountA = 0;
volatile char encoderCountB = 0;

#define encapin    A0    // Motor A encoder   sensor  pin
#define encbpin    A1    // Motor B encoder   sensor  pin
#define dirapin    10    // Motor A direction control pin
#define pwmapin     5    // Motor A speed     control pin
#define pwmbpin     6    // Motor B speed     control pin
#define dirbpin    11    // Motor B direction control pin
#define voltpin     7    // Battery monitor   sensor  pins
#define ledpin     13    // LED pin

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

  // set motor speeds to zero
  analogWrite( pwmapin, 0 );
  analogWrite( pwmbpin, 0 );

  // enable motors
  digitalWrite( dirapin, HIGH );
  digitalWrite( dirbpin, HIGH );

  // attach encoder interrupts
  enableInterrupt(encapin, encoderA, RISING );
  enableInterrupt(encbpin, encoderB, RISING );

  Serial.begin(9600);
  
  // record first timestamp
  previousTime = millis();
}

void loop() {
  // put your main code here, to run repeatedly:
  
  // check sample interval
  unsigned long currentTime = millis();

  if ( currentTime - previousTime >= sampleInterval )
  {
    // disable interrupts
    noInterrupts();
    
    // copy encoder counters
    char encoderCountAsave = encoderCountA;
    char encoderCountBsave = encoderCountB;
    
    // reset encoder counters
    encoderCountA = 0;
    encoderCountB = 0;

    // re-enable interrupts
    interrupts();

    // write to serial
    Serial.write(encoderCountAsave);
    Serial.write(encoderCountBsave);
    
    // record sample time
    previousTime = currentTime;

  }

  analogWrite( pwmapin, 0 );
  analogWrite( pwmbpin, 0 );
}

