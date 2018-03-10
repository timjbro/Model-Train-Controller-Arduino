#include <MsTimer2.h>
// MIT License
// Copyright (c) 2018 Timothy Brook
// For more information check out the GitHub repository:
// https://github.com/timjbro/Model-Train-Controller-Arduino

// **********************************************************************************************************
// ***** These variables can be modified to suit the circuit, model railway scale and locomotives used. *****
// **********************************************************************************************************

// The Power, Reverser and Break lever controls are input through the ADC
const int powerControlPin = A0;
const int reverserControlPin = A1;
const int breakControlPin = A2;

// Timer2 is normally used for PWM on pins 3 and 11
// As we are repurposing Timer2 as an interrupt we can't use pin 3 or 11 for the PWM drive.
const int pwmDriverPin = 10;

// Constant values for the Speed algorithm.
const double adcMaxValue = 1024.0; // Set for Arduino Uno
const double pwmMinValue = 10.0; // Must be just large enough to stop the DC motor from stalling.
const double pwmMaxValue = 255.0; // Reduce this to limit the maximum speed of the locomotive.
const double accRatio = 0.0005; // The rate of acceleration & decelleration when NOT breaking.
const double decRate = 0.001; // This is the rate of breaking. Must be greater than the accRatio otherwise weird things happen!

// **********************************************************************************************************
// **********************************************************************************************************
// **********************************************************************************************************

// Power & Speed
double PowerLeverPosition = 0;
double TargetSpeed = 0;
volatile double ActualSpeed = 0;

// Reverser & Direction
enum ReverserStates { Reverse, Neutral, Forward };
ReverserStates ReverserLeverPosition = Neutral;
ReverserStates ActualDirection = Neutral;

// Break
enum BreakStates { Release, On1, On2, FullService, Emergency };
BreakStates BreakLeverPosition = FullService;

void dspRecalculateInterrupt ()
{
  if (ActualSpeed < pwmMinValue)
  {
    if (TargetSpeed == 0 || BreakLeverPosition != Release)
      ActualSpeed -= pwmMinValue / 10;
    else
      ActualSpeed += pwmMinValue / 10;
  }
  else
  {
    ActualSpeed = ActualSpeed * (1.0 - ((double)BreakLeverPosition * decRate));
    ActualSpeed = ActualSpeed * (1.0 - accRatio) + TargetSpeed * accRatio;
  }
  SetDirection();
  SetTargetSpeed();
  analogWrite(pwmDriverPin, (int)ActualSpeed);
}

void SetDirection()
{
  if (ActualSpeed <= 0)
  {
    ActualSpeed = 0;
    ActualDirection = ReverserLeverPosition;
  }
}

void SetTargetSpeed()
{ 
  if ((ReverserLeverPosition == Reverse && ActualDirection == Reverse)
  || (ReverserLeverPosition == Forward && ActualDirection == Forward))
  {
    double setSpeed = (PowerLeverPosition / adcMaxValue) * pwmMaxValue;
    if (setSpeed < pwmMinValue)
      TargetSpeed = 0;
    else
      TargetSpeed = setSpeed;
  }
  else
  {
    TargetSpeed = 0;
  }
}

void setup() {
  pinMode(powerControlPin, INPUT);
  pinMode(reverserControlPin, INPUT);
  pinMode(breakControlPin, INPUT);
  pinMode(pwmDriverPin, OUTPUT);
  MsTimer2::set(10, dspRecalculateInterrupt); // 10ms ISR period.
  MsTimer2::start();
}

void loop() {
  noInterrupts();
  // Power
  PowerLeverPosition = (double)analogRead(powerControlPin);
  // Reverser
  int reverserValue = analogRead(reverserControlPin);
  if (reverserValue < 200)
    ReverserLeverPosition = Reverse;
  else if (reverserValue > 800)
    ReverserLeverPosition = Forward;
  else
    ReverserLeverPosition = Neutral;
  // Break
  int breakValue = analogRead(breakControlPin);
  if (breakValue > 800)
    BreakLeverPosition = Emergency;
  else if (breakValue > 600)
    BreakLeverPosition = FullService;
  else if (breakValue > 400)
    BreakLeverPosition = On2;
  else if (breakValue > 200)
    BreakLeverPosition = On1;
  else
    BreakLeverPosition = Release;
  // Set properties
  SetDirection();
  SetTargetSpeed();
  interrupts();
  delay(100); // This delay is required as it allows time for the ISR to run.
}
