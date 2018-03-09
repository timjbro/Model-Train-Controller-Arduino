#include <MsTimer2.h>

const int powerControlPin = A0;
const int reverserControlPin = A1;
const int breakControlPin = A2;
const int pwmDriverPin = 13;

const double adcMaxValue = 1024.0;
const double pwmMinValue = 10.0;
const double pwmMaxValue = 255.0;
const double accRatio = 0.005;
const double decRate = 0.01;

double PowerLeverPosition = 0;
double TargetSpeed = 0;
double ActualSpeed = 0;

enum ReverserStates { Reverse, Neutral, Forward };
ReverserStates ReverserLeverPosition = Neutral;
ReverserStates ActualDirection = Neutral;

enum BreakStates { Release, On1, On2, FullService, Emergency };
BreakStates BreakLeverPosition = FullService;

void dspRecalculate ()
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
  analogWrite(pwmDriverPin, 0);
  MsTimer2::set(100, dspRecalculate); // 100ms period.
  MsTimer2::start();
}

void loop() {
  noInterrupts();
  // Power
  PowerLeverPosition = ((double)analogRead(powerControlPin) / adcMaxValue) * pwmMaxValue;
  // Reverser
  int reverserValue = analogRead(reverserControlPin);
  if (reverserValue < 200)
    ReverserLeverPosition = Reverse;
  else if (reverserValue > 800)
    ReverserLeverPosition = Forward;
  else
    ReverserLeverPosition = Neutral;
  // Break
  int breakValue = analogRead(reverserControlPin);
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
  delay(500);
}
