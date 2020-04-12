// Kommentare zu Klassen in Classes.cpp

#ifndef Classes_h
#define Classes_h

#define _boardVoltage 5          //Spannung des Boards
#define _analogStep 0.0048828125 //Spannung pro Step am Analogen Eingang
#define _A 0.00390802            //Konstante für PT1000 Berechnung
#define _B -0.0000005802         //Konstante für PT1000 Berechnung

#include "Arduino.h" //Arduino Standard Header

class AnalogSensor
{
public:
  AnalogSensor(byte pin);
  void setPin(byte pin);

protected:
  byte _pin;
};

class PT1000 : public AnalogSensor
{
public:
  PT1000(byte pin, int rVor);
  void setRVor(int rVor);
  void calculateTemperature();
  float getLastTemperature();
  float getMeanTemperature();

private:
  int _rVor;
  int _RN;
  float _temperatures[10];
  float _voltage;
  float _r;
};

class Timer
{
public:
  Timer(unsigned long delay, char = 'x');
  bool checkTimer(unsigned long now);
  void executed();
  void setLastTime(unsigned long lastTime);
  void setDelayTime(unsigned long delay, char = 'x');
  long getDelayTime();

private:
  unsigned long _now;
  unsigned long _lastTime;
  unsigned long _delay;
};

class Pump
{
public:
  Pump(byte relaisPin, byte PWMPin);
  void setSpeed(int speed);
  void stop();

private:
  byte _relaisPin;
  byte _PWMPin;
};

#endif
