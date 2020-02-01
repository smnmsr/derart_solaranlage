// Kommentare zu Klassen in Classes.cpp

#ifndef Classes_h
#define Classes_h

#include "Arduino.h" //Arduino Standard Header
#include "Wire.h" //Für I2C Bus
#include "PID_v1.h" //Für PID-Regelung
#include "Adafruit_LiquidCrystal.h" //Für Displays

class AnalogSensor
{
  public:
    AnalogSensor(byte pin);
    void setPin(byte pin);
  protected:
    int _pin;
    float _boardVoltage;
    int _analogResolution;
    float _analogStep;
};

class PT1000 : public AnalogSensor
{
  public:
    PT1000(byte pin, int rVor, float correctionFactor);
    void setRVor(int rVor);
    void setCorrectionFactor(float correctionFactor);
    void calculateTemperature();
    float getLastTemperature();
    float getMeanTemperature();
  private:
    int _rVor;
    float _correctionFactor;
    float _A;
    float _B;
    int _RN;
    float _temperatures[5];
    float _voltage;
    float _r;
};

class Potentiometer : public AnalogSensor
{
  public:
    Potentiometer(byte pin, int lowerLimit, int upperLimit, bool reverse);
    float getValue();
  private:
    int _lowerLimit;
    int _upperLimit;
    float _val;
    bool _reverse;
};

class Timer {
  public:
    Timer(unsigned long delay);
    bool checkTimer(unsigned long now);
  private:
    unsigned long _now;
    unsigned long _lastTime;
    unsigned long _delay;
  };

#endif
