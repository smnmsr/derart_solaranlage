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
    void setBoardVoltage(float boardVoltage);
    void setAnalogResulution(int resulution);
  protected:
    byte _pin;
    float _boardVoltage;
    int _analogResolution;
    float _analogStep;
  }

class PT1000 : public AnalogSensor
{
  public:
    PT1000(byte pin, int rVor, float correctionFactor);
    void setRVor(int rVor);
    void setCorrectionFactor(float correctionFactor);
    void setCalculatingConstants(float A, float B);
    void calculateTemperature();
    float getLastTemperature();
    float getMeanTemperature();
  private:
    int _rVor;
    float _correctionFactor;
    float _A;
    float _B;
    float _temperatures[5];
    float _voltage;
    float _r;
  }

 class Potentiometer : public AnalogSensor
 {
  public:
    Potentiometer(byte pin, int rTot);
    void setRTot(int rTot);
    void SetLimits(int lowerLimit, intUpperLimit);
    float getValue();
  private:
    int _rTot;
    int _lowerLimit;
    int _upperLimit;
  }

#endif
