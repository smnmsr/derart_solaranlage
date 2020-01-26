#include "Arduino.h" //Arduino Standard Header
#include "Wire.h" //Für I2C Bus
#include "PID_v1.h" //Für PID-Regelung
#include "Adafruit_LiquidCrystal.h" //Für Displays
#include "Classes.h" //Header für diesed Libary

AnalogSensor::AnalogSensor(byte pin) {
  _pin = pin;
  _boardVoltage = 5;
  _analogResolution = 1024;
  _analogStep = _boardVoltage / _analogResolution;
}

void AnalogSensor::setPin(byte pin) {
  _pin = pin;
}

void AnalogSensor::setBoardVoltage(float boardVoltage) {
  _boardVoltage = boardVoltage;
}
void AnalogSensor::setAnalogResulution(int resulution){
  _analogResolution = resolution;
  }

PT1000::PT1000(byte pin, int rVor = 1000, float correctionFactor = 1.0){
  _pin = pin;
  _rVor = rVor;
  _correctionFactor = correctionFactor;
  _A = 0.00390802;
  _B = -0.0000005802;
  _temperatures = {0,0,0,0,0}
  _voltage = 0;
  _r = 0;
  }
  
 void PT1000:setRVor(int rVor){
  _rVor = rVor;
  }
    void setCorrectionFactor(float correctionFactor){
      _correctionFactor = correctionFactor;
      }
    void setCalculatingConstants(float A, float B){
      _A = A;
      _B = B;
      }
    void calculateTemperature(){
      _voltage = AnalogRead(_pin) * _analogStep;
      _r = (_rVor * _voltage)/(_boardVoltage - _voltage);
      for (int i = 0; i<4; i++){
        _temperatures[4-i] = _temperatures[3-i];
        }
      _temperatures[0] = ()
      }
    float getLastTemperature();
    float getMeanTemperature();

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
