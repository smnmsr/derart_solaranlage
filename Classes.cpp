#include "Classes.h" //Header für diesed Libary

//____________________________________________
//Konstruktor der AnalogSensor Klasse
AnalogSensor::AnalogSensor(byte pin)
{
  _pin = pin;                                      //Pin, an dem Sensor angeschlossen ist
}

//Setzt den Pin bei einem AnalogSensor Objekt
void AnalogSensor::setPin(byte pin)
{
  _pin = pin;
}

//____________________________________________
//Konstruktr der AnalogSensor Subklasse PT1000
PT1000::PT1000(byte pin, int rVor = 1000) : AnalogSensor(pin)
{
  _rVor = rVor;                         //Vorwiderstand (Spannungsteiler)
  _RN = 1000;                           //Normwiderstand des PT1000 bei 0 °C
  for (size_t i = 0; i < 10; i++)
  {
    _temperatures[i] = 0; //Temperaturen (immer die letzten zehn)
  }
  _voltage = 0; //Spannung am Analogeingang
  _r = 0;       //Widerstand des PT1000
}

//Setzt den Vorwiderstand bei einem PT1000 Objekt
void PT1000::setRVor(int rVor)
{
  _rVor = rVor;
}

//Berechnet die Temperatur, die der PT1000 misst
void PT1000::calculateTemperature()
{
  _voltage = ((analogRead(_pin) + analogRead(_pin) + analogRead(_pin)) * _analogStep) / 3; //Spannung am Analog Eingang
  _r = (_rVor * _voltage) / (_boardVoltage - _voltage);                                    //Widerstand des PT1000 (Spannungsteiler)
  for (int i = 0; i < 9; i++)
  { //ältester Messwert löschen, andere einen Index nach hinten schieben
    _temperatures[9 - i] = _temperatures[8 - i];
  }
  _temperatures[0] = (-(_A * _RN) + sqrt(pow(_A * _RN, 2) - 4 * _B * _RN * (_RN - _r))) / (2 * _B * _RN); //Berechnet aus _R die Temperatur
}

//Berechnet Temperatur und diese zurück
float PT1000::getLastTemperature()
{
  return _temperatures[0];
}

//Berechnet Temperatur und gibt Mittelwert der letzten fünf Messungen zurück
float PT1000::getMeanTemperature()
{
  if (_temperatures[9] != 0)
  {
    return (_temperatures[0] + _temperatures[1] + _temperatures[2] + _temperatures[3] + _temperatures[4] + _temperatures[5] + _temperatures[6] + _temperatures[7] + _temperatures[8] + _temperatures[9]) / 10;
  }
  else
  {
    return _temperatures[0];
  }
}

//____________________________________________
//Konstruktr der Klasse Timer
Timer::Timer(unsigned long delay, char mode)
{
  _now = 0;      //Zeit der Abfrage
  _lastTime = 0; //Zeit der letzten Ausführung
  switch (mode)
  {
  case 's':
    _delay = delay * 1000;
    break;
  case 'm':
    _delay = delay * 60000;
    break;
  case 'h':
    _delay = delay * 3600000;
    break;
  case 'd':
    _delay = delay * 86400000;
    break;
  default:
    _delay = delay;
  }
}

//Nimmt aktuellen millis() Wert und Prüft, ob eine nächste Ausführung notwendig ist
bool Timer::checkTimer(unsigned long now)
{
  _now = now;
  if ((unsigned long)(_now - _lastTime) >= _delay)
  {
    return true;
  }
  else
    return false;
}

void Timer::executed()
{
  _lastTime = _now;
}

//Setzt die lastTime Variable
void Timer::setLastTime(unsigned long lastTime)
{
  _lastTime = lastTime;
}

//Setzt die Delay-Time
void Timer::setDelayTime(unsigned long delay, char mode)
{
  switch (mode)
  {
  case 's':
    _delay = delay * 1000;
    break;
  case 'm':
    _delay = delay * 60000;
    break;
  case 'h':
    _delay = delay * 3600000;
    break;
  case 'd':
    _delay = delay * 86400000;
    break;
  default:
    _delay = delay;
  }
}

//Gibt die Delay-Time zurück
long Timer::getDelayTime()
{
  return _delay;
}

//____________________________________________
//Konstruktor der Pumpe Klasse
Pump::Pump(byte relaisPin, byte PWMPin)
{
  _relaisPin = relaisPin;
  _PWMPin = PWMPin;
}

void Pump::setSpeed(int speed)
{
  if (speed >= 255)
  {
    speed = 255;
  }
  if (speed <= 0)
  {
    speed = 0;
  }

  analogWrite(_PWMPin, speed);
  if (speed > 0)
  {
    digitalWrite(_relaisPin, HIGH);
  }
}

void Pump::stop()
{
  analogWrite(_PWMPin, 0);
  digitalWrite(_relaisPin, LOW);
}