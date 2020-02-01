#include "Classes.h" //Header für diesed Libary

//____________________________________________
//Konstruktor der AnalogSensor Klasse
AnalogSensor::AnalogSensor(byte pin) {
  _pin = pin; //Pin, an dem Sensor angeschlossen ist
  _boardVoltage = 5; //Spannung des Boards
  _analogResolution = 1024; //Auflösung des Analogen Eingangs
  _analogStep = _boardVoltage / _analogResolution; //Spannung pro Step am Analogen Eingang
}

//Setzt den Pin bei einem AnalogSensor Objekt
void AnalogSensor::setPin(byte pin) {
  _pin = pin;
}

//____________________________________________
//Konstruktr der AnalogSensor Subklasse PT1000
PT1000::PT1000(byte pin, int rVor = 1000, float correctionFactor = 1.0) : AnalogSensor(pin) {
  _rVor = rVor; //Vorwiderstand (Spannungsteiler)
  _correctionFactor = correctionFactor; //Korrekturfaktor
  _A = 0.00390802; //Konstante für PT1000 Berechnung
  _B = -0.0000005802; //Konstante für PT1000 Berechnung
  _RN = 1000; //Normwiderstand des PT1000 bei 0 °C
  _temperatures[0] = 0; //Temperaturen (immer die letzten fünf)
  _temperatures[1] = 0;
  _temperatures[2] = 0;
  _temperatures[3] = 0;
  _temperatures[4] = 0;
  _voltage = 0; //Spannung am Analogeingang
  _r = 0; //Widerstand des PT1000
}

//Setzt den Vorwiderstand bei einem PT1000 Objekt
void PT1000::setRVor(int rVor) {
  _rVor = rVor;
}

//Setzt einen Korrekturfaktor bei einem PT1000 Objekt
void PT1000::setCorrectionFactor(float correctionFactor) {
  _correctionFactor = correctionFactor;
}

//Berechnet die Temperatur, die der PT1000 misst
void PT1000::calculateTemperature() {
  _voltage = analogRead(_pin) * _analogStep; //Spannung am Analog Eingang
  _r = (_rVor * _voltage) / (_boardVoltage - _voltage); //Widerstand des PT1000 (Spannungsteiler)
  for (int i = 0; i < 4; i++) { //ältester Messwert löschen, andere einen Index nach hinten schieben
    _temperatures[4 - i] = _temperatures[3 - i];
  }
  _temperatures[0] = (-(_A * _RN) + sqrt(pow(_A * _RN, 2) - 4 * _B * _RN * (_RN - _r))) / (2 * _B * _RN); //Berechnet aus _R die Temperatur
  _temperatures[0] *= _correctionFactor;
}

//Berechnet Temperatur und diese zurück
float PT1000::getLastTemperature() {
  return _temperatures[0];
}

//Berechnet Temperatur und gibt Mittelwert der letzten fünf Messungen zurück
float PT1000::getMeanTemperature() {
  return (_temperatures[0] + _temperatures[1] + _temperatures[2] + _temperatures[3] + _temperatures[4]) / 5;
}

//____________________________________________
//Konstruktor der Potentiometer Klasse
Potentiometer::Potentiometer(byte pin, int lowerLimit = 5, int upperLimit = 95, bool reverse = false) : AnalogSensor(pin) {
  _lowerLimit = lowerLimit; //erst wenn dieses Limit überschritten wird, steigt der Wert
  _upperLimit = upperLimit; //erst wenn dieses Limit unterschritten wird, sinkt der Wert
  _val = 0;
  _reverse = reverse;
}

//Liest Analog Eingang und gibt Potentiometer-Wert in  % zurück
float Potentiometer::getValue() {
  _val = ((analogRead(_pin) * _analogStep) / _boardVoltage) * 100; //Wert am Analogeingang in % der Bordspannung
  _val -= _lowerLimit;
  _val /= (_upperLimit - _lowerLimit) * 100;
  if (_val < 0) {
    _val = 0;
  } else if (_val > 100) {
    _val = 100;
  }
  if (_reverse) {
    _val = -1 * (_val - 100);
  }
  return _val;
}

//____________________________________________
//Konstruktr der Klasse Timer
Timer::Timer(unsigned long delay) {
  _now = 0; //Zeit der Abfrage
  _lastTime = 0; //Zeit der letzten Ausführung
  _delay = delay; //Zeitabstand zwischen Ausführungen
  }

//Nimmt aktuellen millis() Wert und Prüft, ob eine nächste Ausführung notwendig ist
bool Timer::checkTimer (unsigned long now){
  _now = now;
  if ((unsigned long)(_now - _lastTime) >= _delay){
    _lastTime = _now;
    return true;
    }
    else return false;
  }
