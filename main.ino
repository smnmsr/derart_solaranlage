// Software für die Funktionen gemäss README-Datei
//
// STRUKTUR:
//  1. HEADER-Dateien, Definitionen und Libaries
//  2. Variabeln und Konstanten
//  3. PIN-Adressen und BUS-Adressierung
//  4. Initialisierungen
//  5. Funktionen
//  6. Setup Sequenz
//  7. Programmschleife

// ============================================
// 1. HEADER-Dateien, Definitionen und Libaries
// ============================================
#include "PID_v1.h"            //Für PID-Regelung
#include "Classes.h"           //Eigene Klassen
#include <SPI.h>               //Ethernet
#include <Ethernet.h>          //Ethernet
#include <EthernetUdp.h>       //EthernetUdp
#include <NTPClient.h>         //NTP-Libary
#include <ArduinoMqttClient.h> //MQTT
#include "secrets.h"           //MQTT Passwörter
#include <Wire.h>              // I2C-Libary
#include <Adafruit_Sensor.h>   // Adafruit i2c sensors
#include "Adafruit_TSL2591.h"  // LUX-Meter Libary

// ===========================
// 2. Variabeln und Konstanten
// ===========================
// Konstanten
// Temperaturen
const byte ALARMTEMPERATUR_KOLLEKTOR_LUFT = 90; // Alarmtemperatur für Kollektor (Lufttemperatur)
const byte ALARMTEMPERATUR_KOLLEKTOR_VL = 95;   // Alarmtemperatur für Kollektor (Vorlauftemperatur)
const byte ALARMTEMPERATUR_SOLE = 32;           // Alarmtemperatur für Sole-Pumpe
const byte CRITICAL_TEMPERATURE_SOLE = 30;      // Kritische Sole-Temperatur
const byte MIN_DIFFERENZ_NACH_ALARM = 3;        // erst wenn die Temperatur um diesen Wert gesunken ist, geht der Alarmmodus aus
const byte SOLE_EXIT_TEMPERATURE = 6;           // Temperatur zur Sonde, bei der der Solemodus abgebrochen wird
const byte SOLE_START_TEMPERATURE = 25;         // Temperatur im Kollektor (Luft), bei der der Solemodus gestartet wird
const byte SOLE_VL_EXIT_TEMPERATURE = 23;       // Temperatur im Sole Wärmetauscher VL, bei der der Solemodus abgebrochen wird
const byte MIN_KOLLEKTOR_LUFT = 36;             // Mindest-Regeltemperatur für den Kollektor-Vorlauf
const byte MAX_KOLLEKTOR_LUFT_BOILERMODUS = 77; // Maximal-Regeltemperatur für Kollektor Vorlauf bei Boilermodus
const byte MAX_KOLLEKTOR_LUFT_SOLEMODUS = 70;   // Maximal-Regeltemperatur für Kollektor Vorlauf bei Boilermodus
const byte BOILER_DIRECT_EXIT_DIFF = 8;         // Maximaler Temperaturunterschied zwischen Boilertemperatur und Boiler VL bevor direkter Abbruch
const byte MIN_DIFFERENZ_VL_RL_BOILER = 12;     // Wenn VL und RL weniger als Diese Differenz haben, wird der Modus abgebrochen
const byte MIN_DIFFERENZ_VL_RL_SOLE = 10;       // Wenn VL und RL weniger als Diese Differenz haben, wird der Modus abgebrochen
const byte MIN_WAERMER_KOLLEKTOR_VL_BOILER = 3; // mindest Temperaturunterschied zwischen Boiler und Kollektor VL, bei dem der Boilermodus gestartet wird
const byte BOILER_FULL_TEMPERATURE = 65;        // hat der Boiler oben mindestens diese Temperatur, wird bevorzugt in den Boden gearbeitet
const byte BOILER_NOT_FULL_TEMPERATUR = 60;     // hat der Boiler oben weniger als diese Temperatur, wird bevorzugt in den Boden gearbeitet

// PID Tuning Parameter
const double PID_P_KOLLEKTOR = 5;         // Verstärkung des Proportionalen Anteils des PID-Reglers der Kollektorpumpe
const double PID_I_KOLLEKTOR = 0.015;     // Verstärkung des Integralen Anteils des PID-Reglers der Kollektorpumpe
const double PID_D_KOLLEKTOR = 0;         // Verstärkung des differentialen Anteils des PID-Reglers der Kollektorpumpe
const byte PID_KOLLEKTOR_MIN_SPEED = 70;  // Minimale Kollektorpumpen-Geschwindigkeit
const byte PID_KOLLEKTOR_MAX_SPEED = 255; // Maximale Kollektorpumpen-Geschwindigkeit

// Ethernet Konstanten
const byte mac[] = {0xA8, 0x61, 0x0A, 0xAE, 0x3D, 0xB7}; // Hardware-Adresse des Ethernet-Boards (Kleber auf Board)

// Variabeln
byte operationMode = 0;                   // Betriebsmodus: 0=aus, 1=Solemodus, 2=Boilermodus, 3=ManuellerModus
unsigned long now = 0;                    // Jeweils aktueller millis()-Wert
bool displayOn = false;                   // true, wenn Displays eingeschaltet sein soll
bool boilerHighTemperatur = false;        // true, wenn Boiler auf höherer Temperatur ist
bool kollektorAlarm = false;              // true, wenn kollektor zu heiss ist
bool soleAlarm = false;                   // true, wenn solepumpe zu heiss ist
bool initializing = false;                // Ist derzeit ein neuer Modus am Initialisieren?
bool tooLowValue = false;                 // True, wenn Sollwert das erste mal unterschritten
bool lastStateFlowMeterBoiler = LOW;      // Lester Status des Flow Meter Boiler
bool lastStateFlowMeterSole = LOW;        // Lester Status des Flow Meter Boiler
bool temperatureErrorMessageSent = false; // Fehlermeldung wegen unrealistischer Temperatur bereits gesendet?
bool badWeather = false;                  // Bei Schlechtwetter wird nicht versucht, den Boiler zu heizen
bool manualSpeed = false;                 // Ist die Pumpengeschwindigkeit auf Manuell?
bool soleCriticalTemp = false;            // Sole läuft Heiss?
bool solePriority = false;                // Sole Priorität?

// PID Variabeln
double PIDInputKollektorPumpe = 0;     // PID Input
double PIDOutputKollektorPumpe = 0;    // PID Output
double PIDSetpointKollektorPumpe = 50; // PID Setpoint

// ====================================
// 3. PIN-Adressen und BUS-Adressierung
// ====================================

// Analog Pins
const byte FUEHLER_KOLLEKTOR_VL_PIN = A7;   // S0 Anschluss PIN
const byte FUEHLER_KOLLEKTOR_LUFT_PIN = A0; // S1 Anschluss PIN
const byte FUEHLER_BOILER_1_PIN = A1;       // S2 oben Anschluss PIN
const byte FUEHLER_BOILER_2_PIN = A13;      // S2 mitte Anschluss PIN
const byte FUEHLER_BOILER_3_PIN = A14;      // S2 unten Anschluss PIN
const byte FUEHLER_BOILER_VL_PIN = A3;      // S4B Anschluss PIN
const byte FUEHLER_SOLE_VL_PIN = A4;        // S4S Anschluss PIN
const byte FUEHLER_BOILER_RL_PIN = A5;      // S6B Anschluss PIN
const byte FUEHLER_SOLE_RL_PIN = A6;        // S6S Anschluss PIN
const byte FUEHLER_SOLE_PIN = A2;           // S7 Anschluss PIN

// Digital In Pins
const byte FLOW_METER_BOILER = 30; // S5B Anschluss PIN Durchflussmesser Boilerkreis
const byte FLOW_METER_SOLE = 32;   // S5S Anschluss PIN Durchflussmesser Solekreis

// Digital Out Pins
const byte RELAIS_SOLE_PUMPE = 23;      // R0 Relais-Ausgang Solepumpe
const byte RELAIS_KOLLEKTOR_PUMPE = 25; // R1 Relais-Ausgang Kollektorpumpe
const byte STELLWERK_SOLE_BOILER = 29;  // R2B Relais-Ausgang Stellwerk Sole/Boiler, high = Boiler
const byte STELLWERK_WP_KREIS = 27;     // R2A Relais-Ausgang Stellwerk WP-Kreis, low = direkt, high = ueber Waermetauscher
const byte STELLWERK_BOILER_TEMP = 31;  // R4 Relais-Ausgang Stellwerk Solltemperatur Boilereinsatz, high = höhere Temperatur
const byte RELAIS_BOILER = 33;          // R5 Relais-Ausgang für Sperrschuetz WWSP

// PWM Pins
const byte PWM_KOLLEKTOR_PUMPE = 9; // PWM-Ausgang Kollektorpumpe

// ====================
// 4. INITIALISIERUNGEN
// ====================

// Setup aller PT1000 Fühler
// Schema: PT1000 <Name des Fühlers>(<PIN>,<Vorwiderstand>,<Korrekturfaktor>);
PT1000 fuehlerKollektorVL(FUEHLER_KOLLEKTOR_VL_PIN, 1000);
PT1000 fuehlerKollektorLuft(FUEHLER_KOLLEKTOR_LUFT_PIN, 1000);
PT1000 fuehlerBoiler1(FUEHLER_BOILER_1_PIN, 1000);
PT1000 fuehlerBoiler2(FUEHLER_BOILER_2_PIN, 1000);
PT1000 fuehlerBoiler3(FUEHLER_BOILER_3_PIN, 1000);
PT1000 fuehlerBoilerVL(FUEHLER_BOILER_VL_PIN, 1000);
PT1000 fuehlerSoleVL(FUEHLER_SOLE_VL_PIN, 1000);
PT1000 fuehlerBoilerRL(FUEHLER_BOILER_RL_PIN, 1000);
PT1000 fuehlerSoleRL(FUEHLER_SOLE_RL_PIN, 1000);
PT1000 fuehlerSole(FUEHLER_SOLE_PIN, 1000);

// Helligkeitssensor
Adafruit_TSL2591 luxMitte = Adafruit_TSL2591(2591); // Lichtsensor Mitte

// Setup der PWM gesteuerten Pumpen
Pump kollektorPumpe(RELAIS_KOLLEKTOR_PUMPE, PWM_KOLLEKTOR_PUMPE);

// Setup der Timer (leer für ms, 's' für s, 'm' für min, 'd' für Tage)
// Timer über normale Clock
Timer timer1s(1, 's'); // 1s Timer
Timer timer5s(5, 's'); // 5s Timer
Timer timer1m(1, 'm'); // 1m Timer
Timer timer3m(3, 'm'); // 3min Timer

// Timer für bestimmte Funktionen
Timer timerLegionellenschaltung(7, 'd');   // Legio-Timer (Zeit. nach der elektrisch auf hohe Boilertemperatur geheizt wird, falls diese nie erreicht wurde)
Timer initialOperationModeTimeout(3, 'm'); // Zeit bevor ein Modus EXIT-Kriterien berücksichtigt (Achtung, Variabel)
Timer exitTimeout(4, 'm');                 // Solange muss der Sollwert mindestens unterschritten sein, bevor Abbruch
Timer flowMeterBoilerTimeout(2, 's');      // Durchflussmeter 1 Timeout
Timer boilerTimeout(2, 'd');               // Boiler-Ausschaltzeit
Timer MQTTSendTimer(5, 's');               // Sendeinterval Daten an Dashboard (Achtung, Variabel)
Timer boilerEnoughFull(1, 'd');            // Timeout-Zeit, wenn Boiler heiss war und in der Zeit Sole priorisiert werden kann
Timer soleTooHot(10, 'm');                 // Timer für Sole-Alarm

// PWM Setup
PID PIDReglerKollektorPumpe(&PIDInputKollektorPumpe, &PIDOutputKollektorPumpe, &PIDSetpointKollektorPumpe, PID_P_KOLLEKTOR, PID_I_KOLLEKTOR, PID_D_KOLLEKTOR, REVERSE);

// MQTT Setup
EthernetClient client;
MqttClient mqttClient(client);

const char broker[] = "192.168.1.123"; // MQTT-Broker-Server
const int port = 1883;                 // MQTT-Broker-Server-Port
const String topic = "derart/";        // Standard MQTT-Topic
String recievedTopic;                  // Empfangenes Thema
char recievedPayload;                  // Empfangenes Zeichen

// Ethernet Client Setup
EthernetClient httpClient; // httpClient für SMS

// NTP Client Setup
const char ntpServer[] = "europe.pool.ntp.org";       // NTP Server
EthernetUDP ntpUDP;                                   // UDP Client für NTP Server
NTPClient timeClient(ntpUDP, ntpServer, 7200, 60000); // NTP Client

// =============
// 5. FUNKTIONEN
// =============

// Sendet eine SMS
void sendSMS(String message, String reciever = "0041795085611", String from = "Solaranlage")
{
  message.replace(" ", "%20");
  String data = SMS_BASE_URL;
  data += message;
  data += "&from=";
  data += from;
  data += "&to=";
  data += reciever;
  data += " HTTP/1.0";
  Serial.println("SMS wurde versendet");
  if (httpClient.connect("www.lox24.eu", 80))
  {
    httpClient.println(data);
    httpClient.println("Host: www.lox24.eu");
    httpClient.println("Connection: close");
    httpClient.println();
    httpClient.stop();
  }
  else
  {
    Serial.println("Senden Fehlgeschlagen");
  }
}

// Sendet eine MQTT-Message mit dem Thema <subtopic> und dem Fliesskommazahl-Wert <value>
void sendMQTT(String subtopic, float value)
{
  mqttClient.beginMessage(topic + subtopic);
  mqttClient.print(value);
  mqttClient.endMessage();
}

// Sendet eine MQTT-Message mit dem Thema <subtopic> und dem ganzzahligen Wert <value>
void sendMQTT(String subtopic, int value)
{
  mqttClient.beginMessage(topic + subtopic);
  mqttClient.print(value);
  mqttClient.endMessage();
}

// Sendet eine MQTT-Message mit dem Thema <subtopic> und dem ganzzahligen, positiven Long-Wert <value>
void sendMQTT(String subtopic, unsigned long value)
{
  mqttClient.beginMessage(topic + subtopic);
  mqttClient.print(value);
  mqttClient.endMessage();
}

// Sendet eine MQTT-Message mit dem Thema <subtopic> und dem String <value>
void sendMQTT(String subtopic, String value)
{
  int valueLength = value.length() + 1;
  char charValue[valueLength];
  value.toCharArray(charValue, valueLength);
  mqttClient.beginMessage(topic + subtopic);
  mqttClient.print(charValue);
  mqttClient.endMessage();
}

// Sendet alle Daten an Dashboard
void sendMQTTAll()
{
  // Fuehlerwerte an MQTT Senden
  sendMQTT("fuehlerKollektorVL", fuehlerKollektorVL.getMeanTemperature());
  sendMQTT("fuehlerKollektorLuft", fuehlerKollektorLuft.getMeanTemperature());
  sendMQTT("fuehlerBoiler/1", fuehlerBoiler1.getMeanTemperature());
  sendMQTT("fuehlerBoiler/2", fuehlerBoiler2.getMeanTemperature());
  sendMQTT("fuehlerBoiler/3", fuehlerBoiler3.getMeanTemperature());
  sendMQTT("fuehlerBoilerVL", fuehlerBoilerVL.getMeanTemperature());
  sendMQTT("fuehlerSoleVL", fuehlerSoleVL.getMeanTemperature());
  sendMQTT("fuehlerBoilerRL", fuehlerBoilerRL.getMeanTemperature());
  sendMQTT("fuehlerSoleRL", fuehlerSoleRL.getMeanTemperature());
  sendMQTT("fuehlerSole", fuehlerSole.getMeanTemperature());

  // Speed der Kollektorpumpe an Dashboard senden
  if (!soleAlarm && !kollektorAlarm)
  {
    sendMQTT("speedKollektorPumpe", (int)round(PIDOutputKollektorPumpe));
  }

  // Informationen an Dashboard
  sendMQTT("kollektorPumpe", digitalRead(RELAIS_KOLLEKTOR_PUMPE));         // Info an Dashboard: Kollektorpumpe
  sendMQTT("solePumpe", digitalRead(RELAIS_SOLE_PUMPE));                   // Info an Dashboard, Solepumpe
  sendMQTT("stellwerkSoleBoiler", digitalRead(STELLWERK_SOLE_BOILER) + 1); // Info an Dashboard, Stellwerk Sole/Boiler
  sendMQTT("stellwerkWP", digitalRead(STELLWERK_WP_KREIS) + 1);            // Info an Dashboard, Stellwerk WP-Kreis
  sendMQTT("operationMode", operationMode);                                // Info an Dashboard, Modus
}

// Alle Fühler auslesen
void fuehlerCalculateAll()
{
  bool unrealisticTemperature = false;
  // Alle Fühler auslesen
  if (!fuehlerKollektorVL.calculateTemperature())
  {
    unrealisticTemperature = true;
  }
  if (!fuehlerKollektorLuft.calculateTemperature())
  {
    unrealisticTemperature = true;
  }
  if (!fuehlerBoiler1.calculateTemperature())
  {
    unrealisticTemperature = true;
  }
  if (!fuehlerBoiler2.calculateTemperature())
  {
    unrealisticTemperature = true;
  }
  if (!fuehlerBoiler3.calculateTemperature())
  {
    unrealisticTemperature = true;
  }
  if (!fuehlerBoilerVL.calculateTemperature())
  {
    unrealisticTemperature = true;
  }
  if (!fuehlerSoleVL.calculateTemperature())
  {
    unrealisticTemperature = true;
  }
  if (!fuehlerBoilerRL.calculateTemperature())
  {
    unrealisticTemperature = true;
  }
  if (!fuehlerSoleRL.calculateTemperature())
  {
    unrealisticTemperature = true;
  }
  if (!fuehlerSole.calculateTemperature())
  {
    unrealisticTemperature = true;
  }

  // Fehlermeldung per SMS, falls unrealistische Temperatur auftritt
  if (unrealisticTemperature && !temperatureErrorMessageSent)
  {
    sendSMS("Ein Temperaturfuehler macht Probleme. Bei der Steuerung koennten schwerwiegende Probleme auftreten. Bitte Hardware kontrollieren. Nach Fehlerbehebung ist Neustart erforderlich.");
    temperatureErrorMessageSent = true;
  }
}

// Gibt Soll Vorlauftemperatur am Kollektor zurücksetzen
void calculateTargetTemperature()
{
  // Differenz zwischen Kollektor Luft und VL bestimmen
  float differenzLuftVL = fuehlerKollektorVL.getMeanTemperature() - fuehlerKollektorLuft.getMeanTemperature();
  double newPIDSetpointKollektorPumpe = PIDSetpointKollektorPumpe;  // neuer PID Setpoint
  double lastPIDSetpointKollektorPumpe = PIDSetpointKollektorPumpe; // letzter PID Setpoint
  if (operationMode)                                                // läuft die Anlage?
  {
    if (differenzLuftVL > 10)
    {
      differenzLuftVL = 10;
    }
    else if (differenzLuftVL < 0)
    {
      differenzLuftVL = 0;
    }

    if (digitalRead(STELLWERK_SOLE_BOILER)) // Boilermodus?
    {
      solePriority = false;
      if (!initializing) // nicht am initialisieren?
      {
        if (fuehlerBoilerVL.getMeanTemperature() - fuehlerBoilerRL.getMeanTemperature() < MIN_DIFFERENZ_VL_RL_BOILER + 2) // VL und RL nähern sich zu stark an
        {
          newPIDSetpointKollektorPumpe = ceil(fuehlerBoiler1.getMeanTemperature() + MIN_WAERMER_KOLLEKTOR_VL_BOILER + 5 - differenzLuftVL); // Solltemperatur für Boilermodus
        }
        else if (fuehlerBoilerVL.getMeanTemperature() - fuehlerBoilerRL.getMeanTemperature() > MIN_DIFFERENZ_VL_RL_BOILER + 5) // VL und RL sind weit voneinander entfernt
        {
          newPIDSetpointKollektorPumpe = ceil(fuehlerBoiler1.getMeanTemperature() + MIN_WAERMER_KOLLEKTOR_VL_BOILER - differenzLuftVL); // Solltemperatur für Boilermodus
        }
        else if (PIDSetpointKollektorPumpe < fuehlerBoiler1.getMeanTemperature() + MIN_WAERMER_KOLLEKTOR_VL_BOILER - differenzLuftVL)
        {
          newPIDSetpointKollektorPumpe = ceil(fuehlerBoiler1.getMeanTemperature() + MIN_WAERMER_KOLLEKTOR_VL_BOILER - differenzLuftVL); // Solltemperatur für Boilermodus
        }
        if (floor(fuehlerBoiler1.getMeanTemperature()) > BOILER_FULL_TEMPERATURE && !manualSpeed) // Boiler heiss genug und nicht manuelle Geschwindigkeit?
        {
          PIDReglerKollektorPumpe.SetOutputLimits(115, PID_KOLLEKTOR_MAX_SPEED); // mindest-Geschwindigkeit fuer Boiler erhöhen
        }
        else if (!manualSpeed)
        { // Pumpengeschwindigkeit auf standard setzen, wenn nicht auf manuell
          PIDReglerKollektorPumpe.SetOutputLimits(PID_KOLLEKTOR_MIN_SPEED, PID_KOLLEKTOR_MAX_SPEED);
        }
      }
      else // nicht am Initialisieren im Boilermodus
      {
        newPIDSetpointKollektorPumpe = ceil(fuehlerBoiler1.getMeanTemperature() + MIN_WAERMER_KOLLEKTOR_VL_BOILER - differenzLuftVL); // Solltemperatur für Boilermodus
      }
    }
    else // Solemodus?
    {
      newPIDSetpointKollektorPumpe = ceil(fuehlerBoiler1.getMeanTemperature() + MIN_WAERMER_KOLLEKTOR_VL_BOILER - differenzLuftVL); // Solltemperatur für Solemodus
      if (fuehlerBoiler1.getMeanTemperature() > BOILER_FULL_TEMPERATURE)                                                            // Boiler berets heiss?
      {
        newPIDSetpointKollektorPumpe = MIN_KOLLEKTOR_LUFT; // Sole mit hoher Drehzahl, da Boiler bereits heiss
        solePriority = true;                               // Sole Priorität setzen
      }
      else if (fuehlerBoiler1.getMeanTemperature() > BOILER_NOT_FULL_TEMPERATUR + 1 && !(boilerEnoughFull.checkTimer(now)))
      {
        newPIDSetpointKollektorPumpe = MIN_KOLLEKTOR_LUFT; // Sole mit hoher Drehzahl, da Boiler in den letzten 24h heiss war
        solePriority = true;                               // Sole Priorität setzen
      }
      else if (fuehlerBoiler1.getMeanTemperature() > BOILER_NOT_FULL_TEMPERATUR && !(boilerEnoughFull.checkTimer(now)) && solePriority)
      {                                                    // Umschaltung bald
        newPIDSetpointKollektorPumpe = MIN_KOLLEKTOR_LUFT; // Sole mit hoher Drehzahl, da Boiler in den letzten 24h heiss war
      }
      else
      {
        solePriority = false; // Sole Priorität beenden
      }
      if (badWeather)
      {
        newPIDSetpointKollektorPumpe = MIN_KOLLEKTOR_LUFT; // Sole mit hoher Drehzahl, da zu schlechtes Wetter für Boiler
      }
      if (fuehlerSole.getMeanTemperature() > CRITICAL_TEMPERATURE_SOLE) // Sole VL zu heiss?
      {
        newPIDSetpointKollektorPumpe = MIN_KOLLEKTOR_LUFT + 14;
        soleCriticalTemp = true;
      }
      else if (soleCriticalTemp && fuehlerSole.getMeanTemperature() > CRITICAL_TEMPERATURE_SOLE - 2)
      {
        newPIDSetpointKollektorPumpe = MIN_KOLLEKTOR_LUFT + 14;
      }
      else if (fuehlerSole.getMeanTemperature() <= CRITICAL_TEMPERATURE_SOLE - 2 && soleCriticalTemp)
      {
        newPIDSetpointKollektorPumpe = MIN_KOLLEKTOR_LUFT;
        soleCriticalTemp = false;
      }
    }

    if (timeClient.getHours() == 0 && badWeather)
    {
      badWeather = false; // Schlechtwetter-Reset
    }

    if (timeClient.getHours() < 11 || (timeClient.getHours() == 11 && timeClient.getMinutes() <= 30) && !soleCriticalTemp)
    {
      newPIDSetpointKollektorPumpe = MIN_KOLLEKTOR_LUFT; // Sole mit hoher Drehzahl, da zu früh um Boiler zu heizen
    }

    if (newPIDSetpointKollektorPumpe > PIDSetpointKollektorPumpe)
    {
      PIDSetpointKollektorPumpe = newPIDSetpointKollektorPumpe;
    }
    else if (newPIDSetpointKollektorPumpe < PIDSetpointKollektorPumpe - 5)
    {
      PIDSetpointKollektorPumpe = newPIDSetpointKollektorPumpe;
    }

    if (PIDSetpointKollektorPumpe < MIN_KOLLEKTOR_LUFT)
    {
      PIDSetpointKollektorPumpe = MIN_KOLLEKTOR_LUFT;
    }
    else if (PIDSetpointKollektorPumpe > MAX_KOLLEKTOR_LUFT_SOLEMODUS)
    {
      if (digitalRead(STELLWERK_SOLE_BOILER) && PIDSetpointKollektorPumpe > MAX_KOLLEKTOR_LUFT_BOILERMODUS) // Boilermodus?
      {
        PIDSetpointKollektorPumpe = MAX_KOLLEKTOR_LUFT_BOILERMODUS;
      }
      else if (!digitalRead(STELLWERK_SOLE_BOILER)) // Solemodus?
      {
        PIDSetpointKollektorPumpe = MAX_KOLLEKTOR_LUFT_SOLEMODUS;
      }
    }
  }
  else
  {
    PIDSetpointKollektorPumpe = 0;
    solePriority = false;
  }
  // Solltemperatur verändert?
  if (PIDSetpointKollektorPumpe != lastPIDSetpointKollektorPumpe)
  {
    // Serial.print("Neue Regel-Solltemperatur: ");
    // Serial.println(PIDSetpointKollektorPumpe, 1);
    // sendMQTT("message", (String) "Neue Regel-Solltemperatur: " + ceil(PIDSetpointKollektorPumpe));
    sendMQTT("controlTemperature", (int)(ceil(PIDSetpointKollektorPumpe)));
  }
}

// Startet den Boilermodus
void boilerModusStart()
{
  digitalWrite(RELAIS_KOLLEKTOR_PUMPE, HIGH);                                                // Kollektorpumpe einschalten
  digitalWrite(RELAIS_SOLE_PUMPE, LOW);                                                      // Solepumpe ausschalten
  digitalWrite(STELLWERK_SOLE_BOILER, HIGH);                                                 // Stellwerk auf Boiler umschalten
  digitalWrite(STELLWERK_WP_KREIS, HIGH);                                                    // Stellwerk ueber Warmetauscher lassen
  PIDReglerKollektorPumpe.SetMode(1);                                                        // PID-Regler Kollektorpumpe einschalten
  initializing = true;                                                                       // Initialisierung starten
  initialOperationModeTimeout.setDelayTime(6, 'm');                                          // Initialisierungszeit setzen
  initialOperationModeTimeout.setLastTime(now);                                              // Initialisierungs Timer starten
  operationMode = 2;                                                                         // Modus auf Boiler schalten
  tooLowValue = false;                                                                       // tooLowValue zurücksetzen
  PIDReglerKollektorPumpe.SetOutputLimits(PID_KOLLEKTOR_MIN_SPEED, PID_KOLLEKTOR_MAX_SPEED); // PID-Regler auf Standard-Geschwindigkeit setzen
  MQTTSendTimer.setDelayTime(5, 's');
  if (!digitalRead(RELAIS_KOLLEKTOR_PUMPE))
  {
    sendSMS("Beim einschalten des Boilermodus ist ein Fehler aufgetreten"); // Alarmnachricht per SMS"
  }
}

// Startet den Solemodus
void soleModusStart()
{
  digitalWrite(RELAIS_KOLLEKTOR_PUMPE, HIGH); // Kollektorpumpe ausschalten
  digitalWrite(RELAIS_SOLE_PUMPE, HIGH);      // Solepumpe einschalten
  digitalWrite(STELLWERK_SOLE_BOILER, LOW);   // Stellwerk auf Sole umschalten
  digitalWrite(STELLWERK_WP_KREIS, HIGH);     // Stellwerk ueber Warmetauscher lassen
  PIDReglerKollektorPumpe.SetMode(1);         // PID-Regler Kollektorpumpe einschalten
  initializing = true;                        // Initialisierung starten
  switch (operationMode)                      // Initialisierungszeit setzen
  {
  case 0:                                              // War ausgeschaltet?
    initialOperationModeTimeout.setDelayTime(15, 'm'); // Initialisierungszeit, wenn Modus 0-->1
    break;
  case 2:                                             // War im Boilermodus??
    initialOperationModeTimeout.setDelayTime(3, 'm'); // Initialisierungszeit, wenn MNodus 2-->1
    break;
  }
  initialOperationModeTimeout.setLastTime(now);                                              // Initialisierungs Timer starten
  operationMode = 1;                                                                         // Modus auf Sole schalten
  tooLowValue = false;                                                                       // tooLowValue zurücksetzen
  PIDReglerKollektorPumpe.SetOutputLimits(PID_KOLLEKTOR_MIN_SPEED, PID_KOLLEKTOR_MAX_SPEED); // PID-Regler auf Standard-Geschwindigkeit setzen
  MQTTSendTimer.setDelayTime(5, 's');
  if (!(digitalRead(RELAIS_KOLLEKTOR_PUMPE) && digitalRead(RELAIS_SOLE_PUMPE)))
  {
    sendSMS("Beim einschalten des Solemodus ist ein Fehler aufgetreten"); // Alarmnachricht per SMS"
  }
}

// Schaltet die Anlage aus
void turnOffModusStart()
{
  digitalWrite(RELAIS_KOLLEKTOR_PUMPE, LOW);        // Kollektorpumpe ausschalten
  digitalWrite(RELAIS_SOLE_PUMPE, LOW);             // Solepumpe ausschalten
  digitalWrite(STELLWERK_SOLE_BOILER, LOW);         // Stellwerk auf Sole umschalten
  digitalWrite(STELLWERK_WP_KREIS, LOW);            // Stellwerk ueber Warmetauscher lassen
  PIDReglerKollektorPumpe.SetMode(0);               // PID-Regler Kollektorpumpe ausschalten
  PIDOutputKollektorPumpe = 0;                      // Speed auf Null setzen
  kollektorPumpe.stop();                            // kollektorpumpe stoppen
  initialOperationModeTimeout.setDelayTime(3, 'm'); // Wie lange mindestens ausgeschaltet?
  initialOperationModeTimeout.setLastTime(now);     // Initialisierungs Timer starten
  initializing = true;                              // Initialisierung starten
  operationMode = 0;                                // Modus auf aus schalten
  tooLowValue = false;                              // tooLowValue zurücksetzen
}

// Schaltet nur die Kollektorpumpe ein
void kollektorPumpStart()
{
  digitalWrite(RELAIS_KOLLEKTOR_PUMPE, HIGH); // Kollektorpumpe einschalten
  PIDReglerKollektorPumpe.SetMode(1);         // PID-Regler Kollektorpumpe einschalten
  operationMode = 3;                          // Modus auf Boiler schalten
  MQTTSendTimer.setDelayTime(5, 's');
  if (!digitalRead(RELAIS_KOLLEKTOR_PUMPE))
  {
    sendSMS("Beim einschalten der Kollektorpumpe ist ein Fehler aufgetreten"); // Alarmnachricht per SMS"
  }
}

void subscribeToMQTTTopics()
{
  mqttClient.subscribe("derart/toArduino/LegionellenModus");
  mqttClient.subscribe("derart/toArduino/Betriebsmodus");
  mqttClient.subscribe("derart/toArduino/SpeedKollektorpumpe");
  mqttClient.subscribe("derart/toArduino/severalFunctions");
  mqttClient.subscribe("derart/toArduino/BadWeather");
}

void boilerAdditionalHeatingOn()
{
  digitalWrite(RELAIS_BOILER, HIGH);
  sendMQTT("boilerAdditionalHeating", "on");
}
void boilerAdditionalHeatingOff()
{
  digitalWrite(RELAIS_BOILER, LOW);
  sendMQTT("boilerAdditionalHeating", "off");
}

// ================
// 6. Setup-Sequenz
// ================
void setup()
{
  // Serielle Schnittstelle starten
  Serial.begin(9600);
  while (!Serial)
  {
    ; // warten, bis die Serielle Schnittstelle sich verbindet
  }
  Serial.println("Setup gestartet");

  // Setup der PINS
  // Input
  pinMode(FLOW_METER_BOILER, INPUT);
  pinMode(FLOW_METER_SOLE, INPUT);

  // Output
  pinMode(RELAIS_SOLE_PUMPE, OUTPUT);
  pinMode(RELAIS_KOLLEKTOR_PUMPE, OUTPUT);
  pinMode(STELLWERK_SOLE_BOILER, OUTPUT);
  pinMode(STELLWERK_WP_KREIS, OUTPUT);
  pinMode(STELLWERK_BOILER_TEMP, OUTPUT);
  pinMode(PWM_KOLLEKTOR_PUMPE, OUTPUT);
  pinMode(RELAIS_BOILER, OUTPUT);

  // PID Setup
  PIDReglerKollektorPumpe.SetOutputLimits(PID_KOLLEKTOR_MIN_SPEED, 255);
  PIDReglerKollektorPumpe.SetSampleTime(5000);

  // Initialisiere Internet und MQTT
  // Versuch, sich mit dem Internet zu verbinden
  Serial.println("Internet-Verbindungsversuch ueber DHCP");
  while (Ethernet.begin(mac) == 0)
  {
    // fehlgeschlagen, erneut versuchen
    Serial.print(".");
    delay(5000);
  }

  Serial.println("Mit dem Internet verbunden! IP-Adresse: ");
  Serial.println(Ethernet.localIP());
  Serial.println();

  // MQTT Verbindung
  Serial.print("Versuch, sich mit dem MQTT Broker zu verbinden ");
  mqttClient.setUsernamePassword(MQTT_USER, MQTT_PASS); // User und Passwort von MQTT
  Serial.println(broker);                               // Name des Broker-Servers ausgeben

  for (uint8_t i = 0; i < 100; i++)
  {
    if (!mqttClient.connect(broker, port))
    {
      Serial.print("MQTT Verbindung fehlgeschlagen! Fehlercode = ");
      Serial.println(mqttClient.connectError());
      Serial.println("Neuer Verbindungsversuch ...");
      delay(1500);
    }
    else
    {
      break;
    }
  }

  Serial.println("Du bist mit dem MQTT Broker verbunden!");
  Serial.println();

  subscribeToMQTTTopics();
  Serial.println("Du hast die MQTT Topics abonniert!");
  Serial.println();

  timeClient.begin();
  Serial.println("Du bist mit dem Zeitserver verbunden!");
  sendMQTT("startTime", 1);
  Serial.println();

  // erster Betriebsmodus nach dem Starten (Ohne Initialisierungszeit)
  turnOffModusStart();
  initializing = false;

  // Boiler bei Start immer auf 65 Grad heizen
  digitalWrite(STELLWERK_BOILER_TEMP, HIGH); // Boiler auf höhere Temperatur stellen
  boilerHighTemperatur = true;
  boilerTimeout.setLastTime(now);       // Timer stellen
  timerLegionellenschaltung.executed(); // Timer zurücksetzen
  boilerAdditionalHeatingOn();
  sendMQTT("message", "Boiler bei erhöhter Solltemperatur für elektrisches Heizen. (Legionellenschaltung)");
  sendMQTT("boilerTermostat", "hoch");

  // Helligkeitssensor starten und konfigurieren
  if (luxMitte.begin())
  {
    Serial.println(F("Helligkeitssensor gefunden"));
  }
  else
  {
    Serial.println(F("Kein Helligkeitssensor gefunden.?"));
  }
  luxMitte.setGain(TSL2591_GAIN_LOW); // 1x gain (bright light)
  luxMitte.setTiming(TSL2591_INTEGRATIONTIME_100MS);
  Serial.println(F("Helligkeitssensor konfiguriert"));

  Serial.println("Setup beendet");
}

// ===================
// 7. Programmschleife
// ===================
void loop()
{
  // Dieser Programmteil wird in jeder Schleife durchgefuehrt
  timeClient.update();

  // Prüfen, ob der Durchflussmesser Boiler einen Impuls abgibt
  if (flowMeterBoilerTimeout.checkTimer(now) && (operationMode == 2 || operationMode == 3))
  {
    if (digitalRead(FLOW_METER_BOILER) && !lastStateFlowMeterBoiler)
    {
      sendMQTT("flowMeterBoiler", 1);
      lastStateFlowMeterBoiler = HIGH;
    }
    else if (!digitalRead(FLOW_METER_BOILER) && lastStateFlowMeterBoiler)
    {
      lastStateFlowMeterBoiler = LOW;
    }

    flowMeterBoilerTimeout.executed();
  }

  // Nachrichten empfangen und verarbeiten
  int messageSize = mqttClient.parseMessage();
  if (messageSize)
  {
    recievedTopic = mqttClient.messageTopic();
    recievedPayload = (char)mqttClient.read();

    if (recievedTopic == "derart/toArduino/LegionellenModus")
    {
      if (recievedPayload == '1')
      {
        timerLegionellenschaltung.setLastTime((unsigned long)(now - timerLegionellenschaltung.getDelayTime()));
        Serial.println("Legionellenschaltung aus der Ferne eingeschaltet");
      }
      else if (recievedPayload == '0')
      {
        timerLegionellenschaltung.setLastTime(now);
        boilerHighTemperatur = false;
        digitalWrite(STELLWERK_BOILER_TEMP, LOW);
        boilerAdditionalHeatingOff();
        sendMQTT("boilerTermostat", "normal");
        sendMQTT("message", "Boiler wieder bei normaler Solltemperatur für elektrisches Heizen.");
        Serial.println("Legionellenschaltung aus der Ferne ausgeschaltet");
      }
    }
    if (recievedTopic == "derart/toArduino/Betriebsmodus")
    {
      if (recievedPayload == '0')
      {
        turnOffModusStart();
        sendMQTT("message", "Anlage von Hand ausgeschaltet. Nach Initialisierung wieder Automatikbetrieb.");
      }
      else if (recievedPayload == '1')
      {
        soleModusStart();
        sendMQTT("message", "Anlage von Hand zu Sole laden umgeschaltet. Nach Initialisierung wieder Automatikbetrieb.");
      }
      else if (recievedPayload == '2')
      {
        boilerModusStart();
        sendMQTT("message", "Anlage von Hand zu Boiler laden umgeschaltet. Nach Initialisierung wieder Automatikbetrieb.");
      }
      else if (recievedPayload == '3')
      {
        turnOffModusStart();
        operationMode = 3;
        sendMQTT("message", "Handbetrieb: Ausgeschaltet. Automatikfunktion aus.");
      }
      else if (recievedPayload == '4')
      {
        soleModusStart();
        operationMode = 3;
        sendMQTT("message", "Handbetrieb: Sole laden. Automatikfunktion aus.");
      }
      else if (recievedPayload == '5')
      {
        boilerModusStart();
        operationMode = 3;
        sendMQTT("message", "Handbetrieb: Boiler laden. Automatikfunktion aus.");
      }
    }
    if (recievedTopic == "derart/toArduino/SpeedKollektorpumpe")
    {
      switch (recievedPayload)
      {
      case '0':
        PIDReglerKollektorPumpe.SetOutputLimits(PID_KOLLEKTOR_MIN_SPEED, PID_KOLLEKTOR_MAX_SPEED);
        manualSpeed = false;
        sendMQTT("message", "Pumpen-Geschwindigkeit wieder bei Auto.");
        break;
      case '1':
        PIDReglerKollektorPumpe.SetOutputLimits(70, 71);
        manualSpeed = true;
        sendMQTT("message", "Kollektorpumpe: Manuell sehr langsam");
        if (!operationMode)
        // Wenn die Anlage nicht läuft, wird die Pumpe gestartet
        {
          kollektorPumpStart();
        }
        break;
      case '2':
        PIDReglerKollektorPumpe.SetOutputLimits(110, 111);
        manualSpeed = true;
        sendMQTT("message", "Kollektorpumpe: Manuell langsam");
        if (!operationMode)
        // Wenn die Anlage nicht läuft, wird die Pumpe gestartet
        {
          kollektorPumpStart();
        }
        break;
      case '3':
        PIDReglerKollektorPumpe.SetOutputLimits(150, 151);
        manualSpeed = true;
        sendMQTT("message", "Kollektorpumpe: Manuell mittel");
        if (!operationMode)
        // Wenn die Anlage nicht läuft, wird die Pumpe gestartet
        {
          kollektorPumpStart();
        }
        break;
      case '4':
        PIDReglerKollektorPumpe.SetOutputLimits(190, 191);
        manualSpeed = true;
        sendMQTT("message", "Kollektorpumpe: Manuell schnell");
        if (!operationMode)
        // Wenn die Anlage nicht läuft, wird die Pumpe gestartet
        {
          kollektorPumpStart();
        }
        break;
      case '5':
        PIDReglerKollektorPumpe.SetOutputLimits(250, 255);
        manualSpeed = true;
        sendMQTT("message", "Kollektorpumpe: Manuell sehr schnell");
        if (!operationMode)
        // Wenn die Anlage nicht läuft, wird die Pumpe gestartet
        {
          kollektorPumpStart();
        }
        break;
      }
    }
    if (recievedTopic == "derart/toArduino/BadWeather")
    {
      if (recievedPayload == '0')
      {
        badWeather = false;
      }
      else if (recievedPayload == '1')
      {
        badWeather = true;
      }
    }
  }

  // PID Regler berechnen
  if (PIDReglerKollektorPumpe.Compute())
  {
    kollektorPumpe.setSpeed(PIDOutputKollektorPumpe); // neuen Speed Kollektorpumpe setzen
  }

  // Dieser Programmteil wird alle 1s ausgeführt
  if (timer1s.checkTimer(now))
  {
    // Alle Fühler auslesen
    fuehlerCalculateAll();

    PIDInputKollektorPumpe = fuehlerKollektorLuft.getMeanTemperature(); // Kollektor Vorlauftemperatur in PID schreiben

    // Ethernet aktuell halten
    Ethernet.maintain();

    // MQTT Client am Leben halten
    mqttClient.poll();

    // Timer zurücksetzen
    timer1s.executed();
  }

  // Dieser Programmteil wird alle 5s ausgeführt
  if (timer5s.checkTimer(now))
  {
    //Überprüfen, ob Alarmwerte überschritten wurden
    if (fuehlerSole.getLastTemperature() > ALARMTEMPERATUR_SOLE && soleTooHot.checkTimer(now))
    {
      soleAlarm = true;
    }
    else if (fuehlerSole.getLastTemperature() < ALARMTEMPERATUR_SOLE)
    {
      soleTooHot.setLastTime(now);
    }

    if (fuehlerKollektorLuft.getLastTemperature() > ALARMTEMPERATUR_KOLLEKTOR_LUFT || fuehlerKollektorVL.getLastTemperature() > ALARMTEMPERATUR_KOLLEKTOR_VL)
    {
      kollektorAlarm = true;
    }

    // Soll Kollektor VL temperatur berechnen
    calculateTargetTemperature();

    // Alarmmodus
    // Sole AlArm?
    if (soleAlarm)
    {
      sendMQTT("alarm", "Alarm. Die Soletemperatur ist zu hoch."); // Alarmnachricht an Dashboard
      sendSMS("Alarm. Die Soletemperatur ist zu hoch.");           // Alarmnachricht per SMS"
      digitalWrite(RELAIS_SOLE_PUMPE, LOW);                        // Solepumpe ausschalten
      digitalWrite(RELAIS_KOLLEKTOR_PUMPE, HIGH);                  // Kollektorpumpe einschalten
      digitalWrite(STELLWERK_SOLE_BOILER, LOW);                    // Wärme über Wärmetauscher abkühlen
      digitalWrite(STELLWERK_WP_KREIS, HIGH);                      // Wärme an WP-kreis geben
      analogWrite(PWM_KOLLEKTOR_PUMPE, 255);                       // Kollektorpumpe auf 100%
      sendMQTT("speedKollektorPumpe", 255);                        // Speed der Kollektorpumpe an Dashboard senden
      operationMode = 4;                                           // Betriebsmodus Sole-Alarm
      if (!digitalRead(RELAIS_KOLLEKTOR_PUMPE))
      {
        sendSMS("Achtung: wir sind im Alarmmodus, aber die Kollektorpumpe laeuft nicht"); // Alarmnachricht per SMS"
      }
    }

    // Kollektor Alarm?
    if (kollektorAlarm)
    {
      sendMQTT("alarm", "Alarm. Die Kollektortemperatur ist zu hoch.");
      sendSMS("Alarm. Die Kollektortemperatur ist zu hoch."); // Alarmnachricht per SMS"
      digitalWrite(RELAIS_KOLLEKTOR_PUMPE, HIGH);             // Kollektorpumpe einschalten
      analogWrite(PWM_KOLLEKTOR_PUMPE, 255);                  // Kollektorpumpe auf 100%
      sendMQTT("speedKollektorPumpe", 255);                   // Speed der Kollektorpumpe an Dashboard senden
      digitalWrite(STELLWERK_SOLE_BOILER, HIGH);              // Wärme in Boiler leiten
      digitalWrite(STELLWERK_WP_KREIS, HIGH);                 // Wärme an WP-kreis geben
      operationMode = 5;                                      // Betriebsmodus Kollektor-Alarm
      if (!digitalRead(RELAIS_KOLLEKTOR_PUMPE))
      {
        sendSMS("Achtung: wir sind im Alarmmodus, aber die Kollektorpumpe laeuft nicht"); // Alarmnachricht per SMS"
      }
    }

    while (soleAlarm || kollektorAlarm)
    {
      // Alle Temperaturen auslesen
      fuehlerCalculateAll();
      if (fuehlerSole.getLastTemperature() < ALARMTEMPERATUR_SOLE - MIN_DIFFERENZ_NACH_ALARM && soleAlarm)
      {
        soleAlarm = false;
        sendMQTT("message", "Die Soletemperatur ist nicht mehr zu hoch."); // Dashboard Nachricht: Alarm beendet
        soleModusStart();                                                  // Nach Alarm in Boilermodus starten
      }

      // Kollektor VL und Luft prüfen, ob ausreichend abgekühlt
      if (fuehlerKollektorVL.getLastTemperature() < ALARMTEMPERATUR_KOLLEKTOR_VL - MIN_DIFFERENZ_NACH_ALARM && fuehlerKollektorLuft.getLastTemperature() < ALARMTEMPERATUR_KOLLEKTOR_LUFT - MIN_DIFFERENZ_NACH_ALARM && kollektorAlarm)
      {
        kollektorAlarm = false;
        sendMQTT("message", "Die Kollektortemperatue ist nicht mehr zu hoch."); // Dashboard Nachricht: Alarm beendet
        boilerModusStart();                                                     // Nach Alarm in Boilermodus starten
      }

      // Fuehlerwerte an MQTT Senden
      sendMQTTAll();

      // Dashbord sagen, dass online
      sendMQTT("alive", 1);
      delay(5000);
    }

    // Prüfen, ob Initialisierung abgeschlossen ist
    if (initialOperationModeTimeout.checkTimer(now) && initializing)
    {
      sendMQTT("message", "Die Initialisierung des aktuellen Betriebsmodus ist abgeschlossen.");
      initializing = false;
    }

    // Entscheidung Betriebsmodus
    switch (operationMode)
    {
    case 0: // Im Modus ausgeschaltet?
    {
      if (!initializing)
      // Nicht am Initialisieren
      {
        if (fuehlerKollektorLuft.getMeanTemperature() > SOLE_START_TEMPERATURE)
        // Kollektor-Luft ist heiss genug für Solemodus
        {
          soleModusStart();
          sendMQTT("message", "Solemodus aus ausgeschaltenem Modus aufgrund hoher Kollektor-Luft-Temperatur gestartet. Initialisierung beginnt.");
        }
      }
    }
    break;
    case 1: // Im Modus Sole?
    {
      if (tooLowValue)
      // Sollwert bereits unterschritten
      {
        if ((fuehlerSoleVL.getMeanTemperature() > SOLE_VL_EXIT_TEMPERATURE) && (fuehlerSole.getMeanTemperature() > SOLE_EXIT_TEMPERATURE))
        // Sollwerte fur Solemodus alle wieder erreicht?
        {
          if (fuehlerSoleVL.getMeanTemperature() - fuehlerSoleRL.getMeanTemperature() > MIN_DIFFERENZ_VL_RL_SOLE)
          {
            tooLowValue = false; // Abbruch abbrechen
          }
        }
      }

      if (initializing)
      // Am Initialisieren
      {
        if ((fuehlerKollektorVL.getMeanTemperature() > fuehlerBoiler1.getMeanTemperature() + MIN_WAERMER_KOLLEKTOR_VL_BOILER) && (fuehlerSoleVL.getMeanTemperature() > fuehlerBoiler1.getMeanTemperature()))
        // Kollektor und Sole VL so heiss, dass Boiler geladen werden kann? --> Direkt zu Boiler umschalten.
        {
          boilerModusStart();
          sendMQTT("message", "Boilermodus direkt aus Sole-Modus aufgrund hoher Vorlauf Temperaturen gestartet. Initialisierung beginnt.");
        }
      }
      else
      // Nicht am Initialisieren
      {
        if (fuehlerKollektorVL.getMeanTemperature() > fuehlerBoiler1.getMeanTemperature() + MIN_WAERMER_KOLLEKTOR_VL_BOILER)
        // Kollektor VL genug heiss fuer Boilermodus
        {
          boilerModusStart();
          sendMQTT("message", "Boilermodus aus Sole-Modus aufgrund hoher Kollektor-Vorlauf Temperaturen gestartet. Initialisierung beginnt.");
        }
        else if ((fuehlerSoleVL.getMeanTemperature() < SOLE_VL_EXIT_TEMPERATURE) || (fuehlerSole.getMeanTemperature() < SOLE_EXIT_TEMPERATURE) || (fuehlerSoleVL.getMeanTemperature() - fuehlerSoleRL.getMeanTemperature() < MIN_DIFFERENZ_VL_RL_SOLE))
        // Wurde einer der Sollwerte Unterschritten?
        {
          if (!tooLowValue)
          {
            tooLowValue = true;           // Abbruchvariable setzen
            exitTimeout.setLastTime(now); // Abbruchtimer setzen
          }
          else if (tooLowValue && exitTimeout.checkTimer(now))
          {
            turnOffModusStart();    // Anlage ausschalten
            exitTimeout.executed(); // Abbruchtimer zurücksetzen
            sendMQTT("message", "Der Solemodus wurde abgebrochen, da die Solltemperatur zu lange unterschritten wurde. Die Anlage ist jetzt ausgeschaltet.");
          }
        }
      }
    }
    break;
    case 2: // Im Modus Boiler?
    {
      if (tooLowValue)
      // Sollwert bereits unterschritten
      {
        if ((fuehlerBoilerVL.getMeanTemperature() > fuehlerBoiler1.getMeanTemperature()) && (fuehlerBoilerVL.getMeanTemperature() - fuehlerBoilerRL.getMeanTemperature() > MIN_DIFFERENZ_VL_RL_BOILER))
        // Sollwerte fur Boilermodus alle wieder erreicht?
        {
          tooLowValue = false; // Abbruch abbrechen
        }
      }

      if (!initializing)
      // Nicht mehr am Initialisieren?
      {
        if (fuehlerBoilerVL.getMeanTemperature() < fuehlerBoiler1.getMeanTemperature() - BOILER_DIRECT_EXIT_DIFF)
        // Boiler VL so kalt, dass direkt abgebrochen werden muss?
        {
          soleModusStart(); // Solemodus starten
          sendMQTT("message", "Vom Boiler-Laden zum Solemodus gewechselt, da der Boiler Vorlauf viel zu kalt war. Sole-Initialisierung beginnt.");
        }
        else if (fuehlerBoilerVL.getMeanTemperature() - fuehlerBoilerRL.getMeanTemperature() < MIN_DIFFERENZ_VL_RL_BOILER)
        // Boiler VL und Rücklauf zu nahe beieinander?
        {
          soleModusStart(); // Solemodus starten
          sendMQTT("message", "Vom Boiler-Laden zum Solemodus gewechselt, da der Boiler Vorlauf und Rücklauf zu nahe zueinander kamen. Sole-Initialisierung beginnt.");
        }
        else if (fuehlerBoilerVL.getMeanTemperature() < fuehlerBoiler1.getMeanTemperature())
        // Boiler VL hat Sollwert unterschritten?
        {
          if (!tooLowValue)
          {
            tooLowValue = true;           // Abbruchvariable setzen
            exitTimeout.setLastTime(now); // Abbruchtimer setzen
          }
          else if (tooLowValue && exitTimeout.checkTimer(now))
          {
            soleModusStart();       // In Solemodus wechseln
            exitTimeout.executed(); // Abbrucht imer zurücksetzen
            sendMQTT("message", "Vom Boiler laden zum Solemodus gewechselt, da der Boiler Vorlauf zu lange zu kühl war. Sole-Initialisierung beginnt.");
          }
        }
      }
    }
    break;
    case 3: // Im Modus Manuell?
      break;
    default:
    {
      turnOffModusStart(); // Standardmässig ausschalten
    }
    }

    // Dashbord sagen, dass online
    sendMQTT("alive", 1);

    timer5s.executed();
  }

  if (MQTTSendTimer.checkTimer(now)) // Daten an Dashboard senden
  {
    // Alle Daten an MQTT übermitteln
    sendMQTTAll();

    MQTTSendTimer.executed();
  }

  if (timer1m.checkTimer(now))
  {
    // Energieberechnung Sole Volumen, dass umgesetzt wird, bei gegebener Pumpenleistung. Berechnung erfolgt online
    if (operationMode && !(digitalRead(STELLWERK_SOLE_BOILER)) && digitalRead(RELAIS_SOLE_PUMPE))
    {
      sendMQTT("flowMeterSole", (float)(1.467 * pow(10, 14) * exp(-(pow((PIDOutputKollektorPumpe - 1352) / 193.7, 2))) + 4.173 * exp(-(pow((PIDOutputKollektorPumpe - 205) / 90.28, 2)))));
    }

    // Prüfen ob MQTT noch verbunden ist, allenfalls neu-verbinden
    if (!mqttClient.connected())
    { // wurde die Verbindung unterbrochen?
      Serial.println("MQTT-Verbindung unterbrochen, versuche Neuverbindung");
      Serial.println();
      uint8_t i = 0;

      while (!mqttClient.connected() && !mqttClient.connect(broker, port) && i < 5) // starte 5 Verbindungsversuche
      {
        Serial.print("MQTT Verbindung fehlgeschlagen! Fehlercode = ");
        Serial.println(mqttClient.connectError());
        Serial.println("Neuer Verbindungsversuch ...");
        delay(2000);
        i++;
      }
      if (mqttClient.connected())
      {
        Serial.println("MQTT-Verbindung wiederhergestellt.");
        subscribeToMQTTTopics();
      }
    }

    timer1m.executed();
  }
  if (timer3m.checkTimer(now))
  {
    // Prüfen, welcher Sendeintervall geeignet ist
    if (operationMode)
    {
      MQTTSendTimer.setDelayTime(20, 's');
    }
    else if (fuehlerKollektorLuft.getMeanTemperature() > SOLE_START_TEMPERATURE - 5)
    {
      MQTTSendTimer.setDelayTime(20, 's');
    }
    else if (fuehlerKollektorLuft.getMeanTemperature() > SOLE_START_TEMPERATURE - 10)
    {
      MQTTSendTimer.setDelayTime(5, 'm');
    }
    else
    {
      MQTTSendTimer.setDelayTime(15, 'm');
    }

    // Prüfen, ob Boiler über 65 Grad ist
    if (fuehlerBoiler1.getMeanTemperature() > 65)
    {
      timerLegionellenschaltung.setLastTime(now); // Legionellenschaltung hinauszögern
      boilerEnoughFull.setLastTime(now);          // Boiler als genuegend voll setzen
      if (boilerHighTemperatur)                   // Legionellenschaltung ausschalten, falls eingeschaltet
      {
        boilerHighTemperatur = false;
        digitalWrite(STELLWERK_BOILER_TEMP, LOW);
        boilerAdditionalHeatingOff();
        sendMQTT("boilerTermostat", "normal");
        sendMQTT("message", "Boiler wieder bei normaler Solltemperatur für elektrisches Heizen.");
      }
      sendMQTT("boilerLegionellenTemperatur", "Temperatur erreicht");
    }

    // Aktuelle System-Zeit senden
    sendMQTT("timeHours", timeClient.getHours());
    sendMQTT("timeMinutes", timeClient.getMinutes());

    // Helligkeitswert auslesen
    uint16_t lum = luxMitte.getLuminosity(TSL2591_FULLSPECTRUM);
    uint16_t ir = luxMitte.getLuminosity(TSL2591_INFRARED);
    uint16_t vis = luxMitte.getLuminosity(TSL2591_VISIBLE);

    float lux = luxMitte.calculateLux(lum, ir);

    sendMQTT("luxMiddle", lux);
    sendMQTT("fullMiddle", (unsigned long)lum);
    sendMQTT("visibleMiddle", (unsigned long)vis);
    sendMQTT("irMiddle", (unsigned long)ir);

    // Debugging
    Serial.println("Ich lebe!");

    timer3m.executed();
  }
  if (((fuehlerBoiler1.getMeanTemperature() + fuehlerBoiler2.getMeanTemperature()) / 2) < 50 && !operationMode)
  {
    boilerAdditionalHeatingOn();
  }
  else if ((((fuehlerBoiler1.getMeanTemperature() + fuehlerBoiler2.getMeanTemperature()) / 2) > 55 && boilerHighTemperatur == false) || operationMode)
  {
    boilerAdditionalHeatingOff();
  }

  if (timerLegionellenschaltung.checkTimer(now))
  {
    digitalWrite(STELLWERK_BOILER_TEMP, HIGH); // Boiler auf höhere Temperatur stellen
    boilerHighTemperatur = true;
    boilerTimeout.setLastTime(now);       // Timer stellen
    timerLegionellenschaltung.executed(); // Timer zurücksetzen
    boilerAdditionalHeatingOn();
    sendMQTT("message", "Boiler bei erhöhter Solltemperatur für elektrisches Heizen. (Legionellenschaltung)");
    sendMQTT("boilerTermostat", "hoch");
  }

  if (boilerHighTemperatur && (boilerTimeout.checkTimer(now) || fuehlerBoiler1.getMeanTemperature() > BOILER_FULL_TEMPERATURE))
  {
    boilerHighTemperatur = false;
    digitalWrite(STELLWERK_BOILER_TEMP, LOW);
    boilerAdditionalHeatingOff();
    sendMQTT("boilerTermostat", "normal");
    sendMQTT("message", "Boiler wieder bei normaler Solltemperatur für elektrisches Heizen.");
  }

  now = millis();
}
