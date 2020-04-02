//Software für die Funktionen gemäss README-Datei
//
//STRUKTUR:
// 1. HEADER-Dateien, Definitionen und Libaries
// 2. Variabeln und Konstanten
// 3. PIN-Adressen und BUS-Adressierung
// 4. Initialisierungen
// 5. Funktionen
// 6. Setup Sequenz
// 7. Programmschleife

// ============================================
// 1. HEADER-Dateien, Definitionen und Libaries
// ============================================
#include "Wire.h"                   //Für I2C Bus
#include "PID_v1.h"                 //Für PID-Regelung
#include "Adafruit_LiquidCrystal.h" //Für Displays
#include "Classes.h"                //Eigene Klassen
#include <SPI.h>                    //Ethernet
#include <Ethernet.h>               //Ethernet
#include <ArduinoMqttClient.h>      //MQTT
#include "secrets.h"                //MQTT Passwörter

// ===========================
// 2. Variabeln und Konstanten
// ===========================
// Konstanten
//Korrekturfaktoren
const float FUEHLER_KOLLEKTOR_VL_KORREKTURFAKTOR = 1.0;   //Korrekturfaktor S0
const float FUEHLER_KOLLEKTOR_LUFT_KORREKTURFAKTOR = 1.0; //Korrekturfaktor S1
const float FUEHLER_BOILER_1_KORREKTURFAKTOR = 1.0;       //Korrekturfaktor S2 oben
const float FUEHLER_BOILER_2_KORREKTURFAKTOR = 1.0;       //Korrekturfaktor S2 mitte
const float FUEHLER_BOILER_3_KORREKTURFAKTOR = 1.0;       //Korrekturfaktor S2 unten
const float FUEHLER_BOILER_VL_KORREKTURFAKTOR = 1.0;      //Korrekturfaktor S4B
const float FUEHLER_SOLE_VL_KORREKTURFAKTOR = 1.0;        //Korrekturfaktor S4S
const float FUEHLER_BOILER_RL_KORREKTURFAKTOR = 1.0;      //Korrekturfaktor S6B
const float FUEHLER_SOLE_RL_KORREKTURFAKTOR = 1.0;        //Korrekturfaktor S6S
const float FUEHLER_SOLE_KORREKTURFAKTOR = 1.0;           //Korrekturfaktor S7

//Temperaturen
const int ALARMTEMPERATUR_KOLLEKTOR_LUFT = 90;    //Alarmtemperatur für Kollektor (Lufttemperatur)
const int ALARMTEMPERATUR_KOLLEKTOR_VL = 95;      //Alarmtemperatur für Kollektor (Vorlauftemperatur)
const int ALARMTEMPERATUR_SOLE = 30;              //Alarmtemperatur für Sole-Pumpe
const int ACHTUNG_TEMPERATUR_SOLE = 20;           //Alarmtemperatur für Sole-Pumpe
const int MIN_DIFFERENZ_NACH_ALARM = 3;           //erst wenn die Temperatur um diesen Wert gesunken ist, geht der Alarmmodus aus
const int SOLE_EXIT_TEMPERATURE = 6;              //Temperatur zur Sonde, bei der der Solemodus abgebrochen wird
const int SOLE_START_TEMPERATURE = 25;            //Temperatur im Kollektor (Luft), bei der der Solemodus gestartet wird
const int SOLE_VL_EXIT_TEMPERATURE = 23;          //Temperatur im Sole Wärmetauscher VL, bei der der Solemodus abgebrochen wird
const int MIN_KOLLEKTOR_LUFT = 50;                //Mindest-Regeltemperatur für den Kollektor-Vorlauf
const int MAX_KOLLEKTOR_LUFT = 80;                //Maximal-Regeltemperatur für Kollektor Vorlauf
const int SOLL_KOLLEKTOR_LUFT_SOLEMODUS_2 = 50;   //Solltemperatzr, auf die der Kollektor VL geregelt werden soll, wenn Solemodus
const int BOILER_DIRECT_EXIT_DIFF = -4;           //Maximaler Temperaturunterschied zwischen Boilertemperatur und Boiler VL bevor direkter Abbruch
const int MIN_DIFFERENZ_VL_RL_BOILER = 8;         //Wenn VL und RL weniger als Diese Differenz haben, wird der Modus abgebrochen
const int MIN_DIFFERENZ_VL_RL_SOLE = 5;           //Wenn VL und RL weniger als Diese Differenz haben, wird der Modus abgebrochen
const int MIN_WAERMER_KOLLEKTOR_VL_BOILER = 4;    //mindest Temperaturunterschied zwischen Boiler und Kollektor VL, bei dem der Boilermodus gestartet wird
const int MIN_WAERMER_KOLLEKTOR_LUFT_BOILER = 10; //mindest Temperaturunterschied zwischen Boiler und Kollektor LUFT, bei dem der Boilermodus gestartet wird

//PID Tuning Parameter
const double PID_P_KOLLEKTOR = 5;         //Verstärkung des Proportionalen Anteils des PID-Reglers der Kollektorpumpe
const double PID_I_KOLLEKTOR = 0.01;      //Verstärkung des Integralen Anteils des PID-Reglers der Kollektorpumpe
const double PID_D_KOLLEKTOR = 0;         //Verstärkung des differentialen Anteils des PID-Reglers der Kollektorpumpe
const byte PID_KOLLEKTOR_MIN_SPEED = 70;  //Minimale Kollektorpumpen-Geschwindigkeit
const byte PID_KOLLEKTOR_MAX_SPEED = 255; //Maximale Kollektorpumpen-Geschwindigkeit

// Ethernet Konstanten
byte mac[] = {0xA8, 0x61, 0x0A, 0xAE, 0x3D, 0xB7}; //Hardware-Adresse des Ethernet-Boards (Kleber auf Board)

//Variabeln
byte operationMode = 0;              //Betriebsmodus: 0=aus, 1=Solemodus, 2=Boilermodus
unsigned long now = 0;               //Jeweils aktueller millis()-Wert
bool displayOn = false;              //true, wenn Displays eingeschaltet sein soll
bool boilerHighTemperatur = false;   //true, wenn Boiler auf höherer Temperatur ist
bool kollektorAlarm = false;         //true, wenn kollektor zu heiss ist
bool soleAlarm = false;              //true, wenn solepumpe zu heiss ist
bool initializing = false;           //Ist derzeit ein neuer Modus am Initialisieren?
bool tooLowValue = false;            //True, wenn Sollwert das erste mal unterschritten
bool lastStateFlowMeterBoiler = LOW; //Lester Status des Flow Meter Boiler
bool lastStateFlowMeterSole = LOW;   //Lester Status des Flow Meter Boiler

//PID Variabeln
double PIDInputKollektorPumpe = 0;         //PID Input
double PIDOutputKollektorPumpe = 0;        //PID Output
double PIDSetpointKollektorPumpe = 50;     //PID Setpoint
double lastPIDSetpointKollektorPumpe = 50; //letzter PID Setpoint

// ====================================
// 3. PIN-Adressen und BUS-Adressierung
// ====================================

// Analog Pins
const byte FUEHLER_KOLLEKTOR_VL_PIN = A7;   //S0 Anschluss PIN
const byte FUEHLER_KOLLEKTOR_LUFT_PIN = A0; //S1 Anschluss PIN
const byte FUEHLER_BOILER_1_PIN = A1;       //S2 oben Anschluss PIN
const byte FUEHLER_BOILER_2_PIN = A14;      //S2 mitte Anschluss PIN
const byte FUEHLER_BOILER_3_PIN = A13;      //S2 unten Anschluss PIN
const byte FUEHLER_BOILER_VL_PIN = A2;      //S4B Anschluss PIN
const byte FUEHLER_SOLE_VL_PIN = A3;        //S4S Anschluss PIN
const byte FUEHLER_BOILER_RL_PIN = A4;      //S6B Anschluss PIN
const byte FUEHLER_SOLE_RL_PIN = A5;        //S6S Anschluss PIN
const byte FUEHLER_SOLE_PIN = A6;           //S7 Anschluss PIN

// Digital In Pins
const byte FLOW_METER_BOILER = 32; //Anschluss PIN Durchflussmesser Boilerkreis
const byte FLOW_METER_SOLE = 34;   //Anschluss PIN Durchflussmesser Solekreis

// Digital Out Pins
const byte RELAIS_SOLE_PUMPE = 23;      //Relais-Ausgang Solepumpe
const byte RELAIS_KOLLEKTOR_PUMPE = 25; //Relais-Ausgang Kollektorpumpe
const byte STELLWERK_SOLE_BOILER = 27;  //Relais-Ausgang Stellwerk-Legionellen, high = höhere Temperatur
const byte STELLWERK_BOILER_TEMP = 29;  //Relais-Ausgang Stellwerk Sole/Boiler, high = Boiler

// PWM Pins
const byte PWM_KOLLEKTOR_PUMPE = 9; //PWM-Ausgang Kollektorpumpe

// ====================
// 4. INITIALISIERUNGEN
// ====================

// Setup aller PT1000 Fühler
// Schema: PT1000 <Name des Fühlers>(<PIN>,<Vorwiderstand>,<Korrekturfaktor>);
PT1000 fuehlerKollektorVL(FUEHLER_KOLLEKTOR_VL_PIN, 1000, FUEHLER_KOLLEKTOR_VL_KORREKTURFAKTOR);
PT1000 fuehlerKollektorLuft(FUEHLER_KOLLEKTOR_LUFT_PIN, 1000, FUEHLER_KOLLEKTOR_LUFT_KORREKTURFAKTOR);
PT1000 fuehlerBoiler1(FUEHLER_BOILER_1_PIN, 1000, FUEHLER_BOILER_1_KORREKTURFAKTOR);
PT1000 fuehlerBoiler2(FUEHLER_BOILER_2_PIN, 1000, FUEHLER_BOILER_2_KORREKTURFAKTOR);
PT1000 fuehlerBoiler3(FUEHLER_BOILER_3_PIN, 1000, FUEHLER_BOILER_3_KORREKTURFAKTOR);
PT1000 fuehlerBoilerVL(FUEHLER_BOILER_VL_PIN, 1000, FUEHLER_BOILER_VL_KORREKTURFAKTOR);
PT1000 fuehlerSoleVL(FUEHLER_SOLE_VL_PIN, 1000, FUEHLER_SOLE_VL_KORREKTURFAKTOR);
PT1000 fuehlerBoilerRL(FUEHLER_BOILER_RL_PIN, 1000, FUEHLER_BOILER_RL_KORREKTURFAKTOR);
PT1000 fuehlerSoleRL(FUEHLER_SOLE_RL_PIN, 1000, FUEHLER_SOLE_RL_KORREKTURFAKTOR);
PT1000 fuehlerSole(FUEHLER_SOLE_PIN, 1000, FUEHLER_SOLE_KORREKTURFAKTOR);

// Setup der PWM gesteuerten Pumpen
Pump kollektorPumpe(RELAIS_KOLLEKTOR_PUMPE, PWM_KOLLEKTOR_PUMPE);

// Setup der Timer (leer für ms, 's' für s, 'm' für min, 'd' für Tage)
// Timer über normale Clock
Timer timer1s(1, 's'); //1s Timer
Timer timer5s(5, 's'); //5s Timer
Timer timer1m(1, 'm'); //1m Timer
Timer timer3m(3, 'm'); //3min Timer

//Timer für bestimmte Funktionen
Timer timerLegionellenschaltung(7, 'd');   //Legio-Timer (Zeit. nach der elektrisch auf hohe Boilertemperatur geheizt wird, falls diese nie erreicht wurde)
Timer initialOperationModeTimeout(3, 'm'); //Zeit bevor ein Modus EXIT-Kriterien berücksichtigt (Achtung, Variabel)
Timer exitTimeout(2, 'm');                 //Solange muss der Sollwert mindestens unterschritten sein, bevor Abbruch
Timer flowMeterBoilerTimeout(1, 's');      //Durchflussmeter 1 Timeout
Timer flowMeterSoleTimeout(1, 's');        //Durchflussmeter 2 Timeout
Timer displayButtonTimeout(1000);          //Display-Button Timeout
Timer displayTimeout(2, 'm');              //Display-Ausschaltzeit
Timer boilerTimeout(1, 'd');               //Boiler-Ausschaltzeit
Timer MQTTSendTimer(5, 's');               //Sendeinterval Daten an Dashboard (Achtung, Variabel)

//PWM Setup
PID PIDReglerKollektorPumpe(&PIDInputKollektorPumpe, &PIDOutputKollektorPumpe, &PIDSetpointKollektorPumpe, PID_P_KOLLEKTOR, PID_I_KOLLEKTOR, PID_D_KOLLEKTOR, REVERSE);

//MQTT Setup
EthernetClient client;
MqttClient mqttClient(client);

const char broker[] = "storage.moser-artz.ch"; //MQTT-Broker-Server
int port = 1883;                               //MQTT-Broker-Server-Port
String topic = "derart/";                      //Standard MQTT-Topic
String recievedTopic;                          //Empfangenes Thema
char recievedPayload;                          //Empfangenes Zeichen

// =============
// 5. FUNKTIONEN
// =============

//Sendet eine MQTT-Message mit dem Thema <subtopic> und dem Fliesskommazahl-Wert <value>
void sendMQTT(String subtopic, float value)
{
  mqttClient.beginMessage(topic + subtopic);
  mqttClient.print(value);
  mqttClient.endMessage();
}

//Sendet eine MQTT-Message mit dem Thema <subtopic> und dem ganzzahligen Wert <value>
void sendMQTT(String subtopic, int value)
{
  mqttClient.beginMessage(topic + subtopic);
  mqttClient.print(value);
  mqttClient.endMessage();
}

// //Sendet eine MQTT-Message mit dem Thema <subtopic> und dem String <value>
// void sendMQTT(String subtopic, const char value[])
// {
//   mqttClient.beginMessage(topic + subtopic);
//   mqttClient.print(value);
//   mqttClient.endMessage();
// }

//Sendet eine MQTT-Message mit dem Thema <subtopic> und dem String <value>
void sendMQTT(String subtopic, String value)
{
  int valueLength = value.length();
  char charValue[valueLength];
  value.toCharArray(charValue, valueLength);
  mqttClient.beginMessage(topic + subtopic);
  mqttClient.print(charValue);
  mqttClient.endMessage();
}

// Gibt Soll Vorlauftemperatur am Kollektor zurücksetzen
void calculateTargetTemperature()
{
  if (operationMode) //läuft die Anlage?
  {
    //Different zwischen Kollektor Luft und VL bestimmen
    float differenzLuftVL = fuehlerKollektorVL.getMeanTemperature() - fuehlerKollektorLuft.getMeanTemperature();
    if (differenzLuftVL > 5)
    {
      differenzLuftVL = 5;
    }
    else if (differenzLuftVL < 0)
    {
      differenzLuftVL = 0;
    }

    if (digitalRead(STELLWERK_SOLE_BOILER)) //Boilermodus?
    {
      if (!initializing)
      {
        if (fuehlerBoilerVL.getMeanTemperature() - fuehlerBoilerRL.getMeanTemperature() < MIN_DIFFERENZ_VL_RL_BOILER + 2) //VL und RL nähern sich zu stark an
        {
          PIDSetpointKollektorPumpe = ceil(fuehlerBoiler1.getMeanTemperature() + MIN_WAERMER_KOLLEKTOR_VL_BOILER + 7 - differenzLuftVL); //Solltemperatur für Boilermodus
        }
        else if (fuehlerBoilerVL.getMeanTemperature() - fuehlerBoilerRL.getMeanTemperature() > MIN_DIFFERENZ_VL_RL_BOILER + 5) //VL und RL sind weit voneinander entfernt
        {
          PIDSetpointKollektorPumpe = ceil(fuehlerBoiler1.getMeanTemperature() + MIN_WAERMER_KOLLEKTOR_VL_BOILER + 2 - differenzLuftVL); //Solltemperatur für Boilermodus
        }
        else if (PIDSetpointKollektorPumpe < fuehlerBoiler1.getMeanTemperature() + MIN_WAERMER_KOLLEKTOR_VL_BOILER + 2 - differenzLuftVL)
        {
          PIDSetpointKollektorPumpe = ceil(fuehlerBoiler1.getMeanTemperature() + MIN_WAERMER_KOLLEKTOR_VL_BOILER + 2 - differenzLuftVL); //Solltemperatur für Boilermodus
        }
      }
      else //nicht am Initialisieren im Boilermodus
      {
        PIDSetpointKollektorPumpe = ceil(fuehlerBoiler1.getMeanTemperature() + MIN_WAERMER_KOLLEKTOR_VL_BOILER + 2 - differenzLuftVL); //Solltemperatur für Boilermodus
      }
    }

    else //Solemodus?
    {
      PIDSetpointKollektorPumpe = ceil(fuehlerBoiler1.getMeanTemperature() + MIN_WAERMER_KOLLEKTOR_VL_BOILER + 2 - differenzLuftVL); //Solltemperatur für Solemodus
    }

    if (PIDSetpointKollektorPumpe < 50)
    {
      PIDSetpointKollektorPumpe = 50;
    }
    else if (PIDSetpointKollektorPumpe > 80)
    {
      PIDSetpointKollektorPumpe = 80;
    }

    //Solltemperatur verändert?
    if (PIDSetpointKollektorPumpe != lastPIDSetpointKollektorPumpe)
    {
      lastPIDSetpointKollektorPumpe = PIDSetpointKollektorPumpe;
      Serial.print("Neue Regel-Solltemperatur: ");
      Serial.println(PIDSetpointKollektorPumpe, 1);
      sendMQTT("message", (String) "Neue Regel-Solltemperatur: " + ceil(PIDSetpointKollektorPumpe));
    }
  }
}

//Startet den Boilermodus
void boilerModusStart()
{
  digitalWrite(RELAIS_KOLLEKTOR_PUMPE, HIGH);       //Kollektorpumpe einschalten
  digitalWrite(RELAIS_SOLE_PUMPE, LOW);             //Solepumpe ausschalten
  digitalWrite(STELLWERK_SOLE_BOILER, HIGH);        //Stellwerk auf Boiler umschalten
  PIDReglerKollektorPumpe.SetMode(1);               //PID-Regler Kollektorpumpe einschalten
  initializing = true;                              //Initialisierung starten
  initialOperationModeTimeout.setDelayTime(6, 'm'); //Initialisierungszeit setzen
  initialOperationModeTimeout.setLastTime(now);     //Initialisierungs Timer starten
  operationMode = 2;                                //Modus auf Boiler schalten
  tooLowValue = false;                              //tooLowValue zurücksetzen
  MQTTSendTimer.setDelayTime(5, 's');
}

//Startet den Solemodus
void soleModusStart()
{
  digitalWrite(RELAIS_KOLLEKTOR_PUMPE, HIGH); //Kollektorpumpe ausschalten
  digitalWrite(RELAIS_SOLE_PUMPE, HIGH);      //Solepumpe einschalten
  digitalWrite(STELLWERK_SOLE_BOILER, LOW);   //Stellwerk auf Sole umschalten
  PIDReglerKollektorPumpe.SetMode(1);         //PID-Regler Kollektorpumpe einschalten
  initializing = true;                        //Initialisierung starten
  switch (operationMode)                      //Initialisierungszeit setzen
  {
  case 0:                                              //War ausgeschaltet?
    initialOperationModeTimeout.setDelayTime(15, 'm'); //Initialisierungszeit, wenn Modus 0-->1
    break;
  case 2:                                             //War im Boilermodus??
    initialOperationModeTimeout.setDelayTime(3, 'm'); //Initialisierungszeit, wenn MNodus 2-->1
    break;
  }
  initialOperationModeTimeout.setLastTime(now); //Initialisierungs Timer starten
  operationMode = 1;                            //Modus auf Sole schalten
  tooLowValue = false;                          //tooLowValue zurücksetzen
  MQTTSendTimer.setDelayTime(5, 's');
}

//Schaltet die Anlage aus
void turnOffModusStart()
{
  digitalWrite(RELAIS_KOLLEKTOR_PUMPE, LOW);        //Kollektorpumpe ausschalten
  digitalWrite(RELAIS_SOLE_PUMPE, LOW);             //Solepumpe ausschalten
  digitalWrite(STELLWERK_SOLE_BOILER, LOW);         //Stellwerk auf Sole umschalten
  PIDReglerKollektorPumpe.SetMode(0);               //PID-Regler Kollektorpumpe ausschalten
  PIDOutputKollektorPumpe = 0;                      //Speed auf Null setzen
  kollektorPumpe.stop();                            //kollektorpumpe stoppen
  initialOperationModeTimeout.setDelayTime(3, 'm'); //Wie lange mindestens ausgeschaltet?
  initialOperationModeTimeout.setLastTime(now);     //Initialisierungs Timer starten
  initializing = true;                              //Initialisierung starten
  operationMode = 0;                                //Modus auf aus schalten
  tooLowValue = false;                              //tooLowValue zurücksetzen
}

void subscribeToMQTTTopics()
{
  mqttClient.subscribe("derart/toArduino/LegionellenModus");
  mqttClient.subscribe("derart/toArduino/Betriebsmodus");
  mqttClient.subscribe("derart/toArduino/SpeedKollektorpumpe");
}

// ================
// 6. Setup-Sequenz
// ================
void setup()
{
  //Serielle Schnittstelle starten
  Serial.begin(9600);
  while (!Serial)
  {
    ; // warten, bis die Serielle Schnittstelle sich verbindet
  }
  Serial.println("Setup gestartet");

  //PID Setup
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

  //MQTT Verbindung
  Serial.print("Versuch, sich mit dem MQTT Broker zu verbinden ");
  mqttClient.setUsernamePassword(MQTT_USER, MQTT_PASS); //User und Passwort von MQTT
  Serial.println(broker);                               //Name des Broker-Servers ausgeben

  while (!mqttClient.connect(broker, port))
  {
    Serial.print("MQTT Verbindung fehlgeschlagen! Fehlercode = ");
    Serial.println(mqttClient.connectError());
    Serial.println("Neuer Verbindungsversuch ...");
    delay(1500);
  }

  Serial.println("Du bist mit dem MQTT Broker verbunden!");
  Serial.println();

  subscribeToMQTTTopics();

  //erster Betriebsmodus nach dem Starten (Ohne Initialisierungszeit)
  turnOffModusStart();
  initializing = false;

  Serial.println("Setup beendet");
}

// ===================
// 7. Programmschleife
// ===================
void loop()
{
  //Dieser Programmteil wird in jeder Schleife durchgefuehrt

  //Prüfen, ob der Durchflussmesser Boiler einen Impuls abgeben
  if (flowMeterBoilerTimeout.checkTimer(now))
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

  //Nachrichten empfangen und verarbeiten
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
        sendMQTT("message", "Pumpen-Geschwindigkeit wieder bei Auto.");
        break;
      case '1':
        PIDReglerKollektorPumpe.SetOutputLimits(70, 71);
        sendMQTT("message", "Kollektorpumpe: Manuell sehr langsam");
        break;
      case '2':
        PIDReglerKollektorPumpe.SetOutputLimits(110, 111);
        sendMQTT("message", "Kollektorpumpe: Manuell langsam");
        break;
      case '3':
        PIDReglerKollektorPumpe.SetOutputLimits(150, 151);
        sendMQTT("message", "Kollektorpumpe: Manuell mittel");
        break;
      case '4':
        PIDReglerKollektorPumpe.SetOutputLimits(190, 191);
        sendMQTT("message", "Kollektorpumpe: Manuell schnell");
        break;
      case '5':
        PIDReglerKollektorPumpe.SetOutputLimits(250, 255);
        sendMQTT("message", "Kollektorpumpe: Manuell sehr schnell");
        break;
      }
    }
  }

  //PID Regler berechnen
  if (PIDReglerKollektorPumpe.Compute())
  {
    kollektorPumpe.setSpeed(PIDOutputKollektorPumpe); //neuen Speed Kollektorpumpe setzen
  }

  //Dieser Programmteil wird alle 1s ausgeführt
  if (timer1s.checkTimer(now))
  {
    //Alle Fühler auslesen
    fuehlerKollektorVL.calculateTemperature();
    fuehlerKollektorLuft.calculateTemperature();
    fuehlerBoiler1.calculateTemperature();
    fuehlerBoiler2.calculateTemperature();
    fuehlerBoiler3.calculateTemperature();
    fuehlerBoilerVL.calculateTemperature();
    fuehlerSoleVL.calculateTemperature();
    fuehlerBoilerRL.calculateTemperature();
    fuehlerSoleRL.calculateTemperature();
    fuehlerSole.calculateTemperature();

    PIDInputKollektorPumpe = fuehlerKollektorLuft.getMeanTemperature(); //Kollektor Vorlauftemperatur in PID schreiben

    //Ethernet aktuell halten
    Ethernet.maintain();

    // MQTT Client am Leben halten
    mqttClient.poll();

    //Timer zurücksetzen
    timer1s.executed();
  }

  //Dieser Programmteil wird alle 5s ausgeführt
  if (timer5s.checkTimer(now))
  {
    //Überprüfen, ob Alarmwerte überschritten wurden
    if (fuehlerSole.getLastTemperature() > ALARMTEMPERATUR_SOLE)
    {
      soleAlarm = true;
    }
    if (fuehlerKollektorLuft.getLastTemperature() > ALARMTEMPERATUR_KOLLEKTOR_LUFT || fuehlerKollektorVL.getLastTemperature() > ALARMTEMPERATUR_KOLLEKTOR_VL)
    {
      kollektorAlarm = true;
    }

    //Soll Kollektor VL temperatur berechnen
    calculateTargetTemperature();

    //Alarmmodus
    //Sole AlArm?
    if (soleAlarm)
    {
      sendMQTT("alarm", "Alarm. Die Soletemperatur ist zu hoch."); //Alarmnachricht an Dashboard
      digitalWrite(RELAIS_SOLE_PUMPE, LOW);                        //Solepumpe ausschalten
      sendMQTT("solePumpe", 0);                                    //Solepumpen-Status an Dashboard
      digitalWrite(STELLWERK_SOLE_BOILER, LOW);                    //Wärme über Wärmetauscher abkühlen
      analogWrite(PWM_KOLLEKTOR_PUMPE, 255);                       //Kollektorpumpe auf 100%
      sendMQTT("speedKollektorPumpe", 255);                        //Info an Dashboard, Kollektorpumpe bei 100%
      sendMQTT("stellwerkSoleBoiler", 1);                          //Info an Dashboard, Stellwerk zeigt auf Boiler
    }

    //Kollektor Alarm?
    if (kollektorAlarm)
    {
      sendMQTT("alarm", "Alarm. Die Kollektortemperatur ist zu hoch.");
      digitalWrite(RELAIS_KOLLEKTOR_PUMPE, HIGH); //Kollektorpumpe einschalten
      analogWrite(PWM_KOLLEKTOR_PUMPE, 255);      //Kollektorpumpe auf 100%
      sendMQTT("speedKollektorPumpe", 255);       //Info an Dashboard, Kollektorpumpe bei 100%
      sendMQTT("kollektorPumpe", 1);              //Kollektorpumpe-Status an Dashboard
      digitalWrite(STELLWERK_SOLE_BOILER, HIGH);  //Wärme in Boiler leiten
      sendMQTT("stellwerkSoleBoiler", 2);         //Info an Dashboard, Stellwerk zeigt auf Boiler
    }

    while (soleAlarm || kollektorAlarm)
    {
      //Sole Temperatur abfragen und prüfen, ob ausreichend abgekühlt
      fuehlerSole.calculateTemperature();
      if (fuehlerSole.getLastTemperature() < ALARMTEMPERATUR_SOLE - MIN_DIFFERENZ_NACH_ALARM && soleAlarm)
      {
        soleAlarm = false;
        sendMQTT("message", "Die Soletemperatur ist nicht mehr zu hoch."); //Dashboard Nachricht: Alarm beendet
        soleModusStart();                                                  //Nach Alarm in Boilermodus starten
      }

      //Kollektor VL und Luft Temperatur abfragen und prüfen, ob ausreichend abgekühlt
      fuehlerKollektorLuft.calculateTemperature();
      fuehlerKollektorVL.calculateTemperature();
      if (fuehlerKollektorVL.getLastTemperature() < ALARMTEMPERATUR_KOLLEKTOR_VL - MIN_DIFFERENZ_NACH_ALARM && fuehlerKollektorLuft.getLastTemperature() < ALARMTEMPERATUR_KOLLEKTOR_LUFT - MIN_DIFFERENZ_NACH_ALARM && kollektorAlarm)
      {
        kollektorAlarm = false;
        sendMQTT("message", "Die Kollektortemperatue ist nicht mehr zu hoch."); //Dashboard Nachricht: Alarm beendet
        boilerModusStart();                                                     //Nach Alarm in Boilermodus starten
      }
      delay(500);
    }

    //Prüfen, ob Initialisierung abgeschlossen ist
    if (initialOperationModeTimeout.checkTimer(now) && initializing)
    {
      sendMQTT("message", "Die Initialisierung des aktuellen Betriebsmodus ist abgeschlossen.");
      initializing = false;
    }

    //Entscheidung Betriebsmodus
    switch (operationMode)
    {
    case 0: //Im Modus ausgeschaltet?
    {
      if (!initializing)
      //Nicht am Initialisieren
      {
        if (fuehlerKollektorLuft.getMeanTemperature() > fuehlerBoiler1.getMeanTemperature() + MIN_WAERMER_KOLLEKTOR_LUFT_BOILER)
        //Kollektor Luft ist heiss genug für Boilermodus
        {
          boilerModusStart();
          sendMQTT("message", "Boilermodus aus ausgeschaltenem Modus aufgrund hoher Kollektur-Luft-Temperatur gestartet. Initialisierung beginnt.");
        }
        else if (fuehlerKollektorLuft.getMeanTemperature() > SOLE_START_TEMPERATURE)
        //Kollektor-Luft ist heiss genug für Solemodus
        {
          soleModusStart();
          sendMQTT("message", "Solemodus aus ausgeschaltenem Modus aufgrund hoher Kollektor-Luft-Temperatur gestartet. Initialisierung beginnt.");
        }
      }
    }
    break;
    case 1: //Im Modus Sole?
    {
      if (tooLowValue)
      //Sollwert bereits unterschritten
      {
        if ((fuehlerSoleVL.getMeanTemperature() > SOLE_VL_EXIT_TEMPERATURE) && (fuehlerSole.getMeanTemperature() > SOLE_EXIT_TEMPERATURE))
        //Sollwerte fur Solemodus alle wieder erreicht?
        {
          if (fuehlerSoleVL.getMeanTemperature() - fuehlerSoleRL.getMeanTemperature() > MIN_DIFFERENZ_VL_RL_SOLE)
          {
            tooLowValue = false; //Abbruch abbrechen
            sendMQTT("message", "Solltemperatur für Solemodus wurde wieder erreicht, Modus wird nicht abgebrochen.");
          }
        }
      }

      if (initializing)
      //Am Initialisieren
      {
        if ((fuehlerKollektorVL.getMeanTemperature() > fuehlerBoiler1.getMeanTemperature() + MIN_WAERMER_KOLLEKTOR_VL_BOILER) && (fuehlerSoleVL.getMeanTemperature() > fuehlerBoiler1.getMeanTemperature()))
        //Kollektor und Sole VL so heiss, dass Boiler geladen werden kann? --> Direkt zu Boiler umschalten.
        {
          boilerModusStart();
          sendMQTT("message", "Boilermodus direkt aus Sole-Modus aufgrund hoher Vorlauf Temperaturen gestartet. Initialisierung beginnt.");
        }
      }
      else
      //Nicht am Initialisieren
      {
        if (fuehlerKollektorVL.getMeanTemperature() > fuehlerBoiler1.getMeanTemperature() + MIN_WAERMER_KOLLEKTOR_VL_BOILER)
        //Kollektor VL genug heiss fuer Boilermodus
        {
          boilerModusStart();
          sendMQTT("message", "Boilermodus aus Sole-Modus aufgrund hoher Kollektor-Vorlauf Temperaturen gestartet. Initialisierung beginnt.");
        }
        else if ((fuehlerSoleVL.getMeanTemperature() < SOLE_VL_EXIT_TEMPERATURE) || (fuehlerSole.getMeanTemperature() < SOLE_EXIT_TEMPERATURE) || (fuehlerSoleVL.getMeanTemperature() - fuehlerSoleRL.getMeanTemperature() < MIN_DIFFERENZ_VL_RL_SOLE))
        //Wurde einer der Sollwerte Unterschritten?
        {
          if (!tooLowValue)
          {
            tooLowValue = true;           //Abbruchvariable setzen
            exitTimeout.setLastTime(now); //Abbruchtimer setzen
            sendMQTT("message", "Solltemperatur für Solemodus wurde unterschritten, Modus wird demnächst abgebrochen.");
          }
          else if (tooLowValue && exitTimeout.checkTimer(now))
          {
            turnOffModusStart();    //Anlage ausschalten
            exitTimeout.executed(); //Abbruchtimer zurücksetzen
            sendMQTT("message", "Der Solemodus wurde abgebrochen, da die Solltemperatur zu lange unterschritten wurde. Die Anlage ist jetzt ausgeschaltet.");
          }
        }
      }

      //Temperatur herunterregeln, wenn SOLE VL zu warm wird
      if (fuehlerSole.getMeanTemperature() > ACHTUNG_TEMPERATUR_SOLE)
      {
        PIDSetpointKollektorPumpe = SOLL_KOLLEKTOR_LUFT_SOLEMODUS_2;
      }
      else if (PIDSetpointKollektorPumpe == SOLL_KOLLEKTOR_LUFT_SOLEMODUS_2)
      {
        calculateTargetTemperature();
      }
    }
    break;
    case 2: //Im Modus Boiler?
    {
      if (tooLowValue)
      //Sollwert bereits unterschritten
      {
        if ((fuehlerBoilerVL.getMeanTemperature() > fuehlerBoiler1.getMeanTemperature()) && (fuehlerBoilerVL.getMeanTemperature() - fuehlerBoilerRL.getMeanTemperature() > MIN_DIFFERENZ_VL_RL_BOILER))
        //Sollwerte fur Boilermodus alle wieder erreicht?
        {
          tooLowValue = false; //Abbruch abbrechen
          sendMQTT("message", "Solltemperatur für Boilermodus wurde wieder erreicht, Modus wird nicht abgebrochen.");
        }
      }

      if (!initializing)
      //Nicht mehr am Initialisieren?
      {
        if (fuehlerBoilerVL.getMeanTemperature() < fuehlerBoiler1.getMeanTemperature() + BOILER_DIRECT_EXIT_DIFF)
        //Boiler VL so kalt, dass direkt abgebrochen werden muss?
        {
          soleModusStart(); //Solemodus starten
          sendMQTT("message", "Vom Boiler-Laden zum Solemodus gewechselt, da der Boiler Vorlauf viel zu kalt war. Sole-Initialisierung beginnt.");
        }
        else if ((fuehlerBoilerVL.getMeanTemperature() < fuehlerBoiler1.getMeanTemperature()) || (fuehlerBoilerVL.getMeanTemperature() - fuehlerBoilerRL.getMeanTemperature() < MIN_DIFFERENZ_VL_RL_BOILER))
        //Boiler VL hat Sollwert unterschritten?
        {
          if (!tooLowValue)
          {
            tooLowValue = true;           //Abbruchvariable setzen
            exitTimeout.setLastTime(now); //Abbruchtimer setzen
            sendMQTT("message", "Solltemperatur für Boilermodus wurde unterschritten, Modus wird demnächst abgebrochen.");
          }
          else if (tooLowValue && exitTimeout.checkTimer(now))
          {
            soleModusStart();       //In Solemodus wechseln
            exitTimeout.executed(); //Abbrucht imer zurücksetzen
            sendMQTT("message", "Vom Boiler laden zum Solemodus gewechselt, da der Boiler Vorlauf zu lange zu kühl war. Sole-Initialisierung beginnt.");
          }
        }
      }
    }
    break;
    case 3: //Im Modus Manuell?
      break;
    default:
    {
      turnOffModusStart(); //Standardmässig ausschalten
    }
    }

    //Dashbord sagen, dass online
    sendMQTT("alive", 1);

    timer5s.executed();
  }

  if (MQTTSendTimer.checkTimer(now)) //Daten an Dashboard senden
  {
    //Fuehlerwerte an MQTT Senden
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

    //Speed der Kollektorpumpe an Dashboard senden
    sendMQTT("speedKollektorPumpe", (int)round(PIDOutputKollektorPumpe));

    //Informationen an Dashboard
    sendMQTT("kollektorPumpe", digitalRead(RELAIS_KOLLEKTOR_PUMPE));         //Info an Dashboard: Kollektorpumpe
    sendMQTT("solePumpe", digitalRead(RELAIS_SOLE_PUMPE));                   // Info an Dashboard, Solepumpe
    sendMQTT("stellwerkSoleBoiler", digitalRead(STELLWERK_SOLE_BOILER) + 1); //Info an Dashboard, Stellwerk
    sendMQTT("operationMode", operationMode);                                //Info an Dashboard, Modus

    MQTTSendTimer.executed();
  }

  if (timer1m.checkTimer(now))
  {
    //Energieberechnung Sole Volumen, dass umgesetzt wird, bei gegebener Pumpenleistung. Berechnung erfolgt online
    if (operationMode == 1)
    {
      sendMQTT("flowMeterSole", (float)((-3.4015 * pow(10, -7)) * pow(PIDOutputKollektorPumpe, 3) + (2.0240 * pow(10, -4)) * pow(PIDOutputKollektorPumpe, 2) - 0.0095 * PIDOutputKollektorPumpe + 0.1153));
    }

    //Prüfen ob MQTT noch verbunden ist, allenfalls neu-verbinden
    if (!mqttClient.connected())
    { // wurde die Verbindung unterbrochen?
      Serial.println("MQTT-Verbindung unterbrochen, versuche Neuverbindung");
      Serial.println();
      uint8_t i = 0;

      while (!mqttClient.connected() && !mqttClient.connect(broker, port) && i < 5) //starte 5 Verbindungsversuche
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
    //Prüfen, welcher Sendeintervall geeignet ist
    if (operationMode)
    {
      MQTTSendTimer.setDelayTime(5, 's');
    }
    else if (fuehlerKollektorLuft.getMeanTemperature() > SOLE_START_TEMPERATURE - 5)
    {
      MQTTSendTimer.setDelayTime(5, 's');
    }
    else if (fuehlerKollektorLuft.getMeanTemperature() > SOLE_START_TEMPERATURE - 10)
    {
      MQTTSendTimer.setDelayTime(5, 'm');
    }
    else
    {
      MQTTSendTimer.setDelayTime(30, 'm');
    }

    //Prüfen, ob Boiler über 65 Grad ist
    if (fuehlerBoiler1.getMeanTemperature() > 65)
    {
      timerLegionellenschaltung.setLastTime(now); //Legionellenschaltung hinauszögern
      if (boilerHighTemperatur)                   //Legionellenschaltung ausschalten, falls eingeschaltet
      {
        boilerHighTemperatur = false;
        digitalWrite(STELLWERK_BOILER_TEMP, LOW);
        sendMQTT("boilerTermostat", "normal");
        sendMQTT("message", "Boiler wieder bei normaler Solltemperatur für elektrisches Heizen.");
      }
      sendMQTT("boilerLegionellenTemperatur", "Temperatur erreicht");
    }

    timer3m.executed();
  }

  if (timerLegionellenschaltung.checkTimer(now))
  {
    digitalWrite(STELLWERK_BOILER_TEMP, HIGH); //Boiler auf höhere Temperatur stellen
    boilerHighTemperatur = true;
    boilerTimeout.setLastTime(now);       //Timer stellen
    timerLegionellenschaltung.executed(); //Timer zurücksetzen
    sendMQTT("message", "Boiler bei erhöhter Solltemperatur für elektrisches Heizen. (Legionellenschaltung)");
    sendMQTT("boilerTermostat", "hoch");
  }

  if (boilerHighTemperatur && boilerTimeout.checkTimer(now))
  {
    boilerHighTemperatur = false;
    digitalWrite(STELLWERK_BOILER_TEMP, LOW);
    sendMQTT("boilerTermostat", "normal");
    sendMQTT("message", "Boiler wieder bei normaler Solltemperatur für elektrisches Heizen.");
  }

  now = millis();
}