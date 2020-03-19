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
const float FUEHLER_BOILER_KORREKTURFAKTOR = 1.0;         //Korrekturfaktor S2
const float FUEHLER_BOILER_VL_KORREKTURFAKTOR = 1.0;      //Korrekturfaktor S4B
const float FUEHLER_SOLE_VL_KORREKTURFAKTOR = 1.0;        //Korrekturfaktor S4S
const float FUEHLER_BOILER_RL_KORREKTURFAKTOR = 1.0;      //Korrekturfaktor S6B
const float FUEHLER_SOLE_RL_KORREKTURFAKTOR = 1.0;        //Korrekturfaktor S6S
const float FUEHLER_SOLE_KORREKTURFAKTOR = 1.0;           //Korrekturfaktor S7

//Temperaturen
const int ALARMTEMPERATUR_KOLLEKTOR_LUFT = 90;    //Alarmtemperatur für Kollektor (Lufttemperatur)
const int ALARMTEMPERATUR_KOLLEKTOR_VL = 95;      //Alarmtemperatur für Kollektor (Vorlauftemperatur)
const int ALARMTEMPERATUR_SOLE = 30;              //Alarmtemperatur für Sole-Pumpe
const int ACHTUNG_TEMPERATUR_SOLE = 25;           //Alarmtemperatur für Sole-Pumpe
const int MIN_DIFFERENZ_NACH_ALARM = 3;           //erst wenn die Temperatur um diesen Wert gesunken ist, geht der Alarmmodus aus
const int SOLE_EXIT_TEMPERATURE = 6;              //Temperatur zur Sonde, bei der der Solemodus abgebrochen wird
const int SOLE_START_TEMPERATURE = 25;            //Temperatur im Kollektor (Luft), bei der der Solemodus gestartet wird
const int SOLE_VL_EXIT_TEMPERATURE = 23;          //Temperatur im Sole Wärmetauscher VL, bei der der Solemodus abgebrochen wird
const int SOLL_KOLLEKTOR_VL_BOILERMODUS = 80;     //Solltemperatzr, auf die der Kollektor VL geregelt werden soll, wenn Boilermodus
const int SOLL_KOLLEKTOR_VL_SOLEMODUS = 75;       //Solltemperatzr, auf die der Kollektor VL geregelt werden soll, wenn Solemodus
const int SOLL_KOLLEKTOR_VL_SOLEMODUS_2 = 50;       //Solltemperatzr, auf die der Kollektor VL geregelt werden soll, wenn Solemodus
const int BOILER_DIRECT_EXIT_DIFF = -3;           //Maximaler Temperaturunterschied zwischen Boilertemperatur und Boiler VL bevor direkter Abbruch
const int MIN_DIFFERENZ_VL_RL_BOILER = 10;        //Wenn VL und RL weniger als Diese Differenz haben, wird der Modus abgebrochen
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

//PID Variabeln
double PIDInputKollektorPumpe, PIDOutputKollektorPumpe, PIDSetpointKollektorPumpe; //Variabeln für PID-Regler der Kollektorpumpe

// ====================================
// 3. PIN-Adressen und BUS-Adressierung
// ====================================

// i2c Adressen der Displays
Adafruit_LiquidCrystal LCD_00(0x74);
Adafruit_LiquidCrystal LCD_01(0x77);
Adafruit_LiquidCrystal LCD_02(0x73);
Adafruit_LiquidCrystal LCD_03(0x72);
Adafruit_LiquidCrystal LCD_04(0x75);
Adafruit_LiquidCrystal LCD_05(0x76);
Adafruit_LiquidCrystal LCD_06(0x74);
Adafruit_LiquidCrystal LCD_07(0x75);

// Analog Pins
const byte FUEHLER_KOLLEKTOR_VL_PIN = A7;   //S0 Anschluss PIN
const byte FUEHLER_KOLLEKTOR_LUFT_PIN = A0; //S1 Anschluss PIN
const byte FUEHLER_BOILER_PIN = A1;         //S2 Anschluss PIN
const byte FUEHLER_BOILER_VL_PIN = A2;      //S4B Anschluss PIN
const byte FUEHLER_SOLE_VL_PIN = A3;        //S4S Anschluss PIN
const byte FUEHLER_BOILER_RL_PIN = A4;      //S6B Anschluss PIN
const byte FUEHLER_SOLE_RL_PIN = A5;        //S6S Anschluss PIN
const byte FUEHLER_SOLE_PIN = A6;           //S7 Anschluss PIN

// Digital In Pins
const byte DISPLAY_BUTTON = 30;    //Anschluss PIN Display-ON-Button
const byte UP_BUTTON = 24;         //Anschluss PIN UP-Button
const byte DOWN_BUTTON = 28;       //Anschluss PIN DOWN-Button
const byte LEFT_BUTTON = 26;       //Anschluss PIN LEFT-Button
const byte RIGHT_BUTTON = 22;      //Anschluss PIN RIGHT-Button
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
PT1000 fuehlerBoiler(FUEHLER_BOILER_PIN, 1000, FUEHLER_BOILER_KORREKTURFAKTOR);
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

//Sendet eine MQTT-Message mit dem Thema <subtopic> und dem String <value>
void sendMQTT(String subtopic, const char value[])
{
  mqttClient.beginMessage(topic + subtopic);
  mqttClient.print(value);
  mqttClient.endMessage();
}

//Startet den Boilermodus
void boilerModusStart()
{
  digitalWrite(RELAIS_KOLLEKTOR_PUMPE, HIGH);                //Kollektorpumpe einschalten
  digitalWrite(RELAIS_SOLE_PUMPE, LOW);                      //Solepumpe ausschalten
  digitalWrite(STELLWERK_SOLE_BOILER, HIGH);                 //Stellwerk auf Boiler umschalten
  PIDReglerKollektorPumpe.SetMode(1);                        //PID-Regler Kollektorpumpe einschalten
  PIDSetpointKollektorPumpe = SOLL_KOLLEKTOR_VL_BOILERMODUS; //Kollektor Vorlauf Sollwert setzen
  initializing = true;                                       //Initialisierung starten
  initialOperationModeTimeout.setDelayTime(5, 'm');          //Initialisierungszeit setzen
  initialOperationModeTimeout.setLastTime(now);              //Initialisierungs Timer starten
  operationMode = 2;                                         //Modus auf Boiler schalten
  tooLowValue = false;                                       //tooLowValue zurücksetzen
}

//Startet den Solemodus
void soleModusStart()
{
  digitalWrite(RELAIS_KOLLEKTOR_PUMPE, HIGH);              //Kollektorpumpe ausschalten
  digitalWrite(RELAIS_SOLE_PUMPE, HIGH);                   //Solepumpe einschalten
  digitalWrite(STELLWERK_SOLE_BOILER, LOW);                //Stellwerk auf Sole umschalten
  PIDReglerKollektorPumpe.SetMode(1);                      //PID-Regler Kollektorpumpe einschalten
  PIDSetpointKollektorPumpe = SOLL_KOLLEKTOR_VL_SOLEMODUS; //Kollektor Vorlauf Sollwert setzen
  initializing = true;                                     //Initialisierung starten
  switch (operationMode)                                   //Initialisierungszeit setzen
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

//Alle Displays löschen
void eraseDisplays()
{
  LCD_00.clear();
  LCD_01.clear();
  LCD_02.clear();
  LCD_03.clear();
  LCD_04.clear();
  LCD_05.clear();
  LCD_06.clear();
  LCD_07.clear();
}

//Displays schreiben
void writeDisplays()
{
  //Bildschirm oben links
  LCD_00.setCursor(0, 0);
  LCD_00.print("Betriebsmodus:");
  LCD_00.setCursor(1, 0);
  switch (operationMode)
  {
  case 0:
    LCD_00.print("Ausgeschaltet");
    break;
  case 1:
    LCD_00.print("Sole laden");
    break;
  case 2:
    LCD_00.print("Boiler laden");
    break;
  }

  //Bildschirm oben rechts
  LCD_01.setCursor(0, 0);
  LCD_01.print("K-Pumpe:");
  LCD_01.setCursor(0, 11);
  if (round(((PIDOutputKollektorPumpe / 2.55) - 15) * (9 / 8) + 10) <= 2)
  {
    LCD_01.print("Aus");
  }
  else if (round(((PIDOutputKollektorPumpe / 2.55) - 15) * (9 / 8) + 10) >= 100)
  {
    LCD_01.print("100 %");
  }
  else
  {
    LCD_01.print(round(((PIDOutputKollektorPumpe / 2.55) - 15) * (9 / 8) + 10));
    LCD_01.print(" % ");
  }
  LCD_01.setCursor(1, 0);
  LCD_01.print("S-Pumpe:");
  LCD_01.setCursor(1, 11);
  if (digitalRead(RELAIS_SOLE_PUMPE))
  {
    LCD_01.print("Ein");
  }
  else
  {
    LCD_01.print("Aus");
  }

  //Bildschirm mitteOben rechts
  LCD_03.setCursor(0, 0);
  LCD_03.print("K-Luft:");
  LCD_03.setCursor(0, 11);
  LCD_03.print(fuehlerKollektorLuft.getMeanTemperature(), 1);
  LCD_03.setCursor(0, 0);
  LCD_03.print("K-VL:");
  LCD_03.setCursor(0, 11);
  LCD_03.print(fuehlerKollektorVL.getMeanTemperature(), 1);
}

//Displays ausschalten
void turnOffDisplays()
{
  LCD_00.setBacklight(LOW);
  LCD_01.setBacklight(LOW);
  LCD_02.setBacklight(LOW);
  LCD_03.setBacklight(LOW);
  LCD_04.setBacklight(LOW);
  LCD_05.setBacklight(LOW);
  LCD_06.setBacklight(LOW);
  LCD_07.setBacklight(LOW);
}

//Displays einschalten
void turnOnDisplays()
{
  LCD_00.setBacklight(HIGH);
  LCD_01.setBacklight(HIGH);
  LCD_02.setBacklight(HIGH);
  LCD_03.setBacklight(HIGH);
  LCD_04.setBacklight(HIGH);
  LCD_05.setBacklight(HIGH);
  LCD_06.setBacklight(HIGH);
  LCD_07.setBacklight(HIGH);
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

  // Setup der PINS
  // Input
  pinMode(DISPLAY_BUTTON, INPUT);
  pinMode(FLOW_METER_BOILER, INPUT);
  pinMode(FLOW_METER_SOLE, INPUT);

  // Output
  pinMode(RELAIS_SOLE_PUMPE, OUTPUT);
  pinMode(RELAIS_KOLLEKTOR_PUMPE, OUTPUT);
  pinMode(STELLWERK_SOLE_BOILER, OUTPUT);
  pinMode(STELLWERK_BOILER_TEMP, OUTPUT);
  pinMode(PWM_KOLLEKTOR_PUMPE, OUTPUT);

  //PID Setup
  PIDReglerKollektorPumpe.SetOutputLimits(PID_KOLLEKTOR_MIN_SPEED, 255);
  PIDReglerKollektorPumpe.SetSampleTime(5000);

  //Display Setup
  LCD_06.begin(16, 2);
  LCD_07.begin(16, 2);
  LCD_00.begin(16, 2);
  LCD_01.begin(16, 2);
  LCD_02.begin(16, 2);
  LCD_03.begin(16, 2);
  LCD_04.begin(16, 2);
  LCD_05.begin(16, 2);
  eraseDisplays();
  turnOffDisplays();

  // Initialisiere Internet und MQTT
  // Versuch, sich mit dem Internet zu verbinden
  Serial.print("Internet-Verbindungsversuch ueber DHCP");
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

  // subscribe to a topic
  mqttClient.subscribe("derart/toArduino/LegionellenModus");

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
      else if (recievedPayload == '2')
      {
        PIDSetpointKollektorPumpe = 25;
        sendMQTT("message", "Neuer Sollwert: 25 degC");
      }
      else if (recievedPayload == '3')
      {
        PIDSetpointKollektorPumpe = 30;
        sendMQTT("message", "Neuer Sollwert: 30 degC");
      }
      else if (recievedPayload == '4')
      {
        PIDSetpointKollektorPumpe = 35;
        sendMQTT("message", "Neuer Sollwert: 35 degC");
      }
      else if (recievedPayload == '5')
      {
        PIDSetpointKollektorPumpe = 40;
        sendMQTT("message", "Neuer Sollwert: 40 degC");
      }
      else if (recievedPayload == '6')
      {
        PIDSetpointKollektorPumpe = 45;
        sendMQTT("message", "Neuer Sollwert: 45 degC");
      }
    }
  }

  //Prüfen, ob der Display Button gedrückt wird
  if (digitalRead(DISPLAY_BUTTON) == HIGH && displayButtonTimeout.checkTimer(now))
  {
    Serial.println("Display wurde eingeschaltet.");
    displayOn = true;
    displayTimeout.setLastTime(now);
    displayButtonTimeout.executed();
  }

  PIDInputKollektorPumpe = fuehlerKollektorLuft.getMeanTemperature(); //Kollektor Vorlauftemperatur auslesen
  if (PIDReglerKollektorPumpe.Compute())                              //PID Regler berechnen
  {
    kollektorPumpe.setSpeed(PIDOutputKollektorPumpe); //neuen Speed Kollektorpumpe setzen
  }

  //Dieser Programmteil wird alle 1s ausgeführt
  if (timer1s.checkTimer(now))
  {
    //Alle Fühler auslesen
    fuehlerKollektorVL.calculateTemperature();
    fuehlerKollektorLuft.calculateTemperature();
    fuehlerBoiler.calculateTemperature();
    fuehlerBoilerVL.calculateTemperature();
    fuehlerSoleVL.calculateTemperature();
    fuehlerBoilerRL.calculateTemperature();
    fuehlerSoleRL.calculateTemperature();
    fuehlerSole.calculateTemperature();

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
    if (displayOn) //Soll das Display eingeschaltet sein?
    {
      //Displays Schreiben
      turnOnDisplays();
      writeDisplays();
    }

    //Überprüfen, ob Alarmwerte überschritten wurden
    if (fuehlerSole.getLastTemperature() > ALARMTEMPERATUR_SOLE)
    {
      soleAlarm = true;
    }
    if (fuehlerKollektorLuft.getLastTemperature() > ALARMTEMPERATUR_KOLLEKTOR_LUFT || fuehlerKollektorVL.getLastTemperature() > ALARMTEMPERATUR_KOLLEKTOR_VL)
    {
      kollektorAlarm = true;
    }

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
        sendMQTT("alarm", "Die Soletemperatur ist nicht mehr zu hoch."); //Dashboard Nachricht: Alarm beendet
        soleModusStart();                                              //Nach Alarm in Boilermodus starten
      }

      //Kollektor VL und Luft Temperatur abfragen und prüfen, ob ausreichend abgekühlt
      fuehlerKollektorLuft.calculateTemperature();
      fuehlerKollektorVL.calculateTemperature();
      if (fuehlerKollektorVL.getLastTemperature() < ALARMTEMPERATUR_KOLLEKTOR_VL - MIN_DIFFERENZ_NACH_ALARM && fuehlerKollektorLuft.getLastTemperature() < ALARMTEMPERATUR_KOLLEKTOR_LUFT - MIN_DIFFERENZ_NACH_ALARM && kollektorAlarm)
      {
        kollektorAlarm = false;
        sendMQTT("alarm", "Die Kollektortemperatue ist nicht mehr zu hoch."); //Dashboard Nachricht: Alarm beendet
        boilerModusStart();                                                   //Nach Alarm in Boilermodus starten
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
        if (fuehlerKollektorLuft.getMeanTemperature() > fuehlerBoiler.getMeanTemperature() + MIN_WAERMER_KOLLEKTOR_LUFT_BOILER)
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
        if ((fuehlerKollektorVL.getMeanTemperature() > fuehlerBoiler.getMeanTemperature() + MIN_WAERMER_KOLLEKTOR_VL_BOILER) && (fuehlerSoleVL.getMeanTemperature() > fuehlerBoiler.getMeanTemperature()))
        //Kollektor und Sole VL so heiss, dass Boiler geladen werden kann? --> Direkt zu Boiler umschalten.
        {
          boilerModusStart();
          sendMQTT("message", "Boilermodus direkt aus Sole-Modus aufgrund hoher Vorlauf Temperaturen gestartet. Initialisierung beginnt.");
        }
      }
      else
      //Nicht am Initialisieren
      {
        if (fuehlerKollektorVL.getMeanTemperature() > fuehlerBoiler.getMeanTemperature() + MIN_WAERMER_KOLLEKTOR_VL_BOILER)
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
      if (fuehlerSole.getMeanTemperature() > ACHTUNG_TEMPERATUR_SOLE) {
        PIDSetpointKollektorPumpe = SOLL_KOLLEKTOR_VL_SOLEMODUS_2;
      } else {
        PIDSetpointKollektorPumpe = SOLL_KOLLEKTOR_VL_SOLEMODUS;
      }
    }
    break;
    case 2: //Im Modus Boiler?
    {
      if (tooLowValue)
      //Sollwert bereits unterschritten
      {
        if ((fuehlerBoilerVL.getMeanTemperature() > fuehlerBoiler.getMeanTemperature()) && (fuehlerBoilerVL.getMeanTemperature() - fuehlerBoilerRL.getMeanTemperature() > MIN_DIFFERENZ_VL_RL_BOILER))
        //Sollwerte fur Boilermodus alle wieder erreicht?
        {
          tooLowValue = false; //Abbruch abbrechen
          sendMQTT("message", "Solltemperatur für Boilermodus wurde wieder erreicht, Modus wird nicht abgebrochen.");
        }
      }

      if (!initializing)
      //Nicht mehr am Initialisieren?
      {
        if (fuehlerBoilerVL.getMeanTemperature() < fuehlerBoiler.getMeanTemperature() + BOILER_DIRECT_EXIT_DIFF)
        //Boiler VL so kalt, dass direkt abgebrochen werden muss?
        {
          soleModusStart(); //Solemodus starten
          sendMQTT("message", "Vom Boiler-Laden zum Solemodus gewechselt, da der Boiler Vorlauf viel zu kalt war. Sole-Initialisierung beginnt.");
        }
        else if ((fuehlerBoilerVL.getMeanTemperature() < fuehlerBoiler.getMeanTemperature()) || (fuehlerBoilerVL.getMeanTemperature() - fuehlerBoilerRL.getMeanTemperature() < MIN_DIFFERENZ_VL_RL_BOILER))
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
    sendMQTT("fuehlerBoiler", fuehlerBoiler.getMeanTemperature());
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

  if (timer3m.checkTimer(now))
  {
    /*     //DEBUG: Regelmässig mit Serieller Schnittstelle kommunizieren und Modus durchgeben
    Serial.println("Ich lebe noch!");
    Serial.println("IP-Adresse: ");
    Serial.println(Ethernet.localIP()); */

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
      }
    }

    //Prüfen, ob Boiler über 65 Grad ist
    if (fuehlerBoiler.getMeanTemperature() > 65)
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

  if (displayOn && displayTimeout.checkTimer(now))
  {
    displayOn = false;
    Serial.println("Display ausgeschaltet");
    eraseDisplays();
    turnOffDisplays();
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