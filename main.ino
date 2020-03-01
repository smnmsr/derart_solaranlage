//Software für die Funktionen gemäss README-Datei
//
//STRUKTUR:
// 1. HEADER-Dateien und Libaries
// 2. Variabeln und Konstanten
// 3. PIN-Adressen und BUS-Definitionen
// 4. Initialisierungen
// 5. Funktionen
// 6. Setup Sequenz
// 7. Programmschleife

// ==============================
// 1. HEADER-Dateien und Libaries
// ==============================
#include "Wire.h"                   //Für I2C Bus
#include "PID_v1.h"                 //Für PID-Regelung
#include "Adafruit_LiquidCrystal.h" //Für Displays
#include "Classes.h"                //Eigene Klassen
#include "Adafruit_Sensor.h"        //Für Adafruit Sensoren
#include "Adafruit_TSL2591.h"       //Für Adafruit LUX-Sensoren
#include <SPI.h>                    //Ethernet
#include <Ethernet.h>               //Ethernet
#include <ArduinoMqttClient.h>      //MQTT
#include "secrets.h"                //MQTT Passwords

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
const int ALARMTEMPERATUR_KOLLEKTOR_LUFT = 90; //Alarmtemperatur für Kollektor (Lufttemperatur)
const int ALARMTEMPERATUR_KOLLEKTOR_VL = 85;   //Alarmtemperatur für Kollektor (Vorlauftemperatur)
const int ALARMTEMPERATUR_SOLE = 30;           //Alarmtemperatur für Sole-Pumpe
const int MIN_DIFFERENZ_NACH_ALARM = 3;        //erst wenn die Temperatur um diesen Wert gesunken ist, geht der Alarmmodus aus
const int SOLE_EXIT_TEMPERATURE = 10;          //Temperatur zur Sonde, bei der der Solemodus abgebrochen wird
const int SOLE_START_TEMPERATURE = 30;       //Temperatur im Sole Wärmetauscher VL, bei der der Solemodus gestartet wird
const int SOLE_VL_EXIT_TEMPERATURE = 25;       //Temperatur im Sole Wärmetauscher VL, bei der der Solemodus abgebrochen wird
const int SOLL_KOLLEKTOR_VL_BOILERMODUS = 75;  //Solltemperatzr, auf die der Kollektor VL geregelt werden soll, wenn Boilermodus
const int SOLL_KOLLEKTOR_VL_SOLEMODUS = 65;    //Solltemperatzr, auf die der Kollektor VL geregelt werden soll, wenn Solemodus
const int SOLL_SOLE_VL_SOLEMODUS = 30;         //Solltemperatur, auf die der Sole VL geregelt werden soll
const int MIN_WAERMER_BOILER = 3;              //Mindest Temperaturunterschied zwischen Boiler VL und Boiler
const int BOILER_UPPER_EXIT_TEMPERATURE = 70;  //Temperatur, auf diese der Boiler erwärmt werden soll
const int BOILER_MAX_START_TEMPERATURE = 65;   //Wenn der Boiler wärmer ist als diese Temperatur, wird er nicht mehr beheizt

//Helligkeiten
const unsigned long BOILER_START_BRIGHTNESS = 40000; //Helligkeit, bei der zum Boilermodus gewechselt wird
const unsigned long SOLE_START_BRIGHTNESS = 10000;   //Helligkeit, bei der zum Solemodus gewechselt wird

//PID Tuning Parameter
const double PID_P_KOLLEKTOR = 10;
const double PID_I_KOLLEKTOR = 1;
const double PID_D_KOLLEKTOR = 0;
const double PID_P_SOLE = 5;
const double PID_I_SOLE = 5;
const double PID_D_SOLE = 0;

// Ethernet Konstanten
byte mac[] = {0xA8, 0x61, 0x0A, 0xAE, 0x3D, 0xB7};

//Variabeln
byte operationMode = 0;            //Betriebsmodus: 0=aus, 1=Solemodus, 2=Boilermodus
byte displayMode = 0;              //Betriebsmodus anzeige
unsigned long now = 0;             //Jeweils aktueller millis()-Wert
bool displayOn = false;            //true, wenn Displays eingeschaltet sein soll
bool boilerHighTemperatur = false; //true, wenn Boiler auf höherer Temperatur ist
bool kollektorAlarm = false;       //true, wenn kollektor zu heiss ist
bool soleAlarm = false;            //true, wenn solepumpe zu heiss ist
float boilerLowerExitTemperature;  //Temperatur, bei der der Boilermodus abgebrochen wird
float boilerDirectExitTemperature; //Temperatur, bei der der Boilermodus direkt (ohne Verzögerung abgebrochen wird)
long brightness = 1;               //gemessene Helligkeit
bool initializing = false;         //Ist derzeit ein neuer Modus am Initialisieren?
bool tooLowValue = false;          //True, wenn Sollwert das erste mal unterschritten

//PID Variabeln
double PIDInputKollektorPumpe, PIDOutputKollektorPumpe, PIDSetpointKollektorPumpe;

// ==============================
// 3. PIN-Adressen und BUS-Definitionen
// ==============================

/* // i2c Multiplexer
const unsigned int MUX_OUT = 0x70;
const unsigned int MUX_IN = 0x71;

// i2c Adressen
Adafruit_LiquidCrystal LCD_00(0x74);
Adafruit_LiquidCrystal LCD_01(0x77);
Adafruit_LiquidCrystal LCD_02(0x73);
Adafruit_LiquidCrystal LCD_03(0x72);
Adafruit_LiquidCrystal LCD_04(0x75);
Adafruit_LiquidCrystal LCD_05(0x76);
Adafruit_LiquidCrystal LCD_06(0x74);
Adafruit_LiquidCrystal LCD_07(0x75); */

// Analog Pins
const byte FUEHLER_KOLLEKTOR_VL_PIN = A7;   //S0
const byte FUEHLER_KOLLEKTOR_LUFT_PIN = A0; //S1
const byte FUEHLER_BOILER_PIN = A1;         //S2
const byte FUEHLER_BOILER_VL_PIN = A2;      //S4B
const byte FUEHLER_SOLE_VL_PIN = A3;        //S4S
const byte FUEHLER_BOILER_RL_PIN = A4;      //S6B
const byte FUEHLER_SOLE_RL_PIN = A5;        //S6S
const byte FUEHLER_SOLE_PIN = A6;           //S7
// const byte POTENTIOMETER_1_PIN = A14; //aktuell nicht gebraucht
// const byte POTENTIOMETER_2_PIN = A15; //aktuell nicht gebraucht

// Digital In Pins
const byte DISPLAY_BUTTON = 30;
const byte UP_BUTTON = 24;
const byte DOWN_BUTTON = 28;
const byte LEFT_BUTTON = 26;
const byte RIGHT_BUTTON = 22;
const byte FLOW_METER_BOILER = 45;
const byte FLOW_METER_SOLE = 43;

// Digital Out Pins
const byte RELAIS_SOLE_PUMPE = 23;
const byte RELAIS_KOLLEKTOR_PUMPE = 25;
const byte STELLWERK_SOLE_BOILER = 27; //high = höhere Temperatur?
const byte STELLWERK_BOILER_TEMP = 29; //high = Boiler?

// PWM Pins
const byte PWM_KOLLEKTOR_PUMPE = 9;

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

// Setup aller Potentiometer
// Schema: Potentiometer <Name des Poti>(<PIN>); oder Optional: Potentiometer <Name des Poti>(<PIN>,<Mindestwert>,<Höchstwert>,<Umgekehrt>);
//Potentiometer potentiometer1(POTENTIOMETER_1_PIN); //aktuell unbenutzt
//Potentiometer potentiometer2(POTENTIOMETER_2_PIN); //aktuell unbenutzt

// Setup der LUX-Meter
Adafruit_TSL2591 luxMeter1 = Adafruit_TSL2591(0x29);

// Setup der Pumpen
Pump kollektorPumpe(RELAIS_KOLLEKTOR_PUMPE, PWM_KOLLEKTOR_PUMPE);

// Setup der Timer (leer für ms, 's' für s, 'm' für min, 'd' für Tage)
// Timer über normale Clock
Timer timer1s(1, 's');  //1s Timer
Timer timer5s(5, 's');  //5s Timer
Timer timer3m(30, 's'); //3min Timer
Timer timer7d(7, 'd');  //7d Timer

//Timer für bestimmte Funktionen
Timer initialOperationModeTimeout(3, 'm'); //Zeit bevor ein Modus EXIT-Kriterien berücksichtigt
Timer exitTimeout(2, 'm');                 //Solange muss der Sollwert mindestens unterschritten sein, bevor Abbruch
Timer flowMeterBoilerTimeout(500);         //Durchflussmeter 1 Timeout
Timer flowMeterSoleTimeout(500);           //Durchflussmeter 2 Timeout
Timer displayButtonTimeout(1000);          //Display-Button Timeout
Timer displayTimeout(30, 's');             //Display-Ausschaltzeit
Timer boilerTimeout(1, 'd');               //Boiler-Ausschaltzeit

//PWM Setup
PID PIDReglerKollektorPumpe(&PIDInputKollektorPumpe, &PIDOutputKollektorPumpe, &PIDSetpointKollektorPumpe, PID_P_KOLLEKTOR, PID_I_KOLLEKTOR, PID_D_KOLLEKTOR, REVERSE);

//MQTT Setup
EthernetClient client;
MqttClient mqttClient(client);

const char broker[] = "storage.moser-artz.ch";
int port = 1883;
String topic = "derart/";

// =============
// 5. FUNKTIONEN
// =============

void sendMQTT(String subtopic, float value)
{
  mqttClient.beginMessage(topic + subtopic);
  mqttClient.print(value);
  mqttClient.endMessage();
}
void sendMQTT(String subtopic, int value)
{
  mqttClient.beginMessage(topic + subtopic);
  mqttClient.print(value);
  mqttClient.endMessage();
}
void sendMQTT(String subtopic, const char value[])
{
  mqttClient.beginMessage(topic + subtopic);
  mqttClient.print(value);
  mqttClient.endMessage();
}

void i2cSelect(int mux, uint8_t i)
{
  if (i > 7)
    return;
  Wire.beginTransmission(mux);
  Wire.write(1 << i);
  Wire.endTransmission();
}

void boilerModusStart()
{
  sendMQTT("message","Zum Boilermodus gewechselt.");
  digitalWrite(RELAIS_KOLLEKTOR_PUMPE, HIGH); //Kollektorpumpe einschalten
  sendMQTT("kollektorPumpe", 1);
  digitalWrite(RELAIS_SOLE_PUMPE, LOW); //Solepumpe ausschalten
  sendMQTT("solePumpe", 0);
  digitalWrite(STELLWERK_SOLE_BOILER, HIGH); //Stellwerk auf Boiler umschalten
  sendMQTT("stellwerkSoleBoiler",2);
  PIDReglerKollektorPumpe.SetMode(1);                        //PID-Regler Kollektorpumpe einschalten
  PIDSetpointKollektorPumpe = SOLL_KOLLEKTOR_VL_BOILERMODUS; //Kollektor Vorlauf Sollwert setzen
  initializing = true;                                       //Initialisierung starten
  initialOperationModeTimeout.setLastTime(now);              //Initialisierungs Timer starten
  operationMode = 2;                                         //Modus auf Boiler schalten
  tooLowValue = false;                                       //tooLowValue zurücksetzen
  sendMQTT("operationMode", 2);
}

void soleModusStart()
{
  sendMQTT("message","Zum Solemodus gewechselt.");
  digitalWrite(RELAIS_KOLLEKTOR_PUMPE, HIGH); //Kollektorpumpe ausschalten
  sendMQTT("kollektorPumpe",1);
  digitalWrite(RELAIS_SOLE_PUMPE, HIGH); //Solepumpe einschalten
  sendMQTT("solePumpe",1);
  digitalWrite(STELLWERK_SOLE_BOILER, LOW); //Stellwerk auf Sole umschalten
  sendMQTT("stellwerkSoleBoiler",1);
  PIDReglerKollektorPumpe.SetMode(1);                      //PID-Regler Kollektorpumpe einschalten
  PIDSetpointKollektorPumpe = SOLL_KOLLEKTOR_VL_SOLEMODUS; //Kollektor Vorlauf Sollwert setzen
  initializing = true;                                     //Initialisierung starten
  initialOperationModeTimeout.setLastTime(now);            //Initialisierungs Timer starten
  operationMode = 1;                                       //Modus auf Sole schalten
  tooLowValue = false;                                     //tooLowValue zurücksetzen
  sendMQTT("operationMode", 1);
}

void turnOffModusStart()
{
  sendMQTT("message","In ausgeschaltenen Modus gewechselt.");
  digitalWrite(RELAIS_KOLLEKTOR_PUMPE, LOW); //Kollektorpumpe ausschalten
  sendMQTT("kollektorPumpe",0);
  digitalWrite(RELAIS_SOLE_PUMPE, LOW); //Solepumpe ausschalten
  sendMQTT("solePumpe",0);
  digitalWrite(STELLWERK_SOLE_BOILER, LOW); //Stellwerk auf Sole umschalten
  sendMQTT("stellwerkSoleBoiler",1);
  PIDReglerKollektorPumpe.SetMode(0); //PID-Regler Kollektorpumpe ausschalten
  initializing = false;               //Initialisierung stoppen
  operationMode = 0;                  //Modus auf aus schalten
  tooLowValue = false;                //tooLowValue zurücksetzen
  sendMQTT("operationMode",0);
}

void eraseDisplays()
{

  /*   i2cSelect(MUX_IN,1);
  LCD_00.clear();
  LCD_01.clear();
  LCD_02.clear();
  LCD_03.clear();
  LCD_04.clear();
  LCD_05.clear();

  i2cSelect(MUX_IN,0);
  LCD_06.clear();
  LCD_07.clear(); */
}

void writeDisplays(byte mode)
{
  /*   i2cSelect(MUX_IN,1);
  LCD_00.print("Test1");
  LCD_01.print("Test2");
  LCD_02.print("Test3");
  LCD_03.print("Test4");
  LCD_04.print("Test5");
  LCD_05.print("Test6");
  i2cSelect(MUX_IN,0);
  LCD_06.print("Test7");
  LCD_07.print("Test8"); */
}

void turnOffDisplays()
{
  /*     i2cSelect(MUX_IN,1);
  LCD_00.setBacklight(LOW);
  LCD_01.setBacklight(LOW);
  LCD_02.setBacklight(LOW);
  LCD_03.setBacklight(LOW);
  LCD_04.setBacklight(LOW);
  LCD_05.setBacklight(LOW);

  i2cSelect(MUX_IN,0);
  LCD_06.setBacklight(LOW);
  LCD_07.setBacklight(LOW); */
}

void turnOnDisplays()
{
  /*   i2cSelect(MUX_IN,1);
  LCD_00.setBacklight(HIGH);
  LCD_01.setBacklight(HIGH);
  LCD_02.setBacklight(HIGH);
  LCD_03.setBacklight(HIGH);
  LCD_04.setBacklight(HIGH);
  LCD_05.setBacklight(HIGH);

  i2cSelect(MUX_IN,0);
  LCD_06.setBacklight(HIGH);
  LCD_07.setBacklight(HIGH); */
}

// ================
// 6. Setup-Sequenz
// ================
void setup()
{
  Serial.begin(9600);
  while (!Serial)
  {
    ; // wait for serial port to connect. Needed for native USB port only
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
  PIDReglerKollektorPumpe.SetOutputLimits(70, 255);
  PIDReglerKollektorPumpe.SetSampleTime(800);

  //Display Setup
  // i2cSelect(MUX_IN,0);
  // LCD_06.begin(16, 2);
  // LCD_07.begin(16, 2);
  // i2cSelect(MUX_IN,1);
  // LCD_00.begin(16, 2);
  // LCD_01.begin(16, 2);
  // LCD_02.begin(16, 2);
  // LCD_03.begin(16, 2);
  // LCD_04.begin(16, 2);
  // LCD_05.begin(16, 2);
  // eraseDisplays();
  // turnOffDisplays();

  //Luxmeter Setup
  luxMeter1.setGain(TSL2591_GAIN_LOW);
  luxMeter1.setTiming(TSL2591_INTEGRATIONTIME_200MS);
  delay(1000);

  // Initialise Internet and MQTT
  // attempt to connect to  network:
  Serial.print("Attempting to connect to Internet via DHCP ");
  while (Ethernet.begin(mac) == 0)
  {
    // failed, retry
    Serial.print(".");
    delay(5000);
  }

  Serial.println("You're connected to the Internet. our IP: ");
  Serial.println(Ethernet.localIP());
  Serial.println();

  mqttClient.setUsernamePassword(MQTT_USER, MQTT_PASS);

  Serial.print("Attempting to connect to the MQTT broker: ");
  Serial.println(broker);

  while (!mqttClient.connect(broker, port))
  {
    Serial.print("MQTT connection failed! Error code = ");
    Serial.println(mqttClient.connectError());

    delay(5000);
  }

  Serial.println("You're connected to the MQTT broker!");
  Serial.println();

  turnOffModusStart();

  Serial.println("Setup beendet");
}

// ===================
// 7. Programmschleife
// ===================
void loop()
{
  /*     //Prüfen, ob je nach Betriebsmodus der entsprechende Durchflussmesser einen Impuls ausgibt
  switch (operationMode)
  {
  case 0:
    break;
  case 1:
    if (digitalRead(FLOW_METER_SOLE) == HIGH && flowMeterSoleTimeout.checkTimer(now) == true)
    {
      Serial.println("Flow Meter Sole gibt an");
      flowMeterSoleTimeout.executed();
    }
    break;
  case 2:
    if (digitalRead(FLOW_METER_BOILER) == HIGH && flowMeterBoilerTimeout.checkTimer(now) == true)
    {
      Serial.println("Flow Meter Boiler gibt an");
      flowMeterBoilerTimeout.executed();
    }
    break;
  } */

  //Prüfen, ob der Display Button gedrückt wird
  if (digitalRead(DISPLAY_BUTTON) == HIGH && displayButtonTimeout.checkTimer(now))
  {
    Serial.println("Display wurde eingeschaltet.");
    displayOn = true;
    displayTimeout.setLastTime(now);
    displayButtonTimeout.executed();
  }

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

    //Fuehlerwerte an MQTT Senden
    sendMQTT("fuehlerKollektorVL",fuehlerKollektorVL.getMeanTemperature());
    sendMQTT("fuehlerKollektorLuft",fuehlerKollektorLuft.getMeanTemperature());
    sendMQTT("fuehlerBoiler",fuehlerBoiler.getMeanTemperature());
    sendMQTT("fuehlerBoilerVL",fuehlerBoilerVL.getMeanTemperature());
    sendMQTT("fuehlerSoleVL",fuehlerSoleVL.getMeanTemperature());
    sendMQTT("fuehlerBoilerRL",fuehlerBoilerRL.getMeanTemperature());
    sendMQTT("fuehlerSoleRL",fuehlerSoleRL.getMeanTemperature());
    sendMQTT("fuehlerSole",fuehlerSole.getMeanTemperature());


    if (displayOn)
    {
      //Displays Schreiben
      turnOnDisplays();
      writeDisplays(displayMode);
    }
    //PID Regler berechnen
    if (operationMode == 1)
    {
      PIDInputKollektorPumpe = fuehlerKollektorLuft.getMeanTemperature(); //Kollektor Vorlauftemperatur auslesen
      PIDReglerKollektorPumpe.Compute();
      kollektorPumpe.setSpeed(PIDOutputKollektorPumpe);
      sendMQTT("speedKollektorPumpe",(int)round(PIDOutputKollektorPumpe));
    }
    else if (operationMode == 2)
    {
      PIDInputKollektorPumpe = fuehlerKollektorLuft.getMeanTemperature(); //Kollektor Vorlauftemperatur auslesen
      PIDReglerKollektorPumpe.Compute();
      kollektorPumpe.setSpeed(PIDOutputKollektorPumpe);
      sendMQTT("speedKollektorPumpe",(int)round(PIDOutputKollektorPumpe));
    }
    else {
      sendMQTT("speedKollektorPumpe",(int)round(PIDOutputKollektorPumpe));
    }

    // MQTT Client am Leben halten
    mqttClient.poll();

    //Timer zurücksetzen
    timer1s.executed();
  }

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

    //Alarmmodus
    //Sole AlArm?
    if (soleAlarm)
    {
      //Solepumpe ausschalten
      digitalWrite(RELAIS_SOLE_PUMPE, LOW);
      sendMQTT("alarm","Alarm. Die Soletemperatur ist zu hoch.");
    }

    //Kollektor Alarm?
    if (kollektorAlarm)
    {
      //Kollektorpumpe auf 100% und Wärme in Boiler
      analogWrite(PWM_KOLLEKTOR_PUMPE, 255);
      digitalWrite(STELLWERK_SOLE_BOILER, HIGH);
      sendMQTT("alarm","Alarm. Die Kollektortemperatur ist zu hoch.");
    }

    while (soleAlarm || kollektorAlarm)
    {
      //Sole Temperatur abfragen und prüfen, ob ausreichend abgekühlt
      fuehlerSole.calculateTemperature();
      if (fuehlerSole.getLastTemperature() < ALARMTEMPERATUR_SOLE - MIN_DIFFERENZ_NACH_ALARM && soleAlarm)
      {
        soleAlarm = false;
        sendMQTT("alarm","Die Soletemperatur ist nicht mehr zu hoch.");
      }

      //Kollektor VL und Luft Temperatur abfragen und prüfen, ob ausreichend abgekühlt
      fuehlerKollektorLuft.calculateTemperature();
      fuehlerKollektorVL.calculateTemperature();
      if (fuehlerKollektorVL.getLastTemperature() < ALARMTEMPERATUR_KOLLEKTOR_VL - MIN_DIFFERENZ_NACH_ALARM && fuehlerKollektorLuft.getLastTemperature() < ALARMTEMPERATUR_KOLLEKTOR_LUFT - MIN_DIFFERENZ_NACH_ALARM && kollektorAlarm)
      {
        kollektorAlarm = false;
        sendMQTT("alarm","Die Kollektortemperatue ist nicht mehr zu hoch.");
      }
      delay(200);
    }

    //Prüfen, ob Initialisierung abgeschlossen ist
    if (initialOperationModeTimeout.checkTimer(now) && initializing)
    {
      sendMQTT("message","Die Initialisierung des aktuellen Betriebsmodus ist abgeschlossen.");
      initializing = false;
    }

    boilerLowerExitTemperature = fuehlerBoiler.getMeanTemperature() + MIN_WAERMER_BOILER; //Mindest Boilertemperatur
    boilerDirectExitTemperature = fuehlerBoiler.getMeanTemperature();                     //Boilertemperatur für direkten Abbruch

    //Entscheidung Betriebsmodus
    switch (operationMode)
    {
    case 0: //Im Modus ausgeschaltet?
    {
      if (fuehlerKollektorLuft.getMeanTemperature() > boilerLowerExitTemperature+5 && fuehlerBoiler.getMeanTemperature() < BOILER_MAX_START_TEMPERATURE && !initializing) //Temperatur so hoch, dass Boiler geheizt werden könnte?
      {
        boilerModusStart();
        sendMQTT("message","Boilermodus aus ausgeschaltenem Modus aufgrund hoher Vorlauftemperatur gestartet");
      }
      else if (fuehlerKollektorLuft.getMeanTemperature() > SOLE_START_TEMPERATURE && !initializing) //Temperatur genügenr für Solemodus?
      {
        soleModusStart();
        sendMQTT("message","Solemodus aus ausgeschaltenem Modus aufgrund hoher Vorlauftemperatur gestartet");
      }
/*       if (brightness > BOILER_START_BRIGHTNESS && fuehlerBoiler.getMeanTemperature() < BOILER_MAX_START_TEMPERATURE) //Kriterien für Boilermodus erfüllt?
      {
        boilerModusStart();
        sendMQTT("message","Boilermodus aus ausgeschaltenem Modus gestartet");
      }
      else if (brightness > SOLE_START_BRIGHTNESS) //Kriterien für Solemodus erfüllt?
      {
        soleModusStart();
        sendMQTT("message","Solemodus aus ausgeschaltenem Modus gestartet");
      } */
    }
    break;
    case 1: //Im Modus Sole?
    {
/*       if (brightness > BOILER_START_BRIGHTNESS && fuehlerBoiler.getMeanTemperature() < BOILER_MAX_START_TEMPERATURE && !initializing) //Kriterien für Boilermodus erfüllt?
      {
        boilerModusStart();
        sendMQTT("message","Boilermodus aus Solemodus aufgrund Helligkeit gestartet");
      } */

      if (fuehlerKollektorVL.getMeanTemperature() > boilerLowerExitTemperature && fuehlerBoiler.getMeanTemperature() < BOILER_MAX_START_TEMPERATURE && !initializing) //Temperatur so hoch, dass Boiler geheizt werden könnte?
      {
        boilerModusStart();
        sendMQTT("message","Boilermodus aus Solemodus aufgrund hoher Vorlauftemperatur gestartet");
      }

      if ((fuehlerSole.getMeanTemperature() > SOLE_EXIT_TEMPERATURE || fuehlerSoleVL.getMeanTemperature() > SOLE_VL_EXIT_TEMPERATURE) && tooLowValue) //ausreichende Temperatur?
      {
        tooLowValue = false; //Abbruch abbrechen
      }

      if (fuehlerSole.getMeanTemperature() < SOLE_EXIT_TEMPERATURE || fuehlerSoleVL.getMeanTemperature() < SOLE_VL_EXIT_TEMPERATURE && !initializing) //zu tiefe Temperatur im Solemodus?
      {
        if (!tooLowValue)
        {
          tooLowValue = true;
          exitTimeout.setLastTime(now);
        }
        else if (tooLowValue && exitTimeout.checkTimer(now))
        {
          turnOffModusStart();
          exitTimeout.executed();
          sendMQTT("message","Von Solemodus zu ausgeschaltet, da zu lange zu kühl");
        }
      }
    }
    break;
    case 2: //Im Modus Boiler?
    {
      if (fuehlerBoilerVL.getMeanTemperature() < boilerDirectExitTemperature && !initializing) //Vorlauftemperatur viel zu tief? --> direkter Abbruch
      {
        soleModusStart();
        sendMQTT("message","von Boilermodus zu Solemodus da Boiler VL viel zu klühl");
        break;
      }

      if (fuehlerBoilerVL.getMeanTemperature() > boilerLowerExitTemperature && tooLowValue) //ausreichende Temperatur?
      {
        tooLowValue = false; //Abbruch abbrechen
      }

      if (fuehlerBoilerVL.getMeanTemperature() < boilerLowerExitTemperature && !initializing) //zu Tiefe Temperatur im Boilermodus
      {
        if (!tooLowValue)
        {
          tooLowValue = true;
          exitTimeout.setLastTime(now);
        }
        else if (tooLowValue && exitTimeout.checkTimer(now))
        {
          soleModusStart();
          exitTimeout.executed();
          sendMQTT("message","von Boilermodus zu Solemodus da Boiler VL zu lange zu kühl");
        }
      }
    }
    break;
    default:
    {
      operationMode = 0;
    }
    }
    timer5s.executed();
  }

  if (timer3m.checkTimer(now))
  {
/*     // Helligkeit auslesen
    brightness = luxMeter1.getLuminosity(TSL2591_FULLSPECTRUM); //
    Serial.print("Helligkeitmessung abgeschlossen. Helligkeit :");
    Serial.print(brightness);
    Serial.println(" lux");
    sendMQTT("brightness", (int)round(brightness / 1000)); */

    //Ethernet aktuell halten
    Ethernet.maintain();

    timer3m.executed();
  }

  if (timer7d.checkTimer(now))
  {
    digitalWrite(STELLWERK_BOILER_TEMP, HIGH); //Boiler auf höhere Temperatur stellen
    boilerHighTemperatur = true;
    boilerTimeout.setLastTime(now); //Timer stellen
    timer7d.executed();             //Timer zurücksetzen
    sendMQTT("message","Boiler hohe Temperatur angefangen");
    sendMQTT("boilerTermostat","hoch");
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
    sendMQTT("boilerTermostat","normal");
    sendMQTT("message","Boiler hohe Temperatur beendet");
  }

  now = millis();
}