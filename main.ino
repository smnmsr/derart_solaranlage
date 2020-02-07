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

// ===========================
// 2. Variabeln und Konstanten
// ===========================
// Konstanten
//Korrekturfaktoren
const float FUEHLER_KOLLEKTOR_VL_KORREKTURFAKTOR = 1.07;   //Korrekturfaktor S0
const float FUEHLER_KOLLEKTOR_LUFT_KORREKTURFAKTOR = 1.07; //Korrekturfaktor S1
const float FUEHLER_BOILER_KORREKTURFAKTOR = 1.07;         //Korrekturfaktor S2
const float FUEHLER_BOILER_VL_KORREKTURFAKTOR = 1.07;      //Korrekturfaktor S4B
const float FUEHLER_SOLE_VL_KORREKTURFAKTOR = 1.07;        //Korrekturfaktor S4S
const float FUEHLER_BOILER_RL_KORREKTURFAKTOR = 1.07;      //Korrekturfaktor S6B
const float FUEHLER_SOLE_RL_KORREKTURFAKTOR = 1.07;        //Korrekturfaktor S6S
const float FUEHLER_SOLE_KORREKTURFAKTOR = 1.07;           //Korrekturfaktor S7

//Temperaturen
const int ALARMTEMPERATUR_KOLLEKTOR_LUFT = 90; //Alarmtemperatur für Kollektor (Lufttemperatur)
const int ALARMTEMPERATUR_KOLLEKTOR_VL = 90;   //Alarmtemperatur für Kollektor (Vorlauftemperatur)
const int ALARMTEMPERATUR_SOLE = 30;           //Alarmtemperatur für Sole-Pumpe
const int MIN_DIFFERENZ_NACH_ALARM = 3;        //erst wenn die Temperatur um diesen Wert gesunken ist, geht der Alarmmodus aus
const int SOLE_EXIT_TEMPERATURE = 12;          //Temperatur, bei der der Solemodus abgebrochen wird
const int SOLL_KOLLEKTOR_VL_BOILERMODUS = 80;  //Solltemperatzr, auf die der Kollektor VL geregelt werden soll, wenn Boilermodus
const int SOLL_KOLLEKTOR_VL_SOLEMODUS = 60;    //Solltemperatzr, auf die der Kollektor VL geregelt werden soll, wenn Solemodus
const int SOLL_SOLE_VL_SOLEMODUS = 20;         //Solltemperatur, auf die der Sole VL geregelt werden soll
const int MIN_WAERMER_BOILER = 3;              //Mindest Temperaturunterschied zwischen Boiler VL und Boiler

//Helligkeiten
const int BOILER_START_BRIGHTNESS = 35000; //Helligkeit, bei der zum Boilermodus gewechselt wird
const int SOLE_START_BRIGHTNESS = 10000;   //Helligkeit, bei der zum Solemodus gewechselt wird

//PID Tuning Parameter
const double PID_P_KOLLEKTOR = 1;
const double PID_I_KOLLEKTOR = 1;
const double PID_D_KOLLEKTOR = 1;
const double PID_P_SOLE = 1;
const double PID_I_SOLE = 1;
const double PID_D_SOLE = 1;

//Variabeln
byte operationMode = 0;            //Betriebsmodus: 0=aus, 1=Solemodus, 2=Boilermodus
unsigned long now = 0;             //Jeweils aktueller millis()-Wert
bool displayOn = false;            //true, wenn Displays eingeschaltet sein soll
bool boilerHighTemperatur = false; //true, wenn Boiler auf höherer Temperatur ist
bool kollektorAlarm = false;       //true, wenn kollektor zu heiss ist
bool soleAlarm = false;            //true, wenn solepumpe zu heiss ist
float boilerExitTemperature;       //Temperatur, bei der der Boilermodus abgebrochen wird
int brightness;                    //gemessene Helligkeit
bool initializing = false;         //Ist derzeit ein neuer Modus am Initialisieren?
bool tooLowValue = false;          //True, wenn Sollwert das erste mal unterschritten

//PID Variabeln
double PIDInputKollektorPumpe, PIDOutputKollektorPumpe, PIDSetpointKollektorPumpe, PIDInputSolePumpe, PIDOutputSolePumpe, PIDSetpointSolePumpe;

// ==============================
// 3. PIN-Adressen und BUS-Definitionen
// ==============================
// i2c Adressen
Adafruit_LiquidCrystal LCD_00(0x72);
Adafruit_LiquidCrystal LCD_01(0x73);
Adafruit_LiquidCrystal LCD_02(0x75);
Adafruit_LiquidCrystal LCD_03(0x70);

// Analog Pins
const byte FUEHLER_KOLLEKTOR_VL_PIN = A7;   //S0
const byte FUEHLER_KOLLEKTOR_LUFT_PIN = A0; //S1
const byte FUEHLER_BOILER_PIN = A1;         //S2
const byte FUEHLER_BOILER_VL_PIN = A2;      //S4B
const byte FUEHLER_SOLE_VL_PIN = A5;        //S4S
const byte FUEHLER_BOILER_RL_PIN = A3;      //S6B
const byte FUEHLER_SOLE_RL_PIN = A6;        //S6S
const byte FUEHLER_SOLE_PIN = A4;           //S7
const byte POTENTIOMETER_1_PIN = A14;
const byte POTENTIOMETER_2_PIN = A15;

// Digital In Pins
const byte DISPLAY_BUTTON = 22;
const byte FLOW_METER_BOILER = 23;
const byte FLOW_METER_SOLE = 24;

// Digital Out Pins
const byte RELAIS_SOLE_PUMPE = 25;
const byte RELAIS_KOLLEKTOR_PUMPE = 26;
const byte STELLWERK_SOLE_BOILER = 27; //high = höhere Temperatur?
const byte STELLWERK_BOILER_TEMP = 28; //high = Boiler?

// PWM Pins
const byte PWM_SOLE_PUMPE = 4;
const byte PWM_KOLLEKTOR_PUMPE = 13;

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
Potentiometer potentiometer1(POTENTIOMETER_1_PIN);
Potentiometer potentiometer2(POTENTIOMETER_2_PIN);

// Setup der Displays

// Setup der LUX-Meter
Adafruit_TSL2591 luxMeter1 = Adafruit_TSL2591(2591);

// Setup der Pumpen
Pump kollektorPumpe(RELAIS_KOLLEKTOR_PUMPE, PWM_KOLLEKTOR_PUMPE);
Pump solePumpe(RELAIS_SOLE_PUMPE, PWM_SOLE_PUMPE);

// Setup der Timer (leer für ms, 's' für s, 'm' für min, 'd' für Tage)
// Timer über normale Clock
Timer timer200ms(500); //500ms Timer
Timer timer1s(1, 's'); //1s Timer
Timer timer5s(5, 's'); //5s Timer
Timer timer3m(3, 'm'); //3min Timer
Timer timer7d(7, 'd'); //7d Timer

//Timer für bestimmte Funktionen
Timer initialOperationModeTimeout(3, 'm'); //Zeit bevor ein Modus EXIT-Kriterien berücksichtigt
Timer exitTimeout(2, 'm');                 //Solange muss der Sollwert mindestens unterschritten sein, bevor Abbruch
Timer flowMeterBoilerTimeout(100);         //Durchflussmeter 1 Timeout
Timer flowMeterSoleTimeout(100);           //Durchflussmeter 2 Timeout
Timer displayTimeout(15, 's');             //Display-Ausschaltzeit
Timer boilerTimeout(1, 'd');               //Boiler-Ausschaltzeit

//PWM Setup
PID PIDReglerKollektorPumpe(&PIDInputKollektorPumpe, &PIDOutputKollektorPumpe, &PIDSetpointKollektorPumpe, PID_P_KOLLEKTOR, PID_I_KOLLEKTOR, PID_D_KOLLEKTOR, DIRECT);
PID PIDReglerSolePumpe(&PIDInputSolePumpe, &PIDOutputSolePumpe, &PIDSetpointSolePumpe, PID_P_SOLE, PID_I_SOLE, PID_D_SOLE, DIRECT);

// =============
// 5. FUNKTIONEN
// =============

void boilerModusStart()
{
  Serial.println('Boilermodus gestartet');
  digitalWrite(RELAIS_KOLLEKTOR_PUMPE, HIGH);                //Kollektorpumpe einschalten
  digitalWrite(RELAIS_SOLE_PUMPE, LOW);                      //Solepumpe ausschalten
  digitalWrite(STELLWERK_SOLE_BOILER, HIGH);                 //Stellwerk auf Boiler umschalten
  PIDReglerKollektorPumpe.SetMode(1);                        //PID-Regler Kollektorpumpe einschalten
  PIDReglerSolePumpe.SetMode(0);                             //PID-Regler Solepumpe ausschalten
  PIDOutputSolePumpe = 0;                                    //Output der Solepumpe auf 0 stellen
  PIDSetpointKollektorPumpe = SOLL_KOLLEKTOR_VL_BOILERMODUS; //Kollektor Vorlauf Sollwert setzen
  initializing = true;                                       //Initialisierung starten
  initialOperationModeTimeout.executed();                    //Initialisierungs Timer starten
  operationMode = 2;                                         //Modus auf Boiler schalten
  tooLowValue = false;                                       //tooLowValue zurücksetzen
}

void soleModusStart()
{
  Serial.println('Solemodus gestartet');
  digitalWrite(RELAIS_KOLLEKTOR_PUMPE, LOW);               //Kollektorpumpe ausschalten
  digitalWrite(RELAIS_SOLE_PUMPE, HIGH);                   //Solepumpe einschalten
  digitalWrite(STELLWERK_SOLE_BOILER, LOW);                //Stellwerk auf Sole umschalten
  PIDReglerKollektorPumpe.SetMode(1);                      //PID-Regler Kollektorpumpe einschalten
  PIDReglerSolePumpe.SetMode(0);                           //PID-Regler Solepumpe einschalten
  PIDSetpointKollektorPumpe = SOLL_KOLLEKTOR_VL_SOLEMODUS; //Kollektor Vorlauf Sollwert setzen
  PIDSetpointSolePumpe = SOLL_SOLE_VL_SOLEMODUS;           //Sole Vorlauf Sollwert setzen
  initializing = true;                                     //Initialisierung starten
  initialOperationModeTimeout.executed();                  //Initialisierungs Timer starten
  operationMode = 2;                                       //Modus auf Sole schalten
  tooLowValue = false;                                     //tooLowValue zurücksetzen
}

void turnOffModusStart()
{
  Serial.println('Modus Ausgeschaltet');
  digitalWrite(RELAIS_KOLLEKTOR_PUMPE, LOW); //Kollektorpumpe ausschalten
  digitalWrite(RELAIS_SOLE_PUMPE, LOW);      //Solepumpe ausschalten
  digitalWrite(STELLWERK_SOLE_BOILER, LOW);  //Stellwerk auf Sole umschalten
  PIDReglerKollektorPumpe.SetMode(0);        //PID-Regler Kollektorpumpe ausschalten
  PIDReglerSolePumpe.SetMode(0);             //PID-Regler Solepumpe ausschalten
  initializing = false;                      //Initialisierung stoppen
  operationMode = 0;                         //Modus auf aus schalten
  tooLowValue = false;                       //tooLowValue zurücksetzen
}

// ================
// 6. Setup-Sequenz
// ================
void setup()
{
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
  pinMode(PWM_SOLE_PUMPE, OUTPUT);
  pinMode(PWM_KOLLEKTOR_PUMPE, OUTPUT);

  //PWM Setup
  PIDReglerKollektorPumpe.SetOutputLimits(25, 255);
  PIDReglerKollektorPumpe.SetSampleTime(800);
  PIDReglerSolePumpe.SetOutputLimits(25, 255);
  PIDReglerSolePumpe.SetSampleTime(800);

  //Luxmeter Setup
  luxMeter1.setGain(TSL2591_GAIN_LOW);
  luxMeter1.setTiming(TSL2591_INTEGRATIONTIME_200MS);
}

// ===================
// 7. Programmschleife
// ===================
void loop()
{
  //Prüfen, ob je nach Betriebsmodus der entsprechende Durchflussmesser einen Impuls ausgibt
  switch (operationMode)
  {
  case 0:
    return;
  case 1:
    if (digitalRead(FLOW_METER_SOLE) == HIGH && flowMeterSoleTimeout.checkTimer(now) == true)
    {
      Serial.print(now);
      Serial.println("Flow Meter Sole gibt an");
    }
    break;
  case 2:
    if (digitalRead(FLOW_METER_BOILER) == HIGH && flowMeterBoilerTimeout.checkTimer(now) == true)
    {
      Serial.print(now);
      Serial.println("Flow Meter Boiler gibt an");
    }
    break;
  }

  //Prüfen, ob der Display Button gedrückt wird
  if (digitalRead(DISPLAY_BUTTON) == HIGH)
  {
    Serial.println("Display wurde eingeschaltet.");
    displayOn = true;
    displayTimeout.setLastTime(now);
  }

  if (timer200ms.checkTimer(now))
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

    //Timer zurücksetzen
    timer200ms.executed();
  }

  if (timer1s.checkTimer(now))
  {
    if (displayOn)
    {

      //Displays Schreiben
      Serial.print("Kollektor VL Temperatur: ");
      Serial.print(fuehlerKollektorVL.getMeanTemperature(), 1);
      Serial.println(" °C");
      Serial.print("Boiler VL Temperatur: ");
      Serial.print(fuehlerBoilerVL.getMeanTemperature(), 1);
      Serial.println(" °C");
      Serial.print("Boiler Temperatur: ");
      Serial.print(fuehlerBoiler.getMeanTemperature(), 1);
      Serial.println(" °C");
    }
    //PID Regler berechnen
    PIDInputSolePumpe = fuehlerSole.getMeanTemperature();             //Sole-Temperatur auslesen;
    PIDInputKollektorPumpe = fuehlerKollektorVL.getMeanTemperature(); //Kollektor Vorlauftemperatur auslesen
    PIDReglerSolePumpe.Compute();
    PIDReglerKollektorPumpe.Compute();
    solePumpe.setSpeed(PIDOutputSolePumpe);
    kollektorPumpe.setSpeed(PIDOutputKollektorPumpe);

    //Timer zurücksetzen
    timer1s.executed();
  }

  if (timer5s.checkTimer(now) && displayOn)
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
      analogWrite(PWM_SOLE_PUMPE, 255);
      digitalWrite(RELAIS_SOLE_PUMPE, LOW);
      Serial.println("SOLE ALARM!!");
    }

    //Kollektor Alarm?
    if (kollektorAlarm)
    {
      //Kollektorpumpe auf 100% und Wärme in Boiler
      analogWrite(PWM_KOLLEKTOR_PUMPE, 0);
      digitalWrite(STELLWERK_SOLE_BOILER, HIGH);
      Serial.println("KOLLEKTOR ALARM!!");
    }

    while (soleAlarm || kollektorAlarm)
    {
      //Sole Temperatur abfragen und prüfen, ob ausreichend abgekühlt
      fuehlerSole.calculateTemperature();
      if (fuehlerSole.getLastTemperature() < ALARMTEMPERATUR_SOLE - MIN_DIFFERENZ_NACH_ALARM)
      {
        soleAlarm = false;
      }

      //Kollektor VL und Luft Temperatur abfragen und prüfen, ob ausreichend abgekühlt
      fuehlerKollektorLuft.calculateTemperature();
      fuehlerKollektorVL.calculateTemperature();
      if (fuehlerKollektorVL.getLastTemperature() < ALARMTEMPERATUR_KOLLEKTOR_VL - MIN_DIFFERENZ_NACH_ALARM && fuehlerKollektorLuft.getLastTemperature() < ALARMTEMPERATUR_KOLLEKTOR_LUFT - MIN_DIFFERENZ_NACH_ALARM)
      {
        kollektorAlarm = false;
      }
    }

    //Prüfen, ob Initialisierung abgeschlossen ist
    if (initialOperationModeTimeout.checkTimer(now) && initializing)
    {
      Serial.println('Inititialisierung abgeschlossen');
      initializing = false;
    }

    boilerExitTemperature = fuehlerBoiler.getMeanTemperature() + MIN_WAERMER_BOILER; //Mindest Boilertemperatur

    //Entscheidung Betriebsmodus
    switch (operationMode)
    {
    case 0: //Im Modus ausgeschaltet?
    {
      if (brightness > BOILER_START_BRIGHTNESS) //Kriterien für Boilermodus erfüllt?
      {
        boilerModusStart();
      }
      else if (brightness > SOLE_START_BRIGHTNESS) //Kriterien für Solemodus erfüllt?
      {
        soleModusStart();
      }
    }
    break;
    case 1: //Im Modus Sole?
    {
      if (brightness > BOILER_START_BRIGHTNESS) //Kriterien für Boilermodus erfüllt?
      {
        boilerModusStart();
      }

      if (fuehlerSole.getMeanTemperature() > SOLE_EXIT_TEMPERATURE && tooLowValue) //ausreichende Temperatur?
      {
        tooLowValue = false;
      }

      if (fuehlerSole.getMeanTemperature() < SOLE_EXIT_TEMPERATURE && !initializing) //zu tiefe Temperatur im Solemodus?
      {
        if (!tooLowValue)
        {
          tooLowValue = true;
          exitTimeout.executed();
        }
        else if (tooLowValue && exitTimeout.checkTimer(now))
        {
          turnOffModusStart();
        }
      }
    }
    break;
    case 2: //Im Modus Boiler?
    {
      if (fuehlerBoilerVL.getMeanTemperature() > boilerExitTemperature && tooLowValue) //ausreichende Temperatur?
      {
        tooLowValue = false;
      }

      if (fuehlerBoilerVL.getMeanTemperature() < boilerExitTemperature && !initializing) //zu Tiefe Temperatur im Boilermodus
      {
        if (!tooLowValue)
        {
          tooLowValue = true;
          exitTimeout.executed();
        }
        else if (tooLowValue && exitTimeout.checkTimer(now))
        {
          turnOffModusStart();
        }
      }
    }
    break;
    default:
    {
      operationMode = 0;
    }
    }
  }

  if (timer3m.checkTimer(now))
  {
    brightness = luxMeter1.getLuminosity(TSL2591_FULLSPECTRUM); //
    timer3m.executed();
  }

  if (timer7d.checkTimer(now))
  {
    digitalWrite(STELLWERK_BOILER_TEMP, HIGH); //Boiler auf höhere Temperatur stellen
    boilerHighTemperatur = true;
    boilerTimeout.executed();                    //Timer stellen
    timer7d.executed();                        //Timer zurücksetzen
  }

  if (displayOn && displayTimeout.checkTimer(now))
  {
    displayOn = false;
  }

  if (boilerHighTemperatur && boilerTimeout.checkTimer(now))
  {
    boilerHighTemperatur = false;
    digitalWrite(STELLWERK_BOILER_TEMP, true);
  }

  now = millis();
}
