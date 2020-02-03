//Software für die Funktionen gemäss README-Datei
//
//STRUKTUR:
// 1. HEADER-Dateien und Libaries
// 2. Variabeln und Konstanten
// 3. PIN-Adressen und BUS-Definitionen
// 4. Funktionen
// 5. Initialisierungen
// 6. Setup Sequenz
// 7. Programmschleife

// ==============================
// 1. HEADER-Dateien und Libaries
// ==============================
#include "Wire.h" //Für I2C Bus
#include "PID_v1.h" //Für PID-Regelung
#include "Adafruit_LiquidCrystal.h" //Für Displays
#include "Classes.h" //Eigene Klassen
#include "Adafruit_Sensor.h" //Für Adafruit Sensoren
#include "Adafruit_TSL2591.h" //Für Adafruit LUX-Sensoren

// ===========================
// 2. Variabeln und Konstanten
// ===========================
// Konstanten
const float FUEHLER_KOLLEKTOR_VL_KORREKTURFAKTOR = 1.07; //Korrekturfaktor S0
const float FUEHLER_KOLLEKTOR_LUFT_KORREKTURFAKTOR = 1.07; //Korrekturfaktor S1
const float FUEHLER_BOILER_KORREKTURFAKTOR = 1.07; //Korrekturfaktor S2
const float FUEHLER_BOILER_VL_KORREKTURFAKTOR = 1.07; //Korrekturfaktor S4B
const float FUEHLER_SOLE_VL_KORREKTURFAKTOR = 1.07; //Korrekturfaktor S4S
const float FUEHLER_BOILER_RL_KORREKTURFAKTOR = 1.07; //Korrekturfaktor S6B
const float FUEHLER_SOLE_RL_KORREKTURFAKTOR = 1.07; //Korrekturfaktor S6S
const float FUEHLER_SOLE_KORREKTURFAKTOR = 1.07; //Korrekturfaktor S7

//Variabeln
byte operationMode = 0; //Betriebsmodus: 0=aus, 1=Solemodus, 2=Boilermodus
unsigned long now = 0; //Jeweils aktueller millis()-Wert
bool displayOn = false; //true, wenn Displays eingeschaltet sein soll

// ==============================
// 3. PIN-Adressen und BUS-Definitionen
// ==============================
// i2c Adressen
Adafruit_LiquidCrystal LCD_00(0x72);
Adafruit_LiquidCrystal LCD_01(0x73);
Adafruit_LiquidCrystal LCD_02(0x75);
Adafruit_LiquidCrystal LCD_03(0x70);


// Analog Pins
const byte FUEHLER_KOLLEKTOR_VL_PIN = A7; //S0
const byte FUEHLER_KOLLEKTOR_LUFT_PIN = A0; //S1
const byte FUEHLER_BOILER_PIN = A1; //S2
const byte FUEHLER_BOILER_VL_PIN = A2; //S4B
const byte FUEHLER_SOLE_VL_PIN = A5; //S4S
const byte FUEHLER_BOILER_RL_PIN = A3; //S6B
const byte FUEHLER_SOLE_RL_PIN = A6; //S6S
const byte FUEHLER_SOLE_PIN = A4; //S7
const byte POTENTIOMETER_1_PIN = A14;
const byte POTENTIOMETER_2_PIN = A15;

// Digital In Pins
const byte DISPLAY_BUTTON = 22;
const byte FLOW_METER_BOILER = 23;
const byte FLOW_METER_SOLE = 24;


// Digital Out Pins
const byte RELAIS_SOLE_PUMPE = 25;
const byte RELAIS_KOLLEKTOR_PUMPE = 26;
const byte STELLWERK_SOLE_BOILER = 27;
const byte STELLWERK_BOILER_TEMP = 28;

// PWM Pins
const byte PWM_SOLE_PUMPE = 4;
const byte PWM_KOLLEKTOR_PUMPE = 13;

// =============
// 4. FUNKTIONEN
// =============

// ====================
// 5. INITIALISIERUNGEN
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

  // Setup der Timer (leer für ms, 's' für s, 'm' für min, 'd' für Tage)
  // Timer über normale Clock
  Timer timer200ms(500); //500ms Timer
  Timer timer1s(1, 's'); //1s Timer
  Timer timer3m(2, 'm'); //3min Timer
  Timer timer1h(1, 'h'); //1h Timer
  Timer timer7d(7, 'd'); //7d Timer

  //Timer für bestimmte Funktionen
  Timer flowMeterBoilerTimeout(100); //Durchflussmeter 1 Timeout
  Timer flowMeterSoleTimeout(100); //Durchflussmeter 2 Timeout
  Timer displayTimeout(15, 'm'); //Display-Ausschaltzeit
  Timer boilerTimeout(1, 'd'); //Boiler-Ausschaltzeit
// ================
// 6. Setup-Sequenz
// ================
void setup() {
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
}


// ===================
// 7. Programmschleife
// ===================
void loop() {
  //Prüfen, ob je nach Betriebsmodus der entsprechende Durchflussmesser einen Impuls ausgibt
  switch(operationMode){
    case 0:
      return;
    case 1:
      if(digitalRead(FLOW_METER_SOLE) == HIGH && flowMeterSoleTimeout.checkTimer(now) == true) {
        Serial.print(now);
        Serial.println("Flow Meter Sole gibt an");
        }
      break;
    case 2:
      if(digitalRead(FLOW_METER_BOILER) == HIGH && flowMeterBoilerTimeout.checkTimer(now) == true) {
        Serial.print(now);
        Serial.println("Flow Meter Boiler gibt an");
        }
      break;
    }

  //Prüfen, ob der Display Button gedrückt wird
  if (digitalRead(DISPLAY_BUTTON) == HIGH) {
    displayOn = true;
    displayTimeout.setLastTime(now);
    }

now = millis();
}
