// Diese Datei beinhaltet alle Pin-Definitionen und Bus-Adressen

// i2c Adressen
Adafruit_LiquidCrystal LCD_00(0x72);
Adafruit_LiquidCrystal LCD_01(0x73);
Adafruit_LiquidCrystal LCD_02(0x75);
Adafruit_LiquidCrystal LCD_03(0x70);

// Analog Pins
const byte FUEHLER_KOLLEKTOR_VL = A7; //S0
const byte FUELHER_KOLLEKTOR_LUFT = A0; //S1
const byte FUEHLER_BOILER = A1; //S2
const byte FUEHLER_BOILER_VL = A2; //S4B
const byte FUEHLER_SOLE_VL = A5; //S4S
const byte FUEHLER_BOILER_RL = A3; //S6B
const byte FUEHLER_SOLE_RL = A6; //S6S
const byte FUEHLER_SOLE = A4; //S7
const byte POTENTIOMETER_01 = A14;
const byte POTENTIOMETER_02 = A15;

// Digital Pins
const byte RELAIS_SOLE_PUMPE = 22;
const byte RELAIS_KOLLEKTOR_PUMPE = 23;
const byte STELLWERK_SOLE_BOILER = 24;
const byte STELLWERK_BOILER_TEMP = 25;

// PWM Pins
const byte PWM_SOLE_PUMPE = 4;
const byte PWM_KOLLEKTOR_PUMPE = 13;
