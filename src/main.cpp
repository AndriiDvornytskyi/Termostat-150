#include <Arduino.h>
#include <Wire.h>
// #include <SPI.h>
// #include <Adafruit_GFX.h>
// #include <Adafruit_ILI9341.h>
#include <PID_v1.h>
#include <Adafruit_MAX31865.h>

// #define TFT_CS 37  //підключення дисплея TFT
// #define TFT_RST 9
// #define TFT_DC 8
// #define TFT_MOSI 35
// #define TFT_SCK 36
// Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC, TFT_RST);  // Створення об'єкта дисплея

#define RELAY_PIN 37  // вивід для підключення регулюючого транзистора

#define RREF 430.0
#define RNOMINAL 100.0
Adafruit_MAX31865 thermo = Adafruit_MAX31865(10, 11, 12, 13);  //підключення перетвотювача датчика Pt100

// Змінні для ПІД-регулятора
double Setpoint, Input, Output;
double Kp = 2.5, Ki = 0.5, Kd = 1.5;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

void setup() {
  // SPI.begin(TFT_SCK, MISO, TFT_MOSI);  // Налаштування SPI з альтернативними пінами
  // tft.begin();
  // tft.setRotation(1);
  // tft.fillScreen(ILI9341_BLACK);
  // tft.setTextColor(ILI9341_WHITE);
  // tft.setTextSize(2);
  // tft.setCursor(10, 10);
  // tft.print("Termostat 140");

  Serial.begin(115200);
  thermo.begin(MAX31865_4WIRE);  // 4 провідне підключення Pt100

  Setpoint = 100.0;  // Задана температура

  pinMode(RELAY_PIN, OUTPUT);
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(0, 255);  // Вихід від 0 до 255 (ШІМ)
  myPID.SetSampleTime(100);       // Встановлення періоду опитування ПІД-регулятора на 100 мілісекунд (0.1 секунди)
}

void loop() {
  // Отримання поточного опору RTD
  uint16_t rtd = thermo.readRTD();

  // Перевірка статусу помилок
  uint8_t status = thermo.readFault();
  if (status) {
    Serial.print("Fault: ");
    if (status & MAX31865_FAULT_HIGHTHRESH) Serial.println("RTD High Threshold");
    if (status & MAX31865_FAULT_LOWTHRESH) Serial.println("RTD Low Threshold");
    if (status & MAX31865_FAULT_REFINLOW) Serial.println("REFIN- > 0.85 x Bias");
    if (status & MAX31865_FAULT_REFINHIGH) Serial.println("REFIN- < 0.85 x Bias - FORCE- open");
    if (status & MAX31865_FAULT_RTDINLOW) Serial.println("RTDIN- < 0.85 x Bias - FORCE- open");
    if (status & MAX31865_FAULT_OVUV) Serial.println("Under/Over voltage");
    thermo.clearFault();
  }

  // Виведення значення опору
  //Serial.print("RTD value: ");
  //Serial.println(rtd);

  // Отримання поточної температури
  Input = thermo.temperature(RNOMINAL, RREF);
  // Виклик функції обчислення ПІД-регулятора
  myPID.Compute();

  // Застосування виходу ПІД-регулятора до виконавчого механізму
  analogWrite(RELAY_PIN, Output);

  // Виведення значень для налагодження
  Serial.print("Setpoint: ");
  Serial.print(Setpoint);
  Serial.print(" C | Temperature: ");
  Serial.print(Input, 3);
  Serial.print(" C | Output: ");
  Serial.println(Output);

  delay(100);  // Затримка для стабільного читання датчика
}
