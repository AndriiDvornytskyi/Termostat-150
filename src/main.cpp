#include <Arduino.h>
#include <Wire.h>
#include <PID_v1.h>
#include <Adafruit_MAX31865.h>

#define RELAY_PIN 37  // вивід для підключення регулюючого транзистора

#define RREF 430.0
#define RNOMINAL 100.0
Adafruit_MAX31865 thermo = Adafruit_MAX31865(10, 11, 12, 13);  // SPI MAX31865 (CS-10, MOSI-11, MISO-12, CLK-13)

// Змінні для ПІД-регулятора
double Setpoint, Input, Output;
double Kp = 2.5, Ki = 0.5, Kd = 1.5;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

void setup() {
  Serial.begin(115200);
  Serial.println("Init succesful");
  thermo.begin(MAX31865_4WIRE);  // 4-х провідне підключення Pt100
  Setpoint = 100.0;  // Задана температура
  pinMode(RELAY_PIN, OUTPUT);
  pinMode(5, OUTPUT);
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(0, 255);  // Вихід від 0 до 255 (ШІМ)
  myPID.SetSampleTime(100);       // Встановлення періоду опитування ПІД-регулятора на 100 мілісекунд (0.1 секунди)
}

void loop() {
  uint16_t rtd = thermo.readRTD(); // Отримання поточного опору RTD
  uint8_t status = thermo.readFault(); // Перевірка статусу помилок
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
  Serial.print(" RTD value: ");
  Serial.println(rtd);

  // Отримання поточної температури
  Input = thermo.temperature(RNOMINAL, RREF);
  digitalWrite(5, HIGH);
  // Виклик функції обчислення ПІД-регулятора
  myPID.Compute();

  // Застосування виходу ПІД-регулятора до виконавчого механізму
  analogWrite(RELAY_PIN, Output);

  // Виведення значень для налагодження
  Serial.print("Setpoint: ");
  Serial.print(Setpoint);
  Serial.print(" C Temperature: ");
  Serial.print(Input), 
  Serial.print(" C  Output: ");
  Serial.print(Output);
  

  delay(100);  // Затримка для стабільного читання датчика
}
