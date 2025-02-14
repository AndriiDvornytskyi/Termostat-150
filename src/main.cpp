#include <Arduino.h>
#include <Wire.h>
#include <PID_v1.h>
#include <Adafruit_MAX31865.h>

#define RELAY_PIN 37  // Вивід для керування транзистором

#define RREF 430.0
#define RNOMINAL 100.0
Adafruit_MAX31865 thermo = Adafruit_MAX31865(10, 11, 12, 13);  // Підключення MAX31865

// Змінні для ПІД-регулятора
double Setpoint, Input, Output;
double Kp = 1.5, Ki = 0.3, Kd = 2.5;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

unsigned long prevSecond = 0;
int readCount = 0;

void setup() {
  Serial.begin(115200);
  thermo.begin(MAX31865_4WIRE);
  thermo.enableBias(true);
  thermo.autoConvert(true);

  Setpoint = 55.0;  // Задана температура

  pinMode(RELAY_PIN, OUTPUT);
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(0, 255);
  myPID.SetSampleTime(50);  // 50 мс

  prevSecond = millis();
}

void loop() {
  unsigned long currentMillis = millis();

  // Виводимо "new second" кожну секунду
  if (currentMillis - prevSecond >= 1000) {
    Serial.print("new second | Readings per second: ");
    Serial.println(readCount);
    readCount = 0;
    prevSecond = currentMillis;
  }

  // Отримання поточної температури
  Input = thermo.temperature(RNOMINAL, RREF);
  readCount++;  // Лічильник вимірювань

  myPID.Compute();
  analogWrite(RELAY_PIN, Output);

  Serial.print("Setpoint: ");
  Serial.print(Setpoint);
  Serial.print(" C | Temperature: ");
  Serial.print(Input, 3);
  Serial.print(" C | Output: ");
  Serial.println(Output);

  delay(100);  // Затримка між вимірюваннями
}
