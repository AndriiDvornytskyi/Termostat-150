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
double Kp = 20, Ki = 0.2, Kd = 0.2;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

// Експоненціальний фільтр
double filteredTemp = 0.0;
const double alpha = 0.01;  // Коефіцієнт фільтрації (0.1 - сильне згладжування, 0.5 - швидке)

unsigned long prevSecond = 0;
int readCount = 0;

void setup() {
  Serial.begin(115200);  // Встановлюємо швидкість передачі через серійний порт
  thermo.begin(MAX31865_4WIRE);
  thermo.enableBias(true);
  thermo.autoConvert(true);

  Setpoint = 120.0;  // Задана температура

  pinMode(RELAY_PIN, OUTPUT);
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(0, 255);
  myPID.SetSampleTime(72);
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
  double rawTemp = thermo.temperature(RNOMINAL, RREF);
  
  // Перший раз ініціалізуємо filteredTemp значенням отриманої температури
  if (readCount == 0) {
    filteredTemp = rawTemp;
  } else {
    filteredTemp = alpha * rawTemp + (1 - alpha) * filteredTemp;  // Застосування фільтра
  }

  Input = filteredTemp;  // Передача згладженої температури в ПІД-регулятор
  readCount++;  // Лічильник вимірювань

  myPID.Compute();
  analogWrite(RELAY_PIN, Output);

  // Форматуємо вихід у відповідності до вимог Serial Plotter
  Serial.print(">");
  Serial.print("rawTemp:");
  Serial.print(rawTemp, 5);  // Виведення згладженої температури
  Serial.print(",");
  Serial.print("filteredTemp:");
  Serial.print(filteredTemp, 5);  // Виведення згладженої температури
  Serial.print(",");
  Serial.print("Setpoint:");
  Serial.print(Setpoint, 2);  // Виведення заданої температури
  Serial.print(",");
  Serial.print("Output:");
  Serial.print(Output);
  Serial.print(",");
  // Додаємо вивід P, I, D складових для більш детального моніторингу
  Serial.print("P:");
  Serial.print(myPID.GetPterm());
  Serial.print(",");
  Serial.print("I:");
  Serial.print(myPID.GetIterm());
  Serial.print(",");
  Serial.print("D:");
  Serial.print(myPID.GetDterm());
  Serial.print("\r\n");

  // delay(100);  // Затримка між вимірюваннями
}
