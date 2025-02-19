#include <Arduino.h>
#include <Wire.h>
#include <PID_v1.h>
#include <Adafruit_MAX31865.h>

#define RELAY_PIN 37  // Вивід для керування транзистором
#define RREF 430.0
#define RNOMINAL 100.0

Adafruit_MAX31865 thermo = Adafruit_MAX31865(10, 11, 12, 13);

// Змінні для ПІД-регулятора
double Setpoint, Input, Output;
double Kp = 20, Ki = 0.2, Kd = 0.2;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

// Експоненціальний фільтр
double filteredTemp = 0.0;
const double alpha = 0.01;

unsigned long prevSecond = 0;
int readCount = 0;
TaskHandle_t tempControlTaskHandle;
TaskHandle_t serialMonitorTaskHandle;

void TempControlTask(void *pvParameters) {
    unsigned long startTime, endTime, executionTime;
    const TickType_t xDelay = pdMS_TO_TICKS(100);
    thermo.begin(MAX31865_4WIRE);
    // thermo.enableBias(true);
    thermo.autoConvert(true);

    Setpoint = 100.0;
    pinMode(RELAY_PIN, OUTPUT);
    myPID.SetMode(AUTOMATIC);
    myPID.SetOutputLimits(0, 255);
    unsigned long rawTemp = 0.0;

    unsigned long tempStart = 0.0;
    unsigned long tempEnd = 0.0;

    while (1) {
        startTime = micros();
        Serial.println("[DEBUG] Start TempControlTask");
        tempStart = micros();
        // Отримання поточної температури
        rawTemp = thermo.temperature(RNOMINAL, RREF);
        tempEnd = micros();
        Serial.print("[DEBUG] Temp Measurement Time: ");
        Serial.println(tempEnd - tempStart);

        // Перший раз ініціалізуємо filteredTemp
        unsigned long filterStart = micros();
        if (readCount == 0) {
            filteredTemp = rawTemp;
        } else {
            filteredTemp = alpha * rawTemp + (1 - alpha) * filteredTemp;
        }
        unsigned long filterEnd = micros();
        Serial.print("[DEBUG] Filtering Time: ");
        Serial.println(filterEnd - filterStart);

        Input = filteredTemp;
        readCount++;

        unsigned long pidStart = micros();
        myPID.Compute();
        unsigned long pidEnd = micros();
        Serial.print("[DEBUG] PID Compute Time: ");
        Serial.println(pidEnd - pidStart);

        unsigned long pwmStart = micros();
        analogWrite(RELAY_PIN, Output);
        unsigned long pwmEnd = micros();
        Serial.print("[DEBUG] PWM Write Time: ");
        Serial.println(pwmEnd - pwmStart);

        endTime = micros();
        executionTime = endTime - startTime;
        myPID.SetSampleTime(executionTime / 1000); // Встановлення нового часу дискретизації

        Serial.print("[DEBUG] Task Execution Time: ");
        Serial.println(executionTime);

        // Вивід даних для Serial Plotter
        Serial.print(">rawTemp:");
        Serial.print(rawTemp, 5);
        Serial.print(",filteredTemp:");
        Serial.print(filteredTemp, 5);
        Serial.print(",Setpoint:");
        Serial.print(Setpoint, 2);
        Serial.print(",Output:");
        Serial.print(Output);
        Serial.print(",ExecutionTime:");
        Serial.print(executionTime);
        Serial.print("\r\n");

        vTaskDelay(xDelay);
    }
}

void SerialMonitorTask(void *pvParameters) {
    const TickType_t xDelay = pdMS_TO_TICKS(1000);
    while (1) {
        Serial.print("new second | Readings per second: ");
        Serial.println(readCount);
        readCount = 0;
        vTaskDelay(xDelay);
    }
}

void setup() {
    Serial.begin(115200);

    xTaskCreatePinnedToCore(TempControlTask, "TempControl", 2048, NULL, 0, &tempControlTaskHandle, 1);
    xTaskCreatePinnedToCore(SerialMonitorTask, "SerialMonitor", 2048, NULL, 1, &serialMonitorTaskHandle, 1);
}

void loop() {
    vTaskDelete(NULL);
}
