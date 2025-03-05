#include <Arduino.h>
#include <Wire.h>
#include <PID_v1.h>
#include <Adafruit_MAX31865.h>

#define RREF 430.0
#define RNOMINAL 100.0

#define MAX_CS_PIN 10
#define MAX_DI_PIN 11
#define MAX_DO_PIN 12
#define MAX_CLK_PIN 13

Adafruit_MAX31865 thermo = Adafruit_MAX31865(MAX_CS_PIN, MAX_DI_PIN, MAX_DO_PIN, MAX_CLK_PIN);

#define RELAY_PIN 37

double Setpoint, Input, Output;
double Kp = 10.0, Ki = 0.02, Kd = 0.05;

// Створення PID регулятора з відповідним діапазоном вихідного сигналу
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT, 0, 4095);

static double filteredTemp = 0.0;
static double alpha = 0.01;
static double rawTempGlobal = 0.0;

volatile int readCount = 0;
static const uint16_t pidPeriodMs = 100;
static const uint16_t printPeriodMs = 10000;

void pidTask(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(pidPeriodMs);
    static bool filterInitialized = false; // нова змінна для ініціалізації фільтра
    for (;;) {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        double rawTemp = thermo.temperature(RNOMINAL, RREF);
        rawTempGlobal = rawTemp;
        if (!filterInitialized) {
            filteredTemp = rawTemp;
            filterInitialized = true;
        } else {
            filteredTemp = alpha * rawTemp + (1.0 - alpha) * filteredTemp;
        }
        Input = filteredTemp;
        myPID.Compute();
        analogWrite(RELAY_PIN, (int)Output);
        readCount++;
    }
}

void printTask(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(printPeriodMs);
    for (;;) {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        int rc = readCount;
        readCount = 0;
        Serial.print("New second | Readings per second: ");
        Serial.print(rc);
        Serial.print(" | Kp:");
        Serial.print(Kp);
        Serial.print(" Ki:");
        Serial.print(Ki);
        Serial.print(" Kd:");
        Serial.println(Kd);
        Serial.print(">rawTemp:");
        Serial.print(rawTempGlobal, 5);
        Serial.print(",filteredTemp:");
        Serial.print(filteredTemp, 5);
        Serial.print(",Setpoint:");
        Serial.print(Setpoint, 2);
        Serial.print(",Pterm:");
        Serial.print(myPID.GetPterm(), 3);
        Serial.print(",Iterm:");
        Serial.print(myPID.GetIterm(), 3);
        Serial.print(",Dterm:");
        Serial.print(myPID.GetDterm(), 3);
        Serial.print(",Output:");
        Serial.print(myPID.GetPercentageOutput(), 3);
        Serial.print("\r\n");
    }
}

bool tryParseFloat(const String &s, float &result) {
    char *endPtr = nullptr;
    const char *cstr = s.c_str();
    float val = strtof(cstr, &endPtr);
    if (endPtr == cstr) {
        return false;
    }
    result = val;
    return true;
}

void handleSerialCommands() {
    while (Serial.available() > 0) {
        String line = Serial.readStringUntil('\n');
        line.trim();
        if (line.length() == 0) {
            continue;
        }
        if (line.equalsIgnoreCase("/help")) {
            Serial.println("/setKp <число>");
            Serial.println("/setKi <число>");
            Serial.println("/setKd <число>");
            Serial.println("/setTau <число>");
            continue;
        }
        if (line.startsWith("/setKp ")) {
            String arg = line.substring(7);
            arg.trim();
            float val;
            if (tryParseFloat(arg, val)) {
                Kp = val;
                myPID.SetTunings(Kp, Ki, Kd);
                Serial.print("Kp: ");
                Serial.println(Kp);
            }
            continue;
        }
        if (line.startsWith("/setKi ")) {
            String arg = line.substring(7);
            arg.trim();
            float val;
            if (tryParseFloat(arg, val)) {
                Ki = val;
                myPID.SetTunings(Kp, Ki, Kd);
                Serial.print("Ki: ");
                Serial.println(Ki);
            }
            continue;
        }
        if (line.startsWith("/setKd ")) {
            String arg = line.substring(7);
            arg.trim();
            float val;
            if (tryParseFloat(arg, val)) {
                Kd = val;
                myPID.SetTunings(Kp, Ki, Kd);
                Serial.print("Kd: ");
                Serial.println(Kd);
            }
            continue;
        }
        if (line.startsWith("/setTau ")) {
            String arg = line.substring(7);
            arg.trim();
            float val;
            if (tryParseFloat(arg, val)) {
                alpha = val;
                Serial.print("Tau: ");
                Serial.println(val);
            }
            continue;
        }
        Serial.println("Unknown command");
    }
}

void setup() {
    Serial.begin(115200);
    delay(1000);
    thermo.begin(MAX31865_4WIRE);
    pinMode(RELAY_PIN, OUTPUT);
    Setpoint = 60.0;
    
    // Встановлення 16-бітного режиму ШІМ
    analogWriteResolution(12);
    
    myPID.SetMode(AUTOMATIC);
    myPID.SetOutputLimits(0, 100);
    myPID.SetSampleTime(pidPeriodMs);
    xTaskCreatePinnedToCore(pidTask, "PID Task", 4096, NULL, 2, NULL, 1);
    xTaskCreatePinnedToCore(printTask, "Print Task", 2048, NULL, 1, NULL, 1);
}

void loop() {
    handleSerialCommands();
}
