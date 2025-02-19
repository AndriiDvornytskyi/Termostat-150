/***************************************************/
/*  РОЗКОМЕНТУЙТЕ, ЯКЩО ХОЧЕТЕ ОТРИМУВАТИ ЛОГИ ЧАСУ  */
/***************************************************/
//#define LOGS

#include <Arduino.h>
#include <Wire.h>
#include <PID_v1.h>
#include <Adafruit_MAX31865.h>

/*****************************************************/
/*              НАЛАШТУВАННЯ  MAX31865                */
/*****************************************************/
#define RREF      430.0  // Опір резистора в MAX31865
#define RNOMINAL  100.0  // Номінал Pt100 (100 Ом)

// Приклад підключення до ESP32-S3 (налаштуйте під свій wiring!):
// За замовчуванням SPI на ESP32-S3 Arduino:
// SCK  = 13
// MISO = 12
// MOSI = 11
// SS   = 10 (можна інший)
#define MAX_CS_PIN   10
#define MAX_DI_PIN   11  // MOSI
#define MAX_DO_PIN   12  // MISO
#define MAX_CLK_PIN  13  // SCK

// Створюємо об’єкт MAX31865
Adafruit_MAX31865 thermo = Adafruit_MAX31865(MAX_CS_PIN, MAX_DI_PIN, MAX_DO_PIN, MAX_CLK_PIN);

/*****************************************************/
/*           НАЛАШТУВАННЯ КЕРУВАННЯ ТЕНАМИ           */
/*****************************************************/
// Пін для керування нагрівом (через транзистор / SSR):
// УВАГА: Оберіть пін, який підтримує ШІМ (LEDC) на ESP32-S3!
#define RELAY_PIN 9  // Приклад, переконайтеся що GPIO9 доступний/вільний

/*****************************************************/
/*            НАЛАШТУВАННЯ ПІД-РЕГУЛЯТОРА            */
/*****************************************************/
double Setpoint, Input, Output;
double Kp = 20, Ki = 5, Kd = 0;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

// Параметри фільтра (експоненційний)
static double filteredTemp = 0.0;
static const double alpha = 0.01;  // коеф. згладжування

/*****************************************************/
/*          ГЛОБАЛЬНІ ЗМІННІ ТА КОНСТАНТИ            */
/*****************************************************/
volatile int readCount = 0;           // Кількість вимірювань за секунду
static const uint16_t pidPeriodMs = 100;   // Період ПІД, мс
static const uint16_t printPeriodMs = 1000; // Період виводу, мс

/*****************************************************/
/*                ТАСКИ FREE RTOS                    */
/*****************************************************/

// Таск із вищим пріоритетом (для PID) - викликається кожні 100 мс
void pidTask(void *pvParameters)
{
    // Для гарантованого інтервалу використаємо vTaskDelayUntil
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(pidPeriodMs);

    for (;;)
    {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);

#ifdef LOGS
        unsigned long tStart = micros();
#endif

        // 1. Зчитування температури з MAX31865
        double rawTemp = thermo.temperature(RNOMINAL, RREF);

#ifdef LOGS
        unsigned long tAfterSensor = micros();
        Serial.print("[LOG] Sensor read time (us): ");
        Serial.println(tAfterSensor - tStart);
#endif

        // 2. Фільтрація температури
        if (readCount == 0) {
            // Якщо перший раз, ініціалізуємо фільтр
            filteredTemp = rawTemp;
        } else {
            filteredTemp = alpha * rawTemp + (1.0 - alpha) * filteredTemp;
        }

#ifdef LOGS
        unsigned long tAfterFilter = micros();
        Serial.print("[LOG] Filtering time (us): ");
        Serial.println(tAfterFilter - tAfterSensor);
#endif

        // 3. PID
        Input = filteredTemp;
        myPID.Compute(); // Обчислюємо вихід
        double pidOut = Output;

#ifdef LOGS
        unsigned long tAfterPID = micros();
        Serial.print("[LOG] PID compute time (us): ");
        Serial.println(tAfterPID - tAfterFilter);
#endif

        // 4. Керуємо нагрівачем через ШІМ
        // У Arduino-ESP32 analogWrite() -> LEDC, тож duty 0..255
        analogWrite(RELAY_PIN, (int)pidOut);

#ifdef LOGS
        unsigned long tAfterOutput = micros();
        Serial.print("[LOG] analogWrite time (us): ");
        Serial.println(tAfterOutput - tAfterPID);
#endif

        // 5. Збільшуємо лічильник зчитувань
        readCount++;
    }
}

// Таск з нижчим пріоритетом (вивід інформації кожну 1 с)
void printTask(void *pvParameters)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(printPeriodMs);

    for (;;)
    {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);

#ifdef LOGS
        unsigned long tStart = micros();
#endif

        // Локальна копія readCount (атомарне читання)
        int rc = readCount;
        readCount = 0;

        // Вивід
        Serial.print("new second | Readings per second: ");
        Serial.println(rc);

        Serial.print("Temperature (filtered): ");
        Serial.print(filteredTemp, 2);
        Serial.print(" °C | Setpoint: ");
        Serial.print(Setpoint, 2);
        Serial.print(" | Output: ");
        Serial.println(Output, 1);

#ifdef LOGS
        unsigned long tAfterPrint = micros();
        Serial.print("[LOG] Print task time (us): ");
        Serial.println(tAfterPrint - tStart);
#endif
    }
}

/*****************************************************/
/*                     SETUP (Arduino)               */
/*****************************************************/
void setup()
{
    Serial.begin(115200);

    // Ініціалізація MAX31865
    thermo.begin(MAX31865_4WIRE); // Для Pt100 4-wire
    // У бібліотеці Adafruit MAX31865 зазвичай:
    //  thermo.enableBias(true);       // Автоматично
    //  thermo.autoConvert(true);      // Автоматично
    // Можна налаштувати додатково, але часто вистачає .begin(...)

    // Налаштуємо пін для ШІМ (можливо треба pinMode, але для analogWrite це не завжди обов’язково)
    pinMode(RELAY_PIN, OUTPUT);

    // Початкове значення заданої температури
    Setpoint = 120.0;

    // Параметри PID
    myPID.SetMode(AUTOMATIC);
    myPID.SetOutputLimits(0, 255);          // ШІМ 8-біт
    myPID.SetSampleTime(pidPeriodMs);       // Період обчислення 100 мс

    // Створюємо RTOS-таски (Arduino-ESP32)
    xTaskCreatePinnedToCore(
        pidTask,        // Функція таску
        "PID Task",     // Ім'я таску
        4096,           // Розмір стека (байт)
        NULL,           // Параметр
        2,              // Пріоритет (вищий)
        NULL,           // Хендл таску (якщо треба)
        1               // Номер ядра (ESP32-S3 може бути 0 або 1)
    );

    xTaskCreatePinnedToCore(
        printTask,
        "Print Task",
        2048,
        NULL,
        1,  // нижчий пріоритет
        NULL,
        1   // теж саме ядро, або можна 0
    );
}

void loop()
{
    // Порожній, оскільки вся робота йде у тасках FreeRTOS
}