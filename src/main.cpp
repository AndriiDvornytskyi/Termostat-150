/***************************************************/
/*  РОЗКОМЕНТУЙТЕ, ЯКЩО ХОЧЕТЕ ОТРИМУВАТИ ЛОГИ ЧАСУ  */
/***************************************************/
// #define LOGS // Змінна для логів часу

#include <Arduino.h>
#include <Wire.h>
#include <PID_v1.h>
#include <Adafruit_MAX31865.h>

/*****************************************************/
/*              НАЛАШТУВАННЯ  MAX31865               */
/*****************************************************/
#define RREF      430.0  // Опір резистора в MAX31865
#define RNOMINAL  100.0  // Номінал Pt100 (100 Ом)

// Піни SPI (налаштуйте під свій wiring ESP32-S3, якщо не за замовчуванням):
#define MAX_CS_PIN   10
#define MAX_DI_PIN   11  // MOSI
#define MAX_DO_PIN   12  // MISO
#define MAX_CLK_PIN  13  // SCK

// Об’єкт MAX31865 (Pt100)
Adafruit_MAX31865 thermo = Adafruit_MAX31865(MAX_CS_PIN, MAX_DI_PIN, MAX_DO_PIN, MAX_CLK_PIN);

/*****************************************************/
/*           НАЛАШТУВАННЯ КЕРУВАННЯ ТЕНАМИ           */
/*****************************************************/
#define RELAY_PIN  37   // GPIO для ШІМ (перевірте, чи справді підтримує LEDC)

/*****************************************************/
/*            НАЛАШТУВАННЯ ПІД-РЕГУЛЯТОРА            */
/*****************************************************/
double Setpoint, Input, Output;
double Kp = 10.0, Ki = 0.05, Kd = 0.0; // Початкові коефіцієнти

// Створюємо PID-регулятор із бібліотеки PID_v1
// PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT, 0, 255);

// Експоненціальний фільтр
static double filteredTemp = 0.0;
static const double alpha = 0.01;  // коеф. згладжування (0.01 = сильне згладження)

// Щоб у printTask виводити і сире значення теж (для графіків):
static double rawTempGlobal = 0.0; // збережемо останнє "сире" значення з сенсора

/*****************************************************/
/*          ГЛОБАЛЬНІ ЗМІННІ ТА КОНСТАНТИ            */
/*****************************************************/
volatile int readCount = 0;           // Кількість вимірювань за секунду
static const uint16_t pidPeriodMs   = 100;   // Період ПІД, мс
static const uint16_t printPeriodMs = 1000;  // Період виводу, мс

/*****************************************************/
/*                ТАСКИ FREE RTOS                    */
/*****************************************************/

// Таск із вищим пріоритетом (PID) - викликається кожні 100 мс
void pidTask(void *pvParameters)
{
    TickType_t xLastWakeTime = xTaskGetTickCount(); // Отримуємо поточний час
    const TickType_t xFrequency = pdMS_TO_TICKS(pidPeriodMs);
    // myPID.SetOutputLimits(0, 255); // ШІМ 8-біт
    for (;;)
    {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);// Затримка до наступного запуску

#ifdef LOGS // Директива  компіляції логів часу
        unsigned long tStart = micros();
#endif // Кінець директиви

        // 1. Зчитування температури з MAX31865
        double rawTemp = thermo.temperature(RNOMINAL, RREF);
        // Збережемо в глобальну змінну, щоб printTask міг показати
        rawTempGlobal = rawTemp;

#ifdef LOGS
        unsigned long tAfterSensor = micros();
        Serial.print("[LOG] Sensor read time (us): ");
        Serial.println(tAfterSensor - tStart);
#endif

        // 2. Фільтрація
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

        // 3. PID: вхід = filteredTemp, вихід = Output
        Input = filteredTemp;
        myPID.Compute(); // Обчислюємо вихід у змінну Output

#ifdef LOGS
        unsigned long tAfterPID = micros();
        Serial.print("[LOG] PID compute time (us): ");
        Serial.println(tAfterPID - tAfterFilter);
#endif

        // 4. Керуємо нагрівачем через ШІМ (0..255)
        analogWrite(RELAY_PIN, (int)Output);

#ifdef LOGS
        unsigned long tAfterOutput = micros();
        Serial.print("[LOG] analogWrite time (us): ");
        Serial.println(tAfterOutput - tAfterPID);
#endif

        // 5. Лічильник зчитувань
        readCount++;
    }
}

// Таск з нижчим пріоритетом (вивід інформації щосекунди)
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

        int rc = readCount;
        readCount = 0;

        // ------ Текстовий лог, який плотер ігнорує ------
        Serial.print("New second | Readings per second: ");
        Serial.print(rc);
        Serial.print(" | Kp:");
        Serial.print(Kp);
        Serial.print(" Ki:");
        Serial.print(Ki);
        Serial.print(" Kd:");
        Serial.println(Kd);

        // ------ Дані для Serial Plotter (VS Code) ------
        // Починаємо зі знаку '>' і поділяємо назву змінної та значення двокрапкою
        // Кожна пара відокремлена комою
        // Закінчуємо рядок \r\n
        Serial.print(">rawTemp:");
        Serial.print(rawTempGlobal, 2);
        Serial.print(",filteredTemp:");
        Serial.print(filteredTemp, 2);
        Serial.print(",Setpoint:");
        Serial.print(Setpoint, 2);
        Serial.print(",termKp:");
        Serial.print(myPID.GetPterm(), 2);
        Serial.print(",termKi:");
        Serial.print(myPID.GetIterm(), 2);
        Serial.print(",termKd:");
        Serial.print(myPID.GetDterm(), 2);
        Serial.print(",Output:");
        Serial.print(Output, 2);
        Serial.print("\r\n"); // важливо, щоб було \r\n

#ifdef LOGS
        unsigned long tAfterPrint = micros();
        Serial.print("[LOG] Print task time (us): ");
        Serial.println(tAfterPrint - tStart);
#endif
    }
}

/*****************************************************/
/*         ОБРОБКА КОМАНД ІЗ СЕРІЙНОГО ПОРТУ         */
/*****************************************************/

// Допоміжна функція для перетворення рядка на float з перевіркою
bool tryParseFloat(const String &s, float &result)
{
    // Метод toFloat() не завжди відрізняє 0 від помилки.
    // Використаємо strtod (C-функцію).
    char *endPtr = nullptr;
    const char *cstr = s.c_str();
    float val = strtof(cstr, &endPtr);

    // Якщо endPtr вказує на початок рядка, отже число не зчиталося.
    if (endPtr == cstr) {
        return false;
    }

    // Можемо ще перевірити, чи залишилися зайві символи
    // (але в простому варіанті це необов'язково).
    result = val;
    return true;
}

// Викликається в loop() — перевіряє, чи є нові дані, і обробляє команди
void handleSerialCommands()
{
    while (Serial.available() > 0) {
        String line = Serial.readStringUntil('\n');
        line.trim();  // видаляємо пробіли, \r тощо

        if (line.length() == 0) {
            // Порожній рядок, ігноруємо
            continue;
        }

        if (line.equalsIgnoreCase("/help")) {
            Serial.println("Доступні команди:");
            Serial.println("  /help               - показати цю допомогу");
            Serial.println("  /setKp <число>      - встановити Kp");
            Serial.println("  /setKi <число>      - встановити Ki");
            Serial.println("  /setKd <число>      - встановити Kd");
            Serial.println("Приклад: /setKp 12.5");
            continue;
        }

        // Перевіряємо /setKp
        if (line.startsWith("/setKp ")) {
            String arg = line.substring(7); // все після пробілу
            arg.trim();
            float val;
            if (tryParseFloat(arg, val)) {
                Kp = val;
                myPID.SetTunings(Kp, Ki, Kd);
                Serial.print("Kp успішно змінено на: ");
                Serial.println(Kp);
            } else {
                Serial.println("Помилка: введіть правильне число для Kp!");
            }
            continue;
        }

        // Перевіряємо /setKi
        if (line.startsWith("/setKi ")) {
            String arg = line.substring(7);
            arg.trim();
            float val;
            if (tryParseFloat(arg, val)) {
                Ki = val;
                myPID.SetTunings(Kp, Ki, Kd);
                Serial.print("Ki успішно змінено на: ");
                Serial.println(Ki);
            } else {
                Serial.println("Помилка: введіть правильне число для Ki!");
            }
            continue;
        }

        // Перевіряємо /setKd
        if (line.startsWith("/setKd ")) {
            String arg = line.substring(7);
            arg.trim();
            float val;
            if (tryParseFloat(arg, val)) {
                Kd = val;
                myPID.SetTunings(Kp, Ki, Kd);
                Serial.print("Kd успішно змінено на: ");
                Serial.println(Kd);
            } else {
                Serial.println("Помилка: введіть правильне число для Kd!");
            }
            continue;
        }

        // Якщо жодна з відомих команд не підійшла:
        Serial.println("Невідома команда. Введіть /help для списку команд.");
    }
}

/*****************************************************/
/*                     SETUP                         */
/*****************************************************/
void setup()
{
    Serial.begin(115200);
    delay(1000); // невелика пауза для стабільного старту

    // Ініціалізація MAX31865 (Pt100, 4 дроти)
    thermo.begin(MAX31865_4WIRE);

    pinMode(RELAY_PIN, OUTPUT);

    // Початкове значення заданої температури
    Setpoint = 100.0;

    // Налаштування PID
    myPID.SetMode(AUTOMATIC);
    myPID.SetOutputLimits(0, 255);        // ШІМ 8-біт
    myPID.SetSampleTime(pidPeriodMs);     // Період обчислення ПІД: 100 мс

    // Створюємо два FreeRTOS-таски
    xTaskCreatePinnedToCore(
        pidTask,
        "PID Task",
        4096,   // розмір стека (байт)
        NULL,
        2,      // пріоритет (вищий)
        NULL,
        1       // ядро (ESP32-S3 зазвичай 0 або 1)
    );

    xTaskCreatePinnedToCore(
        printTask,
        "Print Task",
        2048,
        NULL,
        1,      // нижчий пріоритет
        NULL,
        1
    );

    // setup() завершується, і далі все виконується в тасках + loop()
}

/*****************************************************/
/*                      LOOP                         */
/*****************************************************/
// У фреймворку Arduino ми можемо обробляти команди у loop()
// Паралельно RTOS-таски працюють у фоновому режимі.
void loop()
{
    handleSerialCommands();
    // Можна додати інші речі, якщо потрібно
    // Але головна робота зараз іде у pidTask та printTask
}
