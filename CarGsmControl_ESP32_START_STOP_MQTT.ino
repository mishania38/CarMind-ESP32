#include <WiFi.h>
#include <PubSubClient.h>
#include <OneWire.h>
#include <esp_task_wdt.h>
#include <LittleFS.h>
#include <WiFiClientSecure.h>

#include "wifi_manager.h"
#include "mqtt_manager.h"
#include "config.h"  // содержит MQTT_BROKER, MQTT_USER, MQTT_PASS, MQTT_CLIENT_ID, MQTT_PORT

#define SerialMon Serial

/*------------------------------Порты вывода ULN2003-32-38-----------------------*/
#define IGN_PIN         23           // пин зажигание
#define STARTER_PIN     22           // пин стартер
#define ACC_PIN         21           // пин ACC
#define CAR_OP_PIN      19           // сигнал на отпирание
#define CAR_CL_PIN      18           // сигнал на запирание
#define BTN_LED_PIN     5            // Сигнал на подсветку кнопки Старт-Стоп
#define HEAT_ENG_PIN    4            // сигнал на подогрев двигателя

/*------------------------------Порты ввода аналогового сигнала------------------*/
#define TACH_PIN       A3            // Сигнал с тахометра
#define START_BTN      A0            // Сигнал с кнопки Старт/Стоп
#define BAT_V          A7            // Напряжение аккумулятора

// Константы
#define ENGINE_RPM_THRESHOLD 2000
#define STARTER_TIMEOUT_MS   3000

// Глобальные переменные
WiFiClientSecure secureClient;
MQTTManager mqtt(secureClient);
WiFiManager wifi;

OneWire ds18b20(15);

TaskHandle_t Task1;
TaskHandle_t Task2;

/*------------------------------Переменные хранения статуса-----------------------------*/
bool isAlarmEnabled = false;                  // Флаг замков дверей. Закрыт ли автомобиль
bool isEngineHeaterActive = false;            // Флаг включенного подогревателя двигателя
bool isRemoteEngineStarted = false;           // Флаг удаленного запуска двигателя
bool isEngineRunning = false;                 // Флаг заведенного двигателя
volatile bool isStartButtonPressed = false;   // Флаг нажатий кнопки старт-стоп, меняется обработчиком прерываний
bool isStatusCheckRequired = false;           // Флаг на отправку параметрии на сервер

/*------------------------------Переменные с таймерами----------------------------------*/
unsigned long remoteEngineStartTime = 0;      // Время, когда был запущен двигатель
unsigned long engineRunDuration = 900000;     // Интервал, на который двигатель запускается
unsigned long lastMqttUpdate = 0;             // Время последнего обновления соединения
unsigned int engineStartCountdown = 0;        // Обратный отсчет времени работы двигателя

String actionList = "";        // Переменная для хранения команды от телефона
String actionValue = "";

int countNetError = 0;                      // Количество неудачных попыток коннекта (после 3-х рестарт модема и обнуление)
int totalcountNetError = 0;                 // Количество рестартов модема

// Топики Publish
const char startengine[] =     "fiat/startengine";
const char alarmon[] =         "fiat/alarmon";
const char batteryvolt[] =     "fiat/batteryvolt";
const char heatengine[] =      "fiat/heatengine";
const char cartemp[] =         "fiat/cartemp";
const char totalerrorcount[] = "fiat/totalerrorcount";
const char startperiod[] =     "fiat/startperiod";
const char rpminfo[] =         "fiat/rpminfo";
const char startTimer[] =      "fiat/starttimer";

// Прототипы
void StartEngine(bool onTimer);
void StopEngine();
float refreshVoltage();
float refreshTemperature();
void CarOpen();
void CarClose();
void EngineHeat(bool on);
void CheckStatus();
void ShedulerAction();
void StartStopThread();
void MqttThread();
void AllPinOff();
void Task1code(void *parameter);
void Task2code(void *parameter);
IRAM_ATTR void myIsr();
void MqttCallback(char* topic, byte* payload, unsigned int len);

extern const uint8_t certs_start[] asm("_binary_certs_ar_start");
extern const uint8_t certs_end[]   asm("_binary_certs_ar_end");

void setup() {
  SerialMon.begin(115200);

  pinMode(TACH_PIN, INPUT_PULLDOWN);
  pinMode(START_BTN, INPUT_PULLDOWN);
  pinMode(BAT_V, INPUT);

  pinMode(IGN_PIN, OUTPUT);
  pinMode(STARTER_PIN, OUTPUT);
  pinMode(ACC_PIN, OUTPUT);
  pinMode(BTN_LED_PIN, OUTPUT);

  attachInterrupt(START_BTN, myIsr, RISING); // Активируем прерывание по нажатию кнопки старт-стоп

  AllPinOff();

  secureClient.setInsecure();
  



  // Настройки подключения к сети и MQTT брокеру
  wifi.begin();
  mqtt.begin(MQTT_BROKER, MQTT_PORT, MQTT_USER, MQTT_PASS, MQTT_CLIENT_ID);
  mqtt.setCallback(MqttCallback);

  xTaskCreatePinnedToCore(Task1code, "Task1", 20000, NULL, 1, &Task1, 0);
  delay(500);
  xTaskCreatePinnedToCore(Task2code, "Task2", 20000, NULL, 1, &Task2, 1);
  delay(500);

  esp_task_wdt_deinit();
}

void loop() {}

void Task1code(void *parameter) {
  for (;;) {
    MqttThread();
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

void Task2code(void *parameter) {
  for (;;) {
    StartStopThread();
    vTaskDelay(pdMS_TO_TICKS(500));
    ShedulerAction();
    vTaskDelay(pdMS_TO_TICKS(500));
  }
}

void MqttCallback(char* topic, byte* payload, unsigned int len) {
  String _topic = String(topic);
  String _val = "";

  for (unsigned int i = 0; i < len; i++) {
    _val += (char)payload[i];
  }

  actionList = _topic;
  actionValue = _val;

  SerialMon.print("Message arrived [");
  SerialMon.print(topic);
  SerialMon.print("]: ");
  SerialMon.write(payload, len);
  SerialMon.println();
}

void MqttThread() {
  if (!mqtt.isConnected()) { // Подключение к серверу MQTT
    countNetError++; 
    if (countNetError > 2) { //Отслеживание дисконнектов. После 3-х проверка wi-fi соединения
      countNetError = 0;
      wifi.checkConnection();
    }

    if (millis() - lastMqttUpdate > 10000L) {
      lastMqttUpdate = millis();
      if (mqtt.connect()) {
        countNetError = 0;
        lastMqttUpdate = 0;
      }
    }
    return;
  }

  if (isStatusCheckRequired) {
    CheckStatus();
  }

  mqtt.loop();
}

void CheckStatus() {                    // // Сбор данных о состоянии автомобиля и отправка данных на сервер   // Ядро1
  mqtt.publish(startengine, isEngineRunning ? "1" : "0");
  mqtt.publish(alarmon, isAlarmEnabled ? "1" : "0");
  mqtt.publish(batteryvolt, String(refreshVoltage()).c_str());
  mqtt.publish(heatengine, isEngineHeaterActive ? "1" : "0");
  mqtt.publish(cartemp, String(refreshTemperature()).c_str());
  mqtt.publish(totalerrorcount, String(totalcountNetError).c_str());
  mqtt.publish(startperiod, String(engineRunDuration / 60000).c_str());
  mqtt.publish(rpminfo, String(analogRead(TACH_PIN)).c_str());
  mqtt.publish(startTimer, String(engineStartCountdown).c_str());

  isStatusCheckRequired = false;
}

void StartEngine(bool onTimer) {        // Запускаем двигатель
  if (analogRead(TACH_PIN) > ENGINE_RPM_THRESHOLD) return; //Проверяем заведен ли двигатель

  digitalWrite(IGN_PIN, HIGH);

  // Прогрев свечей накала в зависимости от температуры в салоне
  float temp = refreshTemperature();
  delay(temp <= 0 ? 10000 : temp < 15 ? 500 : 100);

  digitalWrite(STARTER_PIN, HIGH);      // Включаем стартер пока двигатель не запустится
  unsigned long startTime = millis();   // Но не более 3 секунд
  while (analogRead(TACH_PIN) <= ENGINE_RPM_THRESHOLD &&
         millis() - startTime < STARTER_TIMEOUT_MS) {
    delay(100);
  }
  digitalWrite(STARTER_PIN, LOW);     // Выключаем стартер
  delay(500);

  if (analogRead(TACH_PIN) >= ENGINE_RPM_THRESHOLD) { // Проверяем по тахометру, завелся ли двигатель
    if (onTimer && isAlarmEnabled) {      // Если стоял флаг таймера и авто под охраной,
      remoteEngineStartTime = millis();   // то сбрасываем счетчик
      lastMqttUpdate = millis();
      isRemoteEngineStarted = true;       // и ставим флаг активного таймера на автозапуск
    } else {
      digitalWrite(BTN_LED_PIN, HIGH);    // Если без охраны, то просто включаем кнопку старт-стоп
    }
  } else {                            // Если же двигатель не завелся, то:
    digitalWrite(IGN_PIN, LOW);       // Выключаем зажигание
    digitalWrite(BTN_LED_PIN, LOW);   // Выключаем подсветку кнопки Старт-стоп
    delay(500);
  }
}

void StopEngine() {                     // Глушим двигатель
  digitalWrite(IGN_PIN, LOW);
  digitalWrite(ACC_PIN, LOW);
  digitalWrite(BTN_LED_PIN, LOW);
}

float refreshVoltage() {                // Замер напряжения на входе через делитель напряжения
  return ((analogRead(BAT_V) * 6.08 * 3.30 ) / 4096);
}

float refreshTemperature() {            // Замер температуры в салоне
  ds18b20.reset();
  ds18b20.write(0xCC);
  ds18b20.write(0x44);
  delay(1000);
  ds18b20.reset();
  ds18b20.write(0xCC);
  ds18b20.write(0xBE);
  int16_t rawTemp = (ds18b20.read() | (ds18b20.read() << 8));
  return rawTemp * 0.0625;
}

void StartStopThread() {                // Обработчик нажатий кнопки старт-стоп, таймер прогрева двигателя,
  isEngineRunning = (analogRead(TACH_PIN) >= ENGINE_RPM_THRESHOLD);  // Проверяем, запущен ли двигатель

  if (isStartButtonPressed) {     // Проверяем флаг кнопки старт-стоп
    if (!isEngineRunning) {       // Если двигатель не запущен
      StartEngine(false);         // То запускаем его
    } else {     // Если двигатель был запущен и нажали кнопку старт-стоп
      StopEngine();               // то глушим двигатель
      delay(500);
    }
    isStartButtonPressed = false; // Сброс флага после обработки
  }
}

void ShedulerAction() {                 // Обработчик команд с сервера MQTT
  if (actionList.length() > 1) {
    if (actionList.endsWith("startenginecom")) {        // Обработка команды для старта/остановки двигателя
      if (actionValue == "1") StartEngine(true);
      else StopEngine();
    } else if (actionList.endsWith("alarmoncom")) {     // Обработка команды для включения/выключения сигнализации
      if (actionValue == "1") CarClose();
      else CarOpen();
    } else if (actionList.endsWith("heatenginecom")) {  // Обработка команды для включения/выключения обогрева двигателя
      EngineHeat(actionValue == "1");
    } else if (actionList.endsWith("startperiodcom")) { // Обработка команды для установки времени работы двигателя
      engineRunDuration = actionValue.toInt() * 60000;
    }
    // Очистка командных переменных и флагов
    actionList = "";
    actionValue = "";
    isStatusCheckRequired = true;
  }
}

void CarOpen() {                        // Открываем автомобиль
  if (isAlarmEnabled) {   // Если до открытия автомобиля, двигатель был запущен на прогрев,
    if (isRemoteEngineStarted) isRemoteEngineStarted = false;   // то оставляем его работающим. Деактивировав таймер.
    digitalWrite(CAR_OP_PIN, HIGH);
    delay(500);
    digitalWrite(CAR_OP_PIN, LOW);
    isAlarmEnabled = false;
  }
}

void CarClose() {                       // Закрываем автомобиль
  if (!isAlarmEnabled) {
    if (analogRead(TACH_PIN) >= ENGINE_RPM_THRESHOLD) {   // Если двигатель работал
      isRemoteEngineStarted = true;                       // Активируем таймеры автозапуска
    }
    digitalWrite(CAR_CL_PIN, HIGH);
    delay(500);
    digitalWrite(CAR_CL_PIN, LOW);
    digitalWrite(BTN_LED_PIN, LOW);
    isAlarmEnabled = true;
  }
}

void EngineHeat(bool on) {                       // Включаем/выключаем подогрев двигателя

  if ((analogRead(TACH_PIN) >= ENGINE_RPM_THRESHOLD) && on) {
    digitalWrite(HEAT_ENG_PIN, HIGH);
    isEngineHeaterActive = true;
  }
  else {
    digitalWrite(HEAT_ENG_PIN, LOW);
    isEngineHeaterActive = false;
  }
}

void AllPinOff() {
  digitalWrite(IGN_PIN, LOW);
  digitalWrite(STARTER_PIN, LOW);
  digitalWrite(ACC_PIN, LOW);
  digitalWrite(CAR_OP_PIN, LOW);
  digitalWrite(CAR_CL_PIN, LOW);
  digitalWrite(BTN_LED_PIN, LOW);
  digitalWrite(HEAT_ENG_PIN, LOW);
}

IRAM_ATTR void myIsr() {
  isStartButtonPressed = true;
}
