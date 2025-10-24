/*
  Программа для микроконтроллера ESP32, для удаленного запуска двигателя
  командой через MQTT, и запуск от кнопки Старт-стоп
*/
#include <WiFi.h>
#include <WiFiMulti.h>
#include <PubSubClient.h>
#include <OneWire.h>
#include <HTTPClient.h>
#include <esp_task_wdt.h>

#define SerialMon Serial
//#define MSG_BUFFER_SIZE	(50)


const int WDT_TIMEOUT = 60;
const char* ssid1 = "phone";
const char* password1 = "191501541";
const char* ssid2 = "HONOR X9b 5G";
const char* password2 = "191501541";
const char* ssid3 = "NEONET-935B";
const char* password3 = "83c310c2";


WiFiMulti wifiMulti;
WiFiClient client;

PubSubClient   mqtt(client);
OneWire         ds18b20(15);

TaskHandle_t Task1;
TaskHandle_t Task2;


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

/*------------------------------Переменные хранения статуса-----------------------------*/
bool isAlarmEnabled = false;                   // Статус замков дверей. Закрыт ли автомобиль
bool isEngineHeaterActive = false;             // Статус включенного подогревателя двигателя
bool isRemoteEngineStarted = false;            // Статус удаленного запуска двигателя
bool isEngineRunning = false;                  // Статус заведенного двигателя
volatile bool isStartButtonPressed = false;    // Статус нажатий кнопки старт-стоп, меняется обработчиком прерываний
bool isStatusCheckRequired = false;            // Флаг на отправку параметрии на сервер
bool isFullParameterCheckEnabled = false;      // Флаг на проверку полной параметрии
bool isStartStopButtonBlinking = 0;            // Статус включенной подсветки кнопки старт-стоп

/*------------------------------Переменные с таймерами----------------------------------*/
unsigned long remoteEngineStartTime = 0;        // Время, когда был запущен двигатель
unsigned long engineRunDuration = 900000;       // Интервал, на который двигатель запускается
unsigned long lastMqttUpdate = millis();        // Время последнего обновления соединения
unsigned long lastRemoteStartTimerUpdate = 0;   // Таймер обновления параметрии во время автозапуска
unsigned int engineStartCountdown = 0;          // Обратный отсчет времени работы двигателя
//unsigned int starterRunDuration = 1200;       // Интервал включения стартера
unsigned int blinkBtnPeriod = 1000;             // Интервал мигания подсветки кнопки Старт-стоп
unsigned int lastBtnBlink = 0;                  // Таймер для мигания лампочкой кнопки Старт-стоп

/*------------------------------Переменные для авторизации на MQTT сервере--------------*/
const char* broker = "57cda94e4ac14f5ea9404f6eb28e83fb.s1.eu.hivemq.cloud";            // Адрес сервера MQTT брокера
const char mqtt_user[] = "user_car";     // Имя пользователя
const char mqtt_pass[] = "yIpIQHBWdSN_v1";       // Пароль сервера MQTT брокера
const char mqtt_cid[] = "fiat";            // Уникальное имя устройства в сети MQTT
unsigned int PORT = 8883;                  // Порт MQTT брокера НЕ SSL !





/*-----------------------------Топики-------------------------------------------------*/
const char startenginecom[] =   "/user_17f8efcd/fiat/startenginecom";
const char startengine[] =      "/user_17f8efcd/fiat/startengine";
const char alarmoncom[] =       "/user_17f8efcd/fiat/alarmoncom";
const char alarmon[] =          "/user_17f8efcd/fiat/alarmon";
const char heatenginecom[] =    "/user_17f8efcd/fiat/heatenginecom";
const char heatengine[] =       "/user_17f8efcd/fiat/heatengine";
const char refreshcom[] =       "/user_17f8efcd/fiat/refreshcom";
const char batteryvolt[] =      "/user_17f8efcd/fiat/batteryvolt";
const char startperiodcom[] =   "/user_17f8efcd/fiat/startperiodcom";
const char startperiod[] =      "/user_17f8efcd/fiat/startperiod";
const char starterperiodcom[] = "/user_17f8efcd/fiat/starterperiodcom";
//const char starterperiod[] =    "/user_17f8efcd/fiat/starterperiod";
const char cartemp[] =          "/user_17f8efcd/fiat/cartemp";
const char totalerrorcount[] =  "/user_17f8efcd/fiat/totalerrorcount";
const char rpminfo[] =          "/user_17f8efcd/fiat/rpminfo";
const char rpmcom[] =           "/user_17f8efcd/fiat/rpmcom";
const char startTimer[] =       "/user_17f8efcd/fiat/starttimer";


String actionList = "";                     // Переменная для хранения команды от телефона
String actionValue = "";


int RPM = 2000;                             // Пороговое значение заведенного двигателя (в попугаях)
int countNetError = 0;                      // Количество неудачных попыток коннекта (после 3-х рестарт модема и обнуление)
int totalcountNetError = 0;                 // Количество рестартов модема


bool MqttConnect();
void CheckStatus();

void MqttCallback(char* topic, byte* payload, unsigned int len);
void AllPinOff();
void StartEngine(int timeIgn = 1000);
void StopEngine();
void ConnectWIFI();
void CheckWIFI();
float refreshTemperature();
float refreshVoltage();
void CarOpen();
void CarClose();
void EngineHeat(bool on);


void ShedulerAction();
void StartStopThread();
void MqttThread();

void CheckStatus()                  // Сбор данных о состоянии автомобиля и отправка данных на сервер   // Ядро1
{
  //float _starterperiod = starterRunDuration;

  mqtt.publish(startengine, isEngineRunning ? "1" : "0");
  mqtt.publish(alarmon, isAlarmEnabled ? "1" : "0");
  mqtt.publish(batteryvolt, String(refreshVoltage()).c_str());
  mqtt.publish(heatengine, isEngineHeaterActive ? "1" : "0");
  mqtt.publish(cartemp, String(refreshTemperature()).c_str());
  mqtt.publish(totalerrorcount, (String(totalcountNetError)).c_str());

  mqtt.publish(startperiod, (String(engineRunDuration / 60000)).c_str());
  //mqtt.publish(starterperiod, (String(starterRunDuration / 1000)).c_str());
  mqtt.publish(rpminfo, (String(analogRead(TACH_PIN))).c_str());
  mqtt.publish(startTimer, (String(engineStartCountdown)).c_str());

  isStatusCheckRequired = false;
  esp_task_wdt_reset();  // Сброс watchdog
}

void StartEngine(bool onTimer)      // Запускаем двигатель
{
  SerialMon.println("Запуск");
  if (analogRead(TACH_PIN) <= RPM) {                  //Проверяем заведен ли двигатель

    digitalWrite(IGN_PIN, LOW);
    digitalWrite(IGN_PIN, HIGH);                    //Включаем зажигание,
    
    float temp = refreshTemperature();

    // Прогрев свечей накала в зависимости от температуры в салоне

    if (temp <= 0) {
      delay( 10000 );
      esp_task_wdt_reset();  // Сброс watchdog
    }
    else if (temp > 0 || temp < 15) {
      delay( 1000 );
    }
    else if (temp >= 15) {
      delay( 500 );
    }
      
    digitalWrite(STARTER_PIN, HIGH);                // Включаем стартер, на установленное время
    
    int t = 0;
    while((analogRead(TACH_PIN) <= RPM) && (t <= 3000))
    {
      delay(100);
      t += 100;
    }
    esp_task_wdt_reset();  // Сброс watchdog

    digitalWrite(STARTER_PIN, LOW);                 // Выключаем стартер
    
    delay( 500 );
      
    if (analogRead(TACH_PIN) >= RPM) {              // Проверяем по тахометру, завелся ли двигатель
      
      if (onTimer && isAlarmEnabled) {              // Если стоял флаг таймера и авто под охраной,
        remoteEngineStartTime = millis();           // то сбрасываем счетчик
        lastRemoteStartTimerUpdate = millis();
        isRemoteEngineStarted = true;               // и ставим флаг активного таймера на автозапуск
      }

      else {digitalWrite(BTN_LED_PIN, HIGH);        // Если без охраны, то просто включаем кнопку старт-стоп
      }       
    }

    else {                                          // Если же двигатель не завелся, то:
      digitalWrite(IGN_PIN, LOW);                   // Выключаем зажигание
      digitalWrite(BTN_LED_PIN, LOW);               // Выключаем подсветку кнопки Старт-стоп
      delay(500);
    } 
      
  }
}

void StopEngine()                   // Глушим двигатель
{
  digitalWrite(IGN_PIN, LOW);                    //Выключаем зажигание
  digitalWrite(ACC_PIN, LOW);                    //Выключаем ACC
  digitalWrite(BTN_LED_PIN, LOW);                //Выключаем подсветку кнопки старт-стоп
}

float refreshVoltage() {            // Измерения напряжения на входе DC/DC преобразователя
  float _voltage;
  _voltage = ((analogRead(BAT_V) * 6.08 * 3.30 ) / 4096); // Замеряем напряжение бортсети                         

  //Расчеты напруги
  // 12,86в / 5700 = 0,00225614035087719298245614035088 А
  // 0,00225614035087719298245614035088 * 1000 = 2,2561403508771929824561403508772
  // k = 5.7
  // V = P / 4096 * 6.08 * 3.3

  return _voltage;
}

float refreshTemperature() {        // Измерение температуры в салоне датчиком DS18B20
  ds18b20.reset(); 
    ds18b20.write(0xCC); 
    ds18b20.write(0x44);  // Запуск измерения температуры
    delay(1000);  // Ждем завершения измерений

    ds18b20.reset(); 
    ds18b20.write(0xCC); 
    ds18b20.write(0xBE);  // Чтение данных температуры

    // Прямое считывание младшего и старшего байтов и вычисление температуры
    int16_t rawTemp = (ds18b20.read() | (ds18b20.read() << 8));  // Собираем 16-битное значение
    float temperature = rawTemp * 0.0625;  // Преобразуем в температуру
    
    return temperature;
    //tempCar = String(temperature);  // Преобразуем в строку (если нужно)
}

void ShedulerAction() {             // Обработчик команд с сервера MQTT
  if (actionList.length() > 1) {

    // Обработка команды для старта/остановки двигателя
    if (actionList == "startenginecom") {
      if (actionValue == "1") {
        StartEngine(true);
      } else if (actionValue == "0") {
        StopEngine();
      }
    }

    // Обработка команды для включения/выключения сигнализации
    else if (actionList == "alarmoncom") {
      if (actionValue == "1") {
        CarClose();
      } else if (actionValue == "0") {
        CarOpen();
      }
    }

    // Обработка команды для включения/выключения обогрева двигателя
    else if (actionList == "heatenginecom") {
      if (actionValue == "1") {
        EngineHeat(true);
      } else if (actionValue == "0") {
        EngineHeat(false);
      }
    }

    // Обработка команды на обновление (пока не реализовано)
    else if (actionList == "refreshcom") {
      // Здесь можно добавить логику для обновления состояния
    }

    // Обработка команды для установки времени работы двигателя
    else if (actionList == "startperiodcom") {
      engineRunDuration = actionValue.toInt();
      engineRunDuration *= 60000;  // Преобразование минут в миллисекунды
      SerialMon.println(actionValue);
    }

    // Обработка команды для установки времени работы стартера
    else if (actionList == "starterperiodcom") {
      //float value = actionValue.toFloat();
      //starterRunDuration = value * 1000;  // Преобразование в миллисекунды
      //SerialMon.println(starterRunDuration);
    }

    // Очистка командных переменных и флагов
    actionList = "";
    actionValue = "";
    isStatusCheckRequired = true;
  }
}

void StartStopThread()              // Обработчик нажатий кнопки старт-стоп, таймер прогрева двигателя,
{           
  isEngineRunning = (analogRead(TACH_PIN) >= RPM);  // Проверяем, запущен ли двигатель

  if (isStartButtonPressed) {
    if (!isEngineRunning){           // Проверяем нажата ли кнопка старт-стоп И двигатель не запущен                     
      StartEngine(false);                                        // Только тогда заводим двигатель
    }
    else {     // Если двигатель был запущен и нажали кнопку старт-стоп
      StopEngine();                                           // то глушим двигатель
      delay(500);
      esp_task_wdt_reset();  // Сброс watchdog
    }
    isStartButtonPressed = false;  // Сброс флага после обработки
  }

  delay(500);
}

IRAM_ATTR void myIsr() {
  if (!isStartButtonPressed) {
    isStartButtonPressed = true;
  }
}
  

void setup() {
  pinMode(TACH_PIN,      INPUT_PULLDOWN);
  pinMode(START_BTN,     INPUT_PULLDOWN);
  pinMode(BAT_V,         INPUT);
  
  
  pinMode(IGN_PIN,      OUTPUT);
  pinMode(STARTER_PIN,  OUTPUT);
  pinMode(ACC_PIN,      OUTPUT);
  pinMode(BTN_LED_PIN,  OUTPUT);

  attachInterrupt(START_BTN, myIsr, RISING);

  const esp_task_wdt_config_t twdt_config = {
    .timeout_ms = 60000, // 60-second timeout
    .idle_core_mask = (1 << configNUM_CORES) - 1, // Subscribe all idle tasks
    .trigger_panic = true // Panic and reset on timeout
  };
  
  

  mqtt.setServer(broker, PORT);                  //Настройки брокера MQTT
  mqtt.setCallback(MqttCallback);


  SerialMon.begin(115200);

  AllPinOff();
  
  xTaskCreatePinnedToCore(Task1code, "Task1", 20000, NULL, 1, &Task1, 0);
  delay(500);
  xTaskCreatePinnedToCore(Task2code, "Task2", 20000, NULL, 1, &Task2, 1);
  delay(500);

  esp_task_wdt_init(&twdt_config);  // Инициализируем WDT на 10 секунд
  
}

void loop() {
  esp_task_wdt_reset();  // Сброс watchdog
}

void Task1code(void *parameter)
{
  for (;;)
  {
    esp_task_wdt_add(NULL);        // Добавляем текущую задачу в наблюдатель WDT
    MqttThread();
    delay(1000);
    esp_task_wdt_reset();  // Сброс watchdog
  }
}

void Task2code(void *parameter)
{
  for (;;)
  {
    StartStopThread();
    delay(500);
    ShedulerAction();
    delay(500);
    esp_task_wdt_reset();  // Сброс watchdog
  }
}


void ConnectWIFI()
{
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  delay(100);

  wifiMulti.addAP(ssid1, password1);
  wifiMulti.addAP(ssid2, password2);
  wifiMulti.addAP(ssid3, password3);

  Serial.println("Connecting Wifi...");

  if (wifiMulti.run() == WL_CONNECTED) {
    Serial.println("");
    Serial.println("WiFi connected");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());
  }
  esp_task_wdt_reset();  // Сброс watchdog
}

void CheckWIFI()
{
  if (wifiMulti.run() != WL_CONNECTED)
  {
    Serial.println("WiFi not connected!");
    ConnectWIFI();
  }
}

void MqttThread()
{

  if (countNetError > 2) { //Отслеживание дисконнектов. После 3-х проверка wi-fi соединения
    SerialMon.println("Count errors: " + String(countNetError));
    countNetError = 0;
    CheckWIFI();
  }

  if (!mqtt.connected()) { // Подключение к серверу MQTT
    SerialMon.println("=== MQTT NOT CONNECTED ===");
    countNetError++;
    unsigned long t = millis();
    
    if (t - lastMqttUpdate > 10000L) {
      SerialMon.println("check connect");
      lastMqttUpdate = t;
      if (MqttConnect()) {
        countNetError = 0;
        lastMqttUpdate = 0;
        esp_task_wdt_reset();  // Сброс watchdog
      }
    }
    delay(100);
    return;
  }

  if (isStatusCheckRequired){
    CheckStatus();
  }
  
  mqtt.loop();
}

void MqttCallback(char* topic, byte* payload, unsigned int len)  // Ядро1
{
  String _topic = String(topic);
  String _val = "";
  
  for (unsigned int i = 0; i != len; i++){
    _val += char(*(payload + i));
  }

  actionList = _topic;
  actionValue = _val;
  
  SerialMon.print("Message arrived [");
  SerialMon.print(topic);
  SerialMon.print("]: ");
  SerialMon.write(payload, len);
  SerialMon.println();
  SerialMon.println(_topic + _val);
}

bool MqttConnect()                         // Ядро1     // Авторизация на MQTT сервере и подписка на топики
{
  SerialMon.print("Connecting to ");
  SerialMon.print(broker);

  // Connect to MQTT Broker
  bool status = mqtt.connect(mqtt_cid, mqtt_user, mqtt_pass);

  if (status == false) {
    SerialMon.println(" fail");
    return false;
  }
  SerialMon.println(" OK");
  mqtt.subscribe(startenginecom);
  mqtt.subscribe(alarmoncom);
  mqtt.subscribe(heatenginecom);
  mqtt.subscribe(refreshcom);
  mqtt.subscribe(startperiodcom);
  mqtt.subscribe(starterperiodcom);
  return mqtt.connected();
  
}

void CarOpen()                                   // Открываем автомобиль
{
  if (isAlarmEnabled) {
    if (isRemoteEngineStarted) {                   // Если до открытия автомобиля, двигатель был запущен на прогрев,
      isRemoteEngineStarted = false;               // то оставляем его работающим. Деактивировав таймер.
    }
    digitalWrite(CAR_OP_PIN, HIGH);
    delay( 500 );
    digitalWrite(CAR_OP_PIN, LOW);
    isAlarmEnabled = false;
  }
}

void CarClose()                                  // Закрываем автомобиль
{
  if (isAlarmEnabled == false) {
    if (analogRead(TACH_PIN) >= RPM) {           // Если двигатель работал
      isRemoteEngineStarted = true;                // Активируем таймеры автозапуска
    }
    digitalWrite(CAR_CL_PIN, HIGH);
    delay( 500 );
    digitalWrite(CAR_CL_PIN, LOW);
    digitalWrite(BTN_LED_PIN, LOW);
    isAlarmEnabled = true;
  }
}

void EngineHeat(bool on)                         // Включаем/выключаем подогрев двигателя
{
  if ((analogRead(TACH_PIN) >= RPM) && on) {
    digitalWrite(HEAT_ENG_PIN, HIGH);
    isEngineHeaterActive = true;
  }
  else {
    digitalWrite(HEAT_ENG_PIN, LOW);
    isEngineHeaterActive = false;
  }
}

void AllPinOff()
{
  digitalWrite(IGN_PIN, LOW);
  digitalWrite(STARTER_PIN, LOW);
  digitalWrite(ACC_PIN, LOW);
  digitalWrite(CAR_OP_PIN, LOW);
  digitalWrite(CAR_CL_PIN, LOW);
  digitalWrite(BTN_LED_PIN, LOW);
  digitalWrite(HEAT_ENG_PIN, LOW); 
}