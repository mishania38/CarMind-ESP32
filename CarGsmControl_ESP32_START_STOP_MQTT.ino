#include <OneWire.h>

#define SerialMon Serial


OneWire         ds18b20(15);

//TaskHandle_t Task1;
//TaskHandle_t Task2;


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
bool alarmOn = false;                           // Статус замков дверей. Закрыт ли автомобиль
bool engineHeatOn = false;                      // Статус включенного подогревателя двигателя
bool remoteEngineStartOn = false;               // Статус удаленного запуска двигателя
bool hasEngineStarted = false;                  // Статус заведенного двигателя
volatile bool hasStartButtonClicked = false;    // Статус нажатий кнопки старт-стоп, меняется обработчиком прерываний
bool doCheckStatus = false;                     // Флаг на отправку параметрии на сервер
bool fullDetection = false;                     // Флаг на проверку полной параметрии
bool blinkBtnstate = 0;                         // Статус включенной подсветки кнопки старт-стоп

/*------------------------------Переменные с таймерами----------------------------------*/
unsigned long whenRemoteEngineStartOn = 0;      // Время, когда был запущен двигатель
unsigned long engineWorkPeriod = 900000;        // Интервал, на который двигатель запускается
unsigned long lastMqttUpdate = millis();        // Время последнего обновления соединения
unsigned long lastRemoteStartTimerUpdate = 0;   // Таймер обновления параметрии во время автозапуска
unsigned int _startTimer = 0;                   // Обратный отсчет времени работы двигателя
unsigned int starterPeriod = 1200;              // Интервал включения стартера
unsigned int blinkBtnPeriod = 1000;             // Интервал мигания подсветки кнопки Старт-стоп
unsigned int lastBtnBlink = 0;                  // Таймер для мигания лампочкой кнопки Старт-стоп

int RPM = 1800;                              // Пороговое значение заведенного двигателя (в попугаях)
int countNetError = 0;                      // Количество неудачных попыток коннекта (после 3-х рестарт модема и обнуление)
int totalcountNetError = 0;                 // Количество рестартов модема




void AllPinOff();
void StartEngine(int timeIgn = 1000);
void StopEngine();
void CarOpen();
void CarClose();
float refreshTemperature();
float refreshVoltage();
void EngineHeat(bool on);


void StartStopThread();




void StartEngine(bool onTimer)       // Запускаем двигатель
{
  SerialMon.println("Запуск");
  if (analogRead(TACH_PIN) <= RPM) {                  //Проверяем заведен ли двигатель

    digitalWrite(IGN_PIN, LOW);
    digitalWrite(IGN_PIN, HIGH);                    //Включаем зажигание,
    
    float temp = refreshTemperature();

    // Прогрев свечей накала в зависимости от температуры в салоне

    if (temp <= 0) {
      delay( 10000 );
    }
    else if (temp > 0 || temp < 15) {
      delay( 1000 );
    }
    else if (temp >= 15) {
      delay( 500 );
    }
      
    digitalWrite(STARTER_PIN, HIGH);                //Включаем стартер, на установленное время
    digitalWrite(BTN_LED_PIN, HIGH); 
    
    int t = 0;
    while((analogRead(TACH_PIN) <= RPM) && (t <= 5000))
    {
      delay(100);
      t += 100;
    }
    digitalWrite(STARTER_PIN, LOW);                 // Выключаем стартер
    
    delay( 500 );
      
    if (analogRead(TACH_PIN) >= RPM) {              //Проверяем по тахометру, завелся ли двигатель
      
      
      if (onTimer && alarmOn) {                     //Если стоял флаг таймера и авто под охраной,
        whenRemoteEngineStartOn = millis();         //то сбрасываем счетчик
        lastRemoteStartTimerUpdate = millis();
        remoteEngineStartOn = true;                 //и ставим флаг активного таймера на автозапуск
      }

      else {digitalWrite(BTN_LED_PIN, HIGH);        //Если без охраны, то просто включаем кнопку старт-стоп
      }       
    }

    else {                                          //Если же двигатель не завелся, то:
      digitalWrite(IGN_PIN, LOW);
      digitalWrite(BTN_LED_PIN, LOW); 
      delay(500);
    } 
      
  }
}

void StopEngine()                                // Глушим двигатель
{
  digitalWrite(IGN_PIN, LOW);                    //Выключаем зажигание
  digitalWrite(ACC_PIN, LOW);                    //Выключаем ACC
  digitalWrite(BTN_LED_PIN, LOW);                //Выключаем подсветку кнопки старт-стоп
}

float refreshVoltage() {
  //float _voltage;
  //_voltage = ((analogRead(BAT_V) * 6.08 * 3.30 ) / 4096); // Замеряем напряжение бортсети                         

  //Расчеты напруги
  // 12,86в / 5700 = 0,00225614035087719298245614035088 А
  // 0,00225614035087719298245614035088 * 1000 = 2,2561403508771929824561403508772
  // k = 5.7
  // V = P / 4096 * 6.08 * 3.3

  return ((analogRead(BAT_V) * 6.08 * 3.30 ) / 4096);
}


float refreshTemperature() {
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

void StartStopThread()                      // Задача-поток отслеживания нажатий кнопки старт-стоп, таймер прогрева двигателя,
{           
  hasEngineStarted = (analogRead(TACH_PIN) >= RPM)      ? true : false;  // Проверяем, запущен ли двигатель

  if (hasStartButtonClicked) {
    if (!hasEngineStarted){           // Проверяем нажата ли кнопка старт-стоп И двигатель не запущен                     
      StartEngine(false);                                        // Только тогда заводим двигатель
      hasStartButtonClicked = false;
    }
    else {     // Если двигатель был запущен и нажали кнопку старт-стоп
      StopEngine();                                           // то глушим двигатель
      delay(500);
      hasStartButtonClicked = false;
    }
  }

  delay(500);
}

IRAM_ATTR void myIsr() {
  if (!hasStartButtonClicked) {
    hasStartButtonClicked = true;
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



  SerialMon.begin(115200);

  AllPinOff();
  /*
  xTaskCreatePinnedToCore(Task1code, "Task1", 20000, NULL, 1, &Task1, 0);
  delay(500);
  xTaskCreatePinnedToCore(Task2code, "Task2", 20000, NULL, 1, &Task2, 1);
  delay(500);
  */
}

void loop() {
  StartStopThread();
  delay(1000);
}
/*
void Task1code(void *parameter)
{
  for (;;)
  {

  }
}

void Task2code(void *parameter)
{
  for (;;)
  {
    StartStopThread();
    delay(500);
  }
}
*/

void CarOpen()                                   // Открываем автомобиль
{
  if (alarmOn) {
    if (remoteEngineStartOn) {                   // Если до открытия автомобиля, двигатель был запущен на прогрев,
      remoteEngineStartOn = false;               // то оставляем его работающим. Деактивировав таймер.
    }
    digitalWrite(CAR_OP_PIN, HIGH);
    delay( 500 );
    digitalWrite(CAR_OP_PIN, LOW);
    alarmOn = false;
  }
}

void CarClose()                                  // Закрываем автомобиль
{
  if (alarmOn == false) {
    if (analogRead(TACH_PIN) >= RPM) {           // Если двигатель работал
      remoteEngineStartOn = true;                // Активируем таймеры автозапуска
    }
    digitalWrite(CAR_CL_PIN, HIGH);
    delay( 500 );
    digitalWrite(CAR_CL_PIN, LOW);
    digitalWrite(BTN_LED_PIN, LOW);
    alarmOn = true;
  }
}

void EngineHeat(bool on)                         // Включаем/выключаем подогрев двигателя
{
  if ((analogRead(TACH_PIN) >= RPM) && on) {
    digitalWrite(HEAT_ENG_PIN, HIGH);
    engineHeatOn = true;
  }
  else {
    digitalWrite(HEAT_ENG_PIN, LOW);
    engineHeatOn = false;
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