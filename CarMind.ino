#include <LittleFS.h>

#include "wifi_manager.h"
#include "mqtt_manager.h"
#include "CarMind.hpp"

#define SerialMon Serial

// Глобальные переменные
WiFiClientSecure secureClient;
MQTTManager mqtt(secureClient);
WiFiManager wifi;
CarMind carmind;

String actionList = "";// Переменная для хранения команды от телефона
String actionValue = "";

int countNetError = 0;// Количество неудачных попыток коннекта (после 3-х рестарт модема и обнуление)
int totalcountNetError = 0; // Количество рестартов модема

// Топики Publish
const char startengine[] = "startengine";
const char alarmon[] = "alarmon";
const char batteryvolt[] = "batteryvolt";
const char heatengine[] ="heatengine";
const char cartemp[] = "cartemp";
const char totalerrorcount[] = "totalerrorcount";
const char startperiod[] = "startperiod";
const char rpminfo[] = "rpminfo";
const char startTimer[] ="starttimer";

// Прототипы
void ShedulerAction();
void MqttThread();
void CheckStatus();
void vTaskComm(void *parameter);
void vTaskCtrl(void *parameter);
void MqttCallback(char *topic, byte *payload, unsigned int len);

void setup()
{
	SerialMon.begin(115200);

	carmind.Init();
	secureClient.setInsecure();
	wifi.begin();
	mqtt.begin(MQTT_BROKER, MQTT_PORT, MQTT_USER, MQTT_PASS, MQTT_CLIENT_ID);
	mqtt.setCallback(MqttCallback);

	xTaskCreatePinnedToCore(vTaskComm, "Task 1", 20000, NULL, 1, NULL, 0);
	delay(500);
	xTaskCreatePinnedToCore(vTaskCtrl, "Task 2", 20000, NULL, 1, NULL, 1);
	delay(500);

	BSP_INIT();
}

void loop()
{
}

// Поток 1 работа с wifi и подключение к mqtt серверу
void vTaskComm(void *parameter)
{
	for (;;) {
		MqttThread();
		vTaskDelay(pdMS_TO_TICKS(500));
	}
}

// Поток 2 Обработка и выполнение команд с сервера, обновление параметров, обработчик кнопки старт-стоп
void vTaskCtrl(void *parameter)
{
	for (;;) {
		carmind.StartStop();
		vTaskDelay(pdMS_TO_TICKS(500));
		ShedulerAction();
		vTaskDelay(pdMS_TO_TICKS(500));
	}
}

void MqttCallback(char *topic, byte *payload, unsigned int len)
{
	String val = "";

	for (unsigned int i = 0; i < len; i++)
		val += (char)payload[i];

	actionList = String(topic);
	actionValue = val;

	SerialMon.print("Message arrived [");
	SerialMon.print(topic);
	SerialMon.print("]: ");
	SerialMon.write(payload, len);
	SerialMon.println();
}

void MqttThread()
{
	if (!mqtt.isConnected()) { // Подключение к серверу MQTT
		countNetError++;
		if (countNetError > 2) { //Отслеживание дисконнектов. После 3-х проверка wi-fi соединения
			countNetError = 0;
			wifi.checkConnection();
		}
		if (millis() - carmind.lastMqttUpdate > 10000L) {
			carmind.lastMqttUpdate = millis();
			if (mqtt.connect()) {
				countNetError = 0;
				carmind.lastMqttUpdate = 0;
			}
		}
		return;
	}
	if (carmind.isStatusCheckRequired)
		CheckStatus();
	mqtt.loop();
}

// Сбор данных о состоянии автомобиля и отправка данных на сервер // Ядро1
void CheckStatus()
{
	carmind.isEngineRunning = analogRead(TACH_PIN) >= ENGINE_RPM_THRESHOLD; // Проверяем, запущен ли двигатель
	mqtt.publish(startengine, carmind.isEngineRunning ? "1" : "0");
	mqtt.publish(alarmon, carmind.isAlarmEnabled ? "1" : "0");
	mqtt.publish(batteryvolt, String(carmind.getVoltage()).c_str());
	mqtt.publish(heatengine, carmind.isEngineHeaterActive ? "1" : "0");
	mqtt.publish(cartemp, String(carmind.getTemperature()).c_str());
	mqtt.publish(totalerrorcount, String(totalcountNetError).c_str());
	mqtt.publish(startperiod, String(carmind.engineRunDuration / 60000).c_str());
	mqtt.publish(rpminfo, String(analogRead(TACH_PIN)).c_str());
	mqtt.publish(startTimer, String(carmind.engineStartCountdown / 60000).c_str());
	carmind.isStatusCheckRequired = false;
}
// Обработчик команд с сервера MQTT
void SchedulerAction()
{
	if (actionList.length() > 1) {
		if (actionList.endsWith("startenginecom"))		// Обработка команды для старта/остановки двигателя
			if (actionValue == "1")
				carmind.StartEngine(true);
			else
				carmind.StopEngine();
		else if (actionList.endsWith("alarmoncom"))		// Обработка команды для включения/выключения сигнализации
			if (actionValue == "1")
				carmind.CloseCar();
			else
				carmind.OpenCar();
		else if (actionList.endsWith("heatenginecom"))	// Обработка команды для включения/выключения обогрева двигателя
			carmind.HeatEngine(actionValue == "1");
		else if (actionList.endsWith("startperiodcom"))	// Обработка команды для установки времени работы двигателя
			carmind.engineRunDuration = actionValue.toInt() * 60000;
		// Очистка командных переменных и флагов
		actionList = "";
		actionValue = "";
		carmind.isStatusCheckRequired = true;
	}
}

void WiFiMonitorThread()
{
	for (;;) {
		if (WiFi.status() != WL_CONNECTED) {
			SerialMon.println("📡 Потеря Wi-Fi. Переподключение...");
			wifi.checkConnection();
		}
		else
			SerialMon.println("📶 Wi-Fi стабилен: " + WiFi.SSID());
		vTaskDelay(pdMS_TO_TICKS(5000));// Проверка каждые 5 секунд
	}
}
