/*
 * Copyright (c) 2025, 38tronics. All rights reserved.
 *              See LICENSE for details.
 */
#include "CarMind.hpp"
#include <FunctionalInterrupt.h>

CarMind::CarMind()
{
	isAlarmEnabled = false;
    isEngineHeaterActive = false;
    isRemoteEngineStarted = false;
    isEngineRunning = false;
    isStartButtonPressed = false;
    isStatusCheckRequired = false;

	remoteEngineStartTime = 0;
	engineRunDuration = 900000;
	lastMqttUpdate = 0;
	engineStartCountdown = 0;

	ds18b20 = OneWire(15);
}

void CarMind::Init()
{
	pinMode(TACH_PIN, INPUT_PULLDOWN);
	pinMode(START_BTN, INPUT_PULLDOWN);
	pinMode(BAT_V, INPUT);

	pinMode(IGN_PIN, OUTPUT);
	pinMode(STARTER_PIN, OUTPUT);
	pinMode(ACC_PIN, OUTPUT);
	pinMode(CAR_OP_PIN, OUTPUT);
	pinMode(CAR_CL_PIN, OUTPUT);
	pinMode(BTN_LED_PIN, OUTPUT);
	pinMode(HEAT_ENG_PIN, OUTPUT);

	attachInterrupt(START_BTN, std::bind(&CarMind::StartBtnISR, this), RISING);

	// Resetting All Pins
	digitalWrite(IGN_PIN, LOW);
	digitalWrite(STARTER_PIN, LOW);
	digitalWrite(ACC_PIN, LOW);
	digitalWrite(CAR_OP_PIN, LOW);
	digitalWrite(CAR_CL_PIN, LOW);
	digitalWrite(BTN_LED_PIN, LOW);
	digitalWrite(HEAT_ENG_PIN, LOW);
}

// Включаем/выключаем подогрев двигателя
void CarMind::HeatEngine(bool on)
{
	if ((analogRead(TACH_PIN) >= ENGINE_RPM_THRESHOLD) && on) {
		digitalWrite(HEAT_ENG_PIN, HIGH);
		isEngineHeaterActive = true;
	}
	else {
		digitalWrite(HEAT_ENG_PIN, LOW);
		isEngineHeaterActive = false;
	}
}

// Запускаем двигатель
void CarMind::StartEngine(bool onTimer)
{
	if (analogRead(TACH_PIN) > ENGINE_RPM_THRESHOLD) // Проверяем заведен ли двигатель
		return;
	digitalWrite(IGN_PIN, HIGH);

	// Прогрев свечей накала в зависимости от температуры в салоне
	float temp = refreshTemperature();
	delay(temp <= 0 ? 10000 : temp < 15 ? 500 : 100);

	digitalWrite(STARTER_PIN, HIGH);	// Включаем стартер пока двигатель не запустится
	unsigned long startTime = millis();	// Но не более 3 секунд
	while (
		analogRead(TACH_PIN) <= ENGINE_RPM_THRESHOLD && millis() - startTime < STARTER_TIMEOUT_MS
	){
		delay(100);
	}
	digitalWrite(STARTER_PIN, LOW); 	// Выключаем стартер
	delay(500);

	if (analogRead(TACH_PIN) >= ENGINE_RPM_THRESHOLD) {	// Проверяем по тахометру, завелся ли двигатель
		if (onTimer && isAlarmEnabled) {		// Если стоял флаг таймера и авто под охраной,
			remoteEngineStartTime = millis();	// то сбрасываем счетчик
			lastMqttUpdate = millis();
			isRemoteEngineStarted = true;		// и ставим флаг активного таймера на автозапуск
		}
		else
			digitalWrite(BTN_LED_PIN, HIGH);	// Если без охраны, то просто включаем кнопку старт-стоп
	}
	else {								// Если же двигатель не завелся, то:
		digitalWrite(IGN_PIN, LOW);				// Выключаем зажигание
		digitalWrite(BTN_LED_PIN, LOW);			// Выключаем подсветку кнопки Старт-стоп
		delay(500);
	}
}

// Глушим двигатель
void CarMind::StopEngine()
{
	digitalWrite(IGN_PIN, LOW);
	digitalWrite(ACC_PIN, LOW);
	digitalWrite(BTN_LED_PIN, LOW);
	engineStartCountdown = 0;
}

// Открываем автомобиль
void CarMind::OpenCar()
{
	if (isAlarmEnabled) {				// Если до открытия автомобиля, двигатель был запущен на прогрев,
		if (isRemoteEngineStarted)
			isRemoteEngineStarted = false;		// то оставляем его работающим. Деактивировав таймер.
		digitalWrite(CAR_OP_PIN, HIGH);
		delay(500);
		digitalWrite(CAR_OP_PIN, LOW);
		isAlarmEnabled = false;
	}
}

// Закрываем автомобиль
void CarMind::CloseCar()
{
	if (!isAlarmEnabled) {
		if (analogRead(TACH_PIN) >= ENGINE_RPM_THRESHOLD) // Если двигатель работал
			isRemoteEngineStarted = true; 		// Активируем таймеры автозапуска
		digitalWrite(CAR_CL_PIN, HIGH);
		delay(500);
		digitalWrite(CAR_CL_PIN, LOW);
		digitalWrite(BTN_LED_PIN, LOW);
		isAlarmEnabled = true;
	}
}

// Обработчик нажатий кнопки старт-стоп, таймер прогрева двигателя
void CarMind::StartStop()
{
	// Обработчик нажатий кнопки старт-стоп
	if (isStartButtonPressed) { // Проверяем флаг кнопки старт-стоп
		isEngineRunning = (analogRead(TACH_PIN) >= ENGINE_RPM_THRESHOLD);// Проверяем, запущен ли двигатель
		if (!isEngineRunning) // Если двигатель не запущен
			StartEngine(false); // То запускаем его
		else { // Если двигатель был запущен и нажали кнопку старт-стоп
			StopEngine(); // то глушим двигатель
			delay(500);
		}
		isStartButtonPressed = false; // Сброс флага после обработки
	}
	// Таймер прогрева двигателя при удаленном запуске
	if (isRemoteEngineStarted) {
		isEngineRunning = (analogRead(TACH_PIN) >= ENGINE_RPM_THRESHOLD);// Проверяем, запущен ли двигатель
		engineStartCountdown = (engineRunDuration - (millis() - remoteEngineStartTime)) ; // Обновляем таймер обратного отсчета
		if (millis() - remoteEngineStartTime <= engineRunDuration) {// Проверяем не вышло ли время на продолжительность дистанционного запуска
			if (millis() - lastMqttUpdate >= 30000) {
				isStatusCheckRequired = true; // Ставим флаг на обновление параметрии
				lastMqttUpdate = millis();// Сбрасываем счетчик обновления параметрии
			}
		}
		else {// Если время продолжительности вышло,
			StopEngine(); // тогда глушим двигатель и сбрасываем флаги
			lastMqttUpdate = 0;
			remoteEngineStartTime = 0;
			engineStartCountdown = 0;
			isRemoteEngineStarted = false;
			isStatusCheckRequired = true; // Ставим флаг на обновление параметрии
		}
	}
}

// Замер напряжения на входе через делитель напряжения
float CarMind::getVoltage()
{
	return ((analogRead(BAT_V) * 6.08 * 3.30 ) / 4096);
}

// Замер температуры в салоне
float CarMind::getTemperature()
{
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

void IRAM_ATTR CarMind::StartBtnPressed()
{
	isStartButtonPressed = true;
}
