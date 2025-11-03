/*
 * Copyright (c) 2025, 38tronics. All rights reserved.
 *              See LICENSE for details.
 */

#pragma once
#include <OneWire.h>
#include "config.h"
/* Note:
	the idea is to define interfaces to remove implementation dependencies
	and to make it easy to supply different implementations
*/
/* Consider:
	- PinCtrl structure to let adding more boards and smooth transfer between frameworks
	- define interfaces for data collection and network communications
*/

#define ENGINE_RPM_THRESHOLD	2000
#define STARTER_TIMEOUT_MS		3000

class CarMind {
	/* ----- Переменные хранения статуса ----- */
	bool isAlarmEnabled;				// Флаг замков дверей. Закрыт ли автомобиль
	bool isEngineHeaterActive;			// Флаг включенного подогревателя двигателя
	bool isRemoteEngineStarted;			// Флаг удаленного запуска двигателя
	bool isEngineRunning;				// Флаг заведенного двигателя
	volatile bool isStartButtonPressed;	// Флаг нажатий кнопки старт-стоп, меняется обработчиком прерываний
	bool isStatusCheckRequired;			// Флаг на отправку параметрии на сервер
	/* ----- Переменные с таймерами ----- */
	unsigned long remoteEngineStartTime;// Время, когда был запущен двигатель
	unsigned long engineRunDuration;	// Интервал, на который двигатель запускается
	unsigned long lastMqttUpdate;		// Время последнего обновления соединения
	unsigned int engineStartCountdown;	// Обратный отсчет времени работы двигател

	OneWire ds18b20;
public:
	CarMind();
	void Init();
	void HeatEngine(bool on);
	void StartEngine(bool onTimer);
	void StopEngine();
	void OpenCar();
	void CloseCar();
	void StartStop();
/* Define interfaces and remove friends */
	friend void CheckStatus();
	friend void SchedulerAction();
	friend void MqttThread();
private:
	float getVoltage();
	float getTemperature();
	void IRAM_ATTR StartBtnPressed();
};
