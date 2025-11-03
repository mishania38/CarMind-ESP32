/*
 * Copyright (c) 2025, 38tronics. All rights reserved.
 *              See LICENSE for details.
 */

#ifndef _BSP_ESP32_H
#define _BSP_ESP32_H

#include <esp_task_wdt.h>

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

#define BSP_INIT()    esp_task_wdt_deinit()

#endif /* _BSP_ESP32_H */
