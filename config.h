#pragma once

// 🛜 Wi-Fi настройки
const char* WIFI_SSIDS[] = {
  "phone",
  "HONOR X9b 5G",
  "NEONET-935B"
};

const char* WIFI_PASSWORDS[] = {
  "191501541",
  "191501541",
  "83c310c2"
};

// 🌐 MQTT настройки
const char* MQTT_BROKER     = "57cda94e4ac14f5ea9404f6eb28e83fb.s1.eu.hivemq.cloud";
const int   MQTT_PORT       = 8883;  // Порт без SSL (если нужен TLS — используйте WiFiClientSecure)

const char* MQTT_USER       = "user_car";
const char* MQTT_PASS       = "yIpIQHBWdSN_v1";
const char* MQTT_CLIENT_ID  = "fiat";

// 📊 MQTT топики (можно вынести в отдельный файл, если потребуется)
const char* TOPIC_STARTENGINECOM     = "/user_17f8efcd/fiat/startenginecom";
const char* TOPIC_ALARMONCOM         = "/user_17f8efcd/fiat/alarmoncom";
const char* TOPIC_HEATENGINECOM      = "/user_17f8efcd/fiat/heatenginecom";
const char* TOPIC_REFRESHCOM         = "/user_17f8efcd/fiat/refreshcom";
const char* TOPIC_STARTPERIODCOM     = "/user_17f8efcd/fiat/startperiodcom";
const char* TOPIC_STARTERPERIODCOM   = "/user_17f8efcd/fiat/starterperiodcom";
