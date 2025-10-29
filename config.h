#pragma once

// 🛜 Wi-Fi настройки
const char* WIFI_SSIDS[] = {
  "ssid1",
  "ssid2",
  "ssid3"
};

const char* WIFI_PASSWORDS[] = {
  "pass1",
  "pass2",
  "pass3"
};

// 🌐 MQTT настройки
const char* MQTT_BROKER     = "***********.s1.eu.hivemq.cloud";
const int   MQTT_PORT       = 8883;

const char* MQTT_USER       = "user";
const char* MQTT_PASS       = "password";
const char* MQTT_CLIENT_ID  = "client_name";

// 📊 MQTT топики (можно вынести в отдельный файл, если потребуется)
const char* TOPIC_STARTENGINECOM     = "startenginecom";
const char* TOPIC_ALARMONCOM         = "alarmoncom";
const char* TOPIC_HEATENGINECOM      = "heatenginecom";
const char* TOPIC_REFRESHCOM         = "refreshcom";
const char* TOPIC_STARTPERIODCOM     = "startperiodcom";
const char* TOPIC_STARTERPERIODCOM   = "starterperiodcom";
