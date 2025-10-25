#pragma once
#include <WiFiClientSecure.h>
#include <PubSubClient.h>

class MQTTManager {
public:
  MQTTManager(WiFiClientSecure& secureClient);
  void begin(const char* broker, int port, const char* user, const char* pass, const char* clientId);
  void loop();
  bool connect();
  void subscribeTopics();
  void setCallback(MQTT_CALLBACK_SIGNATURE);
  bool isConnected();
  bool publish(const char* topic, const char* payload);

private:
  PubSubClient mqtt;
  const char* mqttUser;
  const char* mqttPass;
  const char* mqttClientId;
};
