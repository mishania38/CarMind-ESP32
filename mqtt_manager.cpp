#include "mqtt_manager.h"

MQTTManager::MQTTManager(WiFiClientSecure& secureClient) : mqtt(secureClient) {}

void MQTTManager::begin(const char* broker, int port, const char* user, const char* pass, const char* clientId) {
  mqtt.setServer(broker, port);
  mqttUser = user;
  mqttPass = pass;
  mqttClientId = clientId;
}

bool MQTTManager::connect() {
  Serial.print("Connecting to MQTT broker...");
  bool status = mqtt.connect(mqttClientId, mqttUser, mqttPass);
  Serial.println(status ? "OK" : "Failed");
  if (status) subscribeTopics();
  return status;
}

void MQTTManager::subscribeTopics() {
  mqtt.subscribe("fiat/startenginecom");
  mqtt.subscribe("fiat/alarmoncom");
  mqtt.subscribe("fiat/heatenginecom");
  mqtt.subscribe("fiat/refreshcom");
  mqtt.subscribe("fiat/startperiodcom");
  mqtt.subscribe("fiat/starterperiodcom");
}

void MQTTManager::setCallback(MQTT_CALLBACK_SIGNATURE) {
  mqtt.setCallback(callback);
}

void MQTTManager::loop() {
  mqtt.loop();
}

bool MQTTManager::isConnected() {
  return mqtt.connected();
}

bool MQTTManager::publish(const char* topic, const char* payload) {
  return mqtt.publish(topic, payload);
}
