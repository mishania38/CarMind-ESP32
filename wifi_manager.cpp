#include "wifi_manager.h"

WiFiManager::WiFiManager() {}

void WiFiManager::begin() {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  delay(100);

  for (int i = 0; i < 3; i++) {
    wifiMulti.addAP(ssids[i], passwords[i]);
  }

  Serial.println("Connecting to WiFi...");
  if (wifiMulti.run() == WL_CONNECTED) {
    Serial.println("WiFi connected");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());
  }
}

void WiFiManager::checkConnection() {
  if (wifiMulti.run() != WL_CONNECTED) {
    Serial.println("WiFi not connected!");
    begin();
  }
}

bool WiFiManager::isConnected() {
  return wifiMulti.run() == WL_CONNECTED;
}
