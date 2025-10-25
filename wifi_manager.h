#pragma once
#include <WiFi.h>
#include <WiFiMulti.h>

class WiFiManager {
public:
  WiFiManager();
  void begin();
  void checkConnection();
  bool isConnected();

private:
  WiFiMulti wifiMulti;
  const char* ssids[3] = {"phone", "HONOR X9b 5G", "NEONET-935B"};
  const char* passwords[3] = {"191501541", "191501541", "83c310c2"};
};
