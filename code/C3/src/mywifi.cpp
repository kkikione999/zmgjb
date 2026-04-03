#include "mywifi.h"
#include "myserial.h"
#include <Arduino.h>

// WiFi STA configuration
// const char* ssid = "509";
// const char* password = "409409409";

// const char* ssid = "ESP32";
// const char* password = "888888888";

const char* ssid = "000666";
 const char* password = "sss000666";
// WiFi连接状态
bool wifiConnected = false;
unsigned long lastWiFiCheckTime = 0;
const unsigned long WIFI_CHECK_INTERVAL = 5000; // 5秒检查一次WiFi连接

void onWiFiEvent(arduino_event_id_t event)
{
  switch (event) {
    case ARDUINO_EVENT_WIFI_STA_START:
      Serial.println("WiFi STA Started");
      break;
    case ARDUINO_EVENT_WIFI_STA_CONNECTED:
      Serial.println("WiFi STA Connected to AP");
      break;
    case ARDUINO_EVENT_WIFI_STA_GOT_IP:
      Serial.print("WiFi STA Got IP: ");
      Serial.println(WiFi.localIP());
      wifiConnected = true;
      break;
    case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
      Serial.println("WiFi STA Disconnected");
      wifiConnected = false;
      break;
    default:
      break;
  }
}

void WiFi_init()
{
  WiFi.mode(WIFI_STA);
  WiFi.onEvent(onWiFiEvent);
  
  Serial.println();
  Serial.println("******************************************************");
  Serial.print("Connecting to WiFi: ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);
  
  // 等待连接，最多等待10秒
  unsigned long startTime = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - startTime < 10000) {
    delay(500);
    Serial.print(".");
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println();
    Serial.println("WiFi connected successfully!");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
    wifiConnected = true;
  } else {
    Serial.println();
    Serial.println("WiFi connection failed!");
    wifiConnected = false;
  }
}

void WiFi_reconnect()
{
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi disconnected, attempting to reconnect...");
    wifiConnected = false;
    WiFi.disconnect();
    delay(1000);
    WiFi.begin(ssid, password);
    
    // 等待连接，最多等待5秒
    unsigned long startTime = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - startTime < 5000) {
      delay(500);
      Serial.print(".");
    }
    
    if (WiFi.status() == WL_CONNECTED) {
      Serial.println();
      Serial.println("WiFi reconnected successfully!");
      wifiConnected = true;
    } else {
      Serial.println();
      Serial.println("WiFi reconnection failed!");
    }
  }
}

bool isWiFiConnected()
{
  return wifiConnected;
}

void checkWiFiConnection() {
  unsigned long currentTime = millis();
  
  // 定期检查WiFi连接状态
  if (currentTime - lastWiFiCheckTime >= WIFI_CHECK_INTERVAL) {
    lastWiFiCheckTime = currentTime;
    
    if (!isWiFiConnected()) {
      Serial.println("WiFi connection lost, attempting to reconnect...");
      WiFi_reconnect();
    }
  }
}



