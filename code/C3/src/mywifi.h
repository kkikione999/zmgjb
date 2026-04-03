#ifndef MYWIFI_H
#define MYWIFI_H

#include "main.h"
#include <WiFi.h>

// WiFi配置
extern const char* ssid;
extern const char* password;

// WiFi状态变量
extern bool wifiConnected;
extern unsigned long lastWiFiCheckTime;
extern const unsigned long WIFI_CHECK_INTERVAL;

// 函数声明
void WiFi_init();
void WiFi_reconnect();
bool isWiFiConnected();
void checkWiFiConnection();

#endif // MYWIFI_H
