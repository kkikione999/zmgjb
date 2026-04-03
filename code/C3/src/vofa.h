#ifndef VOFA_H
#define VOFA_H

#include <Arduino.h>
#include <WiFi.h>

// VOFA配置
extern const char* SERVER_IP;
extern const uint16_t SERVER_PORT;

// VOFA状态变量
extern WiFiClient tcpClient;
extern bool tcpConnected;
extern unsigned long lastTCPCheckTime;
extern const unsigned long TCP_CHECK_INTERVAL;

// VOFA函数声明
void VOFA_init();
void VOFA_checkConnection();
void VOFA_sendHandshake();
void VOFA_sendData(int data);
void VOFA_receiveData();
bool VOFA_isConnected();

// VOFA打印函数（类似Serial.print/println）
void VOFA_print(const String &data);
void VOFA_print(const char *data);
void VOFA_print(int data);
void VOFA_print(float data);
void VOFA_print(double data);

void VOFA_println(const String &data);
void VOFA_println(const char *data);
void VOFA_println(int data);
void VOFA_println(float data);
void VOFA_println(double data);
void VOFA_println(); // 只发送换行

bool VOFA_Try_write(const uint8_t* data, size_t len);
#endif // VOFA_H
