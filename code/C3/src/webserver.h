// webserver.h // NEW
#ifndef WEBSERVER_H__
#define WEBSERVER_H__

#include <Arduino.h>
#include <SPIFFS.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include "task.h"      // 里面有 ws_message_t、ws_queue 的声明

// 全局 WebServer / WebSocket 对象
extern AsyncWebServer server;   // NEW
extern AsyncWebSocket ws;       // 与 task.h 中的 extern 保持一致

// 初始化 HTTP + WebSocket 服务
void WebServer_init();          // NEW

// 可选：状态广播接口，后面可以在任务里调用
void ws_broadcast_status(uint16_t adc_raw);

#endif // WEBSERVER_H__
