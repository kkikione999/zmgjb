/*
*/
#include "webserver.h"
#include <ArduinoJson.h>

// 在这里定义全局 server / ws（task.h 里是 extern）
AsyncWebServer server(80);       // NEW
AsyncWebSocket ws("/ws");        // NEW

// 从 WebSocket 回调过来的消息，扔进队列，交给 websocket_task 处理
static void onWsEvent(AsyncWebSocket *server, AsyncWebSocketClient *client,AwsEventType type,void *arg,uint8_t *data,size_t len)
{
    if (type == WS_EVT_CONNECT) 
    {
        Serial.printf("[WS] Client #%u connected from %s\n", client->id(), client->remoteIP().toString().c_str());
    } 
    else if (type == WS_EVT_DISCONNECT) 
    {
        Serial.printf("[WS] Client #%u disconnected\n", client->id());
    } 
    else if (type == WS_EVT_DATA) 
    {
        AwsFrameInfo *info = (AwsFrameInfo *)arg;
        if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) {
            // 文本帧，转成 String
            String payload;
            payload.reserve(len + 1);
            for (size_t i = 0; i < len; i++) {
                payload += (char)data[i];
            }
            // 只做轻量级封装，真正解析放到 websocket_task 里
            ws_message_t msg;
            msg.client = client;
            msg.data   = payload;

            if (ws_queue != NULL) {
                BaseType_t ok = xQueueSend(ws_queue, &msg, 0);//构造ws_message_t结构体（包含客户端指针和payload字符串），然后尝试放入ws_queue（FreeRTOS队列）
                if (ok != pdTRUE) {
                    Serial.println("[WS] ws_queue full, drop message");
                }
            } else {
                Serial.println("[WS] ws_queue not initialized");
            }
        } else {
            // 其他情况（分片帧、二进制等）暂不处理
            Serial.println("[WS] Unsupported frame type");
        }
    } 
    else if (type == WS_EVT_ERROR) 
    {
        Serial.printf("[WS] Error on client #%u\n", client->id());
    } 
    else if (type == WS_EVT_PONG) 
    {
        // 可忽略
    }
}

// 初始化 SPIFFS + HTTP + WebSocket
void WebServer_init()
{
    // 1. 挂载 SPIFFS，index.html 放在 /index.html
    if (!SPIFFS.begin(true)) {  // true: 如果挂载失败则格式化
        Serial.println("[FS] SPIFFS Mount Failed");
    } 
    else {
        Serial.println("[FS] SPIFFS Mounted");
    }

    // 2. 配置 HTTP：根路径返回 index.html
    // 方式一：用 serveStatic，将 SPIFFS 根目录映射为网站根目录
    server.serveStatic("/", SPIFFS, "/").setDefaultFile("index.html");

    // 3. 配置 WebSocket
    ws.onEvent(onWsEvent);
    server.addHandler(&ws);

    // 4. 启动服务器
    server.begin();
    Serial.println("[HTTP] Server started at port 80");
}

// 示例状态广播接口（可在任务中调用）
void ws_broadcast_status(uint16_t adc_raw)
{
    // ArduinoJson 7 推荐用 JsonDocument，自动伸缩容量，始终在堆上分配
    JsonDocument doc;

    doc["type"]    = "status";
    doc["adc_raw"] = adc_raw;

    String out;
    serializeJson(doc, out);       // 这两个函数依旧是这样用
    ws.textAll(out);
}

