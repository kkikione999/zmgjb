#include <Arduino.h>
#include <WiFi.h>
#include "main.h"
#include "mywifi.h"
#include "vofa.h"
#include "myserial.h"
#include "led.h"
#include "webserver.h"


// 全局变量定义
unsigned long lastSendTime = 0;
int sendCounter = 0;
const unsigned long SEND_INTERVAL = 1000;      // 1秒发送一次数据


void sendDataToVOFA() {
  unsigned long currentTime = millis();
  
  // 每秒发送一次数据
  if (VOFA_isConnected() && currentTime - lastSendTime >= SEND_INTERVAL) {
    lastSendTime = currentTime;
    
    // 使用VOFA_println发送递增的整数数据
    VOFA_println(sendCounter);
    Serial.print("Sent data to VOFA: ");
    Serial.println(sendCounter);
    
    sendCounter++;
  }
}

void handleSerialInput() {
  if (Serial_available() && VOFA_isConnected()) {
    String inputData = Serial_readString();
    inputData.trim(); // 去除首尾空白字符
    
    if (inputData.length() > 0) {
      // 使用VOFA_println转发串口输入到VOFA
      VOFA_println(inputData);
      Serial.print("Forwarded to VOFA: ");
      Serial.println(inputData);
    }
  }
}

void setup() 
{
  Serial_init();

  Serial1_init();

  // 初始化WIFI
  WiFi_init();
  
  // 等待WiFi连接
  delay(2000);
  
  Serial.println("Serial is ok");

  // 初始化VOFA连接
  VOFA_init();

  // 启动 HTTP + WebSocket
  WebServer_init();      

  LEDController::init();

  // 任务初始化
  Task_init();
  

  //FreeRTOS
  if(xTaskGetSchedulerState() == taskSCHEDULER_NOT_STARTED) {
      vTaskStartScheduler();
  }

  while(xTaskGetSchedulerState() != taskSCHEDULER_RUNNING) {
        Serial.println("Scheduler Wrong!");
      delay(1000);
  }
}

void loop() 
{
  
}
