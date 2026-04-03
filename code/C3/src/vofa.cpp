#include "vofa.h"
#include "myserial.h"
#include <Arduino.h>

// VOFA配置定义
// 数据接口选择TCP服务端
const char* SERVER_IP = "172.20.10.4";  // 请根据实际电脑的IP地址修改
const uint16_t SERVER_PORT = 777;       // VOFA里面的监听端口

// VOFA状态变量定义
WiFiClient tcpClient;
bool tcpConnected = false;
unsigned long lastTCPCheckTime = 0;
const unsigned long TCP_CHECK_INTERVAL = 2000; // 2秒检查一次TCP连接

void VOFA_init() {
  Serial.println();
  Serial.println("Attempting to connect to VOFA server...");
  Serial.print("Server IP: ");
  Serial.println(SERVER_IP);
  Serial.print("Server Port: ");
  Serial.println(SERVER_PORT);
  
  if (tcpClient.connect(SERVER_IP, SERVER_PORT)) {
    Serial.println("VOFA connected successfully!");
    tcpConnected = true;
    
    // 发送握手数据
    VOFA_sendHandshake();
  } else {
    Serial.println("VOFA connection failed!");
    tcpConnected = false;
  }
}

void VOFA_checkConnection() {
  unsigned long currentTime = millis();
  
  // 定期检查TCP连接状态
  if (currentTime - lastTCPCheckTime >= TCP_CHECK_INTERVAL) {
    lastTCPCheckTime = currentTime;
    
    if (!tcpClient.connected()) {
      Serial.println("VOFA connection lost, attempting to reconnect...");
      tcpConnected = false;
      
      // 尝试重新连接TCP服务器
      if (tcpClient.connect(SERVER_IP, SERVER_PORT)) {
        Serial.println("VOFA reconnected successfully!");
        tcpConnected = true;
        
        // 重新发送握手数据
        VOFA_sendHandshake();
      } else {
        Serial.println("VOFA reconnection failed!");
      }
    }
  }
}

void VOFA_sendHandshake() {
  if (tcpConnected) {
    tcpClient.print("plot0\n");
    Serial.println("Sent handshake data: plot0");
  }
}

void VOFA_sendData(int data) {
  if (tcpConnected) {
    tcpClient.print(data);
    tcpClient.print("\n");  // VOFA需要换行符来分隔数据
    Serial.print("Sent data to VOFA: ");
    Serial.println(data);
  }
}

void VOFA_receiveData() {
  if (tcpConnected && tcpClient.available()) {
    String receivedData = tcpClient.readString();
    Serial.print("Received from VOFA: ");
    Serial.println(receivedData);
  }
}

bool VOFA_isConnected() {
  return tcpConnected;
}

// VOFA打印函数实现
void VOFA_print(const String &data) {
  if (tcpConnected) {
    tcpClient.print(data);
  }
}

void VOFA_print(const char *data) {
  if (tcpConnected) {
    tcpClient.print(data);
  }
}

void VOFA_print(int data) {
  if (tcpConnected) {
    tcpClient.print(data);
  }
}

void VOFA_print(float data) {
  if (tcpConnected) {
    tcpClient.print(data);
  }
}

void VOFA_print(double data) {
  if (tcpConnected) {
    tcpClient.print(data);
  }
}

void VOFA_println(const String &data) {
  if (tcpConnected) {
    tcpClient.print(data);
    tcpClient.print("\n");
  }
}

void VOFA_println(const char *data) {
  if (tcpConnected) {
    tcpClient.print(data);
    tcpClient.print("\n");
  }
}

void VOFA_println(int data) {
  if (tcpConnected) {
    tcpClient.print(data);
    tcpClient.print("\n");
  }
}

void VOFA_println(float data) {
  if (tcpConnected) {
    tcpClient.print(data);
    tcpClient.print("\n");
  }
}

void VOFA_println(double data) {
  if (tcpConnected) {
    tcpClient.print(data);
    tcpClient.print("\n");
  }
}

void VOFA_println() {
  if (tcpConnected) {
    tcpClient.print("\n");
  }
}

bool VOFA_Try_write(const uint8_t* data, size_t len) {
    size_t sent = 0;
    while (sent < len) {
        size_t n = tcpClient.write(data + sent, len - sent);
        if (n == 0) return false; // 这次写不进去，调用者决定下次再来补
        sent += n;
    }
    return true;
}

