#ifndef MAIN_H
#define MAIN_H

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "task.h"
#include "myserial.h"
#include "mywifi.h"
#include "led.h"


// 函数声明
void sendDataToVOFA();
void handleSerialInput();



#endif // MAIN_H
