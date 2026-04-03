#ifndef LED_H
#define LED_H

#include <Arduino.h>

// LED引脚定义
#define LED_WR_PIN 21  // WR LED控制引脚
#define LED_WL_PIN 3   // WL LED控制引脚

class LEDController {
public:
    // 初始化LED控制
    static void init();
    
    // 控制WR LED
    static void setWR(bool state);
    
    // 控制WL LED  
    static void setWL(bool state);
    
    // 切换WR LED状态
    static void toggleWR();
    
    // 切换WL LED状态
    static void toggleWL();
    
    // 获取WR LED当前状态
    static bool getWR();
    
    // 获取WL LED当前状态
    static bool getWL();
};

#endif // LED_H
