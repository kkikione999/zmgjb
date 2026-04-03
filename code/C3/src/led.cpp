#include "led.h"

// WR LED当前状态
static bool wrState = false;
// WL LED当前状态  
static bool wlState = false;

void LEDController::init() {
    // 设置LED引脚为输出模式
    pinMode(LED_WR_PIN, OUTPUT);
    pinMode(LED_WL_PIN, OUTPUT);
    
    // 初始状态：关闭LED（高电平）
    digitalWrite(LED_WR_PIN, HIGH);
    digitalWrite(LED_WL_PIN, HIGH);
    
    wrState = false;
    wlState = false;
}

void LEDController::setWR(bool state) {
    wrState = state;
    // 低电平点亮，所以状态为true时输出低电平
    digitalWrite(LED_WR_PIN, state ? LOW : HIGH);
}

void LEDController::setWL(bool state) {
    wlState = state;
    // 低电平点亮，所以状态为true时输出低电平
    digitalWrite(LED_WL_PIN, state ? LOW : HIGH);
}

void LEDController::toggleWR() {
    setWR(!wrState);
}

void LEDController::toggleWL() {
    setWL(!wlState);
}

bool LEDController::getWR() {
    return wrState;
}

bool LEDController::getWL() {
    return wlState;
}
