#include "task.h"
#include <ArduinoJson.h>   // 用来解析 JSON
#include "webserver.h"     // 使用 ws / 状态广播
// 全局WebSocket消息队列
QueueHandle_t ws_queue;

// STM32数据队列
QueueHandle_t tcp_queue;

// 摇杆数据队列
QueueHandle_t RCdata_queue;

// ADC 数据队列
QueueHandle_t adc_queue;

// PID参数队列
QueueHandle_t pid_queue;

//ARM与DISARM信号量？

// 任务堆栈大小
#define TASK_STACK_SIZE 4096

// 是否启用 UDP 假数据测试
#define USE_UDP_FAKE_RC   0    // NEW: 默认为 0，关闭原来的 rcdata.Ry++ 假数据

// 全局变量定义
extern double_buffer_t g_double_buffer[2];//双缓冲区？
extern SemaphoreHandle_t g_buffer_semaphore;//缓冲区信号量?
extern QueueHandle_t g_quat_queue;
extern parser_context_t g_parser_ctx;

// 当前写缓冲索引
volatile int g_current_write_buffer = 0;

// 任务函数声明
void serial0_task(void *pvParameters);
void serial1_task(void *pvParameters);
void websocket_task(void *pvParameters);
void pwr_task(void *pvParameters);
void vofa_task(void *pvParameters);
void stm32_uart_rx_task(void *pvParameters);
void stm32_uart_tx_task(void *pvParameters);
void stm32_parse_task(void *pvParameters);
void udp_rx_task(void *pvParameters);

void Task_init()
{
    // 初始化双缓冲
    memset(g_double_buffer, 0, sizeof(g_double_buffer));
    for (int i = 0; i < 2; i++) {
        g_double_buffer[i].write_pos = 0;
        g_double_buffer[i].ready_for_parse = false;
    }

    // 创建缓冲信号量
    g_buffer_semaphore = xSemaphoreCreateBinary();
    if (g_buffer_semaphore == NULL) 
    {
        Serial.println("缓冲信号量创建失败!");
    } 
    else {
        Serial.println("缓冲信号量创建成功");
    }

    // 创建 WebSocket 消息队列（用于 onWsEvent -> websocket_task）
    ws_queue = xQueueCreate(10, sizeof(ws_message_t));  // 10 条缓存
    if(ws_queue == NULL) {
        Serial.println("WebSocket队列创建失败!");
    } else {
        Serial.println("WebSocket队列创建成功, 容量10");
    }

    // 创建TCP消息队列(增大队列容量)
    tcp_queue = xQueueCreate(32, sizeof(tx_msg_t));
    if(tcp_queue == NULL) {
        Serial.println("Tcp队列创建失败!");
    } else {
        Serial.println("Tcp队列创建成功, 容量32");
    }

    // 创建遥感数据消息队列(增大队列容量)
    RCdata_queue = xQueueCreate(32, sizeof(RCData_t));
    if(RCdata_queue == NULL) {
        Serial.println("摇杆队列创建失败!");
    } else {
        Serial.println("摇杆队列创建成功, 容量32");
    }

    // 创建电压数据消息队列(增大队列容量)
    adc_queue = xQueueCreate(2, sizeof(uint16_t));
    if(adc_queue == NULL) {
        Serial.println("电压队列创建失败!");
    } else {
        Serial.println("电压队列创建成功, 容量2");
    }

    // 创建PID参数消息队列
    pid_queue = xQueueCreate(10, sizeof(pid_update_msg_t));
    if(pid_queue == NULL) {
        Serial.println("PID队列创建失败!");
    } else {
        Serial.println("PID队列创建成功, 容量10");
    }

    // 创建STM32串口接收任务
    xTaskCreate(
        stm32_uart_rx_task,
        "Stm32_Uart_rx_Task",
        TASK_STACK_SIZE,
        NULL,
        3,  // 较高优先级
        NULL);

    // 创建STM32串口发送任务
    xTaskCreate(
        stm32_uart_tx_task,
        "Stm32_Uart_tx_Task",
        TASK_STACK_SIZE,
        NULL,
        3,  // 较高优先级
        NULL);

    // 创建STM32包解析任务
    xTaskCreate(
        stm32_parse_task,
        "Stm32_Parse_Task",
        TASK_STACK_SIZE,
        NULL,
        2,  // 中等优先级
        NULL);

    // // 创建串口0监听任务(电脑->STM32)
    // xTaskCreate(
    //     serial0_task,
    //     "Serial0_Task",
    //     TASK_STACK_SIZE,
    //     NULL,
    //     1,
    //     NULL);

    // 创建上传电量处理任务(提高优先级)
    xTaskCreate(
        pwr_task,
        "pwr_Task",
        TASK_STACK_SIZE,
        NULL,
        2,  // 提高优先级
        NULL);

    // 创建WebSocket数据处理任务(提高优先级)
    xTaskCreate(
        websocket_task,
        "WebSocket_Task",
        TASK_STACK_SIZE,
        NULL,
        3,  // 提高优先级
        NULL);

    // 创建VOFA+数据发送任务
    xTaskCreate(
        vofa_task,
        "VOFA_Task",
        TASK_STACK_SIZE,
        NULL,
        2,  // 优先级与电源任务相同
        NULL);

    // 创建UDP数据接收任务
    xTaskCreate(
        udp_rx_task,
        "UDP_rx_Task",
        TASK_STACK_SIZE,
        NULL,
        2,  // 优先级与电源任务相同
        NULL);

    // 创建 WebSocket 解析任务
    xTaskCreate(
        websocket_task,         
        "websocket_task",
        TASK_STACK_SIZE,
        NULL,
        2,                       // 建议比普通任务略高一点
        NULL);

}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////            任务函数                 ///////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void serial0_task(void *pvParameters)
{
    uint8_t buffer[SERIAL0_FRAME_SIZE];
    size_t index = 0;

    while (1)
    {
        if (Serial.available())
        {
            buffer[index++] = Serial.read();

            if (index == SERIAL0_FRAME_SIZE)
            {
                if (check_frame(buffer, SERIAL0_FRAME_SIZE))
                {
                    forward_to_serial1(buffer, SERIAL0_FRAME_SIZE);
                }
                index = 0;
            }
        }
        vTaskDelay(1 / portTICK_PERIOD_MS);
    }
}

// 电源监控任务
void pwr_task(void *pvParameters)
{
    Serial.println("电源监控任务启动");

    // 初始化ADC
    analogReadResolution(12);  // 0~4095
    analogSetPinAttenuation(4, ADC_11db);
    
    while (1) {
        // 读取IO4引脚的ADC值
        uint16_t mv = analogReadMilliVolts(4)*2;
        // mv则为读取到的电压
        // VOFA_print("v is ");
        // VOFA_print(mv);
        // VOFA_println("mv");
        if (adc_queue != NULL && xQueueSend(adc_queue, &mv, 0) == pdTRUE) {
            Serial.println("voltage is success!");
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// TCP打印注意事项
// 1. 电脑和ESP32是否在同一个WIFI下
// 2. SERVER_IP和电脑的IP是否相同
// 3. 电脑的监听端口是否和ESP32的SERVER_PORT的端口相同（均为777）
// 4. 电脑的数据接口应该是 TCP服务端模式
// 5. 电脑连接的client(ESP32)有没有消息发过来：有消息发过来的话左上角的连接符号会闪烁，
//    第一次连接ESP32为client0，第二次连接ESP32为client1
// 6. 发现VOFA左上角上闪烁但是数据不出来：ESP32没有发送换行符，VOFA将等到接收缓冲区满再刷新显示数据
/*
该任务的核心任务：不断从 tcp_queue 中取出解析后的 STM32 数据，并通过 TCP 发送给 VOFA 上位机，同时执行相应的数据解析与显示逻辑。
*/
void vofa_task(void *pvParameters)
{
    Serial.println("VOFA无线传输任务启动");
    // 检查WiFi连接
    checkWiFiConnection();
    // 检查VOFA连接
    VOFA_checkConnection();
    
    int times = 0;
    tx_msg_t msg;
    float * quat = nullptr;
    while (1) 
    {
        if (xQueueReceive(tcp_queue, &msg, 5))
        {
            TCPSEND(&msg);
        }
        vTaskDelay(1 / portTICK_PERIOD_MS);
    }
}

// 串口1发送任务 (ESP32->stm32)
void stm32_uart_tx_task(void *pvParameters)
{
    Serial.println("STM32串口发送任务启动");
    RCData_t rc_data;
    pid_update_msg_t pid_msg;
    uint8_t frame[1+1+1+sizeof(RCData_t)+1+1]; // HEADER+CMD+LEN+DATA+SUM+TAIL
    uint8_t pid_frame[1+1+1+sizeof(AxisPID)+1+1]; // 用于PID参数
    uint8_t quat_frame[10]; // 用于0x01四元数命令，实际长度6字节
    uint8_t pause_frame[10]; // 用于0x00暂停命令，实际长度6字节
    // 预定义的PID参数（用于调试）
    static const AxisPID debug_pid[] =
    {
        // 俯仰轴
        { .kp_angle = 2.0f, .ki_angle = 0.1f, .kd_angle = 0.05f,
          .kp_rate = 1.0f, .ki_rate = 0.05f, .kd_rate = 0.01f },
        // 横滚轴
        { .kp_angle = 2.2f, .ki_angle = 0.12f, .kd_angle = 0.06f,
          .kp_rate = 1.1f, .ki_rate = 0.06f, .kd_rate = 0.012f },
        // 偏航轴
        { .kp_angle = 1.8f, .ki_angle = 0.08f, .kd_angle = 0.04f,
          .kp_rate = 0.9f, .ki_rate = 0.04f, .kd_rate = 0.008f }
    };
    static const uint8_t axis_cmd[] = {0x03, 0x04, 0x05}; // 俯仰, 横滚, 偏航
    static int pid_send_index = 0;                        //

    // 定时发送控制变量
    static TickType_t last_quat_send = 0;
    static TickType_t last_pause_send = 0;
    static TickType_t last_debug_pid_send = 0;
    const TickType_t quat_interval = pdMS_TO_TICKS(3000);   // 3秒发送一次0x01
    const TickType_t pause_interval = pdMS_TO_TICKS(3000);  // 5秒发送一次0x00
    const TickType_t debug_pid_interval = pdMS_TO_TICKS(2000); // 2秒发送一次调试PID

    // 添加静态变量用于循环发送手柄数据
     RCData_t last_rc_data = {1.0f, 1.0f, 0.0f, 0.0f}; // 默认中位值
    static TickType_t last_rc_loop_send = 0;
    const TickType_t rc_loop_interval = pdMS_TO_TICKS(100); // 100ms循环发送
    static float count_RCdata=0;
    while (1)
    {
        TickType_t now = xTaskGetTickCount();

        // 发送摇杆数据（即时响应）
        if (xQueueReceive(RCdata_queue, &rc_data, 0))
        {
            Serial.println("xQueueReceive rc_data success");
            // 更新last_rc_data为最新收到的数据
            last_rc_data = rc_data;
            size_t n = pack_frame_u8(0x02, (uint8_t*)&rc_data, sizeof(rc_data), frame, sizeof(frame));//帧打包:vofa下来得摇杆raw数据-->(0x02,数据,数据长度,帧缓冲区,帧缓冲区大小)
            if (n > 0)
            {
                Serial1.write(frame, n);
                Serial.println("Rc_data asend success");
            }
            else
            {
                Serial.println("stm32_uart_tx_task error");//缓冲不够
            }
        }

        // 循环发送手柄数据（固定间隔）
        if (now - last_rc_loop_send >= rc_loop_interval)
        {
            count_RCdata++;
            if(count_RCdata<100)
            { last_rc_data = {count_RCdata, count_RCdata,count_RCdata, count_RCdata};}
            else{count_RCdata=0;}
            size_t n = pack_frame_u8(0x02, (uint8_t*)&last_rc_data, sizeof(last_rc_data), frame, sizeof(frame));
            if (n > 0)
            {
                Serial1.write(frame, n);
                Serial.println("[LOOP] RC data sent (loop)");
            }
            else
            {
                Serial.println("RC loop frame packing error");
            }
            last_rc_loop_send = now;
        }

        // 发送PID参数（即时响应）
        if (xQueueReceive(pid_queue, &pid_msg, 0))
        {
            Serial.println("xQueueReceive pid_msg success");
            uint8_t cmd;
            switch (pid_msg.axis)
            {
                case PID_AXIS_PITCH:
                    cmd = 0x03;
                    break;
                case PID_AXIS_ROLL:
                    cmd = 0x04;
                    break;
                case PID_AXIS_YAW:
                    cmd = 0x05;
                    break;
                default:
                    Serial.println("Unknown axis, skipping");
                    continue;
            }
            size_t n = pack_frame_u8(cmd, (uint8_t*)&pid_msg.pid, sizeof(AxisPID), pid_frame, sizeof(pid_frame));
            if (n > 0)
            {
                Serial1.write(pid_frame, n);
                Serial.printf("PID axis=%d sent, cmd=0x%02X\n", pid_msg.axis, cmd);
            }
            else
            {
                Serial.println("PID frame packing error");
            }
        }

        // 定时发送0x00暂停命令
        if (now - last_pause_send >= pause_interval)
        {
            size_t n = pack_frame_u8(0x00, nullptr, 0, pause_frame, sizeof(pause_frame));
            if (n > 0)
            {
                Serial1.write(pause_frame, n);
                Serial.println("[DEBUG] Auto send PAUSE command (0x00)");
            }
            else
            {
                Serial.println("Pause frame packing error");
            }
            last_pause_send = now;
        }

        // 定时发送调试PID参数（保持原有2秒一个轴）
        if (now - last_debug_pid_send >= debug_pid_interval)
        {
            uint8_t cmd = axis_cmd[pid_send_index];
            const AxisPID* pid = &debug_pid[pid_send_index];
            size_t n = pack_frame_u8(cmd, (uint8_t*)pid, sizeof(AxisPID), pid_frame, sizeof(pid_frame));
            if (n > 0)
            {
                Serial1.write(pid_frame, n);
                Serial.printf("[DEBUG] Auto PID axis=%d sent, cmd=0x%02X\r\n", pid_send_index, cmd);
            }
            pid_send_index = (pid_send_index + 1) % 3;
            last_debug_pid_send = now;
        }
        
        // 循环延迟，减少CPU占用
        vTaskDelay(pdMS_TO_TICKS(10)); // 缩短延迟以更灵敏地响应队列
    }
}



// 串口1监听任务 (STM32 -> ESP32)
void stm32_uart_rx_task(void *pvParameters) 
{
    Serial.println("STM32串口接收任务启动");

    // 临时缓冲
    uint8_t temp_buffer[TEMP_BUFFER_SIZE];

    // 空闲超时机制：20ms 内无新字节则翻转
    const TickType_t IDLE_TICKS = pdMS_TO_TICKS(20);
    TickType_t last_rx_tick = xTaskGetTickCount();

    while (1) 
    {
        // 阻塞式读取串口数据
        int bytes_read = Serial1.readBytes(temp_buffer, sizeof(temp_buffer));

        if (bytes_read > 0) 
        {
            // 获取当前写缓冲
            double_buffer_t *current_buffer = &g_double_buffer[g_current_write_buffer];

            // 检查是否有足够空间
            if (current_buffer->write_pos + bytes_read <= BUFFER_SIZE) {
                // 复制数据到双缓冲
                memcpy(&current_buffer->buffer[current_buffer->write_pos], temp_buffer, bytes_read);
                current_buffer->write_pos += bytes_read;

                // 更新最近接收时间
                last_rx_tick = xTaskGetTickCount();

                // 如果缓冲快要满，则翻转缓冲（立即交给解析）
                if (current_buffer->write_pos >= BUFFER_SIZE_WATER_MARK) {
                    current_buffer->ready_for_parse = true;
                    g_current_write_buffer = (g_current_write_buffer + 1) % 2;
                    xSemaphoreGive(g_buffer_semaphore);
                }

            } else {
                // 缓冲溢出：立即翻转并通知（本批数据丢弃）
                current_buffer->ready_for_parse = true;
                g_current_write_buffer = (g_current_write_buffer + 1) % 2;
                xSemaphoreGive(g_buffer_semaphore);
                Serial.println("stm32_uart_rx_task: overflow, abandoning data from STM32");
            }

        } 
        else 
        {
            // 空闲检测：20ms 内无新数据且当前块有内容 -> 交给解析，降低延迟
            double_buffer_t *current_buffer = &g_double_buffer[g_current_write_buffer];
            TickType_t now = xTaskGetTickCount();
            if (current_buffer->write_pos > 0 && (now - last_rx_tick) >= IDLE_TICKS) 
            {
                current_buffer->ready_for_parse = true;
                g_current_write_buffer = (g_current_write_buffer + 1) % 2;
                xSemaphoreGive(g_buffer_semaphore);
            }
        }

        // 让出 CPU
        vTaskDelay(1 / portTICK_PERIOD_MS);
    }
}

// STM32解析任务
void stm32_parse_task(void *pvParameters) 
{
    Serial.println("STM32解析任务启动");
    reset_parser_context();//初始化解析器上下文,这是g_parser_ctx 首次有意义的赋值操作
    while (1) 
    {
        // 等待信号量通知有缓冲可解析
        if (xSemaphoreTake(g_buffer_semaphore, portMAX_DELAY) == pdTRUE) 
        {
            for (int i = 0; i < 2; i++)                                   // 查找可解析的缓冲 
            {
                if (g_double_buffer[i].ready_for_parse)                   // 解析缓冲
                {
                    //Serial.printf("now is buffer  %d\n", i);
                    parse_buffer(g_double_buffer[i].buffer, g_double_buffer[i].write_pos);
                    // 重置缓冲状态
                    g_double_buffer[i].write_pos = 0;
                    g_double_buffer[i].ready_for_parse = false;
                    
                    // 输出错误统计
                    if (g_parser_ctx.error_count > 0) 
                    {
                        Serial.printf("解析错误计数: %u\n", g_parser_ctx.error_count);
                    }
                }
            }
        }
    }
}

// UDP接收数据任务
// 现在默认关闭“假数据往 RCdata_queue 塞 Ry++”的逻辑，避免干扰真实遥杆。
void udp_rx_task(void *pvParameters)
{
    Serial.println("UDP接收数据任务启动");

#if USE_UDP_FAKE_RC
    RCData_t rcdata{};
    while (1) {
        rcdata.Ry++;
        xQueueSend(RCdata_queue, &rcdata, 0);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
#else
    // 暂时不使用 UDP 遥控，只做占位和调试输出
    while (1) {
        // 你以后可以在这里实现真正的 UDP 接收 → RCData_t 映射
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
#endif
}

// WebSocket 消息处理任务
// 负责从 ws_queue 取出 JSON 字符串，解析 type / Lx/Ly/Rx/Ry / action，
// 然后把遥杆数据塞进 RCdata_queue，由 stm32_uart_tx_task 发给 STM32。
void websocket_task(void *pvParameters)
{
    Serial.println("websocket_task started");

    ws_message_t msg; //WEBSOCKET消息队列
    JsonDocument doc; //JSON文档对象，用于解析JSON字符串
    uint16_t voltage;
    while (1) {
        // 阻塞等待 WebSocket 消息
        if (xQueueReceive(ws_queue, &msg, portMAX_DELAY) == pdTRUE) {
            // 解析 JSON
            DeserializationError err = deserializeJson(doc, msg.data);//
            if (err) {
                Serial.print("[WS] JSON parse error: ");
                Serial.println(err.c_str());
                continue;
            }

            const char *type = doc["type"];
            if (!type) {
                Serial.println("[WS] JSON missing 'type'");
                continue;
            }

            if (strcmp(type, "rc") == 0) 
            {
                // 遥杆控制消息
                RCData_t rc{};
                rc.Lx = doc["Lx"] | 0.0f;
                rc.Ly = doc["Ly"] | 0.0f;
                rc.Rx = doc["Rx"] | 0.0f;
                rc.Ry = doc["Ry"] | 0.0f;
                // whc mark
                // 打印一下方便调试
                Serial.printf("[WS] RC: L(%.2f, %.2f) R(%.2f, %.2f)\n",
                              rc.Lx, rc.Ly, rc.Rx, rc.Ry);

                if (RCdata_queue != NULL) {
                    BaseType_t ok = xQueueSend(RCdata_queue, &rc, 0);
                    if (ok != pdTRUE) {
                        Serial.println("[WS] RCdata_queue full, drop rc");
                    }
                }

            } 
            else if (strcmp(type, "cmd") == 0) 
            {
                // 按钮指令消息
                const char *action = doc["action"] | "";
                Serial.printf("[WS] CMD: %s\n", action);

                // TODO：这里可以调用你已有的“给 STM32 发模式/解锁指令”的接口
                // 例如 send_flight_mode_cmd(action); 等等（占位）

                // 回发一个简单 ack
                if (msg.client && msg.client->canSend()) 
                {
                    JsonDocument ack;          // NEW: JsonDocument 代替 StaticJsonDocument
                    ack["type"]   = "ack";
                    ack["action"] = action;
                    String out;
                    serializeJson(ack, out);
                    msg.client->text(out);
                }

            } 
            else if (strcmp(type, "ping") == 0) {      // NEW: ping 处理
                // 用 double 接收 JS 端传来的 performance.now()（是一个浮点数）
                double t = doc["t"] | 0.0;   // 如果没带 t，就当 0.0

                JsonDocument pongDoc;
                pongDoc["type"] = "pong";
                pongDoc["t"]    = t;        // 原样带回

                String out;
                serializeJson(pongDoc, out);

                if (msg.client && msg.client->canSend()) {
                    msg.client->text(out);  // 把 pong 发回给当前客户端
                }
                // 不必放入任何队列，不需要转发给 STM32
            }
            
            else if (strcmp(type, "pid") == 0) {      // PID参数更新消息
                // 解析轴标识
                const char *axis_str = doc["axis"] | "";
                pid_axis_t axis;
                if (strcmp(axis_str, "roll") == 0) {
                    axis = PID_AXIS_ROLL;
                } else if (strcmp(axis_str, "pitch") == 0) {
                    axis = PID_AXIS_PITCH;
                } else if (strcmp(axis_str, "yaw") == 0) {
                    axis = PID_AXIS_YAW;
                } else {
                    Serial.printf("[WS] Unknown axis: %s\n", axis_str);
                    continue;
                }

                // 填充PID参数
                pid_update_msg_t pid_msg;
                pid_msg.axis = axis;
                pid_msg.pid.kp_angle = doc["kp_angle"] | 0.0f;
                pid_msg.pid.ki_angle = doc["ki_angle"] | 0.0f;
                pid_msg.pid.kd_angle = doc["kd_angle"] | 0.0f;
                pid_msg.pid.kp_rate  = doc["kp_rate"]  | 0.0f;
                pid_msg.pid.ki_rate  = doc["ki_rate"]  | 0.0f;
                pid_msg.pid.kd_rate  = doc["kd_rate"]  | 0.0f;

                Serial.printf("[WS] PID update: axis=%d, kp_angle=%.3f, ki_angle=%.3f, kd_angle=%.3f, kp_rate=%.3f, ki_rate=%.3f, kd_rate=%.3f\n",
                              axis,
                              pid_msg.pid.kp_angle, pid_msg.pid.ki_angle, pid_msg.pid.kd_angle,
                              pid_msg.pid.kp_rate, pid_msg.pid.ki_rate, pid_msg.pid.kd_rate);

                // 发送到PID队列
                if (pid_queue != NULL) {
                    BaseType_t ok = xQueueSend(pid_queue, &pid_msg, 0);
                    if (ok != pdTRUE) {
                        Serial.println("[WS] pid_queue full, drop pid");
                    }
                }

                // 可选：发送确认回客户端
                if (msg.client && msg.client->canSend()) {
                    JsonDocument ack;
                    ack["type"] = "pid_ack";
                    ack["axis"] = axis_str;
                    String out;
                    serializeJson(ack, out);
                    msg.client->text(out);
                }
            }
            else {
                Serial.printf("[WS] Unknown type: %s\n", type);
            }
        }
        if (adc_queue != NULL &&
            xQueueReceive(adc_queue, &voltage, 0) == pdTRUE) {

            // 通过统一的状态广播接口发给所有 WebSocket 客户端
            ws_broadcast_status(voltage);
        }
    }
}
