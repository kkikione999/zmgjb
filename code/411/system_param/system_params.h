#ifndef SYSTEM_PARAMS_H
#define SYSTEM_PARAMS_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// -------- 你可以按需修改的容量参数 --------
#ifndef PARAM_ARRAY_SIZE
#define PARAM_ARRAY_SIZE   50
#endif

#ifndef NUM_ARRAY_BANKS
#define NUM_ARRAY_BANKS    6   // 6 组数组
#endif

// Flash 存储位置（F411 的 Sector 7，一般从 0x08060000 开始，128KB）
#ifndef SYSTEM_PARAMS_FLASH_ADDR
#define SYSTEM_PARAMS_FLASH_ADDR   (0x08060000UL)
#endif

// 版本、校验配置
#define SYSTEM_PARAMS_MAGIC        (0xA55AF11FUL)
#define SYSTEM_PARAMS_VERSION      (0x00010001UL)
#define SYSTEM_PARAMS_USE_CRC32    (1)



// API
void system_params_set_defaults(void);
void system_params_print(void);       // 使用 printf（你可改为 RTT/UART）
bool SystemParams_Load(void);
bool SystemParams_Save(void);
void SystemParams_Init(void);         // Load 失败则默认化
void system_params_write(void);
#ifdef __cplusplus
}
#endif

#endif // SYSTEM_PARAMS_H
