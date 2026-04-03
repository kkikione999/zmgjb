#include "system_params.h"
#include <string.h>
#include <stdio.h>
#include "usart.h"
#include "pid.h"
// ====== 芯片/HAL 相关头 ======
#include "stm32f4xx_hal.h"

// ========= 全局 =========
system_params_t g_system_params;   //定义一个系统 可变参数结构体 变量 作为默认参数组合
//system_params_t global_system_params_write;   //定义一个系统 可变参数结构体 变量  用于实际的写入操作
//不对把，其实只用一个g_system_params也是可以的
CascadedPIDParams g_runtime_pid =  //这个是全局运行时PID
{
    .roll  = {1,0,0,  1,0,0},      // 先给一组Kp,默认值（后面会被 Flash 里的参数覆盖）
    .pitch = {1,0,0,  1,0,0},
    .yaw   = {1,0,0,  1,0,0},
};

// ========= 镜像布局（整块序列化）=========
//typedef __packed struct
//{
//    uint32_t magic;
//    uint32_t version;
//    uint32_t length;
//    uint32_t reserved;
//    system_params_t payload;
//#if SYSTEM_PARAMS_USE_CRC32
//    uint32_t crc32;
//#endif
//} __attribute__((aligned(4))) system_params_image_t;

typedef struct 
{
    uint32_t magic;
    uint32_t version;
    uint32_t length;
    uint32_t reserved;
    system_params_t payload;
#if SYSTEM_PARAMS_USE_CRC32
    uint32_t crc32;
#endif
} system_params_image_t;


// ========= 本地工具 =========
static void flash_clear_all_flags(void)
{
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP   |
                           FLASH_FLAG_OPERR |
                           FLASH_FLAG_WRPERR|
                           FLASH_FLAG_PGAERR|
                           FLASH_FLAG_PGPERR|
                           FLASH_FLAG_PGSERR|
                           FLASH_FLAG_RDERR );
}

// 简单 CRC32（poly=0xEDB88320，初值全 1，结果按常规 XOR）
static uint32_t crc32_update(uint32_t crc, const uint8_t *data, uint32_t len)
{
    crc = ~crc;
    for (uint32_t i = 0; i < len; ++i)
    {
        crc ^= data[i];
        for (int b = 0; b < 8; ++b)
            crc = (crc >> 1) ^ (0xEDB88320UL & (-(int32_t)(crc & 1)));
    }
    return ~crc;
}

static uint32_t calc_crc32(const void *buf, uint32_t len)
{
#if SYSTEM_PARAMS_USE_CRC32
    return crc32_update(0xFFFFFFFFUL, (const uint8_t*)buf, len);
#else
    (void)buf; (void)len; return 0;
#endif
}

// Flash 里已有 payload 是否与 RAM 一致（一致则可跳过写入）
static bool payload_equal_to_flash(void)
{
    const system_params_image_t *img =
        (const system_params_image_t *)SYSTEM_PARAMS_FLASH_ADDR;

    if (img->magic   != SYSTEM_PARAMS_MAGIC ||
        img->version != SYSTEM_PARAMS_VERSION ||
        img->length  != sizeof(system_params_t))
        return false;

#if SYSTEM_PARAMS_USE_CRC32
    uint32_t crc = calc_crc32(&img->payload, sizeof(system_params_t));
    if (crc != img->crc32) return false;
#endif
    return (memcmp(&img->payload, &g_system_params, sizeof(system_params_t)) == 0);
}

// ========= 默认值 =========
void system_params_set_defaults(void)
{
memset(&g_system_params, 0, sizeof(g_system_params));

    // 示例：保留你原来的 pid[3]/cal[3] 默认值
    g_system_params.pid[0] = 0.10f;
    g_system_params.pid[1] = 0.40f;
    g_system_params.pid[2] = 0.30f;

    g_system_params.cal[0] = 1.0f;
    g_system_params.cal[1] = 0.0f;
    g_system_params.cal[2] = 0.0f;

    // 先整体清零
    for (int b = 0; b < NUM_ARRAY_BANKS; ++b)
        for (int i = 0; i < PARAM_ARRAY_SIZE; ++i)
            g_system_params.bank[b][i] = 0.0f;

    // ==== 给三个轴 PID 写一个默认值（按你飞控的实际经验改）====
    // 这里示例：角度环 (4,0,0)，角速度环 (0.1, 0, 0.02)
    float angP = 4.0f, angI = 0.0f, angD = 0.0f;
    float rateP = 0.1f, rateI = 0.0f, rateD = 0.02f;

    // Pitch
    g_system_params.bank[PID_BANK_PITCH][PID_IDX_KP_ANGLE] = angP;
    g_system_params.bank[PID_BANK_PITCH][PID_IDX_KI_ANGLE] = angI;
    g_system_params.bank[PID_BANK_PITCH][PID_IDX_KD_ANGLE] = angD;
    g_system_params.bank[PID_BANK_PITCH][PID_IDX_KP_RATE ] = rateP;
    g_system_params.bank[PID_BANK_PITCH][PID_IDX_KI_RATE ] = rateI;
    g_system_params.bank[PID_BANK_PITCH][PID_IDX_KD_RATE ] = rateD;

    // Roll：先直接复制一份
    g_system_params.bank[PID_BANK_ROLL][PID_IDX_KP_ANGLE] = angP;
    g_system_params.bank[PID_BANK_ROLL][PID_IDX_KI_ANGLE] = angI;
    g_system_params.bank[PID_BANK_ROLL][PID_IDX_KD_ANGLE] = angD;
    g_system_params.bank[PID_BANK_ROLL][PID_IDX_KP_RATE ] = rateP;
    g_system_params.bank[PID_BANK_ROLL][PID_IDX_KI_RATE ] = rateI;
    g_system_params.bank[PID_BANK_ROLL][PID_IDX_KD_RATE ] = rateD;

    // Yaw：可以单独给一套（这里只是示例）
    g_system_params.bank[PID_BANK_YAW][PID_IDX_KP_ANGLE] = 3.0f;
    g_system_params.bank[PID_BANK_YAW][PID_IDX_KI_ANGLE] = 0.0f;
    g_system_params.bank[PID_BANK_YAW][PID_IDX_KD_ANGLE] = 0.0f;
    g_system_params.bank[PID_BANK_YAW][PID_IDX_KP_RATE ] = 0.08f;
    g_system_params.bank[PID_BANK_YAW][PID_IDX_KI_RATE ] = 0.0f;
    g_system_params.bank[PID_BANK_YAW][PID_IDX_KD_RATE ] = 0.015f;
}

/**
 * @brief 这个函数肯定会在PID更新任务重被调用，传参以及形参还需要进一步设计。但是有一点我们因该是可以确定的：
这个函数里面需要包含SystemParams_Save()函数，并且由于flash的物理特性，本函数最好要放在循环里被频率调用，虽
然save里面有判断save前后参数组有无阿变化的逻辑
 * @param 
 * @param 
 * @retval 
*/
void system_params_write(void)
{
	memset(&g_system_params, 0, sizeof(g_system_params));

    g_system_params.pid[0] = 0.10f;
    g_system_params.pid[1] = 0.90f;
    g_system_params.pid[2] = 0.40f;

    g_system_params.cal[0] = 1.1f;
    g_system_params.cal[1] = 0.1f;
    g_system_params.cal[2] = 0.1f;

    // 先整体清零
    for (int b = 0; b < NUM_ARRAY_BANKS; ++b)
        for (int i = 0; i < PARAM_ARRAY_SIZE; ++i)
            g_system_params.bank[b][i] = 0.0f;

    // ==== 给三个轴 PID 写一个默认值（按你飞控的实际经验改）====
    // 这里示例：角度环 (4,0,0)，角速度环 (0.1, 0, 0.02)
    float angP = 4.0f, angI = 0.0f, angD = 0.0f;
    float rateP = 0.1f, rateI = 0.0f, rateD = 0.02f;

    // Pitch
    g_system_params.bank[PID_BANK_PITCH][PID_IDX_KP_ANGLE] = angP;
    g_system_params.bank[PID_BANK_PITCH][PID_IDX_KI_ANGLE] = angI;
    g_system_params.bank[PID_BANK_PITCH][PID_IDX_KD_ANGLE] = angD;
    g_system_params.bank[PID_BANK_PITCH][PID_IDX_KP_RATE ] = rateP;
    g_system_params.bank[PID_BANK_PITCH][PID_IDX_KI_RATE ] = rateI;
    g_system_params.bank[PID_BANK_PITCH][PID_IDX_KD_RATE ] = rateD;

    // Roll：先直接复制一份
    g_system_params.bank[PID_BANK_ROLL][PID_IDX_KP_ANGLE] = angP;
    g_system_params.bank[PID_BANK_ROLL][PID_IDX_KI_ANGLE] = angI;
    g_system_params.bank[PID_BANK_ROLL][PID_IDX_KD_ANGLE] = angD;
    g_system_params.bank[PID_BANK_ROLL][PID_IDX_KP_RATE ] = rateP;
    g_system_params.bank[PID_BANK_ROLL][PID_IDX_KI_RATE ] = rateI;
    g_system_params.bank[PID_BANK_ROLL][PID_IDX_KD_RATE ] = rateD;

    // Yaw：可以单独给一套（这里只是示例）
    g_system_params.bank[PID_BANK_YAW][PID_IDX_KP_ANGLE] = 3.0f;
    g_system_params.bank[PID_BANK_YAW][PID_IDX_KI_ANGLE] = 0.0f;
    g_system_params.bank[PID_BANK_YAW][PID_IDX_KD_ANGLE] = 0.0f;
    g_system_params.bank[PID_BANK_YAW][PID_IDX_KP_RATE ] = 0.08f;
    g_system_params.bank[PID_BANK_YAW][PID_IDX_KI_RATE ] = 0.0f;
    g_system_params.bank[PID_BANK_YAW][PID_IDX_KD_RATE ] = 0.015f;
	
	SystemParams_Save();
}

// ========= 打印 =========
void system_params_print(void)
{
    printf("[PARAM] PID=%.3f, %.3f, %.3f\r\n",g_system_params.pid[0], g_system_params.pid[1], g_system_params.pid[2]);
    // 只示例打印前几项，避免刷屏
    for (int b = 0; b < NUM_ARRAY_BANKS; ++b)
    {
        printf("[PARAM] bank[%d][0..4] =", b);
        for (int i = 0; i < (PARAM_ARRAY_SIZE < 5 ? PARAM_ARRAY_SIZE : 5); ++i)
            printf(" %.3f", g_system_params.bank[b][i]);
        printf(" ...\r\n");
    }
}

// ========= Load =========
bool SystemParams_Load(void)
{
    const system_params_image_t *img = (const system_params_image_t *)SYSTEM_PARAMS_FLASH_ADDR;//创建一个system_params_image_t类型的指针变量，并将flash中的抹个地址赋值给他，把 Flash 里某段内存当成 system_params_image_t 结构来读。
    if (img->magic   != SYSTEM_PARAMS_MAGIC ||img->version != SYSTEM_PARAMS_VERSION 
		||img->length  != sizeof(system_params_t))//只要一个不等于，判断就为假的
        return false;
#if SYSTEM_PARAMS_USE_CRC32
    uint32_t crc = calc_crc32(&img->payload, sizeof(system_params_t));
    if (crc != img->crc32) return false;
#endif
    memcpy(&g_system_params, &img->payload, sizeof(system_params_t));
	/*“从 img 结构体指针所指向的结构体的 payload 成员所在的位置开始，复制 system_params_t 
		那么大的一个内存块（通常就是一个 system_params_t 结构体），并将其内容覆盖到
		全局变量 g_system_params 所在的内存区域。”*/
    return true;
}

// ========= Save（整块拷贝，天然覆盖 6 组数组）=========
bool SystemParams_Save(void)
{
    // 变更检测：相同就不写，保护 Flash 寿命
    if (payload_equal_to_flash())
    {
        printf("[FLASH] Save skipped (no changes)\r\n");
        return true;
    }
    bool ok = false;
    // 组镜像
    system_params_image_t img;//定义镜像
    memset(&img, 0, sizeof(img));//批量内存清零操作，它将变量 img 所占用的每一个字节都设置为0。&img就意味着，清零操作是从结构体变量Img的起始地址开始的
    img.magic   = SYSTEM_PARAMS_MAGIC;
    img.version = SYSTEM_PARAMS_VERSION;
    img.length  = sizeof(system_params_t);
    img.reserved= 0;
    memcpy(&img.payload, &g_system_params, sizeof(system_params_t));//核心一句，我们现在把img作为中间人，把g_system_params的
	//地址复制给img的payload,其实月就是值的传递了.
#if SYSTEM_PARAMS_USE_CRC32
    img.crc32   = calc_crc32(&img.payload, sizeof(system_params_t));
#endif
    __disable_irq();
    flash_clear_all_flags();
    FLASH_EraseInitTypeDef erase = 
	{
        .TypeErase    = FLASH_TYPEERASE_SECTORS,
        .Sector       = FLASH_SECTOR_7,
        .NbSectors    = 1,
        .VoltageRange = FLASH_VOLTAGE_RANGE_3
    };
    uint32_t page_err = 0;
    if (HAL_FLASH_Unlock() == HAL_OK && HAL_FLASHEx_Erase(&erase, &page_err) == HAL_OK)//这里开始为Img赋予实际的地址
    {                                                                 
        uint32_t addr = SYSTEM_PARAMS_FLASH_ADDR;
        const uint32_t *p   = (const uint32_t *)&img;
        const uint32_t *end = (const uint32_t *)((const uint8_t*)&img + sizeof(img));
        while (p < end)
        {
            if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr, *p) != HAL_OK)
                goto EXIT;
            addr += 4;
            p++;
        }
        // 写回校验
        const system_params_image_t *vfy =(const system_params_image_t *)SYSTEM_PARAMS_FLASH_ADDR;

        if (vfy->magic   != img.magic   ||vfy->version != img.version ||vfy->length  != img.length)
            goto EXIT;

#if SYSTEM_PARAMS_USE_CRC32
        if (vfy->crc32 != img.crc32) goto EXIT;
#endif
        if (memcmp(&vfy->payload, &img.payload, sizeof(system_params_t)) != 0)
            goto EXIT;

        ok = true;
    }

EXIT:
    HAL_FLASH_Lock();
    __enable_irq();
    printf(ok ? "[FLASH] Save OK\r\n" : "[FLASH] Save FAIL\r\n");
    return ok;
}

// ========= Init =========
void SystemParams_Init(void)
{
    if (SystemParams_Load())//我们先尝试从flash中加载，如果加载失败就使用咱们默认的值(  system_params_set_defaults(); )
    {
        printf("[PARAM] Load OK\r\n");
        return;//如果从这里出去了，那后面自然看不见打印的"save_enter"
    }
    printf("[PARAM] Load FAIL, use defaults\r\n");
    system_params_set_defaults();
	printf("save_enter");
    (void)SystemParams_Save();  // 可选：首启即固化默认值
}
