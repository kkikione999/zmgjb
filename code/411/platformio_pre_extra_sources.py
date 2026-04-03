import os

Import("env")


PROJECT_DIR = env.subst("$PROJECT_DIR")
BUILD_DIR = env.subst("$BUILD_DIR")
FREERTOS_DIR = os.path.join(
    PROJECT_DIR, "Middlewares", "Third_Party", "FreeRTOS", "Source"
)


env.BuildSources(
    os.path.join(BUILD_DIR, "extra_freertos"),
    FREERTOS_DIR,
    src_filter=[
        "+<croutine.c>",
        "+<event_groups.c>",
        "+<list.c>",
        "+<queue.c>",
        "+<stream_buffer.c>",
        "+<tasks.c>",
        "+<timers.c>",
        "+<CMSIS_RTOS_V2/cmsis_os2.c>",
        "+<portable/GCC/ARM_CM4F/port.c>",
        "+<portable/MemMang/heap_4.c>",
    ],
)
