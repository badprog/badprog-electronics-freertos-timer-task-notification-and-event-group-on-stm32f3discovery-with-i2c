// badprog.com
#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

// ---------------------------------------------------------------------------
// FreeRTOSConfig.h : kernel configuration for STM32F3Discovery (Cortex-M4)
//
// This file is the single place where the FreeRTOS kernel is tuned for the
// target hardware and application requirements.
//
// Reference: https://www.freertos.org/a00110.html
// ---------------------------------------------------------------------------

// ---------------------------------------------------------------------------
// SCHEDULER BEHAVIOUR
// ---------------------------------------------------------------------------

#define configUSE_PREEMPTION 1
#define configUSE_TIME_SLICING 1
#define configUSE_PORT_OPTIMISED_TASK_SELECTION 1
#define configUSE_TICKLESS_IDLE 0

// ---------------------------------------------------------------------------
// CLOCK AND TICK
// ---------------------------------------------------------------------------

#define configCPU_CLOCK_HZ (8000000UL)
#define configTICK_RATE_HZ (1000)
#define configTICK_TYPE_WIDTH_IN_BITS TICK_TYPE_WIDTH_32_BITS

// ---------------------------------------------------------------------------
// TASK PRIORITIES AND STACK
// ---------------------------------------------------------------------------

// Priority levels:
//   0 = idle (FreeRTOS internal)
//   1 = low   (led_task)
//   2 = normal (display_task, button_task)
//   3 = high  (sensor_task)
//   4 = reserved
#define configMAX_PRIORITIES (5)
#define configMINIMAL_STACK_SIZE (128)
#define configTOTAL_HEAP_SIZE (14 * 1024)
#define configMAX_TASK_NAME_LEN (16)

// ---------------------------------------------------------------------------
// HOOKS
// ---------------------------------------------------------------------------

#define configUSE_IDLE_HOOK 0
#define configUSE_TICK_HOOK 0

// ---------------------------------------------------------------------------
// MEMORY ALLOCATION
// ---------------------------------------------------------------------------

#define configSUPPORT_DYNAMIC_ALLOCATION 1
#define configSUPPORT_STATIC_ALLOCATION 0

// ---------------------------------------------------------------------------
// STACK OVERFLOW DETECTION
// ---------------------------------------------------------------------------

#define configCHECK_FOR_STACK_OVERFLOW 2

// ---------------------------------------------------------------------------
// MALLOC FAILURE HOOK
// ---------------------------------------------------------------------------

#define configUSE_MALLOC_FAILED_HOOK 1

// ---------------------------------------------------------------------------
// RUNTIME STATS AND DEBUGGING
// ---------------------------------------------------------------------------

#define configUSE_TRACE_FACILITY 1
#define configUSE_STATS_FORMATTING_FUNCTIONS 1
#define configGENERATE_RUN_TIME_STATS 0

// ---------------------------------------------------------------------------
// KERNEL FEATURES
// ---------------------------------------------------------------------------

#define configUSE_QUEUE_SETS 0
#define configUSE_MUTEXES 1
#define configUSE_RECURSIVE_MUTEXES 0
#define configUSE_COUNTING_SEMAPHORES 0

// Software timers : enabled for this exo
// configTIMER_TASK_PRIORITY : the timer daemon task priority.
//   Set it above led_task (1) and below sensor_task (3) so timer callbacks
//   do not starve sensor reads but still preempt LED rotation.
// configTIMER_QUEUE_LENGTH : number of pending timer commands the daemon
//   can queue. 10 is more than enough for our use case.
// configTIMER_TASK_STACK_DEPTH : stack for the timer daemon task in words.
//   128 words = 512 bytes is sufficient for simple callbacks.
#define configUSE_TIMERS 1
#define configTIMER_TASK_PRIORITY (2)
#define configTIMER_QUEUE_LENGTH 10
#define configTIMER_TASK_STACK_DEPTH (128)

// Task notifications : one 32-bit notification value per task.
// Used in this exo to replace the binary semaphore for button events.
// Faster than a semaphore: no kernel object created, value stored in TCB.
#define configUSE_TASK_NOTIFICATIONS 1
#define configTASK_NOTIFICATION_ARRAY_ENTRIES 1

// ---------------------------------------------------------------------------
// CORTEX-M4 INTERRUPT PRIORITY CONFIGURATION
// ---------------------------------------------------------------------------

#define configLIBRARY_LOWEST_INTERRUPT_PRIORITY 15
#define configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY 5

#define configKERNEL_INTERRUPT_PRIORITY \
  (configLIBRARY_LOWEST_INTERRUPT_PRIORITY << (8 - 4))

#define configMAX_SYSCALL_INTERRUPT_PRIORITY \
  (configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY << (8 - 4))

// ---------------------------------------------------------------------------
// OPTIONAL API FUNCTIONS
// ---------------------------------------------------------------------------

#define INCLUDE_vTaskDelay 1
#define INCLUDE_vTaskDelete 1
#define INCLUDE_vTaskSuspend 1
#define INCLUDE_xTaskGetCurrentTaskHandle 1
#define INCLUDE_uxTaskGetStackHighWaterMark 1

// ---------------------------------------------------------------------------
// MAP FREERTOS HANDLERS TO CORTEX-M VECTOR TABLE NAMES
// ---------------------------------------------------------------------------

#define xPortPendSVHandler PendSV_Handler
#define vPortSVCHandler SVC_Handler
#define xPortSysTickHandler SysTick_Handler

#endif  // FREERTOS_CONFIG_H