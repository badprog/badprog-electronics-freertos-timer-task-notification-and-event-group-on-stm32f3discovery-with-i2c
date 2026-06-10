// badprog.com
/**
 * @file    main.c
 * @brief   FreeRTOS timers, task notifications and event groups on STM32F3Discovery
 *
 * Bare metal C with FreeRTOS. No HAL, no CubeMX.
 *
 * This exercise builds on the previous one (semaphores and mutexes) and
 * introduces three new FreeRTOS mechanisms:
 *
 *   1. Software timer
 *      A periodic timer callback toggles PE8 every 500ms. No dedicated task,
 *      no vTaskDelay loop. The timer daemon handles it internally.
 *
 *   2. Task notification
 *      The USER button ISR sends a direct notification to button_task instead
 *      of using a binary semaphore. Faster, lighter, no kernel object needed.
 *
 *   3. Event group
 *      display_task waits for two bits to be set before it starts displaying:
 *        bit 0 = I2C initialised
 *        bit 1 = UART initialised
 *      Both bits are set in main() after each peripheral init. display_task
 *      blocks until both are set simultaneously.
 *
 * Tasks:
 *   sensor_task  (priority 3): reads LSM303DLHC via I2C, sends to queue
 *   display_task (priority 2): waits on event group, then receives from
 *                               queue and prints via UART (mutex protected)
 *   button_task  (priority 2): waits on task notification, prints via UART
 *   led_task     (priority 1): rotates LEDs PE9..PE15 (PE8 reserved for timer)
 *
 * Hardware:
 *   PA0     : USER button (EXTI0, rising edge)
 *   PB6     : I2C1 SCL
 *   PB7     : I2C1 SDA
 *   PA9     : USART1 TX (115200 baud)
 *   PE8     : blink LED (software timer, 500ms)
 *   PE9..PE15: compass LEDs (led_task rotation)
 *
 * CPU clock: 8 MHz (default HSI)
 *
 * -------------------------------------------------------------------------
 * NEW FREERTOS CONCEPTS
 * -------------------------------------------------------------------------
 *
 * Software timer (xTimerCreate / xTimerStart):
 *   A callback function called automatically by the timer daemon task after
 *   a configurable period. Two modes:
 *     pdTRUE  = auto-reload: repeats forever at the given period
 *     pdFALSE = one-shot: fires once then stops
 *   The callback runs in the timer daemon context, not in the calling task.
 *   Rules for timer callbacks:
 *     - Must never block (no vTaskDelay, no xSemaphoreTake with timeout)
 *     - Must be short and fast (shared daemon with all other timers)
 *   The timer daemon task priority is set in FreeRTOSConfig.h via
 *   configTIMER_TASK_PRIORITY.
 *
 * Task notification (xTaskNotifyGive / ulTaskNotifyTake):
 *   A lightweight signalling mechanism built into every task's TCB.
 *   No kernel object to create. Each task has a 32-bit notification value
 *   that acts as a lightweight counting semaphore.
 *   xTaskNotifyGive(handle) : increments the target task's notification value
 *   ulTaskNotifyTake(pdTRUE, timeout): blocks until value > 0, then clears it
 *   From an ISR: use xTaskNotifyGiveFromISR() instead.
 *   Faster than a binary semaphore because there is no queue involved.
 *   Limitation: one notification value per task, one sender at a time makes
 *   sense (multiple senders can overwrite each other).
 *
 * Event group (xEventGroupCreate / xEventGroupSetBits / xEventGroupWaitBits):
 *   A 24-bit register where each bit represents an independent event flag.
 *   Multiple tasks can set bits. Multiple tasks can wait on bits.
 *   xEventGroupSetBits(group, bits) : set one or more bits atomically
 *   xEventGroupWaitBits(group, bits, clearOnExit, waitAll, timeout):
 *     blocks until the specified bits match the condition:
 *       waitAll = pdTRUE : wait until ALL specified bits are set (AND)
 *       waitAll = pdFALSE: wake up when ANY specified bit is set (OR)
 *     clearOnExit = pdTRUE: clear the waited bits automatically on wake-up
 *   Useful for synchronising startup sequences or combining multiple events.
 *
 * -------------------------------------------------------------------------
 * REGISTER GLOSSARY (same as exo08, no new registers)
 * -------------------------------------------------------------------------
 */

#include <string.h>
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "timers.h"
#include "event_groups.h"
#include "stm32f303xc.h"

// ---------------------------------------------------------------------------
// Event group bit definitions
//
// Each bit represents one initialisation step that must complete before
// display_task is allowed to start showing data.
// BIT_I2C_READY : set after i2c1_init() and lsm303_init() succeed
// BIT_UART_READY: set after uart_init() succeeds
// display_task waits for both bits simultaneously (AND condition).
// ---------------------------------------------------------------------------
#define BIT_I2C_READY   ( 1UL << 0 )
#define BIT_UART_READY  ( 1UL << 1 )
#define BITS_ALL_READY  ( BIT_I2C_READY | BIT_UART_READY )

// ---------------------------------------------------------------------------
// Shared FreeRTOS objects
// ---------------------------------------------------------------------------

#define QUEUE_LENGTH    5

typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
} AccelData;

static QueueHandle_t      xAccelQueue   = NULL;
static SemaphoreHandle_t  xUartMutex    = NULL;
static EventGroupHandle_t xInitEvents   = NULL;
static TaskHandle_t       xButtonTask   = NULL;   // needed for direct notification
static TimerHandle_t      xBlinkTimer   = NULL;

// ---------------------------------------------------------------------------
// Task stack sizes (words)
// ---------------------------------------------------------------------------

#define SENSOR_TASK_STACK_SIZE  256
#define DISPLAY_TASK_STACK_SIZE 256
#define BUTTON_TASK_STACK_SIZE  128
#define LED_TASK_STACK_SIZE     128

// ---------------------------------------------------------------------------
// UART: USART1 on PA9, 115200 baud
// ---------------------------------------------------------------------------

static void uart_init(void)
{
    RCC->AHBENR  |= RCC_AHBENR_IOPAEN;
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;

    GPIOA->MODER  &= ~(0x3UL << 18);
    GPIOA->MODER  |=  (0x2UL << 18);
    GPIOA->AFR[1] &= ~(0xFUL << 4);
    GPIOA->AFR[1] |=  (0x7UL << 4);

    USART1->BRR = 69;
    USART1->CR1 = USART_CR1_UE | USART_CR1_TE;
}

static void uart_print_safe(const char *str)
{
    xSemaphoreTake(xUartMutex, portMAX_DELAY);
    const char *p = str;
    while (*p) {
        while (!(USART1->ISR & USART_ISR_TXE));
        USART1->TDR = (uint32_t)(*p++);
    }
    xSemaphoreGive(xUartMutex);
}

// ---------------------------------------------------------------------------
// LEDs: PE8..PE15
// PE8 is reserved for the blink timer. led_task rotates PE9..PE15.
// ---------------------------------------------------------------------------

static void leds_init(void)
{
    RCC->AHBENR |= RCC_AHBENR_IOPEEN;
    GPIOE->MODER &= ~(0xFFFFUL << 16);
    GPIOE->MODER |=  (0x5555UL << 16);
    GPIOE->BSRR   = LED_ALL_PINS_OFF;
}

// ---------------------------------------------------------------------------
// Button: PA0 as input with EXTI0 on rising edge
// ---------------------------------------------------------------------------

static void button_init(void)
{
    RCC->AHBENR  |= RCC_AHBENR_IOPAEN;
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;

    GPIOA->MODER &= ~(0x3UL << 0);

    SYSCFG->EXTICR[0] &= ~(0xFUL << 0);
    SYSCFG->EXTICR[0] |=  (0x0UL << 0);

    EXTI->RTSR |= (1UL << 0);
    EXTI->IMR  |= (1UL << 0);

    NVIC_SetPriority(EXTI0_IRQn, 6);
    NVIC_EnableIRQ(EXTI0_IRQn);
}

// ---------------------------------------------------------------------------
// EXTI0 ISR: fires when USER button (PA0) is pressed
//
// Instead of giving a binary semaphore (exo08), we send a direct task
// notification to button_task. xTaskNotifyGiveFromISR() increments the
// target task's internal notification counter.
// No semaphore object needed, slightly faster and uses less RAM.
// ---------------------------------------------------------------------------

void EXTI0_IRQHandler(void)
{
    EXTI->PR = (1UL << 0);

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    // Send notification directly to button_task by handle.
    // xButtonTask handle is stored globally so the ISR can reach it.
    vTaskNotifyGiveFromISR(xButtonTask, &xHigherPriorityTaskWoken);

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

// ---------------------------------------------------------------------------
// I2C1: PB6 (SCL), PB7 (SDA), 100 kHz
// ---------------------------------------------------------------------------

static void i2c1_init(void)
{
    RCC->AHBENR  |= RCC_AHBENR_IOPBEN;
    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;

    GPIOB->MODER   &= ~((0x3UL << 12) | (0x3UL << 14));
    GPIOB->MODER   |=  (0x2UL << 12) | (0x2UL << 14);
    GPIOB->OTYPER  |=  (1UL << 6) | (1UL << 7);
    GPIOB->OSPEEDR |=  (0x3UL << 12) | (0x3UL << 14);
    GPIOB->PUPDR   &= ~((0x3UL << 12) | (0x3UL << 14));
    GPIOB->PUPDR   |=  (0x1UL << 12) | (0x1UL << 14);
    GPIOB->AFR[0]  &= ~((0xFUL << 24) | (0xFUL << 28));
    GPIOB->AFR[0]  |=  (0x4UL << 24) | (0x4UL << 28);

    I2C1->CR1 &= ~I2C_CR1_PE;
    I2C1->TIMINGR = 0x10420F13U;
    I2C1->CR1 |= I2C_CR1_PE;
}

static void i2c_wait_busy(void)  { while (I2C1->ISR & I2C_ISR_BUSY); }
static void i2c_wait_tc(void)    { while (!(I2C1->ISR & I2C_ISR_TC)); }
static void i2c_write_byte(uint8_t b) { while (!(I2C1->ISR & I2C_ISR_TXIS)); I2C1->TXDR = b; }
static uint8_t i2c_read_byte(void)   { while (!(I2C1->ISR & I2C_ISR_RXNE)); return (uint8_t)I2C1->RXDR; }

static void i2c_start_write(uint8_t addr, uint8_t n)
{
    I2C1->CR2 = ((uint32_t)(addr << 1)) | ((uint32_t)n << I2C_CR2_NBYTES_Pos) | I2C_CR2_START;
}

static void i2c_start_read(uint8_t addr, uint8_t n)
{
    I2C1->CR2 = ((uint32_t)(addr << 1)) | I2C_CR2_RD_WRN
              | ((uint32_t)n << I2C_CR2_NBYTES_Pos) | I2C_CR2_START | I2C_CR2_AUTOEND;
}

static void i2c_stop(void) { I2C1->CR2 |= I2C_CR2_STOP; while (I2C1->ISR & I2C_ISR_BUSY); }

// ---------------------------------------------------------------------------
// LSM303DLHC
// ---------------------------------------------------------------------------

#define LSM303_ADDR         0x19U
#define LSM303_CTRL_REG1_A  0x20U
#define LSM303_CTRL_REG4_A  0x23U
#define LSM303_OUT_X_L_A    0x28U

static void lsm303_write_reg(uint8_t reg, uint8_t val)
{
    i2c_wait_busy();
    i2c_start_write(LSM303_ADDR, 2);
    i2c_write_byte(reg);
    i2c_write_byte(val);
    i2c_wait_tc();
    i2c_stop();
}

static void lsm303_read_xyz(AccelData *data)
{
    uint8_t buf[6];
    i2c_wait_busy();
    i2c_start_write(LSM303_ADDR, 1);
    i2c_write_byte(LSM303_OUT_X_L_A | 0x80U);
    i2c_wait_tc();
    i2c_start_read(LSM303_ADDR, 6);
    for (uint8_t i = 0; i < 6; i++) { buf[i] = i2c_read_byte(); }
    data->x = (int16_t)((uint16_t)(buf[1] << 8) | buf[0]) >> 4;
    data->y = (int16_t)((uint16_t)(buf[3] << 8) | buf[2]) >> 4;
    data->z = (int16_t)((uint16_t)(buf[5] << 8) | buf[4]) >> 4;
}

static void lsm303_init(void)
{
    lsm303_write_reg(LSM303_CTRL_REG1_A, 0x57U);
    lsm303_write_reg(LSM303_CTRL_REG4_A, 0x08U);
}

// ---------------------------------------------------------------------------
// Software timer callback: blink PE8 every 500ms
//
// This function is called by the timer daemon task, not by any application
// task. It must never block. It simply toggles PE8 via BSRR.
//
// xTimerGetTimerID() could be used to identify which timer fired when
// multiple timers share the same callback. Not needed here.
// ---------------------------------------------------------------------------

static void blink_timer_callback(TimerHandle_t xTimer)
{
    (void)xTimer;

    // Toggle PE8: read current ODR state and flip bit 8
    // ODR bit 8 = 1 means PE8 is HIGH (LED on)
    if (GPIOE->ODR & (1UL << 8)) {
        GPIOE->BSRR = (1UL << (8 + 16));   // reset PE8 (LED off)
    } else {
        GPIOE->BSRR = (1UL << 8);           // set PE8 (LED on)
    }
}

// ---------------------------------------------------------------------------
// Task 1: sensor_task (priority 3)
// ---------------------------------------------------------------------------

static void sensor_task(void *pvParameters)
{
    (void)pvParameters;
    AccelData data;

    while (1) {
        lsm303_read_xyz(&data);
        xQueueSend(xAccelQueue, &data, 0);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

// ---------------------------------------------------------------------------
// Task 2: display_task (priority 2)
//
// Waits on the event group until both BIT_I2C_READY and BIT_UART_READY are
// set. This guarantees that I2C and UART are fully initialised before any
// data is read or printed.
//
// xEventGroupWaitBits parameters:
//   xInitEvents   : the event group handle
//   BITS_ALL_READY: the bits to wait for
//   pdTRUE        : clear the bits on exit (consume the events)
//   pdTRUE        : wait for ALL bits (AND condition)
//   portMAX_DELAY : block forever until condition is met
// ---------------------------------------------------------------------------

static void display_task(void *pvParameters)
{
    (void)pvParameters;

    // Block until both peripherals are ready
    xEventGroupWaitBits(xInitEvents, BITS_ALL_READY, pdTRUE, pdTRUE, portMAX_DELAY);

    uart_print_safe("FreeRTOS timers, notifications and event groups\r\n");
    uart_print_safe("---\r\n");
    uart_print_safe("     X        Y        Z\r\n");

    AccelData data;
    char buf[48];

    while (1) {
        if (xQueueReceive(xAccelQueue, &data, portMAX_DELAY) == pdTRUE) {
            int n = 0;
            int16_t vals[3] = {data.x, data.y, data.z};
            for (int axis = 0; axis < 3; axis++) {
                int16_t v = vals[axis];
                buf[n++] = (v < 0) ? '-': ' ';
                if (v < 0) v = -v;
                uint8_t started = 0;
                for (int16_t div = 1000; div >= 1; div /= 10) {
                    int16_t digit = v / div;
                    v %= div;
                    if (digit || started || div == 1) {
                        buf[n++] = '0' + (char)digit;
                        started = 1;
                    } else {
                        buf[n++] = ' ';
                    }
                }
                buf[n++] = ' '; buf[n++] = ' '; buf[n++] = ' ';
            }
            buf[n++] = '\r'; buf[n++] = '\n'; buf[n] = '\0';
            uart_print_safe(buf);
        }
    }
}

// ---------------------------------------------------------------------------
// Task 3: button_task (priority 2)
//
// Waits for a direct task notification instead of a binary semaphore.
// ulTaskNotifyTake(pdTRUE, portMAX_DELAY):
//   pdTRUE       : clear the notification value on exit (behaves like a
//                   binary semaphore: value goes back to 0 after take)
//   portMAX_DELAY: block forever until a notification arrives
// Returns the notification value before it was cleared (> 0 means notified).
// ---------------------------------------------------------------------------

static void button_task(void *pvParameters)
{
    (void)pvParameters;

    while (1) {
        // Block until EXTI0_IRQHandler calls vTaskNotifyGiveFromISR()
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        uart_print_safe("Button pressed!\r\n");
    }
}

// ---------------------------------------------------------------------------
// Task 4: led_task (priority 1)
// Rotates LEDs PE9..PE15. PE8 is handled by the blink timer.
// ---------------------------------------------------------------------------

static void led_task(void *pvParameters)
{
    (void)pvParameters;
    uint8_t led = 9;   // start at PE9 (PE8 reserved for blink timer)

    while (1) {
        // Turn off PE9..PE15 only (leave PE8 for the timer)
        GPIOE->BSRR = (0x7FUL << (9 + 16));
        GPIOE->BSRR = (1UL << led);
        led++;
        if (led > 15) led = 9;
        vTaskDelay(pdMS_TO_TICKS(250));
    }
}

// ---------------------------------------------------------------------------
// main
// ---------------------------------------------------------------------------

int main(void)
{
    // Initialise peripherals
    uart_init();
    leds_init();
    button_init();
    i2c1_init();
    lsm303_init();

    // Create FreeRTOS objects
    xAccelQueue = xQueueCreate(QUEUE_LENGTH, sizeof(AccelData));
    if (xAccelQueue == NULL) { while (1); }

    xUartMutex = xSemaphoreCreateMutex();
    if (xUartMutex == NULL) { while (1); }

    // Create event group for peripheral init synchronisation
    xInitEvents = xEventGroupCreate();
    if (xInitEvents == NULL) { while (1); }

    // Create software timer: auto-reload, 500ms period, blink callback
    // xTimerCreate parameters:
    //   "blink"              : name for debugging
    //   pdMS_TO_TICKS(500)   : period in ticks
    //   pdTRUE               : auto-reload (repeat forever)
    //   (void*)0             : timer ID (not used here)
    //   blink_timer_callback : callback function
    xBlinkTimer = xTimerCreate("blink",
                               pdMS_TO_TICKS(500),
                               pdTRUE,
                               (void *)0,
                               blink_timer_callback);
    if (xBlinkTimer == NULL) { while (1); }

    // Create tasks
    // button_task handle is stored in xButtonTask so the ISR can notify it
    xTaskCreate(sensor_task,  "sensor",  SENSOR_TASK_STACK_SIZE,  NULL, 3, NULL);
    xTaskCreate(display_task, "display", DISPLAY_TASK_STACK_SIZE, NULL, 2, NULL);
    xTaskCreate(button_task,  "button",  BUTTON_TASK_STACK_SIZE,  NULL, 2, &xButtonTask);
    xTaskCreate(led_task,     "leds",    LED_TASK_STACK_SIZE,     NULL, 1, NULL);

    // Signal that peripherals are ready via the event group.
    // display_task is blocked on xEventGroupWaitBits and will unblock
    // as soon as both bits are set.
    // These calls happen before vTaskStartScheduler so the bits are already
    // set when display_task first runs.
    xEventGroupSetBits(xInitEvents, BIT_UART_READY);
    xEventGroupSetBits(xInitEvents, BIT_I2C_READY);

    // Start the software timer.
    // xTimerStart must be called before vTaskStartScheduler because the
    // timer daemon task does not exist yet. The start command is queued and
    // executed as soon as the scheduler starts.
    xTimerStart(xBlinkTimer, 0);

    // Start the scheduler
    vTaskStartScheduler();

    while (1);
    return 0;
}