/* freertos.c - Final Version with Gas Logging */

#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <stdarg.h>
#include "lcd_i2c.h"

/* ============================================================================
   EXTERNAL HANDLES & FUNCTIONS
   ============================================================================ */
extern ADC_HandleTypeDef hadc1;
extern TIM_HandleTypeDef htim3;
extern UART_HandleTypeDef huart2; // Debug UART
extern UART_HandleTypeDef huart1; // ESP8266 UART

// Defined in main.c
extern void sendToESP(const char *msg);

/* ============================================================================
   TYPE DEFINITIONS
   ============================================================================ */
typedef enum {
    LEVEL_EMPTY_SAFE = 0,
    LEVEL_EMPTY_TOXIC,
    LEVEL_FULL_SAFE,
    LEVEL_FULL_TOXIC
} BinStatus;

typedef struct {
    bool is_blocked;
    uint32_t gas_value;
    BinStatus status;
} SensorData;

/* ============================================================================
   GLOBALS & HANDLES
   ============================================================================ */
SensorData sensorData;

// Mutexes
osMutexId_t dataMutexHandle;
osMutexId_t uartMutexHandle;

// Task Handles
osThreadId_t IRSensorTaskHandle;
osThreadId_t GasSensorTaskHandle;
osThreadId_t StatusTaskHandle;
osThreadId_t DisplayTaskHandle;
osThreadId_t ServoTaskHandle;
osThreadId_t IOTTaskHandle;

/* ============================================================================
   HELPER FUNCTIONS
   ============================================================================ */
// Thread-Safe Print for Debugging (UART2)
void Safe_Print(const char *fmt, ...) {
    char buffer[100];
    va_list args;
    // Try to acquire the UART mutex to prevent text scrambling
    if (osMutexAcquire(uartMutexHandle, osWaitForever) == osOK) {
        va_start(args, fmt);
        vsnprintf(buffer, sizeof(buffer), fmt, args);
        va_end(args);
        HAL_UART_Transmit(&huart2, (uint8_t*)buffer, strlen(buffer), 100);
        osMutexRelease(uartMutexHandle);
    }
}

// Servo Logic
void Servo_Stop(void) {
    HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
}

void Servo_PulseRotation(int8_t speed, uint16_t duration_ms) {
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);

    // Clamp speed
    if (speed > 100) speed = 100;
    if (speed < -100) speed = -100;

    // Calculate pulse (1500us is center/stop for continuous servo)
    uint16_t pulse = 1500 + (speed * 5);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, pulse);

    osDelay(duration_ms);
    Servo_Stop();
}

/* ============================================================================
   TASKS
   ============================================================================ */

/* Task 1: Read IR Sensor (100ms) */
void IRSensorTask(void *argument) {
    for(;;) {
        GPIO_PinState pin_state = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0);

        osMutexAcquire(dataMutexHandle, osWaitForever);
        sensorData.is_blocked = (pin_state == GPIO_PIN_RESET);
        osMutexRelease(dataMutexHandle);

        osDelay(100);
    }
}

/* Task 2: Read Gas Sensor (200ms) */
void GasSensorTask(void *argument) {
    for(;;) {
        HAL_ADC_Start(&hadc1);
        HAL_ADC_PollForConversion(&hadc1, 10);
        uint32_t raw_val = HAL_ADC_GetValue(&hadc1);
        HAL_ADC_Stop(&hadc1);

        osMutexAcquire(dataMutexHandle, osWaitForever);
        sensorData.gas_value = raw_val;
        osMutexRelease(dataMutexHandle);

        osDelay(200);
    }
}

/* Task 3: Determine Logic (200ms) */
void StatusTask(void *argument) {
    for(;;) {
        osMutexAcquire(dataMutexHandle, osWaitForever);
        bool full = sensorData.is_blocked;
        bool toxic = (sensorData.gas_value > 1000);

        if (full && toxic) sensorData.status = LEVEL_FULL_TOXIC;
        else if (full && !toxic) sensorData.status = LEVEL_FULL_SAFE;
        else if (!full && toxic) sensorData.status = LEVEL_EMPTY_TOXIC;
        else sensorData.status = LEVEL_EMPTY_SAFE;

        osMutexRelease(dataMutexHandle);
        osDelay(200);
    }
}

/* Task 4: Display (500ms) */
void DisplayTask(void *argument) {
    char line1[17], line2[17];
    char s_fill[10], s_tox[10];
    for(;;) {
        // --- MODIFIED SECTION START ---
        osMutexAcquire(dataMutexHandle, osWaitForever);
        BinStatus st = sensorData.status;
        bool blk = sensorData.is_blocked;
        uint32_t gas_val = sensorData.gas_value; // Read Gas Value
        osMutexRelease(dataMutexHandle);
        // --- MODIFIED SECTION END ---

        switch(st) {
            case LEVEL_EMPTY_SAFE:  strcpy(s_fill, "Empty"); strcpy(s_tox, "Safe"); break;
            case LEVEL_EMPTY_TOXIC: strcpy(s_fill, "Empty"); strcpy(s_tox, "TOXIC"); break;
            case LEVEL_FULL_SAFE:   strcpy(s_fill, "FULL!"); strcpy(s_tox, "Safe"); break;
            case LEVEL_FULL_TOXIC:  strcpy(s_fill, "FULL!"); strcpy(s_tox, "URGENT"); break;
        }

        snprintf(line1, 16, "Fill: %s", blk ? "BLOCKED" : "CLEAR");
        snprintf(line2, 16, "%s | %s", s_fill, s_tox);

        LCD_Clear();
        LCD_SetCursor(0,0);
        LCD_Print(line1);
        LCD_SetCursor(1,0);
        LCD_Print(line2);

        // --- ADDED PRINT STATEMENT ---
        Safe_Print("Gas Level: %lu | Fill: %s\r\n", gas_val, blk ? "BLOCKED" : "CLEAR");

        osDelay(500);
    }
}

/* Task 5: Servo (200ms) */
void ServoTask(void *argument) {
    bool was_blocked = false;
    for(;;) {
        osMutexAcquire(dataMutexHandle, osWaitForever);
        BinStatus st = sensorData.status;
        osMutexRelease(dataMutexHandle);

        if (st != LEVEL_EMPTY_SAFE) {
            if (!was_blocked) {
                Safe_Print("Servo: Closing Bin...\r\n");
                // Rotate to Close
                Servo_PulseRotation(50, 300);
            }
            was_blocked = true;
        } else {
            // Ensure stopped if safe
            Servo_Stop();
            was_blocked = false;
        }
        HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin); // Heartbeat LED
        osDelay(200);
    }
}

/* Task 6: IOT Communicator (1000ms) */
void IOTTask(void *argument) {
    bool message_sent = false;
    Safe_Print("IOT Task Started\r\n");

    for(;;) {
        osMutexAcquire(dataMutexHandle, osWaitForever);
        BinStatus current_status = sensorData.status;
        osMutexRelease(dataMutexHandle);

        // Check if bin is in a state that needs collection (Any "FULL" state)
        bool ready_for_pickup = (current_status == LEVEL_FULL_SAFE || current_status == LEVEL_FULL_TOXIC);

        if (ready_for_pickup && !message_sent) {
            Safe_Print("IOT: Sending Alert to ESP...\r\n");
            sendToESP("ALERT: Bin is FULL. Ready for Collection!");
            message_sent = true;
        }
        else if (!ready_for_pickup) {
            // Reset when bin is emptied
            message_sent = false;
        }

        osDelay(1000);
    }
}

/* ============================================================================
   INIT FUNCTION (Called from main.c)
   ============================================================================ */
void MX_FREERTOS_Init(void) {

    // Hardware Safety Init
    LCD_Init();
    LCD_Clear();
    LCD_Print("OS Starting...");
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET);

    // Create Mutexes
    const osMutexAttr_t dataMutex_attributes = { .name = "dataMutex" };
    dataMutexHandle = osMutexNew(&dataMutex_attributes);

    const osMutexAttr_t uartMutex_attributes = { .name = "uartMutex" };
    uartMutexHandle = osMutexNew(&uartMutex_attributes);

    // Create Tasks
    const osThreadAttr_t irAttr = { .name = "IRTask", .stack_size = 128 * 4, .priority = (osPriority_t) osPriorityNormal };
    IRSensorTaskHandle = osThreadNew(IRSensorTask, NULL, &irAttr);

    const osThreadAttr_t gasAttr = { .name = "GasTask", .stack_size = 128 * 4, .priority = (osPriority_t) osPriorityNormal };
    GasSensorTaskHandle = osThreadNew(GasSensorTask, NULL, &gasAttr);

    const osThreadAttr_t statAttr = { .name = "StatTask", .stack_size = 128 * 4, .priority = (osPriority_t) osPriorityNormal };
    StatusTaskHandle = osThreadNew(StatusTask, NULL, &statAttr);

    const osThreadAttr_t dispAttr = { .name = "DispTask", .stack_size = 256 * 4, .priority = (osPriority_t) osPriorityLow };
    DisplayTaskHandle = osThreadNew(DisplayTask, NULL, &dispAttr);

    const osThreadAttr_t servoAttr = { .name = "ServoTask", .stack_size = 128 * 4, .priority = (osPriority_t) osPriorityHigh };
    ServoTaskHandle = osThreadNew(ServoTask, NULL, &servoAttr);

    const osThreadAttr_t iotAttr = { .name = "IOTTask", .stack_size = 256 * 4, .priority = (osPriority_t) osPriorityLow };
    IOTTaskHandle = osThreadNew(IOTTask, NULL, &iotAttr);

    // Check for Heap Errors
    if (uartMutexHandle == NULL || dataMutexHandle == NULL) {
         LCD_SetCursor(0,0);
         LCD_Print("Heap Error!");
         while(1);
    }
}
