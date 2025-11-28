#include "lcd_i2c.h"
#include "main.h"
#include "string.h"

extern I2C_HandleTypeDef hi2c1;

static void LCD_Send(uint8_t data, uint8_t mode);

void LCD_Init(void) {
    HAL_Delay(200); // Robust Power-on delay (200ms)
    LCD_Command(0x33);
    HAL_Delay(5);    // Delay after first command
    LCD_Command(0x32);
    HAL_Delay(5);    // Delay after 4-bit mode setup
    LCD_Command(0x28); // 2-line, 5x8 font
    LCD_Command(0x0C); // Display ON, Cursor OFF
    LCD_Command(0x06); // Entry mode
    LCD_Command(0x01); // Clear display
    HAL_Delay(2);
}

void LCD_Command(uint8_t cmd) {
    LCD_Send(cmd, 0x00);
}

void LCD_PrintChar(char data) {
    LCD_Send(data, 0x01);
}

void LCD_Print(char *str) {
    while (*str) LCD_PrintChar(*str++);
}

void LCD_Clear(void) {
    LCD_Command(0x01);
    HAL_Delay(2);
}

void LCD_SetCursor(uint8_t row, uint8_t col) {
    uint8_t address = (row == 0 ? 0x80 : 0xC0) + col;
    LCD_Command(address);
}

static void LCD_Send(uint8_t data, uint8_t mode) {
    uint8_t hi = data & 0xF0;
    uint8_t lo = (data << 4) & 0xF0;

    // Control Bits: BL=0x08 (Backlight), E=0x04 (Enable)
    const uint8_t BL_ON = 0x08;
    const uint8_t E_PULSE = 0x04;

    uint8_t data_t[1]; // We transmit one byte at a time

    // --- High Nibble ---
    // 1. E=1 (Start Pulse)
    data_t[0] = hi | mode | BL_ON | E_PULSE;
    HAL_I2C_Master_Transmit(&hi2c1, LCD_ADDR, data_t, 1, 100);

    // 2. E=0 (End Pulse - critical falling edge)
    data_t[0] = hi | mode | BL_ON;
    HAL_I2C_Master_Transmit(&hi2c1, LCD_ADDR, data_t, 1, 100);

    // --- Low Nibble ---
    // 3. E=1 (Start Pulse)
    data_t[0] = lo | mode | BL_ON | E_PULSE;
    HAL_I2C_Master_Transmit(&hi2c1, LCD_ADDR, data_t, 1, 100);

    // 4. E=0 (End Pulse)
    data_t[0] = lo | mode | BL_ON;
    HAL_I2C_Master_Transmit(&hi2c1, LCD_ADDR, data_t, 1, 100);
}
