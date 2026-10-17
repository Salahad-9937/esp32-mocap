#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// 1. Аппаратные выводы ESP32
#define PIN_I2C_SDA             21
#define PIN_I2C_SCL             22
#define PIN_SOFT_RESTART_BTN    4   // Кнопка перезапуска кода (пин сигнального провода, второй на GND)

// 2. Параметры шины I2C
#define I2C_CLOCK_HZ            400000  // Частота Fast Mode 400 кГц
#define I2C_TIMEOUT_MS          3       // Предельный таймаут ожидания транзакции

// 3. Адресация оборудования
#define MUX_ADDR_BASE           0x70    // Базовый адрес мультиплексоров (0x70 .. 0x77)
#define MUX_COUNT_MAX           8       // Максимальное количество мультиплексоров на шине
#define CHANNELS_PER_MUX        8       // Количество каналов на один PCA9548A
#define BMI160_I2C_ADDR         0x68    // Аппаратный адрес датчика (SA0 на GND)
#define MAX_ACTIVE_SENSORS      64      // Максимальный предел емкости системы (8x8)

// 4. Регистры микросхемы Bosch BMI160
#define BMI160_REG_CHIP_ID      0x00
#define BMI160_REG_ERR_REG      0x02
#define BMI160_REG_PMU_STATUS   0x03
#define BMI160_REG_DATA_START   0x0C    // Начало блока 12 байт (GYR_X_L)
#define BMI160_REG_TEMP_START   0x20
#define BMI160_REG_ACC_CONF     0x40
#define BMI160_REG_ACC_RANGE    0x41
#define BMI160_REG_GYR_CONF     0x42
#define BMI160_REG_GYR_RANGE    0x43
#define BMI160_REG_NV_CONF      0x70
#define BMI160_REG_CMD          0x7E

#define BMI160_CHIP_ID_VAL      0xD1
#define BMI160_CMD_SOFT_RESET   0xB6
#define BMI160_CMD_ACC_NORMAL   0x11
#define BMI160_CMD_GYR_NORMAL   0x15

// 5. Настройки бинарного протокола и передачи
#define BT_DEVICE_NAME          "ESP32_MOCAP_HUB"
#define FRAME_PREAMBLE_0        0xAA
#define FRAME_PREAMBLE_1        0x55
#define TARGET_FPS              70      // Целевая частота цикла опроса (Гц)

// 6. Флаги и интервалы отладки
#define DEFAULT_DEBUG_TELEMETRY false   // Вывод сырых данных в Serial (false: выключен, true: включен)
#define DEBUG_PRINT_INTERVAL_MS 100     // Период прореживания вывода в консоль при отладке

// Структура дескриптора активного датчика
struct ActiveNode {
    uint8_t mux_address; // Адрес мультиплексора (0x70 .. 0x77)
    uint8_t channel;     // Номер подканала (0 .. 7)
};

#endif