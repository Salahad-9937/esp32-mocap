#include "sensor_driver.h"
#include "i2c_bus.h"
#include "config.h"

bool mux_select_channel(uint8_t mux_addr, uint8_t channel) {
    if (channel >= CHANNELS_PER_MUX) return false;
    return i2c_write_byte(mux_addr, (uint8_t)(1 << channel));
}

bool mux_disable_all(uint8_t mux_addr) {
    return i2c_write_byte(mux_addr, 0x00);
}

bool mux_probe(uint8_t mux_addr) {
    Wire.beginTransmission(mux_addr);
    return (Wire.endTransmission() == 0);
}

uint8_t bmi160_read_chip_id(uint8_t sensor_addr) {
    uint8_t id = 0;
    if (i2c_read_registers(sensor_addr, BMI160_REG_CHIP_ID, &id, 1)) {
        return id;
    }
    return 0x00;
}

bool bmi160_init_sensor(uint8_t sensor_addr) {
    // 1. Программный сброс кристалла
    if (!i2c_write_register(sensor_addr, BMI160_REG_CMD, BMI160_CMD_SOFT_RESET)) {
        return false;
    }
    delay(100);

    // 2. Проверка Chip ID после перезапуска
    uint8_t chip_id = bmi160_read_chip_id(sensor_addr);
    if (chip_id != BMI160_CHIP_ID_VAL) {
        return false;
    }

    // 3. Включение акселерометра в Normal Mode (задержка > 3.8 мс по даташиту)
    i2c_write_register(sensor_addr, BMI160_REG_CMD, BMI160_CMD_ACC_NORMAL);
    delay(5);

    // 4. Включение гироскопа в Normal Mode (задержка > 80 мс по даташиту)
    i2c_write_register(sensor_addr, BMI160_REG_CMD, BMI160_CMD_GYR_NORMAL);
    delay(85);

    // 5. Проверка статуса готовности аналоговых каскадов питания
    uint8_t pmu_status = 0;
    i2c_read_registers(sensor_addr, BMI160_REG_PMU_STATUS, &pmu_status, 1);
    if ((pmu_status & 0x3C) != 0x14) { // Бит 5:4 == 01 (acc normal), Бит 3:2 == 01 (gyr normal)
        return false;
    }

    // 6. Настройка диапазонов: Accel ±4g (0x05), Gyro ±1000 dps (0x02)
    i2c_write_register(sensor_addr, BMI160_REG_ACC_RANGE, 0x05);
    delayMicroseconds(10);
    i2c_write_register(sensor_addr, BMI160_REG_GYR_RANGE, 0x02);
    delayMicroseconds(10);

    // 7. Настройка частоты вывода данных ODR = 100 Гц в нормальном режиме фильтрации
    i2c_write_register(sensor_addr, BMI160_REG_ACC_CONF, 0x28);
    delayMicroseconds(10);
    i2c_write_register(sensor_addr, BMI160_REG_GYR_CONF, 0x28);
    delayMicroseconds(10);

    // 8. Активация сторожевого таймера шины I2C на 50 мс
    i2c_write_register(sensor_addr, BMI160_REG_NV_CONF, 0x06);
    delayMicroseconds(10);

    return true;
}

bool bmi160_burst_read_data(uint8_t sensor_addr, uint8_t *dest_buffer_12b) {
    return i2c_read_registers(sensor_addr, BMI160_REG_DATA_START, dest_buffer_12b, 12);
}

float bmi160_read_temperature(uint8_t sensor_addr) {
    uint8_t raw[2] = {0, 0};
    if (i2c_read_registers(sensor_addr, BMI160_REG_TEMP_START, raw, 2)) {
        int16_t t_raw = (int16_t)((raw[1] << 8) | raw[0]);
        return 23.0f + ((float)t_raw * 0.001953125f);
    }
    return 0.0f;
}