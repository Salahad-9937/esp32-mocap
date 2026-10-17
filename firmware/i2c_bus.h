#ifndef I2C_BUS_H
#define I2C_BUS_H

#include <Arduino.h>
#include <Wire.h>

/// Инициализирует аппаратную шину I2C с заданной частотой.
void i2c_bus_init();

/// Выполняет процедуру принудительного освобождения шины при залипании линии SDA.
void i2c_bus_recover();

/// Записывает один байт данных напрямую в устройство.
bool i2c_write_byte(uint8_t dev_addr, uint8_t value);

/// Записывает значение в целевой регистр устройства.
bool i2c_write_register(uint8_t dev_addr, uint8_t reg_addr, uint8_t value);

/// Считывает массив байтов из целевого регистра устройства через Repeated Start.
bool i2c_read_registers(uint8_t dev_addr, uint8_t start_reg, uint8_t *buffer, uint8_t length);

#endif