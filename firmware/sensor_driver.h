#ifndef SENSOR_DRIVER_H
#define SENSOR_DRIVER_H

#include <Arduino.h>

/// Переключает активный канал мультиплексора PCA9548A.
bool mux_select_channel(uint8_t mux_addr, uint8_t channel);

/// Отключает все выходные каналы мультиплексора.
bool mux_disable_all(uint8_t mux_addr);

/// Проверяет отклик мультиплексора на I2C шине.
bool mux_probe(uint8_t mux_addr);

/// Считывает идентификационный регистр CHIP_ID датчика.
uint8_t bmi160_read_chip_id(uint8_t sensor_addr);

/// Выполняет пошаговую инициализацию BMI160 с выдержкой аппаратных задержек.
bool bmi160_init_sensor(uint8_t sensor_addr);

/// Считывает непрерывный блок 12 байт (гироскоп 6 байт, акселерометр 6 байт).
bool bmi160_burst_read_data(uint8_t sensor_addr, uint8_t *dest_buffer_12b);

/// Считывает температуру кристалла в градусах Цельсия.
float bmi160_read_temperature(uint8_t sensor_addr);

#endif