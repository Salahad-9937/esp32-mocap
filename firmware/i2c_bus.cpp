#include "i2c_bus.h"
#include "config.h"

void i2c_bus_init() {
    Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL, (uint32_t)I2C_CLOCK_HZ);
    Wire.setTimeOut((uint16_t)I2C_TIMEOUT_MS);
}

void i2c_bus_recover() {
    Wire.end();
    pinMode(PIN_I2C_SDA, INPUT_PULLUP);
    pinMode(PIN_I2C_SCL, OUTPUT);

    // Если SDA прижата к земле ведомым устройством, генерируем до 9 импульсов SCL
    for (uint8_t i = 0; i < 9; i++) {
        if (digitalRead(PIN_I2C_SDA) == HIGH) {
            break;
        }
        digitalWrite(PIN_I2C_SCL, LOW);
        delayMicroseconds(5);
        digitalWrite(PIN_I2C_SCL, HIGH);
        delayMicroseconds(5);
    }

    // Формируем Stop Condition вручную
    pinMode(PIN_I2C_SDA, OUTPUT);
    digitalWrite(PIN_I2C_SDA, LOW);
    delayMicroseconds(5);
    digitalWrite(PIN_I2C_SCL, HIGH);
    delayMicroseconds(5);
    digitalWrite(PIN_I2C_SDA, HIGH);
    delayMicroseconds(5);

    // Реинициализация аппаратного I2C драйвера
    i2c_bus_init();
}

bool i2c_write_byte(uint8_t dev_addr, uint8_t value) {
    Wire.beginTransmission(dev_addr);
    Wire.write(value);
    return (Wire.endTransmission() == 0);
}

bool i2c_write_register(uint8_t dev_addr, uint8_t reg_addr, uint8_t value) {
    Wire.beginTransmission(dev_addr);
    Wire.write(reg_addr);
    Wire.write(value);
    return (Wire.endTransmission() == 0);
}

bool i2c_read_registers(uint8_t dev_addr, uint8_t start_reg, uint8_t *buffer, uint8_t length) {
    Wire.beginTransmission(dev_addr);
    Wire.write(start_reg);
    if (Wire.endTransmission(false) != 0) {
        return false;
    }

    uint8_t received = Wire.requestFrom((uint8_t)dev_addr, (uint8_t)length);
    if (received != length) {
        return false;
    }

    for (uint8_t i = 0; i < length; i++) {
        buffer[i] = Wire.read();
    }
    return true;
}