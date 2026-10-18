#include "bus_scanner.h"
#include "i2c_bus.h"
#include "sensor_driver.h"

uint8_t bus_scan_and_init(ActiveNode *out_nodes, uint8_t max_capacity) {
    uint8_t active_count = 0;

    for (uint8_t addr = MUX_ADDR_BASE; addr < MUX_ADDR_BASE + MUX_COUNT_MAX; addr++) {
        if (mux_probe(addr)) {
            mux_disable_all(addr);
        }
    }

    for (uint8_t m = 0; m < MUX_COUNT_MAX; m++) {
        uint8_t mux_addr = MUX_ADDR_BASE + m;

        if (!mux_probe(mux_addr)) {
            continue;
        }

        for (uint8_t ch = 0; ch < CHANNELS_PER_MUX; ch++) {
            if (active_count >= max_capacity) {
                break;
            }

            mux_select_channel(mux_addr, ch);
            delayMicroseconds(50);

            uint8_t chip_id = bmi160_read_chip_id(BMI160_I2C_ADDR);

            if (chip_id == BMI160_CHIP_ID_VAL) {
                if (bmi160_init_sensor(BMI160_I2C_ADDR)) {
                    out_nodes[active_count].mux_address = mux_addr;
                    out_nodes[active_count].channel = ch;
                    active_count++;
                }
            }

            mux_disable_all(mux_addr);
            delayMicroseconds(20);
        }
    }

    return active_count;
}

void print_startup_diagnostics(const ActiveNode *nodes, uint8_t count) {
    Serial.println();
    Serial.println("==============================================================");
    Serial.println("           ОТЧЕТ АППАРАТНОГО СКАНИРОВАНИЯ ШИНЫ I2C            ");
    Serial.println("==============================================================");

    uint8_t mux_found = 0;
    for (uint8_t m = 0; m < MUX_COUNT_MAX; m++) {
        uint8_t mux_addr = MUX_ADDR_BASE + m;
        bool exists = mux_probe(mux_addr);

        if (exists) {
            mux_found++;
            Serial.printf("[MUX 0x%02X] Обнаружен на основной шине:\n", mux_addr);

            for (uint8_t ch = 0; ch < CHANNELS_PER_MUX; ch++) {
                mux_select_channel(mux_addr, ch);
                delayMicroseconds(50);

                uint8_t id = bmi160_read_chip_id(BMI160_I2C_ADDR);
                if (id == BMI160_CHIP_ID_VAL) {
                    float temp = bmi160_read_temperature(BMI160_I2C_ADDR);
                    Serial.printf("  |-- Канал %d: [BMI160] ChipID=0x%02X | Температура=%.2f °C | СТАТУС: OK\n", ch, id, temp);
                } else if (id != 0x00) {
                    Serial.printf("  |-- Канал %d: [НЕИЗВЕСТНЫЙ] ChipID=0x%02X | СТАТУС: ОШИБКА\n", ch, id);
                } else {
                    Serial.printf("  |-- Канал %d: ПУСТО\n", ch);
                }
                mux_disable_all(mux_addr);
            }
        }
    }

    Serial.println("--------------------------------------------------------------");
    Serial.printf("ИТОГО НАЙДЕНО: Мультиплексоров: %d | Активных BMI160: %d\n", mux_found, count);
    Serial.printf("РАЗМЕР БИНАРНОГО КАДРА: %d байт\n", 7 + count * NODE_PAYLOAD_SIZE);
    Serial.println("==============================================================");
    Serial.println();
}