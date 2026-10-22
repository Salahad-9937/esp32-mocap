#include "bus_scanner.h"
#include "i2c_bus.h"
#include "sensor_driver.h"

uint8_t bus_scan_and_init(ActiveNode *out_nodes, uint8_t max_capacity, uint8_t *out_mux_mask) {
    uint8_t active_count = 0;
    uint8_t found_mux_mask = 0;

    for (uint8_t m = 0; m < MUX_COUNT_MAX; m++) {
        uint8_t addr = MUX_ADDR_BASE + m;
        if (mux_probe(addr)) {
            found_mux_mask |= (1 << m);
            mux_disable_all(addr);
        }
    }

    if (out_mux_mask != nullptr) {
        *out_mux_mask = found_mux_mask;
    }

    const uint8_t sensor_addrs[2] = {BMI160_I2C_ADDR_LOW, BMI160_I2C_ADDR_HIGH};

    for (uint8_t m = 0; m < MUX_COUNT_MAX; m++) {
        uint8_t mux_addr = MUX_ADDR_BASE + m;
        if ((found_mux_mask & (1 << m)) == 0) {
            continue;
        }

        for (uint8_t ch = 0; ch < CHANNELS_PER_MUX; ch++) {
            mux_select_channel(mux_addr, ch);
            delayMicroseconds(50);

            for (uint8_t a = 0; a < 2; a++) {
                if (active_count >= max_capacity) {
                    break;
                }

                uint8_t s_addr = sensor_addrs[a];
                uint8_t chip_id = bmi160_read_chip_id(s_addr);

                if (chip_id == BMI160_CHIP_ID_VAL) {
                    if (bmi160_init_sensor(s_addr)) {
                        out_nodes[active_count].mux_address = mux_addr;
                        out_nodes[active_count].channel = ch;
                        out_nodes[active_count].sensor_address = s_addr;
                        active_count++;
                    }
                }
            }

            mux_disable_all(mux_addr);
            delayMicroseconds(20);
        }
    }

    return active_count;
}

void print_startup_diagnostics(const ActiveNode *nodes, uint8_t count, uint8_t mux_mask) {
    Serial.println();
    Serial.println("==============================================================");
    Serial.println("           ОТЧЕТ АППАРАТНОГО СКАНИРОВАНИЯ ШИНЫ I2C            ");
    Serial.println("==============================================================");

    uint8_t mux_found = 0;
    const uint8_t sensor_addrs[2] = {BMI160_I2C_ADDR_LOW, BMI160_I2C_ADDR_HIGH};

    for (uint8_t m = 0; m < MUX_COUNT_MAX; m++) {
        uint8_t mux_addr = MUX_ADDR_BASE + m;
        bool exists = (mux_mask & (1 << m)) != 0;

        if (exists) {
            mux_found++;
            Serial.printf("[MUX 0x%02X] Обнаружен на основной шине (Бит %d):\n", mux_addr, m);

            for (uint8_t ch = 0; ch < CHANNELS_PER_MUX; ch++) {
                mux_select_channel(mux_addr, ch);
                delayMicroseconds(50);

                bool channel_has_device = false;
                for (uint8_t a = 0; a < 2; a++) {
                    uint8_t s_addr = sensor_addrs[a];
                    uint8_t id = bmi160_read_chip_id(s_addr);
                    if (id == BMI160_CHIP_ID_VAL) {
                        float temp = bmi160_read_temperature(s_addr);
                        Serial.printf("  |-- Канал %d [ADDR 0x%02X]: [BMI160] ChipID=0x%02X | Темп=%.2f °C | СТАТУС: OK\n",
                                      ch, s_addr, id, temp);
                        channel_has_device = true;
                    } else if (id != 0x00 && id != 0xFF) {
                        Serial.printf("  |-- Канал %d [ADDR 0x%02X]: [НЕИЗВЕСТНЫЙ] ChipID=0x%02X | СТАТУС: ОШИБКА\n",
                                      ch, s_addr, id);
                        channel_has_device = true;
                    }
                }

                if (!channel_has_device) {
                    Serial.printf("  |-- Канал %d: ПУСТО\n", ch);
                }
                mux_disable_all(mux_addr);
            }
        } else {
            Serial.printf("[MUX 0x%02X] Не отвечает\n", mux_addr);
        }
    }

    Serial.println("--------------------------------------------------------------");
    Serial.printf("ИТОГО: Мультиплексоров: %d (Маска: 0x%02X) | Активных BMI160: %d\n", mux_found, mux_mask, count);
    Serial.printf("РАЗМЕР БИНАРНОГО КАДРА: %d байт\n", 8 + count * NODE_PAYLOAD_SIZE);
    Serial.println("==============================================================");
    Serial.println();
}