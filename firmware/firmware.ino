#include "config.h"
#include "i2c_bus.h"
#include "sensor_driver.h"
#include "bus_scanner.h"
#include "packet_builder.h"
#include "bt_transport.h"

// Статический пул зарегистрированных активных узлов
static ActiveNode active_nodes[MAX_ACTIVE_SENSORS];
static uint8_t active_node_count = 0;

// Статические буферы данных (Zero Heap Allocation)
static uint8_t raw_payload_buffer[MAX_ACTIVE_SENSORS * 12];
static uint8_t tx_frame_buffer[7 + MAX_ACTIVE_SENSORS * 12];
static uint16_t frame_sequence_counter = 0;

// Состояние отладки и интервалов
static bool debug_telemetry_enabled = DEFAULT_DEBUG_TELEMETRY;
static unsigned long last_frame_micros = 0;
static unsigned long last_debug_print_millis = 0;
static const unsigned long frame_interval_micros = 1000000UL / TARGET_FPS;

// Антидребезг тактовой кнопки перезапуска кода
static unsigned long btn_press_start_time = 0;
static bool btn_was_pressed = false;

/// Полная процедура инициализации и пересканирования системы без перезагрузки платы
void execute_system_soft_restart() {
    Serial.println();
    Serial.println("[SYSTEM] Запущен программный перезапуск симуляционного контура...");

    // 1. Аппаратный сброс шины I2C на случай залипания ведомых устройств
    i2c_bus_recover();

    // 2. Повторное автосканирование топологии и инициализация датчиков
    active_node_count = bus_scan_and_init(active_nodes, MAX_ACTIVE_SENSORS);

    // 3. Вывод детального аналитического отчета о подключениях
    print_startup_diagnostics(active_nodes, active_node_count);

    // 4. Сброс счетчика пакетов и таймеров
    frame_sequence_counter = 0;
    last_frame_micros = micros();
    last_debug_print_millis = millis();

    Serial.println("[SYSTEM] Перезапуск завершен. Поток возобновлен.");
    Serial.println("[INFO] Отправьте символ 'd' в Serial для включения/выключения вывода данных.");
}

void setup() {
    Serial.begin(115200);
    delay(1000);

    // Конфигурация пина кнопки со встроенной подтяжкой к питанию
    pinMode(PIN_SOFT_RESTART_BTN, INPUT_PULLUP);

    Serial.println();
    Serial.println("==============================================================");
    Serial.println("          ESP32 IMU MOCAP SYSTEM :: INITIALIZING              ");
    Serial.println("==============================================================");

    // Инициализация Bluetooth SPP
    if (bt_transport_init(BT_DEVICE_NAME)) {
        Serial.printf("[BT] Bluetooth Classic запущен под именем: %s\n", BT_DEVICE_NAME);
    } else {
        Serial.println("[BT] КРИТИЧЕСКАЯ ОШИБКА: Сбой инициализации Bluetooth!");
    }

    // Первичный запуск шины, сканирования и калибровки
    execute_system_soft_restart();
}

void loop() {
    // 1. Обработка кнопки горячего перезапуска кода
    bool btn_current_state = (digitalRead(PIN_SOFT_RESTART_BTN) == LOW);
    if (btn_current_state && !btn_was_pressed) {
        btn_press_start_time = millis();
        btn_was_pressed = true;
    } else if (btn_current_state && btn_was_pressed) {
        if (millis() - btn_press_start_time >= 40) { // Антидребезг 40 мс
            execute_system_soft_restart();
            btn_was_pressed = false;
        }
    } else if (!btn_current_state) {
        btn_was_pressed = false;
    }

    // 2. Обработка переключения отладки через консоль Serial
    if (Serial.available() > 0) {
        char cmd = (char)Serial.read();
        if (cmd == 'd' || cmd == 'D') {
            debug_telemetry_enabled = !debug_telemetry_enabled;
            Serial.printf("\n[DEBUG] Вывод телеметрии в консоль: %s\n", debug_telemetry_enabled ? "ВКЛЮЧЕН" : "ВЫКЛЮЧЕН");
        }
    }

    // 3. Контроль частоты кадрового цикла (70 Гц)
    unsigned long current_micros = micros();
    if (current_micros - last_frame_micros < frame_interval_micros) {
        return;
    }
    last_frame_micros = current_micros;

    if (active_node_count == 0) {
        return;
    }

    // 4. Последовательный опрос активных узлов
    for (uint8_t i = 0; i < active_node_count; i++) {
        uint8_t mux_addr = active_nodes[i].mux_address;
        uint8_t ch = active_nodes[i].channel;

        mux_select_channel(mux_addr, ch);
        delayMicroseconds(15);

        uint8_t *dest = &raw_payload_buffer[i * 12];
        if (!bmi160_burst_read_data(BMI160_I2C_ADDR, dest)) {
            memset(dest, 0, 12);
        }

        mux_disable_all(mux_addr);
        delayMicroseconds(10);
    }

    // 5. Сборка бинарного фрейма и отправка в Bluetooth
    size_t frame_size = build_binary_frame(
        frame_sequence_counter++,
        active_node_count,
        raw_payload_buffer,
        tx_frame_buffer
    );

    bt_transport_send(tx_frame_buffer, frame_size);

    // 6. Вывод отладочной информации (если включен)
    if (debug_telemetry_enabled && (millis() - last_debug_print_millis >= DEBUG_PRINT_INTERVAL_MS)) {
        last_debug_print_millis = millis();

        Serial.printf("[FRAME #%05u] Активных узлов: %d | Размер: %u байт\n",
                      frame_sequence_counter, active_node_count, frame_size);

        for (uint8_t i = 0; i < active_node_count; i++) {
            const uint8_t *d = &raw_payload_buffer[i * 12];
            int16_t gx = (int16_t)((d[1] << 8) | d[0]);
            int16_t gy = (int16_t)((d[3] << 8) | d[2]);
            int16_t gz = (int16_t)((d[5] << 8) | d[4]);
            int16_t ax = (int16_t)((d[7] << 8) | d[6]);
            int16_t ay = (int16_t)((d[9] << 8) | d[8]);
            int16_t az = (int16_t)((d[11] << 8) | d[10]);

            Serial.printf("  Узел %02d (MUX 0x%02X:CH%d) -> G:[%6d, %6d, %6d] | A:[%6d, %6d, %6d]\n",
                          i + 1, active_nodes[i].mux_address, active_nodes[i].channel, gx, gy, gz, ax, ay, az);
        }
    }
}