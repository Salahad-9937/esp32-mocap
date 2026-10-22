#include "config.h"
#include "i2c_bus.h"
#include "sensor_driver.h"
#include "bus_scanner.h"
#include "packet_builder.h"
#include "bt_transport.h"

// Статический пул зарегистрированных активных узлов
static ActiveNode active_nodes[MAX_ACTIVE_SENSORS];
static uint8_t active_node_count = 0;
static uint8_t detected_mux_mask = 0;

// Статические буферы данных (15 байт на узел: MUX + CH + ADDR + 12B IMU)
static uint8_t raw_payload_buffer[MAX_ACTIVE_SENSORS * NODE_PAYLOAD_SIZE];
static uint8_t tx_frame_buffer[8 + MAX_ACTIVE_SENSORS * NODE_PAYLOAD_SIZE];
static uint16_t frame_sequence_counter = 0;

// Состояние отладки и интервалов
static bool debug_telemetry_enabled = DEFAULT_DEBUG_TELEMETRY;
static unsigned long last_frame_micros = 0;
static unsigned long last_debug_print_millis = 0;
static const unsigned long frame_interval_micros = 1000000UL / TARGET_FPS;

// Антидребезг тактовой кнопки перезапуска
static unsigned long btn_press_start_time = 0;
static bool btn_was_pressed = false;

void execute_system_soft_restart() {
    Serial.println();
    Serial.println("[SYSTEM] Запущен программный перезапуск симуляционного контура...");

    i2c_bus_recover();
    active_node_count = bus_scan_and_init(active_nodes, MAX_ACTIVE_SENSORS, &detected_mux_mask);
    print_startup_diagnostics(active_nodes, active_node_count, detected_mux_mask);

    frame_sequence_counter = 0;
    last_frame_micros = micros();
    last_debug_print_millis = millis();

    Serial.println("[SYSTEM] Перезапуск завершен. Поток возобновлен.");
}

void setup() {
    Serial.begin(115200);
    delay(1000);

    pinMode(PIN_SOFT_RESTART_BTN, INPUT_PULLUP);

    Serial.println();
    Serial.println("==============================================================");
    Serial.println("          ESP32 IMU MOCAP SYSTEM :: INITIALIZING              ");
    Serial.println("==============================================================");

    if (bt_transport_init(BT_DEVICE_NAME)) {
        Serial.printf("[BT] Bluetooth Classic запущен под именем: %s\n", BT_DEVICE_NAME);
    } else {
        Serial.println("[BT] КРИТИЧЕСКАЯ ОШИБКА: Сбой инициализации Bluetooth!");
    }

    execute_system_soft_restart();
}

void loop() {
    bool btn_current_state = (digitalRead(PIN_SOFT_RESTART_BTN) == LOW);
    if (btn_current_state && !btn_was_pressed) {
        btn_press_start_time = millis();
        btn_was_pressed = true;
    } else if (btn_current_state && btn_was_pressed) {
        if (millis() - btn_press_start_time >= 40) {
            execute_system_soft_restart();
            btn_was_pressed = false;
        }
    } else if (!btn_current_state) {
        btn_was_pressed = false;
    }

    if (Serial.available() > 0) {
        char cmd = (char)Serial.read();
        if (cmd == 'd' || cmd == 'D') {
            debug_telemetry_enabled = !debug_telemetry_enabled;
            Serial.printf("\n[DEBUG] Вывод телеметрии: %s\n", debug_telemetry_enabled ? "ВКЛ" : "ВЫКЛ");
        }
    }

    if (!bt_transport_is_connected()) {
        delay(20);
        return;
    }

    unsigned long current_micros = micros();
    if (current_micros - last_frame_micros < frame_interval_micros) {
        return;
    }
    last_frame_micros = current_micros;

    if (active_node_count == 0) {
        return;
    }

    // Опрос узлов с сохранением MUX, Channel и SensorAddress в тело пакета
    for (uint8_t i = 0; i < active_node_count; i++) {
        uint8_t mux_addr = active_nodes[i].mux_address;
        uint8_t ch = active_nodes[i].channel;
        uint8_t s_addr = active_nodes[i].sensor_address;

        mux_select_channel(mux_addr, ch);
        delayMicroseconds(15);

        uint8_t *dest = &raw_payload_buffer[i * NODE_PAYLOAD_SIZE];
        dest[0] = mux_addr;
        dest[1] = ch;
        dest[2] = s_addr;

        if (!bmi160_burst_read_data(s_addr, &dest[3])) {
            memset(&dest[3], 0, 12);
        }

        mux_disable_all(mux_addr);
        delayMicroseconds(10);
    }

    size_t frame_size = build_binary_frame(
        frame_sequence_counter++,
        active_node_count,
        detected_mux_mask,
        raw_payload_buffer,
        tx_frame_buffer
    );

    bt_transport_send(tx_frame_buffer, frame_size);

    if (debug_telemetry_enabled && (millis() - last_debug_print_millis >= DEBUG_PRINT_INTERVAL_MS)) {
        last_debug_print_millis = millis();
        Serial.printf("[FRAME #%05u] Узлов: %d | Маска MUX: 0x%02X | Размер: %u Б\n",
                      frame_sequence_counter, active_node_count, detected_mux_mask, frame_size);
    }
}