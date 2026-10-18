#include "bt_transport.h"
#include "BluetoothSerial.h"
#include "esp_gap_bt_api.h"

static BluetoothSerial SerialBT;
static volatile bool client_connected = false;

// Обработчик системных событий SPP для мониторинга статуса соединения
static void spp_event_callback(esp_spp_cb_event_t event, esp_spp_cb_param_t *param) {
    switch (event) {
        case ESP_SPP_SRV_OPEN_EVT:
            client_connected = true;
            Serial.println("[BT] Клиент подключен к каналу SPP.");
            break;
        case ESP_SPP_CLOSE_EVT:
            client_connected = false;
            Serial.println("[BT] Соединение с клиентом закрыто.");
            break;
        default:
            break;
    }
}

bool bt_transport_init(const char *device_name) {
    SerialBT.register_callback(spp_event_callback);

    if (!SerialBT.begin(device_name)) {
        return false;
    }

    // У платы нет ни дисплея, ни клавиатуры, поэтому объявляем возможности
    // "NoInputNoOutput". По стандарту Bluetooth это включает сопряжение
    // Just Works: ПК подключается одним кликом, без PIN-кода и сверки цифр.
    esp_bt_io_cap_t iocap = ESP_BT_IO_CAP_NONE;
    esp_bt_gap_set_security_param(ESP_BT_SP_IOCAP_MODE, &iocap, sizeof(uint8_t));

    return true;
}

bool bt_transport_is_connected() {
    return client_connected || SerialBT.hasClient();
}

void bt_transport_send(const uint8_t *buffer, size_t length) {
    if (buffer != nullptr && length > 0 && bt_transport_is_connected()) {
        SerialBT.write(buffer, length);
    }
}