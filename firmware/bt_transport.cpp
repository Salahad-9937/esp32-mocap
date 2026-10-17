#include "bt_transport.h"
#include "BluetoothSerial.h"

static BluetoothSerial SerialBT;

bool bt_transport_init(const char *device_name) {
    return SerialBT.begin(device_name);
}

bool bt_transport_is_connected() {
    return SerialBT.hasClient();
}

void bt_transport_send(const uint8_t *buffer, size_t length) {
    if (buffer != nullptr && length > 0) {
        SerialBT.write(buffer, length);
    }
}