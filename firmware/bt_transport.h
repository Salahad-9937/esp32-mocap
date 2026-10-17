#ifndef BT_TRANSPORT_H
#define BT_TRANSPORT_H

#include <Arduino.h>

/// Инициализирует профиль Bluetooth Classic SPP.
bool bt_transport_init(const char *device_name);

/// Проверяет наличие активного подключения со стороны ПК.
bool bt_transport_is_connected();

/// Отправляет массив байтов в канал Bluetooth SPP.
void bt_transport_send(const uint8_t *buffer, size_t length);

#endif