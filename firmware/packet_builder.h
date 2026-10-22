#ifndef PACKET_BUILDER_H
#define PACKET_BUILDER_H

#include <Arduino.h>
#include "config.h"

/// Вычисляет контрольную сумму CRC-16-CCITT для массива байтов.
uint16_t crc16_ccitt(const uint8_t *data, size_t length);

/// Формирует полный бинарный пакет кадра телеметрии (15 байт на датчик + маска MUX).
size_t build_binary_frame(
    uint16_t sequence_id,
    uint8_t node_count,
    uint8_t mux_mask,
    const uint8_t *raw_payload,
    uint8_t *out_frame_buffer
);

#endif