#include "packet_builder.h"

uint16_t crc16_ccitt(const uint8_t *data, size_t length) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < length; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (uint8_t bit = 0; bit < 8; bit++) {
            if (crc & 0x8000) {
                crc = (crc << 1) ^ 0x1021;
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}

size_t build_binary_frame(
    uint16_t sequence_id,
    uint8_t node_count,
    uint8_t mux_mask,
    const uint8_t *raw_payload,
    uint8_t *out_frame_buffer
) {
    size_t payload_len = (size_t)node_count * NODE_PAYLOAD_SIZE;

    // 1. Преамбула синхронизации (0xAA 0x55)
    out_frame_buffer[0] = FRAME_PREAMBLE_0;
    out_frame_buffer[1] = FRAME_PREAMBLE_1;

    // 2. Инкрементный счетчик пакетов (Little-Endian)
    out_frame_buffer[2] = (uint8_t)(sequence_id & 0xFF);
    out_frame_buffer[3] = (uint8_t)((sequence_id >> 8) & 0xFF);

    // 3. Количество переданных узлов
    out_frame_buffer[4] = node_count;

    // 4. Битовая маска обнаруженных мультиплексоров (биты 0..7 = MUX 0x70..0x77)
    out_frame_buffer[5] = mux_mask;

    // 5. Полезная нагрузка (N * 15 байт)
    if (payload_len > 0 && raw_payload != nullptr) {
        memcpy(&out_frame_buffer[6], raw_payload, payload_len);
    }

    // 6. Контрольная сумма CRC-16 (от счетчика до конца полезной нагрузки)
    size_t crc_calc_len = 4 + payload_len;
    uint16_t checksum = crc16_ccitt(&out_frame_buffer[2], crc_calc_len);

    size_t crc_offset = 6 + payload_len;
    out_frame_buffer[crc_offset] = (uint8_t)(checksum & 0xFF);
    out_frame_buffer[crc_offset + 1] = (uint8_t)((checksum >> 8) & 0xFF);

    return crc_offset + 2;
}