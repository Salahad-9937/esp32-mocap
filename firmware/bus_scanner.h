#ifndef BUS_SCANNER_H
#define BUS_SCANNER_H

#include "config.h"

/// Сканирует дерево I2C шины, регистрирует активные датчики и вычисляет битовую маску найденных MUX.
uint8_t bus_scan_and_init(ActiveNode *out_nodes, uint8_t max_capacity, uint8_t *out_mux_mask);

/// Выводит подробный аналитический отчет обо всех найденных мультиплексорах и датчиках.
void print_startup_diagnostics(const ActiveNode *nodes, uint8_t count, uint8_t mux_mask);

#endif