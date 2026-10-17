#ifndef BUS_SCANNER_H
#define BUS_SCANNER_H

#include "config.h"

/// Сканирует дерево I2C шины и заполняет массив активных датчиков.
uint8_t bus_scan_and_init(ActiveNode *out_nodes, uint8_t max_capacity);

/// Выводит подробный аналитический отчет обо всех найденных мультиплексорах и датчиках.
void print_startup_diagnostics(const ActiveNode *nodes, uint8_t count);

#endif