#pragma once

#include <cstdint>

bool sx1302EuiToMac(uint64_t eui, uint8_t *mac);
bool getSX1302EuiMac(uint8_t *mac);
