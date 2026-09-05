// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2019 Semtech Corporation
//
// ABI declarations derived from Lora-net/sx1302_hal loragw_hal.h.  Keeping the
// small ABI surface here lets meshtasticd load libloragw at runtime without
// making the Semtech HAL a firmware build dependency.

#pragma once

#include <stdbool.h>
#include <stdint.h>

namespace sx1302hal
{

constexpr int SUCCESS = 0;
constexpr uint8_t MOD_LORA = 0x10;
constexpr uint8_t BW_125KHZ = 0x04;
constexpr uint8_t BW_250KHZ = 0x05;
constexpr uint8_t BW_500KHZ = 0x06;
constexpr uint8_t STAT_CRC_OK = 0x10;
constexpr uint8_t IMMEDIATE = 0;
constexpr uint8_t TX_STATUS = 1;
constexpr uint8_t TX_OFF = 1;
constexpr uint8_t TX_FREE = 2;
constexpr uint8_t TX_SCHEDULED = 3;
constexpr uint8_t TX_EMITTING = 4;
constexpr uint8_t TX_GAIN_LUT_SIZE_MAX = 16;
constexpr uint8_t IF_CHAIN_COUNT = 10;
constexpr uint8_t RF_CHAIN_COUNT = 2;
constexpr uint8_t RADIO_TYPE_SX1250 = 5;
constexpr uint8_t COM_SPI = 0;

enum class ComType : int { SPI = COM_SPI, USB, UNKNOWN };
enum class RadioType : int { NONE, SX1255, SX1257, SX1272, SX1276, SX1250 };

struct BoardConfig {
    bool lorawan_public;
    uint8_t clksrc;
    bool full_duplex;
    ComType com_type;
    char com_path[64];
};

struct RssiTempCompensation {
    float coeff_a;
    float coeff_b;
    float coeff_c;
    float coeff_d;
    float coeff_e;
};

struct RfChainConfig {
    bool enable;
    uint32_t freq_hz;
    float rssi_offset;
    RssiTempCompensation rssi_tcomp;
    RadioType type;
    bool tx_enable;
    bool single_input_mode;
};

struct IfChainConfig {
    bool enable;
    uint8_t rf_chain;
    int32_t freq_hz;
    uint8_t bandwidth;
    uint32_t datarate;
    uint8_t sync_word_size;
    uint64_t sync_word;
    bool implicit_hdr;
    uint8_t implicit_payload_length;
    bool implicit_crc_en;
    uint8_t implicit_coderate;
};

struct RxPacket {
    uint32_t freq_hz;
    int32_t freq_offset;
    uint8_t if_chain;
    uint8_t status;
    uint32_t count_us;
    uint8_t rf_chain;
    uint8_t modem_id;
    uint8_t modulation;
    uint8_t bandwidth;
    uint32_t datarate;
    uint8_t coderate;
    float rssic;
    float rssis;
    float snr;
    float snr_min;
    float snr_max;
    uint16_t crc;
    uint16_t size;
    uint8_t payload[256];
    bool ftime_received;
    uint32_t ftime;
};

struct TxPacket {
    uint32_t freq_hz;
    uint8_t tx_mode;
    uint32_t count_us;
    uint8_t rf_chain;
    int8_t rf_power;
    uint8_t modulation;
    int8_t freq_offset;
    uint8_t bandwidth;
    uint32_t datarate;
    uint8_t coderate;
    bool invert_pol;
    uint8_t f_dev;
    uint16_t preamble;
    bool no_crc;
    bool no_header;
    uint16_t size;
    uint8_t payload[256];
};

struct TxGain {
    int8_t rf_power;
    uint8_t dig_gain;
    uint8_t pa_gain;
    uint8_t dac_gain;
    uint8_t mix_gain;
    int8_t offset_i;
    int8_t offset_q;
    uint8_t pwr_idx;
};

struct TxGainLut {
    TxGain lut[TX_GAIN_LUT_SIZE_MAX];
    uint8_t size;
};

static_assert(sizeof(BoardConfig) == 72, "Semtech board-config ABI changed");
static_assert(sizeof(RfChainConfig) == 40, "Semtech RF-chain ABI changed");
static_assert(sizeof(IfChainConfig) == 40, "Semtech IF-chain ABI changed");
static_assert(sizeof(RxPacket) == 316, "Semtech RX-packet ABI changed");
static_assert(sizeof(TxPacket) == 292, "Semtech TX-packet ABI changed");
static_assert(sizeof(TxGain) == 8, "Semtech TX-gain ABI changed");
static_assert(sizeof(TxGainLut) == 129, "Semtech TX-gain LUT ABI changed");

} // namespace sx1302hal
