#if defined(ARCH_PORTDUINO) && !defined(ARCH_PORTDUINO_WASM)

#include "SX1302Interface.h"

#include "PortduinoGlue.h"
#include "RadioTxHook.h"
#include "Router.h"
#include "Throttle.h"
#include "UptimeClock.h"
#include "airtime.h"
#include "configuration.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <dlfcn.h>
#include <iterator>
#include <type_traits>

using namespace sx1302hal;

SX1302Interface::SX1302Interface() : OSThread("SX1302", POLL_INTERVAL_MSEC) {}

SX1302Interface::~SX1302Interface()
{
    while (!txQueue.empty()) {
        releaseQueuedPacket(txQueue.dequeue());
    }
    sleep();
    unloadHal();
}

bool SX1302Interface::loadHal()
{
    libraryHandle = dlopen(portduino_config.sx1302_lib.c_str(), RTLD_NOW | RTLD_LOCAL);
    if (!libraryHandle) {
        LOG_ERROR("Unable to load SX1302 HAL %s: %s", portduino_config.sx1302_lib.c_str(), dlerror());
        return false;
    }

    auto loadSymbol = [this](auto &target, const char *name) {
        dlerror();
        target = reinterpret_cast<std::decay_t<decltype(target)>>(dlsym(libraryHandle, name));
        const char *error = dlerror();
        if (error || !target) {
            LOG_ERROR("SX1302 HAL is missing %s: %s", name, error ? error : "symbol not found");
            return false;
        }
        return true;
    };

    const bool loaded =
        loadSymbol(hal.boardSetConfig, "lgw_board_setconf") && loadSymbol(hal.rfChainSetConfig, "lgw_rxrf_setconf") &&
        loadSymbol(hal.ifChainSetConfig, "lgw_rxif_setconf") && loadSymbol(hal.txGainSetConfig, "lgw_txgain_setconf") &&
        loadSymbol(hal.start, "lgw_start") && loadSymbol(hal.stop, "lgw_stop") && loadSymbol(hal.receive, "lgw_receive") &&
        loadSymbol(hal.send, "lgw_send") && loadSymbol(hal.status, "lgw_status") && loadSymbol(hal.abortTx, "lgw_abort_tx") &&
        loadSymbol(hal.abiVersion, "meshtastic_sx1302_hal_abi_version") &&
        loadSymbol(hal.setSyncWord, "meshtastic_sx1302_set_syncword");
    if (!loaded) {
        unloadHal();
        return false;
    }

    const uint32_t abiVersion = hal.abiVersion();
    if (abiVersion != REQUIRED_HAL_ABI) {
        LOG_ERROR("SX1302 HAL ABI %u is incompatible; expected %u", abiVersion, REQUIRED_HAL_ABI);
        unloadHal();
        return false;
    }

    LOG_INFO("Loaded Meshtastic SX1302 HAL ABI %u from %s", abiVersion, portduino_config.sx1302_lib.c_str());
    return true;
}

void SX1302Interface::unloadHal()
{
    if (libraryHandle) {
        dlclose(libraryHandle);
        libraryHandle = nullptr;
    }
    hal = {};
}

bool SX1302Interface::validateRadioConfig(uint8_t &halBandwidth) const
{
    const int bandwidthKhz = lroundf(bw);
    switch (bandwidthKhz) {
    case 125:
        halBandwidth = BW_125KHZ;
        break;
    case 250:
        halBandwidth = BW_250KHZ;
        break;
    case 500:
        halBandwidth = BW_500KHZ;
        break;
    default:
        LOG_ERROR("SX1302 supports only 125, 250, or 500 kHz LoRa bandwidth; requested %.3f kHz", bw);
        return false;
    }

    if (sf < 5 || sf > 12) {
        LOG_ERROR("SX1302 spreading factor %u is outside the supported SF5-SF12 range", sf);
        return false;
    }
    if (cr < 5 || cr > 8) {
        LOG_ERROR("SX1302 coding rate 4/%u is outside the supported 4/5-4/8 range", cr);
        return false;
    }
    if (savedFreq >= 2400.0f) {
        LOG_ERROR("SX1302 cannot tune the %.3f MHz 2.4 GHz channel", savedFreq);
        return false;
    }
    return true;
}

void SX1302Interface::applyPowerLimit()
{
    const int configuredMax = portduino_config.sx1302_max_power;
    const int safeMax = std::clamp(configuredMax, 0, 27);
    if (safeMax != configuredMax) {
        LOG_WARN("Clamp SX1302_MAX_POWER %d to the supported 0-27 dBm range", configuredMax);
    }
    limitPower(static_cast<int8_t>(safeMax));

    const int8_t minimumPower = minimumTxPower();
    if (safeMax < minimumPower) {
        LOG_ERROR("SX1302_MAX_POWER %d dBm is below the calibrated LUT minimum %d dBm; disable transmit", safeMax, minimumPower);
        disabled = true;
        txDisabledForPower = true;
        return;
    }
    // The LUT floor moves with the band, so the ceiling that was impossible on one region can be
    // fine on the next. Undo only our own latch - disable() is a separate, deliberate one.
    if (txDisabledForPower) {
        LOG_INFO("SX1302_MAX_POWER %d dBm now clears the calibrated LUT minimum %d dBm; re-enable transmit", safeMax,
                 minimumPower);
        disabled = false;
        txDisabledForPower = false;
    }
    if (power < minimumPower) {
        LOG_WARN("Clamp SX1302 Tx power %d to calibrated LUT minimum %d dBm", power, minimumPower);
        power = minimumPower;
    }
}

int8_t SX1302Interface::minimumTxPower() const
{
    return savedFreq < 700.0f ? -6 : 12;
}

void SX1302Interface::resetConcentrator()
{
    if (!portduino_config.lora_reset_pin.enabled || portduino_config.lora_reset_pin.pin == RADIOLIB_NC) {
        LOG_WARN("SX1302 Reset GPIO is not configured; relying on an already-reset concentrator");
        return;
    }

    const int resetPin = portduino_config.lora_reset_pin.pin;
    pinMode(resetPin, OUTPUT);
    digitalWrite(resetPin, HIGH);
    delay(100);
    digitalWrite(resetPin, LOW);
    delay(100);
}

bool SX1302Interface::configureHardware()
{
    if (started) {
        if (sendingPacket) {
            hal.abortTx(0);
            completeSending(false);
        }
        hal.stop();
        started = false;
    }

    uint8_t configuredBandwidth = 0;
    if (!validateRadioConfig(configuredBandwidth)) {
        return false;
    }

    resetConcentrator();

    BoardConfig board{};
    board.lorawan_public = false;
    board.clksrc = 0;
    board.full_duplex = false;
    board.com_type = ComType::SPI;
    if (portduino_config.lora_spi_dev.size() >= sizeof(board.com_path)) {
        LOG_ERROR("SX1302 SPI path is too long: %s", portduino_config.lora_spi_dev.c_str());
        return false;
    }
    strncpy(board.com_path, portduino_config.lora_spi_dev.c_str(), sizeof(board.com_path) - 1);
    if (hal.boardSetConfig(&board) != SUCCESS) {
        LOG_ERROR("lgw_board_setconf failed");
        return false;
    }

    const uint32_t frequencyHz = lroundf(savedFreq * 1000000.0f);
    for (uint8_t chain = 0; chain < RF_CHAIN_COUNT; ++chain) {
        RfChainConfig rf{};
        rf.enable = true;
        rf.freq_hz = frequencyHz;
        rf.rssi_offset = frequencyHz < 700000000 ? -207.0f : -215.4f;
        rf.rssi_tcomp = {0.0f, 0.0f, 20.41f, 2162.56f, 0.0f};
        rf.type = RadioType::SX1250;
        rf.tx_enable = chain == 0;
        rf.single_input_mode = frequencyHz < 700000000;
        if (hal.rfChainSetConfig(chain, &rf) != SUCCESS) {
            LOG_ERROR("lgw_rxrf_setconf failed for RF chain %u", chain);
            return false;
        }
    }

    for (uint8_t chain = 0; chain < IF_CHAIN_COUNT; ++chain) {
        IfChainConfig disabledIf{};
        if (hal.ifChainSetConfig(chain, &disabledIf) != SUCCESS) {
            LOG_ERROR("Unable to disable SX1302 IF chain %u", chain);
            return false;
        }
    }

    IfChainConfig serviceIf{};
    serviceIf.enable = true;
    serviceIf.rf_chain = 0;
    serviceIf.freq_hz = 0;
    serviceIf.bandwidth = configuredBandwidth;
    serviceIf.datarate = sf;
    serviceIf.implicit_hdr = false;
    if (hal.ifChainSetConfig(8, &serviceIf) != SUCCESS) {
        LOG_ERROR("Unable to configure SX1302 single-SF service channel");
        return false;
    }

    // The PA is outside the SX1250, so its calibrated curve belongs to the
    // concentrator board. These values come from the vendors' packet-forwarder
    // configurations. mix_gain is unused for SX1250 but the HAL still validates
    // it and requires a value in the SX1257 range.
    static constexpr TxGain semtechGain[] = {
        {12, 0, 0, 0, 5, 0, 0, 15}, {13, 0, 0, 0, 5, 0, 0, 16}, {14, 0, 0, 0, 5, 0, 0, 17}, {15, 0, 0, 0, 5, 0, 0, 19},
        {16, 0, 0, 0, 5, 0, 0, 20}, {17, 0, 0, 0, 5, 0, 0, 22}, {18, 0, 1, 0, 5, 0, 0, 1},  {19, 0, 1, 0, 5, 0, 0, 2},
        {20, 0, 1, 0, 5, 0, 0, 3},  {21, 0, 1, 0, 5, 0, 0, 4},  {22, 0, 1, 0, 5, 0, 0, 5},  {23, 0, 1, 0, 5, 0, 0, 6},
        {24, 0, 1, 0, 5, 0, 0, 7},  {25, 0, 1, 0, 5, 0, 0, 9},  {26, 0, 1, 0, 5, 0, 0, 11}, {27, 0, 1, 0, 5, 0, 0, 14},
    };
    static constexpr TxGain rakLowBandGain[] = {
        {-6, 0, 0, 0, 5, 0, 0, 0},  {-3, 0, 0, 0, 5, 0, 0, 1},  {0, 0, 0, 0, 5, 0, 0, 2},   {3, 0, 1, 0, 5, 0, 0, 3},
        {6, 0, 1, 0, 5, 0, 0, 4},   {10, 0, 1, 0, 5, 0, 0, 5},  {11, 0, 1, 0, 5, 0, 0, 6},  {12, 0, 1, 0, 5, 0, 0, 7},
        {13, 0, 1, 0, 5, 0, 0, 8},  {14, 0, 1, 0, 5, 0, 0, 9},  {16, 0, 1, 0, 5, 0, 0, 10}, {20, 0, 1, 0, 5, 0, 0, 11},
        {23, 0, 1, 0, 5, 0, 0, 12}, {25, 0, 1, 0, 5, 0, 0, 13}, {26, 0, 1, 0, 5, 0, 0, 14}, {27, 0, 1, 0, 5, 0, 0, 15},
    };
    static constexpr TxGain rak868Gain[] = {
        {12, 0, 1, 0, 5, 0, 0, 4},  {13, 0, 1, 0, 5, 0, 0, 5},  {14, 0, 1, 0, 5, 0, 0, 6},  {15, 0, 1, 0, 5, 0, 0, 7},
        {16, 0, 1, 0, 5, 0, 0, 8},  {17, 0, 1, 0, 5, 0, 0, 9},  {18, 0, 1, 0, 5, 0, 0, 10}, {19, 0, 1, 0, 5, 0, 0, 11},
        {20, 0, 1, 0, 5, 0, 0, 12}, {21, 0, 1, 0, 5, 0, 0, 13}, {22, 0, 1, 0, 5, 0, 0, 14}, {23, 0, 1, 0, 5, 0, 0, 16},
        {24, 0, 1, 0, 5, 0, 0, 17}, {25, 0, 1, 0, 5, 0, 0, 18}, {26, 0, 1, 0, 5, 0, 0, 19}, {27, 0, 1, 0, 5, 0, 0, 22},
    };
    static constexpr TxGain rak900Gain[] = {
        {12, 0, 1, 0, 5, 0, 0, 6},  {13, 0, 1, 0, 5, 0, 0, 7},  {14, 0, 1, 0, 5, 0, 0, 8},  {15, 0, 1, 0, 5, 0, 0, 9},
        {16, 0, 1, 0, 5, 0, 0, 10}, {17, 0, 1, 0, 5, 0, 0, 11}, {18, 0, 1, 0, 5, 0, 0, 12}, {19, 0, 1, 0, 5, 0, 0, 13},
        {20, 0, 1, 0, 5, 0, 0, 14}, {21, 0, 1, 0, 5, 0, 0, 15}, {22, 0, 1, 0, 5, 0, 0, 16}, {23, 0, 1, 0, 5, 0, 0, 17},
        {24, 0, 1, 0, 5, 0, 0, 18}, {25, 0, 1, 0, 5, 0, 0, 19}, {26, 0, 1, 0, 5, 0, 0, 21}, {27, 0, 1, 0, 5, 0, 0, 22},
    };
    static_assert(std::size(semtechGain) == TX_GAIN_LUT_SIZE_MAX);
    static_assert(std::size(rakLowBandGain) == TX_GAIN_LUT_SIZE_MAX);
    static_assert(std::size(rak868Gain) == TX_GAIN_LUT_SIZE_MAX);
    static_assert(std::size(rak900Gain) == TX_GAIN_LUT_SIZE_MAX);

    const TxGain *referenceGain = semtechGain;
    if (portduino_config.sx1302_tx_gain_profile == "rak") {
        referenceGain = frequencyHz < 700000000 ? rakLowBandGain : (frequencyHz < 900000000 ? rak868Gain : rak900Gain);
    } else if (frequencyHz < 700000000) {
        referenceGain = rakLowBandGain;
    }
    TxGainLut gainLut{};
    memcpy(gainLut.lut, referenceGain, sizeof(gainLut.lut));
    gainLut.size = TX_GAIN_LUT_SIZE_MAX;
    if (hal.txGainSetConfig(0, &gainLut) != SUCCESS) {
        LOG_ERROR("lgw_txgain_setconf failed");
        return false;
    }
    LOG_INFO("Using SX1302 TX gain profile %s", portduino_config.sx1302_tx_gain_profile.c_str());

    if (hal.start() != SUCCESS) {
        LOG_ERROR("lgw_start failed for SX1302 on %s", board.com_path);
        return false;
    }
    started = true;

    if (hal.setSyncWord(MESHTASTIC_SYNC_WORD) != SUCCESS) {
        LOG_ERROR("Unable to configure Meshtastic sync word 0x%02x", MESHTASTIC_SYNC_WORD);
        hal.stop();
        started = false;
        return false;
    }

    halBandwidth = configuredBandwidth;
    nextTxAt = 0;
    LOG_INFO("SX1302 started at %.3f MHz, BW %d kHz, SF%u, CR 4/%u, power %d dBm", savedFreq, lroundf(bw), sf, cr, power);
    LOG_INFO("SX1302 HAL has no packet-in-progress CAD API; using Meshtastic randomized transmit scheduling");
    return true;
}

bool SX1302Interface::init()
{
    if (!RadioInterface::init()) {
        return false;
    }
    applyPowerLimit();
    return loadHal() && configureHardware();
}

bool SX1302Interface::reconfigure()
{
    if (!RadioInterface::reconfigure()) {
        return false;
    }
    applyPowerLimit();
    return libraryHandle && configureHardware();
}

bool SX1302Interface::sleep()
{
    if (started) {
        if (sendingPacket) {
            hal.abortTx(0);
            completeSending(false);
        }
        const int result = hal.stop();
        started = false;
        return result == SUCCESS;
    }
    return true;
}

bool SX1302Interface::canSleep(bool deepSleep)
{
    (void)deepSleep;
    return txQueue.empty() && sendingPacket == nullptr;
}

ErrorCode SX1302Interface::send(meshtastic_MeshPacket *p)
{
#ifndef DISABLE_WELCOME_UNSET
    if (config.lora.region == meshtastic_Config_LoRaConfig_RegionCode_UNSET) {
        LOG_WARN("send - lora tx disabled: Region unset");
        packetPool.release(p);
        return ERRNO_DISABLED;
    }
#endif
    if (disabled || !config.lora.tx_enabled || !started) {
        LOG_WARN("send - SX1302 transmitter disabled");
        packetPool.release(p);
        return ERRNO_DISABLED;
    }
    if (p->to == NODENUM_BROADCAST_NO_LORA) {
        return ERRNO_SHOULD_RELEASE;
    }

#ifndef LORA_DISABLE_SENDING
    printPacket("enqueue for SX1302 send", p);
    bool dropped = false;
    const ErrorCode result = txQueue.enqueue(p, &dropped) ? ERRNO_OK : ERRNO_UNKNOWN;
    if (dropped) {
        ++txDrop;
    }
    if (result != ERRNO_OK) {
        packetPool.release(p);
        return result;
    }
    setTransmitDelay();
    return result;
#else
    packetPool.release(p);
    return ERRNO_DISABLED;
#endif
}

meshtastic_QueueStatus SX1302Interface::getQueueStatus()
{
    meshtastic_QueueStatus status{};
    status.free = txQueue.getFree();
    status.maxlen = txQueue.getMaxLen();
    return status;
}

void SX1302Interface::releaseQueuedPacket(meshtastic_MeshPacket *p)
{
    if (p) {
        RadioTxHooks::packetReleased(this, p);
        packetPool.release(p);
    }
}

bool SX1302Interface::cancelSending(NodeNum from, PacketId id)
{
    meshtastic_MeshPacket *packet = txQueue.remove(from, id);
    releaseQueuedPacket(packet);
    return packet != nullptr;
}

bool SX1302Interface::findInTxQueue(NodeNum from, PacketId id)
{
    return txQueue.find(from, id);
}

void SX1302Interface::clampToLateRebroadcastWindow(NodeNum from, PacketId id)
{
    meshtastic_MeshPacket *packet = txQueue.remove(from, id, true, false);
    if (!packet) {
        return;
    }
    packet->tx_after = Time::getMillis() + getTxDelayMsecWeightedWorst(packet->rx_snr);
    bool dropped = false;
    if (!txQueue.enqueue(packet, &dropped)) {
        releaseQueuedPacket(packet);
    }
    if (dropped) {
        ++txDrop;
    }
    setTransmitDelay();
}

bool SX1302Interface::removePendingTXPacket(NodeNum from, PacketId id, uint32_t hop_limit_lt)
{
    meshtastic_MeshPacket *packet = txQueue.remove(from, id, true, true, hop_limit_lt);
    releaseQueuedPacket(packet);
    return packet != nullptr;
}

void SX1302Interface::setTransmitDelay()
{
    meshtastic_MeshPacket *packet = txQueue.getFront();
    if (!packet) {
        nextTxAt = 0;
        return;
    }

    const uint32_t now = Time::getMillis();
    if (packet->tx_after) {
        const uint32_t addedDelay = packet->has_rx_rssi ? getTxDelayMsecWeighted(packet) : getTxDelayMsec();
        const uint32_t earliest = now + addedDelay;
        const uint32_t latest = now + 2 * getTxDelayMsecWeightedWorst(packet->rx_snr);
        packet->tx_after = std::min(std::max(packet->tx_after + addedDelay, earliest), latest);
        nextTxAt = packet->tx_after;
    } else {
        const uint32_t delay = (packet->rx_snr == 0 && !packet->has_rx_rssi) ? getTxDelayMsec() : getTxDelayMsecWeighted(packet);
        nextTxAt = now + std::max(delay, 1U);
    }
}

bool SX1302Interface::startSend(meshtastic_MeshPacket *packet)
{
    if (disabled || !config.lora.tx_enabled || !started) {
        LOG_WARN("Drop SX1302 packet: transmitter disabled");
        releaseQueuedPacket(packet);
        ++txDrop;
        return false;
    }

    const size_t packetSize = beginSending(packet);
    TxPacket tx{};
    tx.freq_hz = lroundf(savedFreq * 1000000.0f);
    tx.tx_mode = IMMEDIATE;
    tx.rf_chain = 0;
    tx.rf_power = power;
    tx.modulation = MOD_LORA;
    tx.bandwidth = halBandwidth;
    tx.datarate = sf;
    tx.coderate = cr - 4;
    tx.invert_pol = false;
    tx.preamble = preambleLength;
    tx.no_crc = false;
    tx.no_header = false;
    tx.size = packetSize;
    memcpy(tx.payload, &radioBuffer, packetSize);

    if (hal.send(&tx) != SUCCESS) {
        LOG_ERROR("lgw_send failed for packet 0x%08x", packet->id);
        completeSending(false);
        return false;
    }

    txStartedAt = Time::getMillis();
    lastTxStart = txStartedAt;
    txTimeoutMsec = RadioInterface::getPacketTime(packet) + 1500;
    printPacket("Started SX1302 Tx", packet);
    return true;
}

void SX1302Interface::completeSending(bool successful)
{
    meshtastic_MeshPacket *packet = sendingPacket;
    sendingPacket = nullptr;
    txStartedAt = 0;
    txTimeoutMsec = 0;
    if (!packet) {
        return;
    }

    if (successful) {
        airTime->logAirtime(TX_LOG, RadioInterface::getPacketTime(packet));
        ++txGood;
        if (!isFromUs(packet)) {
            ++txRelay;
        }
        printPacket("Completed SX1302 Tx", packet);
    } else {
        ++txDrop;
        printPacket("Failed SX1302 Tx", packet);
    }
    releaseQueuedPacket(packet);
    setTransmitDelay();
}

void SX1302Interface::pollTransmit()
{
    if (!sendingPacket) {
        return;
    }

    uint8_t status = 0;
    if (hal.status(0, TX_STATUS, &status) == SUCCESS) {
        if (status == TX_FREE) {
            completeSending(true);
            return;
        }
        if (status == TX_OFF) {
            LOG_ERROR("SX1302 transmitter switched off during packet 0x%08x", sendingPacket->id);
            completeSending(false);
            return;
        }
        if (status != TX_SCHEDULED && status != TX_EMITTING) {
            LOG_WARN("Unexpected SX1302 TX status %u", status);
        }
    }

    if (txStartedAt && Throttle::hasElapsed(txStartedAt, txTimeoutMsec)) {
        LOG_ERROR("SX1302 transmit timeout for packet 0x%08x", sendingPacket->id);
        hal.abortTx(0);
        completeSending(false);
    }
}

void SX1302Interface::handleReceivedPacket(const RxPacket &packet)
{
    const uint32_t airtimeMsec = getPacketTime(std::min<uint16_t>(packet.size, MAX_LORA_PAYLOAD_LEN), true);
    if (packet.status != STAT_CRC_OK) {
        ++rxBad;
        airTime->logAirtime(RX_ALL_LOG, airtimeMsec);
        return;
    }
    if (packet.modulation != MOD_LORA || packet.size < sizeof(PacketHeader) || packet.size > MAX_LORA_PAYLOAD_LEN) {
        LOG_WARN("Ignore invalid SX1302 packet: modulation=%u size=%u", packet.modulation, packet.size);
        ++rxBad;
        airTime->logAirtime(RX_ALL_LOG, airtimeMsec);
        return;
    }

#ifndef DISABLE_WELCOME_UNSET
    if (config.lora.region == meshtastic_Config_LoRaConfig_RegionCode_UNSET) {
        LOG_WARN("lora rx disabled: Region unset");
        airTime->logAirtime(RX_ALL_LOG, airtimeMsec);
        return;
    }
#endif

    memcpy(&radioBuffer, packet.payload, packet.size);
    if (radioBuffer.header.from == 0) {
        LOG_WARN("Ignore SX1302 packet without sender");
        ++rxBad;
        airTime->logAirtime(RX_ALL_LOG, airtimeMsec);
        return;
    }

    meshtastic_MeshPacket *meshPacket = packetPool.allocZeroed();
    if (!meshPacket) {
        airTime->logAirtime(RX_LOG, airtimeMsec);
        return;
    }

    meshPacket->from = radioBuffer.header.from;
    meshPacket->to = radioBuffer.header.to;
    meshPacket->id = radioBuffer.header.id;
    meshPacket->channel = radioBuffer.header.channel;
    meshPacket->hop_limit = radioBuffer.header.flags & PACKET_FLAGS_HOP_LIMIT_MASK;
    meshPacket->hop_start = (radioBuffer.header.flags & PACKET_FLAGS_HOP_START_MASK) >> PACKET_FLAGS_HOP_START_SHIFT;
    meshPacket->want_ack = !!(radioBuffer.header.flags & PACKET_FLAGS_WANT_ACK_MASK);
    meshPacket->via_mqtt = !!(radioBuffer.header.flags & PACKET_FLAGS_VIA_MQTT_MASK);
    meshPacket->next_hop = meshPacket->hop_start == 0 ? NO_NEXT_HOP_PREFERENCE : radioBuffer.header.next_hop;
    meshPacket->relay_node = meshPacket->hop_start == 0 ? NO_RELAY_NODE : radioBuffer.header.relay_node;
    meshPacket->rx_snr = packet.snr;
    meshPacket->rx_rssi = lroundf(packet.rssic);
    meshPacket->has_rx_rssi = true;
    meshPacket->which_payload_variant = meshtastic_MeshPacket_encrypted_tag;
    meshPacket->encrypted.size = packet.size - sizeof(PacketHeader);
    memcpy(meshPacket->encrypted.bytes, radioBuffer.payload, meshPacket->encrypted.size);

    ++rxGood;
    printPacket("SX1302 RX", meshPacket);
    loraRxPacketObservable.notifyObservers(meshPacket->from);
    airTime->logAirtime(RX_LOG, airtimeMsec);
    deliverToReceiver(meshPacket);
}

void SX1302Interface::pollReceive()
{
    static constexpr uint8_t MAX_PACKETS_PER_POLL = 8;
    RxPacket packets[MAX_PACKETS_PER_POLL]{};
    const int count = hal.receive(MAX_PACKETS_PER_POLL, packets);
    if (count < 0) {
        const uint32_t now = Time::getMillis();
        if (!lastReceiveErrorAt || Throttle::hasElapsed(lastReceiveErrorAt, 5000)) {
            LOG_WARN("lgw_receive failed; will keep polling");
            lastReceiveErrorAt = now;
        }
        // Recovery tears the interface down and re-runs initLoRa(), which for this driver exits the
        // process if it fails. A single failed SPI read must not be worth that, so only ask for it
        // once the concentrator has been failing without interruption for RECOVERY_AFTER_MSEC.
        static constexpr uint32_t RECOVERY_AFTER_MSEC = 5000;
        if (++consecutiveReceiveErrors == 1) {
            firstReceiveErrorAt = now;
        } else if (!portduino_status.LoRa_in_error && Throttle::hasElapsed(firstReceiveErrorAt, RECOVERY_AFTER_MSEC)) {
            LOG_ERROR("lgw_receive has failed for %ums; request LoRa recovery", now - firstReceiveErrorAt);
            portduino_status.LoRa_in_error = true;
        }
        return;
    }
    consecutiveReceiveErrors = 0;
    for (int i = 0; i < count; ++i) {
        handleReceivedPacket(packets[i]);
    }
}

int32_t SX1302Interface::runOnce()
{
    if (!started) {
        return 1000;
    }

    pollReceive();
    pollTransmit();

    if (!sendingPacket && !txQueue.empty()) {
        if (!nextTxAt) {
            setTransmitDelay();
        }
        if (nextTxAt && Throttle::deadlinePassed(nextTxAt)) {
            meshtastic_MeshPacket *packet = txQueue.getFront();
            const RadioTxHook::PreTxAction action = RadioTxHooks::beforeTransmit(this, packet);
            if (action == RadioTxHook::PRETX_DROP) {
                releaseQueuedPacket(txQueue.dequeue());
                ++txDrop;
                setTransmitDelay();
            } else if (action == RadioTxHook::PRETX_DEFER) {
                setTransmitDelay();
            } else {
                nextTxAt = 0;
                startSend(txQueue.dequeue());
            }
        }
    }

    return POLL_INTERVAL_MSEC;
}

uint32_t SX1302Interface::getPacketTime(uint32_t packetLength, bool received)
{
    (void)received;
    const float bandwidthHz = bw * 1000.0f;
    const float symbolTime = (1U << sf) / bandwidthHz;
    const bool lowDataRateOptimize = symbolTime > 16e-3f;
    const float preambleTime = (preambleLength + 4.25f) * symbolTime;
    const float payloadSymbols =
        8 + std::max(ceilf(((8.0f * packetLength - 4 * sf + 28 + 16) / (4 * (sf - 2 * lowDataRateOptimize))) * cr), 0.0f);
    return (preambleTime + payloadSymbols * symbolTime) * 1000;
}

#endif
