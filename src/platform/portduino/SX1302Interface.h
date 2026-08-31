#pragma once

#if defined(ARCH_PORTDUINO) && !defined(ARCH_PORTDUINO_WASM)

#include "MeshPacketQueue.h"
#include "RadioInterface.h"
#include "SX1302Hal.h"
#include "concurrency/OSThread.h"

class SX1302Interface : public RadioInterface, protected concurrency::OSThread
{
  public:
    SX1302Interface();
    ~SX1302Interface() override;

    bool init() override;
    bool reconfigure() override;
    bool sleep() override;
    bool canSleep(bool deepSleep = false) override;

    ErrorCode send(meshtastic_MeshPacket *p) override;
    meshtastic_QueueStatus getQueueStatus() override;
    bool cancelSending(NodeNum from, PacketId id) override;
    bool findInTxQueue(NodeNum from, PacketId id) override;
    void clampToLateRebroadcastWindow(NodeNum from, PacketId id) override;
    bool removePendingTXPacket(NodeNum from, PacketId id, uint32_t hop_limit_lt) override;

    uint32_t getPacketTime(uint32_t totalPacketLen, bool received = false) override;

    uint32_t rxBad = 0;
    uint32_t rxGood = 0;
    uint32_t txGood = 0;
    uint32_t txRelay = 0;
    uint16_t txDrop = 0;

  protected:
    int32_t runOnce() override;

  private:
    struct HalApi {
        int (*boardSetConfig)(sx1302hal::BoardConfig *);
        int (*rfChainSetConfig)(uint8_t, sx1302hal::RfChainConfig *);
        int (*ifChainSetConfig)(uint8_t, sx1302hal::IfChainConfig *);
        int (*txGainSetConfig)(uint8_t, sx1302hal::TxGainLut *);
        int (*start)();
        int (*stop)();
        int (*receive)(uint8_t, sx1302hal::RxPacket *);
        int (*send)(sx1302hal::TxPacket *);
        int (*status)(uint8_t, uint8_t, uint8_t *);
        int (*abortTx)(uint8_t);
        uint32_t (*abiVersion)();
        int (*setSyncWord)(uint8_t);
    } hal{};

    static constexpr uint8_t MESHTASTIC_SYNC_WORD = 0x2b;
    static constexpr uint32_t REQUIRED_HAL_ABI = 1;
    static constexpr uint32_t POLL_INTERVAL_MSEC = 5;

    MeshPacketQueue txQueue{MAX_TX_QUEUE};
    void *libraryHandle = nullptr;
    bool started = false;
    uint32_t nextTxAt = 0;
    uint32_t txStartedAt = 0;
    uint32_t txTimeoutMsec = 0;
    uint32_t lastReceiveErrorAt = 0;

    bool loadHal();
    void unloadHal();
    bool configureHardware();
    bool validateRadioConfig(uint8_t &halBandwidth) const;
    void applyPowerLimit();
    void resetConcentrator();
    void setTransmitDelay();
    bool startSend(meshtastic_MeshPacket *p);
    void pollTransmit();
    void completeSending(bool successful);
    void pollReceive();
    void handleReceivedPacket(const sx1302hal::RxPacket &packet);
    void releaseQueuedPacket(meshtastic_MeshPacket *p);
};

#endif
