#include "MeshRadio.h"
#include "NodeDB.h"
#include "PortduinoGlue.h"
#include "RadioTxHook.h"
#include "Router.h"
#include "SX1302Interface.h"
#include "TestUtil.h"
#include "configuration.h"

#include <dlfcn.h>
#include <unity.h>

using namespace sx1302hal;

namespace
{

int startCalls;
int stopCalls;
int releaseCalls;
bool concentratorRunning;

int halSuccessBoard(BoardConfig *)
{
    return SUCCESS;
}
int halSuccessRf(uint8_t, RfChainConfig *)
{
    return SUCCESS;
}
int halSuccessIf(uint8_t, IfChainConfig *)
{
    return SUCCESS;
}
int halSuccessGain(uint8_t, TxGainLut *)
{
    return SUCCESS;
}
int halStart()
{
    ++startCalls;
    concentratorRunning = true;
    return SUCCESS;
}
int halStop()
{
    ++stopCalls;
    concentratorRunning = false;
    return SUCCESS;
}
int halReceiveError(uint8_t, RxPacket *)
{
    return -1;
}
int halSend(TxPacket *)
{
    return SUCCESS;
}
int halStatus(uint8_t, uint8_t, uint8_t *)
{
    return SUCCESS;
}
int halAbort(uint8_t)
{
    return SUCCESS;
}
int halSetSyncWord(uint8_t)
{
    return SUCCESS;
}

class ReconfiguringReleaseHook : public RadioTxHook
{
  public:
    void packetReleased(RadioInterface *iface, const meshtastic_MeshPacket *) override
    {
        ++releaseCalls;
        iface->reconfigure();
    }
};

} // namespace

class SX1302InterfaceTest
{
  public:
    static void installHal(SX1302Interface &radio)
    {
        radio.hal.boardSetConfig = halSuccessBoard;
        radio.hal.rfChainSetConfig = halSuccessRf;
        radio.hal.ifChainSetConfig = halSuccessIf;
        radio.hal.txGainSetConfig = halSuccessGain;
        radio.hal.start = halStart;
        radio.hal.stop = halStop;
        radio.hal.receive = halReceiveError;
        radio.hal.send = halSend;
        radio.hal.status = halStatus;
        radio.hal.abortTx = halAbort;
        radio.hal.setSyncWord = halSetSyncWord;
    }

    static void setStarted(SX1302Interface &radio, bool started) { radio.started = started; }
    static bool isStarted(const SX1302Interface &radio) { return radio.started; }
    static void setBandwidth(SX1302Interface &radio, float bandwidth) { radio.bw = bandwidth; }
    static void setFrequency(SX1302Interface &radio, float frequency) { radio.savedFreq = frequency; }
    static void setPower(SX1302Interface &radio, int8_t power) { radio.power = power; }
    static int8_t power(const SX1302Interface &radio) { return radio.power; }
    static bool isDisabled(const SX1302Interface &radio) { return radio.disabled; }
    static bool configure(SX1302Interface &radio) { return radio.configureHardware(); }
    static void applyPowerLimit(SX1302Interface &radio) { radio.applyPowerLimit(); }
    static void pollReceive(SX1302Interface &radio) { radio.pollReceive(); }
    static void ageFirstReceiveError(SX1302Interface &radio, uint32_t msec) { radio.firstReceiveErrorAt -= msec; }
    static void useProcessHandle(SX1302Interface &radio) { radio.libraryHandle = dlopen(nullptr, RTLD_NOW | RTLD_LOCAL); }
    static void enqueue(SX1302Interface &radio, meshtastic_MeshPacket *packet)
    {
        bool dropped = false;
        TEST_ASSERT_TRUE(radio.txQueue.enqueue(packet, &dropped));
        TEST_ASSERT_FALSE(dropped);
    }
};

void setUp()
{
    startCalls = 0;
    stopCalls = 0;
    releaseCalls = 0;
    concentratorRunning = false;
    portduino_status.LoRa_in_error = false;
    portduino_config.lora_reset_pin.enabled = false;
    portduino_config.lora_spi_dev = "test-spidev";
    portduino_config.sx1302_max_power = 27;
    portduino_config.sx1302_tx_gain_profile = "semtech";

    config.lora = meshtastic_Config_LoRaConfig_init_zero;
    config.lora.region = meshtastic_Config_LoRaConfig_RegionCode_US;
    config.lora.use_preset = true;
    config.lora.modem_preset = meshtastic_Config_LoRaConfig_ModemPreset_LONG_FAST;
    config.lora.tx_enabled = true;
    config.lora.tx_power = 17;
    initRegion();
}

void tearDown() {}

void test_invalid_reconfigure_stops_running_concentrator()
{
    SX1302Interface radio;
    SX1302InterfaceTest::installHal(radio);
    SX1302InterfaceTest::setStarted(radio, true);
    SX1302InterfaceTest::setBandwidth(radio, 62.5f);
    concentratorRunning = true;

    TEST_ASSERT_FALSE(SX1302InterfaceTest::configure(radio));
    TEST_ASSERT_EQUAL_INT(1, stopCalls);
    TEST_ASSERT_FALSE(SX1302InterfaceTest::isStarted(radio));
    TEST_ASSERT_FALSE(concentratorRunning);
}

void test_power_below_high_band_lut_is_clamped()
{
    SX1302Interface radio;
    SX1302InterfaceTest::setFrequency(radio, 906.0f);
    SX1302InterfaceTest::setPower(radio, 5);

    SX1302InterfaceTest::applyPowerLimit(radio);

    TEST_ASSERT_EQUAL_INT8(12, SX1302InterfaceTest::power(radio));
    TEST_ASSERT_FALSE(SX1302InterfaceTest::isDisabled(radio));
}

void test_impossible_power_ceiling_disables_transmit()
{
    SX1302Interface radio;
    SX1302InterfaceTest::setFrequency(radio, 906.0f);
    SX1302InterfaceTest::setPower(radio, 5);
    portduino_config.sx1302_max_power = 5;

    SX1302InterfaceTest::applyPowerLimit(radio);

    TEST_ASSERT_TRUE(SX1302InterfaceTest::isDisabled(radio));
}

void test_reachable_power_ceiling_reenables_transmit()
{
    SX1302Interface radio;
    SX1302InterfaceTest::setFrequency(radio, 906.0f);
    SX1302InterfaceTest::setPower(radio, 5);
    portduino_config.sx1302_max_power = 5;
    SX1302InterfaceTest::applyPowerLimit(radio);
    TEST_ASSERT_TRUE(SX1302InterfaceTest::isDisabled(radio));

    // The LUT floor is -6 below 700 MHz, so the same ceiling is fine on the low band.
    SX1302InterfaceTest::setFrequency(radio, 433.0f);
    SX1302InterfaceTest::applyPowerLimit(radio);

    TEST_ASSERT_FALSE(SX1302InterfaceTest::isDisabled(radio));
}

void test_single_receive_error_does_not_request_recovery()
{
    SX1302Interface radio;
    SX1302InterfaceTest::installHal(radio);

    SX1302InterfaceTest::pollReceive(radio);

    TEST_ASSERT_FALSE(portduino_status.LoRa_in_error);
}

void test_sustained_receive_errors_request_portduino_recovery()
{
    SX1302Interface radio;
    SX1302InterfaceTest::installHal(radio);

    SX1302InterfaceTest::pollReceive(radio);
    TEST_ASSERT_FALSE(portduino_status.LoRa_in_error);

    // Rather than sleep out the window, backdate the first failure past it.
    SX1302InterfaceTest::ageFirstReceiveError(radio, 6000);
    SX1302InterfaceTest::pollReceive(radio);

    TEST_ASSERT_TRUE(portduino_status.LoRa_in_error);
}

void test_packet_counters_are_visible_through_radio_interface()
{
    SX1302Interface radio;
    radio.rxGood = 7;
    radio.rxBad = 2;
    radio.txGood = 5;
    radio.txRelay = 3;
    radio.txDrop = 1;
    RadioInterface *iface = &radio;

    TEST_ASSERT_EQUAL_UINT32(7, iface->rxGood);
    TEST_ASSERT_EQUAL_UINT32(2, iface->rxBad);
    TEST_ASSERT_EQUAL_UINT32(5, iface->txGood);
    TEST_ASSERT_EQUAL_UINT32(3, iface->txRelay);
    TEST_ASSERT_EQUAL_UINT16(1, iface->txDrop);
}

void test_destructor_stops_after_release_hook_reconfigures()
{
    ReconfiguringReleaseHook hook;
    auto *radio = new SX1302Interface();
    SX1302InterfaceTest::installHal(*radio);
    SX1302InterfaceTest::useProcessHandle(*radio);
    SX1302InterfaceTest::setStarted(*radio, true);
    concentratorRunning = true;

    meshtastic_MeshPacket *packet = packetPool.allocZeroed();
    TEST_ASSERT_NOT_NULL(packet);
    packet->id = 0x12345678;
    SX1302InterfaceTest::enqueue(*radio, packet);

    delete radio;

    TEST_ASSERT_EQUAL_INT(1, releaseCalls);
    TEST_ASSERT_EQUAL_INT(1, startCalls);
    TEST_ASSERT_EQUAL_INT(2, stopCalls);
    TEST_ASSERT_FALSE(concentratorRunning);
}

void setup()
{
    initializeTestEnvironment();
    UNITY_BEGIN();
    RUN_TEST(test_invalid_reconfigure_stops_running_concentrator);
    RUN_TEST(test_power_below_high_band_lut_is_clamped);
    RUN_TEST(test_impossible_power_ceiling_disables_transmit);
    RUN_TEST(test_reachable_power_ceiling_reenables_transmit);
    RUN_TEST(test_single_receive_error_does_not_request_recovery);
    RUN_TEST(test_sustained_receive_errors_request_portduino_recovery);
    RUN_TEST(test_packet_counters_are_visible_through_radio_interface);
    RUN_TEST(test_destructor_stops_after_release_hook_reconfigures);
    exit(UNITY_END());
}

void loop() {}
