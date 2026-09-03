#include "SX1302Eui.h"

bool sx1302EuiToMac(uint64_t eui, uint8_t *mac)
{
    if (eui == 0 || eui == UINT64_MAX)
        return false;

    for (int i = 5; i >= 0; --i) {
        mac[i] = static_cast<uint8_t>(eui);
        eui >>= 8;
    }
    mac[0] = (mac[0] & 0xfc) | 0x02;
    return true;
}

#ifdef PORTDUINO_LINUX_HARDWARE

#include "PortduinoGlue.h"
#include "SX1302Hal.h"

#include <cstdio>
#include <cstring>
#include <dlfcn.h>

using namespace sx1302hal;

namespace
{

struct EuiHalApi {
    int (*boardSetConfig)(BoardConfig *);
    int (*rfChainSetConfig)(uint8_t, RfChainConfig *);
    int (*start)();
    int (*stop)();
    int (*getEui)(uint64_t *);
    uint32_t (*abiVersion)();
};

template <typename T> bool loadSymbol(void *handle, T &target, const char *name)
{
    dlerror();
    target = reinterpret_cast<T>(dlsym(handle, name));
    return target != nullptr && dlerror() == nullptr;
}

} // namespace

bool getSX1302EuiMac(uint8_t *mac)
{
    if (portduino_config.lora_module != use_sx1302 || portduino_config.lora_spi_dev.empty())
        return false;

    void *handle = dlopen(portduino_config.sx1302_lib.c_str(), RTLD_NOW | RTLD_LOCAL);
    if (handle == nullptr) {
        std::fprintf(stderr, "Unable to load SX1302 HAL for EUI: %s\n", dlerror());
        return false;
    }

    EuiHalApi hal{};
    const bool loaded = loadSymbol(handle, hal.boardSetConfig, "lgw_board_setconf") &&
                        loadSymbol(handle, hal.rfChainSetConfig, "lgw_rxrf_setconf") &&
                        loadSymbol(handle, hal.start, "lgw_start") && loadSymbol(handle, hal.stop, "lgw_stop") &&
                        loadSymbol(handle, hal.getEui, "lgw_get_eui") &&
                        loadSymbol(handle, hal.abiVersion, "meshtastic_sx1302_hal_abi_version");
    if (!loaded || hal.abiVersion() != 1) {
        std::fprintf(stderr, "SX1302 HAL cannot provide a compatible EUI probe\n");
        dlclose(handle);
        return false;
    }

    if (portduino_config.lora_reset_pin.enabled) {
        pinMode(portduino_config.lora_reset_pin.pin, OUTPUT);
        digitalWrite(portduino_config.lora_reset_pin.pin, HIGH);
        delay(100);
        digitalWrite(portduino_config.lora_reset_pin.pin, LOW);
        delay(100);
    }

    BoardConfig board{};
    board.lorawan_public = false;
    board.clksrc = 0;
    board.full_duplex = false;
    board.com_type = ComType::SPI;
    if (portduino_config.lora_spi_dev.size() >= sizeof(board.com_path)) {
        dlclose(handle);
        return false;
    }
    std::strncpy(board.com_path, portduino_config.lora_spi_dev.c_str(), sizeof(board.com_path) - 1);

    RfChainConfig rf{};
    rf.enable = true;
    rf.freq_hz = 868500000;
    rf.type = RadioType::SX1250;
    rf.tx_enable = false;

    uint64_t eui = 0;
    bool started = false;
    bool success = hal.boardSetConfig(&board) == SUCCESS && hal.rfChainSetConfig(0, &rf) == SUCCESS;
    rf.enable = false;
    success = success && hal.rfChainSetConfig(1, &rf) == SUCCESS;
    if (success) {
        started = hal.start() == SUCCESS;
        success = started && hal.getEui(&eui) == SUCCESS;
    }
    if (started)
        hal.stop();
    dlclose(handle);

    if (!success || !sx1302EuiToMac(eui, mac))
        return false;

    std::printf("Using SX1302 EUI %016llX as MAC\n", static_cast<unsigned long long>(eui));
    return true;
}

#else

bool getSX1302EuiMac(uint8_t *)
{
    return false;
}

#endif
