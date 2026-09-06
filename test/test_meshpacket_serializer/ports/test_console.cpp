#include "../test_helpers.h"

void test_console_text_summary()
{
    const char *text = "Hello Meshtastic!";
    meshtastic_MeshPacket packet =
        create_test_packet(meshtastic_PortNum_TEXT_MESSAGE_APP, reinterpret_cast<const uint8_t *>(text), strlen(text));

    const std::string line = MeshPacketSerializer::ConsoleSerialize(&packet, false);

    TEST_ASSERT_TRUE(line.find(">> PKT") != std::string::npos);
    TEST_ASSERT_TRUE(line.find("11223344 -> 55667788") != std::string::npos);
    TEST_ASSERT_TRUE(line.find("TEXT") != std::string::npos);
    TEST_ASSERT_TRUE(line.find("rssi  -85.0 #####.....") != std::string::npos);
    TEST_ASSERT_TRUE(line.find("snr  10.5") != std::string::npos);
    TEST_ASSERT_TRUE(line.find("port=1 \"Hello Meshtastic!\"") != std::string::npos);
}

void test_console_encrypted_summary()
{
    meshtastic_MeshPacket packet = meshtastic_MeshPacket_init_zero;
    packet.from = 0x11223344;
    packet.to = 0xffffffff;
    packet.channel = 0xa7;
    packet.which_payload_variant = meshtastic_MeshPacket_encrypted_tag;
    packet.encrypted.size = 26;
    packet.rx_rssi = -104;
    packet.rx_snr = -6.0f;

    const std::string line = MeshPacketSerializer::ConsoleSerialize(&packet, false);

    TEST_ASSERT_TRUE(line.find("11223344 -> ffffffff") != std::string::npos);
    TEST_ASSERT_TRUE(line.find("ENCRYPTED") != std::string::npos);
    TEST_ASSERT_TRUE(line.find("rssi -104.0 ##........") != std::string::npos);
    TEST_ASSERT_TRUE(line.find("snr  -6.0") != std::string::npos);
    TEST_ASSERT_TRUE(line.find("26 bytes ch=0xa7") != std::string::npos);
}

void test_console_tx_summary_has_no_rx_signal()
{
    const char *text = "outbound";
    meshtastic_MeshPacket packet =
        create_test_packet(meshtastic_PortNum_TEXT_MESSAGE_APP, reinterpret_cast<const uint8_t *>(text), strlen(text));

    const std::string line = MeshPacketSerializer::ConsoleSerialize(&packet, false, "<<", false);

    TEST_ASSERT_TRUE(line.find("<< PKT") != std::string::npos);
    TEST_ASSERT_TRUE(line.find("rssi     -- ..........") != std::string::npos);
    TEST_ASSERT_TRUE(line.find("snr    --") != std::string::npos);
    TEST_ASSERT_TRUE(line.find("\"outbound\"") != std::string::npos);
}

void test_console_rx_summary_includes_zero_snr()
{
    meshtastic_MeshPacket packet = meshtastic_MeshPacket_init_zero;
    packet.from = 0x11223344;
    packet.to = 0xffffffff;
    packet.which_payload_variant = meshtastic_MeshPacket_encrypted_tag;
    packet.rx_rssi = -90;
    packet.rx_snr = 0.0f;

    const std::string line = MeshPacketSerializer::ConsoleSerialize(&packet, false);

    TEST_ASSERT_TRUE(line.find("snr   0.0") != std::string::npos);
}

void test_console_summary_strips_payload_controls()
{
    const uint8_t text[] = {'o', 'k', 0x1b, '[', '3', '1', 'm'};
    meshtastic_MeshPacket packet = create_test_packet(meshtastic_PortNum_TEXT_MESSAGE_APP, text, sizeof(text));

    const std::string line = MeshPacketSerializer::ConsoleSerialize(&packet, false);

    TEST_ASSERT_TRUE(line.find('\x1b') == std::string::npos);
    TEST_ASSERT_TRUE(line.find("ok [31m") != std::string::npos);
    TEST_ASSERT_TRUE(MeshPacketSerializer::ConsoleSerialize(&packet, true).find('\x1b') != std::string::npos);
}
