#ifndef NRF52_USE_JSON
#include "MeshPacketSerializer.h"
#include "JSON.h"
#include "NodeDB.h"
#include "mesh/generated/meshtastic/mqtt.pb.h"
#include "mesh/generated/meshtastic/telemetry.pb.h"
#include "modules/RoutingModule.h"
#include <DebugConfiguration.h>
#include <algorithm>
#include <cctype>
#include <cmath>
#include <iomanip>
#include <memory>
#include <mesh-pb-constants.h>
#include <sstream>
#if defined(ARCH_ESP32)
#include "../mesh/generated/meshtastic/paxcount.pb.h"
#endif
#include "mesh/generated/meshtastic/remote_hardware.pb.h"
#include <sys/types.h>

static const char *errStr = "Error decoding proto for %s message!";

static std::unique_ptr<JSONValue> tryParseJson(const char *s)
{
    return std::unique_ptr<JSONValue>(JSON::Parse(s));
}

namespace
{
constexpr const char *ANSI_RESET = "\033[0m";
constexpr const char *ANSI_BOLD = "\033[1m";
constexpr const char *ANSI_DIM = "\033[2m";
constexpr const char *ANSI_RED = "\033[31m";
constexpr const char *ANSI_GREEN = "\033[32m";
constexpr const char *ANSI_YELLOW = "\033[33m";
constexpr const char *ANSI_BLUE = "\033[34m";
constexpr const char *ANSI_WHITE = "\033[37m";
constexpr const char *ANSI_BRIGHT_GREEN = "\033[92m";

std::string styledText(const char *color, const std::string &text, bool styled)
{
    return styled ? std::string(color) + text + ANSI_RESET : text;
}

std::string safeConsoleText(std::string text, size_t maxBytes = 60)
{
    if (text.size() > maxBytes)
        text.resize(maxBytes);
    for (char &c : text) {
        const auto byte = static_cast<unsigned char>(c);
        if (byte < 0x20 || byte == 0x7f)
            c = ' ';
    }
    return text;
}

const JSONValue *objectMember(const JSONValue *object, const char *name)
{
    if (!object || !object->IsObject())
        return nullptr;
    const JSONObject &members = object->AsObject();
    const auto member = members.find(name);
    return member == members.end() ? nullptr : member->second;
}

std::string stringMember(const JSONValue *object, const char *name)
{
    const JSONValue *value = objectMember(object, name);
    return value && value->IsString() ? value->AsString() : "";
}

const JSONValue *numberMember(const JSONValue *object, const char *name)
{
    const JSONValue *value = objectMember(object, name);
    return value && value->IsNumber() ? value : nullptr;
}

std::string packetType(const meshtastic_MeshPacket *mp, const JSONValue *json)
{
    if (mp->which_payload_variant != meshtastic_MeshPacket_decoded_tag)
        return "ENCRYPTED";
    const std::string type = stringMember(json, "type");
    if (!type.empty()) {
        std::string upper = type;
        std::transform(upper.begin(), upper.end(), upper.begin(), [](unsigned char c) { return std::toupper(c); });
        return upper;
    }
    return "PORT_" + std::to_string(static_cast<unsigned>(mp->decoded.portnum));
}

std::string signalBar(int32_t rssi, bool styled)
{
    const float clamped = std::max(-120.0f, std::min(-50.0f, static_cast<float>(rssi)));
    const int filled = std::max(0, std::min(10, static_cast<int>(std::lround(((clamped + 120.0f) / 70.0f) * 10.0f))));
    const std::string full = styled ? "▓" : "#";
    const std::string empty = styled ? "░" : ".";
    std::string bar;
    for (int i = 0; i < filled; ++i)
        bar += full;
    for (int i = filled; i < 10; ++i)
        bar += empty;
    const char *color = rssi > -80 ? ANSI_GREEN : (rssi > -100 ? ANSI_YELLOW : ANSI_RED);
    return styledText(color, bar, styled);
}

std::string payloadSummary(const meshtastic_MeshPacket *mp, const JSONValue *json)
{
    std::ostringstream out;
    if (mp->which_payload_variant != meshtastic_MeshPacket_decoded_tag) {
        out << mp->encrypted.size << " bytes ch=0x" << std::hex << std::setw(2) << std::setfill('0')
            << static_cast<unsigned>(mp->channel);
        return out.str();
    }

    out << "port=" << static_cast<unsigned>(mp->decoded.portnum);
    if (mp->pki_encrypted)
        out << " pki";

    const JSONValue *payload = objectMember(json, "payload");
    const std::string type = stringMember(json, "type");
    const JSONValue *latitude = numberMember(payload, "latitude_i");
    const JSONValue *longitude = numberMember(payload, "longitude_i");
    if (type == "text" && !stringMember(payload, "text").empty()) {
        out << " \"" << safeConsoleText(stringMember(payload, "text")) << "\"";
    } else if ((type == "position" || type == "waypoint") && latitude && longitude) {
        out << std::fixed << std::setprecision(4) << " lat=" << latitude->AsNumber() / 1e7
            << " lon=" << longitude->AsNumber() / 1e7;
        const JSONValue *altitude = numberMember(payload, "altitude");
        if (altitude)
            out << std::setprecision(0) << " alt=" << altitude->AsNumber() << 'm';
    } else if (type == "nodeinfo") {
        const std::string longName = stringMember(payload, "longname");
        const std::string name = longName.empty() ? stringMember(payload, "shortname") : longName;
        if (!name.empty())
            out << " \"" << safeConsoleText(name) << "\"";
        const JSONValue *role = numberMember(payload, "role");
        if (role)
            out << " role=" << static_cast<unsigned>(role->AsNumber());
    } else if (type == "telemetry") {
        const JSONValue *battery = numberMember(payload, "battery_level");
        const JSONValue *voltage = numberMember(payload, "voltage");
        const JSONValue *temperature = numberMember(payload, "temperature");
        if (battery)
            out << " batt=" << static_cast<unsigned>(battery->AsNumber()) << '%';
        else if (voltage)
            out << " voltage=" << voltage->AsNumber();
        if (temperature)
            out << " temp=" << temperature->AsNumber() << 'C';
    } else if (type == "detection" && !stringMember(payload, "text").empty()) {
        out << ' ' << safeConsoleText(stringMember(payload, "text"));
    }
    return out.str();
}
} // namespace
std::string MeshPacketSerializer::JsonSerialize(const meshtastic_MeshPacket *mp, bool shouldLog)
{
    // the created jsonObj is immutable after creation, so
    // we need to do the heavy lifting before assembling it.
    std::string msgType;
    JSONObject jsonObj;

    if (mp->which_payload_variant == meshtastic_MeshPacket_decoded_tag) {
        JSONObject msgPayload;
        switch (mp->decoded.portnum) {
        case meshtastic_PortNum_TEXT_MESSAGE_APP: {
            msgType = "text";
            // convert bytes to string
            if (shouldLog)
                LOG_DEBUG("got text message of size %u", mp->decoded.payload.size);

            char payloadStr[(mp->decoded.payload.size) + 1];
            memcpy(payloadStr, mp->decoded.payload.bytes, mp->decoded.payload.size);
            payloadStr[mp->decoded.payload.size] = 0; // null terminated string
            // check if this is a JSON payload
            JSONValue *json_value = JSON::Parse(payloadStr);
            if (json_value != NULL) {
                if (shouldLog)
                    LOG_INFO("text message payload is of type json");

                // if it is, then we can just use the json object
                jsonObj["payload"] = json_value;
            } else {
                // if it isn't, then we need to create a json object
                // with the string as the value
                if (shouldLog)
                    LOG_INFO("text message payload is of type plaintext");

                msgPayload["text"] = new JSONValue(payloadStr);
                jsonObj["payload"] = new JSONValue(msgPayload);
            }
            break;
        }
        case meshtastic_PortNum_TELEMETRY_APP: {
            msgType = "telemetry";
            meshtastic_Telemetry scratch;
            meshtastic_Telemetry *decoded = NULL;
            memset(&scratch, 0, sizeof(scratch));
            if (pb_decode_from_bytes(mp->decoded.payload.bytes, mp->decoded.payload.size, &meshtastic_Telemetry_msg, &scratch)) {
                decoded = &scratch;
                if (decoded->which_variant == meshtastic_Telemetry_device_metrics_tag) {
                    // If battery is present, encode the battery level value
                    // TODO - Add a condition to send a code for a non-present value
                    if (decoded->variant.device_metrics.has_battery_level) {
                        msgPayload["battery_level"] = new JSONValue((int)decoded->variant.device_metrics.battery_level);
                    }
                    msgPayload["voltage"] = new JSONValue(decoded->variant.device_metrics.voltage);
                    msgPayload["channel_utilization"] = new JSONValue(decoded->variant.device_metrics.channel_utilization);
                    msgPayload["air_util_tx"] = new JSONValue(decoded->variant.device_metrics.air_util_tx);
                    msgPayload["uptime_seconds"] = new JSONValue((unsigned int)decoded->variant.device_metrics.uptime_seconds);
                } else if (decoded->which_variant == meshtastic_Telemetry_environment_metrics_tag) {
                    // Avoid sending 0s for sensors that could be 0
                    if (decoded->variant.environment_metrics.has_temperature) {
                        msgPayload["temperature"] = new JSONValue(decoded->variant.environment_metrics.temperature);
                    }
                    if (decoded->variant.environment_metrics.has_relative_humidity) {
                        msgPayload["relative_humidity"] = new JSONValue(decoded->variant.environment_metrics.relative_humidity);
                    }
                    if (decoded->variant.environment_metrics.has_barometric_pressure) {
                        msgPayload["barometric_pressure"] =
                            new JSONValue(decoded->variant.environment_metrics.barometric_pressure);
                    }
                    if (decoded->variant.environment_metrics.has_gas_resistance) {
                        msgPayload["gas_resistance"] = new JSONValue(decoded->variant.environment_metrics.gas_resistance);
                    }
                    if (decoded->variant.environment_metrics.has_voltage) {
                        msgPayload["voltage"] = new JSONValue(decoded->variant.environment_metrics.voltage);
                    }
                    if (decoded->variant.environment_metrics.has_current) {
                        msgPayload["current"] = new JSONValue(decoded->variant.environment_metrics.current);
                    }
                    if (decoded->variant.environment_metrics.has_lux) {
                        msgPayload["lux"] = new JSONValue(decoded->variant.environment_metrics.lux);
                    }
                    if (decoded->variant.environment_metrics.has_white_lux) {
                        msgPayload["white_lux"] = new JSONValue(decoded->variant.environment_metrics.white_lux);
                    }
                    if (decoded->variant.environment_metrics.has_iaq) {
                        msgPayload["iaq"] = new JSONValue((uint)decoded->variant.environment_metrics.iaq);
                    }
                    if (decoded->variant.environment_metrics.has_distance) {
                        msgPayload["distance"] = new JSONValue(decoded->variant.environment_metrics.distance);
                    }
                    if (decoded->variant.environment_metrics.has_wind_speed) {
                        msgPayload["wind_speed"] = new JSONValue(decoded->variant.environment_metrics.wind_speed);
                    }
                    if (decoded->variant.environment_metrics.has_wind_direction) {
                        msgPayload["wind_direction"] = new JSONValue((uint)decoded->variant.environment_metrics.wind_direction);
                    }
                    if (decoded->variant.environment_metrics.has_wind_gust) {
                        msgPayload["wind_gust"] = new JSONValue(decoded->variant.environment_metrics.wind_gust);
                    }
                    if (decoded->variant.environment_metrics.has_wind_lull) {
                        msgPayload["wind_lull"] = new JSONValue(decoded->variant.environment_metrics.wind_lull);
                    }
                    if (decoded->variant.environment_metrics.has_radiation) {
                        msgPayload["radiation"] = new JSONValue(decoded->variant.environment_metrics.radiation);
                    }
                    if (decoded->variant.environment_metrics.has_ir_lux) {
                        msgPayload["ir_lux"] = new JSONValue(decoded->variant.environment_metrics.ir_lux);
                    }
                    if (decoded->variant.environment_metrics.has_uv_lux) {
                        msgPayload["uv_lux"] = new JSONValue(decoded->variant.environment_metrics.uv_lux);
                    }
                    if (decoded->variant.environment_metrics.has_weight) {
                        msgPayload["weight"] = new JSONValue(decoded->variant.environment_metrics.weight);
                    }
                    if (decoded->variant.environment_metrics.has_rainfall_1h) {
                        msgPayload["rainfall_1h"] = new JSONValue(decoded->variant.environment_metrics.rainfall_1h);
                    }
                    if (decoded->variant.environment_metrics.has_rainfall_24h) {
                        msgPayload["rainfall_24h"] = new JSONValue(decoded->variant.environment_metrics.rainfall_24h);
                    }
                    if (decoded->variant.environment_metrics.has_soil_moisture) {
                        msgPayload["soil_moisture"] = new JSONValue((uint)decoded->variant.environment_metrics.soil_moisture);
                    }
                    if (decoded->variant.environment_metrics.has_soil_temperature) {
                        msgPayload["soil_temperature"] = new JSONValue(decoded->variant.environment_metrics.soil_temperature);
                    }
                } else if (decoded->which_variant == meshtastic_Telemetry_air_quality_metrics_tag) {
                    if (decoded->variant.air_quality_metrics.has_pm10_standard) {
                        msgPayload["pm10"] = new JSONValue((unsigned int)decoded->variant.air_quality_metrics.pm10_standard);
                    }
                    if (decoded->variant.air_quality_metrics.has_pm25_standard) {
                        msgPayload["pm25"] = new JSONValue((unsigned int)decoded->variant.air_quality_metrics.pm25_standard);
                    }
                    if (decoded->variant.air_quality_metrics.has_pm100_standard) {
                        msgPayload["pm100"] = new JSONValue((unsigned int)decoded->variant.air_quality_metrics.pm100_standard);
                    }
                    if (decoded->variant.air_quality_metrics.has_co2) {
                        msgPayload["co2"] = new JSONValue((unsigned int)decoded->variant.air_quality_metrics.co2);
                    }
                    if (decoded->variant.air_quality_metrics.has_co2_temperature) {
                        msgPayload["co2_temperature"] = new JSONValue(decoded->variant.air_quality_metrics.co2_temperature);
                    }
                    if (decoded->variant.air_quality_metrics.has_co2_humidity) {
                        msgPayload["co2_humidity"] = new JSONValue(decoded->variant.air_quality_metrics.co2_humidity);
                    }
                    if (decoded->variant.air_quality_metrics.has_form_formaldehyde) {
                        msgPayload["form_formaldehyde"] = new JSONValue(decoded->variant.air_quality_metrics.form_formaldehyde);
                    }
                    if (decoded->variant.air_quality_metrics.has_form_temperature) {
                        msgPayload["form_temperature"] = new JSONValue(decoded->variant.air_quality_metrics.form_temperature);
                    }
                    if (decoded->variant.air_quality_metrics.has_form_humidity) {
                        msgPayload["form_humidity"] = new JSONValue(decoded->variant.air_quality_metrics.form_humidity);
                    }
                } else if (decoded->which_variant == meshtastic_Telemetry_power_metrics_tag) {
                    if (decoded->variant.power_metrics.has_ch1_voltage) {
                        msgPayload["voltage_ch1"] = new JSONValue(decoded->variant.power_metrics.ch1_voltage);
                    }
                    if (decoded->variant.power_metrics.has_ch1_current) {
                        msgPayload["current_ch1"] = new JSONValue(decoded->variant.power_metrics.ch1_current);
                    }
                    if (decoded->variant.power_metrics.has_ch2_voltage) {
                        msgPayload["voltage_ch2"] = new JSONValue(decoded->variant.power_metrics.ch2_voltage);
                    }
                    if (decoded->variant.power_metrics.has_ch2_current) {
                        msgPayload["current_ch2"] = new JSONValue(decoded->variant.power_metrics.ch2_current);
                    }
                    if (decoded->variant.power_metrics.has_ch3_voltage) {
                        msgPayload["voltage_ch3"] = new JSONValue(decoded->variant.power_metrics.ch3_voltage);
                    }
                    if (decoded->variant.power_metrics.has_ch3_current) {
                        msgPayload["current_ch3"] = new JSONValue(decoded->variant.power_metrics.ch3_current);
                    }
                }
                jsonObj["payload"] = new JSONValue(msgPayload);
            } else if (shouldLog) {
                LOG_ERROR(errStr, msgType.c_str());
            }
            break;
        }
        case meshtastic_PortNum_NODEINFO_APP: {
            msgType = "nodeinfo";
            meshtastic_User scratch;
            meshtastic_User *decoded = NULL;
            memset(&scratch, 0, sizeof(scratch));
            if (pb_decode_from_bytes(mp->decoded.payload.bytes, mp->decoded.payload.size, &meshtastic_User_msg, &scratch)) {
                decoded = &scratch;
                msgPayload["id"] = new JSONValue(decoded->id);
                msgPayload["longname"] = new JSONValue(decoded->long_name);
                msgPayload["shortname"] = new JSONValue(decoded->short_name);
                msgPayload["hardware"] = new JSONValue(decoded->hw_model);
                msgPayload["role"] = new JSONValue((int)decoded->role);
                jsonObj["payload"] = new JSONValue(msgPayload);
            } else if (shouldLog) {
                LOG_ERROR(errStr, msgType.c_str());
            }
            break;
        }
        case meshtastic_PortNum_POSITION_APP: {
            msgType = "position";
            meshtastic_Position scratch;
            meshtastic_Position *decoded = NULL;
            memset(&scratch, 0, sizeof(scratch));
            if (pb_decode_from_bytes(mp->decoded.payload.bytes, mp->decoded.payload.size, &meshtastic_Position_msg, &scratch)) {
                decoded = &scratch;
                if ((int)decoded->time) {
                    msgPayload["time"] = new JSONValue((unsigned int)decoded->time);
                }
                if ((int)decoded->timestamp) {
                    msgPayload["timestamp"] = new JSONValue((unsigned int)decoded->timestamp);
                }
                msgPayload["latitude_i"] = new JSONValue((int)decoded->latitude_i);
                msgPayload["longitude_i"] = new JSONValue((int)decoded->longitude_i);
                if ((int)decoded->altitude) {
                    msgPayload["altitude"] = new JSONValue((int)decoded->altitude);
                }
                if ((int)decoded->ground_speed) {
                    msgPayload["ground_speed"] = new JSONValue((unsigned int)decoded->ground_speed);
                }
                if (int(decoded->ground_track)) {
                    msgPayload["ground_track"] = new JSONValue((unsigned int)decoded->ground_track);
                }
                if (int(decoded->sats_in_view)) {
                    msgPayload["sats_in_view"] = new JSONValue((unsigned int)decoded->sats_in_view);
                }
                if ((int)decoded->PDOP) {
                    msgPayload["PDOP"] = new JSONValue((int)decoded->PDOP);
                }
                if ((int)decoded->HDOP) {
                    msgPayload["HDOP"] = new JSONValue((int)decoded->HDOP);
                }
                if ((int)decoded->VDOP) {
                    msgPayload["VDOP"] = new JSONValue((int)decoded->VDOP);
                }
                if ((int)decoded->precision_bits) {
                    msgPayload["precision_bits"] = new JSONValue((int)decoded->precision_bits);
                }
                jsonObj["payload"] = new JSONValue(msgPayload);
            } else if (shouldLog) {
                LOG_ERROR(errStr, msgType.c_str());
            }
            break;
        }
        case meshtastic_PortNum_WAYPOINT_APP: {
            msgType = "waypoint";
            meshtastic_Waypoint scratch;
            meshtastic_Waypoint *decoded = NULL;
            memset(&scratch, 0, sizeof(scratch));
            if (pb_decode_from_bytes(mp->decoded.payload.bytes, mp->decoded.payload.size, &meshtastic_Waypoint_msg, &scratch)) {
                decoded = &scratch;
                msgPayload["id"] = new JSONValue((unsigned int)decoded->id);
                msgPayload["name"] = new JSONValue(decoded->name);
                msgPayload["description"] = new JSONValue(decoded->description);
                msgPayload["expire"] = new JSONValue((unsigned int)decoded->expire);
                msgPayload["locked_to"] = new JSONValue((unsigned int)decoded->locked_to);
                msgPayload["latitude_i"] = new JSONValue((int)decoded->latitude_i);
                msgPayload["longitude_i"] = new JSONValue((int)decoded->longitude_i);
                jsonObj["payload"] = new JSONValue(msgPayload);
            } else if (shouldLog) {
                LOG_ERROR(errStr, msgType.c_str());
            }
            break;
        }
        case meshtastic_PortNum_NEIGHBORINFO_APP: {
            msgType = "neighborinfo";
            meshtastic_NeighborInfo scratch;
            meshtastic_NeighborInfo *decoded = NULL;
            memset(&scratch, 0, sizeof(scratch));
            if (pb_decode_from_bytes(mp->decoded.payload.bytes, mp->decoded.payload.size, &meshtastic_NeighborInfo_msg,
                                     &scratch)) {
                decoded = &scratch;
                msgPayload["node_id"] = new JSONValue((unsigned int)decoded->node_id);
                msgPayload["node_broadcast_interval_secs"] = new JSONValue((unsigned int)decoded->node_broadcast_interval_secs);
                msgPayload["last_sent_by_id"] = new JSONValue((unsigned int)decoded->last_sent_by_id);
                msgPayload["neighbors_count"] = new JSONValue(decoded->neighbors_count);
                JSONArray neighbors;
                for (uint8_t i = 0; i < decoded->neighbors_count; i++) {
                    JSONObject neighborObj;
                    neighborObj["node_id"] = new JSONValue((unsigned int)decoded->neighbors[i].node_id);
                    neighborObj["snr"] = new JSONValue((int)decoded->neighbors[i].snr);
                    neighbors.push_back(new JSONValue(neighborObj));
                }
                msgPayload["neighbors"] = new JSONValue(neighbors);
                jsonObj["payload"] = new JSONValue(msgPayload);
            } else if (shouldLog) {
                LOG_ERROR(errStr, msgType.c_str());
            }
            break;
        }
        case meshtastic_PortNum_TRACEROUTE_APP: {
            if (mp->decoded.request_id) { // Only report the traceroute response
                msgType = "traceroute";
                meshtastic_RouteDiscovery scratch;
                meshtastic_RouteDiscovery *decoded = NULL;
                memset(&scratch, 0, sizeof(scratch));
                if (pb_decode_from_bytes(mp->decoded.payload.bytes, mp->decoded.payload.size, &meshtastic_RouteDiscovery_msg,
                                         &scratch)) {
                    decoded = &scratch;
                    JSONArray route;      // Route this message took
                    JSONArray routeBack;  // Route this message took back
                    JSONArray snrTowards; // Snr for forward route
                    JSONArray snrBack;    // Snr for reverse route

                    // Lambda function for adding a long name to the route
                    auto addToRoute = [](JSONArray *route, NodeNum num) {
                        char long_name[40] = "Unknown";
                        meshtastic_NodeInfoLite *node = nodeDB->getMeshNode(num);
                        bool name_known = node ? node->has_user : false;
                        if (name_known)
                            memcpy(long_name, node->user.long_name, sizeof(long_name));
                        route->push_back(new JSONValue(long_name));
                    };
                    addToRoute(&route, mp->to); // Started at the original transmitter (destination of response)
                    for (uint8_t i = 0; i < decoded->route_count; i++) {
                        addToRoute(&route, decoded->route[i]);
                    }
                    addToRoute(&route, mp->from); // Ended at the original destination (source of response)

                    addToRoute(&routeBack, mp->from); // Started at the original destination (source of response)
                    for (uint8_t i = 0; i < decoded->route_back_count; i++) {
                        addToRoute(&routeBack, decoded->route_back[i]);
                    }
                    addToRoute(&routeBack, mp->to); // Ended at the original transmitter (destination of response)

                    for (uint8_t i = 0; i < decoded->snr_back_count; i++) {
                        snrBack.push_back(new JSONValue((float)decoded->snr_back[i] / 4));
                    }

                    for (uint8_t i = 0; i < decoded->snr_towards_count; i++) {
                        snrTowards.push_back(new JSONValue((float)decoded->snr_towards[i] / 4));
                    }

                    msgPayload["route"] = new JSONValue(route);
                    msgPayload["route_back"] = new JSONValue(routeBack);
                    msgPayload["snr_back"] = new JSONValue(snrBack);
                    msgPayload["snr_towards"] = new JSONValue(snrTowards);
                    jsonObj["payload"] = new JSONValue(msgPayload);
                } else if (shouldLog) {
                    LOG_ERROR(errStr, msgType.c_str());
                }
            }
            break;
        }
        case meshtastic_PortNum_DETECTION_SENSOR_APP: {
            msgType = "detection";
            char payloadStr[(mp->decoded.payload.size) + 1];
            memcpy(payloadStr, mp->decoded.payload.bytes, mp->decoded.payload.size);
            payloadStr[mp->decoded.payload.size] = 0; // null terminated string
            msgPayload["text"] = new JSONValue(payloadStr);
            jsonObj["payload"] = new JSONValue(msgPayload);
            break;
        }
#ifdef ARCH_ESP32
        case meshtastic_PortNum_PAXCOUNTER_APP: {
            msgType = "paxcounter";
            meshtastic_Paxcount scratch;
            meshtastic_Paxcount *decoded = NULL;
            memset(&scratch, 0, sizeof(scratch));
            if (pb_decode_from_bytes(mp->decoded.payload.bytes, mp->decoded.payload.size, &meshtastic_Paxcount_msg, &scratch)) {
                decoded = &scratch;
                msgPayload["wifi_count"] = new JSONValue((unsigned int)decoded->wifi);
                msgPayload["ble_count"] = new JSONValue((unsigned int)decoded->ble);
                msgPayload["uptime"] = new JSONValue((unsigned int)decoded->uptime);
                jsonObj["payload"] = new JSONValue(msgPayload);
            } else if (shouldLog) {
                LOG_ERROR(errStr, msgType.c_str());
            }
            break;
        }
#endif
        case meshtastic_PortNum_REMOTE_HARDWARE_APP: {
            meshtastic_HardwareMessage scratch;
            meshtastic_HardwareMessage *decoded = NULL;
            memset(&scratch, 0, sizeof(scratch));
            if (pb_decode_from_bytes(mp->decoded.payload.bytes, mp->decoded.payload.size, &meshtastic_HardwareMessage_msg,
                                     &scratch)) {
                decoded = &scratch;
                if (decoded->type == meshtastic_HardwareMessage_Type_GPIOS_CHANGED) {
                    msgType = "gpios_changed";
                    msgPayload["gpio_value"] = new JSONValue((unsigned int)decoded->gpio_value);
                    jsonObj["payload"] = new JSONValue(msgPayload);
                } else if (decoded->type == meshtastic_HardwareMessage_Type_READ_GPIOS_REPLY) {
                    msgType = "gpios_read_reply";
                    msgPayload["gpio_value"] = new JSONValue((unsigned int)decoded->gpio_value);
                    msgPayload["gpio_mask"] = new JSONValue((unsigned int)decoded->gpio_mask);
                    jsonObj["payload"] = new JSONValue(msgPayload);
                }
            } else if (shouldLog) {
                LOG_ERROR(errStr, "RemoteHardware");
            }
            break;
        }
        // add more packet types here if needed
        default:
            break;
        }
    } else if (shouldLog) {
        LOG_WARN("Couldn't convert encrypted payload of MeshPacket to JSON");
    }

    jsonObj["id"] = new JSONValue((unsigned int)mp->id);
    jsonObj["timestamp"] = new JSONValue((unsigned int)mp->rx_time);
    jsonObj["to"] = new JSONValue((unsigned int)mp->to);
    jsonObj["from"] = new JSONValue((unsigned int)mp->from);
    jsonObj["channel"] = new JSONValue((unsigned int)mp->channel);
    jsonObj["type"] = new JSONValue(msgType.c_str());
    jsonObj["sender"] = new JSONValue(nodeDB->getNodeId().c_str());
    if (mp->rx_rssi != 0)
        jsonObj["rssi"] = new JSONValue((int)mp->rx_rssi);
    if (mp->rx_snr != 0)
        jsonObj["snr"] = new JSONValue((float)mp->rx_snr);
    const int8_t hopsAway = getHopsAway(*mp);
    if (hopsAway >= 0) {
        jsonObj["hops_away"] = new JSONValue((unsigned int)(hopsAway));
        jsonObj["hop_start"] = new JSONValue((unsigned int)(mp->hop_start));
    }

    // serialize and write it to the stream
    JSONValue *value = new JSONValue(jsonObj);
    std::string jsonStr = value->Stringify();

    if (shouldLog)
        LOG_INFO("serialized json message: %s", jsonStr.c_str());

    delete value;
    return jsonStr;
}

std::string MeshPacketSerializer::JsonSerializeEncrypted(const meshtastic_MeshPacket *mp)
{
    JSONObject jsonObj;

    jsonObj["id"] = new JSONValue((unsigned int)mp->id);
    jsonObj["time_ms"] = new JSONValue((double)millis());
    jsonObj["timestamp"] = new JSONValue((unsigned int)mp->rx_time);
    jsonObj["to"] = new JSONValue((unsigned int)mp->to);
    jsonObj["from"] = new JSONValue((unsigned int)mp->from);
    jsonObj["channel"] = new JSONValue((unsigned int)mp->channel);
    jsonObj["want_ack"] = new JSONValue(mp->want_ack);

    if (mp->rx_rssi != 0)
        jsonObj["rssi"] = new JSONValue((int)mp->rx_rssi);
    if (mp->rx_snr != 0)
        jsonObj["snr"] = new JSONValue((float)mp->rx_snr);
    const int8_t hopsAway = getHopsAway(*mp);
    if (hopsAway >= 0) {
        jsonObj["hops_away"] = new JSONValue((unsigned int)(hopsAway));
        jsonObj["hop_start"] = new JSONValue((unsigned int)(mp->hop_start));
    }
    jsonObj["size"] = new JSONValue((unsigned int)mp->encrypted.size);
    auto encryptedStr = bytesToHex(mp->encrypted.bytes, mp->encrypted.size);
    jsonObj["bytes"] = new JSONValue(encryptedStr.c_str());

    // serialize and write it to the stream
    JSONValue *value = new JSONValue(jsonObj);
    std::string jsonStr = value->Stringify();

    delete value;
    return jsonStr;
}
std::string MeshPacketSerializer::ConsoleSerialize(const meshtastic_MeshPacket *mp, bool styled, const char *direction,
                                                   bool includeSignal)
{
    std::unique_ptr<JSONValue> json;
    if (mp->which_payload_variant == meshtastic_MeshPacket_decoded_tag) {
        const std::string serialized = JsonSerialize(mp, false);
        json = tryParseJson(serialized.c_str());
    }

    const std::string type = packetType(mp, json.get());
    const std::string summary = payloadSummary(mp, json.get());
    std::ostringstream out;
    out << ' ' << styledText(ANSI_BRIGHT_GREEN, direction, styled) << ' ' << styledText(ANSI_BOLD, "PKT", styled) << "  "
        << styledText(ANSI_BLUE, "meshtastic", styled) << "  ";

    std::ostringstream route;
    route << std::hex << std::setw(8) << std::setfill('0') << mp->from << " -> " << std::setw(8) << mp->to;
    out << styledText(ANSI_WHITE, route.str(), styled) << "  " << styledText(ANSI_YELLOW, type, styled);
    if (type.size() < 12)
        out << std::string(12 - type.size(), ' ');
    out << ' ';

    const bool hasRxSignal = includeSignal && mp->rx_rssi != 0;
    if (hasRxSignal) {
        out << styledText(ANSI_DIM, "rssi", styled) << ' ' << std::fixed << std::setw(6) << std::setprecision(1)
            << static_cast<double>(mp->rx_rssi) << ' ' << signalBar(mp->rx_rssi, styled) << ' ';
    } else {
        const std::string emptyBar = styled ? "░░░░░░░░░░" : "..........";
        out << styledText(ANSI_DIM, "rssi", styled) << "     -- " << styledText(ANSI_DIM, emptyBar, styled) << ' ';
    }

    // rx_snr has no presence bit; a nonzero RX RSSI indicates receive metadata, where 0 dB is valid.
    if (hasRxSignal)
        out << styledText(ANSI_DIM, "snr", styled) << ' ' << std::fixed << std::setw(5) << std::setprecision(1) << mp->rx_snr;
    else
        out << styledText(ANSI_DIM, "snr", styled) << "    --";
    if (!summary.empty())
        out << "  " << styledText(ANSI_DIM, summary, styled);
    return out.str();
}
#endif
