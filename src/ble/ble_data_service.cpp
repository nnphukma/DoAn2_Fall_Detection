#include "ble_data_service.h"
#include <NimBLEDevice.h>
#include "ble_data_protocol.h"
#include "config.h"

namespace
{
NimBLECharacteristic *dataCharacteristic = nullptr;
volatile bool connected = false;
volatile uint16_t negotiatedMtu = 23;
volatile uint32_t notificationFailures = 0;
uint16_t frameId = 0;

uint16_t batteryVoltageMv = 0;
uint8_t batteryPercent = 0;

void updateBatteryTelemetry()
{
    static bool hasSample = false;
    static uint32_t lastSampleMs = 0;

    // Battery telemetry changes slowly; do not run ADC conversion at 100 Hz.
    const uint32_t now = millis();
    if (hasSample && (now - lastSampleMs < 1000))
        return;

    constexpr uint8_t SAMPLE_COUNT = 8;
    uint32_t adcMillivoltsSum = 0;
    for (uint8_t i = 0; i < SAMPLE_COUNT; ++i)
        adcMillivoltsSum += analogReadMilliVolts(Config::BATTERY_ADC_PIN);

    const uint32_t adcMillivolts = (adcMillivoltsSum + SAMPLE_COUNT / 2) / SAMPLE_COUNT;
    uint32_t measuredBatteryMv = adcMillivolts * Config::BATTERY_DIVIDER_RATIO;
    if (measuredBatteryMv > 65535u)
        measuredBatteryMv = 65535u;

    batteryVoltageMv = static_cast<uint16_t>(measuredBatteryMv);

    if (batteryVoltageMv <= Config::BATTERY_EMPTY_MV)
        batteryPercent = 0;
    else if (batteryVoltageMv >= Config::BATTERY_FULL_MV)
        batteryPercent = 100;
    else
    {
        const uint32_t numerator =
            static_cast<uint32_t>(batteryVoltageMv - Config::BATTERY_EMPTY_MV) * 100u;
        const uint32_t denominator = Config::BATTERY_FULL_MV - Config::BATTERY_EMPTY_MV;
        batteryPercent = static_cast<uint8_t>((numerator + denominator / 2) / denominator);
    }

    lastSampleMs = now;
    hasSample = true;
}

class ServerCallbacks : public NimBLEServerCallbacks
{
    void onConnect(NimBLEServer *, NimBLEConnInfo &) override { connected = true; }
    void onDisconnect(NimBLEServer *, NimBLEConnInfo &, int) override
    {
        connected = false;
        NimBLEDevice::startAdvertising();
    }
    void onMTUChange(uint16_t mtu, NimBLEConnInfo &) override { negotiatedMtu = mtu; }
};
}

bool beginBleDataService()
{
    // GPIO0 is ADC1_CH0 on ESP32-C3. The divided battery voltage is 1.8..2.1 V.
    analogReadResolution(12);
    analogSetPinAttenuation(Config::BATTERY_ADC_PIN, ADC_11db);
    updateBatteryTelemetry();

    NimBLEDevice::init(BLE_DEVICE_NAME);
    NimBLEDevice::setMTU(247);
    NimBLEServer *server = NimBLEDevice::createServer();
    server->setCallbacks(new ServerCallbacks());
    NimBLEService *service = server->createService(BLE_SERVICE_UUID);
    dataCharacteristic = service->createCharacteristic(
        BLE_DATA_CHARACTERISTIC_UUID, NIMBLE_PROPERTY::NOTIFY);
    NimBLEAdvertising *advertising = NimBLEDevice::getAdvertising();
    advertising->addServiceUUID(BLE_SERVICE_UUID);
    advertising->start();
    return true;
}

bool bleClientSubscribed()
{
    // NimBLE sends only to subscribed peers internally. Do not suppress the
    // producer because its subscription callback may race start_notify().
    return connected && dataCharacteristic != nullptr;
}

uint32_t bleNotificationFailures() { return notificationFailures; }

bool bleNotifySample(const BleLogItem &item, uint32_t droppedPackets)
{
    if (!bleClientSubscribed()) return false;

    updateBatteryTelemetry();

    uint8_t packet[BleProtocol::PACKET_SIZE];
    BleProtocol::serializeSample(item.sequence, droppedPackets, item.sample, item.result,
                                 batteryVoltageMv, batteryPercent, packet);

    // ATT notification payload is MTU - 3. Every fragment has a six-byte
    // envelope; the packet CRC validates the fully reassembled payload.
    uint16_t available = negotiatedMtu > (3 + BleProtocol::FRAME_HEADER_SIZE)
        ? negotiatedMtu - 3 - BleProtocol::FRAME_HEADER_SIZE : 1;
    uint8_t fragments = static_cast<uint8_t>((BleProtocol::PACKET_SIZE + available - 1) / available);
    const uint16_t id = frameId++;

    for (uint8_t index = 0; index < fragments; ++index)
    {
        const size_t offset = static_cast<size_t>(index) * available;
        const size_t remaining = BleProtocol::PACKET_SIZE - offset;
        const size_t bytes = remaining < available ? remaining : available;
        uint8_t frame[BleProtocol::FRAME_HEADER_SIZE + BleProtocol::PACKET_SIZE];
        frame[0] = BleProtocol::FRAME_MAGIC;
        frame[1] = BleProtocol::VERSION;
        frame[2] = static_cast<uint8_t>(id & 0xff);
        frame[3] = static_cast<uint8_t>(id >> 8);
        frame[4] = index;
        frame[5] = fragments;
        memcpy(frame + BleProtocol::FRAME_HEADER_SIZE, packet + offset, bytes);
        if (!dataCharacteristic->notify(frame, BleProtocol::FRAME_HEADER_SIZE + bytes))
        {
            ++notificationFailures;
            return false;
        }
    }
    return true;
}
