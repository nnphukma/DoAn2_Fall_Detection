#include "ble_data_service.h"

#include <NimBLEDevice.h>
#include "ble_data_protocol.h"

namespace
{
NimBLECharacteristic *dataCharacteristic = nullptr;
volatile bool connected = false;
volatile bool notificationsEnabled = false;
volatile uint16_t negotiatedMtu = 23;
volatile uint32_t notificationFailures = 0;
uint16_t frameId = 0;

class ServerCallbacks : public NimBLEServerCallbacks
{
    void onConnect(NimBLEServer *, NimBLEConnInfo &) override { connected = true; }
    void onDisconnect(NimBLEServer *, NimBLEConnInfo &, int) override {
        connected = false;
        notificationsEnabled = false;
        NimBLEDevice::startAdvertising();
    }
    void onMTUChange(uint16_t mtu, NimBLEConnInfo &) override { negotiatedMtu = mtu; }
};

// NimBLE-Arduino 2.x/3.x does not expose getSubscribedCount() on a
// characteristic. Track the Client Characteristic Configuration Descriptor
// state through the supported subscription callback instead.
class DataCharacteristicCallbacks : public NimBLECharacteristicCallbacks
{
    void onSubscribe(NimBLECharacteristic *, NimBLEConnInfo &, uint16_t subValue) override
    {
        // Bit 0 means notifications are enabled (bit 1 is indications).
        notificationsEnabled = (subValue & 0x0001u) != 0;
    }
};
}

bool beginBleDataService()
{
    NimBLEDevice::init(BLE_DEVICE_NAME);
    NimBLEDevice::setMTU(247);
    NimBLEServer *server = NimBLEDevice::createServer();
    server->setCallbacks(new ServerCallbacks());
    NimBLEService *service = server->createService(BLE_SERVICE_UUID);
    dataCharacteristic = service->createCharacteristic(
        BLE_DATA_CHARACTERISTIC_UUID, NIMBLE_PROPERTY::NOTIFY);
    dataCharacteristic->setCallbacks(new DataCharacteristicCallbacks());
    NimBLEAdvertising *advertising = NimBLEDevice::getAdvertising();
    advertising->addServiceUUID(BLE_SERVICE_UUID);
    advertising->start();
    return true;
}

bool bleClientSubscribed()
{
    return connected && notificationsEnabled && dataCharacteristic != nullptr;
}

uint32_t bleNotificationFailures() { return notificationFailures; }

bool bleNotifySample(const BleLogItem &item, uint32_t droppedPackets)
{
    if (!bleClientSubscribed()) return false;
    uint8_t packet[BleProtocol::PACKET_SIZE];
    BleProtocol::serializeSample(item.sequence, droppedPackets, item.sample, item.result, packet);
    // ATT notification payload is MTU - 3. Every fragment has a six-byte
    // envelope; the packet CRC validates the fully reassembled payload.
    uint16_t available = negotiatedMtu > (3 + BleProtocol::FRAME_HEADER_SIZE)
        ? negotiatedMtu - 3 - BleProtocol::FRAME_HEADER_SIZE : 1;
    uint8_t fragments = static_cast<uint8_t>((BleProtocol::PACKET_SIZE + available - 1) / available);
    const uint16_t id = frameId++;
    for (uint8_t index = 0; index < fragments; ++index) {
        const size_t offset = static_cast<size_t>(index) * available;
        const size_t remaining = BleProtocol::PACKET_SIZE - offset;
        const size_t bytes = remaining < available ? remaining : available;
        uint8_t frame[BleProtocol::FRAME_HEADER_SIZE + 64];
        frame[0] = BleProtocol::FRAME_MAGIC;
        frame[1] = BleProtocol::VERSION;
        frame[2] = static_cast<uint8_t>(id & 0xff);
        frame[3] = static_cast<uint8_t>(id >> 8);
        frame[4] = index;
        frame[5] = fragments;
        memcpy(frame + BleProtocol::FRAME_HEADER_SIZE, packet + offset, bytes);
        if (!dataCharacteristic->notify(frame, BleProtocol::FRAME_HEADER_SIZE + bytes)) {
            ++notificationFailures;
            return false;
        }
    }
    return true;
}
