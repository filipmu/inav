#include <stdbool.h>
#include <stdint.h>
#include "platform.h"

#include "build/build_config.h"
#include "build/debug.h"

#include "io/serial.h"

#if defined(USE_RANGEFINDER_TOFSENSEF)
#include "drivers/rangefinder/rangefinder_virtual.h"
#include "drivers/time.h"
#include "drivers/serial.h"
#include "io/rangefinder.h"


// --- Packet definitions (from your prototype) ---
typedef __attribute__((packed)) struct {
    uint8_t frame_header;
    uint8_t function_mark;
    uint8_t reserved;
    uint8_t id;
    uint8_t system_time0;
    uint8_t system_time1;
    uint8_t system_time2;
    uint8_t system_time3;
    uint8_t dist0;
    uint8_t dist1;
    uint8_t dist2;
    uint8_t status;
    uint8_t sig_strength0;
    uint8_t sig_strength1;
    uint8_t range_precision;
    uint8_t checksum;
} tofsensefPacket8_t;

typedef __attribute__((packed)) struct {
    uint32_t addr;
    uint32_t sys_time;
    uint32_t dist;
    uint32_t quality;
} tofsensefPacket32_t;

#define TOFSENSEF_PACKET_SIZE sizeof(tofsensefPacket8_t)
#define TOFSENSEF_MIN_QUALITY 20
#define TOFSENSEF_TIMEOUT_MS 200

// --- Driver state ---
static serialPort_t * serialPort = NULL;
static serialPortConfig_t * portConfig;
static uint8_t buffer[TOFSENSEF_PACKET_SIZE];
static unsigned bufferPtr;
static timeMs_t lastProtocolActivityMs;
static bool hasNewData = false;
static int32_t sensorData = RANGEFINDER_NO_NEW_DATA;
static uint32_t lastSysTime;

// --- Init function ---
static bool tofsensefInit(void)
{
    serialPort = openSerialPort(portConfig->identifier, FUNCTION_RANGEFINDER, NULL, NULL, 921600, MODE_RXTX, SERIAL_NOT_INVERTED);
    if (!serialPort) {
        return false;
    }

    bufferPtr = 0;
    hasNewData = false;
    sensorData = RANGEFINDER_NO_NEW_DATA;
    lastProtocolActivityMs = 0;
    lastSysTime = 0;
    return true;
}

// --- Update function (called periodically) ---
static void tofsensefUpdate(void)
{
    tofsensefPacket8_t *pkt8 = (tofsensefPacket8_t *)buffer;
    tofsensefPacket32_t *pkt32 = (tofsensefPacket32_t *)buffer;

    while (serialRxBytesWaiting(serialPort)) {
        uint8_t c = serialRead(serialPort);

        if (bufferPtr < TOFSENSEF_PACKET_SIZE) {
            buffer[bufferPtr++] = c;
        }

        // Header checks
        if ((bufferPtr == 1) && (pkt8->frame_header != 0x57)) {
            bufferPtr = 0;
            continue;
        }
        if ((bufferPtr == 2) && (pkt8->function_mark != 0x00)) {
            bufferPtr = 0;
            continue;
        }

        // Full packet
        if (bufferPtr == TOFSENSEF_PACKET_SIZE) {
            uint8_t sum = 0;
            for (unsigned i = 0; i < TOFSENSEF_PACKET_SIZE - 1; i++) {
                sum += buffer[i];
            }

            if (pkt8->checksum == sum) {
                // Valid packet
                hasNewData = true;
                sensorData = (pkt32->dist & 0x00FFFFFF) / 10; // mm → cm
                lastProtocolActivityMs = millis();

                uint16_t qual = (pkt8->sig_strength0) | (pkt8->sig_strength1 << 8);
                bool sensorIssue = (lastSysTime >= pkt32->sys_time);
                lastSysTime = pkt32->sys_time;

                if (sensorData == 0 || qual <= TOFSENSEF_MIN_QUALITY || pkt8->status != 1 || sensorIssue) {
                    sensorData = RANGEFINDER_OUT_OF_RANGE;
                }
            }
            // Prepare for next packet
            bufferPtr = 0;
        }
    }
}

// --- Read function ---
static int32_t tofsensefRead(void)
{
    if (hasNewData) {
        hasNewData = false;
        return (sensorData > 0) ? sensorData : RANGEFINDER_OUT_OF_RANGE;
    } else {
        return RANGEFINDER_NO_NEW_DATA;
    }
}

// --- Detect function (optional, for auto-detect) ---
static bool tofsensefDetect(void)
{
    portConfig = findSerialPortConfig(FUNCTION_RANGEFINDER);
    if (!portConfig) {
        return false;
    }

    return true;
}

// --- Driver registration ---
virtualRangefinderVTable_t rangefinderTOFSenseFVtable = {
    .init = tofsensefInit,
    .update = tofsensefUpdate,
    .read = tofsensefRead,
    .detect = tofsensefDetect,
};


#endif // USE_RANGEFINDER_TOFSENSEF