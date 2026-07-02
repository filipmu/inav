#include <stdbool.h>
#include <stdint.h>
#include "platform.h"

#if defined(USE_RANGEFINDER_TOFSENSEF)

#include "build/build_config.h"
#include "build/debug.h"

#include "io/serial.h"

#include "navigation/navigation_pos_estimator_private.h"


#include "drivers/rangefinder/rangefinder_virtual.h"
#include "drivers/time.h"
//#include "drivers/serial.h"
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


#define TOFSENSEF_PACKET_SIZE sizeof(tofsensefPacket8_t)
#define TOFSENSEF_MIN_QUALITY 20
#define TOFSENSEF_CONSECUTIVE_VALID_DATA 3
#define TOFSENSEF_MAX_DISTANCE_CM 1000 // 10 meters, adjust as needed

// --- Driver state ---
static serialPort_t * serialPort = NULL;
static serialPortConfig_t * portConfig = NULL;
static uint8_t buffer[TOFSENSEF_PACKET_SIZE];
static unsigned bufferPtr = 0;
static bool hasNewData = false;
static uint8_t consecutive_valid_data = TOFSENSEF_CONSECUTIVE_VALID_DATA;
static int32_t sensorData = RANGEFINDER_NO_NEW_DATA;


// --- Init function ---
static void tofsensefInit(void)
{
    if (!portConfig) return;
    serialPort = openSerialPort(portConfig->identifier, FUNCTION_RANGEFINDER, NULL, NULL, 115200, MODE_RX, SERIAL_NOT_INVERTED);
    if (!serialPort) return;
    bufferPtr = 0;
    hasNewData = false;
    sensorData = RANGEFINDER_NO_NEW_DATA;
    consecutive_valid_data = TOFSENSEF_CONSECUTIVE_VALID_DATA;
}




// --- Update function (called periodically) ---
static void tofsensefUpdate(void)
{
    while (serialRxBytesWaiting(serialPort) > 0) {
        uint8_t c = serialRead(serialPort);

        // Add to buffer if space available
        if (bufferPtr < TOFSENSEF_PACKET_SIZE) {
            buffer[bufferPtr++] = c;
        }

        // Only check header if minimum needed bytes buffered
        if (bufferPtr == 1 && buffer[0] != 0x57) {
            bufferPtr = 0;
            continue;
        }
        if (bufferPtr == 2 && buffer[1] != 0x00) {
            bufferPtr = 0;
            continue;
        }

        // Wait for a full packet
        if (bufferPtr == TOFSENSEF_PACKET_SIZE) {
            // Verify checksum
            uint8_t sum = 0;
            for (unsigned i = 0; i < TOFSENSEF_PACKET_SIZE - 1; i++) {
                sum += buffer[i];
            }

            if (buffer[TOFSENSEF_PACKET_SIZE - 1] == sum) {
                // Only now overlay the struct!
                tofsensefPacket8_t *pkt = (tofsensefPacket8_t *)buffer;

                // Extract distance from 3 bytes (LSB first)
                uint32_t rawDist = ((uint32_t)pkt->dist2 << 16) | ((uint32_t)pkt->dist1 << 8) | ((uint32_t)pkt->dist0);
                int32_t dist_cm = (rawDist & 0x00FFFFFF) / 10;

                // Extract quality
                uint16_t qual = ((uint16_t)pkt->sig_strength1 << 8) | pkt->sig_strength0;


                // Check validity by all flags
                if (dist_cm == 0 || qual <= TOFSENSEF_MIN_QUALITY || pkt->status != 1 || posEstimator.est.pos.z > TOFSENSEF_MAX_DISTANCE_CM) {
                    sensorData = RANGEFINDER_OUT_OF_RANGE;
                    consecutive_valid_data = TOFSENSEF_CONSECUTIVE_VALID_DATA;
                } else if(consecutive_valid_data > 0) { //check that we have received a few valid measurements in a row before reporting a valid distance
                    consecutive_valid_data--;
                } else { // we have received enough valid measurements in a row, report the distance
                    sensorData = dist_cm;
                }
                hasNewData = true;

            }

            // Reset buffer for next packet regardless
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
    } 
    return RANGEFINDER_NO_NEW_DATA;

}

// --- Detect function (optional, for auto-detect) ---
static bool tofsensefDetect(void)
{
    portConfig = findSerialPortConfig(FUNCTION_RANGEFINDER);
    return portConfig != NULL;
}

// --- Driver registration ---
virtualRangefinderVTable_t rangefinderTOFSenseFVtable = {
    .init = tofsensefInit,
    .update = tofsensefUpdate,
    .read = tofsensefRead,
    .detect = tofsensefDetect,
};


#endif // USE_RANGEFINDER_TOFSENSEF