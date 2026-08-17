/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <string.h>

#include "platform.h"

#ifdef USE_SERIALRX_SRXL2

#include "common/crc.h"
#include "common/maths.h"
#include "common/printf.h"
#include "common/streambuf.h"

#include "drivers/nvic.h"
#include "drivers/time.h"
#include "drivers/serial.h"
#include "drivers/serial_uart.h"

#include "io/serial.h"

#include "rx/srxl2.h"
#include "rx/srxl2_types.h"
#include "io/spektrum_vtx_control.h"

#define SRXL2_DEBUG 1
#ifndef SRXL2_DEBUG
#define SRXL2_DEBUG 0
#endif

#if SRXL2_DEBUG
//void cliPrintf(const char *format, ...);
//#define DEBUG_PRINTF(format, ...) cliPrintf(format, __VA_ARGS__)
#define DEBUG_PRINTF(...) printf(__VA_ARGS__)
#else
#define DEBUG_PRINTF(...)
#endif



#define SRXL2_MAX_CHANNELS             32
#define SRXL2_FRAME_PERIOD_US   11000 // 5500 for DSMR
#define SRXL2_SAME_FRAME_US            (SRXL2_FRAME_PERIOD_US / 10)
#define SRXL2_CHANNEL_SHIFT            2
#define SRXL2_CHANNEL_CENTER           0x8000

#define SRXL2_PORT_BAUDRATE_DEFAULT    115200
#define SRXL2_PORT_BAUDRATE_HIGH       400000
#define SRXL2_PORT_OPTIONS             (SERIAL_STOPBITS_1 | SERIAL_PARITY_NO)
#define SRXL2_PORT_MODE                MODE_RXTX

#define SRXL2_REPLY_QUIESCENCE         (2 * 10 * 1000000 / SRXL2_PORT_BAUDRATE_DEFAULT) // 2 * (lastIdleTimestamp - lastReceiveTimestamp). Time taken to send 2 bytes

#define SRXL2_ID                       0xA6
#define SRXL2_U_ID_2                   0x00000000
#define SRXL2_MAX_PACKET_LENGTH        80
#define SRXL2_DEVICE_ID_BROADCAST      0xFF
#define SRXL2_DEVICE_ID_NONE           0

#define SRXL2_FRAME_TIMEOUT_US         50000

#define SRXL2_LISTEN_FOR_ACTIVITY_TIMEOUT_US 50000
#define SRXL2_SEND_HANDSHAKE_TIMEOUT_US 50000
#define SRXL2_LISTEN_FOR_HANDSHAKE_TIMEOUT_US 200000

#define SPEKTRUM_PULSE_OFFSET          988 // Offset value to convert digital data into RC pulse

typedef union {
        uint8_t raw[SRXL2_MAX_PACKET_LENGTH];
        Srxl2Header header;
} Srxl2Frame;

struct rxBuf {
    volatile unsigned len;
    Srxl2Frame packet;
};

typedef struct SRXL2Bus {
    uint8_t unitId;
    uint8_t baudRate;
    
    Srxl2State state;
    uint32_t timeoutTimestamp;
    uint32_t fullTimeoutTimestamp;
    uint32_t lastValidPacketTimestamp;
    volatile uint32_t lastReceiveTimestamp;
    volatile uint32_t lastIdleTimestamp;
    
    struct rxBuf readBuffer[2];
    struct rxBuf* readBufferPtr;
    struct rxBuf* processBufferPtr;
    volatile unsigned readBufferIdx;
    volatile bool transmittingTelemetry;
    uint8_t writeBuffer[SRXL2_MAX_PACKET_LENGTH];
    unsigned writeBufferIdx;
    
    serialPort_t *serialPort;
    uint8_t busMasterDeviceId;
    bool telemetryRequested;
} SRXL2Bus;
#define SRXL2_MAX_BUSES 2
#define SRXL2_PRIMARY_BUS 0
SRXL2Bus srxl2bus[SRXL2_MAX_BUSES] = {
    // Must set these members on init for devs > 0
    {
        .readBufferPtr = &srxl2bus[0].readBuffer[0],
        .processBufferPtr = &srxl2bus[0].readBuffer[1],
        .busMasterDeviceId = 0xFF,
    }
};

static uint8_t telemetryFrame[22];

uint8_t globalResult = 0;

static void srxl2RxWriteData(const void *data, int len, SRXL2Bus *bus);

static void srxl2SendHandshake(uint8_t destDeviceId, SRXL2Bus *bus)
{
    Srxl2HandshakeFrame response = {
        .header = {
            .id = SRXL2_ID,
            .packetType = Handshake,
            .length = sizeof(Srxl2HandshakeFrame),
        },
        .payload = {
            .sourceDeviceId = ((FlightController << 4) | bus->unitId),
            .destinationDeviceId = destDeviceId,
            .priority = 10,
            .baudSupported = bus->baudRate,
            .info = 0,
            .uniqueId = SRXL2_U_ID_2, /* this isn't very unique */
        }
    };

    srxl2RxWriteData(&response, sizeof(response), bus);
}

/* handshake protocol
    1. listen for 50ms for serial activity and go to State::Running if found, autobaud may be necessary
    2. if srxl2_unitId = 0:
            send a Handshake with destinationDeviceId = 0 every 50ms for at least 200ms
        else:
            listen for Handshake for at least 200ms
    3.  respond to Handshake as currently implemented in process if rePst received
    4.  respond to broadcast Handshake
*/

// if 50ms with not activity, go to default baudrate and to step 1

bool srxl2ProcessHandshake(const Srxl2Header* header, SRXL2Bus *bus)
{
    const Srxl2HandshakeSubHeader* handshake = (Srxl2HandshakeSubHeader*)(header + 1);
    if (handshake->destinationDeviceId == Broadcast) {
        DEBUG_PRINTF("broadcast handshake from %x on bus %d\r\n", handshake->sourceDeviceId,
                     bus - srxl2bus);
        bus->busMasterDeviceId = handshake->sourceDeviceId;

        if (handshake->baudSupported == 1) {
            serialSetBaudRate(bus->serialPort, SRXL2_PORT_BAUDRATE_HIGH);
            DEBUG_PRINTF("switching to %d baud\r\n", SRXL2_PORT_BAUDRATE_HIGH);
        }

        bus->state = Running;

        return true;
    }


    if (handshake->destinationDeviceId != ((FlightController << 4) | bus->unitId)) {
        return true;
    }

    DEBUG_PRINTF("FC handshake from %x info %x on bus %d\r\n", handshake->sourceDeviceId, handshake->info, bus - srxl2bus);

    srxl2SendHandshake(handshake->sourceDeviceId, bus);

    return true;
}

void srxl2ProcessChannelData(const Srxl2ChannelDataHeader* channelData, rxRuntimeState_t *rxRuntimeState) {
    globalResult = RX_FRAME_COMPLETE;

    if (channelData->rssi >= 0) {
        const int rssiPercent = channelData->rssi;
        setRssi(scaleRange(rssiPercent, 0, 100, 0, RSSI_MAX_VALUE), RSSI_SOURCE_RX_PROTOCOL);
    }

    //If receiver is in a connected state, and a packet is missed, the channel mask will be 0.
    if (!channelData->channelMask.u32) {
        globalResult |= RX_FRAME_DROPPED;
        return;
    }

    const uint16_t *frameChannels = (const uint16_t *) (channelData + 1);
    uint32_t channelMask = channelData->channelMask.u32;
    while (channelMask) {
        unsigned idx = __builtin_ctz (channelMask);
        uint32_t mask = 1 << idx;
        rxRuntimeState->channelData[idx] = *frameChannels++;
        channelMask &= ~mask;
    }

    // DEBUG_PRINTF("channel data: %d %d %lx\r\n", channelData->rssi, channelData->frameLosses, channelData->channelMask.u32);
}

bool srxl2ProcessControlData(const Srxl2Header* header, rxRuntimeState_t *rxRuntimeState,
                             SRXL2Bus *bus)
{
    const Srxl2ControlDataSubHeader* controlData = (Srxl2ControlDataSubHeader*)(header + 1);
    const uint8_t ownId = (FlightController << 4) | bus->unitId;
    if (controlData->replyId == ownId) {
        bus->telemetryRequested = true;
        // DEBUG_PRINTF("command: %x replyId: %x ownId: %x bus: %d\r\n", controlData->command, controlData->replyId, ownId, bus - srxl2bus);
    }

    switch (controlData->command) {
    case ChannelData:
        if ((rxRuntimeState->lastRcFrameTimeUs >= bus->lastIdleTimestamp) ||
            (bus->lastIdleTimestamp - rxRuntimeState->lastRcFrameTimeUs) < SRXL2_SAME_FRAME_US) {
            // DEBUG_PRINTF("Ignoring channel data on bus %d because good data has been retrieved %ld\r\n",
            //              bus - srxl2bus, bus->lastIdleTimestamp - rxRuntimeState->lastRcFrameTimeUs);
            break;
        }
        srxl2ProcessChannelData((const Srxl2ChannelDataHeader *) (controlData + 1), rxRuntimeState);
        break;

    case FailsafeChannelData: {
        // Because failsafe data can be out of sync with a transmitter, we ignore it for an
        // entire frame period after getting good data rather than expecting it to be within 1/10
        // of a frame period.
        if ((bus->lastIdleTimestamp - rxRuntimeState->lastRcFrameTimeUs) < SRXL2_FRAME_PERIOD_US) {
            // DEBUG_PRINTF("Ignoring failsafe data on bus %d because good data has been retrieved\r\n",
            //              bus - srxl2bus);
            break;
        }
        globalResult |= RX_FRAME_FAILSAFE;
        setRssiDirect(0, RSSI_SOURCE_RX_PROTOCOL);
        // DEBUG_PRINTF("fs channel data\r\n");
    } break;

    case VTXData: {
#if defined(USE_SPEKTRUM_VTX_CONTROL) && defined(USE_VTX_COMMON)
        Srxl2VtxData *vtxData = (Srxl2VtxData*)(controlData + 1);
        DEBUG_PRINTF("vtx data\r\n");
        DEBUG_PRINTF("vtx band: %x\r\n", vtxData->band);
        DEBUG_PRINTF("vtx channel: %x\r\n", vtxData->channel);
        DEBUG_PRINTF("vtx pit: %x\r\n", vtxData->pit);
        DEBUG_PRINTF("vtx power: %x\r\n", vtxData->power);
        DEBUG_PRINTF("vtx powerDec: %x\r\n", vtxData->powerDec);
        DEBUG_PRINTF("vtx region: %x\r\n", vtxData->region);
        // Pack data as it was used before srxl2 to use existing functions.
        // Get the VTX control bytes in a frame
        uint32_t vtxControl =   (0xE0 << 24) | (0xE0 << 8) |
                                ((vtxData->band & 0x07) << 21) |
                                ((vtxData->channel & 0x0F) << 16) |
                                ((vtxData->pit & 0x01) << 4) |
                                ((vtxData->region & 0x01) << 3) |
                                ((vtxData->power & 0x07));
        spektrumHandleVtxControl(vtxControl);
#endif
    } break;
    }

    return true;
}

static bool srxl2ProcessBindInfo(const Srxl2Header* header, rxRuntimeState_t *rxRuntimeState, SRXL2Bus *bus) {
    UNUSED(rxRuntimeState);
    
    const Srxl2BindInfoPayload* bindInfo = (Srxl2BindInfoPayload*)(header + 1);
    DEBUG_PRINTF("Bind Info: rq: %x dev: %x bindType: %x options: %x guid: %08lx%08lx uid: %lx bus: %d\r\n",
                 bindInfo->request, bindInfo->deviceId, bindInfo->bindType,
                 bindInfo->options, (uint32_t)(bindInfo->guid >> 32), (uint32_t)bindInfo->guid,
                 bindInfo->uid, bus - srxl2bus);
    if (bus != &srxl2bus[SRXL2_PRIMARY_BUS] || bindInfo->bindType == NotBound
                         || bindInfo->request != BoundDataReport)
        return true;
    Srxl2BindInfoFrame bind = {
        .header = {
            .id = SRXL2_ID,
            .packetType = BindInfo,
            .length = sizeof(Srxl2BindInfoFrame),
        },
        .payload = {
            .request = SetBindInfo,
            .bindType = bindInfo->bindType,
            .options = bindInfo->options & ~(SRXL_BIND_OPT_TELEM_TX_ENABLE | SRXL_BIND_OPT_BIND_TX_ENABLE),
            .guid = bindInfo->guid,
        }
    };
    for (int i = 0; i < SRXL2_MAX_BUSES; i++) {
        if (i != SRXL2_PRIMARY_BUS && srxl2bus[i].serialPort) {
            bind.payload.deviceId = srxl2bus[i].busMasterDeviceId;
            srxl2RxWriteData(&bind, bind.header.length, &srxl2bus[i]);
        }
    }
    return true;
}

bool srxl2ProcessPacket(const Srxl2Header* header, rxRuntimeState_t *rxRuntimeState, SRXL2Bus *bus)
{
    switch (header->packetType) {
    case Handshake:
        return srxl2ProcessHandshake(header, bus);
    case ControlData:
        return srxl2ProcessControlData(header, rxRuntimeState, bus);
    case BindInfo:
        return srxl2ProcessBindInfo(header, rxRuntimeState, bus);
    default:
        DEBUG_PRINTF("Other packet type, ID: %x on bus %d\r\n", header->packetType, bus - srxl2bus);
        break;
    }

    return false;
}

bool srxl2IsPacketValid(SRXL2Bus *bus)
{
    if (bus->processBufferPtr->packet.header.id != SRXL2_ID || bus->processBufferPtr->len != bus->processBufferPtr->packet.header.length) {
        DEBUG_PRINTF("invalid header id: %x, or length: %x received vs %x expected \r\n", bus->processBufferPtr->packet.header.id, bus->processBufferPtr->len, bus->processBufferPtr->packet.header.length);
        globalResult = RX_FRAME_DROPPED;
        return false;
    }

    const uint16_t calculatedCrc = crc16_ccitt_update(0, bus->processBufferPtr->packet.raw, bus->processBufferPtr->packet.header.length);

    //Invalid if crc non-zero
    if (calculatedCrc) {
        globalResult = RX_FRAME_DROPPED;
        DEBUG_PRINTF("crc mismatch %x\r\n", calculatedCrc);
        return false;
    }
    return true;
}

// @note assumes packet is fully there
void srxl2Process(rxRuntimeState_t *rxRuntimeState, SRXL2Bus* bus)
{
    if (!srxl2IsPacketValid(bus)) {
        return;
    }

    //Packet is valid only after ID and CRC check out
    bus->lastValidPacketTimestamp = micros();

    if (srxl2ProcessPacket(&bus->processBufferPtr->packet.header, rxRuntimeState, bus)) {
        return;
    }

    DEBUG_PRINTF("could not parse packet: %x on bus %d\r\n", bus->processBufferPtr->packet.header.packetType, bus - srxl2bus);
    globalResult = RX_FRAME_DROPPED;
}


static void srxl2DataReceive(uint16_t character, void *data)
{
    SRXL2Bus *bus = (SRXL2Bus *)data;

    bus->lastReceiveTimestamp = microsISR();

    //If the buffer len is not reset for whatever reason, disable reception
    if (bus->readBufferPtr->len > 0 || bus->readBufferIdx >= SRXL2_MAX_PACKET_LENGTH) {
        bus->readBufferIdx = 0;
        globalResult = RX_FRAME_DROPPED;
    }
    else {
        bus->readBufferPtr->packet.raw[bus->readBufferIdx] = character;
        bus->readBufferIdx++;
    }
}

static void srxl2Idle(void* data)
{
    SRXL2Bus *bus = (SRXL2Bus *)data;

    if (bus->transmittingTelemetry) { // Transmitting telemetry triggers idle interrupt as well. We dont want to change buffers then
        bus->transmittingTelemetry = false;
    }
    else if (bus->readBufferIdx == 0) { // Packet was invalid
        bus->readBufferPtr->len = 0;
    }
    else {
        bus->lastIdleTimestamp = microsISR();
        //Swap read and process buffer pointers
        if (bus->processBufferPtr == &bus->readBuffer[0]) {
            bus->processBufferPtr = &bus->readBuffer[1];
            bus->readBufferPtr = &bus->readBuffer[0];
        } else {
            bus->processBufferPtr = &bus->readBuffer[0];
            bus->readBufferPtr = &bus->readBuffer[1];
        }
        bus->processBufferPtr->len = bus->readBufferIdx;
    }

    bus->readBufferIdx = 0;
}

static uint8_t srxl2FrameStatusPerBus(rxRuntimeState_t *rxRuntimeState, SRXL2Bus *bus)
{
    UNUSED(rxRuntimeState);
    
#if SRXL2_DEBUG
    int busnum = bus - srxl2bus;
#endif
    globalResult = RX_FRAME_PENDING;

    // len should only be set after an idle interrupt (packet reception complete)
    if (bus->processBufferPtr != NULL && bus->processBufferPtr->len) {
        srxl2Process(rxRuntimeState, bus);
        bus->processBufferPtr->len = 0;
    }

    uint8_t result = globalResult;

    const uint32_t now = micros();

    switch (bus->state) {
    case Disabled: break;

    case ListenForActivity: {
        // activity detected
        if (bus->lastValidPacketTimestamp != 0) {
            // as ListenForActivity is done at default baud-rate, we don't need to change anything
            // @todo if there were non-handshake packets - go to running,
            // if there were - go to either Send Handshake or Listen For Handshake
            bus->state = Running;
        } else if (cmpTimeUs(bus->lastIdleTimestamp, bus->lastReceiveTimestamp) > 0) {
            // This means we received something invalid.
            if (bus->baudRate != 0) {
                uint32_t currentBaud = serialGetBaudRate(bus->serialPort);

                if(currentBaud == SRXL2_PORT_BAUDRATE_DEFAULT)
                    serialSetBaudRate(bus->serialPort, SRXL2_PORT_BAUDRATE_HIGH);
                else
                    serialSetBaudRate(bus->serialPort, SRXL2_PORT_BAUDRATE_DEFAULT);
                bus->lastIdleTimestamp = 0;
            }
        } else if (cmpTimeUs(now, bus->timeoutTimestamp) >= 0) {
            // @todo if there was activity - detect baudrate and ListenForHandshake

            if (bus->unitId == 0) {
                bus->state = SendHandshake;
                bus->timeoutTimestamp = now + SRXL2_SEND_HANDSHAKE_TIMEOUT_US;
                bus->fullTimeoutTimestamp = now + SRXL2_LISTEN_FOR_HANDSHAKE_TIMEOUT_US;
                bus->lastIdleTimestamp = bus->lastReceiveTimestamp + 1; // Allow transmission
                DEBUG_PRINTF("Sending first handshake to 0 on bus %d\r\n", busnum);
                serialSetBaudRate(bus->serialPort, SRXL2_PORT_BAUDRATE_DEFAULT);
                srxl2SendHandshake(SRXL2_DEVICE_ID_NONE, bus);
                result |= RX_FRAME_PROCESSING_REQUIRED;
            } else {
                bus->state = ListenForHandshake;
                bus->timeoutTimestamp = now + SRXL2_LISTEN_FOR_HANDSHAKE_TIMEOUT_US;
            }
        }
    } break;

    case SendHandshake: {
        if (bus->lastValidPacketTimestamp != 0) {
            bus->state = Running;
        } else if (cmpTimeUs(now, bus->fullTimeoutTimestamp) >= 0) {
            serialSetBaudRate(bus->serialPort, SRXL2_PORT_BAUDRATE_DEFAULT);
            DEBUG_PRINTF("case SendHandshake: switching to %d baud on bus %d\r\n", SRXL2_PORT_BAUDRATE_DEFAULT, busnum);
            bus->timeoutTimestamp = now + SRXL2_LISTEN_FOR_ACTIVITY_TIMEOUT_US;
            result = (result & ~RX_FRAME_PENDING) | RX_FRAME_FAILSAFE;

            bus->state = ListenForActivity;
            bus->lastReceiveTimestamp = 0;
        } else if (cmpTimeUs(now, bus->timeoutTimestamp) >= 0) {
            bus->timeoutTimestamp = now + SRXL2_SEND_HANDSHAKE_TIMEOUT_US;
            bus->lastIdleTimestamp = bus->lastReceiveTimestamp + 1; // Allow transmission
            DEBUG_PRINTF("Sending handshake to 0 on bus %d\r\n", busnum);
            srxl2SendHandshake(SRXL2_DEVICE_ID_NONE, bus);
            result |= RX_FRAME_PROCESSING_REQUIRED;
        }

    } break;

    case ListenForHandshake: {
        if (cmpTimeUs(now, bus->timeoutTimestamp) >= 0)  {
            serialSetBaudRate(bus->serialPort, SRXL2_PORT_BAUDRATE_DEFAULT);
            DEBUG_PRINTF("case ListenForHandshake: switching to %d baud on bus %d\r\n", SRXL2_PORT_BAUDRATE_DEFAULT, busnum);
            bus->timeoutTimestamp = now + SRXL2_LISTEN_FOR_ACTIVITY_TIMEOUT_US;
            result = (result & ~RX_FRAME_PENDING) | RX_FRAME_FAILSAFE;

            bus->state = ListenForActivity;
            bus->lastReceiveTimestamp = 0;
        }
    } break;

    case Running: {
        // frame timed out, reset state
        if (cmpTimeUs(now, bus->lastValidPacketTimestamp) >= SRXL2_FRAME_TIMEOUT_US) {
            serialSetBaudRate(bus->serialPort, SRXL2_PORT_BAUDRATE_DEFAULT);
            DEBUG_PRINTF("case Running: switching to %d baud: %ld %ld on bus %d\r\n", SRXL2_PORT_BAUDRATE_DEFAULT, now, bus->lastValidPacketTimestamp, busnum);
            bus->timeoutTimestamp = now + SRXL2_LISTEN_FOR_ACTIVITY_TIMEOUT_US;
            result = (result & ~RX_FRAME_PENDING) | RX_FRAME_FAILSAFE;

            bus->state = ListenForActivity;
            bus->lastReceiveTimestamp = 0;
            bus->lastValidPacketTimestamp = 0;
        }
    } break;
    };

    if (bus->writeBufferIdx) {
        result |= RX_FRAME_PROCESSING_REQUIRED;
    }

    if ((result & RX_FRAME_COMPLETE) && !(result & (RX_FRAME_FAILSAFE | RX_FRAME_DROPPED))) {
        rxRuntimeState->lastRcFrameTimeUs = bus->lastIdleTimestamp;
    }

    return result;
}

static uint8_t srxl2FrameStatus(rxRuntimeState_t *rxRuntimeState)
{
    uint8_t result = RX_FRAME_FAILSAFE | RX_FRAME_DROPPED;
    bool frameComplete = false;
    for (int i = 0; i < SRXL2_MAX_BUSES; i++) {
        if (srxl2bus[i].serialPort) {
            uint8_t bus_result = srxl2FrameStatusPerBus(rxRuntimeState, &srxl2bus[i]);
            // If any receiever requires frame processing, we ask for it.  If any receiver gets
            // a complete frame with no issues, we report that (ignoring drops/failsafes).
            // Otherwise, if any receiver drops or failsafes, we report that.
            result |= bus_result & RX_FRAME_PROCESSING_REQUIRED;
            if (!frameComplete) {
                if ((bus_result & (RX_FRAME_FAILSAFE | RX_FRAME_DROPPED | RX_FRAME_COMPLETE)) == RX_FRAME_COMPLETE) {
                    result &= ~(RX_FRAME_FAILSAFE | RX_FRAME_DROPPED);
                    result |= RX_FRAME_COMPLETE;
                    frameComplete = true;
                } else {
                    result |= bus_result &
                        (RX_FRAME_FAILSAFE | RX_FRAME_DROPPED | RX_FRAME_COMPLETE);
                }
            }
            result &= (~(RX_FRAME_FAILSAFE | RX_FRAME_DROPPED)) | bus_result;
        }
    }
    return result;
}

static bool srxl2ProcessFramePerBus(const rxRuntimeState_t *rxRuntimeState, SRXL2Bus* bus)
{
    UNUSED(rxRuntimeState);

    if (bus->writeBufferIdx == 0) {
        return true;
    }

    const uint32_t now = micros();

    if (cmpTimeUs(bus->lastIdleTimestamp, bus->lastReceiveTimestamp) > 0) {
        // time sufficient for at least 2 characters has passed
        if (cmpTimeUs(now, bus->lastReceiveTimestamp) > SRXL2_REPLY_QUIESCENCE) {
            bus->transmittingTelemetry = true;
            serialWriteBuf(bus->serialPort, bus->writeBuffer, bus->writeBufferIdx);
            bus->writeBufferIdx = 0;
        } else {
            DEBUG_PRINTF("not enough time to send 2 characters passed yet, %ld us since last receive, %d required\r\n", now - bus->lastReceiveTimestamp, SRXL2_REPLY_QUIESCENCE);
        }
    } else {
        DEBUG_PRINTF("still receiving a frame, %ld %ld bus %d\r\n", bus->lastIdleTimestamp, bus->lastReceiveTimestamp, bus - srxl2bus);
    }

    return true;
}

static bool srxl2ProcessFrame(const rxRuntimeState_t *rxRuntimeState)
{
    bool result = true;
    for (int i = 0; i < SRXL2_MAX_BUSES; i++) {
        if (srxl2bus[i].serialPort) {
            result &= srxl2ProcessFramePerBus(rxRuntimeState, &srxl2bus[i]);
        }
    }
    return result;
}

static float srxl2ReadRawRC(const rxRuntimeState_t *rxRuntimeState, uint8_t channelIdx)
{
    if (channelIdx >= rxRuntimeState->channelCount) {
        return 0;
    }

    return ((float)(rxRuntimeState->channelData[channelIdx] >> SRXL2_CHANNEL_SHIFT) / 16) + SPEKTRUM_PULSE_OFFSET;
}

static void srxl2RxWriteData(const void *data, int len, SRXL2Bus *bus)
{
    const uint16_t crc = crc16_ccitt_update(0, (uint8_t*)data, len - 2);
    ((uint8_t*)data)[len-2] = ((uint8_t *) &crc)[1] & 0xFF;
    ((uint8_t*)data)[len-1] = ((uint8_t *) &crc)[0] & 0xFF;

    len = MIN(len, (int)sizeof(bus->writeBuffer));
    memcpy(bus->writeBuffer, data, len);
    bus->writeBufferIdx = len;
}

void validateAndFixSrxl2Config(void)
{
    // Force half duplex
    rxConfigMutable()->halfDuplex = true;
}

#include "pg/sbus_output.h"
static void srxl2InitSerialPort(const rxConfig_t *rxConfig)
{
    const serialPortConfig_t *portConfig = findSerialPortConfig(FUNCTION_RX_SERIAL);
    if (!portConfig) {
        srxl2bus[SRXL2_PRIMARY_BUS].serialPort = NULL;
        return;
    }

    srxl2bus[SRXL2_PRIMARY_BUS].serialPort = openSerialPort(
                     portConfig->identifier,
                     FUNCTION_RX_SERIAL,
                     srxl2DataReceive,
                     &srxl2bus[SRXL2_PRIMARY_BUS],
                     SRXL2_PORT_BAUDRATE_DEFAULT,
                     SRXL2_PORT_MODE,
                     SRXL2_PORT_OPTIONS |
                     (rxConfig->serialrx_inverted ? SERIAL_INVERTED : SERIAL_NOT_INVERTED) |
                     (rxConfig->halfDuplex ? SERIAL_BIDIR : SERIAL_UNIDIR) |
                     (rxConfig->pinSwap ? SERIAL_PINSWAP : SERIAL_NOSWAP)
                 );
    if (srxl2bus[SRXL2_PRIMARY_BUS].serialPort)
        srxl2bus[SRXL2_PRIMARY_BUS].serialPort->idleCallback = srxl2Idle;

    DEBUG_PRINTF("Trying to find SRXL2 satellite 1\r\n");
    const serialPortConfig_t* portConfig2 = findSerialPortConfig(FUNCTION_SBUS_OUT);

    if (!portConfig2) {
        DEBUG_PRINTF("No port config for SRXL2 satellite 1\r\n");
        return;
    }
    DEBUG_PRINTF("Found port config for SRXL2 satellite 1\r\n");

    srxl2bus[1].serialPort = openSerialPort(
                     portConfig2->identifier,
                     FUNCTION_SBUS_OUT,
                     srxl2DataReceive,
                     &srxl2bus[1],
                     SRXL2_PORT_BAUDRATE_DEFAULT,
                     SRXL2_PORT_MODE,
                     SRXL2_PORT_OPTIONS |
                     (sbusOutConfig()->inverted ? SERIAL_INVERTED : SERIAL_NOT_INVERTED) |
                     SERIAL_BIDIR |
                     (sbusOutConfig()->pinSwap ? SERIAL_PINSWAP : SERIAL_NOSWAP));
    if (srxl2bus[1].serialPort)
        srxl2bus[1].serialPort->idleCallback = srxl2Idle;
    DEBUG_PRINTF("Found serial port for SRXL2 satellite 1: %s\r\n", srxl2bus[1].serialPort ? "Yes" : "No");
}

void srxl2RxEarlyInit(const rxConfig_t *rxConfig)
{
    // Get full size receivers to switch to SRXL2 mode. This must be done very shortly (less than
    // 200ms) after receiver power-on, or the receiver port (typically port "1") will revert to
    // PWM.  This is only done if our own unit ID is 0 (as specified in the SRXL2 protocol)
    //
    // This does not implement the entire handshake algorithm, because that could take a long
    // time and delay initialization -- e.g. the AR6610T polls 10 devices, taking about 30ms
    // each.
    //
    // Instead, it implements only the first 2 steps of the algorithm.
    //
    // 1) Wait 50ms for activity on the line.  If there is any, just exit and do the full
    // handshake at startup.  Don't validate packets since we may be at the wrong baud rate.
    //
    // 2) Send up to three handshake packets to device 0, 50ms apart.  If we see a valid packet
    //    on the bus at any point, exit.
    //
    // When full initialization occurs, we will re-sync as if communication was lost.
    srxl2bus[SRXL2_PRIMARY_BUS].unitId = rxConfig->srxl2_unit_id;
    srxl2bus[SRXL2_PRIMARY_BUS].baudRate = rxConfig->srxl2_baud_fast;
    if (srxl2bus[SRXL2_PRIMARY_BUS].unitId != 0 || rxConfig->serialrx_provider != SERIALRX_SRXL2)
        return;
    srxl2InitSerialPort(rxConfig);
    if (!srxl2bus[SRXL2_PRIMARY_BUS].serialPort) {
        return;
    }
    DEBUG_PRINTF("Running srxl2RxEarlyInit\r\n");
    uint32_t start_micros = micros();
    uint32_t now = start_micros;
    while ((now - start_micros) < 50000) {
        if (srxl2bus[SRXL2_PRIMARY_BUS].lastReceiveTimestamp > 0) {
            // Any activity on the bus is sufficient to skip sending the initial handshake
            // packets.
            DEBUG_PRINTF("Received something, allow regular algo to take it\r\n");
            return;
        }
        now = micros();
    }

    ++srxl2bus[SRXL2_PRIMARY_BUS].lastIdleTimestamp; // ProcessFrame won't send data if srxl2bus[SRXL2_PRIMARY_BUS].lastIdleTimestamp = srxl2bus[SRXL2_PRIMARY_BUS].lastReceiveTimestamp
    for (int i = 0; i < 3; i++) {
        // Since we receive our own data, srxl2bus[SRXL2_PRIMARY_BUS].lastReceiveTimestamp will be set forward whenever we
        // transmit, and since we don't count the associated idle, prevent a subsequent
        // transmission.
        srxl2bus[SRXL2_PRIMARY_BUS].lastReceiveTimestamp = 0;
        DEBUG_PRINTF("srlx2RxEarlyInit: Sending handshake to 0 (%d)\r\n", i);
        srxl2SendHandshake(SRXL2_DEVICE_ID_NONE, &srxl2bus[SRXL2_PRIMARY_BUS]);
        start_micros = now;
        while ((now - start_micros) < 50000) {
            if (srxl2bus[SRXL2_PRIMARY_BUS].writeBufferIdx) {
                // This writes the handshake packet.
                srxl2ProcessFramePerBus(NULL, &srxl2bus[SRXL2_PRIMARY_BUS]);
            }
            if (srxl2bus[SRXL2_PRIMARY_BUS].processBufferPtr != NULL && srxl2bus[SRXL2_PRIMARY_BUS].processBufferPtr->len) {
                bool packetIsValid = srxl2IsPacketValid(&srxl2bus[SRXL2_PRIMARY_BUS]);
                srxl2bus[SRXL2_PRIMARY_BUS].processBufferPtr->len = 0;
                if (packetIsValid) {
                    // We don't process this packet because we're not fully initialized.
                    DEBUG_PRINTF("Exiting srxl2RxEarlyInit successfully\r\n");
                    return;
                }
            }
            now = micros();
        }
    }
    DEBUG_PRINTF("Exiting srxl2RxEarlyInit init without seeing activity\r\n");
}

bool srxl2RxInit(const rxConfig_t *rxConfig, rxRuntimeState_t *rxRuntimeState)
{
    static uint16_t channelData[SRXL2_MAX_CHANNELS];
    for (size_t i = 0; i < SRXL2_MAX_CHANNELS; ++i) {
        channelData[i] = SRXL2_CHANNEL_CENTER;
    }
    for (size_t i = 0; i < SRXL2_MAX_BUSES; ++i) {
      srxl2bus[i].readBufferPtr = &srxl2bus[i].readBuffer[0];
      srxl2bus[i].processBufferPtr = &srxl2bus[i].readBuffer[1];
      srxl2bus[i].busMasterDeviceId = 0xFF;
      srxl2bus[i].unitId = 1;
      srxl2bus[i].baudRate = rxConfig->srxl2_baud_fast;
    }

    srxl2bus[SRXL2_PRIMARY_BUS].unitId = rxConfig->srxl2_unit_id;

    rxRuntimeState->channelData = channelData;
    rxRuntimeState->channelCount = SRXL2_MAX_CHANNELS;
    rxRuntimeState->rxRefreshRate = SRXL2_FRAME_PERIOD_US;

    rxRuntimeState->rcReadRawFn = srxl2ReadRawRC;
    rxRuntimeState->rcFrameStatusFn = srxl2FrameStatus;
    rxRuntimeState->rcFrameTimeUsFn = rxFrameTimeUs;
    rxRuntimeState->rcProcessFrameFn = srxl2ProcessFrame;

    // Serial ports may have been initialized in srxl2EarlyInit()
    if (!srxl2bus[SRXL2_PRIMARY_BUS].serialPort) {
        srxl2InitSerialPort(rxConfig);
    }
    if (!srxl2bus[SRXL2_PRIMARY_BUS].serialPort) {
        return false;
    }

    for (size_t i = 0; i < SRXL2_MAX_BUSES; ++i) {
        if (srxl2bus[i].serialPort) {
            srxl2bus[i].state = ListenForActivity;
            srxl2bus[i].timeoutTimestamp = micros() + SRXL2_LISTEN_FOR_ACTIVITY_TIMEOUT_US;
        }
    }

    if (rssiSource == RSSI_SOURCE_NONE) {
        rssiSource = RSSI_SOURCE_RX_PROTOCOL;
    }

    return (bool)srxl2bus[SRXL2_PRIMARY_BUS].serialPort;
}

bool srxl2RxIsActive(void)
{
    return srxl2bus[SRXL2_PRIMARY_BUS].serialPort;
}

bool srxl2TelemetryRequested(void)
{
    return srxl2bus[SRXL2_PRIMARY_BUS].telemetryRequested;
}

void srxl2InitializeFrame(sbuf_t *dst)
{
    dst->ptr = telemetryFrame;
    dst->end = ARRAYEND(telemetryFrame);

    sbufWriteU8(dst, SRXL2_ID);
    sbufWriteU8(dst, TelemetrySensorData);
    sbufWriteU8(dst, ARRAYLEN(telemetryFrame));
    sbufWriteU8(dst, srxl2bus[SRXL2_PRIMARY_BUS].busMasterDeviceId);
}

void srxl2FinalizeFrame(sbuf_t *dst)
{
  sbufSwitchToReader(dst, telemetryFrame);
  // Include 2 additional bytes of length since we're letting the srxl2RxWriteData function add the CRC in
  srxl2RxWriteData(sbufPtr(dst), sbufBytesRemaining(dst) + 2, &srxl2bus[SRXL2_PRIMARY_BUS]);
  srxl2bus[SRXL2_PRIMARY_BUS].telemetryRequested = false;
}

void srxl2Bind(void)
{
    const size_t length = sizeof(Srxl2BindInfoFrame);

    Srxl2BindInfoFrame bind = {
        .header = {
            .id = SRXL2_ID,
            .packetType = BindInfo,
            .length = length
        },
        .payload = {
            .request = EnterBindMode,
            .deviceId = srxl2bus[SRXL2_PRIMARY_BUS].busMasterDeviceId,
            .bindType = DMSX_11ms,
            .options = SRXL_BIND_OPT_TELEM_TX_ENABLE | SRXL_BIND_OPT_BIND_TX_ENABLE,
        }
    };

    srxl2RxWriteData(&bind, length, &srxl2bus[SRXL2_PRIMARY_BUS]);
}

#endif
