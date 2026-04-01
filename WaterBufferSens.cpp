#include "WaterBufferSens.h"

namespace AvrSensorLink {

bool Transport::readFrame(Frame& out) {

    while (serial_.available() > 0) {
        const uint8_t c = uint8_t(serial_.read());
        switch (state_) {
            case State::WaitStart1:
                if (c == kStart1) state_ = State::WaitStart2;
                break;
            case State::WaitStart2:
                if (c == kStart2) {
                    idx_ = 0;
                    state_ = State::ReadBody;
                } else {
                    state_ = State::WaitStart1;
                }
                break;
            case State::ReadBody:
                body_[idx_++] = c;
                if (idx_ >= kBodySize) {
                    state_ = State::WaitStart1;
                    const uint16_t rxCrc =
                        uint16_t(body_[kBodySize - 2]) |
                        (uint16_t(body_[kBodySize - 1]) << 8);
                    const uint16_t calcCrc = crc16_ccitt(body_, kBodySize - 2);
                    if (rxCrc != calcCrc) {
                        idx_ = 0;
                        return false;
                    }
                    out.flags = body_[0];
                    memcpy(&out.blob, &body_[1], sizeof(Blob));
                    idx_ = 0;
                    return true;
                }
                break;
        }
    }
    return false;
}

bool Transport::writeFrame(uint8_t flags, const Blob& blob) {
    uint8_t buf[2 + kBodySize];
    buf[0] = kStart1;
    buf[1] = kStart2;
    buf[2] = flags;
    memcpy(&buf[3], &blob, sizeof(Blob));
    const uint16_t crc = crc16_ccitt(&buf[2], 1 + sizeof(Blob));
    buf[2 + 1 + sizeof(Blob)] = uint8_t(crc & 0xFF);
    buf[2 + 1 + sizeof(Blob) + 1] = uint8_t(crc >> 8);
    const size_t n = serial_.write(buf, sizeof(buf));
    // serial_.flush(); // not needed here
    return n == sizeof(buf);
}

void Master::loop() {
    if (!busy_) return;
    Transport::Frame frame;
    if (transport_.readFrame(frame)) {
        response_ = frame.blob;
        hasResponse_ = true;
        busy_ = false;
        return;
    }
    if ((int32_t)(millis() - deadlineMs_) >= 0) {
        timedOut_ = true;
        busy_ = false;
    }
}

void Slave::loop() {
    Transport::Frame frame;
    if (!transport_.readFrame(frame)) return;
    if (frame.flags & kFlagWrite) {
        blob_.config = frame.blob.config;
    }
    
    // On both READ and WRITE: respond with current blob.
    // For malformed/unknown flags: still return current blob.
    if (frame.flags & (kFlagRead | kFlagWrite)) {
        Blob tx(blob_);
        transport_.sendWriteRequest(tx);
    }
}

}//namespace AvrSensorLink