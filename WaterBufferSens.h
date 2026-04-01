#pragma once
#include <Arduino.h>

namespace AvrSensorLink {

// -------------------- fixed data model --------------------

struct __attribute__((packed)) Config {
    int16_t tripB = 0;
    int16_t rearmA = 0;
};

struct __attribute__((packed)) Runtime {
    uint16_t ref = 0;
    uint16_t a = 0;
    uint16_t b = 0;
    int16_t dA = 0;
    int16_t dB = 0;
    uint8_t inhibit = 1;
    uint8_t warn = 0;
};

struct __attribute__((packed)) Blob {
    Config config{};
    Runtime runtime{};
};

static_assert(sizeof(Config) == 4, "Unexpected Config size");
static_assert(sizeof(Runtime) == 12, "Unexpected Runtime size");
static_assert(sizeof(Blob) == 16, "Unexpected Blob size");

// -------------------- protocol constants --------------------

static constexpr uint8_t kStart1 = 0xA5;
static constexpr uint8_t kStart2 = 0x5A;

static constexpr uint8_t kFlagRead  = 0x01;
static constexpr uint8_t kFlagWrite = 0x02;

// -------------------- helpers --------------------

static inline uint16_t crc16_ccitt(const uint8_t* data, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; ++i) {
        crc ^= uint16_t(data[i]) << 8;
        for (uint8_t b = 0; b < 8; ++b) {
            crc = (crc & 0x8000) ? uint16_t((crc << 1) ^ 0x1021) : uint16_t(crc << 1);
        }
    }
    return crc;
}

// -------------------- transport --------------------

class Transport {
public:
    struct Frame {
        uint8_t flags = 0;
        Blob blob{};
    };

    explicit Transport(Stream& serial) : serial_(serial) {}

    bool sendReadRequest() {
        Blob dummy{};
        return writeFrame(kFlagRead, dummy);
    }

    bool sendWriteRequest(const Blob& blob) {
        return writeFrame(kFlagWrite, blob);
    }

    // direct use, if wanted
    bool readResponse(Blob& blob) {
        Frame f;
        if (!readFrame(f)) return false;
        blob = f.blob;
        return true;
    }

    // used by Master/Slave
    bool readFrame(Frame& out);

private:
    static constexpr size_t kBodySize = 1 + sizeof(Blob) + 2; // flags + blob + crc16

    enum class State : uint8_t {
        WaitStart1,
        WaitStart2,
        ReadBody
    };

    bool writeFrame(uint8_t flags, const Blob& blob);

    Stream& serial_;
    State state_ = State::WaitStart1;
    uint8_t body_[kBodySize] = {0};
    size_t idx_ = 0;
};

// -------------------- master --------------------
class Master {
public:
    explicit Master(Stream& serial) : transport_(serial) {}

    bool requestRead(uint32_t timeoutMs = 50) {
        if (busy_) return false;
        if (!transport_.sendReadRequest()) return false;
        startWait(timeoutMs);
        return true;
    }

    bool requestWrite(const Blob& blob, uint32_t timeoutMs = 50) {
        if (busy_) return false;
        if (!transport_.sendWriteRequest(blob)) return false;
        startWait(timeoutMs);
        return true;
    }

    void loop();

    bool busy() const { return busy_; }

    bool takeResponse(Blob& out) {
        if (!hasResponse_) return false;
        out = response_;
        hasResponse_ = false;
        timedOut_ = false;
        return true;
    }

    bool consumeTimeout() {
        if (!timedOut_) return false;
        timedOut_ = false;
        hasResponse_ = false;
        return true;
    }


private:
    void startWait(uint32_t timeoutMs) {
        busy_ = true;
        hasResponse_ = false;
        timedOut_ = false;
        deadlineMs_ = millis() + timeoutMs;
    }

    Transport transport_;
    Blob response_{};
    bool busy_ = false;
    bool hasResponse_ = false;
    bool timedOut_ = false;
    uint32_t deadlineMs_ = 0;
};


// -------------------- slave --------------------

class Slave {
    /*
    Do not use the blob in any way in an ISR context or threaded context!
    This class is not designed for that and does not use any synchronization primitives. 
    The blob is meant to be used in the main loop context only, 
    and the user should ensure that no concurrent access happens 
    (e.g. by disabling interrupts around blob access if needed).
    */
public:
    explicit Slave(Stream& serial, Blob& blob)
        : transport_(serial), blob_(blob) {}

    void loop();

    bool consumeConfigChangedFlag(){
        if (newConfig_) {
            newConfig_ = false;
            return true;
        }
        return false;
    }

    Blob currentBlob() const { return blob_; }//copies not atomic, but good enough for this use case since no ISRs

    //always false since the Serial buffer is used under the hood.
    bool busy() const { return false; }

private:
    Transport transport_;
    Blob& blob_;
    bool newConfig_ = false;
};


// -------------------- client API --------------------

class AvrSensorClient {
public:
    enum class State : uint8_t {
        Idle,
        Busy,
        Timeout
    };

    explicit AvrSensorClient(Stream& serial)
        : master_(serial) {}

    void loop() {
        master_.loop();

        Blob rx;
        if (master_.takeResponse(rx)) {
            blob_ = rx;
            state_ = State::Idle;
            return;
        }

        if (master_.consumeTimeout()) {
            state_ = State::Timeout;
        }
    }

    bool read(uint32_t timeoutMs = 50) {
        if (state_ == State::Busy) return false;
        if (!master_.requestRead(timeoutMs)) return false;
        state_ = State::Busy;
        return true;
    }

    bool write(uint32_t timeoutMs = 50) {
        if (state_ == State::Busy) return false;
        if (!master_.requestWrite(blob_, timeoutMs)) return false;
        state_ = State::Busy;
        return true;
    }

    State state() const { return state_; }
    bool busy() const { return state_ == State::Busy; }

    Blob& blob() { return blob_; }
    const Blob& blob() const { return blob_; }

private:
    Master master_;
    Blob blob_{};
    State state_ = State::Idle;
};

} // namespace AvrSensorLink