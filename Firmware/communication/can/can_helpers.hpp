#pragma once

#include <stdint.h>
#include <algorithm>
#include <cstring>
#include <iterator>

struct can_Message_t {
    uint32_t id = 0x000;  // 11-bit max is 0x7ff, 29-bit max is 0x1FFFFFFF
    bool isExt = false;
    bool rtr = false;
    uint8_t len = 8;
    uint8_t buf[8] = {0, 0, 0, 0, 0, 0, 0, 0};
} ;

struct can_Signal_t {
    const uint8_t startBit;
    const uint8_t length;
    const bool isIntel;
    const float factor;
    const float offset;
};

struct can_Cyclic_t {
    uint32_t cycleTime_ms;
    uint32_t lastTime_ms;
};

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wstrict-aliasing" // Make sure to check these functions on your system
template <typename T>
constexpr T can_getSignal(const can_Message_t& msg, const uint8_t startBit, const uint8_t length, const bool isIntel) {
    uint64_t mask = length < 64 ? (1ULL << length) - 1ULL : -1ULL;

    uint64_t tempVal = *(reinterpret_cast<const uint64_t*>(msg.buf));
    if (isIntel) {
        tempVal = (tempVal >> startBit) & mask;
    } else {
        tempVal = __builtin_bswap64 (tempVal);
        tempVal = (tempVal >> (64 - startBit - length)) & mask;
    }

    return *(reinterpret_cast<T*>(&tempVal));
}

template <typename T>
constexpr void can_setSignal(can_Message_t& msg, const T& val, const uint8_t startBit, const uint8_t length, const bool isIntel) {
    union aliastype {
        aliastype() : valAsBits(0){}
        T tempVal;
        uint64_t valAsBits;
    };

    const uint64_t mask = length < 64 ? (1ULL << length) - 1ULL : -1ULL;
    uint64_t data = *(reinterpret_cast<const uint64_t*>(msg.buf));

    aliastype valAlias;
    valAlias.tempVal = val;
    valAlias.valAsBits &= mask;

    if (isIntel) {
        data &= ~(mask << startBit);
        data |= valAlias.valAsBits << startBit;

        *(reinterpret_cast<uint64_t*>(msg.buf)) = data;
    } else {
        data = __builtin_bswap64 (data);

        data &= ~(mask << (64 - startBit - length));
        data |= valAlias.valAsBits << (64 - startBit - length);

        data = __builtin_bswap64 (data);
        *(reinterpret_cast<uint64_t*>(msg.buf)) = data;

    }
}
#pragma GCC diagnostic pop

template<typename T>
void can_setSignal(can_Message_t& msg, const T& val, const uint8_t startBit, const uint8_t length, const bool isIntel, const float factor, const float offset) {
    T scaledVal = static_cast<T>((val - offset) / factor);
    can_setSignal<T>(msg, scaledVal, startBit, length, isIntel);
}

template<typename T>
float can_getSignal(can_Message_t msg, const uint8_t startBit, const uint8_t length, const bool isIntel, const float factor, const float offset) {
    T retVal = can_getSignal<T>(msg, startBit, length, isIntel);
    return (retVal * factor) + offset;
}

template <typename T>
float can_getSignal(can_Message_t msg, const can_Signal_t& signal) {
    return can_getSignal<T>(msg, signal.startBit, signal.length, signal.isIntel, signal.factor, signal.offset);
}

template <typename T>
void can_setSignal(can_Message_t& msg, const T& val, const can_Signal_t& signal) {
    can_setSignal(msg, val, signal.startBit, signal.length, signal.isIntel, signal.factor, signal.offset);
}