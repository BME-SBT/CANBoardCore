//
// Created by Barrow099 on 2023. 04. 24..
//

#ifndef CANBOARDCOMMON_DATATYPES_H
#define CANBOARDCOMMON_DATATYPES_H

#include "lib/inttypes.h"

template<typename T>
void to_be_bytes(T datatype, u8 *out) {
    size_t byte_count = sizeof(T);

    // we cannot use reinterpret_cast<u8*>... as that depends on the platform endianness
    for (size_t byte_nc = byte_count; byte_nc > 0; byte_nc--) {
        size_t byte_n = byte_nc - 1;
        T mask = (0xff << (byte_n * 8));
        u8 byte = (datatype & mask) >> (byte_n * 8);
        out[byte_count - byte_n - 1] = byte;
    }
}

template<typename T>
T from_be_bytes(const u8 *bytes) {
    auto byte_count = sizeof(T);
    T value = 0;
    for (u32 byte_n = 0; byte_n < byte_count; byte_n++) {
        u8 byte = bytes[byte_n];
        auto offset = byte_count - byte_n - 1;
        T v = ((T)byte << (offset * 8));
        value |= v;
    }
    return value;
}


#endif // CANBOARDCOMMON_DATATYPES_H
