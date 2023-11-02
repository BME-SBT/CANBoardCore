//
// Created by Barrow099 on 2023. 04. 24..
//

#ifndef CANBOARDCOMMON_DATATYPES_H
#define CANBOARDCOMMON_DATATYPES_H

#include "lib/inttypes.h"

template <typename T> void to_be_bytes(T datatype, u8 *out) {
    size_t byte_count = sizeof(T);

    // we cannot use reinterpret_cast<u8*>... as that depends on the platform
    // endianness
    for (size_t byte_nc = byte_count; byte_nc > 0; byte_nc--) {
        size_t byte_n = byte_nc - 1;
        T mask = (0xff << (byte_n * 8));
        u8 byte = (datatype & mask) >> (byte_n * 8);
        out[byte_count - byte_n - 1] = byte;
    }
}

template <typename T> T from_be_bytes(const u8 *bytes) {
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

class DataType {
  public:
    virtual void set_u8(u8) {}
    virtual void set_u16(u16) {}
    virtual void set_u32(u32) {}
    virtual void set_u64(u64) {}
    virtual void set_i8(i8) {}
    virtual void set_i16(i16) {}
    virtual void set_i32(i32) {}
    virtual void set_i64(i64) {}
    virtual void set_f64(f64) {}
    virtual void set_f32(f32) {}
    // Place random setters here

    template <class DV> void set_value(DV value){};

    template <u8> void set_value(u8 value) { this->set_u8(value); }
    template <u16> void set_value(u16 value) { this->set_u16(value); }
    template <u32> void set_value(u32 value) { this->set_u32(value); }
    template <u64> void set_value(u64 value) { this->set_u64(value); }
    template <i8> void set_value(i8 value) { this->set_i8(value); }
    template <i16> void set_value(i16 value) { this->set_i16(value); }
    template <i32> void set_value(i32 value) { this->set_i32(value); }
    template <i64> void set_value(i64 value) { this->set_i64(value); }

    virtual u8 *get_value() = 0;
    virtual u8 len() const = 0;
};

class RPM_Datatype : public DataType {
  public:
    void set_i16(i16 v) override { value = v; }
    u8 *get_value() override {
        to_be_bytes(value, bytes);
        return this->bytes;
    }
    u8 len() const override { return 2; }

    i16 value;
    u8 bytes[2];
};

class GPS_Datatype : public DataType {
  public:
    void set_coord1(f32 v) { coord1 = v; }
    void set_coord2(f32 v) { coord2 = v; }
    u8 *get_value() override {
        to_be_bytes(*reinterpret_cast<u32*>(&coord1), bytes);
        to_be_bytes(*reinterpret_cast<u32*>(&coord2), bytes+4);
        return this->bytes;
    }
    u8 len() const override { return 8; }

    f32 coord1;
    f32 coord2;
    u8 bytes[8];
};


#endif // CANBOARDCOMMON_DATATYPES_H
