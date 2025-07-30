#pragma once

#include <bit>
#include <cassert>
#include <cstdint>

namespace adptsysc {

template <typename T, bool IsLE, int SZ = sizeof(T)>
class Integer {
 public:
  constexpr Integer() = default;

  constexpr Integer(T v) { store(v); }

  constexpr operator T() const { return load(); }

  Integer& operator=(T v) {
    store(v);
    return *this;
  }

  Integer& operator++() { return *this = *this + 1; }

  Integer operator++(int) {
    Integer x = *this;
    ++*this;
    return x;
  }

  Integer& operator--() { return *this = *this - 1; }

  Integer operator--(int) {
    Integer x = *this;
    --*this;
    return x;
  }

  Integer& operator+=(T v) { return *this = *this + v; }

  Integer& operator-=(T v) { return *this = *this - v; }

  Integer& operator&=(T v) { return *this = *this & v; }

  Integer& operator|=(T v) { return *this = *this | v; }

 private:
  constexpr T load() const {
    T v = 0;

    for (int i = 0; i < SZ; i++) {
      int j = IsLE ? i : (SZ - i - 1);
      v |= (T)buf[j] << (i * 8);
    }
    return v;
  }

  constexpr void store(T v) {
    for (int i = 0; i < SZ; i++) {
      int j = IsLE ? i : (SZ - i - 1);
      buf[j] = v >> (i * 8);
    }
  }

  uint8_t buf[SZ];
};

using u8 = uint8_t;
using u16 = uint16_t;
using u32 = uint32_t;
using u64 = uint64_t;

using i8 = int8_t;
using i16 = int16_t;
using i32 = int32_t;
using i64 = int64_t;
using il16 = Integer<i16, true>;
using il32 = Integer<i32, true>;
using il64 = Integer<i64, true>;

using ul16 = Integer<u16, true>;
using ul24 = Integer<u32, true, 3>;
using ul32 = Integer<u32, true>;
using ul64 = Integer<u64, true>;

using ib16 = Integer<i16, false>;
using ib32 = Integer<i32, false>;
using ib64 = Integer<i64, false>;

using ub16 = Integer<u16, false>;
using ub24 = Integer<u32, false, 3>;
using ub32 = Integer<u32, false>;
using ub64 = Integer<u64, false>;

inline u64 align_to(u64 val, u64 align) {
  if (align == 0)
    return val;
  assert(std::has_single_bit(align));
  return (val + align - 1) & ~(align - 1);
}

inline u64 align_down(u64 val, u64 align) {
  assert(std::has_single_bit(align));
  return val & ~(align - 1);
}

inline u64 bit(u64 val, i64 pos) {
  return (val >> pos) & 1;
};

// Returns [hi:lo] bits of val.
inline u64 bits(u64 val, u64 hi, u64 lo) {
  return (val >> lo) & ((1LL << (hi - lo + 1)) - 1);
}

// Cast val to a signed N bit integer.
// For example, sign_extend(x, 32) == (i32)x for any integer x.
inline i64 sign_extend(u64 val, i64 n) {
  return (i64)(val << (64 - n)) >> (64 - n);
}

inline bool is_int(u64 val, i64 n) {
  return sign_extend(val, n) == val;
}

}  // namespace adptsysc
