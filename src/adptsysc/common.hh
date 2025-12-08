#pragma once
#include <adptsysc/integers.hh>

#include <algorithm>
#include <array>
#include <atomic>
#include <bit>
#include <bitset>
#include <cassert>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <filesystem>
#include <iostream>
#include <mutex>
#include <optional>
#include <span>
#include <sstream>
#include <string>
#include <string_view>
#include <syncstream>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#include <unordered_map>
#include <vector>

namespace adptsysc {
namespace fs = std::filesystem;

inline char *output_tmpfile = nullptr;
inline u8 *output_buffer_start = nullptr;
inline u8 *output_buffer_end = nullptr;

template <typename T, typename Compare = std::less<T>>
void update_minimum(std::atomic<T>& atomic, u64 new_val, Compare cmp = {}) {
  T old_val = atomic.load(std::memory_order_relaxed);
  while (cmp(new_val, old_val)
         && !atomic.compare_exchange_weak(
             old_val, new_val, std::memory_order_relaxed));
}

template <typename T, typename Compare = std::less<T>>
void update_maximum(std::atomic<T>& atomic, u64 new_val, Compare cmp = {}) {
  T old_val = atomic.load(std::memory_order_relaxed);
  while (cmp(old_val, new_val)
         && !atomic.compare_exchange_weak(
             old_val, new_val, std::memory_order_relaxed));
}

template <typename T>
inline void 
append(std::vector<T>& x, const auto& y) {
  x.insert(x.end(), y.begin(), y.end());
}

template <typename T>
inline std::vector<T> 
flatten(std::vector<std::vector<T>>& vec) {
  i64 size = 0;
  for (std::vector<T>& v : vec)
    size += v.size();

  std::vector<T> ret;
  ret.reserve(size);
  for (std::vector<T>& v : vec)
    append(ret, v);
  return ret;
}

template <typename T>
inline void 
remove_duplicates(std::vector<T>& vec) {
  vec.erase(std::unique(vec.begin(), vec.end()), vec.end());
}

inline i64 write_string(void* buf, std::string_view str) {
  memcpy(buf, str.data(), str.size());
  *((u8*)buf + str.size()) = '\0';
  return str.size() + 1;
}

template <typename T>
inline void 
write_vector(void* buf, const std::vector<T>& vec) {
  if (!vec.empty())
    memcpy(buf, vec.data(), vec.size() * sizeof(T));
}

inline void encode_uleb(std::vector<u8>& vec, u64 val) {
  do {
    u8 byte = val & 0x7f;
    val >>= 7;
    vec.push_back(val ? (byte | 0x80) : byte);
  } while (val);
}

inline void encode_sleb(std::vector<u8>& vec, i64 val) {
  for (;;) {
    u8 byte = val & 0x7f;
    val >>= 7;

    bool neg = (byte & 0x40);
    if ((val == 0 && !neg) || (val == -1 && neg)) {
      vec.push_back(byte);
      break;
    }
    vec.push_back(byte | 0x80);
  }
}

inline i64 write_uleb(u8* buf, u64 val) {
  i64 i = 0;
  do {
    u8 byte = val & 0x7f;
    val >>= 7;
    buf[i++] = val ? (byte | 0x80) : byte;
  } while (val);
  return i;
}

inline u64 read_uleb(u8** buf) {
  u64 val = 0;
  u8 shift = 0;
  u8 byte;
  do {
    byte = *(*buf)++;
    val |= (byte & 0x7f) << shift;
    shift += 7;
  } while (byte & 0x80);
  return val;
}

inline u64 read_uleb(u8* buf) {
  u8* tmp = buf;
  return read_uleb(&tmp);
}

inline i64 read_sleb(u8** buf) {
  u64 val = 0;
  u8 shift = 0;
  u8 byte;
  do {
    byte = *(*buf)++;
    val |= (byte & 0x7f) << shift;
    shift += 7;
  } while (byte & 0x80);
  return sign_extend(val, shift);
}

inline i64 read_sleb(u8* buf) {
  u8* tmp = buf;
  return read_sleb(&tmp);
}

inline u64 read_uleb(std::string_view* str) {
  u8* start = (u8*)str->data();
  u8* ptr = start;
  u64 val = read_uleb(&ptr);
  *str = str->substr(ptr - start);
  return val;
}

inline u64 read_uleb(std::string_view str) {
  std::string_view tmp = str;
  return read_uleb(&tmp);
}

inline i64 uleb_size(u64 val) {
  for (int i = 1; i < 9; i++)
    if (val < (1LL << (7 * i)))
      return i;
  return 9;
}

inline void overwrite_uleb(u8* loc, u64 val) {
  while (*loc & 0b1000'0000) {
    *loc++ = 0b1000'0000 | (val & 0b0111'1111);
    val >>= 7;
  }
  *loc = val & 0b0111'1111;
}

inline fs::path path_dirname(std::string_view path) {
  return fs::path(path).parent_path();
}

inline std::string path_filename(std::string_view path) {
  return fs::path(path).filename().string();
}

inline std::string path_clean(std::string_view path) {
  return fs::path(path).lexically_normal().string();
}

void get_random_bytes(u8* buf, const i64 size);

std::string get_self_path();

std::string errno_string();

void cleanup();

}  // namespace adptsysc