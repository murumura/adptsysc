#pragma once
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

using i8 = int8_t;
using i16 = int16_t;
using i32 = int32_t;
using i64 = int64_t;

using u8 = uint8_t;
using u16 = uint16_t;
using u32 = uint32_t;
using u64 = uint64_t;

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

}  // namespace adptsysc