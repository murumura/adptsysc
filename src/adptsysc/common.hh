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