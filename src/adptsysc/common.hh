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


inline std::string 
strip_space(std::string s) {
  auto is_ws = [](unsigned char c) { return std::isspace(c); };

  s.erase(s.begin(), std::find_if(s.begin(), s.end(), 
          [&](char c) {return !is_ws(static_cast<unsigned char>(c));}));

  s.erase(std::find_if(s.rbegin(), s.rend(),
          [&](char c) {return !is_ws(static_cast<unsigned char>(c));}).base(), s.end());

  return s;
}

inline std::string 
strip_point(std::string s) {
  std::string out;
  out.reserve(s.size());

  for (char c : s) {
    if (c != '.') {
      out.push_back(c);
    }
  }

  return out;
}

inline std::string 
strip_radix_prefix(std::string s) {
  if (s.size() >= 2 && s[0] == '0') {
    const char p = static_cast<char>(std::tolower(static_cast<unsigned char>(s[1])));

    if (p == 'b' || p == 'x' || p == 'o') {
      return s.substr(2);
    }
  }

  return s;
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