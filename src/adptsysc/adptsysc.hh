#pragma once

#include <adptsysc/common.hh>
#include <adptsysc/arch.hh>
#include <algorithm>
#include <cassert>
#include <cstdint>
#include <functional>
#include <iostream>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <regex>
#include <span>
#include <string>
#include <string_view>
#include <sysc/datatypes/fx/sc_fixed.h>
#include <type_traits>
#include <unistd.h>
#include <unordered_map>
#include <unordered_set>
#include <variant>
#include <vector>

namespace adptsysc {

struct LMS {
  int a;
  using FXPT_T = sc_dt::sc_fixed<16, 15, sc_dt::SC_TRN, sc_dt::SC_SAT>;
  using EVAL_T = float;
};

// Forward declarations
template <typename E> class OutputFile;

class MappedFile {
 public:
  ~MappedFile() { unmap(); }

  void unmap() {
    if (parent == nullptr && data != nullptr && size > 0) {
      munmap(data, size);
      data = nullptr;
      size = 0;
    }
  }

  void close_fd() {
    if (fd != -1) {
      close(fd);
      fd = -1;
    }
  }

  void reopen_fd(const std::string& path) {
    close_fd();
    fd = open(path.c_str(), O_RDONLY);
    if (fd == -1) {
      std::cerr << "Failed to reopen file: " << path << "\n";
    }
  }

  template <typename Context>
  MappedFile* slice(Context& ctx, std::string name, std::size_t start, std::size_t size) {
    MappedFile* mf = new MappedFile;
    mf->name = name;
    mf->data = data + start;
    mf->size = size;
    mf->parent = this;

    ctx.mf_pool.emplace_back(mf);
    return mf;
  }

  std::string_view get_contents() { return std::string_view((char*)data, size); }

  std::size_t get_offset() const {
    return parent ? (data - parent->data + parent->get_offset()) : 0;
  }

  std::string get_identifier() const {
    if (parent)
      return parent->name + ":" + std::to_string(get_offset());

    if (thin_parent)
      return thin_parent->name + ":" + name;

    return name;
  }

  std::string name;
  uint8_t* data = nullptr;
  std::size_t size = 0;
  bool given_fullpath = true;
  MappedFile* parent = nullptr;
  MappedFile* thin_parent = nullptr;
  bool is_dependency = true;
  int fd = -1;
};

MappedFile* open_file_impl(const std::string& path, std::string& error) {
  int fd = open(path.c_str(), O_RDONLY);
  if (fd == -1) {
    error = "Cannot open file: " + path + ", errno: " + std::to_string(errno);
    return nullptr;
  }

  off_t file_size = lseek(fd, 0, SEEK_END);
  if (file_size == -1) {
    error = "Failed to determine file size: " + path;
    close(fd);
    return nullptr;
  }

  void* mapped = mmap(nullptr, file_size, PROT_READ, MAP_PRIVATE, fd, 0);
  if (mapped == MAP_FAILED) {
    error = "Failed to mmap file: " + path;
    close(fd);
    return nullptr;
  }

  auto* mf = new MappedFile;
  mf->name = path;
  mf->data = reinterpret_cast<uint8_t*>(mapped);
  mf->size = file_size;
  mf->fd = fd;

  return mf;
}

template <typename Context>
MappedFile* open_file(Context& ctx, std::string path) {
  std::string error;
  MappedFile* mf = open_file_impl(path, error);
  if (!error.empty())
    Fatal(ctx) << error;

  if (mf)
    ctx.mf_pool.emplace_back(mf);
  return mf;
}

template <typename Context>
MappedFile* must_open_file(Context& ctx, std::string path) {
  MappedFile* mf = open_file(ctx, path);
  if (!mf)
    Fatal(ctx) << "cannot open " << path << "\n";
  return mf;
}

template <typename E, typename = void>
struct DataType {
  using fxptype = sc_dt::sc_fixed<16, 12>;  // fallback
  using evaltype = float;
};

template <typename E>
struct DataType<E, std::void_t<typename E::STEP_T>> {
  using fxptype = typename E::FXPT_T;
  using evaltype = typename E::EVAL_T;
};

// Context holds filter parameters and runtime state
template <typename E>
struct Context {
  using EVAL_T = typename DataType<E>::evaltype;
  using FXPT_T = typename DataType<E>::fxptype;

  Context() {
    // Initialize default filter parameters
    arg.step_size = static_cast<EVAL_T>(0.01);
    arg.filter_order = 32;
    arg.max_iters = 10000;
  }

  struct {
    EVAL_T step_size;
    std::size_t filter_order;
    std::size_t max_iters;
    bool stats = false;
    bool perf = false;
    bool trace = false;
    bool norm = true;
    bool realtime = false;
    bool fork = true;
    bool color_diagnostics = true;
    bool noinhibit_exec = false;
    bool suppress_warnings = false;
    bool fatal_warnings = false;
    bool use_scfxcast = false;
    bool use_polyphase = false;
    bool behavior_filter = true;
    std::string directory;
    std::string chroot;
    std::string rpaths;
    std::string dependency_file;
    std::string output = "a.out";
    std::string filter_type = "LMS";
    std::string in_src;
    std::string desired_sigsrc;
    i64 thread_count = 0;
  } arg;

  // Fully-expanded command line args
  std::vector<std::string_view> cmdline_args;

  // Input and output handlers
  std::unique_ptr<OutputFile<E>> output_file;
  std::vector<std::unique_ptr<MappedFile>> mf_pool;
  std::vector<std::unique_ptr<u8[]>> string_pool;

  // Runtime buffers and states
  std::vector<EVAL_T> coeffs;
  std::vector<EVAL_T> in_history;
  std::vector<EVAL_T> err_history;

  bool has_converged = false;
  bool has_error = false;

  void reset() {
    std::fill(coeffs.begin(), coeffs.end(), 0);
    in_history.clear();
    err_history.clear();
    has_converged = false;
  }
};

template <typename E>
std::string_view save_string(Context<E> &ctx, const std::string &str) {
  u8 *buf = new u8[str.size() + 1];
  memcpy(buf, str.data(), str.size());
  buf[str.size()] = '\0';
  ctx.string_pool.emplace_back(buf);
  return {(char *)buf, str.size()};
}

template <typename E> 
std::vector<std::string_view> 
expand_response_files(Context<E>& ctx, char** argv);

template <typename E> 
std::vector<std::string> 
parse_nonpositional_args(Context<E>& ctx);

}  // namespace adptsysc
