#pragma once

#include <adptsysc/common.hh>
#include <adptsysc/integers.hh>
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
#include <type_traits>
#include <unistd.h>
#include <unordered_map>
#include <unordered_set>
#include <variant>
#include <vector>

namespace adptsysc {

// Forward declarations
template <typename E>
class InputFile;

template <typename E>
class OutputFile;

// Context holds filter parameters and runtime state
template <typename E>
struct Context {
  using STEP_T
      = std::conditional_t<requires { typename E::STEP_T; },  // Check if E::STEP_T exists
          typename E::STEP_T,  // Use E::STEP_T if it exists
          float                // Fallback to float
          >;

  Context() {
    // Initialize default filter parameters
    arg.step_size = static_cast<STEP_T>(0.01);
    arg.filter_order = 32;
    arg.max_iters = 10000;
  }

  struct {
    STEP_T step_size;
    size_t filter_order;
    size_t max_iters;
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

  // Input and output handlers
  std::vector<InputFile<E>*> inputs;
  std::unique_ptr<OutputFile<E>> output_file;

  // Runtime buffers and states
  std::vector<E> coeffs;
  std::vector<E> in_history;
  std::vector<E> err_history;

  bool has_converged = false;
  bool has_error = false;

  void checkpoint() {
    std::cout << "Checkpoint reached: step_size=" << arg.step_size
              << " filter_order=" << arg.filter_order << "\n";
  }

  void reset() {
    std::fill(coeffs.begin(), coeffs.end(), 0);
    in_history.clear();
    err_history.clear();
    has_converged = false;
  }
};

// InputFile represents input signal or config sources
template <typename E>
class InputFile {
 public:
  InputFile(Context<E>&, const std::string& filename) : filename(filename) {}

  InputFile() : filename("<internal>") {}

  virtual ~InputFile() = default;

  virtual std::span<E> get_data(Context<E>& ctx, size_t n_samples) = 0;

  virtual void resolve_symbols(Context<E>& ctx) = 0;

  virtual void mark_live_objects(
      Context<E>& ctx, std::function<void(InputFile<E>*)> feeder)
      = 0;

  std::string filename;
  bool is_realtime_stream = false;
  bool is_config_file = false;

  std::vector<E> in_samples;
  std::vector<std::string> input_labels;

  bool enable = true;
};

// File-based signal input
template <typename E>
class SignalFile : public InputFile<E> {
 public:
  SignalFile(Context<E>& ctx, const std::string& filename, const std::vector<E>& data)
      : InputFile<E>(ctx, filename) {
    this->in_samples = data;
  }

  std::span<E> get_data(Context<E>& ctx, size_t n_samples) override {
    if (n_samples > this->in_samples.size())
      n_samples = this->in_samples.size();
    return std::span<E>(this->in_samples.data(), n_samples);
  }

  void resolve_symbols(Context<E>& ctx) override {
    if (this->in_samples.size() < ctx.arg.filter_order) {
      Error(ctx) << "Input signal too short: " << this->in_samples.size()
                 << " samples, but filter order is " << ctx.arg.filter_order << "\n";
    }
  }

  void mark_live_objects(
      Context<E>& ctx, std::function<void(InputFile<E>*)> feeder) override {
    feeder(this);
  }
};

// OutputFile handles traces and filtered results
template <typename E>
class OutputFile {
 public:
  OutputFile(const std::string& filename) : filename(filename) {}

  std::vector<E> fltr_output;
  std::vector<E> err_trace;
  std::vector<std::vector<E>> coeff_history;

  void write_trace(Context<E>& ctx) {
    if (ctx.has_error) {
      Error(ctx) << "Writing trace to " << filename << "\n";
    } else {
      std::cout << "Writing trace to " << filename << "\n";
    }

    std::cout << "Error Trace: ";
    for (const auto& e : err_trace)
      std::cout << e << " ";
    std::cout << "\n";
  }

  void write_output(Context<E>& ctx) {
    if (ctx.has_error) {
      Error(ctx) << "Writing output to " << filename << "\n";
    } else {
      std::cout << "Filtered Output: ";
      for (const auto& y : fltr_output)
        std::cout << y << " ";
      std::cout << "\n";
    }
  }

  std::string filename;
  bool log_error = true;
  bool log_coeffs = true;
};

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

  size_t get_offset() const {
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

}  // namespace adptsysc
