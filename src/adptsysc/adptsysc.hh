#pragma once

#include <adptsysc/common.hh>
#include <adptsysc/arch.hh>
#include <adptsysc/config.hh>
#include <adptsysc/object.hh>
#include <algorithm>
#include <cassert>
#include <cstdint>
#include <deque>
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
#include <sys/stat.h>
#include <tlm>
#include <tlm_utils/simple_target_socket.h>

namespace adptsysc {

// Forward declarations
template <typename E> class  OutputFile;
template <typename E> struct Context;

class MappedFile;

template <typename E>
class Out {
 public:
  Out(Context<E> &ctx) {}

  template <typename T>
  Out& operator<<(T&& val) {
    out << std::forward<T>(val);
    return *this;
  }

 private:
  std::osyncstream out{std::cout};
};

template <typename E>
class Fatal {
 public:
  Fatal(Context<E> &ctx);

  [[noreturn]] ~Fatal();

  template <typename T>
  Fatal& operator<<(T&& val) {
    out << std::forward<T>(val);
    return *this;
  }

 private:
  std::osyncstream out{std::cerr};
};

template <typename E>
class Error {
 public:
  Error(Context<E> &ctx);

  template <typename T>
  Error& operator<<(T&& val) {
    out << std::forward<T>(val);
    return *this;
  }

 private:
  std::osyncstream out{std::cerr};
};

template <typename E>
class Warn {
 public:
  Warn(Context<E> &ctx);

  template <typename T>
  Warn& operator<<(T&& val) {
    if (out)
      *out << std::forward<T>(val);
    return *this;
  }

 private:
  std::optional<std::osyncstream> out;
};

template <typename E, typename = void>
struct DataType {
  using Fxpt_T = sc_dt::sc_fixed<16, 12>;  // fallback
  using Eval_T = float;
};

template <typename E>
struct DataType<E, std::void_t<typename E::Fxpt_T>> {
  using Fxpt_T = typename E::Fxpt_T;
  using Eval_T = typename E::Eval_T;
};

// Context holds filter parameters and runtime state
template <typename E>
struct Context {
  using Eval_T = typename DataType<E>::Eval_T;
  using Fxpt_T = typename DataType<E>::Fxpt_T;

  Context() {}

  struct {
    bool color_diagnostics = true;
    bool noinhibit_exec = false;
    bool suppress_warnings = false;
    bool fatal_warnings = false;
    bool fixedpoint_eval = false;
    bool use_polyphase = false;
    bool behavior_filter = true;
    bool out_shared = false;
    int  filler = -1;   // -1 = no filler
    bool oformat_binary = false;
    bool verbose = true;
    bool quick_exit = true;
    bool oformat_hex = false;
    bool trace_enabled = false; // Create VCD trace file
    bool run_testbench = false;
    std::string directory;
    std::string chroot;
    std::string rpaths;
    std::string dependency_file;
    std::string load_file;
    std::string text_loadfile;
    int load_offset = -1;
    std::string output;
    std::string text_output;
    std::string filter_type = "LMSArch";
    std::string_view emulation;
    i64 thread_count = 0;
    int mem_read_lat  = 0;   // cycles of RAM read latency
    int mem_write_lat = 0;   // cycles of RAM write latency
  } arg;

  void checkpoint() {
    if (has_error) {
      cleanup();
      _exit(1);
    }
  }

  // Fully-expanded command line args
  std::vector<std::string_view> cmdline_args;
  std::vector<std::unique_ptr<MappedFile>> mf_pool;
  std::vector<std::unique_ptr<u8[]>> string_pool;
  bool has_error = false;

  // Output buffer
  std::unique_ptr<OutputFile<E>> output_file;
  u8 *buf = nullptr;
  bool overwrite_output_file = false;


  void reset() {}
};

class MappedFile {
 public:
  ~MappedFile() { unmap(); }

  void unmap() {
    if (data != nullptr && size > 0) {
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

  template <typename E>
  MappedFile* slice(Context<E>& ctx, std::string name, 
    std::size_t start, std::size_t size) {
    MappedFile* mf = new MappedFile;
    mf->name = name;
    mf->data = data + start;
    mf->size = size;
    ctx.mf_pool.emplace_back(mf);
    return mf;
  }

  std::string_view get_contents() { 
    return std::string_view((char*)data, size); 
  }

  std::size_t get_offset() const {
    return 0;
  }

  std::string get_identifier() const {
    return name;
  }

  std::string name;
  uint8_t* data = nullptr;
  std::size_t size = 0;
  bool given_fullpath = true;
  bool is_dependency = true;
  int fd = -1;
};

MappedFile* open_file_impl(const std::string& path, std::string& error);

template <typename E>
MappedFile *open_file(Context<E> &ctx, std::string path) {
  if (path.starts_with('/') && !ctx.arg.chroot.empty())
    path = ctx.arg.chroot + "/" + path_clean(path);

  std::string error;
  MappedFile *mf = open_file_impl(path, error);
  if (!error.empty())
    Fatal(ctx) << error;

  if (mf)
    ctx.mf_pool.emplace_back(mf);
  return mf;
}

template <typename E>
MappedFile *must_open_file(Context<E> &ctx, std::string path) {
  MappedFile *mf = open_file(ctx, path);
  if (!mf)
    Fatal(ctx) << "cannot open " << path << ": " << errno_string();
  return mf;
}

template <typename E>
class OutputFile {
public:
  static std::unique_ptr<OutputFile<E>>
  open(Context<E> &ctx, std::string path, i64 filesize, mode_t perm);

  virtual void close(Context<E> &ctx) = 0;
  virtual ~OutputFile() = default;

  u8 *buf = nullptr;
  std::vector<u8> buf2;
  std::string path;
  int fd = -1;
  i64 filesize = 0;
  bool is_mmapped = false;

protected:
  OutputFile(std::string path, i64 filesize, bool is_mmapped)
    : path(path), filesize(filesize), is_mmapped(is_mmapped) {}
};

template <typename E>
class MallocOutputFile : public OutputFile<E> {
public:
  MallocOutputFile(Context<E> &ctx, std::string path, 
                   i64 filesize, mode_t perm)
    : OutputFile<E>(path, filesize, false), ptr(new u8[filesize]), perm(perm) {
    this->buf = ptr.get();
  }

  void close(Context<E> &ctx) override {
    FILE *fp;

    if (this->path == "-") {
      // Write to standard output
      fp = stdout;
    } else {
      // Write to regular file
      i64 fd = ::open(this->path.c_str(), O_RDWR | O_CREAT, perm);
      if (fd == -1)
        Fatal(ctx) << "cannot open " << this->path << ": " << errno_string();
      fp = fdopen(fd, "w");
    }

    fwrite(this->buf, this->filesize, 1, fp);
    if (!this->buf2.empty())
      fwrite(this->buf2.data(), this->buf2.size(), 1, fp);
    fclose(fp);
  }

private:
  std::unique_ptr<u8[]> ptr;
  mode_t perm;
};

template <typename E>
class SyscMemory : public ObjectWithMutableHyperparams, 
                   public sc_core::sc_module {
public:
  using T = typename E::Eval_T;

  // Static creation function following OutputFile pattern
  static std::unique_ptr<SyscMemory<E>> 
  create(Context<E> &ctx, sc_core::sc_module_name name,
         std::size_t size = 0, T* initptr = nullptr);

  // Static function for testbed simulation
  static bool run_testbench(Context<E> &ctx);

  tlm_utils::simple_target_socket<SyscMemory> targ_socket;
  
  sc_core::sc_in<bool> clk;
  sc_core::sc_in<bool> reset;
  sc_core::sc_in<bool> ram_en;  
  sc_core::sc_in<bool> write_en;     
  sc_core::sc_in<bool> in_valid;     
  sc_core::sc_out<bool> out_ready; 

  sc_core::sc_in<std::size_t> addr;
  sc_core::sc_in<T> in_data;
  sc_core::sc_out<T> out_data;
  
  void update_hyperparams(const json& params) override;
  json hyperparams() const override;
  
  void allocate(std::size_t size);
  void allocate(const std::shared_ptr<ParametricObject<T>>& target);
  
  T* data() const { return mem_data.get(); }
  std::size_t size() const { return mem_size; }
  
  void mem_reset(Context<E> &ctx);

  bool is_valid_addr(const std::size_t addr) const { 
    return addr < mem_size; 
  }

  // Serialization
  json serialize(Context<E> &ctx) const;
  void deserialize(Context<E> &ctx, const json& data);

  // File I/O operations using Context<E>
  void load_from_file(Context<E> &ctx);
  void save_to_file(Context<E> &ctx);
  void save_to_text_file(Context<E> &ctx);
  void load_from_text_file(Context<E> &ctx);

  // Memory dump for debugging
  void dump_memory(Context<E> &ctx, const std::string& desc = "") const;
  int get_read_latency() const  { return read_latency; }
  int get_write_latency() const { return write_latency; }
  virtual ~SyscMemory() = default;

protected:
  // Protected constructor - use create() static method
  SyscMemory(sc_core::sc_module_name name, 
             Context<E> &ctx,
             std::size_t size = 0, 
             T* initptr = nullptr);

  void b_transport(tlm::tlm_generic_payload& trans, sc_core::sc_time& delay);
  bool get_direct_mem_ptr(tlm::tlm_generic_payload& trans, tlm::tlm_dmi& dmi_data);
  unsigned int transport_dbg(tlm::tlm_generic_payload& trans);

  void memory_read();
  void memory_write();

private:

  std::unique_ptr<T[]> mem_data;
  std::size_t mem_size;
  bool is_init;

  // Per-instance latencies (in cycles)
  int read_latency  = 0;
  int write_latency = 0;

  struct ReadReq {
    std::size_t addr;
    int cycles_left;
  };

  struct WriteReq {
    std::size_t addr;
    T data;
    int cycles_left;
  };

  std::deque<ReadReq>  read_q;
  std::deque<WriteReq> write_q;
};

template <typename E>
std::string_view 
save_string(Context<E> &ctx, const std::string &str) {
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

template <typename E>
int redo_main(std::string_view target, int argc, char **argv);

template <typename E>
int adptsysc_main(int argc, char **argv);

}  // namespace adptsysc
