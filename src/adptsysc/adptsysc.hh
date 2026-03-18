#pragma once

#include <adptsysc/common.hh>
#include <adptsysc/arch.hh>
#include <adptsysc/config.hh>
#include <cstdint>
#include <iostream>
#include <memory>
#include <string>
#include <string_view>
#include <unistd.h>
#include <vector>
#include <sys/stat.h>

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

// Context holds filter parameters and runtime state
template <typename E>
struct Context {
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
    int mem_rddly_cycls  = 0;   // cycles of RAM read latency
    int mem_wrdly_cycls = 0;   // cycles of RAM write latency
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
  ~MappedFile() { 
    unmap();
    close_fd();
  }

  void unmap() {
    if (owns_mapping && data != nullptr && size > 0) {
      munmap(data, size);
    }
    data = nullptr;
    size = 0;
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
                    std::size_t start, std::size_t sz) {
   if (start > size || sz > size - start)
    return nullptr;
    MappedFile* mf = new MappedFile;
    mf->name = std::move(name);
    mf->data = data + start;
    mf->size = sz;
    mf->fd = -1;
    mf->owns_mapping = false;
    mf->given_fullpath = given_fullpath;
    mf->is_dependency  = is_dependency;
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
  bool owns_mapping = true;
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
