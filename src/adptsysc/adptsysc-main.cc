#include <adptsysc/adptsysc.hh>
#include <adptsysc/arch.hh>
#include <adptsysc/config.hh>
#include <adptsysc/sysc-mem.hh>
#include <adptsysc/sysc-r2sdffft.hh>
namespace adptsysc {

template <typename E>
static int
open_or_create_file(Context<E> &ctx, std::string path, 
                    std::string tmpfile, mode_t perm) {
  // Reuse an existing file if exists and writable because on Linux,
  // writing to an existing file is much faster than creating a fresh
  // file and writing to it.
  if (ctx.overwrite_output_file && rename(path.c_str(), tmpfile.c_str()) == 0) {
    i64 fd = ::open(tmpfile.c_str(), O_RDWR | O_CREAT, perm);
    if (fd != -1)
      return fd;
    unlink(tmpfile.c_str());
  }

  i64 fd = ::open(tmpfile.c_str(), O_RDWR | O_CREAT, perm);
  if (fd == -1)
    Fatal(ctx) << "cannot open " << tmpfile << ": " << errno_string();
  return fd;
}

// Get umask for permission calculations
static mode_t get_umask() {
  mode_t orig_mask = umask(0);
  umask(orig_mask); // restore
  return orig_mask;
}

template <typename E>
class MemoryMappedOutputFile : public OutputFile<E> {
public:
  MemoryMappedOutputFile(Context<E> &ctx, std::string path, 
                        i64 filesize, mode_t perm)
    : OutputFile<E>(path, filesize, true) {
    std::string pid = std::to_string(getpid());
    std::string tmpfile = path_dirname(path) / ("." + path_filename(path) + "." + pid);

    this->fd = open_or_create_file(ctx, path, tmpfile, perm);

    if (fchmod(this->fd, perm & ~get_umask()) == -1)
      Fatal(ctx) << "fchmod failed: " << errno_string();
    
    // Set file size
    if (ftruncate(this->fd, filesize) == -1)
      Fatal(ctx) << "ftruncate failed: " << errno_string();

    // Pre-allocate disk space (Linux specific)
    if (posix_fallocate(this->fd, 0, filesize) != 0) {
      // Fallback: if fallocate fails, use ftruncate
      if (ftruncate(this->fd, filesize) == -1)
        Fatal(ctx) << "ftruncate failed: " << errno_string();
    }

    this->buf = (u8 *)mmap(nullptr, filesize, PROT_READ | PROT_WRITE, MAP_SHARED, this->fd, 0);
    if (this->buf == MAP_FAILED)
      Fatal(ctx) << path << ": mmap failed: " << errno_string();

    adptsysc::output_buffer_start = this->buf;
    adptsysc::output_buffer_end = this->buf + filesize;
    adptsysc::output_tmpfile = (char *)save_string(ctx, tmpfile).data();
  }

  ~MemoryMappedOutputFile() {
    if (fd2 != -1)
      ::close(fd2);
  }

  void close(Context<E> &ctx) override {

    if (this->is_mmapped)
      munmap(this->buf, this->filesize);

    if (this->buf2.empty()) {
      ::close(this->fd);
    } else {
      FILE *out = fdopen(this->fd, "w");
      fseek(out, 0, SEEK_END);
      fwrite(&this->buf2[0], this->buf2.size(), 1, out);
      fclose(out);
    }

    // If an output file already exists, open a file and then remove it.
    // This is the fastest way to unlink a file, as it does not make the
    // system to immediately release disk blocks occupied by the file.
    fd2 = ::open(this->path.c_str(), O_RDONLY);
    if (fd2 != -1)
      unlink(this->path.c_str());

    if (rename(adptsysc::output_tmpfile, this->path.c_str()) == -1)
      Fatal(ctx) << this->path << ": rename failed: " << errno_string();
    adptsysc::output_tmpfile = nullptr;
  }

private:
  int fd2 = -1;
};

template <typename E>
std::unique_ptr<OutputFile<E>>
OutputFile<E>::open(Context<E> &ctx, std::string path,
                    i64 filesize, mode_t perm) {

  if (path.starts_with('/') && !ctx.arg.chroot.empty())
    path = ctx.arg.chroot + "/" + path_clean(path);

  std::error_code error;
  bool is_special = path == "-" || (!std::filesystem::is_regular_file(path, error) && !error);

  OutputFile<E> *file;
  if (is_special)
    file = new MallocOutputFile(ctx, path, filesize, perm);
  else
    file = new MemoryMappedOutputFile(ctx, path, filesize, perm);

#ifdef MADV_HUGEPAGE
  // Enable transparent huge page for an output memory-mapped file.
  // On Linux, it has an effect only on tmpfs mounted with `huge=advise`,
  // but it can make the linker ~10% faster. You can try it by creating
  // a tmpfs with the following commands
  //
  //  $ mkdir tmp
  //  $ sudo mount -t tmpfs -o size=2G,huge=advise none tmp
  //
  // and then specifying a path under the directory as an output file.
  madvise(file->buf, filesize, MADV_HUGEPAGE);
#endif

  if (ctx.arg.filler != -1)
    memset(file->buf, ctx.arg.filler, filesize);
  return std::unique_ptr<OutputFile>(file);
}

// Since adptsysc_main is a template, we can't run it without a type parameter.
// We speculatively run adptsysc_main with SyscMemArch, 
// and if the speculation was wrong, re-run it with an actual target type.
template <typename E>
int redo_main(std::string_view target, int argc, char **argv) {
  if constexpr (HAVE_SyscMemArch) 
    if (target == SyscMemArch::name)
      return adptsysc_main<SyscMemArch>(argc, argv);
  
  if constexpr (HAVE_R2SdfFFTTLMArch) 
    if (target == R2SdfFFTTLMArch::name)
      return adptsysc_main<R2SdfFFTTLMArch>(argc, argv);
  
  abort();
}

template <typename E>
int adptsysc_main(int argc, char **argv) {
  Context<E> ctx;
  // Parse non-positional command line options
  ctx.cmdline_args = expand_response_files(ctx, argv);
  std::vector<std::string> file_args = parse_nonpositional_args(ctx);

  // If no -e option is given, deduce it from input files.
  if (ctx.arg.emulation.empty())
    Fatal(ctx) << "Emulation option empty\n";

  // Redo if -e does not match with our speculation.
  if (ctx.arg.emulation != E::name)
    return redo_main<E>(ctx.arg.emulation, argc, argv);

  if (!ctx.arg.directory.empty())
    if (chdir(ctx.arg.directory.c_str()) == -1)
      Fatal(ctx) << "chdir failed: " << ctx.arg.directory
                 << ": " << errno_string() << "\n";
  
  // Run testbench if requested
  if (ctx.arg.run_testbench) {
    Out(ctx) << "run_testbench for " <<  E::name << "\n";
    bool test_success = E::Impl_T::run_testbench(ctx);
    if (!test_success) {
      return 1; // Return error code if testbench failed
    }
  }

  if (ctx.arg.quick_exit)
    _exit(0);
  ctx.checkpoint();
  return 0;
}

using E = ADPT_TARGET;
template class OutputFile<E>;
template int adptsysc_main<E>(int, char **);
template int redo_main<E>(std::string_view, int, char **);

}