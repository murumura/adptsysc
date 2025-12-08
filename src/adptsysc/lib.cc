#include <adptsysc/common.hh>
#include <adptsysc/adptsysc.hh>
#include <random>
namespace adptsysc {

MappedFile* open_file_impl(const std::string& path, 
                           std::string& error) {
  i64 fd = ::open(path.c_str(), O_RDONLY);
  if (fd == -1) {
    if (errno != ENOENT)
      error = "opening " + path + " failed: " + errno_string();
    return nullptr;
  }

  struct stat st;
  if (fstat(fd, &st) == -1)
    error = path + ": fstat failed: " + errno_string();

  MappedFile *mf = new MappedFile;
  mf->name = path;
  mf->size = st.st_size;

  if (st.st_size > 0) {
    mf->data = (u8 *)mmap(nullptr, st.st_size, PROT_READ | PROT_WRITE,
                          MAP_PRIVATE, fd, 0);
    if (mf->data == MAP_FAILED)
      error = path + ": mmap failed: " + errno_string();
  }

  close(fd);
  return mf;
}

std::string errno_string() {
  // strerror is not thread-safe
  // so guard it with a lock.
  static std::mutex mu;
  std::scoped_lock lock(mu);
  return std::strerror(errno);
}

void cleanup() {
  if (adptsysc::output_tmpfile)
    unlink(adptsysc::output_tmpfile);
}

void get_random_bytes(u8* buf, i64 size) {
  std::random_device rand;
  i64 i = 0;

  for (; i < size - 4; i += 4) {
    u32 val = rand();
    memcpy(buf + i, &val, 4);
  }

  u32 val = rand();
  memcpy(buf + i, &val, size - i);
}

std::string get_self_path() {
  return fs::read_symlink("/proc/self/exe").string();
}

} // namespace adptsysc