#include <adptsysc/common.hh>
#include <linux/sysctl.h>
#include <random>
#include <unistd.h>

namespace adptsysc {
  
std::string errno_string() {
  // strerror is not thread-safe, so guard it with a lock.
  static std::mutex mu;
  std::scoped_lock lock(mu);
  return std::strerror(errno);
}

void cleanup() {
  if (output_tmpfile)
    unlink(output_tmpfile);
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
  return std::filesystem::read_symlink("/proc/self/exe").string();
}

}  // namespace adptsysc