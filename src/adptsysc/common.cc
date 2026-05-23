#include <adptsysc/common.hh>
#include <adptsysc/arch.hh>
#include <adptsysc/adptsysc.hh>
#include <linux/sysctl.h>
#include <unistd.h>

namespace adptsysc {


static std::string_view fatal_mono = "adptsysc: fatal: ";
static std::string_view fatal_color = "adptsysc: \033[0;1;31mfatal:\033[0m ";
static std::string_view error_mono = "adptsysc: error: ";
static std::string_view error_color = "adptsysc: \033[0;1;31merror:\033[0m ";
static std::string_view warning_mono = "adptsysc: warning: ";
static std::string_view warning_color = "adptsysc: \033[0;1;35mwarning:\033[0m ";

template <typename E>
Fatal<E>::Fatal(Context<E> &ctx) {
  out << (ctx.arg.color_diagnostics ? fatal_color : fatal_mono);
}

template <typename E>
[[noreturn]] Fatal<E>::~Fatal() {
  out.emit();
  _exit(1);
}

template <typename E>
Error<E>::Error(Context<E> &ctx) {
  if (ctx.arg.noinhibit_exec) {
    out << (ctx.arg.color_diagnostics ? warning_color : warning_mono);
  } else {
    out << (ctx.arg.color_diagnostics ? error_color : error_mono);
    ctx.has_error = true;
  }
}

template <typename E>
Warn<E>::Warn(Context<E> &ctx) {
  if (ctx.arg.suppress_warnings)
    return;

  out.emplace(std::cerr);

  if (ctx.arg.fatal_warnings) {
    *out << (ctx.arg.color_diagnostics ? error_color : error_mono);
    ctx.has_error = true;
  } else {
    *out << (ctx.arg.color_diagnostics ? warning_color : warning_mono);
  }
}


template <typename E, typename T>
void write_syscfixed_binword(OutputFile<E>& out, const T& x) {
  out.write_line(archnum_to_syscfixed_binword<E>(x));
}

template <typename E, typename T>
void write_syscfixed_hexword(OutputFile<E>& out, const T& x) {
  out.write_line(archnum_to_syscfixed_hexword<E>(x));
}

// -----------------------------------------------------------------------------
// Convenience vector dump helpers
// -----------------------------------------------------------------------------

template <typename E, typename T>
void
write_syscfixed_vector_mem(Context<E>& ctx,
                           const std::string& path,
                           const std::vector<T>& v,
                           bool hex = true,
                           i64 filesize = 1 << 20,
                           mode_t perm = 0777) {
  auto out = OutputFile<E>::open(ctx, path, filesize, perm);

  for (const auto& x : v) {
    if (hex) {
      write_syscfixed_hexword(*out, x);
    } else {
      write_syscfixed_binword(*out, x);
    }
  }

  out->close(ctx);
}

template <typename E, typename T>
void
write_syscfixed_complex_mem(Context<E>& ctx,
                            const std::string& path_re,
                            const std::string& path_im,
                            const std::vector<std::complex<T>>& v,
                            bool hex = true,
                            i64 filesize = 1 << 20,
                            mode_t perm = 0777) {
  auto re = OutputFile<E>::open(ctx, path_re, filesize, perm);
  auto im = OutputFile<E>::open(ctx, path_im, filesize, perm);

  for (const auto& z : v) {
    if (hex) {
      write_syscfixed_hexword(*re, z.real());
      write_syscfixed_hexword(*im, z.imag());
    } else {
      write_syscfixed_binword(*re, z.real());
      write_syscfixed_binword(*im, z.imag());
    }
  }

  re->close(ctx);
  im->close(ctx);
}

using E = ADPT_TARGET;
template class Fatal<E>;
template class Error<E>;
template class Warn<E>;


}  // namespace adptsysc