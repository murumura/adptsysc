#pragma once

#include <sysc/datatypes/fx/sc_fixed.h>
#include <sysc/datatypes/fx/sc_ufixed.h>

#include <complex>
#include <cctype>
#include <string>
#include <utility>
#include <vector>
#include <type_traits>

namespace adptsysc {

// Forward declarations are enough if the real definitions are included
// before these templates are instantiated.
template <typename E>
struct Context;

template <typename E>
class OutputFile;


// -----------------------------------------------------------------------------
// Type traits
// -----------------------------------------------------------------------------

template <typename T>
struct is_sysc_fixed_like : std::false_type {};

template <int W, int I, sc_dt::sc_q_mode Q, sc_dt::sc_o_mode O, int N>
struct is_sysc_fixed_like<sc_dt::sc_fixed<W, I, Q, O, N>> : std::true_type {};

template <int W, int I, sc_dt::sc_q_mode Q, sc_dt::sc_o_mode O, int N>
struct is_sysc_fixed_like<sc_dt::sc_ufixed<W, I, Q, O, N>> : std::true_type {};

template <typename T>
inline constexpr bool is_sysc_fixed_like_v =
    is_sysc_fixed_like<std::remove_cv_t<std::remove_reference_t<T>>>::value;

// -----------------------------------------------------------------------------
// SystemC fixed-point text-word helpers
// -----------------------------------------------------------------------------
//
// These helpers assume the input type has:
//   - to_bin()
//   - to_hex()
//
// SystemC fixed-point types such as sc_fixed/sc_ufixed provide these.
//
// Important:
//   Do not dump raw sc_fixed object bytes for SV/VCS comparison.
//   Use to_bin()/to_hex() and normalize the string into a clean word.
//
// Example:
//   sc_fixed<16,12> x = -1.5;
//   x.to_bin() may look like:
//     0b111111111110.1000
//
//   syscfx_to_binaryword(x) returns:
//     1111111111101000
//
// This is directly suitable for:
//   $readmemb("file.mem", mem);
//
// For hex:
//   syscfx_to_hexword(x)
//
// is directly suitable for:
//   $readmemh("file.mem", mem);
// -----------------------------------------------------------------------------

inline std::string
syscfx_strip_number_prefix(std::string s) {
  if (s.size() >= 2 && s[0] == '0') {
    const char p =
        static_cast<char>(std::tolower(static_cast<unsigned char>(s[1])));

    if (p == 'b' || p == 'x' || p == 'o') {
      return s.substr(2);
    }
  }

  return s;
}

inline std::string
syscfx_remove_point(std::string s) {
  std::string out;
  out.reserve(s.size());

  for (char c : s) {
    if (c != '.') {
      out.push_back(c);
    }
  }

  return out;
}

template <typename FxT>
std::string
syscfx_to_binaryword(const FxT& x, bool with_prefix = false) {
  std::string s = x.to_bin();

  if (!with_prefix) {
    s = syscfx_strip_number_prefix(std::move(s));
  }

  return syscfx_remove_point(std::move(s));
}

template <typename FxT>
std::string
syscfx_to_hexword(const FxT& x, bool with_prefix = false) {
  std::string s = x.to_hex();

  if (!with_prefix) {
    s = syscfx_strip_number_prefix(std::move(s));
  }

  return syscfx_remove_point(std::move(s));
}

template <typename E, typename FxT>
void write_syscfx_binaryword_line(OutputFile<E>& out, const FxT& x) {
  out.write_line(syscfx_to_binaryword(x, false));
}

template <typename E, typename FxT>
void
write_syscfx_hexword_line(OutputFile<E>& out, const FxT& x) {
  out.write_line(syscfx_to_hexword(x, false));
}

// -----------------------------------------------------------------------------
// Convenience vector dump helpers
// -----------------------------------------------------------------------------
//
// These avoid creating CsvTraceWriter/SvMemTraceWriter classes.
// Use OutputFile directly.
//
// Default output:
//   hex = true  -> $readmemh-compatible
//   hex = false -> $readmemb-compatible
// -----------------------------------------------------------------------------

template <typename E, typename FxT>
void write_syscfx_vector_mem(Context<E>& ctx,
                             const std::string& path,
                             const std::vector<FxT>& v,
                             bool hex = true,
                             i64 filesize = 1 << 20,
                             mode_t perm = 0777) {
  auto out = OutputFile<E>::open(ctx, path, filesize, perm);

  for (const auto& x : v) {
    if (hex) {
      write_syscfx_hexword_line(*out, x);
    } else {
      write_syscfx_binaryword_line(*out, x);
    }
  }

  out->close(ctx);
}

template <typename E, typename FxT>
void write_syscfx_complex_mem(Context<E>& ctx,
                              const std::string& path_re,
                              const std::string& path_im,
                              const std::vector<std::complex<FxT>>& v,
                              bool hex = true,
                              i64 filesize = 1 << 20,
                              mode_t perm = 0777) {
  auto re = OutputFile<E>::open(ctx, path_re, filesize, perm);
  auto im = OutputFile<E>::open(ctx, path_im, filesize, perm);

  for (const auto& z : v) {
    if (hex) {
      write_syscfx_hexword_line(*re, z.real());
      write_syscfx_hexword_line(*im, z.imag());
    } else {
      write_syscfx_binaryword_line(*re, z.real());
      write_syscfx_binaryword_line(*im, z.imag());
    }
  }

  re->close(ctx);
  im->close(ctx);
}

} // namespace adptsysc