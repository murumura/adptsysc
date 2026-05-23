#pragma once

#include <sysc/datatypes/fx/sc_fixed.h>
#include <sysc/datatypes/fx/sc_ufixed.h>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <complex>
#include <cstdint>
#include <string>
#include <utility>
#include <vector>

namespace adptsysc {

// -----------------------------------------------------------------------------
// String helpers
// -----------------------------------------------------------------------------

inline std::string
strip_ws(std::string s) {
  auto is_ws = [](unsigned char c) { return std::isspace(c); };

  s.erase(s.begin(),
          std::find_if(s.begin(), s.end(),
                       [&](char c) {return !is_ws(static_cast<unsigned char>(c));}));

  s.erase(std::find_if(s.rbegin(), s.rend(),
                       [&](char c) {
                         return !is_ws(static_cast<unsigned char>(c));
                       }).base(),
          s.end());

  return s;
}

inline std::string
strip_sysc_radix_prefix(std::string s) {
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
strip_sysc_point(std::string s) {
  std::string out;
  out.reserve(s.size());

  for (char c : s) {
    if (c != '.') {
      out.push_back(c);
    }
  }

  return out;
}

// -----------------------------------------------------------------------------
// SystemC fixed-point -> SV word text
// -----------------------------------------------------------------------------

template <typename SysCFx_T>
std::string
syscfx_to_binword(const SysCFx_T& x, bool prefix = false) {
  std::string s = x.to_bin();

  if (!prefix) {
    s = strip_sysc_radix_prefix(std::move(s));
  }

  return strip_sysc_point(std::move(s));
}

template <typename SysCFx_T>
std::string
syscfx_to_hexword(const SysCFx_T& x, bool prefix = false) {
  std::string s = x.to_hex();

  if (!prefix) {
    s = strip_sysc_radix_prefix(std::move(s));
  }

  return strip_sysc_point(std::move(s));
}

// -----------------------------------------------------------------------------
// Arch value -> arch SystemC fixed-point word
// -----------------------------------------------------------------------------
//
// Every arch defines:
//
//   using Fxpt_T = sc_dt::sc_fixed<...>;
//   static constexpr int fx_word_bits;
//   static constexpr int fx_integer_bits;
//   static constexpr int fx_frac_bits;
//   static constexpr bool fx_signed;
//
// Therefore we always cast through E::Fxpt_T.
// This makes float/int/Eval_T output follow the arch quantization policy.
// -----------------------------------------------------------------------------

template <typename E, typename T>
std::string
archval_to_syscfx_binword(const T& x) {
  typename E::Fxpt_T q = x;
  return syscfx_to_binword(q);
}

template <typename E, typename T>
std::string
archval_to_syscfx_hexword(const T& x) {
  typename E::Fxpt_T q = x;
  return syscfx_to_hexword(q);
}

// -----------------------------------------------------------------------------
// OutputFile convenience
// -----------------------------------------------------------------------------

template <typename E, typename T>
void
write_syscfx_binword(OutputFile<E>& out, const T& x) {
  out.write_line(archval_to_syscfx_binword<E>(x));
}

template <typename E, typename T>
void
write_syscfx_hexword(OutputFile<E>& out, const T& x) {
  out.write_line(archval_to_syscfx_hexword<E>(x));
}

// -----------------------------------------------------------------------------
// Convenience vector dump helpers
// -----------------------------------------------------------------------------

template <typename E, typename T>
void
write_syscfx_vector_mem(Context<E>& ctx,
                        const std::string& path,
                        const std::vector<T>& v,
                        bool hex = true,
                        i64 filesize = 1 << 20,
                        mode_t perm = 0777) {
  auto out = OutputFile<E>::open(ctx, path, filesize, perm);

  for (const auto& x : v) {
    if (hex) {
      write_syscfx_hexword(*out, x);
    } else {
      write_syscfx_binword(*out, x);
    }
  }

  out->close(ctx);
}

template <typename E, typename T>
void
write_syscfx_complex_mem(Context<E>& ctx,
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
      write_syscfx_hexword(*re, z.real());
      write_syscfx_hexword(*im, z.imag());
    } else {
      write_syscfx_binword(*re, z.real());
      write_syscfx_binword(*im, z.imag());
    }
  }

  re->close(ctx);
  im->close(ctx);
}

// -----------------------------------------------------------------------------
// SV word text -> architecture fixed-point
// -----------------------------------------------------------------------------

inline std::uint64_t
parse_word_u64(const std::string& s, int base) {
  std::string t = strip_ws(s);
  t = strip_sysc_radix_prefix(std::move(t));

  if (t.empty()) {
    return 0;
  }

  return static_cast<std::uint64_t>(std::stoull(t, nullptr, base));
}

inline std::int64_t
signext_u64(std::uint64_t raw, int width) {
  if (width <= 0 || width >= 64) {
    return static_cast<std::int64_t>(raw);
  }

  const std::uint64_t sign_bit = std::uint64_t{1} << (width - 1);
  const std::uint64_t mask = (std::uint64_t{1} << width) - 1;

  raw &= mask;

  if (raw & sign_bit) {
    raw |= ~mask;
  }

  return static_cast<std::int64_t>(raw);
}

template <typename E>
typename E::Fxpt_T
archsyscfx_from_word(const std::string& s, int base) {
  static_assert(E::fx_word_bits <= 64,
                "archsyscfx_from_word currently supports width <= 64");

  const std::uint64_t raw = parse_word_u64(s, base);

  long double scaled = 0.0L;

  if constexpr (E::fx_signed) {
    const std::int64_t q = signext_u64(raw, E::fx_word_bits);
    scaled = static_cast<long double>(q);
  } else {
    const std::uint64_t mask =
        (E::fx_word_bits >= 64)
            ? ~std::uint64_t{0}
            : ((std::uint64_t{1} << E::fx_word_bits) - 1);

    scaled = static_cast<long double>(raw & mask);
  }

  const long double value = std::ldexp(scaled, -E::fx_frac_bits);

  typename E::Fxpt_T out = static_cast<double>(value);
  return out;
}

template <typename E>
typename E::Fxpt_T
archsyscfx_from_hexword(const std::string& s) {
  return archsyscfx_from_word<E>(s, 16);
}

template <typename E>
typename E::Fxpt_T
archsyscfx_from_binword(const std::string& s) {
  return archsyscfx_from_word<E>(s, 2);
}

} // namespace adptsysc