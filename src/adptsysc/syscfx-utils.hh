#pragma once

#include <sysc/datatypes/fx/sc_fixed.h>
#include <sysc/datatypes/fx/sc_ufixed.h>
#include <sys/types.h>

#include <adptsysc/common.hh>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <complex>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace adptsysc {

template <typename E>
struct Context;

template <typename E>
class OutputFile;


// -----------------------------------------------------------------------------
// SystemC fixed-point -> SV word text
// -----------------------------------------------------------------------------

template <typename SysCFx_T>
std::string
syscfx_to_binword(const SysCFx_T& x, bool prefix = false) {
  std::string s = x.to_bin();

  if (!prefix) {
    s = strip_radix_prefix(std::move(s));
  }

  return strip_point(std::move(s));
}

template <typename SysCFx_T>
std::string
syscfx_to_hexword(const SysCFx_T& x, bool prefix = false) {
  std::string s = x.to_hex();

  if (!prefix) {
    s = strip_radix_prefix(std::move(s));
  }

  return strip_point(std::move(s));
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
void write_syscfx_binword(OutputFile<E>& out, const T& x) {
  out.write_line(archval_to_syscfx_binword<E>(x));
}

template <typename E, typename T>
void write_syscfx_hexword(OutputFile<E>& out, const T& x) {
  out.write_line(archval_to_syscfx_hexword<E>(x));
}

// -----------------------------------------------------------------------------
// Convenience vector dump helpers
// -----------------------------------------------------------------------------

template <typename E, typename T>
void write_syscfxvec(Context<E>& ctx, const std::string& path,
                     const std::vector<T>& v, bool hex = true, 
                     i64 filesize = 1 << 20, mode_t perm = 0777) {
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
void write_syscfxcmplx(Context<E>& ctx, const std::string& path_re,
                       const std::string& path_im, 
                       const std::vector<std::complex<T>>& v,
                       bool hex = true, i64 filesize = 1 << 20, mode_t perm = 0777) {
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
  std::string t = strip_space(s);
  t = strip_radix_prefix(std::move(t));

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
  static_assert(E::fx_word_bits > 0, "fixed-point word width must be positive");
  static_assert(E::fx_word_bits <= 64, "archsyscfx_from_word currently supports width <= 64");
  static_assert(E::fx_integer_bits >= 0, "fixed-point integer width must be non-negative");
  static_assert(E::fx_frac_bits >= 0, "fixed-point fractional width must be non-negative");
  static_assert(E::fx_frac_bits == E::fx_word_bits - E::fx_integer_bits, "fx_frac_bits must equal fx_word_bits - fx_integer_bits");

  if (base != 2 && base != 16) {
    throw std::invalid_argument(
        "archsyscfx_from_word supports only base 2 or base 16");
  }

  const std::uint64_t raw = parse_word_u64(s, base);

  long double scaled = 0.0L;

  if constexpr (E::fx_signed) {
    const std::int64_t q = signext_u64(raw, E::fx_word_bits);
    scaled = static_cast<long double>(q);
  } else {
    const std::uint64_t mask = (E::fx_word_bits >= 64) ? ~std::uint64_t{0} : ((std::uint64_t{1} << E::fx_word_bits) - 1);
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

// -----------------------------------------------------------------------------
// Generic arch fixed-point quantization helpers
// -----------------------------------------------------------------------------
//
// These helpers convert a value through E::Fxpt_T, then back to T.
// This means the architecture fixed-point quantization/overflow policy is used.
//
// Example:
//   float x = 1.25;
//   scalar_from_archsyscfx<MyArch>(x)
//
// does:
//   MyArch::Fxpt_T q = x;
//   return static_cast<float>(q);
// -----------------------------------------------------------------------------

template <typename E, typename T>
T scalar_from_archsyscfx(const T& x) {
  typename E::Fxpt_T q = x;
  return static_cast<T>(q);
}

template <typename E, typename T>
std::complex<T>
cmplx_from_archsyscfx(const std::complex<T>& z) {
  return std::complex<T>(
    scalar_from_archsyscfx<E, T>(z.real()),
    scalar_from_archsyscfx<E, T>(z.imag()));
}


template <typename E, typename T>
std::vector<T>
vec_from_archsyscfx(const std::vector<T>& in) {
  std::vector<T> out;
  out.reserve(in.size());

  for (const auto& x : in) {
    out.push_back(scalar_from_archsyscfx<E, T>(x));
  }

  return out;
}


template <typename E, typename T>
std::vector<std::complex<T>>
cmplxvec_from_archsyscfx(const std::vector<std::complex<T>>& in) {
  std::vector<std::complex<T>> out;
  out.reserve(in.size());

  for (const auto& z : in) {
    out.push_back(cmplx_from_archsyscfx<E, T>(z));
  }

  return out;
}


} // namespace adptsysc