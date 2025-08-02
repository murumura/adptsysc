#pragma once

#include <adptsysc/integers.hh>

#include <concepts>
#include <ostream>
#include <string>
#include <type_traits>

namespace adptsysc {

// Endian-aware integer aliases
template <typename E> using I16 = std::conditional_t<E::is_le, il16, ib16>;
template <typename E> using I32 = std::conditional_t<E::is_le, il32, ib32>;
template <typename E> using I64 = std::conditional_t<E::is_le, il64, ib64>;
template <typename E> using U16 = std::conditional_t<E::is_le, ul16, ub16>;
template <typename E> using U24 = std::conditional_t<E::is_le, ul24, ub24>;
template <typename E> using U32 = std::conditional_t<E::is_le, ul32, ub32>;
template <typename E> using U64 = std::conditional_t<E::is_le, ul64, ub64>;
template <typename E> using Word = std::conditional_t<E::is_64, U64<E>, U32<E>>;
template <typename E> using SWord = std::conditional_t<E::is_64, I64<E>, I32<E>>;

struct PPF {
  static constexpr bool is_le = true;
  static constexpr bool is_64 = false;
  static constexpr bool is_base = true;
  static constexpr bool need_train = false;
  static constexpr bool need_rom = false;
  static constexpr bool is_interp = false;
  static constexpr bool is_decim = false;
  static constexpr bool is_multistage = false;
  static constexpr std::string_view Name = "base-ppf";
};

struct PPFU : PPF {
  static constexpr bool is_interp = true;
  static constexpr bool need_rom = true;
  static constexpr std::string_view Name = "polyphase-upsampler";

  // Interpolator-specific features
  static constexpr uint32_t kPhaseCount = 8;
};

struct PPFD : PPF {
  static constexpr bool is_decim = true;
  static constexpr bool need_rom = true;
  static constexpr std::string_view Name = "polyphase-decimator";

  // Decimator-specific features
  static constexpr uint32_t kDecimationFactor = 4;
};

template <typename E> concept need_rom    = E::need_rom;
template <typename E> concept is_base     = E::is_base;
template <typename E> concept is_interp   = E::is_interp;
template <typename E> concept is_decim    = E::is_decim;
template <typename E> concept need_train  = E::need_train;

}  // namespace adptsysc
