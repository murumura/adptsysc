#pragma once

#include <adptsysc/integers.hh>

#include <concepts>
#include <ostream>
#include <string>
#include <type_traits>

namespace adptsysc {

// Endian-aware integer aliases
template <typename E> using I16 = std::conditional_t<E::IsLE, il16, ib16>;
template <typename E> using I32 = std::conditional_t<E::IsLE, il32, ib32>;
template <typename E> using I64 = std::conditional_t<E::IsLE, il64, ib64>;
template <typename E> using U16 = std::conditional_t<E::IsLE, ul16, ub16>;
template <typename E> using U24 = std::conditional_t<E::IsLE, ul24, ub24>;
template <typename E> using U32 = std::conditional_t<E::IsLE, ul32, ub32>;
template <typename E> using U64 = std::conditional_t<E::IsLE, ul64, ub64>;
template <typename E> using Word = std::conditional_t<E::Is64, U64<E>, U32<E>>;
template <typename E> using SWord = std::conditional_t<E::Is64, I64<E>, I32<E>>;

struct PPF {
  static constexpr bool IsLE = true;
  static constexpr bool Is64 = false;
  static constexpr bool IsBase = true;
  static constexpr bool NeedsTrain = false;
  static constexpr bool NeedsROM = false;
  static constexpr bool IsInterp = false;
  static constexpr bool IsDecim = false;
  static constexpr bool IsMultiStage = false;
  static constexpr std::string_view Name = "base-ppf";
};

struct PPFU : PPF {
  static constexpr bool IsInterp = true;
  static constexpr bool NeedsROM = true;
  static constexpr std::string_view Name = "polyphase-upsampler";

  // Interpolator-specific features
  static constexpr uint32_t kPhaseCount = 8;
};

struct PPFD : PPF {
  static constexpr bool IsDecim = true;
  static constexpr bool NeedsROM = true;
  static constexpr std::string_view Name = "polyphase-decimator";

  // Decimator-specific features
  static constexpr uint32_t kDecimationFactor = 4;
};

template <typename E> concept NeedsROM   = E::NeedsROM;
template <typename E> concept IsBase     = E::IsBase;
template <typename E> concept IsInterp   = E::IsInterp;
template <typename E> concept IsDecim    = E::IsDecim;
template <typename E> concept NeedsTrain = E::NeedsTrain;

}  // namespace adptsysc
