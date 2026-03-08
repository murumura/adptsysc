#pragma once

#include <adptsysc/integers.hh>
#include <sysc/datatypes/fx/sc_fixed.h>
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

struct LMSArch {
  static constexpr std::string_view name = "lms";
  static constexpr bool debug = true;
  using Fxpt_T = sc_dt::sc_fixed<16, 15, sc_dt::SC_TRN, sc_dt::SC_SAT>;
  using Eval_T = float;
  // using Impl_T = SyscLms<LMSArch>;
  static constexpr bool is_le = true;   // or false
  static constexpr bool is_64 = false;  // LMS is float-based, so 32-bit word?
};

struct OlsConvAlgo {
  static constexpr std::string_view name = "olsconv_algo";
  static constexpr bool debug = true;
  using Fxpt_T = sc_dt::sc_fixed<16, 15, sc_dt::SC_TRN, sc_dt::SC_SAT>;
  using Eval_T = float;
};

// Forward declarations
template<typename E> class SyscMemory;
struct SyscMemArch {
  static constexpr std::string_view name = "syscmem";
  static constexpr bool debug = true;
  using Eval_T = int;
  using Fxpt_T = sc_dt::sc_fixed<16, 12>;
  using Impl_T = SyscMemory<SyscMemArch>;
  static constexpr bool support_rw_lactency  = true;
};

struct PolyPhaseFilter {
  static constexpr bool is_le = true;
  static constexpr bool is_64 = false;
  static constexpr bool is_base = true;
  static constexpr bool need_train = false;
  static constexpr bool need_rom = false;
  static constexpr bool is_interp = false;
  static constexpr bool is_decim = false;
  static constexpr bool is_multistage = false;
  static constexpr bool debug = false;
  static constexpr std::string_view name = "polyphase-base";
};

struct PolyPhaseUpSampler : PolyPhaseFilter {
  static constexpr bool is_interp = true;
  static constexpr bool need_rom = true;
  static constexpr std::string_view name = "polyphase-upsampler";

  // Interpolator-specific features
  static constexpr uint32_t kPhaseCount = 8;
};

struct PolyPhaseDownSampler : PolyPhaseFilter {
  static constexpr bool is_decim = true;
  static constexpr bool need_rom = true;
  static constexpr std::string_view name = "polyphase-decimator";

  // Decimator-specific features
  static constexpr uint32_t kDecimationFactor = 4;
};
template <typename E> concept support_rwlat   = requires { E::support_rw_lactency; };
template <typename E> concept have_implt      = requires { typename E::Impl_T; };
template <typename E> concept need_rom        = requires { E::need_rom; };
template <typename E> concept is_base         = E::is_base;
template <typename E> concept is_interp       = E::is_interp;
template <typename E> concept is_decim        = E::is_decim;
template <typename E> concept need_train      = E::need_train;

}  // namespace adptsysc
