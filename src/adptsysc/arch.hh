#pragma once

#include <sysc/datatypes/fx/sc_fixed.h>
#include <concepts>
#include <ostream>
#include <string>
#include <type_traits>

namespace adptsysc {

struct LMSArch {
  static constexpr std::string_view name = "lms";
  static constexpr bool debug = true;

  // Numeric types
  using Eval_T = float;
  using Fxpt_T = sc_dt::sc_fixed<16, 15, sc_dt::SC_TRN, sc_dt::SC_SAT>;

  // Algorithm config
  static constexpr bool normalized = false;
  static constexpr bool sign_error = false;
  static constexpr bool sign_data  = false;

  // Step-size config
  static constexpr float mu_default = 0.01f;


};

struct OlsConvAlgo {
  static constexpr std::string_view name = "olsconv_algo";
  static constexpr bool debug = true;

  using Eval_T = float;
  using Fxpt_T = sc_dt::sc_fixed<16, 15, sc_dt::SC_TRN, sc_dt::SC_SAT>;

  // Core config
  static constexpr std::size_t max_filter_len = 4096;
  static constexpr bool use_overlap_save = true;
  static constexpr bool use_overlap_add  = false;

  // FFT config
  static constexpr std::size_t fft_radix = 2;
  static constexpr bool use_bitrev = true;

  // Architecture
  static constexpr bool streaming = false;
  static constexpr bool block_mode = true;
};

// Forward declarations
template<typename E> class SyscMemory;
struct SyscMemArch {
  static constexpr std::string_view name = "syscmem";
  static constexpr bool debug = true;
  using Eval_T = int;
  using Fxpt_T = sc_dt::sc_fixed<16, 12>;
  using Impl_T = SyscMemory<SyscMemArch>;
  static constexpr bool support_rdwr_delay  = true;
};

template<typename E> class R2SdfFFTTLM;

struct R2SdfFFTTLMArch {
  static constexpr std::string_view name = "r2sdf_fft_tlm";
  static constexpr bool debug = true;

  using Eval_T = float;
  using Fxpt_T = sc_dt::sc_fixed<16, 12>;
  using Impl_T = R2SdfFFTTLM<R2SdfFFTTLMArch>;

  static constexpr std::size_t fft_size = 512;

  static constexpr bool use_dit = true;

  // FFT/IFFT support
  static constexpr bool support_fft  = true;
  static constexpr bool support_ifft = true;

  // Scaling convention:
  // false: IFFT applies final 1/N scaling at output
  // true : IFFT applies 1/2 scaling per radix-2 stage
  static constexpr bool scale_each_stage = false;

  // Controller path
  static constexpr bool use_ctrl = true;

  // Timing model
  static constexpr unsigned butterfly_latency = 1;
  static constexpr unsigned twiddle_latency   = 1;
  static constexpr unsigned memory_latency    = 1;
  static constexpr unsigned cmplxmul_latency  = 1;

  static constexpr bool support_rdwr_delay = true;
};

/*
template<typename E> class OverlapSaveFdafTLM;
struct OverlapSaveFdafTLMArch {
  static constexpr std::string_view name = "ovsfdaf-tlm";
  static constexpr bool debug = true;

  using Eval_T = float;
  using Fxpt_T = sc_dt::sc_fixed<16, 12>;
  using Impl_T = OverlapSaveFdafTLM<OverlapSaveFdafTLMArch>;
  static constexpr std::size_t filter_len = 256; // M
  static constexpr std::size_t fft_size   = 2 * filter_len;
  static constexpr std::size_t block_size = filter_len;

  // Algorithm params
  static constexpr float mu_default    = 0.01f;
  static constexpr float alpha_default = 0.9f;
  static constexpr float eps_default   = 1e-8f;
};
*/

struct PolyPhaseFilter {
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
template <typename E> concept support_rdwr_delay   = requires { E::support_rdwr_delay; };
template <typename E> concept have_implt           = requires { typename E::Impl_T; };
template <typename E> concept need_rom             = requires { E::need_rom; };
template <typename E> concept is_base              = E::is_base;
template <typename E> concept is_interp            = E::is_interp;
template <typename E> concept is_decim             = E::is_decim;
template <typename E> concept need_train           = E::need_train;

}  // namespace adptsysc
