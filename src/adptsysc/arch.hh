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
  static constexpr int fx_word_bits = 16;
  static constexpr int fx_integer_bits = 12;
  static constexpr int fx_frac_bits = fx_word_bits - fx_integer_bits;
  static constexpr bool fx_signed = true;
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

  static constexpr int fx_word_bits = 16;
  static constexpr int fx_integer_bits = 12;
  static constexpr int fx_frac_bits = fx_word_bits - fx_integer_bits;
  static constexpr bool fx_signed = true;
  using Impl_T = R2SdfFFTTLM<R2SdfFFTTLMArch>;
  
  static constexpr std::size_t fft_size = 32;

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

template<typename E> class SyscMemoryCycle;
struct SyscMemoryCycleArch {
  static constexpr std::string_view name = "syscmem_cycle";
  static constexpr bool debug = true;

  using Eval_T = float;
  using Fxpt_T = sc_dt::sc_fixed<16, 12>;

  static constexpr int fx_word_bits = 16;
  static constexpr int fx_integer_bits = 12;
  static constexpr int fx_frac_bits = fx_word_bits - fx_integer_bits;
  static constexpr bool fx_signed = true;

  using Impl_T = SyscMemoryCycle<SyscMemoryCycleArch>;
};

template<typename E> class R2SdfFFTCycle;
struct R2SdfFFTCycleArch {
  static constexpr std::string_view name = "r2sdf_fft_cycle";
  static constexpr bool debug = true;

  using Eval_T = float;
  using Fxpt_T = sc_dt::sc_fixed<16, 12>;

  static constexpr int fx_word_bits = 16;
  static constexpr int fx_integer_bits = 12;
  static constexpr int fx_frac_bits = fx_word_bits - fx_integer_bits;
  static constexpr bool fx_signed = true;

  using Impl_T = R2SdfFFTCycle<R2SdfFFTCycleArch>;

  static constexpr std::size_t fft_size = 32;
  static constexpr bool use_dit = true;
  static constexpr bool support_fft = true;
  static constexpr bool support_ifft = true;
  static constexpr bool scale_each_stage = false;

  // Cycle model: one radix-2 butterfly transaction at a time.
  static constexpr unsigned butterfly_latency = 1;
  static constexpr unsigned cmplxmul_latency = 1;
  static constexpr unsigned memory_latency = 1;
};

template<typename E> class OverlapSaveFdafCycle;
struct OverlapSaveFdafCycleArch {
  static constexpr std::string_view name = "overlap_save_fdaf_cycle";
  static constexpr bool debug = true;

  using Eval_T = float;
  using Fxpt_T = sc_dt::sc_fixed<16, 12>;

  static constexpr int fx_word_bits = 16;
  static constexpr int fx_integer_bits = 12;
  static constexpr int fx_frac_bits = fx_word_bits - fx_integer_bits;
  static constexpr bool fx_signed = true;

  using Impl_T = OverlapSaveFdafCycle<OverlapSaveFdafCycleArch>;

  static constexpr std::size_t filter_ncoeff = 16;
  static constexpr std::size_t fft_size = 2 * filter_ncoeff;
  static constexpr std::size_t block_size = filter_ncoeff;

  static constexpr bool use_dit = true;
  static constexpr bool scale_each_stage = false;
  static constexpr unsigned butterfly_latency = 1;
  static constexpr unsigned cmplxmul_latency = 1;
  static constexpr unsigned memory_latency = 1;

  static constexpr float mu_default = 0.01f;
  static constexpr float alpha_default = 0.9f;
  static constexpr float eps_default = 1e-8f;
  static constexpr bool use_power_norm = true;
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

constexpr unsigned ceil_log2(std::size_t value) {
  unsigned bits = 0;
  std::size_t n = value > 1 ? value - 1 : 0;
  while (n != 0) { ++bits; n >>= 1; }
  return bits;
}

// WBCIC input Q format is the only datapath format fixed at compile time.
// Runtime internal stage widths and pruning are in WbcicQuantConfig<E> (engine.hh).
template <int InputW = 16, int InputI = 12, std::size_t M = 16, std::size_t K = 4, int B = 0>
struct WbcicParams {
  static_assert(InputW > 1 && InputI > 0 && InputI <= InputW);
  static_assert(M > 1 && K > 0 && K % 2 == 0, "Fig. 3(b) assumes K = 2*k");
  static_assert(B >= -2 && B <= 16);

  static constexpr std::size_t m = M;
  static constexpr std::size_t k = K;  // CIC stages; Fig. 3(b)'s k = sharpening_k.
  static constexpr std::size_t sharpening_k = K / 2;
  static constexpr int b = B;
  static constexpr int input_word_bits = InputW;
  static constexpr int input_integer_bits = InputI;
  static constexpr int input_frac_bits = InputW - InputI;
  static constexpr int fx_word_bits = input_word_bits;  // IO compatibility only.
  static constexpr int fx_integer_bits = input_integer_bits;
  static constexpr int fx_frac_bits = input_frac_bits;
  static constexpr bool fx_signed = true;

  static constexpr int cic_full_bits = InputW + static_cast<int>(K * ceil_log2(M));
  static constexpr int squared_cic_baseline_bits = InputW + static_cast<int>(2 * K * ceil_log2(M));
  // Dyadic shift is valid only if M is a power of two; fixed mode checks this.
  static constexpr int scale_shift = B + 2 + static_cast<int>(K * ceil_log2(M));
  static constexpr int internal_frac_bits = input_frac_bits + 2 * scale_shift;
  // Finite-horizon simulation budget, NOT an RTL minimum or unlimited-stream guarantee.
  static constexpr unsigned planning_register_bits = 192;
  static constexpr double fs_in_default = 91.392e6;
};

template <typename E> class WidebandCicTLM;
template <typename E> class WidebandCicCycle;

struct WidebandCicTLMArch : WbcicParams<16, 12, 16, 4, 0> {
  static constexpr std::string_view name = "wideband_cic_tlm";
  static constexpr bool debug = true;
  using Eval_T = float;
  using Fxpt_T = sc_dt::sc_fixed<fx_word_bits, fx_integer_bits, sc_dt::SC_TRN, sc_dt::SC_WRAP>;
  using Impl_T = WidebandCicTLM<WidebandCicTLMArch>;
};

struct WidebandCicCycleArch : WbcicParams<16, 12, 16, 4, 0> {
  static constexpr std::string_view name = "wideband_cic_cycle";
  static constexpr bool debug = true;
  using Eval_T = float;
  using Fxpt_T = sc_dt::sc_fixed<fx_word_bits, fx_integer_bits, sc_dt::SC_TRN, sc_dt::SC_WRAP>;
  using Impl_T = WidebandCicCycle<WidebandCicCycleArch>;
};

// Standalone engine test target; does not construct TLM sockets or clocked modules.
template <typename E> class WbcicEngine;
struct WbcicEngineArch : WbcicParams<16, 12, 16, 4, 0> {
  static constexpr std::string_view name = "wideband_cic_engine";
  static constexpr bool debug = true;
  using Eval_T = double;
  using Fxpt_T = sc_dt::sc_fixed<fx_word_bits, fx_integer_bits, sc_dt::SC_TRN, sc_dt::SC_WRAP>;
  using Impl_T = WbcicEngine<WbcicEngineArch>;
};

template <typename E> concept support_rdwr_delay   = requires { E::support_rdwr_delay; };
template <typename E> concept have_impltype        = requires { typename E::Impl_T; };
template <typename E> concept need_rom             = requires { E::need_rom; };
template <typename E> concept is_base              = E::is_base;
template <typename E> concept is_interp            = E::is_interp;
template <typename E> concept is_decim             = E::is_decim;
template <typename E> concept need_train           = E::need_train;

}  // namespace adptsysc