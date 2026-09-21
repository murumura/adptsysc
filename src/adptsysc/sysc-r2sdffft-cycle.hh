#pragma once

#include <systemc>

#include <adptsysc/adptsysc.hh>
#include <adptsysc/design-lib.hh>
#include <adptsysc/object.hh>
#include <adptsysc/sysc-cmplxmul-cycle.hh>

#include <cstddef>
#include <memory>
#include <vector>

namespace adptsysc {

template <typename E>
struct Context;

template <typename E>
class R2SdfCtrlCycle : public sc_core::sc_module {
public:
  using CountT = sc_dt::sc_uint<64>;
  using AddrT = sc_dt::sc_uint<64>;

  sc_core::sc_in<bool> clk{"clk"};
  sc_core::sc_in<bool> rst_n{"rst_n"};
  sc_core::sc_in<bool> sample_fire{"sample_fire"};

  sc_core::sc_out<CountT> count{"count"};
  sc_core::sc_out<bool> frame_first{"frame_first"};
  sc_core::sc_out<bool> frame_last{"frame_last"};

  sc_core::sc_vector<sc_core::sc_out<bool>> stage_sel;
  sc_core::sc_vector<sc_core::sc_out<bool>> twiddle_enable;
  sc_core::sc_vector<sc_core::sc_out<AddrT>> twiddle_addr;

  R2SdfCtrlCycle(sc_core::sc_module_name name,
                 std::size_t fft_size, FFTFlowMode flow_mode);

  std::size_t get_fftsize() const { return fft_size; }
  unsigned get_nstages() const { return nstages; }

private:
  std::size_t fft_size = 0;
  unsigned nstages = 0;
  FFTFlowMode flow_mode = FFTFlowMode::DIT;

  static unsigned ilog2(std::size_t n);
  void drive_decode(std::size_t c, bool fire);
  void run();
};

// One clock-driven radix-2 butterfly transaction.  The FFT top reuses this
// engine for every butterfly, making the cycle model deterministic and close
// to a small-area RTL implementation.
template <typename E>
class R2SdfStageCycle : public sc_core::sc_module {
public:
  using T = typename E::Fxpt_T;

  sc_core::sc_in<bool> clk{"clk"};
  sc_core::sc_in<bool> rst_n{"rst_n"};

  sc_core::sc_in<bool> in_valid{"in_valid"};
  sc_core::sc_out<bool> in_ready{"in_ready"};

  sc_core::sc_in<T> a_re{"a_re"};
  sc_core::sc_in<T> a_im{"a_im"};
  sc_core::sc_in<T> b_re{"b_re"};
  sc_core::sc_in<T> b_im{"b_im"};
  sc_core::sc_in<T> tw_re{"tw_re"};
  sc_core::sc_in<T> tw_im{"tw_im"};
  sc_core::sc_in<bool> use_twiddle{"use_twiddle"};
  sc_core::sc_in<bool> dit_mode{"dit_mode"};
  sc_core::sc_in<bool> inverse{"inverse"};
  sc_core::sc_in<bool> scale_half{"scale_half"};

  sc_core::sc_out<bool> out_valid{"out_valid"};
  sc_core::sc_in<bool> out_ready{"out_ready"};
  sc_core::sc_out<T> y0_re{"y0_re"};
  sc_core::sc_out<T> y0_im{"y0_im"};
  sc_core::sc_out<T> y1_re{"y1_re"};
  sc_core::sc_out<T> y1_im{"y1_im"};


  explicit R2SdfStageCycle(sc_core::sc_module_name name);

private:
  struct Pair {
    T y0_re{};
    T y0_im{};
    T y1_re{};
    T y1_im{};
  };

  static constexpr unsigned base_latency();
  static Pair butterfly(const T& ar,
                        const T& ai,
                        const T& br,
                        const T& bi,
                        const T& wr,
                        const T& wi,
                        bool use_twiddle,
                        bool dit_mode,
                        bool inverse,
                        bool scale_half);
  void run();
};

template <typename E>
class R2SdfFFTCycle : public ObjectWithMutableHyperparams,
                      public sc_core::sc_module {
public:
  using T = typename E::Fxpt_T;
  using EvalT = typename E::Eval_T;
  // Keep complex fixed-point state in a plain aggregate.
  // std::complex<T> is only fully portable for the standard floating types in
  // C++20; ComplexPlain<T> also maps more directly to RTL re/im signals.
  using CxT = ComplexPlain<T>;

  sc_core::sc_in<bool> clk{"clk"};
  sc_core::sc_in<bool> rst_n{"rst_n"};

  sc_core::sc_in<bool> inverse{"inverse"};

  sc_core::sc_in<bool> in_valid{"in_valid"};
  sc_core::sc_out<bool> in_ready{"in_ready"};
  sc_core::sc_in<T> din_re{"din_re"};
  sc_core::sc_in<T> din_im{"din_im"};
  sc_core::sc_in<bool> in_last{"in_last"};

  sc_core::sc_out<bool> out_valid{"out_valid"};
  sc_core::sc_in<bool> out_ready{"out_ready"};
  sc_core::sc_out<T> dout_re{"dout_re"};
  sc_core::sc_out<T> dout_im{"dout_im"};
  sc_core::sc_out<bool> out_last{"out_last"};
  sc_core::sc_out<bool> busy{"busy"};


  static std::unique_ptr<R2SdfFFTCycle<E>>
  create(Context<E>& ctx,
         sc_core::sc_module_name name,
         std::size_t fft_size = E::fft_size,
         FFTFlowMode flow_mode = E::use_dit ? FFTFlowMode::DIT : FFTFlowMode::DIF);

  static bool run_testbench(Context<E>& ctx);

  R2SdfFFTCycle(Context<E>& ctx, sc_core::sc_module_name name,
                std::size_t fft_size, FFTFlowMode flow_mode);

  std::size_t get_fftsize() const { return fft_size; }
  unsigned get_nstages() const { return nstages; }

  void update_hyperparams(const json& params) override;
  json get_hyperparams() const override;

private:
  enum class State {
    COLLECT,
    PREPARE,
    STAGE_SEND,
    STAGE_WAIT,
    NEXT_STAGE,
    FINALIZE,
    OUTPUT,
  };

  Context<E>& ctx;
  std::size_t fft_size = 0;
  unsigned nstages = 0;
  FFTFlowMode flow_mode = FFTFlowMode::DIT;
  bool scale_each_stage = false;

  std::vector<CxT> data;
  std::vector<CxT> twiddle;

  std::size_t input_idx = 0;
  std::size_t output_idx = 0;
  unsigned stage_idx = 0;
  std::size_t group_base = 0;
  std::size_t butterfly_idx = 0;
  bool inverse_latched = false;

  // Shared cycle butterfly engine.
  std::unique_ptr<R2SdfStageCycle<E>> stage_core;
  sc_core::sc_signal<bool> stg_in_valid{"stg_in_valid"};
  sc_core::sc_signal<bool> stg_in_ready{"stg_in_ready"};
  sc_core::sc_signal<T> stg_a_re{"stg_a_re"};
  sc_core::sc_signal<T> stg_a_im{"stg_a_im"};
  sc_core::sc_signal<T> stg_b_re{"stg_b_re"};
  sc_core::sc_signal<T> stg_b_im{"stg_b_im"};
  sc_core::sc_signal<T> stg_tw_re{"stg_tw_re"};
  sc_core::sc_signal<T> stg_tw_im{"stg_tw_im"};
  sc_core::sc_signal<bool> stg_use_twiddle{"stg_use_twiddle"};
  sc_core::sc_signal<bool> stg_dit_mode{"stg_dit_mode"};
  sc_core::sc_signal<bool> stg_inverse{"stg_inverse"};
  sc_core::sc_signal<bool> stg_scale_half{"stg_scale_half"};
  sc_core::sc_signal<bool> stg_out_valid{"stg_out_valid"};
  sc_core::sc_signal<bool> stg_out_ready{"stg_out_ready"};
  sc_core::sc_signal<T> stg_y0_re{"stg_y0_re"};
  sc_core::sc_signal<T> stg_y0_im{"stg_y0_im"};
  sc_core::sc_signal<T> stg_y1_re{"stg_y1_re"};
  sc_core::sc_signal<T> stg_y1_im{"stg_y1_im"};

  State state = State::COLLECT;

  static unsigned ilog2(std::size_t n);
  static std::size_t bit_reverse_index(std::size_t x, unsigned bits);
  void bit_reverse_data();
  void build_twiddle();
  std::size_t stage_span() const;
  std::size_t stage_half() const;
  std::size_t twiddle_index() const;
  bool stage_has_twiddle() const;
  void advance_butterfly();
  void drive_stage_request();
  void run();
};

}  // namespace adptsysc
