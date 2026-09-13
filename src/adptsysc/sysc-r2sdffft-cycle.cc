#include <adptsysc/config.hh>

#include <adptsysc/adptsysc.hh>
#include <adptsysc/sysc-r2sdffft-cycle.hh>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace adptsysc {

using E = ADPT_TARGET;

// -----------------------------------------------------------------------------
// R2SdfCtrlCycle
// -----------------------------------------------------------------------------
template <typename E>
unsigned R2SdfCtrlCycle<E>::ilog2(std::size_t n) {
  unsigned v = 0;
  while (n > 1) {
    n >>= 1;
    ++v;
  }
  return v;
}

template <typename E>
R2SdfCtrlCycle<E>::R2SdfCtrlCycle(sc_core::sc_module_name name,
                                  std::size_t size,
                                  FFTFlowMode mode)
    : sc_core::sc_module(name),
      stage_sel("stage_sel", ilog2(size)),
      twiddle_enable("twiddle_enable", ilog2(size)),
      twiddle_addr("twiddle_addr", ilog2(size)),
      fft_size(size),
      nstages(ilog2(size)),
      flow_mode(mode) {
  if (fft_size == 0 || (fft_size & (fft_size - 1)) != 0) {
    SC_REPORT_FATAL(this->name(), "fft_size must be a power of two");
  }

  SC_CTHREAD(run, clk.pos());
  async_reset_signal_is(rst_n, false);
}

template <typename E>
void R2SdfCtrlCycle<E>::drive_decode(std::size_t c, bool fire) {
  count.write(static_cast<unsigned long long>(c));
  frame_first.write(fire && c == 0);
  frame_last.write(fire && c == fft_size - 1);

  for (unsigned p = 0; p < nstages; ++p) {
    const unsigned bit_idx =
        flow_mode == FFTFlowMode::DIF ? (nstages - 1 - p) : p;
    const bool sel = fire && (((c >> bit_idx) & 1u) != 0u);
    stage_sel[p].write(sel);

    bool tw_en = false;
    std::size_t tw_addr = 0;
    if (fire) {
      if (flow_mode == FFTFlowMode::DIT) {
        if (p > 0) {
          const std::size_t half = std::size_t{1} << p;
          const std::size_t local = c & (half - 1);
          tw_addr = local * (fft_size >> (p + 1));
          tw_en = sel;
        }
      } else {
        if (p + 1 < nstages) {
          const std::size_t half = std::size_t{1} << (nstages - p - 1);
          const std::size_t local = c & (half - 1);
          tw_addr = local * (fft_size >> (nstages - p));
          tw_en = sel;
        }
      }
    }

    twiddle_enable[p].write(tw_en);
    twiddle_addr[p].write(static_cast<unsigned long long>(tw_addr));
  }
}

template <typename E>
void R2SdfCtrlCycle<E>::run() {
  std::size_t c = 0;
  drive_decode(0, false);
  wait();

  while (true) {
    const bool fire = sample_fire.read();
    drive_decode(c, fire);
    if (fire) {
      c = (c + 1) % fft_size;
    }
    wait();
  }
}

// -----------------------------------------------------------------------------
// R2SdfStageCycle
// -----------------------------------------------------------------------------
template <typename E>
constexpr unsigned R2SdfStageCycle<E>::base_latency() {
  if constexpr (requires { E::butterfly_latency; }) {
    return std::max(1u, static_cast<unsigned>(E::butterfly_latency));
  }
  return 1u;
}

template <typename E>
typename R2SdfStageCycle<E>::Pair
R2SdfStageCycle<E>::butterfly(const T& ar,
                              const T& ai,
                              const T& br,
                              const T& bi,
                              const T& wr,
                              const T& wi_in,
                              bool use_tw,
                              bool dit,
                              bool inv,
                              bool scale_half) {
  const T wi = inv ? T(-wi_in) : wi_in;

  auto cmul = [](const T& xr, const T& xi,
                 const T& yr, const T& yi) {
    return std::pair<T, T>{T(xr * yr - xi * yi),
                           T(xr * yi + xi * yr)};
  };

  Pair y{};
  if (dit) {
    T tr = br;
    T ti = bi;
    if (use_tw) {
      const auto p = cmul(br, bi, wr, wi);
      tr = p.first;
      ti = p.second;
    }
    y.y0_re = T(ar + tr);
    y.y0_im = T(ai + ti);
    y.y1_re = T(ar - tr);
    y.y1_im = T(ai - ti);
  } else {
    const T sr = T(ar + br);
    const T si = T(ai + bi);
    const T dr = T(ar - br);
    const T di = T(ai - bi);

    y.y0_re = sr;
    y.y0_im = si;
    y.y1_re = dr;
    y.y1_im = di;

    if (use_tw) {
      const auto p = cmul(dr, di, wr, wi);
      y.y1_re = p.first;
      y.y1_im = p.second;
    }
  }

  if (scale_half) {
    y.y0_re = T(y.y0_re / T(2));
    y.y0_im = T(y.y0_im / T(2));
    y.y1_re = T(y.y1_re / T(2));
    y.y1_im = T(y.y1_im / T(2));
  }
  return y;
}

template <typename E>
R2SdfStageCycle<E>::R2SdfStageCycle(sc_core::sc_module_name name)
    : sc_core::sc_module(name) {
  SC_CTHREAD(run, clk.pos());
  async_reset_signal_is(rst_n, false);
}

template <typename E>
void R2SdfStageCycle<E>::run() {
  bool active = false;
  bool valid = false;
  unsigned cycles_left = 0;
  Pair pending{};

  in_ready.write(true);
  out_valid.write(false);
  y0_re.write(T(0));
  y0_im.write(T(0));
  y1_re.write(T(0));
  y1_im.write(T(0));
  wait();

  while (true) {
    if (valid && out_ready.read()) {
      valid = false;
    }

    if (active) {
      if (cycles_left > 1) {
        --cycles_left;
      } else {
        active = false;
        valid = true;
        y0_re.write(pending.y0_re);
        y0_im.write(pending.y0_im);
        y1_re.write(pending.y1_re);
        y1_im.write(pending.y1_im);
      }
    }

    const bool ready_now = !active && !valid;
    in_ready.write(ready_now);

    if (ready_now && in_valid.read()) {
      pending = butterfly(a_re.read(), a_im.read(),
                          b_re.read(), b_im.read(),
                          tw_re.read(), tw_im.read(),
                          use_twiddle.read(), dit_mode.read(),
                          inverse.read(), scale_half.read());

      unsigned latency = base_latency();
      if (use_twiddle.read()) {
        if constexpr (requires { E::cmplxmul_latency; }) {
          latency += static_cast<unsigned>(E::cmplxmul_latency);
        } else {
          latency += 1u;
        }
      }
      cycles_left = std::max(1u, latency);
      active = true;
      in_ready.write(false);
    }

    out_valid.write(valid);
    wait();
  }
}

// -----------------------------------------------------------------------------
// R2SdfFFTCycle
// -----------------------------------------------------------------------------
template <typename E>
unsigned R2SdfFFTCycle<E>::ilog2(std::size_t n) {
  unsigned v = 0;
  while (n > 1) {
    n >>= 1;
    ++v;
  }
  return v;
}

template <typename E>
std::size_t R2SdfFFTCycle<E>::bit_reverse_index(std::size_t x,
                                                unsigned bits) {
  std::size_t y = 0;
  for (unsigned i = 0; i < bits; ++i) {
    y = (y << 1) | ((x >> i) & 1u);
  }
  return y;
}

template <typename E>
std::unique_ptr<R2SdfFFTCycle<E>>
R2SdfFFTCycle<E>::create(Context<E>& ctx,
                         sc_core::sc_module_name name,
                         std::size_t size,
                         FFTFlowMode mode) {
  return std::make_unique<R2SdfFFTCycle<E>>(ctx, name, size, mode);
}

template <typename E>
R2SdfFFTCycle<E>::R2SdfFFTCycle(Context<E>& c,
                                sc_core::sc_module_name name,
                                std::size_t size,
                                FFTFlowMode mode)
    : sc_core::sc_module(name),
      ctx(c),
      fft_size(size),
      nstages(ilog2(size)),
      flow_mode(mode),
      scale_each_stage(E::scale_each_stage),
      data(size, CxT{T(0), T(0)}),
      twiddle(size / 2, CxT{T(0), T(0)}) {
  if (fft_size < 2 || (fft_size & (fft_size - 1)) != 0) {
    SC_REPORT_FATAL(this->name(), "fft_size must be a power of two >= 2");
  }

  build_twiddle();

  stage_core = std::make_unique<R2SdfStageCycle<E>>("stage_core");
  stage_core->clk(clk);
  stage_core->rst_n(rst_n);
  stage_core->in_valid(stg_in_valid);
  stage_core->in_ready(stg_in_ready);
  stage_core->a_re(stg_a_re);
  stage_core->a_im(stg_a_im);
  stage_core->b_re(stg_b_re);
  stage_core->b_im(stg_b_im);
  stage_core->tw_re(stg_tw_re);
  stage_core->tw_im(stg_tw_im);
  stage_core->use_twiddle(stg_use_twiddle);
  stage_core->dit_mode(stg_dit_mode);
  stage_core->inverse(stg_inverse);
  stage_core->scale_half(stg_scale_half);
  stage_core->out_valid(stg_out_valid);
  stage_core->out_ready(stg_out_ready);
  stage_core->y0_re(stg_y0_re);
  stage_core->y0_im(stg_y0_im);
  stage_core->y1_re(stg_y1_re);
  stage_core->y1_im(stg_y1_im);

  SC_CTHREAD(run, clk.pos());
  async_reset_signal_is(rst_n, false);
}

template <typename E>
void R2SdfFFTCycle<E>::build_twiddle() {
  constexpr double pi = 3.141592653589793238462643383279502884;
  for (std::size_t k = 0; k < fft_size / 2; ++k) {
    const double a = -2.0 * pi * static_cast<double>(k) /
                     static_cast<double>(fft_size);
    twiddle[k] = CxT{T(std::cos(a)), T(std::sin(a))};
  }
}

template <typename E>
void R2SdfFFTCycle<E>::bit_reverse_data() {
  for (std::size_t i = 0; i < fft_size; ++i) {
    const std::size_t j = bit_reverse_index(i, nstages);
    if (j > i) {
      std::swap(data[i], data[j]);
    }
  }
}

template <typename E>
std::size_t R2SdfFFTCycle<E>::stage_span() const {
  if (flow_mode == FFTFlowMode::DIT) {
    return std::size_t{1} << (stage_idx + 1);
  }
  return std::size_t{1} << (nstages - stage_idx);
}

template <typename E>
std::size_t R2SdfFFTCycle<E>::stage_half() const {
  return stage_span() >> 1;
}

template <typename E>
std::size_t R2SdfFFTCycle<E>::twiddle_index() const {
  const std::size_t span = stage_span();
  const std::size_t stride = fft_size / span;
  return (butterfly_idx * stride) & (fft_size - 1);
}

template <typename E>
bool R2SdfFFTCycle<E>::stage_has_twiddle() const {
  if (flow_mode == FFTFlowMode::DIT) {
    return stage_idx > 0;
  }
  return stage_idx + 1 < nstages;
}

template <typename E>
void R2SdfFFTCycle<E>::drive_stage_request() {
  const std::size_t i0 = group_base + butterfly_idx;
  const std::size_t i1 = i0 + stage_half();
  const CxT& a = data[i0];
  const CxT& b = data[i1];

  std::size_t k = twiddle_index();
  if (k >= twiddle.size()) {
    k %= twiddle.size();
  }
  const CxT w = twiddle[k];

  stg_a_re.write(a.re);
  stg_a_im.write(a.im);
  stg_b_re.write(b.re);
  stg_b_im.write(b.im);
  stg_tw_re.write(w.re);
  stg_tw_im.write(w.im);
  stg_use_twiddle.write(stage_has_twiddle());
  stg_dit_mode.write(flow_mode == FFTFlowMode::DIT);
  stg_inverse.write(inverse_latched);
  stg_scale_half.write(inverse_latched && scale_each_stage);
}

template <typename E>
void R2SdfFFTCycle<E>::advance_butterfly() {
  ++butterfly_idx;
  if (butterfly_idx >= stage_half()) {
    butterfly_idx = 0;
    group_base += stage_span();
  }
}

template <typename E>
void R2SdfFFTCycle<E>::run() {
  state = State::COLLECT;
  input_idx = 0;
  output_idx = 0;
  stage_idx = 0;
  group_base = 0;
  butterfly_idx = 0;
  inverse_latched = false;

  in_ready.write(true);
  out_valid.write(false);
  out_last.write(false);
  dout_re.write(T(0));
  dout_im.write(T(0));
  busy.write(false);

  stg_in_valid.write(false);
  stg_out_ready.write(true);
  stg_a_re.write(T(0));
  stg_a_im.write(T(0));
  stg_b_re.write(T(0));
  stg_b_im.write(T(0));
  stg_tw_re.write(T(1));
  stg_tw_im.write(T(0));
  stg_use_twiddle.write(false);
  stg_dit_mode.write(true);
  stg_inverse.write(false);
  stg_scale_half.write(false);

  bool output_presented = false;
  wait();

  while (true) {
    switch (state) {
      case State::COLLECT: {
        busy.write(false);
        in_ready.write(true);
        out_valid.write(false);
        out_last.write(false);

        if (in_valid.read()) {
          if (input_idx == 0) {
            inverse_latched = inverse.read();
          }

          data[input_idx] = CxT{din_re.read(), din_im.read()};
          const bool final_sample = (input_idx + 1 == fft_size);
          if (in_last.read() != final_sample) {
            SC_REPORT_WARNING(this->name(),
                              "in_last does not match configured fft_size");
          }

          ++input_idx;
          if (input_idx == fft_size) {
            input_idx = 0;
            in_ready.write(false);
            busy.write(true);
            state = State::PREPARE;
          }
        }
        break;
      }

      case State::PREPARE:
        busy.write(true);
        in_ready.write(false);
        if (flow_mode == FFTFlowMode::DIT) {
          bit_reverse_data();
        }
        stage_idx = 0;
        group_base = 0;
        butterfly_idx = 0;
        state = State::STAGE_SEND;
        break;

      case State::STAGE_SEND:
        busy.write(true);
        drive_stage_request();
        stg_in_valid.write(true);
        if (stg_in_ready.read()) {
          state = State::STAGE_WAIT;
        }
        break;

      case State::STAGE_WAIT:
        busy.write(true);
        stg_in_valid.write(false);
        if (stg_out_valid.read()) {
          const std::size_t i0 = group_base + butterfly_idx;
          const std::size_t i1 = i0 + stage_half();
          data[i0] = CxT{stg_y0_re.read(), stg_y0_im.read()};
          data[i1] = CxT{stg_y1_re.read(), stg_y1_im.read()};

          advance_butterfly();
          if (group_base >= fft_size) {
            state = State::NEXT_STAGE;
          } else {
            state = State::STAGE_SEND;
          }
        }
        break;

      case State::NEXT_STAGE:
        group_base = 0;
        butterfly_idx = 0;
        ++stage_idx;
        if (stage_idx >= nstages) {
          state = State::FINALIZE;
        } else {
          state = State::STAGE_SEND;
        }
        break;

      case State::FINALIZE:
        if (flow_mode == FFTFlowMode::DIF) {
          bit_reverse_data();
        }
        output_idx = 0;
        output_presented = false;
        state = State::OUTPUT;
        break;

      case State::OUTPUT: {
        busy.write(true);
        if (!output_presented) {
          CxT z = data[output_idx];
          if (inverse_latched && !scale_each_stage) {
            z = CxT{T(z.re / T(static_cast<int>(fft_size))),
                    T(z.im / T(static_cast<int>(fft_size)))};
          }
          dout_re.write(z.re);
          dout_im.write(z.im);
          out_last.write(output_idx + 1 == fft_size);
          out_valid.write(true);
          output_presented = true;
        } else if (out_ready.read()) {
          out_valid.write(false);
          out_last.write(false);
          output_presented = false;
          ++output_idx;
          if (output_idx == fft_size) {
            output_idx = 0;
            busy.write(false);
            state = State::COLLECT;
          }
        }
        break;
      }
    }

    wait();
  }
}

template <typename E>
void R2SdfFFTCycle<E>::update_hyperparams(const json& params) {
  if (params.contains("scale_each_stage")) {
    scale_each_stage = params.at("scale_each_stage").template get<bool>();
  }
}

template <typename E>
json R2SdfFFTCycle<E>::get_hyperparams() const {
  return {
      {"otype", "r2sdf_fft_cycle"},
      {"fft_size", fft_size},
      {"flow_mode", flow_mode == FFTFlowMode::DIT ? "dit" : "dif"},
      {"scale_each_stage", scale_each_stage},
      {"butterflies_per_cycle", 1},
  };
}

// -----------------------------------------------------------------------------
// Testbench
// -----------------------------------------------------------------------------
template <typename E>
class R2SdfFFTCycleTB : public sc_core::sc_module {
public:
  using T = typename E::Fxpt_T;

  sc_core::sc_in<bool> clk{"clk"};
  sc_core::sc_out<bool> rst_n{"rst_n"};
  sc_core::sc_out<bool> inverse{"inverse"};
  sc_core::sc_out<bool> in_valid{"in_valid"};
  sc_core::sc_in<bool> in_ready{"in_ready"};
  sc_core::sc_out<T> din_re{"din_re"};
  sc_core::sc_out<T> din_im{"din_im"};
  sc_core::sc_out<bool> in_last{"in_last"};
  sc_core::sc_in<bool> out_valid{"out_valid"};
  sc_core::sc_out<bool> out_ready{"out_ready"};
  sc_core::sc_in<T> dout_re{"dout_re"};
  sc_core::sc_in<T> dout_im{"dout_im"};
  sc_core::sc_in<bool> out_last{"out_last"};

  bool pass = true;

  R2SdfFFTCycleTB(sc_core::sc_module_name name,
                   Context<E>& ctx,
                   std::size_t fft_size)
      : sc_core::sc_module(name), ctx(ctx), fft_size(fft_size) {
    SC_THREAD(run);
    sensitive << clk.pos();
  }

private:
  Context<E>& ctx;
  std::size_t fft_size = 0;

  void run() {
    rst_n.write(false);
    inverse.write(ctx.arg.compute_ifft);
    in_valid.write(false);
    din_re.write(T(0));
    din_im.write(T(0));
    in_last.write(false);
    out_ready.write(true);
    wait(3);
    rst_n.write(true);
    wait();

    // Exact fixed-point smoke vectors:
    // FFT : delta[n] -> all ones.
    // IFFT: all ones -> delta[n].
    for (std::size_t i = 0; i < fft_size; ++i) {
      const T re = ctx.arg.compute_ifft ? T(1) : (i == 0 ? T(1) : T(0));
      while (!in_ready.read()) {
        wait();
      }
      din_re.write(re);
      din_im.write(T(0));
      in_last.write(i + 1 == fft_size);
      in_valid.write(true);
      wait();
      in_valid.write(false);
      in_last.write(false);
    }

    std::size_t out_idx = 0;
    while (out_idx < fft_size) {
      wait();
      if (!out_valid.read()) {
        continue;
      }

      const T exp_re = ctx.arg.compute_ifft
                           ? (out_idx == 0 ? T(1) : T(0))
                           : T(1);
      const T exp_im = T(0);
      if (dout_re.read() != exp_re || dout_im.read() != exp_im) {
        pass = false;
        SC_REPORT_ERROR("R2SdfFFTCycleTB", "FFT/IFFT smoke mismatch");
      }
      if (out_last.read() != (out_idx + 1 == fft_size)) {
        pass = false;
        SC_REPORT_ERROR("R2SdfFFTCycleTB", "out_last mismatch");
      }
      ++out_idx;
    }

    if (ctx.arg.verbose) {
      Out(ctx) << "R2SdfFFTCycle testbench pass="
               << (pass ? "true" : "false") << "\n";
    }
    sc_core::sc_stop();
  }
};

template <typename E>
bool R2SdfFFTCycle<E>::run_testbench(Context<E>& ctx) {
  sc_core::sc_clock clk("clk", sc_core::sc_time(10, sc_core::SC_NS));
  sc_core::sc_signal<bool> rst_n;
  sc_core::sc_signal<bool> inverse;
  sc_core::sc_signal<bool> in_valid;
  sc_core::sc_signal<bool> in_ready;
  sc_core::sc_signal<T> din_re;
  sc_core::sc_signal<T> din_im;
  sc_core::sc_signal<bool> in_last;
  sc_core::sc_signal<bool> out_valid;
  sc_core::sc_signal<bool> out_ready;
  sc_core::sc_signal<T> dout_re;
  sc_core::sc_signal<T> dout_im;
  sc_core::sc_signal<bool> out_last;
  sc_core::sc_signal<bool> busy;

  auto dut = R2SdfFFTCycle<E>::create(
      ctx, "r2sdf_fft_cycle_dut", E::fft_size,
      E::use_dit ? FFTFlowMode::DIT : FFTFlowMode::DIF);
  R2SdfFFTCycleTB<E> tb("r2sdf_fft_cycle_tb", ctx, E::fft_size);

  dut->clk(clk);
  dut->rst_n(rst_n);
  dut->inverse(inverse);
  dut->in_valid(in_valid);
  dut->in_ready(in_ready);
  dut->din_re(din_re);
  dut->din_im(din_im);
  dut->in_last(in_last);
  dut->out_valid(out_valid);
  dut->out_ready(out_ready);
  dut->dout_re(dout_re);
  dut->dout_im(dout_im);
  dut->out_last(out_last);
  dut->busy(busy);

  tb.clk(clk);
  tb.rst_n(rst_n);
  tb.inverse(inverse);
  tb.in_valid(in_valid);
  tb.in_ready(in_ready);
  tb.din_re(din_re);
  tb.din_im(din_im);
  tb.in_last(in_last);
  tb.out_valid(out_valid);
  tb.out_ready(out_ready);
  tb.dout_re(dout_re);
  tb.dout_im(dout_im);
  tb.out_last(out_last);

  sc_core::sc_start();
  return tb.pass;
}

template class R2SdfCtrlCycle<E>;
template class R2SdfStageCycle<E>;
template class R2SdfFFTCycle<E>;

}  // namespace adptsysc
