#include <adptsysc/config.hh>

#include <adptsysc/adptsysc.hh>
#include <adptsysc/ovsfdaft-cycle.hh>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <memory>
#include <stdexcept>
#include <vector>

namespace adptsysc {

using E = ADPT_TARGET;

template <typename E>
std::unique_ptr<OverlapSaveFdafCycle<E>>
OverlapSaveFdafCycle<E>::create(Context<E>& ctx,
                                sc_core::sc_module_name name,
                                std::size_t ncoeff,
                                const T* initptr) {
  return std::make_unique<OverlapSaveFdafCycle<E>>(ctx, name, ncoeff, initptr);
}

template <typename E>
OverlapSaveFdafCycle<E>::OverlapSaveFdafCycle(Context<E>& c,
                                              sc_core::sc_module_name name,
                                              std::size_t ncoeff,
                                              const T* initptr)
    : sc_core::sc_module(name),
      ctx(c),
      filter_ncoeff(ncoeff),
      fft_size(2 * ncoeff),
      block_size(ncoeff),
      initial_weights(ncoeff, T(0)),
      x_hist(ncoeff, T(0)),
      x_new(ncoeff, T(0)),
      d_block(ncoeff, T(0)),
      y_block(ncoeff, T(0)),
      e_block(ncoeff, T(0)),
      x_time(2 * ncoeff, CxT{T(0), T(0)}),
      x_freq(2 * ncoeff, CxT{T(0), T(0)}),
      y_freq(2 * ncoeff, CxT{T(0), T(0)}),
      y_time(2 * ncoeff, CxT{T(0), T(0)}),
      e_time(2 * ncoeff, CxT{T(0), T(0)}),
      e_freq(2 * ncoeff, CxT{T(0), T(0)}),
      w_freq(2 * ncoeff, CxT{T(0), T(0)}),
      w_time(2 * ncoeff, CxT{T(0), T(0)}),
      pow_est(2 * ncoeff, eps) {
  if (initptr != nullptr) {
    std::copy(initptr, initptr + filter_ncoeff, initial_weights.begin());
  }

  if constexpr (requires { E::mu_default; }) {
    mu = static_cast<double>(E::mu_default);
  }
  if constexpr (requires { E::alpha_default; }) {
    alpha = static_cast<double>(E::alpha_default);
  }
  if constexpr (requires { E::eps_default; }) {
    eps = static_cast<double>(E::eps_default);
  }
  if constexpr (requires { E::use_power_norm; }) {
    use_power_norm = static_cast<bool>(E::use_power_norm);
  }

  validate_config();
  pow_est.assign(fft_size, eps);

  fft_core = R2SdfFFTCycle<E>::create(
      ctx, "fdaf_fft_core", fft_size,
      E::use_dit ? FFTFlowMode::DIT : FFTFlowMode::DIF);

  fft_core->clk(clk);
  fft_core->rst_n(rst_n);
  fft_core->inverse(fft_inverse);
  fft_core->in_valid(fft_in_valid);
  fft_core->in_ready(fft_in_ready);
  fft_core->din_re(fft_din_re);
  fft_core->din_im(fft_din_im);
  fft_core->in_last(fft_in_last);
  fft_core->out_valid(fft_out_valid);
  fft_core->out_ready(fft_out_ready);
  fft_core->dout_re(fft_dout_re);
  fft_core->dout_im(fft_dout_im);
  fft_core->out_last(fft_out_last);
  fft_core->busy(fft_busy);

  SC_CTHREAD(run, clk.pos());
  async_reset_signal_is(rst_n, false);
}

template <typename E>
void OverlapSaveFdafCycle<E>::validate_config() const {
  if (filter_ncoeff == 0 || block_size != filter_ncoeff ||
      fft_size != 2 * filter_ncoeff ||
      (fft_size & (fft_size - 1)) != 0) {
    throw std::invalid_argument(
        "OverlapSaveFdafCycle requires M>0, N=2M and power-of-two N");
  }
  if (mu < 0.0 || alpha < 0.0 || alpha >= 1.0 || eps <= 0.0) {
    throw std::invalid_argument("invalid FDAF numeric configuration");
  }
}

template <typename E>
typename OverlapSaveFdafCycle<E>::CxT
OverlapSaveFdafCycle<E>::cmul(const CxT& a, const CxT& b) {
  return CxT{T(a.re * b.re - a.im * b.im),
             T(a.re * b.im + a.im * b.re)};
}

template <typename E>
typename OverlapSaveFdafCycle<E>::CxT
OverlapSaveFdafCycle<E>::conj_mul(const CxT& a, const CxT& b) {
  return CxT{T(a.re * b.re + a.im * b.im),
             T(a.re * b.im - a.im * b.re)};
}

template <typename E>
void OverlapSaveFdafCycle<E>::reset_vectors() {
  std::fill(x_hist.begin(), x_hist.end(), T(0));
  std::fill(x_new.begin(), x_new.end(), T(0));
  std::fill(d_block.begin(), d_block.end(), T(0));
  std::fill(y_block.begin(), y_block.end(), T(0));
  std::fill(e_block.begin(), e_block.end(), T(0));

  const CxT zero{T(0), T(0)};
  std::fill(x_time.begin(), x_time.end(), zero);
  std::fill(x_freq.begin(), x_freq.end(), zero);
  std::fill(y_freq.begin(), y_freq.end(), zero);
  std::fill(y_time.begin(), y_time.end(), zero);
  std::fill(e_time.begin(), e_time.end(), zero);
  std::fill(e_freq.begin(), e_freq.end(), zero);
  std::fill(w_freq.begin(), w_freq.end(), zero);
  std::fill(w_time.begin(), w_time.end(), zero);
  std::fill(pow_est.begin(), pow_est.end(), eps);

  collect_idx = 0;
  bin_idx = 0;
  out_idx = 0;
  fft_send_idx = 0;
  fft_recv_idx = 0;
  fft_send_presented = false;
  output_presented = false;
  block_count = 0;
}

template <typename E>
void OverlapSaveFdafCycle<E>::start_fft(std::vector<CxT>& src,
                                        std::vector<CxT>& dst,
                                        bool inv,
                                        State next,
                                        State send_state) {
  fft_send_vec = &src;
  fft_recv_vec = &dst;
  fft_inverse_mode = inv;
  fft_next_state = next;
  fft_send_idx = 0;
  fft_recv_idx = 0;
  fft_send_presented = false;
  std::fill(dst.begin(), dst.end(), CxT{T(0), T(0)});
  fft_inverse.write(inv);
  state = send_state;
}

template <typename E>
bool OverlapSaveFdafCycle<E>::fft_send_step() {
  if (fft_send_vec == nullptr) {
    SC_REPORT_FATAL(this->name(), "null FFT send vector");
  }

  auto drive_current = [&]() {
    const CxT z = (*fft_send_vec)[fft_send_idx];
    fft_din_re.write(z.re);
    fft_din_im.write(z.im);
    fft_in_last.write(fft_send_idx + 1 == fft_size);
    fft_in_valid.write(true);
  };

  // SystemC signal writes take effect after the current evaluation phase.
  // Present a word for a full clock before counting the ready/valid handshake.
  if (!fft_send_presented) {
    drive_current();
    fft_send_presented = true;
    return false;
  }

  if (fft_in_ready.read()) {
    ++fft_send_idx;
    if (fft_send_idx == fft_size) {
      fft_in_valid.write(false);
      fft_in_last.write(false);
      fft_send_presented = false;
      return true;
    }

    // Keep valid asserted and advance data for the next cycle.
    drive_current();
  }
  return false;
}

template <typename E>
bool OverlapSaveFdafCycle<E>::fft_recv_step() {
  if (fft_recv_vec == nullptr) {
    SC_REPORT_FATAL(this->name(), "null FFT receive vector");
  }

  if (fft_out_valid.read()) {
    (*fft_recv_vec)[fft_recv_idx] = CxT{fft_dout_re.read(), fft_dout_im.read()};
    const bool expected_last = fft_recv_idx + 1 == fft_size;
    if (fft_out_last.read() != expected_last) {
      SC_REPORT_WARNING(this->name(), "FFT child out_last mismatch");
    }
    ++fft_recv_idx;
    if (fft_recv_idx == fft_size) {
      fft_recv_idx = 0;
      return true;
    }
  }
  return false;
}

template <typename E>
void OverlapSaveFdafCycle<E>::run() {
  reset_vectors();
  state = State::INIT_W_PREP;

  in_ready.write(false);
  out_valid.write(false);
  out_last.write(false);
  y_out.write(T(0));
  e_out.write(T(0));
  busy.write(true);

  fft_inverse.write(false);
  fft_in_valid.write(false);
  fft_in_last.write(false);
  fft_din_re.write(T(0));
  fft_din_im.write(T(0));
  fft_out_ready.write(true);
  wait();

  while (true) {
    switch (state) {
      case State::INIT_W_PREP:
        busy.write(true);
        in_ready.write(false);
        std::fill(w_time.begin(), w_time.end(), CxT{T(0), T(0)});
        for (std::size_t i = 0; i < filter_ncoeff; ++i) {
          w_time[i] = CxT{initial_weights[i], T(0)};
        }
        start_fft(w_time, w_freq, false,
                  State::COLLECT, State::INIT_W_SEND);
        break;

      case State::INIT_W_SEND:
        if (fft_send_step()) {
          state = State::INIT_W_RECV;
        }
        break;

      case State::INIT_W_RECV:
        if (fft_recv_step()) {
          state = fft_next_state;
        }
        break;

      case State::COLLECT:
        busy.write(false);
        in_ready.write(true);
        out_valid.write(false);
        out_last.write(false);
        if (in_valid.read()) {
          if (collect_idx == 0) {
            train_block = train.read();
          } else if (train.read() != train_block) {
            SC_REPORT_WARNING(this->name(),
                              "train changed within one FDAF block");
          }
          x_new[collect_idx] = x_in.read();
          d_block[collect_idx] = d_in.read();
          ++collect_idx;
          if (collect_idx == block_size) {
            collect_idx = 0;
            in_ready.write(false);
            busy.write(true);
            state = State::PREP_X;
          }
        }
        break;

      case State::PREP_X:
        for (std::size_t i = 0; i < block_size; ++i) {
          x_time[i] = CxT{x_hist[i], T(0)};
          x_time[block_size + i] = CxT{x_new[i], T(0)};
        }
        start_fft(x_time, x_freq, false,
                  State::FILTER_MUL, State::FFT_X_SEND);
        break;

      case State::FFT_X_SEND:
        if (fft_send_step()) {
          state = State::FFT_X_RECV;
        }
        break;

      case State::FFT_X_RECV:
        if (fft_recv_step()) {
          bin_idx = 0;
          state = fft_next_state;
        }
        break;

      case State::FILTER_MUL:
        y_freq[bin_idx] = cmul(x_freq[bin_idx], w_freq[bin_idx]);
        ++bin_idx;
        if (bin_idx == fft_size) {
          bin_idx = 0;
          start_fft(y_freq, y_time, true,
                    State::OUTPUT_ERROR, State::IFFT_Y_SEND);
        }
        break;

      case State::IFFT_Y_SEND:
        if (fft_send_step()) {
          state = State::IFFT_Y_RECV;
        }
        break;

      case State::IFFT_Y_RECV:
        if (fft_recv_step()) {
          out_idx = 0;
          output_presented = false;
          state = fft_next_state;
        }
        break;

      case State::OUTPUT_ERROR:
        if (!output_presented) {
          const T y = y_time[block_size + out_idx].re;
          const T e = T(d_block[out_idx] - y);
          y_block[out_idx] = y;
          e_block[out_idx] = e;
          y_out.write(y);
          e_out.write(e);
          out_last.write(out_idx + 1 == block_size);
          out_valid.write(true);
          output_presented = true;
        } else if (out_ready.read()) {
          out_valid.write(false);
          out_last.write(false);
          output_presented = false;
          ++out_idx;
          if (out_idx == block_size) {
            out_idx = 0;
            state = train_block ? State::PREP_E
                                : State::FINISH_BLOCK;
          }
        }
        break;

      case State::PREP_E:
        std::fill(e_time.begin(), e_time.end(), CxT{T(0), T(0)});
        for (std::size_t i = 0; i < block_size; ++i) {
          e_time[block_size + i] = CxT{e_block[i], T(0)};
        }
        start_fft(e_time, e_freq, false,
                  State::POWER_GRAD_UPDATE, State::FFT_E_SEND);
        break;

      case State::FFT_E_SEND:
        if (fft_send_step()) {
          state = State::FFT_E_RECV;
        }
        break;

      case State::FFT_E_RECV:
        if (fft_recv_step()) {
          bin_idx = 0;
          state = fft_next_state;
        }
        break;

      case State::POWER_GRAD_UPDATE: {
        const CxT x = x_freq[bin_idx];
        const CxT e = e_freq[bin_idx];
        const double xr = static_cast<double>(x.re);
        const double xi = static_cast<double>(x.im);
        const double xpow = xr * xr + xi * xi;
        pow_est[bin_idx] = alpha * pow_est[bin_idx] +
                           (1.0 - alpha) * xpow;

        const CxT gq = conj_mul(x, e);
        double gr = static_cast<double>(gq.re);
        double gi = static_cast<double>(gq.im);
        if (use_power_norm) {
          const double den = pow_est[bin_idx] + eps;
          gr /= den;
          gi /= den;
        }

        w_freq[bin_idx] = CxT{
            T(static_cast<double>(w_freq[bin_idx].re) + mu * gr),
            T(static_cast<double>(w_freq[bin_idx].im) + mu * gi)};

        ++bin_idx;
        if (bin_idx == fft_size) {
          bin_idx = 0;
          start_fft(w_freq, w_time, true,
                    State::CONSTRAINT_ZERO, State::IFFT_W_SEND);
        }
        break;
      }

      case State::IFFT_W_SEND:
        if (fft_send_step()) {
          state = State::IFFT_W_RECV;
        }
        break;

      case State::IFFT_W_RECV:
        if (fft_recv_step()) {
          state = fft_next_state;
        }
        break;

      case State::CONSTRAINT_ZERO:
        for (std::size_t i = filter_ncoeff; i < fft_size; ++i) {
          w_time[i] = CxT{T(0), T(0)};
        }
        start_fft(w_time, w_freq, false,
                  State::FINISH_BLOCK, State::FFT_W_SEND);
        break;

      case State::FFT_W_SEND:
        if (fft_send_step()) {
          state = State::FFT_W_RECV;
        }
        break;

      case State::FFT_W_RECV:
        if (fft_recv_step()) {
          state = fft_next_state;
        }
        break;

      case State::FINISH_BLOCK:
        x_hist = x_new;
        ++block_count;
        state = State::COLLECT;
        break;
    }

    wait();
  }
}

template <typename E>
void OverlapSaveFdafCycle<E>::update_hyperparams(const json& params) {
  if (params.contains("mu")) {
    mu = params.at("mu").template get<double>();
  }
  if (params.contains("alpha")) {
    alpha = params.at("alpha").template get<double>();
  }
  if (params.contains("eps")) {
    eps = params.at("eps").template get<double>();
  }
  if (params.contains("use_power_norm")) {
    use_power_norm = params.at("use_power_norm").template get<bool>();
  }
  validate_config();
}

template <typename E>
json OverlapSaveFdafCycle<E>::get_hyperparams() const {
  return {
    {"otype", "overlap_save_fdaf_cycle"},
    {"filter_ncoeff", filter_ncoeff},
    {"fft_size", fft_size},
    {"block_size", block_size},
    {"mu", mu},
    {"alpha", alpha},
    {"eps", eps},
    {"use_power_norm", use_power_norm},
    {"shared_fft_core", true},
    {"blocks_processed", block_count},
  };
}

// -----------------------------------------------------------------------------
// Smoke test: identity plant, training disabled.  Zero initial coefficients
// must produce y=0 and e=d exactly; this validates block collection, shared FFT
// scheduling, overlap-save output timing and ready/valid behavior.
// -----------------------------------------------------------------------------
template <typename E>
class OverlapSaveFdafCycleTB : public sc_core::sc_module {
public:
  using T = typename E::Fxpt_T;

  sc_core::sc_in<bool> clk{"clk"};
  sc_core::sc_out<bool> rst_n{"rst_n"};
  sc_core::sc_out<bool> train{"train"};
  sc_core::sc_out<bool> in_valid{"in_valid"};
  sc_core::sc_in<bool> in_ready{"in_ready"};
  sc_core::sc_out<T> x_in{"x_in"};
  sc_core::sc_out<T> d_in{"d_in"};
  sc_core::sc_in<bool> out_valid{"out_valid"};
  sc_core::sc_out<bool> out_ready{"out_ready"};
  sc_core::sc_in<T> y_out{"y_out"};
  sc_core::sc_in<T> e_out{"e_out"};
  sc_core::sc_in<bool> out_last{"out_last"};

  bool pass = true;

  OverlapSaveFdafCycleTB(sc_core::sc_module_name name,
                         Context<E>& ctx,
                         std::size_t block_size)
      : sc_core::sc_module(name), ctx(ctx), block_size(block_size) {
    SC_THREAD(run);
    sensitive << clk.pos();
  }

private:
  Context<E>& ctx;
  std::size_t block_size = 0;

  void run() {
    rst_n.write(false);
    train.write(false);
    in_valid.write(false);
    x_in.write(T(0));
    d_in.write(T(0));
    out_ready.write(true);
    wait(3);
    rst_n.write(true);
    wait();

    for (std::size_t i = 0; i < block_size; ++i) {
      while (!in_ready.read()) {
        wait();
      }
      const T x = T(static_cast<int>(i & 3u) - 1);
      x_in.write(x);
      d_in.write(x);
      in_valid.write(true);
      wait();
      in_valid.write(false);
    }

    std::size_t idx = 0;
    while (idx < block_size) {
      wait();
      if (!out_valid.read()) {
        continue;
      }
      const T expected_e = T(static_cast<int>(idx & 3u) - 1);
      if (y_out.read() != T(0) || e_out.read() != expected_e) {
        pass = false;
        SC_REPORT_ERROR("OverlapSaveFdafCycleTB", "FDAF smoke mismatch");
      }
      if (out_last.read() != (idx + 1 == block_size)) {
        pass = false;
        SC_REPORT_ERROR("OverlapSaveFdafCycleTB", "FDAF out_last mismatch");
      }
      ++idx;
    }

    if (ctx.arg.verbose) {
      Out(ctx) << "OverlapSaveFdafCycle testbench pass="
               << (pass ? "true" : "false") << "\n";
    }
    sc_core::sc_stop();
  }
};

template <typename E>
bool OverlapSaveFdafCycle<E>::run_testbench(Context<E>& ctx) {
  sc_core::sc_clock clk("clk", sc_core::sc_time(10, sc_core::SC_NS));
  sc_core::sc_signal<bool> rst_n;
  sc_core::sc_signal<bool> train;
  sc_core::sc_signal<bool> in_valid;
  sc_core::sc_signal<bool> in_ready;
  sc_core::sc_signal<T> x_in;
  sc_core::sc_signal<T> d_in;
  sc_core::sc_signal<bool> out_valid;
  sc_core::sc_signal<bool> out_ready;
  sc_core::sc_signal<T> y_out;
  sc_core::sc_signal<T> e_out;
  sc_core::sc_signal<bool> out_last;
  sc_core::sc_signal<bool> busy;

  auto dut = OverlapSaveFdafCycle<E>::create(
      ctx, "overlap_save_fdaf_cycle_dut", E::filter_ncoeff);
  OverlapSaveFdafCycleTB<E> tb(
      "overlap_save_fdaf_cycle_tb", ctx, E::filter_ncoeff);

  dut->clk(clk);
  dut->rst_n(rst_n);
  dut->train(train);
  dut->in_valid(in_valid);
  dut->in_ready(in_ready);
  dut->x_in(x_in);
  dut->d_in(d_in);
  dut->out_valid(out_valid);
  dut->out_ready(out_ready);
  dut->y_out(y_out);
  dut->e_out(e_out);
  dut->out_last(out_last);
  dut->busy(busy);

  tb.clk(clk);
  tb.rst_n(rst_n);
  tb.train(train);
  tb.in_valid(in_valid);
  tb.in_ready(in_ready);
  tb.x_in(x_in);
  tb.d_in(d_in);
  tb.out_valid(out_valid);
  tb.out_ready(out_ready);
  tb.y_out(y_out);
  tb.e_out(e_out);
  tb.out_last(out_last);

  sc_core::sc_start();
  return tb.pass;
}

template class OverlapSaveFdafCycle<E>;

}  // namespace adptsysc
