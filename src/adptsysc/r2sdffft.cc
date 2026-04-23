#include <adptsysc/config.hh>
#include <stdint.h>
#include <tlm>
#include <tlm_utils/simple_target_socket.h>
#include <tlm_utils/simple_initiator_socket.h>
#include <adptsysc/adptsysc.hh>
#include <memory>
#include <adptsysc/r2sdffft.hh>
#include <algorithm>
#include <cmath>
#include <sstream>
namespace adptsysc {
using E = ADPT_TARGET;


// -----------------------------------------------------------------------------
// R2SdfStageTLM
// -----------------------------------------------------------------------------
template <typename E>
std::unique_ptr<R2SdfStageTLM<E>>
R2SdfStageTLM<E>::create(Context<E>& ctx,
                         sc_core::sc_module_name name,
                         std::size_t fft_size,
                         std::size_t stage_idx,
                         FFTFlowMode flow_mode,
                         SyscMemory<E>* twiddle_mem) {
  return std::unique_ptr<R2SdfStageTLM<E>>(
      new R2SdfStageTLM<E>(ctx, name, fft_size, stage_idx, flow_mode, twiddle_mem));
}

template <typename E>
R2SdfStageTLM<E>::R2SdfStageTLM(Context<E>&,
                                sc_core::sc_module_name name,
                                std::size_t fft_size,
                                std::size_t stage_idx,
                                FFTFlowMode flow_mode,
                                SyscMemory<E>* twiddle_mem)
    : sc_core::sc_module(name),
      fft_size(fft_size),
      stage_idx_(stage_idx),
      flow_mode(flow_mode),
      twiddle_mem(twiddle_mem) {}

template <typename E>
void R2SdfStageTLM<E>::allocate_state(Context<E>&) {
  shiftreg = std::make_unique<ComplexShiftRegisterTLM<T>>(
      sc_core::sc_gen_unique_name("shiftreg"),
      get_delay_len());

  cmul = std::make_unique<ComplexMultiplierTLM<T>>(
      sc_core::sc_gen_unique_name("cmul"));

  reset_state();
  is_init_ = true;
}

template <typename E>
void R2SdfStageTLM<E>::reset_state() {
  if (shiftreg) {
    shiftreg->clear();
  }
}

template <typename E>
void R2SdfStageTLM<E>::process_block(const std::vector<CxT>& in,
                                     std::vector<CxT>& out,
                                     bool inverse) {
  if (!is_init_) {
    throw std::runtime_error("R2SdfStageTLM not initialized");
  }
  if (in.size() != fft_size) {
    throw std::invalid_argument("R2SdfStageTLM input size mismatch");
  }

  out.assign(fft_size, CxT(0, 0));

  const std::size_t span = get_span();
  const std::size_t half = span >> 1;

  for (std::size_t base = 0; base < fft_size; base += span) {
    shiftreg->clear();

    for (std::size_t i = 0; i < half; ++i) {
      (void)shiftreg->step(to_plain(in[base + i]));
    }

    for (std::size_t i = 0; i < half; ++i) {
      const ComplexPlain<T> delayed_plain =
          shiftreg->step(to_plain(in[base + half + i]));
      const CxT a(delayed_plain.re, delayed_plain.im);
      const CxT b = in[base + half + i];

      const CxT sum  = a + b;
      const CxT diff = a - b;

      const std::size_t tw_idx =
          (flow_mode == FFTFlowMode::DIT)
              ? get_twiddle_index_dit(i)
              : get_twiddle_index_dif(i);

      const CxT w = get_twiddle(tw_idx, inverse);
      const ComplexPlain<T> prod = cmul->mul(to_plain(diff), to_plain(w));
      const CxT diff_tw(prod.re, prod.im);

      out[base + i]        = sum;
      out[base + half + i] = diff_tw;
    }
  }
}

// -----------------------------------------------------------------------------
// R2SdfFFTTLM
// -----------------------------------------------------------------------------
template <typename E>
std::unique_ptr<R2SdfFFTTLM<E>>
R2SdfFFTTLM<E>::create(Context<E>& ctx,
                       sc_core::sc_module_name name,
                       std::size_t fft_size,
                       FFTFlowMode flow_mode) {
  return std::unique_ptr<R2SdfFFTTLM<E>>(
      new R2SdfFFTTLM<E>(ctx, name, fft_size, flow_mode));
}

template <typename E>
R2SdfFFTTLM<E>::R2SdfFFTTLM(Context<E>&,
                            sc_core::sc_module_name name,
                            std::size_t fft_size,
                            FFTFlowMode flow_mode)
    : sc_core::sc_module(name),
      fft_size(fft_size),
      flow_mode(flow_mode) {
  targ_socket.register_b_transport(this, &R2SdfFFTTLM<E>::b_transport);
  targ_socket.register_get_direct_mem_ptr(this, &R2SdfFFTTLM<E>::get_direct_mem_ptr);
  targ_socket.register_transport_dbg(this, &R2SdfFFTTLM<E>::transport_dbg);
}

template <typename E>
void R2SdfFFTTLM<E>::fftreal(const VecR& in, VecC& out) const {
  VecC in_c(in.size(), CxT(0, 0));
  for (std::size_t i = 0; i < in.size(); ++i) {
    in_c[i] = CxT(in[i], 0);
  }
  fftcplx(in_c, out);
}

template <typename E>
void R2SdfFFTTLM<E>::fftcplx(const VecC& in, VecC& out) const {
  process_frame(in, out, false);
}

template <typename E>
void R2SdfFFTTLM<E>::ifftcplx(const VecC& in, VecC& out) const {
  process_frame(in, out, true);
  const T scale = T(1) / static_cast<T>(fft_size);
  for (auto& v : out) {
    v *= scale;
  }
}

template <typename E>
void R2SdfFFTTLM<E>::allocate_state(Context<E>& ctx) {
  validate_fft_size();
  allocate_twiddle(ctx);

  r2sdfstgs.clear();
  r2sdfstgs.reserve(get_nstages());

  for (std::size_t i = 0; i < get_nstages(); ++i) {
    auto stg = R2SdfStageTLM<E>::create(
        ctx,
        sc_core::sc_gen_unique_name("r2sdf_stage"),
        fft_size,
        i,
        flow_mode,
        twiddle_mem.get());

    stg->allocate_state(ctx);
    r2sdfstgs.push_back(std::move(stg));
  }

  last_fftin.assign(fft_size, CxT(0, 0));
  last_fftout.assign(fft_size, CxT(0, 0));
  is_init_ = true;
}

template <typename E>
void R2SdfFFTTLM<E>::state_reset() {
  for (auto& stg : r2sdfstgs) {
    stg->reset_state();
  }
  std::fill(last_fftin.begin(), last_fftin.end(), CxT(0, 0));
  std::fill(last_fftout.begin(), last_fftout.end(), CxT(0, 0));
}

template <typename E>
void R2SdfFFTTLM<E>::allocate_twiddle(Context<E>& ctx) {
  const std::size_t mem_size = 2 * fft_size;
  std::vector<T> init(mem_size, T(0));

  const T pi = static_cast<T>(3.14159265358979323846);
  for (std::size_t k = 0; k < fft_size; ++k) {
    const T ang = static_cast<T>(-2) * pi * static_cast<T>(k) / static_cast<T>(fft_size);
    init[2 * k]     = std::cos(ang);
    init[2 * k + 1] = std::sin(ang);
  }

  twiddle_mem = SyscMemory<E>::create(
      ctx,
      sc_core::sc_gen_unique_name("twiddle_rom"),
      mem_size,
      init.data());
}

template <typename E>
void R2SdfFFTTLM<E>::bit_reverse(VecC& data) const {
  std::size_t j = 0;
  for (std::size_t i = 1; i < data.size(); ++i) {
    std::size_t bit = data.size() >> 1;
    while (j & bit) {
      j ^= bit;
      bit >>= 1;
    }
    j |= bit;
    if (i < j) {
      std::swap(data[i], data[j]);
    }
  }
}

template <typename E>
void R2SdfFFTTLM<E>::process_frame(const VecC& in, VecC& out, bool inverse) const {
  if (!is_init_) {
    throw std::runtime_error("R2SdfFFTTLM not initialized");
  }
  if (in.size() != fft_size) {
    throw std::invalid_argument("R2SdfFFTTLM frame size mismatch");
  }

  auto* self = const_cast<R2SdfFFTTLM<E>*>(this);
  self->last_fftin = in;

  VecC cur = in;
  if (flow_mode == FFTFlowMode::DIT) {
    bit_reverse(cur);
  }

  for (auto& stg : self->r2sdfstgs) {
    VecC nxt;
    stg->process_block(cur, nxt, inverse);

    if (regsscale_each_stage) {
      const T s = T(0.5);
      for (auto& z : nxt) {
        z *= s;
      }
    }

    cur.swap(nxt);
  }

  if (flow_mode == FFTFlowMode::DIF) {
    bit_reverse(cur);
  }

  out = cur;
  self->last_fftout = out;
}

}