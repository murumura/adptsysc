#include <adptsysc/config.hh>
#include <stdint.h>
#include <tlm>
#include <tlm_utils/simple_target_socket.h>
#include <tlm_utils/simple_initiator_socket.h>
#include <adptsysc/adptsysc.hh>
#include <memory>
#include <adptsysc/sysc-r2sdffft.hh>
#include <algorithm>
#include <cmath>
#include <sstream>
namespace adptsysc {

using E = ADPT_TARGET;

// =============================
// R2SdfCtrlTLM
// =============================
template <typename E>
std::shared_ptr<R2SdfCtrlTLM<E>>
R2SdfCtrlTLM<E>::create(Context<E>& ctx,
                        sc_core::sc_module_name name,
                        FFTFlowMode flow_mode) {
  (void)ctx;
  return std::shared_ptr<R2SdfCtrlTLM<E>>(
      new R2SdfCtrlTLM<E>(name, flow_mode));
}

template <typename E>
R2SdfCtrlTLM<E>::R2SdfCtrlTLM(sc_core::sc_module_name name, FFTFlowMode fm)
    : sc_core::sc_module(name), flow_mode(fm) {
  reset();
}

template <typename E>
void R2SdfCtrlTLM<E>::reset() {
  cnt_cur = CountT{0};
  sigs_cur = compute_outputs(cnt_cur, false);
}

template <typename E>
int R2SdfCtrlTLM<E>::stage_to_tw_slot(unsigned p) const {
  if (flow_mode == FFTFlowMode::DIF) {
    if (p >= kNStages - 1) return -1;
    return static_cast<int>(p);
  } else {
    if (p == 0) return -1;
    return static_cast<int>(p - 1);
  }
}

template <typename E>
const typename R2SdfCtrlTLM<E>::R2SdfCtrlSigs&
R2SdfCtrlTLM<E>::step(bool sample_fire) {
  sigs_cur = compute_outputs(cnt_cur, sample_fire);
  if (sample_fire) {
    CountT cnt_next = cnt_cur;
    cnt_next = cnt_next + CountT{1};
    cnt_cur = cnt_next;
  }
  return sigs_cur;
}

template <typename E>
typename R2SdfCtrlTLM<E>::R2SdfCtrlSigs
R2SdfCtrlTLM<E>::decode_from_count(CountT c) const {
  return compute_outputs(c, true);
}

template <typename E>
typename R2SdfCtrlTLM<E>::R2SdfCtrlSigs
R2SdfCtrlTLM<E>::compute_outputs(CountT c, bool sample_fire) const {
  R2SdfCtrlSigs o{};
  o.cnt = c;

  if (!sample_fire) {
    return o;
  }

  for (unsigned p = 0; p < kNStages; ++p) {
    const unsigned bit_idx =
        (flow_mode == FFTFlowMode::DIF)
            ? (kNStages - 1 - p)
            : p;
    o.s[p] = static_cast<bool>(c[bit_idx]);
  }

  for (unsigned q = 0; q < kNTw; ++q) {
    o.tw_rom_en[q] = false;
    o.tw_addr_local[q] = TwAddrT{0};
    o.tw_addr_global[q] = TwAddrT{0};
  }

  for (unsigned p = 0; p < kNStages; ++p) {
    const int q = stage_to_tw_slot(p);
    if (q < 0) {
      continue;
    }

    o.tw_rom_en[static_cast<std::size_t>(q)] = o.s[p];

    unsigned valid_w = 0;
    unsigned shift   = 0;

    if (flow_mode == FFTFlowMode::DIF) {
      valid_w = kNStages - 1 - p;
      shift   = p;
    } else {
      valid_w = p;
      shift   = kNStages - 1 - p;
    }

    TwAddrT local_addr = 0;
    for (unsigned b = 0; b < valid_w; ++b) {
      local_addr[b] = static_cast<bool>(c[b]);
    }

    o.tw_addr_local[static_cast<std::size_t>(q)] = local_addr;
    o.tw_addr_global[static_cast<std::size_t>(q)] =
        static_cast<TwAddrT>(local_addr << shift);
  }

  o.frame_first = sample_fire && (c == CountT{0});
  o.frame_last  = sample_fire && (c == CountT{E::fft_size - 1});
  return o;
}

template <typename E>
void R2SdfCtrlTLM<E>::update_hyperparams(const json& params) {
  (void)params;
}

template <typename E>
json R2SdfCtrlTLM<E>::get_hyperparams() const {
  return {
    {"otype", "r2sdf_ctrl_tlm"},
    {"fft_size", E::fft_size},
    {"flow_mode", flow_mode == FFTFlowMode::DIT ? "dit" : "dif"},
    {"nstages", kNStages},
    {"ntwiddle_stages", kNTw}
  };
}

// -----------------------------------------------------------------------------
// R2SdfStageTLM
// -----------------------------------------------------------------------------
template <typename E>
std::unique_ptr<R2SdfStageTLM<E>>
R2SdfStageTLM<E>::create(Context<E>& ctx,
                         sc_core::sc_module_name name,
                         std::size_t fft_size_,
                         std::size_t stage_idx_,
                         FFTFlowMode flow_mode_,
                         SyscMemory<E>* twiddle_mem_,
                         std::shared_ptr<R2SdfCtrlTLM<E>> ctrl_) {
  return std::unique_ptr<R2SdfStageTLM<E>>(
      new R2SdfStageTLM<E>(ctx, name, fft_size_, stage_idx_, flow_mode_,
                           twiddle_mem_, std::move(ctrl_)));
}

template <typename E>
R2SdfStageTLM<E>::R2SdfStageTLM(Context<E>&,
                                sc_core::sc_module_name name,
                                std::size_t fft_size_,
                                std::size_t stage_idx_,
                                FFTFlowMode flow_mode_,
                                SyscMemory<E>* twiddle_mem_,
                                std::shared_ptr<R2SdfCtrlTLM<E>> ctrl_)
    : sc_core::sc_module(name),
      fft_size(fft_size_),
      stage_idx(stage_idx_),
      flow_mode(flow_mode_),
      twiddle_mem(twiddle_mem_),
      ctrl(std::move(ctrl_)) {}

template <typename E>
void R2SdfStageTLM<E>::set_ctrl(std::shared_ptr<R2SdfCtrlTLM<E>> c) {
  ctrl = std::move(c);
}

template <typename E>
void R2SdfStageTLM<E>::allocate_state(Context<E>&) {
  shiftreg = std::make_unique<ComplexShiftRegisterTLM<T>>(
      sc_core::sc_gen_unique_name("shiftreg"), get_delay_len());

  cmul = std::make_unique<ComplexMultiplierTLM<T>>(
      sc_core::sc_gen_unique_name("cmul"));

  reset_state();
  is_init = true;
}

template <typename E>
void R2SdfStageTLM<E>::reset_state() {
  if (shiftreg) {
    shiftreg->clear();
  }
}

template <typename E>
typename R2SdfStageTLM<E>::template ComplexPlain<T>
R2SdfStageTLM<E>::to_plain(const CxT& z) {
  return ComplexPlain<T>{z.real(), z.imag()};
}

template <typename E>
std::size_t R2SdfStageTLM<E>::get_nstages() const {
  std::size_t n = fft_size;
  std::size_t s = 0;
  while (n > 1) {
    n >>= 1;
    ++s;
  }
  return s;
}

template <typename E>
std::size_t R2SdfStageTLM<E>::get_span() const {
  if (flow_mode == FFTFlowMode::DIT) {
    return std::size_t(1) << (stage_idx + 1);
  }
  return std::size_t(1) << (get_nstages() - stage_idx);
}

template <typename E>
std::size_t R2SdfStageTLM<E>::get_delay_len() const {
  return get_span() >> 1;
}

template <typename E>
std::size_t R2SdfStageTLM<E>::get_twiddle_index_dit(std::size_t local_idx) const {
  const std::size_t group  = std::size_t(1) << (stage_idx + 1);
  const std::size_t stride = fft_size / group;
  return (local_idx * stride) & (fft_size - 1);
}

template <typename E>
std::size_t R2SdfStageTLM<E>::get_twiddle_index_dif(std::size_t local_idx) const {
  const std::size_t group  = std::size_t(1) << (get_nstages() - stage_idx);
  const std::size_t stride = fft_size / group;
  return (local_idx * stride) & (fft_size - 1);
}

template <typename E>
typename R2SdfStageTLM<E>::CxT
R2SdfStageTLM<E>::get_twiddle(std::size_t k, bool inverse) const {
  if (twiddle_mem == nullptr) {
    throw std::runtime_error("R2SdfStageTLM twiddle memory is null");
  }
  const std::size_t addr = 2 * k;
  const T re = twiddle_mem->data()[addr];
  const T im = twiddle_mem->data()[addr + 1];
  const CxT w(re, im);
  return inverse ? std::conj(w) : w;
}

template <typename E>
int R2SdfStageTLM<E>::get_ctrl_tw_slot() const {
  const std::size_t nstages = get_nstages();

  if (flow_mode == FFTFlowMode::DIT) {
    return (stage_idx == 0) ? -1 : static_cast<int>(stage_idx - 1);
  } else {
    return (stage_idx + 1 == nstages) ? -1 : static_cast<int>(stage_idx);
  }
}

template <typename E>
void R2SdfStageTLM<E>::process_block(const std::vector<CxT>& in,
                                     std::vector<CxT>& out,
                                     bool inverse) {
  if (!is_init) {
    throw std::runtime_error("R2SdfStageTLM not initialized");
  }
  if (in.size() != fft_size) {
    throw std::invalid_argument("R2SdfStageTLM input size mismatch");
  }

  out.assign(fft_size, CxT(0, 0));

  const std::size_t span = get_span();
  const std::size_t half = span >> 1;

  const bool ctrl_mode = use_ctrl && static_cast<bool>(ctrl);

  using CtrlT = R2SdfCtrlTLM<E>;
  const int tw_slot = get_ctrl_tw_slot();
  const bool stage_has_nontrivial_tw = (tw_slot >= 0);

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

      bool use_tw = false;
      std::size_t tw_idx = 0;

      if (ctrl_mode && stage_has_nontrivial_tw) {
        const typename CtrlT::CountT sample_idx =
            static_cast<typename CtrlT::CountT>(base + half + i);

        const auto co = ctrl->decode_from_count(sample_idx);

        use_tw = co.tw_rom_en[static_cast<std::size_t>(tw_slot)];
        tw_idx = static_cast<std::size_t>(
            co.tw_addr_global[static_cast<std::size_t>(tw_slot)]);
      } else if (!ctrl_mode) {
        if (flow_mode == FFTFlowMode::DIT) {
          if (stage_idx > 0) {
            use_tw = true;
            tw_idx = get_twiddle_index_dit(i);
          }
        } else {
          if (stage_idx + 1 < get_nstages()) {
            use_tw = true;
            tw_idx = get_twiddle_index_dif(i);
          }
        }
      }

      if (flow_mode == FFTFlowMode::DIT) {
        CxT t = b;
        if (use_tw) {
          const CxT w = get_twiddle(tw_idx, inverse);
          const ComplexPlain<T> tb_plain = cmul->mul(to_plain(b), to_plain(w));
          t = CxT(tb_plain.re, tb_plain.im);
        }

        out[base + i]        = a + t;
        out[base + half + i] = a - t;

      } else {
        const CxT sum  = a + b;
        const CxT diff = a - b;

        CxT diff_tw = diff;
        if (use_tw) {
          const CxT w = get_twiddle(tw_idx, inverse);
          const ComplexPlain<T> prod = cmul->mul(to_plain(diff), to_plain(w));
          diff_tw = CxT(prod.re, prod.im);
        }

        out[base + i]        = sum;
        out[base + half + i] = diff_tw;
      }
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
                       std::size_t fft_size_,
                       FFTFlowMode flow_mode_) {
  return std::unique_ptr<R2SdfFFTTLM<E>>(
      new R2SdfFFTTLM<E>(ctx, name, fft_size_, flow_mode_));
}

template <typename E>
bool R2SdfFFTTLM<E>::run_testbench(Context<E>&) {
  return true;
}

template <typename E>
R2SdfFFTTLM<E>::R2SdfFFTTLM(Context<E>&,
                            sc_core::sc_module_name name,
                            std::size_t fft_size_,
                            FFTFlowMode flow_mode_)
    : sc_core::sc_module(name),
      fft_size(fft_size_),
      flow_mode(flow_mode_) {
  targ_socket.register_b_transport(this, &R2SdfFFTTLM<E>::b_transport);
  targ_socket.register_get_direct_mem_ptr(this, &R2SdfFFTTLM<E>::get_direct_mem_ptr);
  targ_socket.register_transport_dbg(this, &R2SdfFFTTLM<E>::transport_dbg);
}

template <typename E>
std::size_t R2SdfFFTTLM<E>::get_fftsize() const {
  return fft_size;
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

  if (!scale_each_stage) {
    const T scale = T(1) / static_cast<T>(fft_size);
    for (auto& v : out) {
      v *= scale;
    }
  }
}

template <typename E>
void R2SdfFFTTLM<E>::allocate_state(Context<E>& ctx) {
  validate_fft_size();
  allocate_twiddle(ctx);

  if (use_ctrl && !ctrl) {
    ctrl = R2SdfCtrlTLM<E>::create(
        ctx, sc_core::sc_gen_unique_name("r2sdf_ctrl"), flow_mode);
  }

  r2sdfstgs.clear();
  r2sdfstgs.reserve(get_nstages());

  for (std::size_t i = 0; i < get_nstages(); ++i) {
    auto stg = R2SdfStageTLM<E>::create(
        ctx,
        sc_core::sc_gen_unique_name("r2sdf_stage"),
        fft_size,
        i,
        flow_mode,
        twiddle_mem.get(),
        ctrl);

    stg->allocate_state(ctx);
    r2sdfstgs.push_back(std::move(stg));
  }

  last_fftin.assign(fft_size, CxT(0, 0));
  last_fftout.assign(fft_size, CxT(0, 0));
  is_init = true;
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
void R2SdfFFTTLM<E>::update_hyperparams(const json& params) {
  if (params.contains("scale_each_stage")) {
    scale_each_stage = params.at("scale_each_stage").template get<bool>();
  }
  if (params.contains("use_ctrl")) {
    use_ctrl = params.at("use_ctrl").template get<bool>();
  }
}

template <typename E>
json R2SdfFFTTLM<E>::get_hyperparams() const {
  return {
      {"otype", "r2sdf_fft_tlm"},
      {"fft_size", fft_size},
      {"flow_mode", flow_mode == FFTFlowMode::DIT ? "dit" : "dif"},
      {"scale_each_stage", scale_each_stage},
      {"use_ctrl", use_ctrl}
  };
}

template <typename E>
json R2SdfFFTTLM<E>::serialize(Context<E>&) const {
  return {
      {"fft_size", fft_size},
      {"flow_mode", flow_mode == FFTFlowMode::DIT ? "dit" : "dif"},
      {"use_ctrl", use_ctrl},
      {"scale_each_stage", scale_each_stage}
  };
}

template <typename E>
void R2SdfFFTTLM<E>::deserialize(Context<E>&, const json& data) {
  update_hyperparams(data);
}

template <typename E>
void R2SdfFFTTLM<E>::b_transport(tlm::tlm_generic_payload& trans,
                                 sc_core::sc_time& delay) {
  (void)trans;
  delay += butterfly_delay + twiddle_delay + memory_delay + cmplxmul_delay;
}

template <typename E>
bool R2SdfFFTTLM<E>::get_direct_mem_ptr(tlm::tlm_generic_payload&, tlm::tlm_dmi&) {
  return false;
}

template <typename E>
unsigned int R2SdfFFTTLM<E>::transport_dbg(tlm::tlm_generic_payload&) {
  return 0;
}

template <typename E>
void R2SdfFFTTLM<E>::validate_fft_size() const {
  if (fft_size == 0 || ((fft_size & (fft_size - 1)) != 0)) {
    throw std::invalid_argument("R2SdfFFTTLM: fft_size must be power-of-2");
  }
}

template <typename E>
std::size_t R2SdfFFTTLM<E>::get_nstages() const {
  std::size_t n = fft_size;
  std::size_t s = 0;
  while (n > 1) {
    n >>= 1;
    ++s;
  }
  return s;
}

template <typename E>
void R2SdfFFTTLM<E>::allocate_twiddle(Context<E>& ctx) {
  const std::size_t mem_size = 2 * fft_size;
  std::vector<T> init(mem_size, T(0));

  const T pi = static_cast<T>(3.14159265358979323846);
  for (std::size_t k = 0; k < fft_size; ++k) {
    const T ang =
        static_cast<T>(-2) * pi * static_cast<T>(k) / static_cast<T>(fft_size);
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
  if (!is_init) {
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

    if (scale_each_stage) {
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

template class R2SdfFFTTLM<E>;
template class R2SdfCtrlTLM<E>;
template class R2SdfStageTLM<E>;

}