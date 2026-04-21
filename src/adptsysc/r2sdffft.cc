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


template <typename E>
std::unique_ptr<R2SdfStageTlm<E>>
R2SdfStageTlm<E>::create(Context<E>& ctx,
                         sc_core::sc_module_name name,
                         std::size_t fft_size,
                         std::size_t stage_idx,
                         FFTFlowMode flow_mode,
                         SyscMemory<E>* twiddle_mem) {
  return std::unique_ptr<R2SdfStageTlm<E>>(
    new R2SdfStageTlm<E>(ctx, name, fft_size, stage_idx, flow_mode, twiddle_mem)
  );
}

template <typename E>
R2SdfStageTlm<E>::R2SdfStageTlm(Context<E>&,
                                sc_core::sc_module_name name,
                                std::size_t fft_size,
                                std::size_t stage_idx,
                                FFTFlowMode flow_mode,
                                SyscMemory<E>* twiddle_mem)
  : sc_core::sc_module(name),
    fft_size_(fft_size),
    stage_idx_(stage_idx),
    flow_mode_(flow_mode),
    twiddle_mem_(twiddle_mem) {}

template <typename E>
void R2SdfStageTlm<E>::allocate_state(Context<E>&) {
  const std::size_t delay_len =
    (flow_mode_ == FFTFlowMode::DIT)
      ? (std::size_t(1) << stage_idx_)
      : (std::size_t(1) << (n_stages() - stage_idx_ - 1));

  delay_mem_.assign(delay_len, CxT(0, 0));
  wr_ptr_ = 0;
  sample_ctr_ = 0;
  is_init_ = true;
}

template <typename E>
void R2SdfStageTlm<E>::reset_state() {
  std::fill(delay_mem_.begin(), delay_mem_.end(), CxT(0, 0));
  wr_ptr_ = 0;
  sample_ctr_ = 0;
}

template <typename E>
void R2SdfStageTlm<E>::process_sample(const CxT& xin, bool vin, CxT& yout, bool& vout, bool inverse) {
  if (!is_init_) {
    throw std::runtime_error("R2SdfStageTlm not initialized");
  }
  if (!vin) {
    yout = CxT(0, 0);
    vout = false;
    return;
  }

  const std::size_t delay_len = delay_mem_.size();
  const std::size_t period = delay_len << 1;
  const std::size_t pos = sample_ctr_ % period;

  if (pos < delay_len) {
    delay_mem_[wr_ptr_] = xin;
    wr_ptr_ = (wr_ptr_ + 1) % delay_len;
    yout = xin;
    vout = false;
  } else {
    const std::size_t rd_ptr = wr_ptr_;
    const CxT a = delay_mem_[rd_ptr];
    const CxT b = xin;
    const CxT sum = a + b;
    CxT diff = a - b;

    const std::size_t local_idx = pos - delay_len;
    const std::size_t tw_idx =
      (flow_mode_ == FFTFlowMode::DIT)
        ? twiddle_index_dit(local_idx)
        : twiddle_index_dif(local_idx);

    diff *= get_twiddle(tw_idx, inverse);

    if (flow_mode_ == FFTFlowMode::DIT) {
      delay_mem_[rd_ptr] = diff;
      yout = sum;
    } else {
      delay_mem_[rd_ptr] = sum;
      yout = diff;
    }

    wr_ptr_ = (wr_ptr_ + 1) % delay_len;
    vout = true;
  }

  ++sample_ctr_;
}

template <typename E>
std::size_t R2SdfStageTlm<E>::n_stages() const {
  std::size_t n = fft_size_;
  std::size_t s = 0;
  while (n > 1) { n >>= 1; ++s; }
  return s;
}

template <typename E>
std::size_t R2SdfStageTlm<E>::twiddle_index_dit(std::size_t local_idx) const {
  const std::size_t group = std::size_t(1) << (stage_idx_ + 1);
  const std::size_t half = group >> 1;
  const std::size_t stride = fft_size_ / group;
  return ((local_idx % half) * stride) & (fft_size_ - 1);
}

template <typename E>
std::size_t R2SdfStageTlm<E>::twiddle_index_dif(std::size_t local_idx) const {
  const std::size_t group = std::size_t(1) << (n_stages() - stage_idx_);
  const std::size_t half = group >> 1;
  const std::size_t stride = fft_size_ / group;
  return ((local_idx % half) * stride) & (fft_size_ - 1);
}

template <typename E>
typename R2SdfStageTlm<E>::CxT
R2SdfStageTlm<E>::get_twiddle(std::size_t k, bool inverse) const {
  if (twiddle_mem_ == nullptr) {
    throw std::runtime_error("R2SdfStageTlm twiddle memory is null");
  }
  const std::size_t addr = 2 * k;
  const T re = twiddle_mem_->data()[addr];
  const T im = twiddle_mem_->data()[addr + 1];
  CxT w(re, im);
  return inverse ? std::conj(w) : w;
}

// ---------------- top ----------------

template <typename E>
std::unique_ptr<R2SdfFFTTlm<E>>
R2SdfFFTTlm<E>::create(Context<E>& ctx,
                       sc_core::sc_module_name name,
                       std::size_t fft_size,
                       FFTFlowMode flow_mode) {
  return std::unique_ptr<R2SdfFFTTlm<E>>(
    new R2SdfFFTTlm<E>(ctx, name, fft_size, flow_mode)
  );
}

template <typename E>
bool R2SdfFFTTlm<E>::run_testbench(Context<E>&) {
  return true;
}

template <typename E>
R2SdfFFTTlm<E>::R2SdfFFTTlm(Context<E>&,
                            sc_core::sc_module_name name,
                            std::size_t fft_size,
                            FFTFlowMode flow_mode)
  : sc_core::sc_module(name),
    targ_socket("targ_socket"),
    fft_size_(fft_size),
    flow_mode_(flow_mode) {
  targ_socket.register_b_transport(this, &R2SdfFFTTlm<E>::b_transport);
  targ_socket.register_get_direct_mem_ptr(this, &R2SdfFFTTlm<E>::get_direct_mem_ptr);
  targ_socket.register_transport_dbg(this, &R2SdfFFTTlm<E>::transport_dbg);
}

template <typename E>
std::size_t R2SdfFFTTlm<E>::get_fftsize() const {
  return fft_size_;
}

template <typename E>
void R2SdfFFTTlm<E>::fftreal(const VecR& in, VecC& out) const {
  VecC in_c(in.size(), CxT(0, 0));
  for (std::size_t i = 0; i < in.size(); ++i) {
    in_c[i] = CxT(in[i], 0);
  }
  fftcplx(in_c, out);
}

template <typename E>
void R2SdfFFTTlm<E>::fftcplx(const VecC& in, VecC& out) const {
  process_frame(in, out, false);
}

template <typename E>
void R2SdfFFTTlm<E>::ifftcplx(const VecC& in, VecC& out) const {
  process_frame(in, out, true);
  const T scale = T(1) / static_cast<T>(fft_size_);
  for (auto& v : out) {
    v *= scale;
  }
}

template <typename E>
void R2SdfFFTTlm<E>::allocate_state(Context<E>& ctx) {
  validate_fft_size();
  allocate_twiddle(ctx);

  stages_.clear();
  stages_.reserve(n_stages());
  for (std::size_t i = 0; i < n_stages(); ++i) {
    auto stg = R2SdfStageTlm<E>::create(
      ctx,
      sc_core::sc_gen_unique_name("r2sdf_stage"),
      fft_size_,
      i,
      flow_mode_,
      twiddle_mem_.get()
    );
    stg->allocate_state(ctx);
    stages_.push_back(std::move(stg));
  }

  last_fft_in_.assign(fft_size_, CxT(0, 0));
  last_fft_out_.assign(fft_size_, CxT(0, 0));
  is_init_ = true;
}

template <typename E>
void R2SdfFFTTlm<E>::state_reset() {
  for (auto& stg : stages_) {
    stg->reset_state();
  }
  std::fill(last_fft_in_.begin(), last_fft_in_.end(), CxT(0, 0));
  std::fill(last_fft_out_.begin(), last_fft_out_.end(), CxT(0, 0));
}

template <typename E>
void R2SdfFFTTlm<E>::update_hyperparams(const json& params) {
  if (params.contains("scale_each_stage")) {
    scale_each_stage_ = params.at("scale_each_stage").template get<bool>();
  }
}

template <typename E>
json R2SdfFFTTlm<E>::get_hyperparams() const {
  return {
    {"otype", "r2sdf_fft_tlm"},
    {"fft_size", fft_size_},
    {"flow_mode", flow_mode_ == FFTFlowMode::DIT ? "dit" : "dif"},
    {"scale_each_stage", scale_each_stage_}
  };
}

template <typename E>
json R2SdfFFTTlm<E>::serialize(Context<E>&) const {
  return {
    {"fft_size", fft_size_},
    {"flow_mode", flow_mode_ == FFTFlowMode::DIT ? "dit" : "dif"},
    {"scale_each_stage", scale_each_stage_}
  };
}

template <typename E>
void R2SdfFFTTlm<E>::deserialize(Context<E>&, const json& data) {
  update_hyperparams(data);
}

template <typename E>
void R2SdfFFTTlm<E>::dump_state(Context<E>&, const std::string&) const {}

template <typename E>
void R2SdfFFTTlm<E>::b_transport(tlm::tlm_generic_payload&, sc_core::sc_time& delay) {
  delay += butterfly_delay_ + twiddle_delay_ + memory_delay_;
}

template <typename E>
bool R2SdfFFTTlm<E>::get_direct_mem_ptr(tlm::tlm_generic_payload&, tlm::tlm_dmi&) {
  return false;
}

template <typename E>
unsigned int R2SdfFFTTlm<E>::transport_dbg(tlm::tlm_generic_payload&) {
  return 0;
}

template <typename E>
void R2SdfFFTTlm<E>::validate_fft_size() const {
  if (fft_size_ == 0 || ((fft_size_ & (fft_size_ - 1)) != 0)) {
    throw std::invalid_argument("R2SdfFFTTlm: fft_size must be power-of-2");
  }
}

template <typename E>
std::size_t R2SdfFFTTlm<E>::n_stages() const {
  std::size_t n = fft_size_;
  std::size_t s = 0;
  while (n > 1) { n >>= 1; ++s; }
  return s;
}

template <typename E>
void R2SdfFFTTlm<E>::allocate_twiddle(Context<E>& ctx) {
  const std::size_t mem_size = 2 * fft_size_;
  std::vector<T> init(mem_size, T(0));

  const T pi = static_cast<T>(3.14159265358979323846);
  for (std::size_t k = 0; k < fft_size_; ++k) {
    const T ang = static_cast<T>(-2) * pi * static_cast<T>(k) / static_cast<T>(fft_size_);
    init[2 * k]     = std::cos(ang);
    init[2 * k + 1] = std::sin(ang);
  }

  twiddle_mem_ = SyscMemory<E>::create(
    ctx,
    sc_core::sc_gen_unique_name("twiddle_rom"),
    mem_size,
    init.data()
  );
}

template <typename E>
void R2SdfFFTTlm<E>::bit_reverse(VecC& data) const {
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
void R2SdfFFTTlm<E>::process_frame(const VecC& in, VecC& out, bool inverse) const {
  if (!is_init_) {
    throw std::runtime_error("R2SdfFFTTlm not initialized");
  }
  if (in.size() != fft_size_) {
    throw std::invalid_argument("R2SdfFFTTlm frame size mismatch");
  }

  auto* self = const_cast<R2SdfFFTTlm<E>*>(this);
  self->state_reset();
  self->last_fft_in_ = in;

  VecC work = in;
  if (flow_mode_ == FFTFlowMode::DIT) {
    bit_reverse(work);
  }
  if (inverse) {
    for (auto& v : work) {
      v = std::conj(v);
    }
  }

  VecC cur = work;
  for (std::size_t s = 0; s < stages_.size(); ++s) {
    VecC nxt(fft_size_, CxT(0, 0));
    std::vector<bool> valid(fft_size_, false);

    for (std::size_t i = 0; i < fft_size_; ++i) {
      CxT y;
      bool v;
      self->stages_[s]->process_sample(cur[i], true, y, v, inverse);
      nxt[i] = y;
      valid[i] = v;
    }

    // flush stage to collect delayed valid samples
    std::size_t out_idx = 0;
    VecC collected(fft_size_, CxT(0, 0));

    for (std::size_t i = 0; i < fft_size_; ++i) {
      if (valid[i]) {
        collected[out_idx++] = nxt[i];
      }
    }

    while (out_idx < fft_size_) {
      CxT y;
      bool v;
      self->stages_[s]->process_sample(CxT(0, 0), false, y, v, inverse);
      if (v) {
        collected[out_idx++] = y;
      } else {
        break;
      }
    }

    cur.swap(collected);

    if (scale_each_stage_) {
      const T s2 = T(0.5);
      for (auto& z : cur) {
        z *= s2;
      }
    }
  }

  if (flow_mode_ == FFTFlowMode::DIF) {
    bit_reverse(cur);
  }
  if (inverse) {
    for (auto& v : cur) {
      v = std::conj(v);
    }
  }

  out = cur;
  self->last_fft_out_ = out;
}

// explicit instantiation
template class R2SdfStageTlm<R2SdfFFTTlmArch>;
template class R2SdfFFTTlm<R2SdfFFTTlmArch>;


}