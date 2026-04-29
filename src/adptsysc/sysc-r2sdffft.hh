#pragma once
#include <tlm>
#include <tlm_utils/simple_target_socket.h>
#include <deque>
#include <memory>
#include <array> 
#include <vector> 
#include <complex> 
#include <stdexcept>
#include <adptsysc/object.hh>
#include <adptsysc/design-lib.hh>
#include <adptsysc/sysc-mem.hh>
#include <adptsysc/sysc-cmplxmul.hh>
#include <adptsysc/sysc-shiftreg.hh>

namespace adptsysc {
  
template <typename E> struct Context;

template <std::size_t N>
constexpr bool is_pow2_constexpr_v = (N > 0) && ((N & (N - 1)) == 0);

template <typename E>
class R2SdfCtrlTLM : public ObjectWithMutableHyperparams,
                     public sc_core::sc_module {
public:
  static constexpr unsigned kCntW    = clog2_constexpr(E::fft_size);
  static constexpr unsigned kNStages = kCntW;
  static constexpr unsigned kNTw     = kNStages - 1;

  using CountT  = sc_dt::sc_uint<kCntW>;
  using TwAddrT = sc_dt::sc_uint<kCntW>;

  struct R2SdfCtrlSigs {
    CountT cnt{0};

    std::array<bool, kNStages> s{};
    std::array<bool, kNTw> tw_rom_en{};
    std::array<TwAddrT, kNTw> tw_addr_local{};
    std::array<TwAddrT, kNTw> tw_addr_global{};

    bool frame_first{false};
    bool frame_last{false};
  };

  static std::shared_ptr<R2SdfCtrlTLM<E>>
  create(Context<E>& ctx, sc_core::sc_module_name name, FFTFlowMode flow_mode) {
    (void)ctx;
    return std::shared_ptr<R2SdfCtrlTLM<E>>(new R2SdfCtrlTLM<E>(name, flow_mode));
  }

  explicit R2SdfCtrlTLM(sc_core::sc_module_name name, FFTFlowMode fm)
      : sc_core::sc_module(name), flow_mode(fm) {
    reset();
  }

  void reset() {
    cnt_cur = CountT{0};
    sigs_cur = compute_outputs(cnt_cur, false);
  }

  int stage_to_tw_slot(unsigned p) const {
    if (flow_mode == FFTFlowMode::DIF) {
      if (p >= kNStages - 1) return -1;   // last stage is trivial
      return static_cast<int>(p);
    } else {
      if (p == 0) return -1;              // first stage is trivial
      return static_cast<int>(p - 1);
    }
  }

  const R2SdfCtrlSigs& step(bool sample_fire) {
    sigs_cur = compute_outputs(cnt_cur, sample_fire);
    if (sample_fire) {
      CountT cnt_next = cnt_cur;
      cnt_next = cnt_next + CountT{1};
      cnt_cur = cnt_next;
    }
    return sigs_cur;
  }

  R2SdfCtrlSigs decode_from_count(CountT c) const {
    return compute_outputs(c, true);
  }

  R2SdfCtrlSigs compute_outputs(CountT c, bool sample_fire) const;

private:
  CountT cnt_cur{0};
  R2SdfCtrlSigs sigs_cur{};
  FFTFlowMode flow_mode = FFTFlowMode::DIT;
};

template <typename E>
class R2SdfStageTLM : public sc_core::sc_module {
public:
  using T   = typename E::Eval_T;
  using CxT = std::complex<T>;

  static std::unique_ptr<R2SdfStageTLM<E>>
  create(Context<E>& ctx,
         sc_core::sc_module_name name,
         std::size_t fft_size,
         std::size_t stage_idx,
         FFTFlowMode flow_mode,
         SyscMemory<E>* twiddle_mem,
         std::shared_ptr<R2SdfCtrlTLM<E>> ctrl = nullptr
  );
  
  void set_ctrl(std::shared_ptr<R2SdfCtrlTLM<E>> c) { 
    ctrl = std::move(c); 
  }
  
  void allocate_state(Context<E>& ctx);
  void reset_state();
  void process_block(const std::vector<CxT>& in,
                     std::vector<CxT>& out,
                     bool inverse);

  R2SdfStageTLM(Context<E>& ctx,
                sc_core::sc_module_name name,
                std::size_t fft_size,
                std::size_t stage_idx,
                FFTFlowMode flow_mode,
                SyscMemory<E>* twiddle_mem,
                std::shared_ptr<R2SdfCtrlTLM<E>> ctrl);

  static ComplexPlain<T> to_plain(const CxT& z) {
    return ComplexPlain<T>{z.real(), z.imag()};
  }

  std::size_t get_nstages() const {
    std::size_t n = fft_size;
    std::size_t s = 0;
    while (n > 1) {
      n >>= 1;
      ++s;
    }
    return s;
  }

  std::size_t get_span() const {
    if (flow_mode == FFTFlowMode::DIT) {
      return std::size_t(1) << (stage_idx + 1);
    }
    return std::size_t(1) << (get_nstages() - stage_idx);
  }

  std::size_t get_delay_len() const { 
    return get_span() >> 1; 
  }

  std::size_t get_twiddle_index_dit(std::size_t local_idx) const {
    const std::size_t group  = std::size_t(1) << (stage_idx + 1);
    const std::size_t stride = fft_size / group;
    return (local_idx * stride) & (fft_size - 1);
  }

  std::size_t get_twiddle_index_dif(std::size_t local_idx) const {
    const std::size_t group  = std::size_t(1) << (get_nstages() - stage_idx);
    const std::size_t stride = fft_size / group;
    return (local_idx * stride) & (fft_size - 1);
  }

  CxT get_twiddle(std::size_t k, bool inverse) const {
    if (twiddle_mem == nullptr) {
      throw std::runtime_error("R2SdfStageTLM twiddle memory is null");
    }
    const std::size_t addr = 2 * k;
    const T re = twiddle_mem->data()[addr];
    const T im = twiddle_mem->data()[addr + 1];
    const CxT w(re, im);
    return inverse ? std::conj(w) : w;
  }

  int get_ctrl_tw_slot() const {
    const std::size_t nstages = get_nstages();

    if (flow_mode == FFTFlowMode::DIT) {
      // DIT: stage 0 is trivial twiddle stage
      return (stage_idx == 0) ? -1 : static_cast<int>(stage_idx - 1);
    } else {
      // DIF: last stage is trivial twiddle stage
      return (stage_idx + 1 == nstages) ? -1 : static_cast<int>(stage_idx);
    }
  }

private:
  std::size_t fft_size = 0;
  std::size_t stage_idx = 0;
  FFTFlowMode flow_mode = FFTFlowMode::DIT;
  SyscMemory<E>* twiddle_mem = nullptr;
  std::shared_ptr<R2SdfCtrlTLM<E>> ctrl;
  std::unique_ptr<ComplexShiftRegisterTLM<T>> shiftreg;
  std::unique_ptr<ComplexMultiplierTLM<T>> cmul;
  bool is_init = false;
};

template <typename E>
class R2SdfFFTTLM : public ObjectWithMutableHyperparams,
                    public sc_core::sc_module, 
                    public IFFT<typename E::Eval_T> {
public:
  using T       = typename E::Eval_T;
  using CxT     = std::complex<T>;
  using VecR    = typename IFFT<T>::VecR;
  using VecC    = typename IFFT<T>::VecC;
  using FFTMode = typename IFFT<T>::FFTMode;

  static std::unique_ptr<R2SdfFFTTLM<E>>
  create(Context<E>& ctx, sc_core::sc_module_name name,
         std::size_t fft_size = E::fft_size,
         FFTFlowMode flow_mode = E::use_dit ? FFTFlowMode::DIT : FFTFlowMode::DIF);

  static bool run_testbench(Context<E>& ctx) { return true; }

  tlm_utils::simple_target_socket<R2SdfFFTTLM> targ_socket{"targ_socket"};

  std::size_t get_fftsize() const override { return fft_size; }
  void fftreal(const VecR& in, VecC& out) const override;
  void fftcplx(const VecC& in, VecC& out) const override;
  void ifftcplx(const VecC& in, VecC& out) const override;

  void allocate_state(Context<E>& ctx);
  void state_reset();

  void update_hyperparams(const json& params) override {
    if (params.contains("scale_each_stage")) {
      scale_each_stage = params.at("scale_each_stage").template get<bool>();
    }
    if (params.contains("use_ctrl")) {
      use_ctrl = params.at("use_ctrl").template get<bool>();
    }
  }

  json get_hyperparams() const override {
    return {
      {"otype", "r2sdf_fft_tlm"},
      {"fft_size", fft_size},
      {"flow_mode", flow_mode == FFTFlowMode::DIT ? "dit" : "dif"},
      {"scale_each_stage", scale_each_stage},
      {"use_ctrl", use_ctrl}
    };
  }

  json serialize(Context<E>&) const {
    return {
      {"fft_size", fft_size},
      {"flow_mode", flow_mode == FFTFlowMode::DIT ? "dit" : "dif"},
      {"use_ctrl", use_ctrl ? "true" : "false"},
      {"scale_each_stage", scale_each_stage}
    };
  }

  void deserialize(Context<E>&, const json& data) { 
    update_hyperparams(data); 
  }

  void dump_state(Context<E>&, const std::string& = "") const {}

  virtual ~R2SdfFFTTLM() = default;

protected:
  R2SdfFFTTLM(Context<E>& ctx,
              sc_core::sc_module_name name,
              std::size_t fft_size,
              FFTFlowMode flow_mode);

  void b_transport(tlm::tlm_generic_payload& trans, sc_core::sc_time& delay) {
    (void)trans;
    delay += butterfly_delay + twiddle_delay + memory_delay + cmplxmul_delay;
  }

  bool get_direct_mem_ptr(tlm::tlm_generic_payload&, tlm::tlm_dmi&) { 
    return false; 
  }

  unsigned int transport_dbg(tlm::tlm_generic_payload&) { 
    return 0; 
  }

private:
  void validate_fft_size() const {
    if (fft_size == 0 || ((fft_size & (fft_size - 1)) != 0)) {
      throw std::invalid_argument("R2SdfFFTTLM: fft_size must be power-of-2");
    }
  }

  std::size_t get_nstages() const {
    std::size_t n = fft_size;
    std::size_t s = 0;
    while (n > 1) {
      n >>= 1;
      ++s;
    }
    return s;
  }

  void allocate_twiddle(Context<E>& ctx);
  void bit_reverse(VecC& data) const;
  void process_frame(const VecC& in, VecC& out, bool inverse) const;

private:
  std::size_t fft_size = 0;
  FFTFlowMode flow_mode = FFTFlowMode::DIT;

  bool scale_each_stage = E::scale_each_stage;
  sc_core::sc_time butterfly_delay  = sc_core::sc_time(E::butterfly_latency, sc_core::SC_NS);
  sc_core::sc_time twiddle_delay    = sc_core::sc_time(E::twiddle_latency, sc_core::SC_NS);
  sc_core::sc_time memory_delay     = sc_core::sc_time(E::memory_latency, sc_core::SC_NS);
  sc_core::sc_time cmplxmul_delay    = sc_core::sc_time(E::cmplxmul_latency, sc_core::SC_NS);
  std::unique_ptr<SyscMemory<E>> twiddle_mem;
  std::shared_ptr<R2SdfCtrlTLM<E>> ctrl;
  std::vector<std::unique_ptr<R2SdfStageTLM<E>>> r2sdfstgs;
  std::vector<CxT> last_fftin;
  std::vector<CxT> last_fftout;
  bool is_init = false;
  bool use_ctrl = E::use_ctrl;
};


}