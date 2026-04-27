#include <tlm>
#include <tlm_utils/simple_target_socket.h>
#include <deque>
#include <memory>
#include <adptsysc/object.hh>
#include <adptsysc/design-lib.hh>
#include <adptsysc/sysc-mem.hh>
namespace adptsysc {

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
         SyscMemory<E>* twiddle_mem);

  void allocate_state(Context<E>& ctx);
  void reset_state();
  void process_block(const std::vector<CxT>& in,
                     std::vector<CxT>& out,
                     bool inverse);

protected:
  R2SdfStageTLM(Context<E>& ctx,
                sc_core::sc_module_name name,
                std::size_t fft_size,
                std::size_t stage_idx,
                FFTFlowMode flow_mode,
                SyscMemory<E>* twiddle_mem);

private:
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

  std::size_t get_delay_len() const { return get_span() >> 1; }

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

private:
  std::size_t fft_size = 0;
  std::size_t stage_idx = 0;
  FFTFlowMode flow_mode = FFTFlowMode::DIT;
  SyscMemory<E>* twiddle_mem = nullptr;

  std::unique_ptr<ComplexShiftRegisterTLM<T>> shiftreg;
  std::unique_ptr<ComplexMultiplierTLM<T>> cmul;
  bool is_init = false;
};

template <typename E>
class R2SdfFFTTLM :
  public ObjectWithMutableHyperparams,
  public sc_core::sc_module,
  public IFFT<typename E::Eval_T> {
public:
  using T       = typename E::Eval_T;
  using CxT     = std::complex<T>;
  using VecR    = typename IFFT<T>::VecR;
  using VecC    = typename IFFT<T>::VecC;
  using FFTMode = typename IFFT<T>::FFTMode;

  static std::unique_ptr<R2SdfFFTTLM<E>>
  create(Context<E>& ctx,
         sc_core::sc_module_name name,
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
  }

  json get_hyperparams() const override {
    return {
      {"otype", "r2sdf_fft_tlm"},
      {"fft_size", fft_size},
      {"flow_mode", flow_mode == FFTFlowMode::DIT ? "dit" : "dif"},
      {"scale_each_stage", scale_each_stage}
    };
  }

  json serialize(Context<E>&) const {
    return {
      {"fft_size", fft_size},
      {"flow_mode", flow_mode == FFTFlowMode::DIT ? "dit" : "dif"},
      {"scale_each_stage", scale_each_stage}
    };
  }

  void deserialize(Context<E>&, const json& data) { update_hyperparams(data); }
  void dump_state(Context<E>&, const std::string& = "") const {}

  virtual ~R2SdfFFTTLM() = default;

protected:
  R2SdfFFTTLM(Context<E>& ctx,
              sc_core::sc_module_name name,
              std::size_t fft_size,
              FFTFlowMode flow_mode);

  void b_transport(tlm::tlm_generic_payload& trans, sc_core::sc_time& delay) {
    (void)trans;
    delay += butterfly_delay + twiddle_delay + memory_delay;
  }

  bool get_direct_mem_ptr(tlm::tlm_generic_payload&, tlm::tlm_dmi&) { return false; }
  unsigned int transport_dbg(tlm::tlm_generic_payload&) { return 0; }

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
  sc_core::sc_time butterfly_delay = sc_core::sc_time(E::butterfly_latency, sc_core::SC_NS);
  sc_core::sc_time twiddle_delay   = sc_core::sc_time(E::twiddle_latency, sc_core::SC_NS);
  sc_core::sc_time memory_delay    = sc_core::sc_time(E::memory_latency, sc_core::SC_NS);

  std::unique_ptr<SyscMemory<E>> twiddle_mem;
  std::vector<std::unique_ptr<R2SdfStageTLM<E>>> r2sdfstgs;
  std::vector<CxT> last_fftin;
  std::vector<CxT> last_fftout;
  bool is_init = false;
};


}