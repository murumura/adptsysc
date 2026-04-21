#include <tlm>
#include <tlm_utils/simple_target_socket.h>
#include <deque>
#include <memory>
#include <adptsysc/object.hh>
#include <adptsysc/design-lib.hh>
#include <adptsysc/sysc-mem.hh>
namespace adptsysc {

template <typename E>
class R2SdfStageTlm : public sc_core::sc_module {
public:
  using T   = typename E::Eval_T;
  using CxT = std::complex<T>;

  static std::unique_ptr<R2SdfStageTlm<E>>
  create(Context<E>& ctx,
         sc_core::sc_module_name name,
         std::size_t fft_size,
         std::size_t stage_idx,
         FFTFlowMode flow_mode,
         SyscMemory<E>* twiddle_mem);

  void allocate_state(Context<E>& ctx);
  void reset_state();

  void process_sample(const CxT& xin, bool vin, CxT& yout, bool& vout, bool inverse);

protected:
  R2SdfStageTlm(Context<E>& ctx,
                sc_core::sc_module_name name,
                std::size_t fft_size,
                std::size_t stage_idx,
                FFTFlowMode flow_mode,
                SyscMemory<E>* twiddle_mem);

private:
  std::size_t n_stages() const;
  std::size_t twiddle_index_dit(std::size_t local_idx) const;
  std::size_t twiddle_index_dif(std::size_t local_idx) const;
  CxT get_twiddle(std::size_t k, bool inverse) const;

private:
  std::size_t fft_size_ = 0;
  std::size_t stage_idx_ = 0;
  FFTFlowMode flow_mode_ = FFTFlowMode::DIT;
  SyscMemory<E>* twiddle_mem_ = nullptr;

  std::vector<CxT> delay_mem_;
  std::size_t wr_ptr_ = 0;
  std::size_t sample_ctr_ = 0;
  bool is_init_ = false;
};

template <typename E>
class R2SdfFFTTlm :
  public ObjectWithMutableHyperparams,
  public sc_core::sc_module,
  public IFFT<typename E::Eval_T> {
public:
  using T = typename E::Eval_T;
  using CxT = std::complex<T>;
  using VecR = typename IFFT<T>::VecR;
  using VecC = typename IFFT<T>::VecC;
  using FFTMode = typename IFFT<T>::FFTMode;

  static std::unique_ptr<R2SdfFFTTlm<E>>
  create(Context<E>& ctx,
         sc_core::sc_module_name name,
         std::size_t fft_size = E::fft_size,
         FFTFlowMode flow_mode = E::use_dit ? FFTFlowMode::DIT : FFTFlowMode::DIF);

  static bool run_testbench(Context<E>& ctx);

  tlm_utils::simple_target_socket<R2SdfFFTTlm> targ_socket;

  std::size_t get_fftsize() const override;
  void fftreal(const VecR& in, VecC& out) const override;
  void fftcplx(const VecC& in, VecC& out) const override;
  void ifftcplx(const VecC& in, VecC& out) const override;

  void allocate_state(Context<E>& ctx);
  void state_reset();

  void update_hyperparams(const json& params) override;
  json get_hyperparams() const override;

  json serialize(Context<E>& ctx) const;
  void deserialize(Context<E>& ctx, const json& data);
  void dump_state(Context<E>& ctx, const std::string& desc = "") const;

  virtual ~R2SdfFFTTlm() = default;

protected:
  R2SdfFFTTlm(Context<E>& ctx,
              sc_core::sc_module_name name,
              std::size_t fft_size,
              FFTFlowMode flow_mode);

  void b_transport(tlm::tlm_generic_payload& trans, sc_core::sc_time& delay);
  bool get_direct_mem_ptr(tlm::tlm_generic_payload& trans, tlm::tlm_dmi& dmi_data);
  unsigned int transport_dbg(tlm::tlm_generic_payload& trans);

private:
  void validate_fft_size() const;
  std::size_t n_stages() const;
  void allocate_twiddle(Context<E>& ctx);
  void bit_reverse(VecC& data) const;
  void process_frame(const VecC& in, VecC& out, bool inverse) const;

private:
  std::size_t fft_size_ = 0;
  FFTFlowMode flow_mode_ = FFTFlowMode::DIT;

  bool scale_each_stage_ = E::scale_each_stage;
  sc_core::sc_time butterfly_delay_ = sc_core::sc_time(E::butterfly_latency, sc_core::SC_NS);
  sc_core::sc_time twiddle_delay_ = sc_core::sc_time(E::twiddle_latency, sc_core::SC_NS);
  sc_core::sc_time memory_delay_ = sc_core::sc_time(E::memory_latency, sc_core::SC_NS);

  std::unique_ptr<SyscMemory<E>> twiddle_mem_;
  std::vector<std::unique_ptr<R2SdfStageTlm<E>>> stages_;

  std::vector<CxT> last_fft_in_;
  std::vector<CxT> last_fft_out_;

  bool is_init_ = false;
};

}