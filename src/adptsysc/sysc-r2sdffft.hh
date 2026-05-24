#pragma once
#include <tlm>
#include <tlm_utils/simple_target_socket.h>
#include <tlm_utils/simple_initiator_socket.h>
#include <deque>
#include <memory>
#include <array> 
#include <vector> 
#include <complex> 
#include <stdexcept>
#include <adptsysc/adptsysc.hh>
#include <adptsysc/object.hh>
#include <adptsysc/syscfx-utils.hh>
#include <adptsysc/design-lib.hh>
#include <adptsysc/sysc-mem.hh>
#include <adptsysc/sysc-cmplxmul.hh>
#include <adptsysc/sysc-shiftreg.hh>

namespace adptsysc {

template <typename E> struct Context;

template <std::size_t N>
constexpr bool is_pow2_constexpr_v = (N > 0) && ((N & (N - 1)) == 0);

std::size_t clog2int (const std::size_t n);

template <typename E>
class R2SdfCtrlTLM : public ObjectWithMutableHyperparams,
                     public sc_core::sc_module {
public:
  using CountT  = std::size_t;
  using TwAddrT = std::size_t;

  struct R2SdfCtrlSigs {
    CountT cnt{0};

    std::vector<bool> s;
    std::vector<bool> twdlrom_en;
    std::vector<TwAddrT> tw_addr_local;
    std::vector<TwAddrT> tw_addr_global;

    bool frame_first{false};
    bool frame_last{false};
  };

  static std::shared_ptr<R2SdfCtrlTLM<E>>
  create(Context<E>& ctx, sc_core::sc_module_name name, FFTFlowMode fm, std::size_t fftsz);

  explicit R2SdfCtrlTLM(sc_core::sc_module_name name, FFTFlowMode fm, std::size_t fftsz);

  void reset();
  int stage_to_tw_slot(unsigned p) const;
  const R2SdfCtrlSigs& step(bool sample_fire);
  R2SdfCtrlSigs decode_from_count(CountT c) const;
  R2SdfCtrlSigs compute_outputs(CountT c, bool sample_fire) const;

  void update_hyperparams(const json& params) override;
  json get_hyperparams() const override;

  std::size_t get_fftsize() const { return fft_size; }
  unsigned get_nstages() const { return nstages; }
  unsigned get_ntwdls() const { return ntwdls; }

private:
  CountT cnt_cur{0};
  R2SdfCtrlSigs sigs_cur{};
  FFTFlowMode flow_mode{FFTFlowMode::DIT};

  std::size_t fft_size{0};
  unsigned cnt_width{0};
  unsigned nstages{0};
  unsigned ntwdls{0};
};

template <typename E>
class R2SdfStageTLM : public sc_core::sc_module {
public:
  using T   = typename E::Eval_T;
  using CxT = std::complex<T>;

  tlm_utils::simple_initiator_socket<R2SdfStageTLM> cmul_init_socket{"cmul_init_socket"};
  tlm_utils::simple_initiator_socket<R2SdfStageTLM> shiftreg_init_socket{"shiftreg_init_socket"};

  static std::unique_ptr<R2SdfStageTLM<E>>
  create(Context<E>& ctx,
         sc_core::sc_module_name name,
         std::size_t fft_size,
         std::size_t stage_idx,
         FFTFlowMode flow_mode,
         SyscMemory<E>* twiddle_mem,
         std::shared_ptr<TraceFile<E>> tracefile = nullptr,
         std::shared_ptr<R2SdfCtrlTLM<E>> ctrl = nullptr,
         bool use_ctrl = false);

  R2SdfStageTLM(Context<E>& ctx,
                sc_core::sc_module_name name,
                std::size_t fft_size,
                std::size_t stage_idx,
                FFTFlowMode flow_mode,
                SyscMemory<E>* twiddle_mem,
                std::shared_ptr<TraceFile<E>> tracefile,
                std::shared_ptr<R2SdfCtrlTLM<E>> ctrl,
                bool use_ctrl);

  void set_tracefile(std::shared_ptr<TraceFile<E>> tf);
  void set_ctrl(std::shared_ptr<R2SdfCtrlTLM<E>> ctrl);
  void allocate_state(Context<E>& ctx);
  void reset_state();
  void process_block(const std::vector<CxT>& in,
                     std::vector<CxT>& out,
                     bool inverse);

  static ComplexPlain<T> to_plain(const CxT& z);
  std::size_t get_nstages() const;
  std::size_t get_span() const;
  std::size_t get_delay_len() const;
  std::size_t get_twiddle_index_dit(std::size_t local_idx) const;
  std::size_t get_twiddle_index_dif(std::size_t local_idx) const;
  CxT get_twiddle(std::size_t k, bool inverse) const;
  int get_ctrl_tw_slot() const;
  ComplexPlain<T> shiftreg_step_tlm(const ComplexPlain<T>& in);
  void shiftreg_clear_tlm();
  ComplexPlain<T> cmul_mul_tlm(const ComplexPlain<T>& a, const ComplexPlain<T>& b);

private:
  std::size_t fft_size = 0;
  std::size_t stage_idx = 0;
  FFTFlowMode flow_mode = FFTFlowMode::DIT;
  SyscMemory<E>* twiddle_mem = nullptr;
  std::shared_ptr<R2SdfCtrlTLM<E>> ctrl;
  std::shared_ptr<TraceFile<E>> tracefile;
  std::unique_ptr<ComplexShiftRegisterTLM<T>> shiftreg;
  std::unique_ptr<ComplexMultiplierTLM<T>> cmul;
  bool is_init = false;
  bool use_ctrl = false;
};


template <typename E>
class R2SdfFFTTLM : public ObjectWithMutableHyperparams,
                    public sc_core::sc_module, 
                    public FFTIntf<typename E::Eval_T> {
public:
  using T       = typename E::Eval_T;
  using CxT     = std::complex<T>;
  using VecR    = typename FFTIntf<T>::VecR;
  using VecC    = typename FFTIntf<T>::VecC;
  using FFTMode = typename FFTIntf<T>::FFTMode;

  static std::unique_ptr<R2SdfFFTTLM<E>>
  create(Context<E>& ctx,
         sc_core::sc_module_name name,
         std::size_t fft_size,
         FFTFlowMode flow_mode = E::use_dit ? FFTFlowMode::DIT : FFTFlowMode::DIF,
         FFTDirection fft_dir = FFTDirection::FFT);

  static bool run_testbench(Context<E>& ctx);
  tlm_utils::simple_initiator_socket<R2SdfFFTTLM> twiddle_init_socket{"twiddle_init_socket"};
  tlm_utils::simple_target_socket<R2SdfFFTTLM> targ_socket{"targ_socket"};

  std::size_t get_fftsize() const override;
  void fftreal(const VecR& in, VecC& out) const override;
  void fftcplx(const VecC& in, VecC& out) const override;
  void ifftcplx(const VecC& in, VecC& out) const override;

  void allocate_state(Context<E>& ctx);
  void state_reset();

  void update_hyperparams(const json& params) override;
  json get_hyperparams() const override;

  json serialize(Context<E>&) const;
  void deserialize(Context<E>&, const json& data);
  void dump_state(Context<E>&, const std::string& = "") const;

  virtual ~R2SdfFFTTLM();

protected:
  R2SdfFFTTLM(Context<E>& ctx,
              sc_core::sc_module_name name,
              std::size_t fft_size,
              FFTFlowMode flow_mode,
              FFTDirection fft_dir);

  void b_transport(tlm::tlm_generic_payload& trans, sc_core::sc_time& delay);
  bool get_direct_mem_ptr(tlm::tlm_generic_payload&, tlm::tlm_dmi&);
  unsigned int transport_dbg(tlm::tlm_generic_payload&);

private:
  void validate_fft_size() const;
  std::size_t get_nstages() const;
  void allocate_twiddle(Context<E>& ctx);
  void bit_reverse(VecC& data) const;
  void process_frame(const VecC& in, VecC& out, bool inverse) const;

private:
  std::size_t fft_size = 0;
  FFTFlowMode flow_mode = FFTFlowMode::DIT;
  FFTDirection fft_dir = FFTDirection::FFT;
  bool scale_each_stage = false;
  std::unique_ptr<SyscMemory<E>> twiddle_mem;
  std::shared_ptr<R2SdfCtrlTLM<E>> ctrl;
  std::vector<std::unique_ptr<R2SdfStageTLM<E>>> r2sdfstgs;
  std::shared_ptr<TraceFile<E>> tracefile;
  std::vector<CxT> last_fftin;
  std::vector<CxT> last_fftout;
  bool is_init = false;
  bool use_ctrl = false;
};


}