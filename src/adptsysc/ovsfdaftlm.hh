#pragma once

#include <systemc>
#include <tlm>
#include <tlm_utils/simple_target_socket.h>

#include <adptsysc/design-lib.hh>
#include <adptsysc/object.hh>

#include <complex>
#include <cstddef>
#include <memory>
#include <string>
#include <vector>

namespace adptsysc {

template <typename E>
struct Context;

template <typename E>
class OverlapSaveFdafTLM : public ObjectWithMutableHyperparams,
                           public sc_core::sc_module {
public:
  using T = typename E::Eval_T;
  using CxT = std::complex<T>;
  using FFTT = EigenFFTWrapper<T>;

  struct Txn {
    enum class Op {
      PROCESS,
      RESET,
      GET_WEIGHTS,
    };

    Op op = Op::PROCESS;
    std::vector<T> x_in;
    std::vector<T> d_in;
    std::vector<T> y_out;
    std::vector<T> e_out;
    std::vector<T> weights;
    bool train = true;
    bool ok = false;
    std::string error;
  };

  static std::unique_ptr<OverlapSaveFdafTLM<E>>
  create(Context<E>& ctx, sc_core::sc_module_name name,
         std::size_t filter_ncoeff, const T* filter_initptr = nullptr);

  static bool run_testbench(Context<E>& ctx);

  tlm_utils::simple_target_socket<OverlapSaveFdafTLM> targ_socket{
      "targ_socket"};

  void update_hyperparams(const json& params) override;
  json get_hyperparams() const override;

  void allocate_state(Context<E>& ctx);
  void state_reset(Context<E>& ctx);

  json serialize(Context<E>& ctx) const;
  void deserialize(Context<E>& ctx, const json& data);

  void dump_state(Context<E>& ctx,
                  const std::string& desc = "") const;

  void process_block(const std::vector<T>& x_in,
                     const std::vector<T>& d_in,
                     std::vector<T>& y_out,
                     std::vector<T>& e_out,
                     bool train = true);

  std::vector<T> get_time_weights() const;

  ~OverlapSaveFdafTLM() override = default;

protected:
  OverlapSaveFdafTLM(sc_core::sc_module_name name,
                     Context<E>& ctx,
                     std::size_t filter_ncoeff,
                     const T* filter_initptr = nullptr);

  void b_transport(tlm::tlm_generic_payload& trans, sc_core::sc_time& delay);
  bool get_direct_mem_ptr(tlm::tlm_generic_payload& trans, tlm::tlm_dmi& dmi_data);
  unsigned int transport_dbg(tlm::tlm_generic_payload& trans);

private:
  void validate_config() const;
  void forward_fft(const std::vector<CxT>& in, std::vector<CxT>& out) const;
  void inverse_fft(const std::vector<CxT>& in, std::vector<CxT>& out) const;
  void reset_filter_state();
  void constrain_weights();

  // Configuration: M taps, N=2M transform, L=M fresh samples/block.
  std::size_t filter_ncoeff = 0;
  std::size_t fft_size = 0;
  std::size_t block_size = 0;
  T mu = T(0.01);
  T alpha = T(0.9);
  T eps = T(1e-8);
  bool use_power_norm = true;

  std::vector<T> initial_weights;

  // Shared Eigen FFT backend. Complex mode supports every FDAF transform.
  std::unique_ptr<FFTT> fft_engine;

  // Persistent state.
  std::vector<CxT> w_freq;
  std::vector<T> pow_est;

  // Block temporaries.
  std::vector<CxT> x_block;
  std::vector<CxT> x_freq;
  std::vector<CxT> y_freq;
  std::vector<CxT> y_ifft;

  std::vector<T> d_block;
  std::vector<T> y_block;
  std::vector<T> e_block;

  std::vector<CxT> e_pad;
  std::vector<CxT> e_freq;
  std::vector<CxT> grad_freq;
  std::vector<CxT> grad_time;

  std::vector<T> x_hist;
  std::size_t sample_count = 0;
  std::size_t block_count = 0;

  sc_core::sc_time block_delay{};
  sc_core::sc_time fft_delay{};
  sc_core::sc_time ifft_delay{};

  bool is_init = false;
};

}  // namespace adptsysc
