#pragma once
#include <tlm>
#include <tlm_utils/simple_target_socket.h>
#include <deque>
#include <memory>
#include <adptsysc/object.hh>

namespace adptsysc {

template <typename E> struct Context;

template <typename E>
class OverlapSaveFdafTLM : public ObjectWithMutableHyperparams,
                           public sc_core::sc_module {
public:
  using T   = typename E::Eval_T;
  using CxT = std::complex<T>;

  static std::unique_ptr<OverlapSaveFdafTLM<E>>
  create(Context<E>& ctx, sc_core::sc_module_name name,
        const std::size_t filter_ncoeff, const T* filter_initptr = nullptr);

  static bool run_testbench(Context<E>& ctx);

  tlm_utils::simple_target_socket<OverlapSaveFdafTLM> targ_socket;

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

  virtual ~OverlapSaveFdafTLM() = default;

protected:
  OverlapSaveFdafTLM(sc_core::sc_module_name name,
                     Context<E>& ctx,
                     std::size_t filter_ncoeff,
                     const T* filter_initptr = nullptr);

  void b_transport(tlm::tlm_generic_payload& trans,
                   sc_core::sc_time& delay);
  bool get_direct_mem_ptr(tlm::tlm_generic_payload& trans,
                          tlm::tlm_dmi& dmi_data);
  unsigned int transport_dbg(tlm::tlm_generic_payload& trans);

private:
  // configuration
  std::size_t filter_ncoeff = 0; // M
  std::size_t fft_size      = 0; // 2M
  std::size_t block_size;
  T mu                      = T(0.01);
  T alpha                   = T(0.9);
  T eps                     = T(1e-8);
  bool use_power_norm       = true;

  // persistent state
  std::vector<CxT> w_freq;  // size fft_size
  std::vector<T>   pow_est; // size fft_size

  // block temporaries
  std::vector<CxT> x_block; // size fft_size
  std::vector<CxT> x_freq;  // size fft_size
  std::vector<CxT> y_freq;  // size fft_size
  std::vector<CxT> y_ifft;  // size fft_size

  std::vector<T>   d_block; // size M
  std::vector<T>   y_block; // size M
  std::vector<T>   e_block; // size M

  std::vector<CxT> e_pad;      // size fft_size
  std::vector<CxT> e_freq;     // size fft_size
  std::vector<CxT> grad_freq;  // size fft_size
  std::vector<CxT> grad_time;  // size fft_size

  // optional stream state
  std::vector<T> x_hist;    // size M, previous overlap
  std::size_t sample_count = 0;
  std::size_t block_count  = 0;

  // timing
  sc_core::sc_time block_delay{};
  sc_core::sc_time fft_delay{};
  sc_core::sc_time ifft_delay{};

  bool is_init = false;
};

}