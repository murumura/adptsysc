#pragma once

#include <systemc>

#include <adptsysc/adptsysc.hh>
#include <adptsysc/object.hh>
#include <adptsysc/sysc-r2sdffft-cycle.hh>

#include <cstddef>
#include <memory>
#include <vector>

namespace adptsysc {

template <typename E>
class OverlapSaveFdafCycle : public ObjectWithMutableHyperparams,
                             public sc_core::sc_module {
public:
  using T = typename E::Fxpt_T;
  using CxT = ComplexPlain<T>;

  sc_core::sc_in<bool> clk{"clk"};
  sc_core::sc_in<bool> rst_n{"rst_n"};

  sc_core::sc_in<bool> train{"train"};
  sc_core::sc_in<bool> in_valid{"in_valid"};
  sc_core::sc_out<bool> in_ready{"in_ready"};
  sc_core::sc_in<T> x_in{"x_in"};
  sc_core::sc_in<T> d_in{"d_in"};

  sc_core::sc_out<bool> out_valid{"out_valid"};
  sc_core::sc_in<bool> out_ready{"out_ready"};
  sc_core::sc_out<T> y_out{"y_out"};
  sc_core::sc_out<T> e_out{"e_out"};
  sc_core::sc_out<bool> out_last{"out_last"};
  sc_core::sc_out<bool> busy{"busy"};

  static std::unique_ptr<OverlapSaveFdafCycle<E>>
  create(Context<E>& ctx,
         sc_core::sc_module_name name,
         std::size_t filter_ncoeff = E::filter_ncoeff,
         const T* initptr = nullptr);

  static bool run_testbench(Context<E>& ctx);

  OverlapSaveFdafCycle(Context<E>& ctx,
                       sc_core::sc_module_name name,
                       std::size_t filter_ncoeff,
                       const T* initptr = nullptr);

  void update_hyperparams(const json& params) override;
  json get_hyperparams() const override;

  std::size_t get_filter_ncoeff() const { return filter_ncoeff; }
  std::size_t get_fftsize() const { return fft_size; }
  std::size_t get_block_size() const { return block_size; }

private:
  enum class State {
    INIT_W_PREP,
    INIT_W_SEND,
    INIT_W_RECV,
    COLLECT,
    PREP_X,
    FFT_X_SEND,
    FFT_X_RECV,
    FILTER_MUL,
    IFFT_Y_SEND,
    IFFT_Y_RECV,
    OUTPUT_ERROR,
    PREP_E,
    FFT_E_SEND,
    FFT_E_RECV,
    POWER_GRAD_UPDATE,
    IFFT_W_SEND,
    IFFT_W_RECV,
    CONSTRAINT_ZERO,
    FFT_W_SEND,
    FFT_W_RECV,
    FINISH_BLOCK,
  };

  Context<E>& ctx;
  std::size_t filter_ncoeff = 0;
  std::size_t fft_size = 0;
  std::size_t block_size = 0;

  double mu = 0.01;
  double alpha = 0.9;
  double eps = 1e-8;
  bool use_power_norm = true;

  std::vector<T> initial_weights;
  std::vector<T> x_hist;
  std::vector<T> x_new;
  std::vector<T> d_block;
  std::vector<T> y_block;
  std::vector<T> e_block;

  std::vector<CxT> x_time;
  std::vector<CxT> x_freq;
  std::vector<CxT> y_freq;
  std::vector<CxT> y_time;
  std::vector<CxT> e_time;
  std::vector<CxT> e_freq;
  std::vector<CxT> w_freq;
  std::vector<CxT> w_time;
  std::vector<double> pow_est;

  std::size_t collect_idx = 0;
  std::size_t bin_idx = 0;
  std::size_t out_idx = 0;
  std::size_t fft_send_idx = 0;
  std::size_t fft_recv_idx = 0;
  bool fft_send_presented = false;
  bool train_block = true;
  bool output_presented = false;
  std::size_t block_count = 0;

  std::vector<CxT>* fft_send_vec = nullptr;
  std::vector<CxT>* fft_recv_vec = nullptr;
  bool fft_inverse_mode = false;
  State fft_next_state = State::COLLECT;

  std::unique_ptr<R2SdfFFTCycle<E>> fft_core;
  sc_core::sc_signal<bool> fft_inverse{"fft_inverse"};
  sc_core::sc_signal<bool> fft_in_valid{"fft_in_valid"};
  sc_core::sc_signal<bool> fft_in_ready{"fft_in_ready"};
  sc_core::sc_signal<T> fft_din_re{"fft_din_re"};
  sc_core::sc_signal<T> fft_din_im{"fft_din_im"};
  sc_core::sc_signal<bool> fft_in_last{"fft_in_last"};
  sc_core::sc_signal<bool> fft_out_valid{"fft_out_valid"};
  sc_core::sc_signal<bool> fft_out_ready{"fft_out_ready"};
  sc_core::sc_signal<T> fft_dout_re{"fft_dout_re"};
  sc_core::sc_signal<T> fft_dout_im{"fft_dout_im"};
  sc_core::sc_signal<bool> fft_out_last{"fft_out_last"};
  sc_core::sc_signal<bool> fft_busy{"fft_busy"};

  State state = State::INIT_W_PREP;

  static CxT cmul(const CxT& a, const CxT& b);
  static CxT conj_mul(const CxT& a, const CxT& b);

  void validate_config() const;
  void reset_vectors();
  void start_fft(std::vector<CxT>& src,
                 std::vector<CxT>& dst,
                 bool inverse_mode,
                 State next_state,
                 State send_state);
  bool fft_send_step();
  bool fft_recv_step();
  void run();
};

}  // namespace adptsysc
