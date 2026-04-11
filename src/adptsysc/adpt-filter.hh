#pragma once

#include <adptsysc/object.hh>
#include <Eigen/Dense>
#include <cstddef>

namespace adptsysc {

template <typename T, typename PARAMS_T = T, typename ACC_T = float>
class AdaptiveFilter : public ParametricObject<PARAMS_T> {
public:
  using DataMatrix = Eigen::Matrix<ACC_T, Eigen::Dynamic, Eigen::Dynamic>;
  using DataVec    = Eigen::Matrix<ACC_T, Eigen::Dynamic, 1>;
  using AccVec   = Eigen::Matrix<ACC_T, Eigen::Dynamic, 1>;
  using ParamVec = Eigen::Matrix<PARAMS_T, Eigen::Dynamic, 1>;

  virtual ~AdaptiveFilter() = default;
  virtual std::size_t get_n_weights() const = 0;
  virtual void reset() = 0;
  virtual T forward(const Eigen::Ref<const DataVec>& x) const = 0;

  std::size_t n_params() const override {
    return get_n_weights();
  }

  void init_params(float* params_full_precision) override {
    std::fill(params_full_precision, params_full_precision + get_n_weights(), 0.0f);
  }
  virtual Eigen::Ref<AccVec> get_weights_acc() = 0;
  virtual Eigen::Ref<ParamVec> get_weights_q() = 0;

protected:
  using ParametricObject<PARAMS_T>::params;
  using ParametricObject<PARAMS_T>::infer_params;
  using ParametricObject<PARAMS_T>::grads;
};

template <typename T, typename PARAMS_T = T, typename ACC_T = float>
class LMSFilter : public AdaptiveFilter<T, PARAMS_T, ACC_T> {
public:
  using Base     = AdaptiveFilter<T, PARAMS_T, ACC_T>;
  using DataVec  = typename Base::DataVec;
  using AccVec   = typename Base::AccVec;
  using ParamVec = typename Base::ParamVec;

  LMSFilter(const std::size_t n_weights) : n_ws(n_weights) {
    w_acc = AccVec::Zero(n_ws);
    w_q = ParamVec::Zero(n_ws);
  }

  std::size_t get_n_weights() const override { 
    return n_ws; 
  }

  void reset() override { 
    w_acc.setZero(); 
    w_q.setZero();
  }

  T forward(const Eigen::Ref<const DataVec>& x) const override {
    return static_cast<T>(w_acc.dot(x.template cast<ACC_T>()));
  }

  void set_params_impl(PARAMS_T* p, PARAMS_T* inf, PARAMS_T* g) override {
    // Logic for linking external memory if needed
  }

  Eigen::Ref<AccVec> get_weights_acc() override { 
    return w_acc; 
  }

  Eigen::Ref<ParamVec> get_weights_q() override { 
    return w_q; 
  }
  
  json get_hyperparams() const override { 
    return {
      {"otype", "lms_filter"}, 
      {"n_taps", n_ws}
    }; 
  }

private:
  std::size_t n_ws;
  AccVec w_acc;
  ParamVec w_q;
};

template <typename T, typename PARAMS_T = T, typename ACC_T = float>
class RLSFilter : public AdaptiveFilter<T, PARAMS_T, ACC_T> {
public:
  using Base     = AdaptiveFilter<T, PARAMS_T, ACC_T>;
  using DataVec  = typename Base::DataVec;
  using AccVec   = typename Base::AccVec;
  using ParamVec = typename Base::ParamVec;
  using DataMatrix  = typename Base::DataMatrix;

  RLSFilter(const std::size_t n_weights) : n_ws(n_weights) {
    w_acc = AccVec::Zero(n_ws);
    w_q   = ParamVec::Zero(n_ws);
  }

  std::size_t get_n_weights() const override { 
    return n_ws; 
  }

  void reset() override {
    w_acc.setZero();
    w_q.setZero();
  }

  T forward(const Eigen::Ref<const DataVec>& x) const override {
    return static_cast<T>(w_acc.dot(x.template cast<ACC_T>()));
  }

  void set_params_impl(PARAMS_T* p, PARAMS_T* inf, PARAMS_T* g) override {
    // Logic for linking external memory if needed
  }

  Eigen::Ref<AccVec> get_weights_acc() override { return w_acc; }
  Eigen::Ref<ParamVec> get_weights_q() override { return w_q; }

  json get_hyperparams() const override {
    return {
      {"otype", "rls_filter"},
      {"n_taps", n_ws}
    };
  }

private:
  std::size_t n_ws;
  AccVec  w_acc;
  ParamVec w_q;
};

template <typename T, typename PARAMS_T = T, typename ACC_T = float>
class APAFilter : public AdaptiveFilter<T, PARAMS_T, ACC_T> {
public:
  using Base     = AdaptiveFilter<T, PARAMS_T, ACC_T>;
  using DataVec  = typename Base::DataVec;
  using AccVec   = typename Base::AccVec;
  using ParamVec = typename Base::ParamVec;

  APAFilter(const std::size_t n_weights)
    : n_ws(n_weights),
      w_acc(AccVec::Zero(n_ws)),
      w_q(ParamVec::Zero(n_ws)) {}

  std::size_t get_n_weights() const override { return n_ws; }

  void reset() override {
    w_acc.setZero();
    w_q.setZero();
  }

  T forward(const Eigen::Ref<const DataVec>& x) const override {
    assert(x.size() == n_ws);
    return static_cast<T>(w_acc.dot(x.template cast<ACC_T>()));
  }

  void set_params_impl(PARAMS_T* p, PARAMS_T* inf, PARAMS_T* g) override {}

  Eigen::Ref<AccVec> get_weights_acc() override {
    return w_acc;
  }

  Eigen::Ref<ParamVec> get_weights_q() override {
    return w_q;
  }

 json get_hyperparams() const override {
    return {
      {"otype", "affine_projection_filter"},
      {"n_taps", n_ws}
    };
  }

private:
  std::size_t n_ws;
  AccVec   w_acc;
  ParamVec w_q;
};

template <typename T, typename PARAMS_T = T, typename ACC_T = float>
class OverlapSaveFdaf : public AdaptiveFilter<T, PARAMS_T, ACC_T> {
public:
  using Base     = AdaptiveFilter<T, PARAMS_T, ACC_T>;
  using DataVec  = typename Base::DataVec;
  using AccVec   = typename Base::AccVec;
  using ParamVec = typename Base::ParamVec;
  using CxT      = std::complex<ACC_T>;

  OverlapSaveFdaf(const std::size_t n_weights)
    : n_ws(n_weights),
      M(n_weights),
      N(2 * n_weights),
      x_hist(DataVec::Zero(M)),
      y_out_time_last(DataVec::Zero(M)),
      w_time_cache(AccVec::Zero(M)) {}

  std::size_t get_n_weights() const override { return n_ws; }

  void reset() override {
    x_hist.setZero();
    y_out_time_last.setZero();
    w_time_cache.setZero();
  }

  // compatibility API (returns last sample)
  T forward(const Eigen::Ref<const DataVec>& x) const override {
    assert(x.size() == M);
    return static_cast<T>(y_out_time_last[M - 1]);
  }

  void forward_block (
    const Eigen::Ref<const DataVec>& x_in,
    const std::vector<CxT>& w_freq,
    Eigen::Ref<DataVec> y_out
  ) {
    // overlap-save
    Eigen::Matrix<ACC_T, -1, 1> x_block(N);
    x_block << x_hist, x_in;

    x_freq_last = fft(x_block);

    // convolution
    std::vector<CxT> output_freq(N);
    for (int i = 0; i < N; ++i)
      output_freq[i] = x_freq_last[i] * w_freq[i];

    auto y_time = ifft(output_freq);

    y_out = y_time.tail(M).real();

    y_out_time_last = y_out;
    x_hist = x_in;
  }

  // required for optimizer
  const std::vector<CxT>& get_last_input_freq() const {
    return x_freq_last;
  }

  // expose time-domain weights (IFFT of w_freq)
  void update_weight_cache(const std::vector<CxT>& w_freq) {
    auto w_time = ifft(w_freq);

    for (int i = 0; i < M; ++i)
      w_time_cache[i] = std::real(w_time[i]);
  }

  // required by interface
  Eigen::Ref<AccVec> get_weights_acc() override {
    return w_time_cache;
  }

  Eigen::Ref<ParamVec> get_weights_q() override {
    throw std::runtime_error("FDAF does not use quantized weights");
  }

private:
  std::size_t n_ws;
  std::size_t M;
  std::size_t N;

  DataVec x_hist;
  DataVec y_out_time_last;

  std::vector<CxT> x_freq_last;
  AccVec w_time_cache;

  std::vector<CxT> fft(const Eigen::Matrix<ACC_T,-1,1>& x) const;
  std::vector<CxT> ifft(const std::vector<CxT>& X) const;
};

} 