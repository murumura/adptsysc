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


} 