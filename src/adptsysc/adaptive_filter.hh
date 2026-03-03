#pragma once

#include <adptsysc/object.hh>
#include <ostream>
#include <string>
#include <type_traits>
#include <Eigen/Dense>

namespace adptsysc {

std::string to_lower(const std::string str);
std::string to_upper(const std::string str);
bool equals_case_insensitive(const std::string& s1, const std::string& s2);

template <typename T>
struct AFStepState {
  Eigen::Ref<const Eigen::Matrix<T, Eigen::Dynamic, 1>> x; // regressor
  T d;                                                     // desired
};

template <typename T, typename PARAMS_T = T, typename ACC_T = float>
class AdaptiveOptimizer : public ObjectWithMutableHyperparams {
public:
  using DataVec  = Eigen::Matrix<T,       Eigen::Dynamic, 1>;
  using AccVec   = Eigen::Matrix<ACC_T,   Eigen::Dynamic, 1>;
  using ParamVec = Eigen::Matrix<PARAMS_T,Eigen::Dynamic, 1>;

  virtual ~AdaptiveOptimizer() = default;

  virtual void allocate(const std::size_t n_weights) = 0;

  virtual void 
  allocate(const std::shared_ptr<ParametricObject<PARAMS_T>>& target) {
    allocate(static_cast<std::size_t>(target->n_params()));
  }

  virtual void reset() = 0;

  virtual std::size_t get_n_iterations() const = 0;
  virtual std::size_t get_n_weights() const = 0;

  virtual ACC_T get_step_size() const = 0;
  virtual void set_step_size(const ACC_T mu) = 0;

  // Core adaptive-filter update: consume one sample/regressor.
  // weights_fp32: master weights (ACC_T precision)
  // weights_q: optional mirror (PARAMS_T) used for inference / export
  virtual void step_update (
    const AFStepState<T>& s,
    Eigen::Map<AccVec> weights_fp32,
    Eigen::Map<ParamVec>* weights_q = nullptr
  ) = 0;

  virtual json serialize() const { return {}; }
  virtual void deserialize(const json&) {}
};

template <typename T, typename PARAMS_T = T, typename ACC_T = float>
class LMSOptimizer : public AdaptiveOptimizer<T, PARAMS_T, ACC_T> {
public:
  using Base     = AdaptiveOptimizer<T, PARAMS_T, ACC_T>;
  using AccVec   = typename Base::AccVec;
  using ParamVec = typename Base::ParamVec;

  void allocate(const std::size_t n_ws) override {
    n_weights = n_ws;
    n_iters = 0;
  }

  void reset() override { n_iters = 0; }

  std::size_t get_n_iterations() const override { return n_iters; }
  std::size_t get_n_weights() const override { return n_weights; }

  ACC_T get_step_size() const override { return mu; }
  void set_step_size(const ACC_T m) override { mu = m; }

  void step_update(
    const AFStepState<T>& s,
    Eigen::Ref<AccVec> w_acc,
    Eigen::Ref<ParamVec>* w_q = nullptr
  ) override {

    assert(static_cast<std::size_t>(w_acc.size()) == n_weights);
    assert(s.x.size() == w_acc.size());

    ACC_T y;
    ACC_T e;

    if constexpr (std::is_same_v<T, ACC_T>) {
      // No cast path
      y = w_acc.dot(s.x);
      e = static_cast<ACC_T>(s.d) - y;

      if constexpr (Eigen::NumTraits<ACC_T>::IsComplex) {
        // Complex LMS (Diniz)
        w_acc.noalias() += mu * s.x * std::conj(e);
      } else {
        // Real LMS (Diniz)
        w_acc.noalias() += (ACC_T(2) * mu * e) * s.x;
      }

    } else {
      // Cast path

      Eigen::Matrix<ACC_T, Eigen::Dynamic, 1> x_acc =
          s.x.template cast<ACC_T>();

      y = w_acc.dot(x_acc);
      e = static_cast<ACC_T>(s.d) - y;

      if constexpr (Eigen::NumTraits<ACC_T>::IsComplex) {
        w_acc.noalias() += mu * x_acc * std::conj(e);
      } else {
        w_acc.noalias() += (ACC_T(2) * mu * e) * x_acc;
      }
    }

    if (w_q) {
      (*w_q) = w_acc.template cast<PARAMS_T>();
    }

    ++n_iters;
  }

  void update_hyperparams(const json& params) override;
  json hyperparams() const override;

private:
  std::size_t n_weights = 0;
  std::size_t n_iters = 0;
  ACC_T mu = ACC_T(1e-2);
};

template <typename T>
AdaptiveOptimizer<T>* 
create_optimizer(const json& af_params);

}