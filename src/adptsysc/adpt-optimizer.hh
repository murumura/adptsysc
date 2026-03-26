#pragma once

#include <adptsysc/object.hh>
#include <ostream>
#include <string>
#include <type_traits>
#include <Eigen/Dense>

namespace adptsysc {

std::string to_lower(const std::string str);
std::string to_upper(const std::string str);
bool eq_nocase(const std::string& s1, const std::string& s2);

template <typename T>
struct AFStepState {
  using DataVec = Eigen::Matrix<T, Eigen::Dynamic, 1>;
  Eigen::Ref<const DataVec> x;   // regressor
  T d;                           // desired signal
};

template <typename T>
T get_scalar_sign(const T& x, double eps = 1e-12) {
  using RealT = typename Eigen::NumTraits<T>::Real;
  const RealT mag = std::abs(x);
  if (mag <= static_cast<RealT>(eps)) {
    return T(0);
  }

  if constexpr (Eigen::NumTraits<T>::IsComplex) {
    return std::conj(x) / mag;   // ⭐ Diniz correct
  } else {
    return (x > RealT(0)) ? T(1) : T(-1);
  }
}

template <typename Vec>
Vec get_vector_sign(const Vec& x, double eps = 1e-12) {
  using Scalar = typename Vec::Scalar;
  using RealT  = typename Eigen::NumTraits<Scalar>::Real;

  Vec out(x.size());

  for (int i = 0; i < x.size(); ++i) {
    const RealT mag = std::abs(x[i]);

    if (mag <= static_cast<RealT>(eps)) {
      out[i] = Scalar(0);
    } else {
      if constexpr (Eigen::NumTraits<Scalar>::IsComplex) {
        out[i] = x[i] / mag; 
      } else {
        out[i] = (x[i] > RealT(0)) ? Scalar(1) : Scalar(-1);
      }
    }
  }
  return out;
}

template <typename T, typename PARAMS_T = T, typename ACC_T = float>
class AdaptiveOptimizer : public ObjectWithMutableHyperparams {
public:
  using DataVec     = Eigen::Matrix<ACC_T, Eigen::Dynamic, 1>;
  using DataMatrix  = Eigen::Matrix<ACC_T, Eigen::Dynamic, Eigen::Dynamic>;
  using AccVec      = Eigen::Matrix<ACC_T, Eigen::Dynamic, 1>;
  using ParamVec    = Eigen::Matrix<PARAMS_T, Eigen::Dynamic, 1>;
  
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
    Eigen::Ref<AccVec> weights_fp32,
    Eigen::Ref<ParamVec>* weights_q = nullptr
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
  using DataVec  = typename Base::DataVec;
  
  LMSOptimizer(const json& params) {
    update_hyperparams(params);
  }
  
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
      y = w_acc.dot(s.x);
      e = static_cast<ACC_T>(s.d) - y;

      if constexpr (Eigen::NumTraits<ACC_T>::IsComplex) {
        // Complex LMS
        w_acc.noalias() += mu * s.x * std::conj(e);
      } else {
        w_acc.noalias() += (ACC_T(2) * mu * e) * s.x;
      }

    } else {
      DataVec x_acc = s.x.template cast<ACC_T>();

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
  json get_hyperparams() const override;

private:
  std::size_t n_weights = 0;
  std::size_t n_iters = 0;
  ACC_T mu = ACC_T(1e-2);
};


template <typename T, typename PARAMS_T = T, typename ACC_T = float>
class APAOptimizer : public AdaptiveOptimizer<T, PARAMS_T, ACC_T> {
public:
  using Base       = AdaptiveOptimizer<T, PARAMS_T, ACC_T>;
  using DataVec    = typename Base::DataVec;
  using DataMatrix = typename Base::DataMatrix;
  using AccVec     = typename Base::AccVec;
  using ParamVec   = typename Base::ParamVec;
  
  APAOptimizer(const json& params) {
    update_hyperparams(params);
  }

  void allocate(const std::size_t n_ws) override {
    n_weights = n_ws;
    n_iters = 0;

    X_hist = DataMatrix::Zero(n_ws, P+1);
    d_hist = DataVec::Zero(P+1);
  }

  void reset() override {
    n_iters = 0;
    X_hist.setZero();
    d_hist.setZero();
  }

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

    // update history buffers
    X_hist.rightCols(P) = X_hist.leftCols(P);
    d_hist.tail(P) = d_hist.head(P);

    X_hist.col(0) = s.x.template cast<ACC_T>();
    d_hist(0) = static_cast<ACC_T>(s.d);

    // build matrices
    DataVec y = X_hist.transpose() * w_acc;
    DataVec e = d_hist - y;

    DataMatrix R = X_hist.transpose() * X_hist;

    R.diagonal().array() += gamma;

    // solve projection
    DataVec g = R.ldlt().solve(e);

    w_acc.noalias() += mu * X_hist * g;

    if (w_q) {
      (*w_q) = w_acc.template cast<PARAMS_T>();
    }

    ++n_iters;
  }

  void update_hyperparams(const json& params) override;
  json get_hyperparams() const override;

private:

  std::size_t n_weights = 0;
  std::size_t n_iters = 0;

  ACC_T mu = ACC_T(0.1);
  ACC_T gamma = ACC_T(1e-6);
  std::size_t P = 1;

  DataMatrix X_hist;
  DataVec d_hist;
};

template <typename T, typename PARAMS_T = T, typename ACC_T = float>
class SignErrorOptimizer : public AdaptiveOptimizer<T, PARAMS_T, ACC_T> {
public:
  using Base     = AdaptiveOptimizer<T, PARAMS_T, ACC_T>;
  using DataVec  = typename Base::DataVec;
  using AccVec   = typename Base::AccVec;
  using ParamVec = typename Base::ParamVec;

  SignErrorOptimizer(const json& params) { 
    update_hyperparams(params); 
  }

  void allocate(const std::size_t n_ws) override { 
    n_weights = n_ws; 
    n_iters = 0; 
  }

  void reset() override { n_iters = 0; }

  std::size_t get_n_iterations() const override { return n_iters; }
  std::size_t get_n_weights() const override { return n_weights; }
  ACC_T get_step_size() const override { return mu; }
  void set_step_size(const ACC_T m) override { mu = m; }

  void step_update (
    const AFStepState<T>& s,
    Eigen::Ref<AccVec> w_acc,
    Eigen::Ref<ParamVec>* w_q = nullptr
  ) override {

    ACC_T y, e, se;

    if constexpr (std::is_same_v<T, ACC_T>) {
      y  = w_acc.dot(s.x);
      e  = static_cast<ACC_T>(s.d) - y;
      se = get_scalar_sign(e, eps);

      w_acc.noalias() += (ACC_T(2) * mu * se) * s.x;

    } else {
      AccVec x = s.x.template cast<ACC_T>();

      y  = w_acc.dot(x);
      e  = static_cast<ACC_T>(s.d) - y;
      se = get_scalar_sign(e, eps);

      w_acc.noalias() += (ACC_T(2) * mu * se) * x;
    }

    if (w_q) {
      (*w_q) = w_acc.template cast<PARAMS_T>();
    }

    ++n_iters;
  }

  void update_hyperparams(const json& params) override {
    if (params.contains("mu")) 
      mu = params.at("mu").template get<ACC_T>();
    if (params.contains("eps")) 
      eps = params.at("eps").template get<double>();
  }

  json get_hyperparams() const override {
    return {
      {"otype", "sign_error"}, 
      {"mu", mu}, 
      {"eps", eps}
    };
  }

private:
  std::size_t n_weights = 0, n_iters = 0;
  ACC_T mu = ACC_T(1e-2);
  double eps = 1e-12;
};

template <typename T, typename PARAMS_T = T, typename ACC_T = float>
class SignDataOptimizer : public AdaptiveOptimizer<T, PARAMS_T, ACC_T> {
public:
  using Base     = AdaptiveOptimizer<T, PARAMS_T, ACC_T>;
  using DataVec  = typename Base::DataVec;
  using AccVec   = typename Base::AccVec;
  using ParamVec = typename Base::ParamVec;

  SignDataOptimizer(const json& params) { 
    update_hyperparams(params); 
  }

  void allocate(const std::size_t n_ws) override { 
    n_weights = n_ws; 
    n_iters = 0; 
  }
  
  void reset() override { 
    n_iters = 0; 
  }

  std::size_t get_n_iterations() const override { return n_iters; }
  std::size_t get_n_weights() const override { return n_weights; }
  ACC_T get_step_size() const override { return mu; }
  void set_step_size(const ACC_T m) override { mu = m; }

  /**
   * @brief Performs the Sign-Data LMS update.
   * Update Rule: w(n+1) = w(n) + 2 * mu * e(n) * sign(x(n))
   */
  void step_update(
    const AFStepState<T>& s,
    Eigen::Ref<AccVec> w_acc,
    Eigen::Ref<ParamVec>* w_q = nullptr
  ) override {

    auto perform_update = [&](const AccVec& x_vec) {
      const ACC_T y = w_acc.dot(x_vec);
      const ACC_T e = static_cast<ACC_T>(s.d) - y;

      auto sx = get_vector_sign(x_vec, eps);

      if constexpr (Eigen::NumTraits<ACC_T>::IsComplex) {
        w_acc.noalias() += (ACC_T(2) * mu * std::conj(e)) * sx;
      } else {
        w_acc.noalias() += (ACC_T(2) * mu * e) * sx;
      }
    };

    if constexpr (std::is_same_v<T, ACC_T>) {
      perform_update(s.x);
    } else {
      AccVec x_acc = s.x.template cast<ACC_T>();
      perform_update(x_acc);
    }

    if (w_q) {
      (*w_q) = w_acc.template cast<PARAMS_T>();
    }

    ++n_iters;
  }

  void update_hyperparams(const json& params) override {
    if (params.contains("mu")) 
      mu = params.at("mu").template get<ACC_T>();
    if (params.contains("eps")) 
      eps = params.at("eps").template get<double>();
  }

  json get_hyperparams() const override {
    return {
      {"otype", "sign_data"}, 
      {"mu", mu}, 
      {"eps", eps}
    };
  }

private:
  std::size_t n_weights = 0, n_iters = 0;
  ACC_T mu = ACC_T(1e-2);
  double eps = 1e-12; // Small epsilon to prevent sign ambiguity/division by zero
};

template <typename T, typename PARAMS_T = T, typename ACC_T = float>
class NLMSOptimizer : public AdaptiveOptimizer<T, PARAMS_T, ACC_T> {
public:
  using Base     = AdaptiveOptimizer<T, PARAMS_T, ACC_T>;
  using DataVec  = typename Base::DataVec;
  using AccVec   = typename Base::AccVec;
  using ParamVec = typename Base::ParamVec;
  using RealT    = typename Eigen::NumTraits<ACC_T>::Real;

  NLMSOptimizer(const json& params) { update_hyperparams(params); }

  void allocate(const std::size_t n_ws) override {
    n_weights = n_ws;
    n_iters = 0;
  }

  void reset() override { n_iters = 0; }

  std::size_t get_n_iterations() const override { return n_iters; }
  std::size_t get_n_weights() const override { return n_weights; }

  ACC_T get_step_size() const override { return mu; }
  void set_step_size(const ACC_T m) override { mu = m; }

  void step_update (
    const AFStepState<T>& s,
    Eigen::Ref<AccVec> w_acc,
    Eigen::Ref<ParamVec>* w_q = nullptr
  ) override {

    if constexpr (std::is_same_v<T, ACC_T>) {
      const auto& x = s.x; // No cast needed
      const ACC_T y = w_acc.dot(x);
      const ACC_T e = static_cast<ACC_T>(s.d) - y;
      const RealT norm2 = x.squaredNorm();
      
      // NLMS normalization factor
      const ACC_T mu_k = mu / static_cast<ACC_T>(tau + norm2);

      // Correct Complex vs Real logic using NumTraits
      if constexpr (Eigen::NumTraits<ACC_T>::IsComplex) {
          w_acc.noalias() += mu_k * std::conj(e) * x;
      } else {
          w_acc.noalias() += mu_k * e * x;
      }
    } else {
      // Cast input once if types differ
      DataVec x_acc = s.x.template cast<ACC_T>();
      const ACC_T y = w_acc.dot(x_acc);
      const ACC_T e = static_cast<ACC_T>(s.d) - y;
      const RealT norm2 = x_acc.squaredNorm();
      
      const ACC_T mu_k = mu / static_cast<ACC_T>(tau + norm2);

      if constexpr (Eigen::NumTraits<ACC_T>::IsComplex) {
          w_acc.noalias() += mu_k * std::conj(e) * x_acc;
      } else {
          w_acc.noalias() += mu_k * e * x_acc;
      }
    }

    // Quantize/copy weights if output pointer provided
    if (w_q) {
      (*w_q) = w_acc.template cast<PARAMS_T>();
    }
    ++n_iters;
  }

  void update_hyperparams(const json& params) override {
    if (params.contains("mu"))  
      mu  = params.at("mu").template get<ACC_T>();
    if (params.contains("tau")) 
      tau = params.at("tau").template get<RealT>();
  }

  json get_hyperparams() const override {
    return {
      {"otype", "nlms"},
      {"mu", mu},
      {"tau", tau}
    };
  }

private:
  std::size_t n_weights = 0;
  std::size_t n_iters   = 0;
  ACC_T mu = ACC_T(1);
  RealT tau = RealT(1e-3);
};

template <typename T, typename PARAMS_T, typename ACC_T>
AdaptiveOptimizer<T, PARAMS_T, ACC_T>*
create_optimizer(const json& af_params);

}  // namespace adptsysc