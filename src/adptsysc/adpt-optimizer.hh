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
  using DataVec    = Eigen::Matrix<ACC_T, Eigen::Dynamic, 1>;
  using DataMatrix = Eigen::Matrix<ACC_T, Eigen::Dynamic, Eigen::Dynamic>;
  using AccVec     = Eigen::Matrix<ACC_T, Eigen::Dynamic, 1>;
  using ParamVec   = Eigen::Matrix<PARAMS_T, Eigen::Dynamic, 1>;

  virtual ~AdaptiveOptimizer() = default;

  virtual void allocate(const std::size_t n_weights) = 0;

  virtual void allocate(const std::shared_ptr<ParametricObject<PARAMS_T>>& target) {
    allocate(static_cast<std::size_t>(target->n_params()));
  }

  virtual void reset() = 0;

  virtual std::size_t get_n_iterations() const = 0;
  virtual std::size_t get_n_weights() const = 0;

  virtual ACC_T get_step_size() const = 0;
  virtual void set_step_size(const ACC_T mu) = 0;

  // Sample-based update.
  virtual void step_update(
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
  using DataMatrix  = typename Base::DataMatrix;

  struct AnalysisInfo {
    bool   is_stable;
    ACC_T  mu_max;
    ACC_T  mu_trace;
    ACC_T  theor_misadj;
    ACC_T  mu_conservative;
    ACC_T  mu_aggressive;
  };

  AnalysisInfo analyze(const DataMatrix* R = nullptr) const;
  
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
    auto update = [&](const AccVec& x_vec) {
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
      update(s.x);
    } else {
      AccVec x_acc = s.x.template cast<ACC_T>();
      update(x_acc);
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
class SignSignOptimizer : public AdaptiveOptimizer<T, PARAMS_T, ACC_T> {
public:
  using Base     = AdaptiveOptimizer<T, PARAMS_T, ACC_T>;
  using DataVec  = typename Base::DataVec;
  using AccVec   = typename Base::AccVec;
  using ParamVec = typename Base::ParamVec;

  SignSignOptimizer(const json& params) {
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

    auto update = [&](const AccVec& x_vec) {
      const ACC_T y = w_acc.dot(x_vec);
      const ACC_T e = static_cast<ACC_T>(s.d) - y;

      const ACC_T se = get_scalar_sign(e, eps);
      const AccVec sx = get_vector_sign(x_vec, eps);

      w_acc.noalias() += (ACC_T(2) * mu * se) * sx;
    };

    if constexpr (std::is_same_v<T, ACC_T>) {
      update(s.x);
    } else {
      AccVec x_acc = s.x.template cast<ACC_T>();
      update(x_acc);
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
      {"otype", "sign_sign"},
      {"mu", mu},
      {"eps", eps}
    };
  }

private:
  std::size_t n_weights = 0;
  std::size_t n_iters   = 0;

  ACC_T mu = ACC_T(1e-2);
  double eps = 1e-12;
};

template <typename T, typename PARAMS_T = T, typename ACC_T = float>
class DualSignOptimizer : public AdaptiveOptimizer<T, PARAMS_T, ACC_T> {
public:
  using Base     = AdaptiveOptimizer<T, PARAMS_T, ACC_T>;
  using DataVec  = typename Base::DataVec;
  using AccVec   = typename Base::AccVec;
  using ParamVec = typename Base::ParamVec;
  using RealT    = typename Eigen::NumTraits<ACC_T>::Real;

  DualSignOptimizer(const json& params) { update_hyperparams(params); }

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
    auto update = [&](const AccVec& x_vec) {
      const ACC_T y  = w_acc.dot(x_vec);
      const ACC_T e  = static_cast<ACC_T>(s.d) - y;
      const ACC_T se = get_scalar_sign(e, eps);
      const ACC_T g  = (std::abs(e) > rho) ? ACC_T(epsilon_gain) : ACC_T(1);

      w_acc.noalias() += (ACC_T(2) * mu * g * se) * x_vec;
    };

    if constexpr (std::is_same_v<T, ACC_T>) {
      update(s.x);
    } else {
      AccVec x_acc = s.x.template cast<ACC_T>();
      update(x_acc);
    }

    if (w_q) {
      (*w_q) = w_acc.template cast<PARAMS_T>();
    }
    ++n_iters;
  }

  void update_hyperparams(const json& params) override {
    if (params.contains("mu"))      mu = params.at("mu").template get<ACC_T>();
    if (params.contains("rho"))     rho = params.at("rho").template get<RealT>();
    if (params.contains("epsilon")) epsilon_gain = params.at("epsilon").template get<RealT>();
    if (params.contains("eps"))     eps = params.at("eps").template get<double>();
  }

  json get_hyperparams() const override {
    return {
      {"otype", "dual_sign"},
      {"mu", mu},
      {"rho", rho},
      {"epsilon", epsilon_gain},
      {"eps", eps}
    };
  }

private:
  std::size_t n_weights = 0;
  std::size_t n_iters   = 0;
  ACC_T mu = ACC_T(1e-2);
  RealT rho = RealT(1);
  RealT epsilon_gain = RealT(2);
  double eps = 1e-12;
};

template <typename T, typename PARAMS_T = T, typename ACC_T = float>
class PowerOfTwoErrorOptimizer : public AdaptiveOptimizer<T, PARAMS_T, ACC_T> {
public:
  using Base     = AdaptiveOptimizer<T, PARAMS_T, ACC_T>;
  using DataVec  = typename Base::DataVec;
  using AccVec   = typename Base::AccVec;
  using ParamVec = typename Base::ParamVec;
  using RealT    = typename Eigen::NumTraits<ACC_T>::Real;

  PowerOfTwoErrorOptimizer(const json& params) { update_hyperparams(params); }

  void allocate(const std::size_t n_ws) override {
    n_weights = n_ws;
    n_iters = 0;
  }

  void reset() override { n_iters = 0; }

  std::size_t get_n_iterations() const override { return n_iters; }
  std::size_t get_n_weights() const override { return n_weights; }
  ACC_T get_step_size() const override { return mu; }
  void set_step_size(const ACC_T m) override { mu = m; }

  ACC_T p2e(const ACC_T& e) const {
    const RealT abs_e  = std::abs(e);
    const ACC_T s      = get_scalar_sign(e, eps);
    const RealT thresh = std::pow(RealT(2), -static_cast<RealT>(bd - 1));

    if (abs_e >= RealT(1)) {
      return s;
    } else if (abs_e >= thresh) {
      const RealT pow_mag = std::exp2(std::floor(std::log2(abs_e + RealT(eps))));
      return static_cast<ACC_T>(pow_mag) * s;
    } else {
      return static_cast<ACC_T>(tau_floor) * s;
    }
  }

  void step_update(
    const AFStepState<T>& s,
    Eigen::Ref<AccVec> w_acc,
    Eigen::Ref<ParamVec>* w_q = nullptr
  ) override {
    auto update = [&](const AccVec& x_vec) {
      const ACC_T y  = w_acc.dot(x_vec);
      const ACC_T e  = static_cast<ACC_T>(s.d) - y;
      const ACC_T pe = p2e(e);

      w_acc.noalias() += (ACC_T(2) * mu * pe) * x_vec;
    };

    if constexpr (std::is_same_v<T, ACC_T>) {
      update(s.x);
    } else {
      AccVec x_acc = s.x.template cast<ACC_T>();
      update(x_acc);
    }

    if (w_q) {
      (*w_q) = w_acc.template cast<PARAMS_T>();
    }
    ++n_iters;
  }

  void update_hyperparams(const json& params) override {
    if (params.contains("mu"))  mu = params.at("mu").template get<ACC_T>();
    if (params.contains("bd"))  bd = params.at("bd").get<int>();
    if (params.contains("tau")) tau_floor = params.at("tau").template get<RealT>();
    if (params.contains("eps")) eps = params.at("eps").template get<double>();
  }

  json get_hyperparams() const override {
    return {
      {"otype", "power_of_two_error"},
      {"mu", mu},
      {"bd", bd},
      {"tau", tau_floor},
      {"eps", eps}
    };
  }

private:
  std::size_t n_weights = 0;
  std::size_t n_iters   = 0;
  ACC_T mu = ACC_T(1e-2);
  int bd = 8;
  RealT tau_floor = RealT(0);
  double eps = 1e-12;
};

// ------------------------------------------------------------
// LMS-Newton
// ------------------------------------------------------------
template <typename T, typename PARAMS_T = T, typename ACC_T = float>
class LMSNewtonOptimizer : public AdaptiveOptimizer<T, PARAMS_T, ACC_T> {
public:
  using Base       = AdaptiveOptimizer<T, PARAMS_T, ACC_T>;
  using DataVec    = typename Base::DataVec;
  using AccVec     = typename Base::AccVec;
  using ParamVec   = typename Base::ParamVec;
  using DataMatrix = Eigen::Matrix<ACC_T, Eigen::Dynamic, Eigen::Dynamic>;

  LMSNewtonOptimizer(const json& params) { 
    update_hyperparams(params); 
  }

  void allocate(const std::size_t n_ws) override {
    n_weights = n_ws;
    n_iters = 0;
    R_hat_inv = (ACC_T(1) / delta) * DataMatrix::Identity(n_ws, n_ws);
  }

  void reset() override {
    n_iters = 0;
    R_hat_inv = (ACC_T(1) / delta) * DataMatrix::Identity(n_weights, n_weights);
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
    auto update = [&](const AccVec& x_vec) {
      const ACC_T y = w_acc.dot(x_vec);
      const ACC_T e = static_cast<ACC_T>(s.d) - y;

      const AccVec p   = R_hat_inv * x_vec;
      const ACC_T phi  = x_vec.dot(p);
      ACC_T denom = static_cast<ACC_T>((ACC_T(1) - alpha) / alpha) + phi;

      if (std::abs(denom) < eps) {
        denom += ACC_T(eps);
      }

      R_hat_inv = (R_hat_inv - (p * p.adjoint()) / denom) / (ACC_T(1) - alpha);

      if constexpr (Eigen::NumTraits<ACC_T>::IsComplex) {
        w_acc.noalias() += ACC_T(2) * mu * std::conj(e) * (R_hat_inv * x_vec);
      } else {
        w_acc.noalias() += ACC_T(2) * mu * e * (R_hat_inv * x_vec);
      }
    };

    if constexpr (std::is_same_v<T, ACC_T>) {
      update(s.x);
    } else {
      AccVec x_acc = s.x.template cast<ACC_T>();
      update(x_acc);
    }

    if (w_q) {
      (*w_q) = w_acc.template cast<PARAMS_T>();
    }
    ++n_iters;
  }

  void update_hyperparams(const json& params) override {
    if (params.contains("mu"))    mu    = params.at("mu").template get<ACC_T>();
    if (params.contains("alpha")) alpha = params.at("alpha").template get<ACC_T>();
    if (params.contains("delta")) delta = params.at("delta").template get<ACC_T>();
    if (params.contains("eps"))   eps   = params.at("eps").template get<double>();
  }

  json get_hyperparams() const override {
    return {
      {"otype", "lms_newton"},
      {"mu", mu},
      {"alpha", alpha},
      {"delta", delta},
      {"eps", eps}
    };
  }

private:
  std::size_t n_weights = 0;
  std::size_t n_iters   = 0;
  ACC_T mu    = ACC_T(1e-2);
  ACC_T alpha = ACC_T(0.99);
  ACC_T delta = ACC_T(1e-2);
  double eps  = 1e-12;
  DataMatrix R_hat_inv;
};

template <typename T, typename PARAMS_T = T, typename ACC_T = float>
class RLSOptimizer : public AdaptiveOptimizer<T, PARAMS_T, ACC_T> {
public:
  using Base       = AdaptiveOptimizer<T, PARAMS_T, ACC_T>;
  using DataVec    = typename Base::DataVec;
  using AccVec     = typename Base::AccVec;
  using ParamVec   = typename Base::ParamVec;
  using DataMatrix  = typename Base::DataMatrix;

  struct AnalysisInfo {
    bool   is_stable;
    ACC_T  eff_mem;
    ACC_T  theor_misadj;
    ACC_T  P_cond_num;
    ACC_T  lam_slow;
    ACC_T  lam_fast;
  };

  AnalysisInfo analyze(const DataMatrix* R = nullptr) const;

  RLSOptimizer(const json& params) { 
    update_hyperparams(params); 
  }

  void allocate(const std::size_t n_ws) override {
    n_weights = n_ws;
    n_iters = 0;
    S_D = (ACC_T(1) / delta) * DataMatrix::Identity(n_ws, n_ws);
  }

  void reset() override {
    n_iters = 0;
    S_D = (ACC_T(1) / delta) * DataMatrix::Identity(n_weights, n_weights);
  }

  std::size_t get_n_iterations() const override { return n_iters; }
  std::size_t get_n_weights() const override { return n_weights; }
  ACC_T get_step_size() const override { return lambda; }
  void set_step_size(const ACC_T lam) override { lambda = lam; }

  void step_update (
    const AFStepState<T>& s,
    Eigen::Ref<AccVec> w_acc,
    Eigen::Ref<ParamVec>* w_q = nullptr
  ) override {
    auto update = [&](const AccVec& x_vec) {
      const ACC_T y = w_acc.dot(x_vec);
      const ACC_T e = static_cast<ACC_T>(s.d) - y;

      const AccVec num = S_D * x_vec;
      ACC_T denom = lambda + x_vec.dot(num);

      if (std::abs(denom) < eps) {
        denom += ACC_T(eps);
      }

      const AccVec k = num / denom;

      if constexpr (Eigen::NumTraits<ACC_T>::IsComplex) {
        w_acc.noalias() += std::conj(e) * k;
      } else {
        w_acc.noalias() += e * k;
      }

      S_D = (S_D - k * (x_vec.adjoint() * S_D)) / lambda;
    };

    if constexpr (std::is_same_v<T, ACC_T>) {
      update(s.x);
    } else {
      AccVec x_acc = s.x.template cast<ACC_T>();
      update(x_acc);
    }

    if (w_q) {
      (*w_q) = w_acc.template cast<PARAMS_T>();
    }
    ++n_iters;
  }

  void update_hyperparams(const json& params) override {
    if (params.contains("lambda")) lambda = params.at("lambda").template get<ACC_T>();
    if (params.contains("lam"))    lambda = params.at("lam").template get<ACC_T>();
    if (params.contains("delta"))  delta  = params.at("delta").template get<ACC_T>();
    if (params.contains("eps"))    eps    = params.at("eps").template get<double>();
  }

  json get_hyperparams() const override {
    return {
      {"otype", "rls"},
      {"lambda", lambda},
      {"delta", delta},
      {"eps", eps}
    };
  }

private:
  std::size_t n_weights = 0;
  std::size_t n_iters   = 0;
  ACC_T lambda = ACC_T(0.99);
  ACC_T delta  = ACC_T(1e-2);
  double eps   = 1e-12;
  DataMatrix S_D;
};

template <typename T, typename PARAMS_T = T, typename ACC_T = float>
class NLMSOptimizer : public AdaptiveOptimizer<T, PARAMS_T, ACC_T> {
public:
  using Base     = AdaptiveOptimizer<T, PARAMS_T, ACC_T>;
  using DataVec  = typename Base::DataVec;
  using AccVec   = typename Base::AccVec;
  using ParamVec = typename Base::ParamVec;
  using RealT    = typename Eigen::NumTraits<ACC_T>::Real;

  NLMSOptimizer(const json& params) { 
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

template <typename T, typename PARAMS_T = T, typename ACC_T = float>
class FDAFOptimizer : public AdaptiveOptimizer<T, PARAMS_T, ACC_T> {
public:
  using Base     = AdaptiveOptimizer<T, PARAMS_T, ACC_T>;
  using AccVec   = typename Base::AccVec;
  using ParamVec = typename Base::ParamVec;
  using CxT      = std::complex<ACC_T>;

  FDAFOptimizer(const json& params) {
    update_hyperparams(params);
  }

  void allocate(const std::size_t n_ws) override {
    M = n_ws;
    N = 2 * M;
    n_iters = 0;

    w_freq.assign(N, CxT(0, 0));
    pow_est.assign(N, eps);
  }

  void reset() override {
    std::fill(w_freq.begin(), w_freq.end(), CxT(0, 0));
    std::fill(pow_est.begin(), pow_est.end(), eps);
    n_iters = 0;
  }

  std::size_t get_n_iterations() const override { return n_iters; }
  std::size_t get_n_weights() const override { return M; }

  ACC_T get_step_size() const override { return mu; }
  void set_step_size(ACC_T m) override { mu = m; }

  // FDAF is not sample-based.
  void step_update(
    const AFStepState<T>& s,
    Eigen::Ref<AccVec> w_acc,
    Eigen::Ref<ParamVec>* w_q = nullptr
  ) override {
    throw std::runtime_error("FDAFOptimizer does not support sample-based step_update()");
  }

  // FDAF-specific block update.
  void step_update_block(
    const std::vector<CxT>& x_freq,
    const Eigen::Ref<const Eigen::Matrix<ACC_T, Eigen::Dynamic, 1>>& e_block
  ) {
    if (M == 0 || N == 0) {
      throw std::runtime_error("FDAFOptimizer must be allocated before use");
    }
    if (x_freq.size() != N) {
      throw std::invalid_argument("x_freq has wrong size");
    }
    if (static_cast<std::size_t>(e_block.size()) != M) {
      throw std::invalid_argument("e_block has wrong size");
    }

    Eigen::Matrix<ACC_T, Eigen::Dynamic, 1> e_pad(N);
    e_pad.setZero();
    e_pad.tail(M) = e_block;

    auto e_freq = fft(e_pad);

    for (std::size_t i = 0; i < N; ++i) {
      pow_est[i] = alpha * pow_est[i] + (ACC_T(1) - alpha) * std::norm(x_freq[i]);
      e_freq[i] /= (pow_est[i] + eps);
    }

    std::vector<CxT> grad_freq(N);
    for (std::size_t i = 0; i < N; ++i) {
      grad_freq[i] = std::conj(x_freq[i]) * e_freq[i];
    }

    auto grad_time = ifft(grad_freq);
    for (std::size_t i = M; i < N; ++i) {
      grad_time[i] = CxT(0, 0);
    }

    grad_freq = fft(grad_time);

    for (std::size_t i = 0; i < N; ++i) {
      w_freq[i] += mu * grad_freq[i];
    }

    ++n_iters;
  }

  const std::vector<CxT>& get_weight_freq() const {
    return w_freq;
  }

  void update_hyperparams(const json& params) override {
    if (params.contains("mu")) {
      mu = static_cast<ACC_T>(params["mu"].template get<double>());
    }
    if (params.contains("alpha")) {
      alpha = static_cast<ACC_T>(params["alpha"].template get<double>());
    }
    if (params.contains("eps")) {
      eps = static_cast<ACC_T>(params["eps"].template get<double>());
    }
  }

  json get_hyperparams() const override {
    return {
      {"otype", "fdaf_optimizer"},
      {"mu", mu},
      {"alpha", alpha},
      {"eps", eps}
    };
  }

private:
  std::size_t M = 0;
  std::size_t N = 0;
  std::size_t n_iters = 0;

  ACC_T mu    = ACC_T(1e-2);
  ACC_T alpha = ACC_T(0.9);
  ACC_T eps   = ACC_T(1e-8);

  std::vector<CxT> w_freq;
  std::vector<ACC_T> pow_est;

  std::vector<CxT> fft(const Eigen::Matrix<ACC_T, Eigen::Dynamic, 1>& x) const;
  std::vector<CxT> fft(const std::vector<CxT>& x) const;
  std::vector<CxT> ifft(const std::vector<CxT>& X) const;
};

template <typename T, typename PARAMS_T, typename ACC_T>
AdaptiveOptimizer<T, PARAMS_T, ACC_T>*
create_optimizer(const json& af_params);

}  // namespace adptsysc