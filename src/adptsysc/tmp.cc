#pragma once

#include <adptsysc/object.hh>
#include <Eigen/Dense>
#include <complex>
#include <cmath>
#include <stdexcept>
#include <string>
#include <type_traits>

namespace adptsysc {

// ------------------------------------------------------------
// small helpers
// ------------------------------------------------------------
template <typename X>
inline X scalar_conj_if_needed(const X& v) {
  return v;
}

template <typename R>
inline std::complex<R> scalar_conj_if_needed(const std::complex<R>& v) {
  return std::conj(v);
}

template <typename X>
inline X sign_scalar(const X& v, double eps = 1e-12) {
  const auto mag = std::abs(v);
  if (mag <= eps) {
    return X(0);
  }
  return scalar_conj_if_needed(v) / static_cast<typename Eigen::NumTraits<X>::Real>(mag);
}

template <typename Vec>
inline Vec sign_vector(const Vec& x, double eps = 1e-12) {
  Vec out(x.size());
  for (Eigen::Index i = 0; i < x.size(); ++i) {
    using Scalar = typename Vec::Scalar;
    const auto mag = std::abs(x(i));
    out(i) = (mag <= eps) ? Scalar(0) : x(i) / static_cast<typename Eigen::NumTraits<Scalar>::Real>(mag);
  }
  return out;
}

template <typename T>
inline Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>
dct_ortho_matrix(const std::size_t M) {
  Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> Tm(M, M);
  const T pi = static_cast<T>(3.14159265358979323846);

  for (std::size_t n = 0; n < M; ++n) {
    for (std::size_t k = 0; k < M; ++k) {
      Tm(n, k) = std::cos(pi / static_cast<T>(M) * (static_cast<T>(n) + T(0.5)) * static_cast<T>(k));
    }
  }

  Tm.col(0) *= static_cast<T>(1.0 / std::sqrt(static_cast<double>(M)));
  if (M > 1) {
    Tm.rightCols(M - 1) *= static_cast<T>(std::sqrt(2.0 / static_cast<double>(M)));
  }
  return Tm;
}

template <typename T>
inline Eigen::Matrix<std::complex<T>, Eigen::Dynamic, Eigen::Dynamic>
dft_unitary_matrix(const std::size_t M) {
  using C = std::complex<T>;
  Eigen::Matrix<C, Eigen::Dynamic, Eigen::Dynamic> W(M, M);
  const T pi = static_cast<T>(3.14159265358979323846);

  for (std::size_t n = 0; n < M; ++n) {
    for (std::size_t k = 0; k < M; ++k) {
      T theta = static_cast<T>(-2.0) * pi * static_cast<T>(n * k) / static_cast<T>(M);
      W(n, k) = std::exp(C(0, theta));
    }
  }

  W /= static_cast<T>(std::sqrt(static_cast<double>(M)));
  return W;
}

// ------------------------------------------------------------
// NLMS
// ------------------------------------------------------------
template <typename T, typename PARAMS_T = T, typename ACC_T = float>
class NLMSOptimizer : public AdaptiveOptimizer<T, PARAMS_T, ACC_T> {
public:
  using Base     = AdaptiveOptimizer<T, PARAMS_T, ACC_T>;
  using DataVec  = typename Base::DataVec;
  using AccVec   = typename Base::AccVec;
  using ParamVec = typename Base::ParamVec;
  using RealT    = typename Eigen::NumTraits<ACC_T>::Real;

  explicit NLMSOptimizer(const json& params) { update_hyperparams(params); }

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
    DataVec x = s.x.template cast<ACC_T>();
    const ACC_T y = w_acc.dot(x);
    const ACC_T e = static_cast<ACC_T>(s.d) - y;
    const RealT norm2 = x.squaredNorm();
    const ACC_T mu_k = mu / static_cast<ACC_T>(tau + norm2);

    w_acc.noalias() += mu_k * scalar_conj_if_needed(e) * x;

    if (w_q) {
      (*w_q) = w_acc.template cast<PARAMS_T>();
    }
    ++n_iters;
  }

  void update_hyperparams(const json& params) override {
    if (params.contains("mu"))  mu  = params.at("mu").template get<ACC_T>();
    if (params.contains("tau")) tau = params.at("tau").template get<RealT>();
  }

  json hyperparams() const override {
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

// ------------------------------------------------------------
// Sign-Error LMS
// ------------------------------------------------------------
template <typename T, typename PARAMS_T = T, typename ACC_T = float>
class SignErrorOptimizer : public AdaptiveOptimizer<T, PARAMS_T, ACC_T> {
public:
  using Base     = AdaptiveOptimizer<T, PARAMS_T, ACC_T>;
  using DataVec  = typename Base::DataVec;
  using AccVec   = typename Base::AccVec;
  using ParamVec = typename Base::ParamVec;

  explicit SignErrorOptimizer(const json& params) { update_hyperparams(params); }

  void allocate(const std::size_t n_ws) override { n_weights = n_ws; n_iters = 0; }
  void reset() override { n_iters = 0; }

  std::size_t get_n_iterations() const override { return n_iters; }
  std::size_t get_n_weights() const override { return n_weights; }
  ACC_T get_step_size() const override { return mu; }
  void set_step_size(const ACC_T m) override { mu = m; }

  void step_update(const AFStepState<T>& s, Eigen::Ref<AccVec> w_acc,
                   Eigen::Ref<ParamVec>* w_q = nullptr) override {
    DataVec x = s.x.template cast<ACC_T>();
    const ACC_T y = w_acc.dot(x);
    const ACC_T e = static_cast<ACC_T>(s.d) - y;
    const ACC_T se = sign_scalar(e, eps);

    w_acc.noalias() += (ACC_T(2) * mu * se) * x;

    if (w_q) {
      (*w_q) = w_acc.template cast<PARAMS_T>();
    }
    ++n_iters;
  }

  void update_hyperparams(const json& params) override {
    if (params.contains("mu"))  mu  = params.at("mu").template get<ACC_T>();
    if (params.contains("eps")) eps = params.at("eps").template get<double>();
  }

  json hyperparams() const override {
    return {{"otype", "sign_error"}, {"mu", mu}, {"eps", eps}};
  }

private:
  std::size_t n_weights = 0, n_iters = 0;
  ACC_T mu = ACC_T(1e-2);
  double eps = 1e-12;
};

// ------------------------------------------------------------
// Sign-Data LMS
// ------------------------------------------------------------
template <typename T, typename PARAMS_T = T, typename ACC_T = float>
class SignDataOptimizer : public AdaptiveOptimizer<T, PARAMS_T, ACC_T> {
public:
  using Base     = AdaptiveOptimizer<T, PARAMS_T, ACC_T>;
  using DataVec  = typename Base::DataVec;
  using AccVec   = typename Base::AccVec;
  using ParamVec = typename Base::ParamVec;

  explicit SignDataOptimizer(const json& params) { update_hyperparams(params); }

  void allocate(const std::size_t n_ws) override { n_weights = n_ws; n_iters = 0; }
  void reset() override { n_iters = 0; }

  std::size_t get_n_iterations() const override { return n_iters; }
  std::size_t get_n_weights() const override { return n_weights; }
  ACC_T get_step_size() const override { return mu; }
  void set_step_size(const ACC_T m) override { mu = m; }

  void step_update(const AFStepState<T>& s, Eigen::Ref<AccVec> w_acc,
                   Eigen::Ref<ParamVec>* w_q = nullptr) override {
    DataVec x = s.x.template cast<ACC_T>();
    const ACC_T y = w_acc.dot(x);
    const ACC_T e = static_cast<ACC_T>(s.d) - y;
    const DataVec sx = sign_vector(x, eps);

    w_acc.noalias() += (ACC_T(2) * mu * scalar_conj_if_needed(e)) * sx;

    if (w_q) {
      (*w_q) = w_acc.template cast<PARAMS_T>();
    }
    ++n_iters;
  }

  void update_hyperparams(const json& params) override {
    if (params.contains("mu"))  mu  = params.at("mu").template get<ACC_T>();
    if (params.contains("eps")) eps = params.at("eps").template get<double>();
  }

  json hyperparams() const override {
    return {{"otype", "sign_data"}, {"mu", mu}, {"eps", eps}};
  }

private:
  std::size_t n_weights = 0, n_iters = 0;
  ACC_T mu = ACC_T(1e-2);
  double eps = 1e-12;
};

// ------------------------------------------------------------
// Sign-Sign LMS
// ------------------------------------------------------------
template <typename T, typename PARAMS_T = T, typename ACC_T = float>
class SignSignOptimizer : public AdaptiveOptimizer<T, PARAMS_T, ACC_T> {
public:
  using Base     = AdaptiveOptimizer<T, PARAMS_T, ACC_T>;
  using DataVec  = typename Base::DataVec;
  using AccVec   = typename Base::AccVec;
  using ParamVec = typename Base::ParamVec;

  explicit SignSignOptimizer(const json& params) { update_hyperparams(params); }

  void allocate(const std::size_t n_ws) override { n_weights = n_ws; n_iters = 0; }
  void reset() override { n_iters = 0; }

  std::size_t get_n_iterations() const override { return n_iters; }
  std::size_t get_n_weights() const override { return n_weights; }
  ACC_T get_step_size() const override { return mu; }
  void set_step_size(const ACC_T m) override { mu = m; }

  void step_update(const AFStepState<T>& s, Eigen::Ref<AccVec> w_acc,
                   Eigen::Ref<ParamVec>* w_q = nullptr) override {
    DataVec x = s.x.template cast<ACC_T>();
    const ACC_T y = w_acc.dot(x);
    const ACC_T e = static_cast<ACC_T>(s.d) - y;

    const ACC_T se = sign_scalar(e, eps);
    const DataVec sx = sign_vector(x, eps);

    w_acc.noalias() += (ACC_T(2) * mu * se) * sx;

    if (w_q) {
      (*w_q) = w_acc.template cast<PARAMS_T>();
    }
    ++n_iters;
  }

  void update_hyperparams(const json& params) override {
    if (params.contains("mu"))  mu  = params.at("mu").template get<ACC_T>();
    if (params.contains("eps")) eps = params.at("eps").template get<double>();
  }

  json hyperparams() const override {
    return {{"otype", "sign_sign"}, {"mu", mu}, {"eps", eps}};
  }

private:
  std::size_t n_weights = 0, n_iters = 0;
  ACC_T mu = ACC_T(1e-2);
  double eps = 1e-12;
};

// ------------------------------------------------------------
// Dual-Sign LMS
// ------------------------------------------------------------
template <typename T, typename PARAMS_T = T, typename ACC_T = float>
class DualSignOptimizer : public AdaptiveOptimizer<T, PARAMS_T, ACC_T> {
public:
  using Base     = AdaptiveOptimizer<T, PARAMS_T, ACC_T>;
  using DataVec  = typename Base::DataVec;
  using AccVec   = typename Base::AccVec;
  using ParamVec = typename Base::ParamVec;
  using RealT    = typename Eigen::NumTraits<ACC_T>::Real;

  explicit DualSignOptimizer(const json& params) { update_hyperparams(params); }

  void allocate(const std::size_t n_ws) override { n_weights = n_ws; n_iters = 0; }
  void reset() override { n_iters = 0; }

  std::size_t get_n_iterations() const override { return n_iters; }
  std::size_t get_n_weights() const override { return n_weights; }
  ACC_T get_step_size() const override { return mu; }
  void set_step_size(const ACC_T m) override { mu = m; }

  void step_update(const AFStepState<T>& s, Eigen::Ref<AccVec> w_acc,
                   Eigen::Ref<ParamVec>* w_q = nullptr) override {
    DataVec x = s.x.template cast<ACC_T>();
    const ACC_T y = w_acc.dot(x);
    const ACC_T e = static_cast<ACC_T>(s.d) - y;
    const ACC_T se = sign_scalar(e, eps);
    const ACC_T gain = (std::abs(e) > rho) ? ACC_T(epsilon_gain) : ACC_T(1);

    w_acc.noalias() += (ACC_T(2) * mu * gain * se) * x;

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

  json hyperparams() const override {
    return {
      {"otype", "dual_sign"},
      {"mu", mu},
      {"rho", rho},
      {"epsilon", epsilon_gain},
      {"eps", eps}
    };
  }

private:
  std::size_t n_weights = 0, n_iters = 0;
  ACC_T mu = ACC_T(1e-2);
  RealT rho = RealT(1);
  RealT epsilon_gain = RealT(2);
  double eps = 1e-12;
};

// ------------------------------------------------------------
// Power-of-Two Error LMS
// ------------------------------------------------------------
template <typename T, typename PARAMS_T = T, typename ACC_T = float>
class PowerOfTwoErrorOptimizer : public AdaptiveOptimizer<T, PARAMS_T, ACC_T> {
public:
  using Base     = AdaptiveOptimizer<T, PARAMS_T, ACC_T>;
  using DataVec  = typename Base::DataVec;
  using AccVec   = typename Base::AccVec;
  using ParamVec = typename Base::ParamVec;
  using RealT    = typename Eigen::NumTraits<ACC_T>::Real;

  explicit PowerOfTwoErrorOptimizer(const json& params) { update_hyperparams(params); }

  void allocate(const std::size_t n_ws) override { n_weights = n_ws; n_iters = 0; }
  void reset() override { n_iters = 0; }

  std::size_t get_n_iterations() const override { return n_iters; }
  std::size_t get_n_weights() const override { return n_weights; }
  ACC_T get_step_size() const override { return mu; }
  void set_step_size(const ACC_T m) override { mu = m; }

  ACC_T p2e(const ACC_T& e) const {
    const auto abs_e = std::abs(e);
    const auto s = sign_scalar(e, eps);
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

  void step_update(const AFStepState<T>& s, Eigen::Ref<AccVec> w_acc,
                   Eigen::Ref<ParamVec>* w_q = nullptr) override {
    DataVec x = s.x.template cast<ACC_T>();
    const ACC_T y = w_acc.dot(x);
    const ACC_T e = static_cast<ACC_T>(s.d) - y;
    const ACC_T pe = scalar_conj_if_needed(p2e(e));

    w_acc.noalias() += (ACC_T(2) * mu * pe) * x;

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

  json hyperparams() const override {
    return {
      {"otype", "power_of_two_error"},
      {"mu", mu},
      {"bd", bd},
      {"tau", tau_floor},
      {"eps", eps}
    };
  }

private:
  std::size_t n_weights = 0, n_iters = 0;
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
  using DataMatrix = typename Base::DataMatrix;
  using AccVec     = typename Base::AccVec;
  using ParamVec   = typename Base::ParamVec;

  explicit LMSNewtonOptimizer(const json& params) { update_hyperparams(params); }

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

  void step_update(const AFStepState<T>& s, Eigen::Ref<AccVec> w_acc,
                   Eigen::Ref<ParamVec>* w_q = nullptr) override {
    DataVec x = s.x.template cast<ACC_T>();

    const ACC_T y = w_acc.dot(x);
    const ACC_T e = static_cast<ACC_T>(s.d) - y;

    DataVec p = R_hat_inv * x;
    const ACC_T phi = x.dot(p);

    ACC_T denom = static_cast<ACC_T>((ACC_T(1) - alpha) / alpha) + phi;
    if (std::abs(denom) < eps) {
      denom += ACC_T(eps);
    }

    R_hat_inv = (R_hat_inv - (p * p.adjoint()) / denom) / (ACC_T(1) - alpha);
    w_acc.noalias() += ACC_T(2) * mu * e * (R_hat_inv * x);

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

  json hyperparams() const override {
    return {
      {"otype", "lms_newton"},
      {"mu", mu},
      {"alpha", alpha},
      {"delta", delta},
      {"eps", eps}
    };
  }

private:
  std::size_t n_weights = 0, n_iters = 0;
  ACC_T mu    = ACC_T(1e-2);
  ACC_T alpha = ACC_T(0.99);
  ACC_T delta = ACC_T(1e-2);
  double eps  = 1e-12;
  DataMatrix R_hat_inv;
};

// ------------------------------------------------------------
// RLS
// ------------------------------------------------------------
template <typename T, typename PARAMS_T = T, typename ACC_T = float>
class RLSOptimizer : public AdaptiveOptimizer<T, PARAMS_T, ACC_T> {
public:
  using Base       = AdaptiveOptimizer<T, PARAMS_T, ACC_T>;
  using DataVec    = typename Base::DataVec;
  using DataMatrix = typename Base::DataMatrix;
  using AccVec     = typename Base::AccVec;
  using ParamVec   = typename Base::ParamVec;

  explicit RLSOptimizer(const json& params) { update_hyperparams(params); }

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

  void step_update(const AFStepState<T>& s, Eigen::Ref<AccVec> w_acc,
                   Eigen::Ref<ParamVec>* w_q = nullptr) override {
    DataVec x = s.x.template cast<ACC_T>();

    const ACC_T y = w_acc.dot(x);
    const ACC_T e = static_cast<ACC_T>(s.d) - y;

    const DataVec num = S_D * x;
    ACC_T denom = lambda + x.dot(num);
    if (std::abs(denom) < eps) {
      denom += ACC_T(eps);
    }

    const DataVec k = num / denom;
    w_acc.noalias() += scalar_conj_if_needed(e) * k;
    S_D = (S_D - k * (x.adjoint() * S_D)) / lambda;

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

  json hyperparams() const override {
    return {
      {"otype", "rls"},
      {"lambda", lambda},
      {"delta", delta},
      {"eps", eps}
    };
  }

private:
  std::size_t n_weights = 0, n_iters = 0;
  ACC_T lambda = ACC_T(0.99);
  ACC_T delta  = ACC_T(1e-2);
  double eps   = 1e-12;
  DataMatrix S_D;
};

// ------------------------------------------------------------
// Transform-Domain
// NOTE: pair this with a filter whose weight domain matches the
// transform domain. With a plain time-domain FIR filter, this
// optimizer is not mathematically consistent.
// ------------------------------------------------------------
template <typename T, typename PARAMS_T = T, typename ACC_T = float>
class TransformDomainOptimizer : public AdaptiveOptimizer<T, PARAMS_T, ACC_T> {
public:
  using Base       = AdaptiveOptimizer<T, PARAMS_T, ACC_T>;
  using DataVec    = typename Base::DataVec;
  using DataMatrix = typename Base::DataMatrix;
  using AccVec     = typename Base::AccVec;
  using ParamVec   = typename Base::ParamVec;
  using RealT      = typename Eigen::NumTraits<ACC_T>::Real;

  explicit TransformDomainOptimizer(const json& params) { update_hyperparams(params); }

  void allocate(const std::size_t n_ws) override {
    n_weights = n_ws;
    n_iters = 0;
    build_transform();
    power = Eigen::Matrix<RealT, Eigen::Dynamic, 1>::Constant(n_ws, init_power);
  }

  void reset() override {
    n_iters = 0;
    power.setConstant(init_power);
  }

  std::size_t get_n_iterations() const override { return n_iters; }
  std::size_t get_n_weights() const override { return n_weights; }
  ACC_T get_step_size() const override { return mu; }
  void set_step_size(const ACC_T m) override { mu = m; }

  void step_update(const AFStepState<T>& s, Eigen::Ref<AccVec> w_acc,
                   Eigen::Ref<ParamVec>* w_q = nullptr) override {
    DataVec x = s.x.template cast<ACC_T>();
    DataVec sig = Tm * x;

    auto p_new = (alpha * sig.cwiseAbs2().template cast<RealT>().array()
                + (RealT(1) - alpha) * power.array()).matrix();

    const ACC_T y = w_acc.dot(sig);
    const ACC_T e = static_cast<ACC_T>(s.d) - y;

    Eigen::Matrix<ACC_T, Eigen::Dynamic, 1> denom =
      (p_new.array() + gamma).template cast<ACC_T>();

    w_acc.noalias() += ACC_T(2) * mu * scalar_conj_if_needed(e) * sig.cwiseQuotient(denom);
    power = p_new;

    if (w_q) {
      (*w_q) = w_acc.template cast<PARAMS_T>();
    }
    ++n_iters;
  }

  void update_hyperparams(const json& params) override {
    if (params.contains("mu"))         mu         = params.at("mu").template get<ACC_T>();
    if (params.contains("alpha"))      alpha      = params.at("alpha").template get<RealT>();
    if (params.contains("gamma"))      gamma      = params.at("gamma").template get<RealT>();
    if (params.contains("init_power")) init_power = params.at("init_power").template get<RealT>();
    if (params.contains("matrix"))     matrix     = params.at("matrix").get<std::string>();
  }

  json hyperparams() const override {
    return {
      {"otype", "transform_domain"},
      {"mu", mu},
      {"alpha", alpha},
      {"gamma", gamma},
      {"init_power", init_power},
      {"matrix", matrix}
    };
  }

private:
  void build_transform() {
    const std::string kind = to_lower(matrix);

    if (kind == "dct") {
      Tm = dct_ortho_matrix<ACC_T>(n_weights);
    } else if (kind == "dft") {
      using C = std::complex<typename Eigen::NumTraits<ACC_T>::Real>;
      static_assert(std::is_same_v<ACC_T, C>, "DFT transform requires complex ACC_T");
      Tm = dft_unitary_matrix<typename Eigen::NumTraits<ACC_T>::Real>(n_weights);
    } else {
      throw std::runtime_error("Unknown transform matrix kind: " + matrix);
    }
  }

  std::size_t n_weights = 0, n_iters = 0;
  ACC_T mu = ACC_T(1e-2);
  RealT alpha = RealT(0.01);
  RealT gamma = RealT(1e-8);
  RealT init_power = RealT(1.0);
  std::string matrix = "dct";

  DataMatrix Tm;
  Eigen::Matrix<RealT, Eigen::Dynamic, 1> power;
};

// ------------------------------------------------------------
// your existing hyperparam impls can stay, but these are aligned
// ------------------------------------------------------------
template <typename T, typename PARAMS_T, typename ACC_T>
inline json LMSOptimizer<T, PARAMS_T, ACC_T>::hyperparams() const {
  return {
    {"otype", "lms"}, 
    {"mu", mu}, 
    {"n_weights", n_weights}
  };
}

template <typename T, typename PARAMS_T, typename ACC_T>
inline void LMSOptimizer<T, PARAMS_T, ACC_T>::update_hyperparams(const json& params) {
  if (params.contains("mu")) {
    mu = params.at("mu").template get<ACC_T>();
  }
}

template <typename T, typename PARAMS_T, typename ACC_T>
inline json APAOptimizer<T, PARAMS_T, ACC_T>::hyperparams() const {
  return {
    {"otype", "apa"},
    {"mu", mu},
    {"gamma", gamma},
    {"projection_order", P}
  };
}

template <typename T, typename PARAMS_T, typename ACC_T>
inline void APAOptimizer<T, PARAMS_T, ACC_T>::update_hyperparams(const json& params) {
  if (params.contains("mu"))               mu = params.at("mu").template get<ACC_T>();
  if (params.contains("gamma"))            gamma = params.at("gamma").template get<ACC_T>();
  if (params.contains("projection_order")) P = params.at("projection_order").get<std::size_t>();
  if (params.contains("P"))                P = params.at("P").get<std::size_t>();
}

// ------------------------------------------------------------
// factory
// ------------------------------------------------------------
template <typename T, typename PARAMS_T = T, typename ACC_T = float>
AdaptiveOptimizer<T, PARAMS_T, ACC_T>*
create_optimizer(const json& af_params) {
  const std::string type = af_params.value("otype", "lms");

  if (eq_nocase(type, "lms")) {
    return new LMSOptimizer<T, PARAMS_T, ACC_T>{af_params};
  } else if (eq_nocase(type, "nlms")) {
    return new NLMSOptimizer<T, PARAMS_T, ACC_T>{af_params};
  } else if (eq_nocase(type, "apa") || eq_nocase(type, "affineprojection")) {
    return new APAOptimizer<T, PARAMS_T, ACC_T>{af_params};
  } else if (eq_nocase(type, "sign_error")) {
    return new SignErrorOptimizer<T, PARAMS_T, ACC_T>{af_params};
  } else if (eq_nocase(type, "sign_data")) {
    return new SignDataOptimizer<T, PARAMS_T, ACC_T>{af_params};
  } else if (eq_nocase(type, "sign_sign")) {
    return new SignSignOptimizer<T, PARAMS_T, ACC_T>{af_params};
  } else if (eq_nocase(type, "dual_sign")) {
    return new DualSignOptimizer<T, PARAMS_T, ACC_T>{af_params};
  } else if (eq_nocase(type, "power_of_two_error")) {
    return new PowerOfTwoErrorOptimizer<T, PARAMS_T, ACC_T>{af_params};
  } else if (eq_nocase(type, "lms_newton") || eq_nocase(type, "lmsnewton")) {
    return new LMSNewtonOptimizer<T, PARAMS_T, ACC_T>{af_params};
  } else if (eq_nocase(type, "transform_domain") || eq_nocase(type, "tdlms")) {
    return new TransformDomainOptimizer<T, PARAMS_T, ACC_T>{af_params};
  } else if (eq_nocase(type, "rls")) {
    return new RLSOptimizer<T, PARAMS_T, ACC_T>{af_params};
  }

  throw std::runtime_error("Invalid adaptive optimizer type: " + type);
}

template AdaptiveOptimizer<float, float, float>*
create_optimizer<float, float, float>(const json& af_params);

} // namespace adptsysc