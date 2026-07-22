#pragma once

#include <adptsysc/object.hh>
#include <adptsysc/design-lib.hh>
#include <algorithm>
#include <cassert>
#include <cmath>
#include <complex>
#include <cstddef>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <vector>
#include <Eigen/Dense>

namespace adptsysc {

std::string to_lower(const std::string str);
std::string to_upper(const std::string str);
bool eq_nocase(const std::string& s1, const std::string& s2);

template <typename SampleT>
struct AFStepState {
  using SampleVec = Eigen::Matrix<SampleT, Eigen::Dynamic, 1>;

  Eigen::Ref<const SampleVec> x;
  SampleT d;
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

template <typename SampleT, typename CoeffT = SampleT, typename WorkT = SampleT>
class AdaptiveOptimizer : public ObjectWithMutableHyperparams {
public:
  using SampleVec = Eigen::Matrix<SampleT, Eigen::Dynamic, 1>;
  using WorkVec = Eigen::Matrix<WorkT, Eigen::Dynamic, 1>;
  using WorkMat = Eigen::Matrix<WorkT, Eigen::Dynamic, Eigen::Dynamic>;
  using CoeffVec = Eigen::Matrix<CoeffT, Eigen::Dynamic, 1>;
  using RealT = typename Eigen::NumTraits<WorkT>::Real;

  // Source-compatible aliases for the pre-WorkT API.
  using DataVec = WorkVec;
  using DataMatrix = WorkMat;
  using AccVec = WorkVec;
  using ParamVec = CoeffVec;

  static_assert(std::is_floating_point_v<RealT>,
                "WorkT must be a floating-point or complex floating-point type");

  virtual ~AdaptiveOptimizer() = default;

  virtual void allocate(const std::size_t n_weights) = 0;

  virtual void allocate(const std::shared_ptr<ParametricObject<CoeffT>>& target) {
    allocate(static_cast<std::size_t>(target->n_params()));
  }

  virtual void reset() = 0;

  virtual std::size_t get_n_iterations() const = 0;
  virtual std::size_t get_n_weights() const = 0;

  virtual RealT get_step_size() const = 0;
  virtual void set_step_size(RealT mu) = 0;

  // Sample-based update.
  virtual void step_update(
    const AFStepState<SampleT>& s,
    Eigen::Ref<WorkVec> train_weights,
    Eigen::Ref<CoeffVec>* coeffs = nullptr
  ) = 0;

  virtual json serialize() const { return {}; }
  virtual void deserialize(const json&) {}
};

template <typename SampleT, typename CoeffT = SampleT, typename WorkT = SampleT>
class LMSOptimizer : public AdaptiveOptimizer<SampleT, CoeffT, WorkT> {
public:
  using Base     = AdaptiveOptimizer<SampleT, CoeffT, WorkT>;
  using RealT = typename Base::RealT;
  using WorkVec   = typename Base::WorkVec;
  using CoeffVec = typename Base::CoeffVec;
  using WorkMat  = typename Base::WorkMat;

  struct AnalysisInfo {
    bool   is_stable;
    RealT  mu_max;
    RealT  mu_trace;
    RealT  theor_misadj;
    RealT  mu_conservative;
    RealT  mu_aggressive;
  };

  AnalysisInfo analyze(const WorkMat* R = nullptr) const;
  
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

  RealT get_step_size() const override { return mu; }
  void set_step_size(RealT m) override { mu = m; }

  void step_update(
    const AFStepState<SampleT>& s,
    Eigen::Ref<WorkVec> train_weights,
    Eigen::Ref<CoeffVec>* coeffs = nullptr
  ) override {

    assert(static_cast<std::size_t>(train_weights.size()) == n_weights);
    assert(s.x.size() == train_weights.size());

    WorkT y;
    WorkT e;

    if constexpr (std::is_same_v<SampleT, WorkT>) {
      y = train_weights.dot(s.x);
      e = static_cast<WorkT>(s.d) - y;

      if constexpr (Eigen::NumTraits<WorkT>::IsComplex) {
        // Complex LMS
        train_weights.noalias() += mu * s.x * std::conj(e);
      } else {
        train_weights.noalias() += (WorkT(2) * mu * e) * s.x;
      }

    } else {
      WorkVec x_acc = s.x.template cast<WorkT>();

      y = train_weights.dot(x_acc);
      e = static_cast<WorkT>(s.d) - y;

      if constexpr (Eigen::NumTraits<WorkT>::IsComplex) {
        train_weights.noalias() += mu * x_acc * std::conj(e);
      } else {
        train_weights.noalias() += (WorkT(2) * mu * e) * x_acc;
      }
    }

    if (coeffs) {
      (*coeffs) = train_weights.template cast<CoeffT>();
    }

    ++n_iters;
  }

  void update_hyperparams(const json& params) override;
  json get_hyperparams() const override;

private:
  std::size_t n_weights = 0;
  std::size_t n_iters = 0;
  RealT mu = RealT(1e-2);
};


template <typename SampleT, typename CoeffT = SampleT, typename WorkT = SampleT>
class APAOptimizer : public AdaptiveOptimizer<SampleT, CoeffT, WorkT> {
public:
  using Base       = AdaptiveOptimizer<SampleT, CoeffT, WorkT>;
  using RealT = typename Base::RealT;
  using WorkVec    = typename Base::WorkVec;
  using WorkMat = typename Base::WorkMat;
  using CoeffVec   = typename Base::CoeffVec;
  
  APAOptimizer(const json& params) {
    update_hyperparams(params);
  }

  void allocate(const std::size_t n_ws) override {
    n_weights = n_ws;
    n_iters = 0;

    X_hist = WorkMat::Zero(n_ws, P+1);
    d_hist = WorkVec::Zero(P+1);
  }

  void reset() override {
    n_iters = 0;
    X_hist.setZero();
    d_hist.setZero();
  }

  std::size_t get_n_iterations() const override { return n_iters; }
  std::size_t get_n_weights() const override { return n_weights; }

  RealT get_step_size() const override { return mu; }
  void set_step_size(RealT m) override { mu = m; }

  void step_update(
    const AFStepState<SampleT>& s,
    Eigen::Ref<WorkVec> train_weights,
    Eigen::Ref<CoeffVec>* coeffs = nullptr
  ) override {

    assert(static_cast<std::size_t>(train_weights.size()) == n_weights);
    assert(s.x.size() == train_weights.size());

    // update history buffers
    X_hist.rightCols(P) = X_hist.leftCols(P);
    d_hist.tail(P) = d_hist.head(P);

    X_hist.col(0) = s.x.template cast<WorkT>();
    d_hist(0) = static_cast<WorkT>(s.d);

    // build matrices
    WorkVec y = (train_weights.adjoint() * X_hist).transpose();
    WorkVec e = d_hist - y;

    WorkMat R = X_hist.adjoint() * X_hist;
    R.diagonal().array() += WorkT(gamma);

    WorkVec rhs = e;
    if constexpr (Eigen::NumTraits<WorkT>::IsComplex) {
      rhs = e.conjugate();
    }

    WorkVec g = R.ldlt().solve(rhs);
    train_weights.noalias() += WorkT(mu) * X_hist * g;

    if (coeffs) {
      (*coeffs) = train_weights.template cast<CoeffT>();
    }

    ++n_iters;
  }

  void update_hyperparams(const json& params) override;
  json get_hyperparams() const override;

private:

  std::size_t n_weights = 0;
  std::size_t n_iters = 0;

  RealT mu = RealT(0.1);
  RealT gamma = RealT(1e-6);
  std::size_t P = 1;

  WorkMat X_hist;
  WorkVec d_hist;
};

template <typename SampleT, typename CoeffT = SampleT, typename WorkT = SampleT>
class SignErrorOptimizer : public AdaptiveOptimizer<SampleT, CoeffT, WorkT> {
public:
  using Base     = AdaptiveOptimizer<SampleT, CoeffT, WorkT>;
  using RealT = typename Base::RealT;
  using WorkVec  = typename Base::WorkVec;
  using CoeffVec = typename Base::CoeffVec;

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
  RealT get_step_size() const override { return mu; }
  void set_step_size(RealT m) override { mu = m; }

  void step_update (
    const AFStepState<SampleT>& s,
    Eigen::Ref<WorkVec> train_weights,
    Eigen::Ref<CoeffVec>* coeffs = nullptr
  ) override {

    WorkT y, e, se;

    if constexpr (std::is_same_v<SampleT, WorkT>) {
      y  = train_weights.dot(s.x);
      e  = static_cast<WorkT>(s.d) - y;
      se = get_scalar_sign(e, eps);

      train_weights.noalias() += (WorkT(2) * mu * se) * s.x;

    } else {
      WorkVec x = s.x.template cast<WorkT>();

      y  = train_weights.dot(x);
      e  = static_cast<WorkT>(s.d) - y;
      se = get_scalar_sign(e, eps);

      train_weights.noalias() += (WorkT(2) * mu * se) * x;
    }

    if (coeffs) {
      (*coeffs) = train_weights.template cast<CoeffT>();
    }

    ++n_iters;
  }

  void update_hyperparams(const json& params) override {
    if (params.contains("mu")) 
      mu = params.at("mu").template get<RealT>();
    if (params.contains("eps")) 
      eps = params.at("eps").template get<RealT>();
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
  RealT mu = RealT(1e-2);
  RealT eps = RealT(1e-12);
};

template <typename SampleT, typename CoeffT = SampleT, typename WorkT = SampleT>
class SignDataOptimizer : public AdaptiveOptimizer<SampleT, CoeffT, WorkT> {
public:
  using Base     = AdaptiveOptimizer<SampleT, CoeffT, WorkT>;
  using RealT = typename Base::RealT;
  using WorkVec  = typename Base::WorkVec;
  using CoeffVec = typename Base::CoeffVec;

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
  RealT get_step_size() const override { return mu; }
  void set_step_size(RealT m) override { mu = m; }

  /**
   * @brief Performs the Sign-Data LMS update.
   * Update Rule: w(n+1) = w(n) + 2 * mu * e(n) * sign(x(n))
   */
  void step_update(
    const AFStepState<SampleT>& s,
    Eigen::Ref<WorkVec> train_weights,
    Eigen::Ref<CoeffVec>* coeffs = nullptr
  ) override {
    auto update = [&](const WorkVec& x_vec) {
      const WorkT y = train_weights.dot(x_vec);
      const WorkT e = static_cast<WorkT>(s.d) - y;

      auto sx = get_vector_sign(x_vec, eps);

      if constexpr (Eigen::NumTraits<WorkT>::IsComplex) {
        train_weights.noalias() += (WorkT(2) * mu * std::conj(e)) * sx;
      } else {
        train_weights.noalias() += (WorkT(2) * mu * e) * sx;
      }
    };

    if constexpr (std::is_same_v<SampleT, WorkT>) {
      update(s.x);
    } else {
      WorkVec x_acc = s.x.template cast<WorkT>();
      update(x_acc);
    }

    if (coeffs) {
      (*coeffs) = train_weights.template cast<CoeffT>();
    }

    ++n_iters;
  }

  void update_hyperparams(const json& params) override {
    if (params.contains("mu")) 
      mu = params.at("mu").template get<RealT>();
    if (params.contains("eps")) 
      eps = params.at("eps").template get<RealT>();
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
  RealT mu = RealT(1e-2);
  RealT eps = RealT(1e-12); // Small epsilon to prevent sign ambiguity/division by zero
};

template <typename SampleT, typename CoeffT = SampleT, typename WorkT = SampleT>
class SignSignOptimizer : public AdaptiveOptimizer<SampleT, CoeffT, WorkT> {
public:
  using Base     = AdaptiveOptimizer<SampleT, CoeffT, WorkT>;
  using RealT = typename Base::RealT;
  using WorkVec  = typename Base::WorkVec;
  using CoeffVec = typename Base::CoeffVec;

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

  RealT get_step_size() const override { return mu; }
  void set_step_size(RealT m) override { mu = m; }

  void step_update(
    const AFStepState<SampleT>& s,
    Eigen::Ref<WorkVec> train_weights,
    Eigen::Ref<CoeffVec>* coeffs = nullptr
  ) override {

    auto update = [&](const WorkVec& x_vec) {
      const WorkT y = train_weights.dot(x_vec);
      const WorkT e = static_cast<WorkT>(s.d) - y;

      const WorkT se = get_scalar_sign(e, eps);
      const WorkVec sx = get_vector_sign(x_vec, eps);

      train_weights.noalias() += (WorkT(2) * mu * se) * sx;
    };

    if constexpr (std::is_same_v<SampleT, WorkT>) {
      update(s.x);
    } else {
      WorkVec x_acc = s.x.template cast<WorkT>();
      update(x_acc);
    }

    if (coeffs) {
      (*coeffs) = train_weights.template cast<CoeffT>();
    }

    ++n_iters;
  }

  void update_hyperparams(const json& params) override {
    if (params.contains("mu"))
      mu = params.at("mu").template get<RealT>();
    if (params.contains("eps"))
      eps = params.at("eps").template get<RealT>();
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

  RealT mu = RealT(1e-2);
  RealT eps = RealT(1e-12);
};

template <typename SampleT, typename CoeffT = SampleT, typename WorkT = SampleT>
class DualSignOptimizer : public AdaptiveOptimizer<SampleT, CoeffT, WorkT> {
public:
  using Base     = AdaptiveOptimizer<SampleT, CoeffT, WorkT>;
  using RealT = typename Base::RealT;
  using WorkVec  = typename Base::WorkVec;
  using CoeffVec = typename Base::CoeffVec;

  DualSignOptimizer(const json& params) { update_hyperparams(params); }

  void allocate(const std::size_t n_ws) override {
    n_weights = n_ws;
    n_iters = 0;
  }

  void reset() override { n_iters = 0; }

  std::size_t get_n_iterations() const override { return n_iters; }
  std::size_t get_n_weights() const override { return n_weights; }
  RealT get_step_size() const override { return mu; }
  void set_step_size(RealT m) override { mu = m; }

  void step_update(
    const AFStepState<SampleT>& s,
    Eigen::Ref<WorkVec> train_weights,
    Eigen::Ref<CoeffVec>* coeffs = nullptr
  ) override {
    auto update = [&](const WorkVec& x_vec) {
      const WorkT y  = train_weights.dot(x_vec);
      const WorkT e  = static_cast<WorkT>(s.d) - y;
      const WorkT se = get_scalar_sign(e, eps);
      const WorkT g = (std::abs(e) > rho) ? WorkT(epsilon_gain) : WorkT(1);

      train_weights.noalias() += (WorkT(2) * mu * g * se) * x_vec;
    };

    if constexpr (std::is_same_v<SampleT, WorkT>) {
      update(s.x);
    } else {
      WorkVec x_acc = s.x.template cast<WorkT>();
      update(x_acc);
    }

    if (coeffs) {
      (*coeffs) = train_weights.template cast<CoeffT>();
    }
    ++n_iters;
  }

  void update_hyperparams(const json& params) override {
    if (params.contains("mu"))      mu = params.at("mu").template get<RealT>();
    if (params.contains("rho"))     rho = params.at("rho").template get<RealT>();
    if (params.contains("epsilon")) epsilon_gain = params.at("epsilon").template get<RealT>();
    if (params.contains("eps"))     eps = params.at("eps").template get<RealT>();
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
  RealT mu = RealT(1e-2);
  RealT rho = RealT(1);
  RealT epsilon_gain = RealT(2);
  RealT eps = RealT(1e-12);
};

template <typename SampleT, typename CoeffT = SampleT, typename WorkT = SampleT>
class PowerOfTwoErrorOptimizer : public AdaptiveOptimizer<SampleT, CoeffT, WorkT> {
public:
  using Base     = AdaptiveOptimizer<SampleT, CoeffT, WorkT>;
  using RealT = typename Base::RealT;
  using WorkVec  = typename Base::WorkVec;
  using CoeffVec = typename Base::CoeffVec;

  PowerOfTwoErrorOptimizer(const json& params) { update_hyperparams(params); }

  void allocate(const std::size_t n_ws) override {
    n_weights = n_ws;
    n_iters = 0;
  }

  void reset() override { n_iters = 0; }

  std::size_t get_n_iterations() const override { return n_iters; }
  std::size_t get_n_weights() const override { return n_weights; }
  RealT get_step_size() const override { return mu; }
  void set_step_size(RealT m) override { mu = m; }

  WorkT p2e(const WorkT& e) const {
    const RealT abs_e  = std::abs(e);
    const WorkT s      = get_scalar_sign(e, eps);
    const RealT thresh = std::pow(RealT(2), -static_cast<RealT>(bd - 1));

    if (abs_e >= RealT(1)) {
      return s;
    } else if (abs_e >= thresh) {
      const RealT pow_mag = std::exp2(std::floor(std::log2(abs_e + RealT(eps))));
      return static_cast<WorkT>(pow_mag) * s;
    } else {
      return static_cast<WorkT>(tau_floor) * s;
    }
  }

  void step_update(
    const AFStepState<SampleT>& s,
    Eigen::Ref<WorkVec> train_weights,
    Eigen::Ref<CoeffVec>* coeffs = nullptr
  ) override {
    auto update = [&](const WorkVec& x_vec) {
      const WorkT y  = train_weights.dot(x_vec);
      const WorkT e  = static_cast<WorkT>(s.d) - y;
      const WorkT pe = p2e(e);

      train_weights.noalias() += (WorkT(2) * mu * pe) * x_vec;
    };

    if constexpr (std::is_same_v<SampleT, WorkT>) {
      update(s.x);
    } else {
      WorkVec x_acc = s.x.template cast<WorkT>();
      update(x_acc);
    }

    if (coeffs) {
      (*coeffs) = train_weights.template cast<CoeffT>();
    }
    ++n_iters;
  }

  void update_hyperparams(const json& params) override {
    if (params.contains("mu"))  mu = params.at("mu").template get<RealT>();
    if (params.contains("bd"))  bd = params.at("bd").get<int>();
    if (params.contains("tau")) tau_floor = params.at("tau").template get<RealT>();
    if (params.contains("eps")) eps = params.at("eps").template get<RealT>();
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
  RealT mu = RealT(1e-2);
  int bd = 8;
  RealT tau_floor = RealT(0);
  RealT eps = RealT(1e-12);
};

// ------------------------------------------------------------
// LMS-Newton
// ------------------------------------------------------------
template <typename SampleT, typename CoeffT = SampleT, typename WorkT = SampleT>
class LMSNewtonOptimizer : public AdaptiveOptimizer<SampleT, CoeffT, WorkT> {
public:
  using Base       = AdaptiveOptimizer<SampleT, CoeffT, WorkT>;
  using RealT = typename Base::RealT;
  using WorkVec    = typename Base::WorkVec;
  using CoeffVec   = typename Base::CoeffVec;
  using WorkMat = Eigen::Matrix<WorkT, Eigen::Dynamic, Eigen::Dynamic>;

  LMSNewtonOptimizer(const json& params) { 
    update_hyperparams(params); 
  }

  void allocate(const std::size_t n_ws) override {
    n_weights = n_ws;
    n_iters = 0;
    R_hat_inv = (WorkT(1) / delta) * WorkMat::Identity(n_ws, n_ws);
  }

  void reset() override {
    n_iters = 0;
    R_hat_inv = (WorkT(1) / delta) * WorkMat::Identity(n_weights, n_weights);
  }

  std::size_t get_n_iterations() const override { return n_iters; }
  std::size_t get_n_weights() const override { return n_weights; }
  RealT get_step_size() const override { return mu; }
  void set_step_size(RealT m) override { mu = m; }

  void step_update(
    const AFStepState<SampleT>& s,
    Eigen::Ref<WorkVec> train_weights,
    Eigen::Ref<CoeffVec>* coeffs = nullptr
  ) override {
    auto update = [&](const WorkVec& x_vec) {
      const WorkT y = train_weights.dot(x_vec);
      const WorkT e = static_cast<WorkT>(s.d) - y;

      const WorkVec p   = R_hat_inv * x_vec;
      const WorkT phi  = x_vec.dot(p);
      WorkT denom = WorkT((RealT(1) - alpha) / alpha) + phi;

      if (std::abs(denom) < eps) {
        denom += WorkT(eps);
      }

      R_hat_inv = (R_hat_inv - (p * p.adjoint()) / denom) / WorkT(RealT(1) - alpha);

      if constexpr (Eigen::NumTraits<WorkT>::IsComplex) {
        train_weights.noalias() += WorkT(2) * mu * std::conj(e) * (R_hat_inv * x_vec);
      } else {
        train_weights.noalias() += WorkT(2) * mu * e * (R_hat_inv * x_vec);
      }
    };

    if constexpr (std::is_same_v<SampleT, WorkT>) {
      update(s.x);
    } else {
      WorkVec x_acc = s.x.template cast<WorkT>();
      update(x_acc);
    }

    if (coeffs) {
      (*coeffs) = train_weights.template cast<CoeffT>();
    }
    ++n_iters;
  }

  void update_hyperparams(const json& params) override {
    if (params.contains("mu"))    mu    = params.at("mu").template get<RealT>();
    if (params.contains("alpha")) alpha = params.at("alpha").template get<RealT>();
    if (params.contains("delta")) delta = params.at("delta").template get<RealT>();
    if (params.contains("eps"))   eps   = params.at("eps").template get<RealT>();
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
  RealT mu = RealT(1e-2);
  RealT alpha = RealT(0.99);
  RealT delta = RealT(1e-2);
  RealT eps = RealT(1e-12);
  WorkMat R_hat_inv;
};

template <typename SampleT, typename CoeffT = SampleT, typename WorkT = SampleT>
class RLSOptimizer : public AdaptiveOptimizer<SampleT, CoeffT, WorkT> {
public:
  using Base       = AdaptiveOptimizer<SampleT, CoeffT, WorkT>;
  using RealT = typename Base::RealT;
  using WorkVec    = typename Base::WorkVec;
  using CoeffVec   = typename Base::CoeffVec;
  using WorkMat  = typename Base::WorkMat;

  struct AnalysisInfo {
    bool   is_stable;
    RealT  eff_mem;
    RealT  theor_misadj;
    RealT  P_cond_num;
    RealT  lam_slow;
    RealT  lam_fast;
  };

  AnalysisInfo analyze(const WorkMat* R = nullptr) const;

  RLSOptimizer(const json& params) { 
    update_hyperparams(params); 
  }

  void allocate(const std::size_t n_ws) override {
    n_weights = n_ws;
    n_iters = 0;
    S_D = (WorkT(1) / delta) * WorkMat::Identity(n_ws, n_ws);
  }

  void reset() override {
    n_iters = 0;
    S_D = (WorkT(1) / delta) * WorkMat::Identity(n_weights, n_weights);
  }

  std::size_t get_n_iterations() const override { return n_iters; }
  std::size_t get_n_weights() const override { return n_weights; }
  RealT get_step_size() const override { return lambda; }
  void set_step_size(RealT lam) override { lambda = lam; }

  void step_update (
    const AFStepState<SampleT>& s,
    Eigen::Ref<WorkVec> train_weights,
    Eigen::Ref<CoeffVec>* coeffs = nullptr
  ) override {
    auto update = [&](const WorkVec& x_vec) {
      const WorkT y = train_weights.dot(x_vec);
      const WorkT e = static_cast<WorkT>(s.d) - y;

      const WorkVec num = S_D * x_vec;
      WorkT denom = lambda + x_vec.dot(num);

      if (std::abs(denom) < eps) {
        denom += WorkT(eps);
      }

      const WorkVec k = num / denom;

      if constexpr (Eigen::NumTraits<WorkT>::IsComplex) {
        train_weights.noalias() += std::conj(e) * k;
      } else {
        train_weights.noalias() += e * k;
      }

      S_D = (S_D - k * (x_vec.adjoint() * S_D)) / lambda;
    };

    if constexpr (std::is_same_v<SampleT, WorkT>) {
      update(s.x);
    } else {
      WorkVec x_acc = s.x.template cast<WorkT>();
      update(x_acc);
    }

    if (coeffs) {
      (*coeffs) = train_weights.template cast<CoeffT>();
    }
    ++n_iters;
  }

  void update_hyperparams(const json& params) override {
    if (params.contains("lambda")) lambda = params.at("lambda").template get<RealT>();
    if (params.contains("lam"))    lambda = params.at("lam").template get<RealT>();
    if (params.contains("delta"))  delta  = params.at("delta").template get<RealT>();
    if (params.contains("eps"))    eps    = params.at("eps").template get<RealT>();
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
  RealT lambda = RealT(0.99);
  RealT delta  = RealT(1e-2);
  RealT eps = RealT(1e-12);
  WorkMat S_D;
};

template <typename SampleT, typename CoeffT = SampleT, typename WorkT = SampleT>
class NLMSOptimizer : public AdaptiveOptimizer<SampleT, CoeffT, WorkT> {
public:
  using Base     = AdaptiveOptimizer<SampleT, CoeffT, WorkT>;
  using RealT = typename Base::RealT;
  using WorkVec  = typename Base::WorkVec;
  using CoeffVec = typename Base::CoeffVec;

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

  RealT get_step_size() const override { return mu; }
  void set_step_size(RealT m) override { mu = m; }

  void step_update (
    const AFStepState<SampleT>& s,
    Eigen::Ref<WorkVec> train_weights,
    Eigen::Ref<CoeffVec>* coeffs = nullptr
  ) override {

    if constexpr (std::is_same_v<SampleT, WorkT>) {
      const auto& x = s.x; // No cast needed
      const WorkT y = train_weights.dot(x);
      const WorkT e = static_cast<WorkT>(s.d) - y;
      const RealT norm2 = x.squaredNorm();
      
      // NLMS normalization factor
      const RealT mu_k = mu / (tau + norm2);

      // Correct Complex vs Real logic using NumTraits
      if constexpr (Eigen::NumTraits<WorkT>::IsComplex) {
          train_weights.noalias() += mu_k * std::conj(e) * x;
      } else {
          train_weights.noalias() += mu_k * e * x;
      }
    } else {
      // Cast input once if types differ
      WorkVec x_acc = s.x.template cast<WorkT>();
      const WorkT y = train_weights.dot(x_acc);
      const WorkT e = static_cast<WorkT>(s.d) - y;
      const RealT norm2 = x_acc.squaredNorm();
      
      const RealT mu_k = mu / (tau + norm2);

      if constexpr (Eigen::NumTraits<WorkT>::IsComplex) {
          train_weights.noalias() += mu_k * std::conj(e) * x_acc;
      } else {
          train_weights.noalias() += mu_k * e * x_acc;
      }
    }

    // Quantize/copy weights if output pointer provided
    if (coeffs) {
      (*coeffs) = train_weights.template cast<CoeffT>();
    }
    ++n_iters;
  }

  void update_hyperparams(const json& params) override {
    if (params.contains("mu"))  
      mu  = params.at("mu").template get<RealT>();
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
  RealT mu = RealT(1);
  RealT tau = RealT(1e-3);
};

template <typename SampleT, typename CoeffT = SampleT, typename WorkT = SampleT>
class FDAFOptimizer : public AdaptiveOptimizer<SampleT, CoeffT, WorkT> {
public:
  using Base     = AdaptiveOptimizer<SampleT, CoeffT, WorkT>;
  using RealT = typename Base::RealT;
  using WorkVec   = typename Base::WorkVec;
  using CoeffVec = typename Base::CoeffVec;
  static_assert(!Eigen::NumTraits<WorkT>::IsComplex,
                "FDAFOptimizer expects a real WorkT because its FFT backend owns complex samples");
  using CxT = std::complex<RealT>;

  FDAFOptimizer(const json& params,
                std::shared_ptr<FFTIntf<RealT>> fft_if) : fft(fft_if) {
    update_hyperparams(params);
  }

  void allocate(const std::size_t n_ws) override {
    M = n_ws;
    N = 2 * M;

    w_freq.assign(N, CxT(0,0));
    pow_est.assign(N, eps);

    grad_freq.resize(N);
    grad_time.resize(N);
    e_freq.resize(N);
    e_time.resize(N);
  }

  void reset() override {
    std::fill(w_freq.begin(), w_freq.end(), CxT(0, 0));
    std::fill(pow_est.begin(), pow_est.end(), eps);
    n_iters = 0;
  }

  std::size_t get_n_iterations() const override { return n_iters; }
  std::size_t get_n_weights() const override { return M; }

  RealT get_step_size() const override { return mu; }
  void set_step_size(RealT m) override { mu = m; }

  void step_update_block(
    const std::vector<CxT>& x_freq,
    const Eigen::Ref<const WorkVec>& e_block) {
    if (M == 0 || N == 0) {
      throw std::runtime_error("FDAFOptimizer must be allocated before use");
    }
    if (x_freq.size() != N) {
      throw std::invalid_argument("x_freq has wrong size");
    }
    if (static_cast<std::size_t>(e_block.size()) != M) {
      throw std::invalid_argument("e_block has wrong size");
    }
    // 1. build padded error (time)
    std::fill(e_time.begin(), e_time.end(), CxT(0,0));
    for (std::size_t i = 0; i < M; ++i)
      e_time[N - M + i] = CxT(e_block[i], 0);

    // 2. FFT → e_freq
    fft->get_cmplxfft(e_time, e_freq);

    // 3. normalization
    for (std::size_t i = 0; i < N; ++i) {
      pow_est[i] = alpha * pow_est[i] + (RealT(1) - alpha) * std::norm(x_freq[i]);
      e_freq[i] /= (pow_est[i] + eps);
    }

    // 4. gradient in freq
    for (std::size_t i = 0; i < N; ++i)
      grad_freq[i] = std::conj(x_freq[i]) * e_freq[i];

    // 5. IFFT → time domain
    fft->get_cmplxifft(grad_freq, grad_time);

    // 6. enforce FIR constraint
    for (std::size_t i = M; i < N; ++i)
      grad_time[i] = CxT(0,0);

    // 7. FFT back
    fft->get_cmplxfft(grad_time, grad_freq);

    // 8. update weights
    for (std::size_t i = 0; i < N; ++i)
      w_freq[i] += mu * grad_freq[i];

    ++n_iters;
  }

  const std::vector<CxT>& get_weight_freq() const {
    return w_freq;
  }

  void update_hyperparams(const json& params) override {
    if (params.contains("mu")) {
      mu = params["mu"].template get<RealT>();
    }
    if (params.contains("alpha")) {
      alpha = params["alpha"].template get<RealT>();
    }
    if (params.contains("eps")) {
      eps = params["eps"].template get<RealT>();
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

  RealT mu = RealT(1e-2);
  RealT alpha = RealT(0.9);
  RealT eps = RealT(1e-8);

  std::vector<CxT> w_freq;
  std::vector<RealT> pow_est;

  std::shared_ptr<FFTIntf<RealT>> fft;
  std::vector<CxT> grad_freq;
  std::vector<CxT> grad_time;
  std::vector<CxT> e_freq;
  std::vector<CxT> e_time;
};

template <typename SampleT, typename CoeffT, typename WorkT>
json LMSOptimizer<SampleT, CoeffT, WorkT>::get_hyperparams() const {
  return {
    {"otype", "lms"},
    {"mu", mu},
    {"n_weights", n_weights},
  };
}

template <typename SampleT, typename CoeffT, typename WorkT>
void LMSOptimizer<SampleT, CoeffT, WorkT>::update_hyperparams(const json& params) {
  if (params.contains("mu")) {
    mu = params.at("mu").template get<RealT>();
  }
}

template <typename SampleT, typename CoeffT, typename WorkT>
typename LMSOptimizer<SampleT, CoeffT, WorkT>::AnalysisInfo
LMSOptimizer<SampleT, CoeffT, WorkT>::analyze(const WorkMat* R) const {
  AnalysisInfo result{};

  if (!R || R->rows() == 0 || R->cols() == 0) {
    return result;
  }

  Eigen::SelfAdjointEigenSolver<WorkMat> eigensolver(*R);
  if (eigensolver.info() != Eigen::Success) {
    return result;
  }

  const RealT lambda_max = eigensolver.eigenvalues().maxCoeff();
  const RealT trace = std::real(R->trace());
  const RealT eps = std::numeric_limits<RealT>::epsilon();

  if (lambda_max <= eps || trace <= eps) {
    return result;
  }

  result.mu_max = RealT(1) / lambda_max;
  result.mu_trace = RealT(1) / trace;
  result.is_stable = mu > RealT(0) && mu < result.mu_max;

  const RealT denom = RealT(1) - mu * trace;
  result.theor_misadj = denom > eps
      ? (mu * trace) / denom
      : std::numeric_limits<RealT>::infinity();
  result.mu_conservative = RealT(0.1) * result.mu_trace;
  result.mu_aggressive = RealT(0.5) * result.mu_trace;
  return result;
}

template <typename SampleT, typename CoeffT, typename WorkT>
json APAOptimizer<SampleT, CoeffT, WorkT>::get_hyperparams() const {
  return {
    {"otype", "apa"},
    {"mu", mu},
    {"gamma", gamma},
    {"P", P},
  };
}

template <typename SampleT, typename CoeffT, typename WorkT>
void APAOptimizer<SampleT, CoeffT, WorkT>::update_hyperparams(const json& params) {
  if (params.contains("mu")) {
    mu = params.at("mu").template get<RealT>();
  }
  if (params.contains("gamma")) {
    gamma = params.at("gamma").template get<RealT>();
  }

  std::size_t requested_order = P;
  if (params.contains("P")) {
    requested_order = params.at("P").get<std::size_t>();
  }
  if (params.contains("projection_order")) {
    requested_order = params.at("projection_order").get<std::size_t>();
  }

  if (requested_order != P) {
    P = requested_order;
    if (n_weights != 0) {
      X_hist = WorkMat::Zero(static_cast<Eigen::Index>(n_weights),
                             static_cast<Eigen::Index>(P + 1));
      d_hist = WorkVec::Zero(static_cast<Eigen::Index>(P + 1));
    }
  }
}

template <typename SampleT, typename CoeffT, typename WorkT>
typename RLSOptimizer<SampleT, CoeffT, WorkT>::AnalysisInfo
RLSOptimizer<SampleT, CoeffT, WorkT>::analyze(const WorkMat* R) const {
  AnalysisInfo result{};
  const RealT eps = std::numeric_limits<RealT>::epsilon();

  result.is_stable = lambda > RealT(0) && lambda <= RealT(1);
  result.eff_mem = lambda < RealT(1)
      ? RealT(1) / (RealT(1) - lambda)
      : std::numeric_limits<RealT>::infinity();

  if (R && R->size() != 0) {
    const RealT trace = std::real(R->trace());
    result.theor_misadj = (RealT(1) - lambda) * trace / RealT(2);
  }

  if (S_D.size() != 0) {
    Eigen::SelfAdjointEigenSolver<WorkMat> eigensolver(S_D);
    if (eigensolver.info() == Eigen::Success) {
      const auto eigenvalues = eigensolver.eigenvalues();
      const RealT min_eigenvalue = eigenvalues.minCoeff();
      const RealT max_eigenvalue = eigenvalues.maxCoeff();
      result.P_cond_num = std::abs(min_eigenvalue) > eps
          ? max_eigenvalue / min_eigenvalue
          : std::numeric_limits<RealT>::infinity();
    }
  }

  result.lam_slow = RealT(0.995);
  result.lam_fast = RealT(0.98);
  return result;
}

template <typename SampleT, typename CoeffT = SampleT, typename WorkT = SampleT>
std::unique_ptr<AdaptiveOptimizer<SampleT, CoeffT, WorkT>>
make_optimizer(const json& params) {
  const std::string type = params.value("otype", "lms");

  if (eq_nocase(type, "lms")) {
    return std::make_unique<LMSOptimizer<SampleT, CoeffT, WorkT>>(params);
  }
  if (eq_nocase(type, "nlms")) {
    return std::make_unique<NLMSOptimizer<SampleT, CoeffT, WorkT>>(params);
  }
  if (eq_nocase(type, "apa") || eq_nocase(type, "affineprojection") ||
      eq_nocase(type, "affine_projection")) {
    return std::make_unique<APAOptimizer<SampleT, CoeffT, WorkT>>(params);
  }
  if (eq_nocase(type, "signerror") || eq_nocase(type, "sign_error")) {
    return std::make_unique<SignErrorOptimizer<SampleT, CoeffT, WorkT>>(params);
  }
  if (eq_nocase(type, "signdata") || eq_nocase(type, "sign_data")) {
    return std::make_unique<SignDataOptimizer<SampleT, CoeffT, WorkT>>(params);
  }
  if (eq_nocase(type, "signsign") || eq_nocase(type, "sign_sign")) {
    return std::make_unique<SignSignOptimizer<SampleT, CoeffT, WorkT>>(params);
  }
  if (eq_nocase(type, "dualsign") || eq_nocase(type, "dual_sign")) {
    return std::make_unique<DualSignOptimizer<SampleT, CoeffT, WorkT>>(params);
  }
  if (eq_nocase(type, "powerof2") || eq_nocase(type, "power_of_two_error")) {
    return std::make_unique<PowerOfTwoErrorOptimizer<SampleT, CoeffT, WorkT>>(params);
  }
  if (eq_nocase(type, "lms_newton")) {
    return std::make_unique<LMSNewtonOptimizer<SampleT, CoeffT, WorkT>>(params);
  }
  if (eq_nocase(type, "rls")) {
    return std::make_unique<RLSOptimizer<SampleT, CoeffT, WorkT>>(params);
  }

  throw std::invalid_argument("invalid adaptive optimizer type: " + type);
}

template <typename SampleT, typename CoeffT = SampleT, typename WorkT = SampleT>
AdaptiveOptimizer<SampleT, CoeffT, WorkT>*
create_optimizer(const json& params) {
  return make_optimizer<SampleT, CoeffT, WorkT>(params).release();
}

}  // namespace adptsysc
