#include <adptsysc/adpt-optimizer.hh>
#include <algorithm>

namespace adptsysc {

std::string to_lower(std::string str) {
  std::transform(std::begin(str), std::end(str), std::begin(str),
                 [](unsigned char c) { return (char)std::tolower(c); });
  return str;
}

std::string to_upper(std::string str) {
  std::transform(std::begin(str), std::end(str), std::begin(str),
                 [](unsigned char c) { return (char)std::toupper(c); });
  return str;
}

bool 
eq_nocase(const std::string& s1, const std::string& s2) {
  return to_lower(s1) == to_lower(s2);
}

template <typename T, typename PARAMS_T, typename ACC_T>
json LMSOptimizer<T, PARAMS_T, ACC_T>::get_hyperparams() const {
  return {
    {"otype", "lms"},
    {"mu", mu},
    {"n_weights", n_weights}
  };
}

template <typename T, typename PARAMS_T, typename ACC_T>
void LMSOptimizer<T, PARAMS_T, ACC_T>::update_hyperparams(const json& params) {
  if (params.contains("mu")) {
    mu = params.at("mu").template get<ACC_T>();
  }
}

template <typename T, typename PARAMS_T, typename ACC_T>
typename LMSOptimizer<T, PARAMS_T, ACC_T>::AnalysisInfo
LMSOptimizer<T, PARAMS_T, ACC_T>::analyze(const DataMatrix* R) const {

  AnalysisInfo res{};

  // ---- default init (safe)
  res.is_stable = false;
  res.mu_max = ACC_T(0);
  res.mu_trace = ACC_T(0);
  res.theor_misadj = ACC_T(0);
  res.mu_conservative = ACC_T(0);
  res.mu_aggressive = ACC_T(0);

  if (!R) {
    return res;
  }

  // ---- eigenvalues of R (Hermitian)
  Eigen::SelfAdjointEigenSolver<DataMatrix> es(*R);
  const auto eig = es.eigenvalues();

  const ACC_T lam_max = eig.maxCoeff();
  const ACC_T tr = R->trace();

  // ---- stability bound (Diniz Eq 3.19)
  res.mu_max = ACC_T(1) / lam_max;
  res.is_stable = (mu > ACC_T(0) && mu < res.mu_max);

  // ---- practical trace bound
  res.mu_trace = ACC_T(1) / tr;

  // ---- misadjustment (Diniz Eq 3.30)
  const ACC_T denom = ACC_T(1) - mu * tr;
  if (denom > ACC_T(0)) {
    res.theor_misadj = (mu * tr) / denom;
  } else {
    res.theor_misadj = std::numeric_limits<ACC_T>::infinity();
  }

  // ---- recommended μ (practical design)
  res.mu_conservative = ACC_T(0.1) * res.mu_trace;
  res.mu_aggressive   = ACC_T(0.5) * res.mu_trace;

  return res;
}

template <typename T, typename PARAMS_T, typename ACC_T>
json APAOptimizer<T, PARAMS_T, ACC_T>::get_hyperparams() const {
  return {
    {"otype","apa"},
    {"mu",mu},
    {"gamma",gamma},
    {"P",P}
  };
}

template <typename T, typename PARAMS_T, typename ACC_T>
void APAOptimizer<T, PARAMS_T, ACC_T>::update_hyperparams(const json& params) {
  if (params.contains("mu")) {
    mu = params.at("mu").template get<ACC_T>();
  }
  if (params.contains("gamma")) {
    gamma = params.at("gamma").template get<ACC_T>();
  }
  if (params.contains("P")) {
    P = params.at("P").get<std::size_t>();
  }
}

template <typename T, typename PARAMS_T, typename ACC_T>
typename RLSOptimizer<T, PARAMS_T, ACC_T>::AnalysisInfo
RLSOptimizer<T, PARAMS_T, ACC_T>::analyze(const DataMatrix* R) const {

  AnalysisInfo res{};

  const ACC_T lam = lambda;

  // stability (Diniz Ch.5)
  res.is_stable = (lam > ACC_T(0) && lam <= ACC_T(1));

  if (lam < ACC_T(1)) {
    res.eff_mem = ACC_T(1) / (ACC_T(1) - lam);
  } else {
    res.eff_mem = std::numeric_limits<ACC_T>::infinity();
  }

  // misadjustment (needs R)
  if (R) {
    const ACC_T tr = R->trace();

    // Diniz approximation
    res.theor_misadj = (ACC_T(1) - lam) / ACC_T(2) * tr;
  } else {
    res.theor_misadj = ACC_T(0);
  }

  // conditioning of P = S_D
  if (S_D.size() > 0) {
    Eigen::SelfAdjointEigenSolver<DataMatrix> es(S_D);
    const auto eig = es.eigenvalues();

    const ACC_T min_eig = eig.minCoeff();
    const ACC_T max_eig = eig.maxCoeff();

    if (std::abs(min_eig) > ACC_T(1e-12)) {
      res.P_cond_num = max_eig / min_eig;
    } else {
      res.P_cond_num = std::numeric_limits<ACC_T>::infinity();
    }
  } else {
    res.P_cond_num = ACC_T(0);
  }

  res.lam_slow = ACC_T(0.995);
  res.lam_fast = ACC_T(0.98);

  return res;
}

template <typename T, typename PARAMS_T = T, typename ACC_T = float>
AdaptiveOptimizer<T, PARAMS_T, ACC_T>*
create_optimizer(const json& af_params) {
  const std::string type = af_params.value("otype", "lms");

  if (eq_nocase(type, "LMS")) {
      return new LMSOptimizer<T, PARAMS_T, ACC_T>{af_params};
  } else if (eq_nocase(type, "NLMS")) {
      return new NLMSOptimizer<T, PARAMS_T, ACC_T>{af_params};
  } else if (eq_nocase(type, "APA") || eq_nocase(type, "AffineProjection")) {
      return new APAOptimizer<T, PARAMS_T, ACC_T>{af_params};
  } else if (eq_nocase(type, "SignError")) {
      return new SignErrorOptimizer<T, PARAMS_T, ACC_T>{af_params};
  } else if (eq_nocase(type, "SignData")) {
      return new SignDataOptimizer<T, PARAMS_T, ACC_T>{af_params};
  } else if (eq_nocase(type, "SignSign")) {
      return new SignSignOptimizer<T, PARAMS_T, ACC_T>{af_params};
  } else if (eq_nocase(type, "DualSign")) {
      return new DualSignOptimizer<T, PARAMS_T, ACC_T>{af_params};
  } else if (eq_nocase(type, "POWEROF2")) {
      return new PowerOfTwoErrorOptimizer<T, PARAMS_T, ACC_T>{af_params};
  } else if (eq_nocase(type, "RLS")) {
     return new RLSOptimizer<T, PARAMS_T, ACC_T>{af_params};
  }

  throw std::runtime_error("Invalid adaptive optimizer type: " + type);
}

template AdaptiveOptimizer<float, float, float>* 
create_optimizer(const json& af_params);
}
