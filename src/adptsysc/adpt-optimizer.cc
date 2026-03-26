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
  //} else if (eq_nocase(type, "SignSign")) {
  //    return new SignSignOptimizer<T, PARAMS_T, ACC_T>{af_params};
  //} else if (eq_nocase(type, "DualSign")) {
  //    return new DualSignOptimizer<T, PARAMS_T, ACC_T>{af_params};
  //} else if (eq_nocase(type, "POWEROF2")) {
  //    return new PowerOfTwoErrorOptimizer<T, PARAMS_T, ACC_T>{af_params};
  //} else if (eq_nocase(type, "LMSNewton") || eq_nocase(type, "lmsnewton")) {
  //    return new LMSNewtonOptimizer<T, PARAMS_T, ACC_T>{af_params};
  //} else if (eq_nocase(type, "TransformDomain") || eq_nocase(type, "TDLMS")) {
  //    return new TransformDomainOptimizer<T, PARAMS_T, ACC_T>{af_params};
  //} else if (eq_nocase(type, "RLS")) {
  //   return new RLSOptimizer<T, PARAMS_T, ACC_T>{af_params};
  }

  throw std::runtime_error("Invalid adaptive optimizer type: " + type);
}

template AdaptiveOptimizer<float, float, float>* 
create_optimizer(const json& af_params);
}
