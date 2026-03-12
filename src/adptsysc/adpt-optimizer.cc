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
equals_case_insensitive(const std::string& s1, const std::string& s2) {
  return to_lower(s1) == to_lower(s2);
}

template <typename T, typename PARAMS_T, typename ACC_T>
json LMSOptimizer<T, PARAMS_T, ACC_T>::hyperparams() const {
  json j;
  j["otype"] = "lms";        
  j["mu"]    = mu;            
  j["n_weights"] = n_weights; 
  return j;
}

template <typename T, typename PARAMS_T, typename ACC_T>
void LMSOptimizer<T, PARAMS_T, ACC_T>::update_hyperparams(const json& params) {
  if (params.contains("mu")) {
    mu = static_cast<ACC_T>(params.at("mu").get<float>());
  }
}

template <typename T>
AdaptiveOptimizer<T>* 
create_optimizer(const json& af_params) {
  std::string type = af_params.value("otype", "LMS");

  if (equals_case_insensitive(type, "LMS")) {
    return new LMSOptimizer<T>{af_params};
 // } else if (equals_case_insensitive(type, "APA")) {
    //return new APAOptimizer<T>{af_params};
 // } else if (equals_case_insensitive(type, "NLMS")) {
 //   return new NLMSOptimizer<T>{af_params};
 // } else if (equals_case_insensitive(type, "LMSNewton")) {
 //   return new LMSNewtonOptimizer<T>{af_params};
 // } else if (equals_case_insensitive(type, "RLS")) {
 //   return new RLSOptimizer<T>{af_params};
 // } else if (equals_case_insensitive(type, "TransformDomain")) {
 //   return new TransformDomainOptimizer<T>{af_params};
  } else {
    throw std::runtime_error("Invalid adaptive filter type: " + type);
  }
}

template AdaptiveOptimizer<float>* 
create_optimizer(const json& af_params);

}
