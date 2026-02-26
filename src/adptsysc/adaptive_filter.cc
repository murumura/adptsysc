#include <adptsysc/adaptive_filter.hh>
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
  return to_lower(str1) == to_lower(str2);
}

template <typename T>
std::unique_ptr<AdaptiveFilter<T>>
create_adaptive_filter(const json& af_params) {
  std::string type = af_params.value("otype", "LMS");

  if (equals_case_insensitive(type, "LMS")) {
    return new LMSFilter<T>{af_params};
  } else if (equals_case_insensitive(type, "NLMS")) {
    return new NLMSFilter<T>{af_params};
  } else if (equals_case_insensitive(type, "LMSNewton")) {
    return new LMSNewtonFilter<T>{af_params};
  } else if (equals_case_insensitive(type, "APA")) {
    return new APAFilter<T>{af_params};
  } else if (equals_case_insensitive(type, "RLS")) {
    return new RLSFilter<T>{af_params};
  } else if (equals_case_insensitive(type, "TransformDomain")) {
    return new TransformDomainFilter<T>{af_params};
  } else {
    throw std::runtime_error("Invalid adaptive filter type: " + type);
  }
}

template std::unique_ptr<AdaptiveFilter<float>> 
create_adaptive_filter<float>(const json&);

}