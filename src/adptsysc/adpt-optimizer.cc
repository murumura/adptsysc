#include <adptsysc/adpt-optimizer.hh>

#include <algorithm>
#include <cctype>

namespace adptsysc {

std::string to_lower(std::string value) {
  std::transform(value.begin(), value.end(), value.begin(),
                 [](unsigned char ch) { return static_cast<char>(std::tolower(ch)); });
  return value;
}

std::string to_upper(std::string value) {
  std::transform(value.begin(), value.end(), value.begin(),
                 [](unsigned char ch) { return static_cast<char>(std::toupper(ch)); });
  return value;
}

bool eq_nocase(const std::string& lhs, const std::string& rhs) {
  return to_lower(lhs) == to_lower(rhs);
}

}  // namespace adptsysc
