#pragma once
#include <matplot/matplot.h>

namespace matplot {
class zcircle : public circles {
public:
  zcircle(class axes_type* parent, const std::vector<double>& x,
      const std::vector<double>& y, const std::vector<double>& radius = {},
      const std::vector<double>& start_angle = {0.0},
      const std::vector<double>& end_angle = {360},
      const std::vector<double>& color = {});

  /// Constructor from axes_handle
  template <class... Args>
  zcircle(const axes_handle& parent, Args&&... args)
    : zcircle(parent.get(), std::forward<Args>(args)...) {}
};

} // namespace matplot