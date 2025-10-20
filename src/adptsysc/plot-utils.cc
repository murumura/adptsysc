#include <adptsysc/plot-utils.hh>
#include <matplot/core/axes_type.h>

namespace matplot {

zcircle::zcircle(class axes_type* parent, const std::vector<double>& x,
    const std::vector<double>& y, const std::vector<double>& radius,
    const std::vector<double>& start_angle,
    const std::vector<double>& end_angle, const std::vector<double>& color)
    : circles(parent, x, y, radius, start_angle, end_angle, color) {

  // Set circle properties for ZePolA style - cyan unit circle
  this->face_color(color_array{0.522f, 0.471f, 0.522f, 0.9f});  // Transparent fill
  this->line_color(color_array{0.522f, 0.471f, 0.522f, 1.0f});  // Cyan line like ZePolA
  this->line_width(3.0);
  
  // Configure parent axes for dark theme
  if (auto parent_ax = this->parent()) {
    parent_ax->axis(equal);
    parent_ax->x_axis().label("Real");
    parent_ax->y_axis().label("Imaginary");
    parent_ax->title("Pole-Zero Plot");
    parent_ax->box(false);
    
    // Set dark background
    parent_ax->color(color_array{0.1f, 0.1f, 0.15f, 1.0f});
    parent_ax->x_axis().color("white");
    parent_ax->y_axis().color("white");
    parent_ax->title_color(color_array{1.0f, 1.0f, 1.0f, 1.0f});
  }
}


}  // namespace matplot