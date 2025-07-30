#if ADPT_LMS

#ifndef ADPT_TARGET
#error "ADPT_TARGET must be defined before including this file"
#endif

#include <adptsysc/context.hh>
#include <Eigen/Dense>
#include <cassert>
#include <iostream>

namespace adptsysc {

using E = ADPT_TARGET;

template <typename E>
void train_filter(
    Context<E>& ctx, const Eigen::VectorXf& input, float desired, float mu = 0.01f) {
  const int N = ctx.filter_len;
  assert(input.size() == N);

  float y = ctx.weights.dot(input);
  float error = desired - y;
  ctx.weights += mu * error * input;

  if constexpr (E::debug) {
    std::cout << "[" << ctx.name << "] y = " << y << ", error = " << error << "\n";
  }
}

}  // namespace adptsysc

#endif  // ADPT_LMS
