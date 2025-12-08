#if ADPT_LMS

#ifndef ADPT_TARGET
#error "ADPT_TARGET must be defined before including this file"
#endif

#include <adptsysc/adptsysc.hh>
#include <Eigen/Dense>
#include <cassert>
#include <iostream>

namespace adptsysc {

using E = ADPT_TARGET;

template <typename E>
void train_lms (
  Context<E>& ctx, 
  const Eigen::VectorXf& in, 
  float desired, float mu = 0.01f
) {
  const int N = ctx.filter_len;
  assert(in.size() == N);

  float y = ctx.weights.dot(in);
  float error = desired - y;
  ctx.weights += mu * error * in;

  if constexpr (E::debug) {
    Out(ctx) << "  y = " << y 
            << ", error = " << error << "\n";
  }
}

}  // namespace adptsysc

#endif  // ADPT_LMS
