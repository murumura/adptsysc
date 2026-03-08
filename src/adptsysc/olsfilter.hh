#pragma once
#include <adptsysc/adptsysc.hh>
#include <Eigen/Dense>
#include <memory>
#include <adptsysc/object.hh>

namespace adptsysc {

// using E = ADPT_TARGET;

template <typename T>
Eigen::Matrix<T, Eigen::Dynamic, 1>
olsfft_conv(
  const Eigen::Matrix<T, Eigen::Dynamic, 1>& in,
  const Eigen::Matrix<T, Eigen::Dynamic, 1>& weights,
  int N,
  bool debug = false
) {
  const int L = in.size();
  const int M = weights.size();
  const int P = N - (M - 1);

  if (N < M)
    throw std::runtime_error("FFT size must be >= filter length");

  Eigen::FFT<T> fft;

  /* padded input */
  Eigen::Matrix<T, Eigen::Dynamic, 1> xpad(L + M - 1);
  xpad.setZero();
  xpad.tail(L) = in;

  /* FFT of filter */
  Eigen::Matrix<T, Eigen::Dynamic, 1> hpad =
      Eigen::Matrix<T, Eigen::Dynamic, 1>::Zero(N);
  hpad.head(M) = weights;

  std::vector<std::complex<T>> H;
  fft.fwd(H, hpad);

  Eigen::Matrix<T, Eigen::Dynamic, 1> out(L + M - 1);
  out.setZero();

  std::vector<std::complex<T>> X(N), Y(N);
  std::vector<T> y;

  int out_idx = 0;

  for (int start = 0; start < L; start += P) {

    Eigen::Matrix<T, Eigen::Dynamic, 1> block =
        xpad.segment(start, std::min(N, int(xpad.size()) - start));


    Eigen::VectorX<T> block = Eigen::VectorX<T>::Zero(N);
    block.head(std::min(N, int(xpad.size() - start))) =
      xpad.segment(start, std::min(N, int(xpad.size() - start)));

    fft.fwd(X, block);

    for (int i = 0; i < N; ++i)
      Y[i] = X[i] * H[i];

    fft.inv(y, Y);

    for (int i = M - 1; i < N && out_idx < out.size(); ++i)
      out[out_idx++] = y[i];
  }

  return out;
}

}

#endif 
