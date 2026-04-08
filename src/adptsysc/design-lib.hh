#pragma once

#include <iostream>
#include <vector>
#include <complex>
#include <algorithm>
#include <Eigen/Dense>
#include <unsupported/Eigen/FFT>
#include <optional>
#include <stdexcept>

namespace adptsysc {

// using E = ADPT_TARGET;

template <typename T>
Eigen::Matrix<T, Eigen::Dynamic, 1> 
olsfft_conv(
  const Eigen::Ref<const Eigen::Matrix<T, Eigen::Dynamic, 1>>& x,
  const Eigen::Ref<const Eigen::Matrix<T, Eigen::Dynamic, 1>>& h,
  const int N,
  const bool debug = false
) {
  const int L = static_cast<int>(x.size());
  const int M = static_cast<int>(h.size());
  
  if (N < M) {
    throw std::runtime_error("FFT size N must be >= filter length M");
  }

  const int P = N - (M - 1);

  if (debug) {
    std::cout << "[OLS] L=" << L << " M=" << M << " N=" << N << " P=" << P << "\n";
  }

  Eigen::FFT<T> fft;
  
  // 1. Prepare Padded Input: [zeros(M-1), input]
  // Total size: (M-1) + L
  Eigen::Matrix<T, Eigen::Dynamic, 1> xpad = Eigen::Matrix<T, Eigen::Dynamic, 1>::Zero(L + M - 1);
  xpad.segment(M - 1, L) = x;

  Eigen::Matrix<T, Eigen::Dynamic, 1> hpad = Eigen::Matrix<T, Eigen::Dynamic, 1>::Zero(N);
  hpad.head(M) = h;
  Eigen::Matrix<std::complex<T>, Eigen::Dynamic, 1> H(N);
  fft.fwd(H, hpad);

  Eigen::Matrix<std::complex<T>, Eigen::Dynamic, 1> X(N);
  Eigen::Matrix<T, Eigen::Dynamic, 1> y(N);
  Eigen::Matrix<T, Eigen::Dynamic, 1> block(N);

  // Result container (Linear convolution length is L + M - 1)
  const int out_target_size = L + M - 1;
  Eigen::Matrix<T, Eigen::Dynamic, 1> result = Eigen::Matrix<T, Eigen::Dynamic, 1>::Zero(out_target_size);
  int write_ptr = 0;

  // 3. Process blocks
  // Python: for start in range(0, len(x), P)
  for (int start = 0; start < L + M - 1; start += P) {
    block.setZero();
    
    // Calculate how much data is available in xpad starting at 'start'
    int available = static_cast<int>(xpad.size()) - start;
    int take = std::min(N, available);
    
    block.head(take) = xpad.segment(start, take);

    // Frequency domain multiplication
    fft.fwd(X, block);
    X = X.cwiseProduct(H);
    fft.inv(y, X);

    // 4. Overlap-Save: Discard first M-1 samples, keep the rest
    // Valid samples are from index M-1 to N-1
    for (int i = M - 1; i < N && write_ptr < out_target_size; ++i) {
      result[write_ptr++] = y[i];
    }
  }

  return result;
}

template<typename T>
class FFTWrapper {
public:
  using Scalar = T;
  using CxT = std::complex<T>;
  using VecR = std::vector<T>;
  using VecC = std::vector<CxT>;

  enum class FFTMode {
    Complex,
    Real
  };

  explicit FFTWrapper(FFTMode mode, std::size_t fftsize) : fftmode(mode), fftsize(fftsize) {}

  std::size_t get_fftsize() const noexcept { return fftsize; }
  FFTMode get_fftmode() const noexcept { return fftmode; }

  // ============================
  // Real → Complex
  // ============================
  void runfft(const VecR& in, VecC& out) const {
    if (fftmode != FFTMode::Real)
      throw std::invalid_argument("runfft(real): requires Real mode");

    Eigen::FFT<T> fft;

    Eigen::Map<const Eigen::Matrix<T, -1, 1>> xin(in.data(), in.size());
    auto xpad = maybe_pad(xin);

    Eigen::Matrix<CxT, -1, 1> eout;
    fft.fwd(eout, xpad);

    out.resize(eout.size());
    Eigen::Map<Eigen::Matrix<CxT, -1, 1>>(out.data(), out.size()) = eout;
  }

  // ============================
  // Complex → Complex
  // ============================
  void runfft(const VecC& in, VecC& out) const {
    Eigen::FFT<T> fft;

    Eigen::Map<const Eigen::Matrix<CxT, -1, 1>> xin(in.data(), in.size());
    auto xpad = maybe_pad(xin);

    Eigen::Matrix<CxT, -1, 1> eout;
    fft.fwd(eout, xpad);

    out.resize(eout.size());
    Eigen::Map<Eigen::Matrix<CxT, -1, 1>>(out.data(), out.size()) = eout;
  }

  // ============================
  // Complex → Complex inverse
  // ============================
  void runifft(const VecC& in, VecC& out) const {
    Eigen::FFT<T> fft;

    Eigen::Map<const Eigen::Matrix<CxT, -1, 1>> xin(in.data(), in.size());
    auto xpad = maybe_pad(xin);

    Eigen::Matrix<CxT, -1, 1> eout;
    fft.inv(eout, xpad);

    out.resize(eout.size());
    Eigen::Map<Eigen::Matrix<CxT, -1, 1>>(out.data(), out.size()) = eout;
  }

  // ============================
  // Complex → Real inverse
  // ============================
  void runifft(const VecC& in, VecR& out) const {
    if (fftmode != FFTMode::Real)
      throw std::invalid_argument("runifft(real): requires Real mode");

    Eigen::FFT<T> fft;

    Eigen::Map<const Eigen::Matrix<CxT, -1, 1>> xin(in.data(), in.size());
    auto xpad = maybe_pad(xin);

    Eigen::Matrix<T, -1, 1> eout;
    fft.inv(eout, xpad);

    out.resize(eout.size());
    Eigen::Map<Eigen::Matrix<T, -1, 1>>(out.data(), out.size()) = eout;
  }

private:
  template<typename Derived>
  Eigen::Matrix<typename Derived::Scalar, -1, 1>
  maybe_pad(const Eigen::MatrixBase<Derived>& in) const {
    if (fftsize <= 0 || in.size() == fftsize)
      return in;

    if (in.size() > fftsize)
      throw std::invalid_argument("FFT: input larger than fftsize");

    Eigen::Matrix<typename Derived::Scalar, -1, 1>
      out = Eigen::Matrix<typename Derived::Scalar, -1, 1>::Zero(fftsize);

    out.head(in.size()) = in;
    return out;
  }

private:
  FFTMode fftmode;
  std::size_t fftsize;
};

template <typename T>
class Serial2Parallel {
public:
  explicit Serial2Parallel(const std::size_t width)
    : W(width), buffer(width), idx(0) {}

  void reset() {
    idx = 0;
  }

  // Push one sample
  // Return full vector when ready
  std::optional<std::vector<T>> push(const T& sample) {
    buffer[idx++] = sample;

    if (idx == W) {
      idx = 0;
      return buffer;
    }
    return std::nullopt;
  }

private:
  std::size_t W;
  std::vector<T> buffer;
  std::size_t idx;
};

template <typename T>
class Parallel2Serial {
public:
  Parallel2Serial() = default;

  void load(const std::vector<T>& vec) {
    buffer = vec;
    idx = 0;
  }

  bool has_next() const {
    return idx < buffer.size();
  }

  T next() {
    assert(has_next());
    return buffer[idx++];
  }

private:
  std::vector<T> buffer;
  std::size_t idx{0};
};

}
