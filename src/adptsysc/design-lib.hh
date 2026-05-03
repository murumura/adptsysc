#pragma once

#include <iostream>
#include <vector>
#include <complex>
#include <algorithm>
#include <Eigen/Dense>
#include <unsupported/Eigen/FFT>
#include <optional>
#include <stdexcept>
#include <nlohmann/json.hpp>

namespace adptsysc {
using json = nlohmann::json;

template <typename T>
struct ComplexPlain {
  T re{};
  T im{};
};

template <typename T>
struct ComplexMulTLMTrans {
  ComplexPlain<T> a{};
  ComplexPlain<T> b{};
  ComplexPlain<T> y{};
};

template <typename T>
struct ShiftRegTLMTrans {
  enum class Op : uint32_t {
    STEP,
    CLEAR
  };

  Op op{Op::STEP};
  ComplexPlain<T> in{};
  ComplexPlain<T> out{};
};

template <typename T>
struct FFTFrameTxn {
  enum class Op : uint32_t {
    FFT_REAL,
    FFT_CPLX,
    IFFT_CPLX
  };

  Op op{Op::FFT_CPLX};

  std::vector<T> in_real;
  std::vector<std::complex<T>> in_cplx;
  std::vector<std::complex<T>> out_cplx;

  bool ok{false};
  std::string error;
};

template <typename T>
std::vector<T> maybe_pad_real_vec(const std::vector<T>& in, std::size_t fftsize) {
  if (fftsize == 0 || in.size() == fftsize) {
    return in;
  }
  if (in.size() > fftsize) {
    throw std::invalid_argument("FFT real input larger than fft_size");
  }

  std::vector<T> out(fftsize, T(0));
  std::copy(in.begin(), in.end(), out.begin());
  return out;
}

template <typename T>
std::vector<std::complex<T>>
maybe_pad_cplx_vec(const std::vector<std::complex<T>>& in, std::size_t fftsize) {
  if (fftsize == 0 || in.size() == fftsize) {
    return in;
  }
  if (in.size() > fftsize) {
    throw std::invalid_argument("FFT complex input larger than fft_size");
  }

  std::vector<std::complex<T>> out(fftsize, std::complex<T>(0, 0));
  std::copy(in.begin(), in.end(), out.begin());
  return out;
}

template <typename T>
json cvec_to_json(const std::vector<std::complex<T>>& v) {
  json arr = json::array();
  for (const auto& z : v) {
    arr.push_back({
      {"re", z.real()},
      {"im", z.imag()}
    });
  }
  return arr;
}

template <typename T>
std::vector<std::complex<T>> cvec_from_json(const json& j) {
  if (!j.is_array()) {
    throw std::runtime_error("complex vector json must be an array");
  }

  std::vector<std::complex<T>> v;
  v.reserve(j.size());

  for (const auto& elem : j) {
    if (!elem.is_object() || !elem.contains("re") || !elem.contains("im")) {
      throw std::runtime_error("complex vector json element must contain re/im");
    }
    v.emplace_back(elem.at("re").template get<T>(),
                   elem.at("im").template get<T>());
  }
  return v;
}

template <typename T>
std::string cx_to_string(const std::complex<T>& z) {
  std::ostringstream oss;
  oss << "(" << z.real() << ", " << z.imag() << ")";
  return oss.str();
}


enum class FFTFlowMode { DIT, DIF };

template<typename T>
class IFFT {
public:
  enum class FFTMode { Complex, Real };
  
  using CxT = std::complex<T>;
  using VecR = std::vector<T>;
  using VecC = std::vector<CxT>;

  virtual ~IFFT() = default;

  virtual std::size_t get_fftsize() const = 0;

  virtual void fftreal(const VecR& in, VecC& out) const = 0;
  virtual void fftcplx(const VecC& in, VecC& out) const = 0;
  virtual void ifftcplx(const VecC& in, VecC& out) const = 0;
};

template<typename T>
class EigenFFTWrapper : public IFFT<T> {
public:
  using CxT = std::complex<T>;
  using VecR = std::vector<T>;
  using VecC = std::vector<CxT>;
  using FFTMode = typename IFFT<T>::FFTMode;
  
  explicit EigenFFTWrapper(const FFTMode mode, const std::size_t fftsize)
    : fftmode(mode), fftsize(fftsize) {}

  std::size_t get_fftsize() const override { return fftsize; }

  // ============================
  // Real → Complex FFT
  // ============================
  void runfft(const VecR& in, VecC& out) const {
    if (fftmode != FFTMode::Real)
      throw std::invalid_argument("runfft(real): requires Real mode");

    Eigen::Map<const Eigen::Matrix<T, -1, 1>> input_time(in.data(), in.size());
    auto padded = maybe_pad(input_time);

    Eigen::Matrix<CxT, -1, 1> output_freq;
    fft.fwd(output_freq, padded);

    out.assign(output_freq.data(), output_freq.data() + output_freq.size());
  }

  // ============================
  // Complex → Complex FFT
  // ============================
  void runfft(const VecC& in, VecC& out) const {
    Eigen::Map<const Eigen::Matrix<CxT, -1, 1>> input_time(in.data(), in.size());
    auto padded = maybe_pad(input_time);

    Eigen::Matrix<CxT, -1, 1> output_freq;
    fft.fwd(output_freq, padded);

    out.assign(output_freq.data(), output_freq.data() + output_freq.size());
  }

  // ============================
  // Complex → Complex IFFT
  // ============================
  void runifft(const VecC& in, VecC& out) const {
    Eigen::Map<const Eigen::Matrix<CxT, -1, 1>> input_freq(in.data(), in.size());
    auto padded = maybe_pad(input_freq);

    Eigen::Matrix<CxT, -1, 1> output_time;
    fft.inv(output_time, padded);

    out.assign(output_time.data(), output_time.data() + output_time.size());
  }

  // ============================
  // Complex → Real IFFT
  // ============================
  void runifft(const VecC& in, VecR& out) const {
    if (fftmode != FFTMode::Real)
      throw std::invalid_argument("runifft(real): requires Real mode");

    Eigen::Map<const Eigen::Matrix<CxT, -1, 1>> input_freq(in.data(), in.size());
    auto padded = maybe_pad(input_freq);

    Eigen::Matrix<T, -1, 1> output_time;
    fft.inv(output_time, padded);

    out.assign(output_time.data(), output_time.data() + output_time.size());
  }

  void fftreal(const VecR& in, VecC& out) const override {
    runfft(in, out);
  }

  void fftcplx(const VecC& in, VecC& out) const override {
    runfft(in, out);
  }

  void ifftcplx(const VecC& in, VecC& out) const override {
    runifft(in, out);
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

  mutable Eigen::FFT<T> fft;
};

template <typename T>
Eigen::Matrix<T, Eigen::Dynamic, 1> 
olsfft_conv(
  const Eigen::Ref<const Eigen::Matrix<T, Eigen::Dynamic, 1>>& x,
  const Eigen::Ref<const Eigen::Matrix<T, Eigen::Dynamic, 1>>& h,
  const int N,
  const bool debug = false
) {
  using CxT = std::complex<T>;

  const int L = static_cast<int>(x.size());
  const int M = static_cast<int>(h.size());

  if (N < M) {
    throw std::runtime_error("FFT size N must be >= filter length M");
  }

  const int P = N - (M - 1);

  if (debug) {
    std::cout << "[OLS] L=" << L << " M=" << M << " N=" << N << " P=" << P << "\n";
  }

  EigenFFTWrapper<T> fft(EigenFFTWrapper<T>::FFTMode::Complex, N);

  // Result container (Linear convolution length is L + M - 1)
  const int out_target_size = L + M - 1;
  Eigen::Matrix<T, -1, 1> xpad = Eigen::Matrix<T, -1, 1>::Zero(out_target_size);

  // Prepare Padded Input: [zeros(M-1), input]
  // Total size: (M-1) + L
  xpad.segment(M - 1, L) = x;

  // Prepare filter FFT
  std::vector<CxT> h_vec(N, CxT(0,0));
  for (int i = 0; i < M; ++i)
    h_vec[i] = h[i];

  std::vector<CxT> H;
  fft.runfft(h_vec, H);

  Eigen::Matrix<T, -1, 1> result = Eigen::Matrix<T, -1, 1>::Zero(L + M - 1);

  std::vector<CxT> X(N), Y(N);
  std::vector<CxT> y_time;

  int write_ptr = 0;

  // Block processing
  for (int start = 0; start < L + M - 1; start += P) {
    std::vector<CxT> block(N, CxT(0,0));
    // Calculate how much data is available in xpad starting at 'start'
    int available = static_cast<int>(xpad.size()) - start;
    int take = std::min(N, available);

    for (int i = 0; i < take; ++i)
      block[i] = xpad[start + i];

    // FFT
    fft.runfft(block, X);

    // Frequency domain multiplication
    for (int i = 0; i < N; ++i)
      Y[i] = X[i] * H[i];

    fft.runifft(Y, y_time);

    // Overlap-Save: Discard first M-1 samples, keep the rest
    // Valid samples are from index M-1 to N-1
    for (int i = M - 1; i < N && write_ptr < result.size(); ++i) {
      result[write_ptr++] = static_cast<T>(std::real(y_time[i]));
    }
  }

  return result;
}

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
