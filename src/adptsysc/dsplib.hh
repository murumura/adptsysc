#pragma once

#include <Eigen/Dense>
#include <algorithm>
#include <cassert>
#include <cmath>
#include <complex>
#include <numeric>
#include <optional>
#include <ranges>
#include <set>
#include <span>
#include <stdexcept>
#include <string_view>
#include <tuple>
#include <type_traits>
#include <unsupported/Eigen/FFT>
#include <variant>
#include <vector>

#ifdef ENABLE_MATPLOT
#include <matplot/matplot.h>
#endif

namespace adptsysc {

template <typename T, typename = int>
struct is_complex : std::false_type {};

template <typename T>
struct is_complex<T,
    std::enable_if_t<std::is_same_v<decltype(std::declval<T>().real()),
                         typename T::value_type>
                     && std::is_same_v<decltype(std::declval<T>().imag()),
                         typename T::value_type>
                     && (sizeof(T) == 2 * sizeof(typename T::value_type))>>
    : std::true_type {};

template <class T>
inline constexpr bool is_complex_v = is_complex<T>::value;

template <typename T>
class Xcorr {
 public:
  struct CorrEval {
    std::vector<T> corrs;
    std::vector<int> lags;
  };

  CorrEval eval(const std::vector<T>& x, const std::vector<T>& y,
      int maxlag = -1, std::string_view scale = "none",
      bool pos_lag = false) const {
    if (x.empty() || y.empty()) {
      throw std::invalid_argument("Xcorr: input signals must not be empty");
    }

    if (!is_supported_scale(scale)) {
      throw std::invalid_argument("Xcorr: invalid scale option");
    }

    int Nx = static_cast<int>(x.size());
    int Ny = static_cast<int>(y.size());
    int N = std::max(Nx, Ny);

    if (maxlag < 0)
      maxlag = N - 1;
    if (maxlag >= N)
      throw std::invalid_argument("Xcorr: maxlag >= signal length");

    CorrEval res;
    res.lags.resize(2 * maxlag + 1);
    res.corrs.resize(2 * maxlag + 1, T(0));

    for (int i = 0; i <= 2 * maxlag; ++i)
      res.lags[i] = i - maxlag;

    for (int k = -maxlag; k <= maxlag; ++k) {
      T sum = T(0);
      for (int i = 0; i < Nx; ++i) {
        int j = i + k;
        if (j >= 0 && j < Ny) {
          sum += x[i] * y[j];
        }
      }
      res.corrs[k + maxlag] = sum;
    }

    apply_scale(res, x, y, scale);

    std::reverse(res.corrs.begin(), res.corrs.end());
    std::reverse(res.lags.begin(), res.lags.end());
    // return postive lags only
    if (pos_lag) {
      auto it = std::ranges::find_if(res.lags, [](int lag) { return lag < 0; });
      if (it != res.lags.end()) {
        auto idx = std::ranges::distance(res.lags.begin(), it);
        res.lags.erase(it, res.lags.end());
        res.corrs.erase(res.corrs.begin() + idx, res.corrs.end());
      }
    }
    return res;
  }

  CorrEval eval(const std::vector<T>& x, int maxlag = -1,
      std::string_view scale = "none", bool pos_lag = false) const {
    return eval(x, x, maxlag, scale, pos_lag);
  }

 private:
  static constexpr std::array<std::string_view, 5> scale_opts{
      "none", "biased", "unbiased", "coeff", "normalized"};

  static bool is_supported_scale(std::string_view scale) {
    return std::find(scale_opts.begin(), scale_opts.end(), scale)
           != scale_opts.end();
  }

  static void apply_scale(CorrEval& res, const std::vector<T>& x,
      const std::vector<T>& y, std::string_view scale) {
    const int N = std::max(x.size(), y.size());
    const int maxlag = static_cast<int>(res.lags.size() / 2);

    if (scale == "biased") {
      for (T& c : res.corrs)
        c /= static_cast<T>(N);
    } else if (scale == "unbiased") {
      for (int k = -maxlag; k <= maxlag; ++k) {
        int denom = N - std::abs(k);
        res.corrs[k + maxlag] /= (denom > 0 ? denom : 1);
      }
    } else if (scale == "coeff" || scale == "normalized") {
      T xpower = std::accumulate(x.begin(), x.end(), T(0),
          [](T acc, T val) { return acc + val * val; });
      T ypower = std::accumulate(y.begin(), y.end(), T(0),
          [](T acc, T val) { return acc + val * val; });

      T norm = std::sqrt(xpower) * std::sqrt(ypower);
      if (norm == T(0))
        throw std::runtime_error("Xcorr: zero norm in coeff scaling");

      for (T& c : res.corrs)
        c /= norm;
    }
  }
};

template <typename T>
class ARModel {
 public:
  ARModel(const int ar_ord) : ar_ord(ar_ord) {
    // Initialize with ar_ord + 1 elements (including a_0)
    a_params.resize(ar_ord + 1, T(0));
    // Typically a_0 = 1 for AR models
    a_params[0] = T(1);
  }

  ARModel(const int ar_ord, const std::vector<T>& params) : ar_ord(ar_ord) {
    set_params(params);
  }

  // Set AR coefficients (excluding a_0)
  void set_params(std::span<const T> params) {
    if (params.size() != ar_ord) {
      throw std::invalid_argument("Coefficient count must match AR order");
    }
    std::copy(params.begin(), params.end(), a_params.begin() + 1);
  }

  // Generate AR process samples
  std::vector<T> eval(T drive_var, const int sample_size,
      std::optional<int> seed = std::nullopt) const;

  // Alternative version that writes to existing buffer
  void eval_to(std::span<T> output, T drive_var,
      std::optional<int> seed = std::nullopt) const;

  int ar_ord;
  std::vector<T> a_params;
};

// ARPred struct and levinson function (as provided)
template <typename T>
struct YuleResult {
  std::vector<T> a;         // a_0 to a_N
  std::vector<double> eta;  // ε_0 to ε_N
  std::vector<T> rcs;       // k_1 to k_N
};

template <typename T>
YuleResult<T> 
levinson(const std::vector<T>& rxx, int N) {
  // Check input size
  if (rxx.size() != N + 1) {
    throw std::invalid_argument("Expected rxx of length "
                                + std::to_string(N + 1) + ", got "
                                + std::to_string(rxx.size()));
  }

  YuleResult<T> result;

  // Initialize with a = [1], eta = rxx[0], rcs empty
  result.a = {T(1.0)};  // a(0) = [1]
  // eta(0) = r_0 (real part for positive-definite)
  result.eta = {std::real(rxx[0])};
  // Pre-allocate space for reflection coefficients
  result.rcs.reserve(N);

  for (int i = 1; i <= N; ++i) {
    // Compute reflection coefficient
    T numerator = T(0.0);
    for (int j = 0; j < i; ++j) {
      numerator += result.a[i - 1 - j] * rxx[j + 1];  // Reverse indexing
    }
    T rc = -numerator / result.eta.back();
    result.rcs.push_back(rc);

    // Update AR coefficients: a = [a, 0] + [0, rc * reversed_a]
    std::vector<T> anxt(i + 1, T(0.0));

    // [a, 0] part
    std::copy(result.a.begin(), result.a.end(), anxt.begin());

    // [0, rc * reversed_a] part
    for (int j = 0; j < i; ++j) {
      anxt[j + 1] += rc * result.a[i - 1 - j];
    }

    result.a = std::move(anxt);

    // Update error: eta = eta * (1 - |rc|^2)
    result.eta.push_back(result.eta.back() * (1.0 - std::norm(rc)));
  }

  return result;
}

template <typename T>
Eigen::Map<const Eigen::VectorX<T>> map_vector_to_eigen(
    const std::vector<T>& v) {
  return Eigen::Map<const Eigen::VectorX<T>>(v.data(), v.size());
}

enum class WindowType {
  Rect,
  Blackman,
  Blackman2,
  Blackman3,
  Blackman4,
  BlackmanHarris,
  Hann,
  Hamming,
  Kaiser,
  None
};

template <std::size_t N>
std::vector<float> 
coswindow(int ntaps, const std::array<float, N>& coeffs) {
  std::vector<float> taps(ntaps);
  const float M = static_cast<float>(ntaps - 1);

  for (int n = 0; n < ntaps; n++) {
    float sum = 0.0f;
    // Replace accumulate with manual loop
    for (std::size_t k = 0; k < N; k++) {
      const float sign = (k % 2) ? -1.0f : 1.0f;
      const float angle = (2.0f * k * M_PI * n) / M;
      sum += sign * coeffs[k] * std::cos(angle);
    }
    taps[n] = sum;
  }
  return taps;
}

// Default beta value for Kaiser window (typical value for good balance)
static constexpr double kDefaultKaiserBeta = 3.0;
static constexpr int kDefaultBlackHarrisAtten = 92;
std::vector<float> hann(const std::size_t ntaps);
std::vector<float> rect(const std::size_t ntaps);
std::vector<float> hamming(const std::size_t ntaps);
std::vector<float> blackman(const std::size_t ntaps);
std::vector<float> blackman2(const std::size_t ntaps);
std::vector<float> blackman3(const std::size_t ntaps);
std::vector<float> blackman4(const std::size_t ntaps);
std::vector<float> blackman_harris(const std::size_t ntaps, int atten);
std::vector<float> kaiser(const std::size_t ntaps, double beta);

// Parameters for different window types
struct NoParam {};  // For windows needing no parameters

struct AttenParam {
  int atten;
};

struct KaiserParam {
  double beta;
};

using WindowParams = std::variant<NoParam, AttenParam, KaiserParam>;

std::vector<float> 
get_window(std::string_view name, const std::size_t ntaps,
  WindowParams params = NoParam{}, bool norm = false);

class FFT {
 public:
  FFT(bool inverse, bool real, int fftsize = -1)
      : inverse(inverse), real(real), fftsize(fftsize) {
    if (real && inverse) {
      throw std::invalid_argument(
          "Real IFFT requires special handling. Use complex IFFT instead.");
    }
  }

  int fft_size() { return fftsize; }

  // ----- Eigen::VectorXf -> Eigen::VectorXcf
  void eval(const Eigen::VectorXf& in, Eigen::VectorXcf& out) {
    if (fftsize > 0 && in.size() != fftsize) {
      Eigen::VectorXf padded = zero_pad(in, fftsize);
      eval_impl<float>(padded, out);
    } else {
      eval_impl<float>(in, out);
    }
  }

  // ----- std::vector<std::complex<float>> -> std::vector<std::complex<float>>
  void eval(const std::vector<std::complex<float>>& in, std::vector<float>& out) {
    Eigen::VectorXcf ein
        = Eigen::Map<const Eigen::VectorXcf>(in.data(), in.size());
    Eigen::VectorXf eout;
    eval(ein, eout);
    out.assign(eout.data(), eout.data() + eout.size());
  }

  void eval(const Eigen::VectorXcf& in, Eigen::VectorXf& out) {
    if (fftsize > 0 && in.size() != fftsize) {
      Eigen::VectorXcf padded = zero_pad(in, fftsize);
      eval_impl(padded, out);
    } else {
      eval_impl(in, out);
    }
  }
  
  // ----- Eigen::VectorXcf -> Eigen::VectorXcf
  void eval(const Eigen::VectorXcf& in, Eigen::VectorXcf& out) {
    if (fftsize > 0 && in.size() != fftsize) {
      Eigen::VectorXcf padded = zero_pad(in, fftsize);
      eval_impl<std::complex<float>>(padded, out);
    } else {
      eval_impl<std::complex<float>>(in, out);
    }
  }

  // ----- std::vector<float> -> std::vector<std::complex<float>>
  void eval(const std::vector<float>& in, 
            std::vector<std::complex<float>>& out) {
    Eigen::VectorXf ein = Eigen::Map<const Eigen::VectorXf>(in.data(), in.size());
    Eigen::VectorXcf eout;
    eval(ein, eout);
    out.assign(eout.data(), eout.data() + eout.size());
  }

  // ----- std::vector<std::complex<float>> -> std::vector<std::complex<float>>
  void eval(const std::vector<std::complex<float>>& in,
      std::vector<std::complex<float>>& out) {
    Eigen::VectorXcf ein
        = Eigen::Map<const Eigen::VectorXcf>(in.data(), in.size());
    Eigen::VectorXcf eout;
    eval(ein, eout);
    out.assign(eout.data(), eout.data() + eout.size());
  }

 private:
  template <typename T>
  Eigen::Matrix<T, Eigen::Dynamic, 1> 
  zero_pad(const Eigen::Matrix<T, Eigen::Dynamic, 1>& in, int target_size) {
    if (target_size <= in.size()) {
      throw std::invalid_argument(
          "Target size must be larger than input size for zero padding");
    }

    Eigen::Matrix<T, Eigen::Dynamic, 1> padded
        = Eigen::Matrix<T, Eigen::Dynamic, 1>::Zero(target_size);
    padded.head(in.size()) = in;
    return padded;
  }

  template <typename T>
  void eval_impl(const Eigen::Matrix<T, Eigen::Dynamic, 1>& in,
    Eigen::Matrix<std::complex<typename Eigen::NumTraits<T>::Real>,
        Eigen::Dynamic, 1>& out) {
  using R = typename Eigen::NumTraits<T>::Real;
  using Complex = std::complex<R>;
  using ComplexVec = Eigen::Matrix<Complex, Eigen::Dynamic, 1>;

  Eigen::FFT<R> fftimpl;

  if (real && !inverse) {
    // Real-to-complex forward
    out.resize(in.size());
    fftimpl.fwd(out, in);
  } else if (real && inverse) {
    // Complex-to-real inverse (C2R)
    const auto* cptr = reinterpret_cast<const Complex*>(in.data());
    Eigen::Map<const ComplexVec> cin(cptr, in.size());

    Eigen::Matrix<R, Eigen::Dynamic, 1> real_out;
    fftimpl.inv(real_out, cin);
    
    // Convert real output to complex (imag = 0)
    out = real_out.template cast<Complex>();
  } else {
    // Complex-to-complex forward/inverse
    const auto* cptr = reinterpret_cast<const Complex*>(in.data());
    Eigen::Map<const ComplexVec> cin(cptr, in.size());
    out.resize(in.size());
    if (inverse)
      fftimpl.inv(out, cin);
    else
      fftimpl.fwd(out, cin);
  }
 }

  // Complex-to-real inverse FFT
  void eval_impl(const Eigen::VectorXcf& in, Eigen::VectorXf& out) {
    Eigen::FFT<float> fftimpl;
    fftimpl.inv(out, in);  // Use Eigen's inverse FFT from complex to real
  }

  int fftsize;
  bool inverse;
  bool real;
};

using StftAnlys = std::vector<std::vector<std::complex<float>>>;
using StftSynth = std::vector<float>;

struct StftAnlysInfo {
  // Core data
  StftAnlys spgram;

  // Analysis parameters
  float sample_rate;
  std::size_t frame_size;
  std::size_t hop_size;
  std::string win_name;
  float freq_res;
  float time_res;

  // Derived metrics
  float max_freq;
  float dursecs;
  float minval;
  float maxval;
  float mean;

  StftAnlysInfo(StftAnlys spgram, float sample_rate, std::size_t frame_size,
      std::size_t hop_size, std::string win_name)
      : spgram(std::move(spgram)), sample_rate(sample_rate),
        frame_size(frame_size), hop_size(hop_size),
        win_name(std::move(win_name)) {
    derived_prop();
  }
  void derived_prop();
  friend std::ostream& operator<<(std::ostream& os, const StftAnlysInfo& info);
};

StftAnlys
stft_analysis(const std::vector<float>& in, std::size_t frame_size,
    std::size_t hop_size, std::string_view win_name = "hann",
    WindowParams params = NoParam{});


// Synthesize signal (time-frequency to time-domain)
StftSynth
stft_synth(const StftAnlys &spgram, std::size_t frame_size,
    std::size_t hop_size, std::string_view win_name = "hann",
    WindowParams params = NoParam{});

}  // namespace adptsysc
