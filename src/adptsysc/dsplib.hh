#pragma once

#include <Eigen/Dense>
#include <unsupported/Eigen/FFT>
#include <algorithm>
#include <cassert>
#include <cmath>
#include <numeric>
#include <optional>
#include <ranges>
#include <set>
#include <span>
#include <stdexcept>
#include <string_view>
#include <type_traits>
#include <vector>
#include <tuple>
#include <complex>

#ifdef ENABLE_MATPLOT
#include <matplot/matplot.h>
#endif

namespace adptsysc {

template <typename T, typename = int>
struct is_complex : std::false_type {};

template <typename T>
struct is_complex<T, std::enable_if_t
    <
      std::is_same_v<decltype(std::declval<T>().real()), typename T::value_type> && 
      std::is_same_v<decltype(std::declval<T>().imag()), typename T::value_type> && 
      (sizeof(T) == 2 * sizeof(typename T::value_type))>
    >: std::true_type {};

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
  static constexpr std::array<std::string_view, 5> 
  scale_opts{"none", "biased", "unbiased", "coeff", "normalized"};

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
  std::vector<T> 
  eval(T drive_var, const int sample_size, std::optional<int> seed = std::nullopt) const;

  // Alternative version that writes to existing buffer
  void eval_to(std::span<T> output, T drive_var, std::optional<int> seed = std::nullopt) const;

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


enum class WindowType {
  BlackmanHarris,
	Hann,
	None
};

template <typename T>
Eigen::Map<const Eigen::VectorX<T>> 
map_vector_to_eigen(const std::vector<T>& v) {
  return Eigen::Map<const Eigen::VectorX<T>>(v.data(), v.size());
}

class FFT {
public:
  FFT(bool inverse, bool real) : inverse(inverse), real(real) {
    if (real && inverse) {
      throw std::invalid_argument("Real IFFT requires special handling. Use complex IFFT instead.");
    }
  }

  template <typename T>
  void eval(const std::vector<T>& in, std::vector<std::complex<T>>& out) {
    Eigen::Map<const Eigen::Matrix<T, Eigen::Dynamic, 1>> ein(in.data(), in.size());
    Eigen::Matrix<std::complex<T>, Eigen::Dynamic, 1> eout;
    eval(ein, eout);
    out.assign(eout.data(), eout.data() + eout.size());
  }

  template <typename T>
  void eval(const Eigen::Matrix<T, Eigen::Dynamic, 1>& in,
            Eigen::Matrix<std::complex<T>, Eigen::Dynamic, 1>& out) {
    using Complex = std::complex<T>;
    using ComplexVec = Eigen::Matrix<Complex, Eigen::Dynamic, 1>;

    Eigen::FFT<T> fftimpl;

    if (real && !inverse) {
      // Forward real FFT: real -> complex (N/2 + 1)
      out.resize(in.size() / 2 + 1);
      fftimpl.fwd(out, in);
    } else {
      // Complex forward/inverse FFT
      const auto* cin = reinterpret_cast<const Complex*>(in.data());
      Eigen::Map<const ComplexVec> complex_in(cin, in.size());

      out.resize(in.size());
      if (inverse)
        fftimpl.inv(out, complex_in);
      else
        fftimpl.fwd(out, complex_in);
    }
  }

  auto state() const { return std::make_tuple(inverse, real); }

private:
  bool inverse;
  bool real;
};

}  // namespace adptsysc
