#pragma once

#include <Eigen/Dense>
#include <unsupported/Eigen/FFT>
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
#include <string>
#include <tuple>
#include <type_traits>
#include <variant>
#include <vector>
#include <filesystem>
#include <concepts>
#include <iostream>

#ifdef ENABLE_MATPLOT
#include <matplot/matplot.h>
#include <adptsysc/plot-utils.hh>
#endif

namespace adptsysc {
  
namespace fs = std::filesystem;
using cfloat = std::complex<float>;

template<typename T> 
struct is_complex : std::false_type {};

template<typename T> 
struct is_complex<std::complex<T>> : std::true_type {};

template<typename T>
inline constexpr bool is_complex_v = is_complex<T>::value;

template <typename T>
concept Number = std::integral<T> || std::floating_point<T> || is_complex_v<T>;

constexpr float kPi = std::numbers::pi_v<float>;

template <typename T>
class CrossCorrelation {
 public:
  struct CrossCorrelationEval {
    std::vector<T> corrs;
    std::vector<int> lags;
  };

  CrossCorrelationEval 
  eval(const std::vector<T>& x, const std::vector<T>& y,
      int maxlag = -1, std::string_view scale = "none",
      bool pos_lag = false) const {
    if (x.empty() || y.empty()) {
      throw std::invalid_argument("CrossCorrelation: input signals must not be empty");
    }

    if (!is_supported_scale(scale)) {
      throw std::invalid_argument("CrossCorrelation: in_valid scale option");
    }

    int Nx = static_cast<int>(x.size());
    int Ny = static_cast<int>(y.size());
    int N = std::max(Nx, Ny);

    if (maxlag < 0)
      maxlag = N - 1;
    if (maxlag >= N)
      throw std::invalid_argument("CrossCorrelation: maxlag >= signal length");

    CrossCorrelationEval res;
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

  CrossCorrelationEval eval(const std::vector<T>& x, int maxlag = -1,
      std::string_view scale = "none", bool pos_lag = false) const {
    return eval(x, x, maxlag, scale, pos_lag);
  }

 private:
  static constexpr std::array<std::string_view, 5> scale_opts {
      "none", "biased", "unbiased", "coeff", "normalized"};

  static bool is_supported_scale(std::string_view scale) {
    return std::find(scale_opts.begin(), scale_opts.end(), scale)
           != scale_opts.end();
  }

  static void 
  apply_scale(CrossCorrelationEval& res, const std::vector<T>& x,
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
        throw std::runtime_error("CrossCorrelation: zero norm in coeff scaling");

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

  ARModel(const int ar_ord, 
          const std::vector<T>& params) : ar_ord(ar_ord) {
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
  eval(T drive_var, const int sample_size,
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

  YuleResult<T> res;

  // Initialize with a = [1], eta = rxx[0], rcs empty
  res.a = {T(1.0)};  // a(0) = [1]
  // eta(0) = r_0 (real part for positive-definite)
  res.eta = {std::real(rxx[0])};
  // Pre-allocate space for reflection coefficients
  res.rcs.reserve(N);

  for (int i = 1; i <= N; ++i) {
    // Compute reflection coefficient
    T numerator = T(0.0);
    for (int j = 0; j < i; ++j) {
      numerator += res.a[i - 1 - j] * rxx[j + 1];  // Reverse indexing
    }
    T rc = -numerator / res.eta.back();
    res.rcs.emplace_back(rc);

    // Update AR coefficients: a = [a, 0] + [0, rc * reversed_a]
    std::vector<T> anxt(i + 1, T(0.0));

    // [a, 0] part
    std::copy(res.a.begin(), res.a.end(), anxt.begin());

    // [0, rc * reversed_a] part
    for (int j = 0; j < i; ++j) {
      anxt[j + 1] += rc * res.a[i - 1 - j];
    }

    res.a = std::move(anxt);

    // Update error: eta = eta * (1 - |rc|^2)
    res.eta.emplace_back(res.eta.back() * (1.0 - std::norm(rc)));
  }

  return res;
}

template <typename T>
Eigen::Map<const Eigen::VectorX<T>> 
map_vector_to_eigen(const std::vector<T>& v) {
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
std::vector<float> bartlett(const std::size_t ntaps);

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

  int get_fftsize() { return fftsize; }

  // ----- Eigen::VectorXf -> Eigen::VectorXcf
  void eval(const Eigen::VectorXf& in, Eigen::VectorXcf& out) {
    if (fftsize > 0 && in.size() != fftsize) {
      Eigen::VectorXf padded = zero_pad(in, fftsize);
      eval_impl<float>(padded, out);
    } else {
      eval_impl<float>(in, out);
    }
  }

  // ----- std::vector<cfloat> -> std::vector<cfloat>
  void eval(const std::vector<cfloat>& in, std::vector<float>& out) {
    Eigen::VectorXcf ein = Eigen::Map<const Eigen::VectorXcf>(in.data(), in.size());
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
      eval_impl<cfloat>(padded, out);
    } else {
      eval_impl<cfloat>(in, out);
    }
  }

  // ----- std::vector<float> -> std::vector<cfloat>
  void eval(const std::vector<float>& in, 
            std::vector<cfloat>& out) {
    Eigen::VectorXf ein = Eigen::Map<const Eigen::VectorXf>(in.data(), in.size());
    Eigen::VectorXcf eout;
    eval(ein, eout);
    out.assign(eout.data(), eout.data() + eout.size());
  }

  // ----- std::vector<cfloat> -> std::vector<cfloat>
  void eval(const std::vector<cfloat>& in,
      std::vector<cfloat>& out) {
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

std::vector<float> fftshift_1d(const std::vector<float>& in);
std::vector<float> ifftshift_1d(const std::vector<float>& in);

using StftAnlys = std::vector<std::vector<cfloat>>;
using StftSynth = std::vector<float>;

struct StftAnlysInfo {

  StftAnlys spgram;

  // Analysis parameters
  float fs;
  std::size_t frame_size;
  std::size_t hop_size;
  std::string win_name;
  float freq_res;
  float time_res;

  // Derived metrics
  float max_freq;
  float dur_secs;
  float minval;
  float maxval;
  float mean;

  StftAnlysInfo(StftAnlys spgram, float fs, std::size_t frame_size,
      std::size_t hop_size, std::string win_name)
      : spgram(std::move(spgram)), fs(fs),
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

struct PsdInfo {
  std::vector<float> freqs;
  std::vector<float> psd;
  float fs{1.0f};
};

enum class Scale { Density, Spectrum };

// If x is real-valued, pxx is a one-sided PSD estimate. 
// If x is complex-valued, pxx is a two-sided PSD estimate.
// and will override two_side options
template <typename T> PsdInfo 
pwelch(const std::vector<T>& in, std::string_view win_name = "hann",
  int win_size = -1, int nffts = -1, int hop_size = -1, float fs = 1.0,
  WindowParams params = NoParam{}, bool detrend = true,
  Scale scale = Scale::Density, bool avg = true,
  bool two_side = false);

void plot_psd (
  const PsdInfo& psd,
  const std::string& title = "PSD",
  const std::string& fpath = "./psd_plot.png"
);

template <std::floating_point T>
std::vector<T> 
linspace(T start, T end, const std::size_t size) {
  if (size == 0) return {};
  else if (size == 1) return {start};
  T step = (end - start) / static_cast<T>(size - 1);
  auto vw  = std::views::iota(static_cast<std::size_t>(0), size) 
           | std::views::transform([start, step](std::size_t i) {
              return start + static_cast<T>(i) * step;});
  return std::vector<T>(vw.begin(), vw.end());
}

struct Zpk {
  std::vector<cfloat> zeros;
  std::vector<cfloat> poles;
  float k;
};

struct BiquadPair {
  cfloat p1, p2, z1, z2;
};

template <Number T>
struct BiquadSection {
  T a0{1}, a1{0}, a2{0};
  T b0{1}, b1{0}, b2{0};

  constexpr BiquadSection() noexcept = default;

  constexpr BiquadSection(T a0, T a1, T a2, T b0, T b1, T b2) noexcept
      : a0(a0), a1(a1), a2(a2), 
        b0(b0), b1(b1), b2(b2) {}

  constexpr BiquadSection normalized_a0() const noexcept {
    return {T(1), a1 / a0, a2 / a0, 
            b0 / a0, b1 / a0, b2 / a0};
  }

  constexpr BiquadSection normalized_b0() const noexcept {
    return {a0, a1, a2, 
            T(1), b1 / b0, b2 / b0};
  }

  constexpr BiquadSection normalized_all() const noexcept {
    T na1 = a1 / a0;
    T na2 = a2 / a0;
    T nb0 = b0 / a0;
    T nb1 = b1 / a0;
    T nb2 = b2 / a0;

    return {T(1), na1, na2, 
            T(1), nb1 / nb0, nb2 / nb0};
  }
};

template <Number T>
struct BiquadState {
  std::vector<T> s1;
  std::vector<T> s2;
  std::vector<T> out;

  explicit BiquadState(std::size_t n_filtrs = 0)
      : s1(n_filtrs, T{}), s2(n_filtrs, T{}), out(n_filtrs, T{}) {}

  void reset() noexcept {
    std::fill(s1.begin(), s1.end(), T{});
    std::fill(s2.begin(), s2.end(), T{});
    std::fill(out.begin(), out.end(), T{});
  }
};

template <Number T>
struct IirParams {
  std::vector<BiquadSection<T>> sections;

  IirParams() = default;
  explicit IirParams(std::size_t count) : sections(count) {}

  IirParams(const BiquadSection<T>* bq, std::size_t count)
    : sections(bq, bq + count) {}
  
  IirParams(const BiquadSection<T>& one) : sections(1, one) {}
  
  IirParams(std::vector<BiquadSection<T>>&& s) noexcept
    : sections(std::move(s)) {}

  template <typename Container>
  IirParams(const Container& cont)
    : sections(std::begin(cont), std::end(cont)) {}
};

template <Number T>
struct IirState {
  IirParams<T> params;
  BiquadState<T> state;

  explicit IirState(IirParams<T> p)
    : params(std::move(p)), state(params.sections.size()) {}

  void reset() noexcept { 
    state.reset(); 
  }
  
  std::size_t n_sections() const noexcept { 
    return params.sections.size(); 
  }
};

template <Number T>
T apply_biquad_sample(IirState<T>& filt, T x) {
  const std::size_t n = filt.n_sections();

  for (std::size_t i = 0; i < n; ++i) {
    const auto& sec = filt.params.sections[i];

    T y = sec.b0 * x + filt.state.s1[i];

    filt.state.s1[i] = sec.b1 * x - sec.a1 * y + filt.state.s2[i];
    filt.state.s2[i] = sec.b2 * x - sec.a2 * y;
    filt.state.out[i] = y;

    x = y;  // cascade
  }
  return x;
}

template <Number T>
void apply_biquad_block(IirState<T>& filt, std::span<T> data) {
  for (auto& x : data) {
    x = apply_biquad_sample(filt, x);
  }
}

template <Number T>
void apply_biquad_block(IirState<T>& filt, std::vector<T>& data) {
  apply_biquad_block(filt, std::span<T>(data));
}

// Take a list of complex roots and "enforce real-coefficient pairing"
// by collapsing conjugates into a single representative 
// with positive imaginary part.
std::vector<cfloat> 
pair_conjugates (const std::vector<cfloat>& list);

// Converts ZPK that in forms of biqaud pairs 
// to biquad section coefficients
template <Number T> 
BiquadSection<T> 
zpk_to_biquad (const BiquadPair& pairs, const float k) {

  // Builds coefficients [1, - (x+y), x*y] from two roots
  // assume they are both real roots or both conj-symm
  auto poly_from_roots = [&](const cfloat& r1, const cfloat& r2) {
    return std::array<float, 3>{ 
      1.0f, 
      -(r1 + r2).real(), 
      (r1 * r2).real() 
    };
  };

  auto num = poly_from_roots(pairs.z1, pairs.z2);
  auto den = poly_from_roots(pairs.p1, pairs.p2);

  return BiquadSection<T> {
    static_cast<T>(den[0]),
    static_cast<T>(den[1]),
    static_cast<T>(den[2]),
    static_cast<T>(k * num[0]),
    static_cast<T>(k * num[1]),
    static_cast<T>(k * num[2])
  };
}

// Return closest index of closest root (real or complex) 
// from a roots list.
std::size_t 
get_nearest_root (const std::vector<cfloat>& list, const cfloat& val, bool must_real = true);

// Converts a filter specified in zero-pole-gain (ZPK) form
// into second-order sections (SOS), i.e. cascaded biquad filters
// for improves numerical stability in IIR filters.
template <Number T> 
IirParams<T> zpk_to_sos(Zpk &filter);

std::ostream& operator<< (std::ostream& os, const Zpk& zpk);

Zpk butterworth(const std::size_t ntaps);
Zpk chebyshev1(const std::size_t ntaps, const float rp);

// Hold unique complex roots and their multiplicities
struct RootInfo {
  std::vector<cfloat> uniq;
  std::vector<int> mult;
};

void plot_zpk (
  const Zpk& zpk,
  const std::string& title = "PSD",
  const std::string& fpath = "./psd_zpk.png", 
  const float tol=0.0001f
);

RootInfo 
uniq_roots(const std::vector<cfloat> &roots, const float tol = 1e-3);

template <typename T>
T prod(const std::vector<T>& vec, const T val = T(1)) {
  T prod = val;
  for (const auto& value : vec) { 
    prod *= value; 
  }
  return prod;
}

float f_prewarp(float freq, float fs);

Zpk bilinear(const Zpk& proto, const float fs);

Zpk iirlp2hp_s(const Zpk& proto, const float wc);
Zpk iirlp2bp_s(const Zpk& proto, const float wc, const float bw);
Zpk iirlp2bs_s(const Zpk& proto, const float wc, const float bw);

Zpk iirlp2lp_z(const Zpk& proto, const float fc, 
              const float fs, const float fc_new, 
              const float fs_new);

Zpk iirlp2hp_z(const Zpk& proto, const float fc, 
              const float fs, const float fc_new, 
              const float fs_new);

Zpk iirlp2bp_z(const Zpk& proto, const float fc, 
              const float fs, const float fc1_new1, 
              const float fc1_new2, const float fs_new);              

Zpk iirlp2bs_z(const Zpk& proto, const float fc, 
              const float fs, const float fc1_new1, 
              const float fc1_new2, const float fs_new);
}  // namespace adptsysc
