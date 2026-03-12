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
struct CrossCorrelationEval {
  std::vector<T>   corrs;
  std::vector<int> lags;
};
template <typename T> 
CrossCorrelationEval<T>
cross_correlation(const std::vector<T>& x, const std::vector<T>& y, int max_lag = -1,
                  std::string_view scale = "none", bool pos_lag = false);

template <typename T>
CrossCorrelationEval<T>
auto_correlation(const std::vector<T>& x,
                 int max_lag = -1, std::string_view scale = "none",
                 bool pos_lag = false);

template <typename T>
std::vector<T>
ar_process(std::span<const T> a, T noise_var, int N,
           std::optional<uint64_t> seed = std::nullopt);

// AR Pred struct and levinson function
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
      const float angle = (2.0f * k * kPi * n) / M;
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
      : fftsize(fftsize), inverse(inverse), real(real) {}

  int get_fftsize() const { return fftsize; }

  // Real -> Complex (forward only)
  void eval(const std::vector<float>& in, std::vector<cfloat>& out) const {
    if (inverse) {
      throw std::invalid_argument("FFT: real->complex is forward-only");
    }
    Eigen::VectorXf ein = vec2eign(in);
    Eigen::VectorXf xin = maybe_zeropad(ein);

    Eigen::VectorXcf eout;
    r2c_fwd(xin, eout);

    out = eign2vec(eout);
  }

  // Complex -> Complex (forward/inverse)
  void eval(const std::vector<cfloat>& in, std::vector<cfloat>& out) const {
    Eigen::VectorXcf ein = vec2eign_c(in);
    Eigen::VectorXcf cin = maybe_zeropad_c(ein);

    Eigen::VectorXcf eout;
    c2c(cin, eout);

    out = eign2vec(eout);
  }

  // Complex -> Real (inverse only)
  void eval(const std::vector<cfloat>& in, std::vector<float>& out) const {
    if (!inverse) {
      throw std::invalid_argument("FFT: complex->real is inverse-only");
    }

    Eigen::VectorXcf ein = vec2eign_c(in);
    Eigen::VectorXcf cin = maybe_zeropad_c(ein);

    Eigen::VectorXf eout;
    if (real) {
      // Eigen convention: full spectrum in, real time-domain out
      c2r(cin, eout);
      out = eign2vec(eout);
    } else {
      // full C2C inverse then take real part
      Eigen::VectorXcf tmp;
      c2c(cin, tmp);

      out.resize(static_cast<std::size_t>(tmp.size()));
      for (int i = 0; i < tmp.size(); ++i) 
        out[static_cast<std::size_t>(i)] = tmp[i].real();
    }
  }

 private:
  // ---- padding helpers ----
  Eigen::VectorXf maybe_zeropad(const Eigen::VectorXf& in) const {
    if (fftsize <= 0) return in;
    if (in.size() > fftsize) {
      throw std::invalid_argument("FFT: input larger than fftsize");
    }
    if (in.size() == fftsize) return in;

    Eigen::VectorXf padded = Eigen::VectorXf::Zero(fftsize);
    padded.head(in.size()) = in;
    return padded;
  }

  Eigen::VectorXcf maybe_zeropad_c(const Eigen::VectorXcf& in) const {
    if (fftsize <= 0) return in;
    if (in.size() > fftsize) {
      throw std::invalid_argument("FFT: input larger than fftsize");
    }
    if (in.size() == fftsize) return in;

    Eigen::VectorXcf padded = Eigen::VectorXcf::Zero(fftsize);
    padded.head(in.size()) = in;
    return padded;
  }

  // ---- FFT kernels ----

  // Real-to-complex forward: full N complex output (Eigen docs)
  static void r2c_fwd(const Eigen::VectorXf& in, Eigen::VectorXcf& out) {
    Eigen::FFT<float> fft;
    out.resize(in.size());
    fft.fwd(out, in);
  }

  // Complex-to-complex forward/inverse
  void c2c(const Eigen::VectorXcf& in, Eigen::VectorXcf& out) const {
    Eigen::FFT<float> fft;
    out.resize(in.size());
    if (inverse) fft.inv(out, in);
    else         fft.fwd(out, in);
  }

  // Complex-to-real inverse: expects full spectrum (Eigen docs)
  static void c2r(const Eigen::VectorXcf& fullspec, Eigen::VectorXf& out) {
    Eigen::FFT<float> fft;
    out.resize(fullspec.size());
    fft.inv(out, fullspec);
  }

  // conversions (vector <-> Eigen)
  static Eigen::VectorXf vec2eign(const std::vector<float>& v) {
    Eigen::VectorXf out(static_cast<int>(v.size()));
    for (std::size_t i = 0; i < v.size(); ++i)
      out[static_cast<int>(i)] = v[i];
    return out;
  }

  static Eigen::VectorXcf vec2eign_c(const std::vector<cfloat>& v) {
    Eigen::VectorXcf out(static_cast<int>(v.size()));
    for (std::size_t i = 0; i < v.size(); ++i)
      out[static_cast<int>(i)] = v[i];
    return out;
  }

  static std::vector<float> eign2vec(const Eigen::VectorXf& v) {
    std::vector<float> out(static_cast<std::size_t>(v.size()));
    for (int i = 0; i < v.size(); ++i)
      out[static_cast<std::size_t>(i)] = v[i];
    return out;
  }

  static std::vector<cfloat> eign2vec(const Eigen::VectorXcf& v) {
    std::vector<cfloat> out(static_cast<std::size_t>(v.size()));
    for (int i = 0; i < v.size(); ++i)
      out[static_cast<std::size_t>(i)] = v[i];
    return out;
  }

 private:
  int  fftsize;
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

int plot_psd(const PsdInfo& psd, const std::string& title  = "PSD",
              const std::string& prefix = "psd", float fs_override = -1.0f,
              bool log_freq = false, bool linear = false);

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

template <Number T>
class FirFilter {
public:
  FirFilter(const std::vector<T>& taps)
    : taps(taps), delayline(taps.size(), T(0)){}

  T process(T input) {
    // shift delay line
    for (std::size_t i = delayline.size() - 1; i > 0; --i)
      delayline[i] = delayline[i-1];
    delayline[0] = input;
    // convolution
    T y = 0;
    for (std::size_t i = 0; i < taps.size(); ++i)
      y += taps[i] * delayline[i];
    return y;
  }

  const std::vector<T>& get_taps() const{
    return taps;
  }
  const std::vector<T>& get_state() const {
    return delayline;
  }
private:
  std::vector<T> taps;
  std::vector<T> delayline;
};

std::vector<float> 
fir_lowpass_impl(std::size_t ntaps, float fc,
                const std::vector<float>& w, bool norm=true);

std::vector<float> 
fir_highpass_impl(std::size_t ntaps, float fc,
                  const std::vector<float>& w, bool norm=false);

std::vector<float> 
fir_bandpass_impl(std::size_t ntaps, float f1, float f2,
                  const std::vector<float>& w, bool norm=false);

std::vector<float> 
fir_bandstop_impl(std::size_t ntaps, float f1, float f2,
                  const std::vector<float>& w, bool norm=true);
  
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
  
  std::size_t get_nsections() const noexcept { 
    return params.sections.size(); 
  }
};

template <Number T>
T apply_biquad_sample(IirState<T>& filt, T x) {
  const std::size_t n = filt.get_nsections();

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
get_nearest_root (const std::vector<cfloat>& list, 
                  const cfloat& val, bool must_real = true);

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

int plot_zpk(const Zpk& zpk,
             const std::string& title = "ZPK",
             const std::string& prefix = "plot_zpk.png",
             const float tol=0.0001f);

RootInfo 
uniq_roots(const std::vector<cfloat> &roots, const float tol = 1e-3);

template <typename T>
T vec_foldmul(const std::vector<T>& vec, const T val = T(1)) {
  T ret = val;
  for (const auto& value : vec) { 
    ret *= value; 
  }
  return ret;
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
