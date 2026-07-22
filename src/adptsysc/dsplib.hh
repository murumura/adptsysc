#pragma once

#include <Eigen/Dense>
#include <unsupported/Eigen/FFT>
#include <algorithm>
#include <array>
#include <cassert>
#include <cmath>
#include <complex>
#include <numeric>
#include <numbers>
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
#include <iterator>
#include <limits>
#include <utility>

namespace adptsysc {
  
namespace fs = std::filesystem;
using cfloat = std::complex<float>;

template<typename T> 
struct is_complex : std::false_type {};

template<typename T> 
struct is_complex<std::complex<T>> : std::true_type {};

template<typename T>
inline constexpr bool is_complex_v = is_complex<T>::value;

template<typename T>
struct scalar_of {
  using type = T;
};

template<typename T>
struct scalar_of<std::complex<T>> {
  using type = T;
};


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

// Default beta value for Kaiser window (typical value for good balance)
static constexpr double kDefaultKaiserBeta = 3.0;
static constexpr int kDefaultBlackHarrisAtten = 92;

std::vector<float> coswindow(int ntaps, std::span<const float> coeffs);
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

  // Normalize the denominator leading coefficient without changing H(z).
  // Runtime SOS filters require this form because the DF-II-T recurrence
  // assumes a0 == 1.
  [[nodiscard]] constexpr BiquadSection normalized_a0() const noexcept {
    return {T(1), a1 / a0, a2 / a0,
            b0 / a0, b1 / a0, b2 / a0};
  }

  // This is useful only when comparing numerator shapes. It intentionally
  // extracts b0 as an external gain, so do not pass the result to IirFilter.
  [[nodiscard]] constexpr BiquadSection normalized_b0() const noexcept {
    return {a0, a1, a2,
            T(1), b1 / b0, b2 / b0};
  }
};

template <Number T>
struct BiquadState {
  std::vector<T> s1;
  std::vector<T> s2;
  std::vector<T> out;

  explicit BiquadState(std::size_t num_sections = 0)
      : s1(num_sections, T{}),
        s2(num_sections, T{}),
        out(num_sections, T{}) {}

  void reset() noexcept {
    std::fill(s1.begin(), s1.end(), T{});
    std::fill(s2.begin(), s2.end(), T{});
    std::fill(out.begin(), out.end(), T{});
  }
};

// Owned SOS coefficient table.
template <Number T>
struct IirCoeffs {
  std::vector<BiquadSection<T>> sections;

  IirCoeffs() = default;
  explicit IirCoeffs(std::size_t count) : sections(count) {}

  IirCoeffs(const BiquadSection<T>* input, std::size_t count) {
    if (input == nullptr && count != 0) {
      throw std::invalid_argument("IirCoeffs: null section pointer");
    }
    sections.reserve(count);
    for (std::size_t i = 0; i < count; ++i) {
      append(input[i]);
    }
  }

  explicit IirCoeffs(const BiquadSection<T>& section) {
    append(section);
  }

  explicit IirCoeffs(std::vector<BiquadSection<T>> input) {
    sections.reserve(input.size());
    for (const auto& section : input) {
      append(section);
    }
  }

  template <typename Container>
  explicit IirCoeffs(const Container& input) {
    sections.reserve(std::size(input));
    for (const auto& section : input) {
      append(section);
    }
  }

 private:
  static BiquadSection<T> normalize(const BiquadSection<T>& section) {
    using scalar_type = typename scalar_of<T>::type;
    if (std::abs(section.a0) <= std::numeric_limits<scalar_type>::epsilon()) {
      throw std::invalid_argument("IirCoeffs: a0 must not be zero");
    }
    return section.normalized_a0();
  }

  void append(const BiquadSection<T>& section) {
    sections.push_back(normalize(section));
  }
};

// Stateful executable SOS cascade.
// owns both the immutable coefficients and the mutable delay registers.
template <Number T>
struct IirFilter {
  IirCoeffs<T> coeffs;
  BiquadState<T> state;

  explicit IirFilter(IirCoeffs<T> coeffs)
      : coeffs(std::move(coeffs)), state(this->coeffs.sections.size()) {}

  void reset() noexcept {
    state.reset();
  }

  [[nodiscard]] std::size_t get_num_sections() const noexcept {
    return coeffs.sections.size();
  }
};

// Process one sample through a cascade of normalized SOS sections in
// transposed direct form II. Despite the historical name
// apply_biquad_sample, this operation processes the entire SOS cascade.
template <Number T>
T apply_sos_sample(IirFilter<T>& filter, T x) {
  const std::size_t num_sections = filter.get_num_sections();

  for (std::size_t i = 0; i < num_sections; ++i) {
    const auto& section = filter.coeffs.sections[i];

    const T y = section.b0 * x + filter.state.s1[i];

    filter.state.s1[i] = section.b1 * x - section.a1 * y + filter.state.s2[i];
    filter.state.s2[i] = section.b2 * x - section.a2 * y;
    filter.state.out[i] = y;

    x = y;
  }
  return x;
}

template <Number T>
void apply_sos_block(IirFilter<T>& filter, std::span<T> data) {
  for (auto& x : data) {
    x = apply_sos_sample(filter, x);
  }
}

template <Number T>
void apply_sos_block(IirFilter<T>& filter, std::vector<T>& data) {
  apply_sos_block(filter, std::span<T>(data));
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
IirCoeffs<T> zpk_to_sos(const Zpk& filter);

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