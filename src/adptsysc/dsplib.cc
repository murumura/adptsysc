#include <adptsysc/dsplib.hh>
#include <adptsysc/design-lib.hh>
#include <iostream>
#include <iomanip>
#include <fstream>

namespace adptsysc {

static constexpr double kIzeroEPSILON = 1E-21;

// fftshift / ifftshift (NumPy-compatible)
std::vector<float> 
fftshift_1d(const std::vector<float>& in) {
  const std::size_t N = in.size();
  if (N == 0) return {};

  const std::size_t p = (N + 1) / 2;  // ceil(N/2)
  std::vector<float> out(N);

  std::copy(in.begin() + p, in.end(), out.begin());
  std::copy(in.begin(), in.begin() + p, out.begin() + (N - p));
  return out;
}

std::vector<float> 
ifftshift_1d(const std::vector<float>& in) {
  const std::size_t N = in.size();
  if (N == 0) return {};

  const std::size_t p = N / 2;  // floor(N/2)
  std::vector<float> out(N);

  std::copy(in.begin() + p, in.end(), out.begin());
  std::copy(in.begin(), in.begin() + p, out.begin() + (N - p));
  return out;
}

constexpr double Izero(double x) {
  double sum = 1.0;
  double term = 1.0;
  const double halfx = x / 2.0;

  for (int n = 1; term > kIzeroEPSILON * sum; ++n) {
    term *= (halfx / n) * (halfx / n);
    sum += term;
  }
  return sum;
}

std::vector<float> 
rect(const std::size_t ntaps) {
  return std::vector<float>(ntaps, 1.0f);
}

std::vector<float> 
hamming(const std::size_t ntaps) {
  std::vector<float> taps(ntaps);
  float M = static_cast<float>(ntaps - 1);

  for (int n = 0; n < ntaps; n++)
    taps[n] = 0.54 - 0.46 * cos((2 * kPi * n) / M);
  return taps;
}

std::vector<float> 
hann(const std::size_t ntaps) {
  std::vector<float> taps(ntaps);
  float M = static_cast<float>(ntaps - 1);

  for (int n = 0; n < ntaps; n++)
    taps[n] = 0.5 - 0.5 * cos((2 * kPi * n) / M);
  // Force center to 1.0 if ntaps is odd
  if (ntaps % 2 == 1) {
    taps[ntaps / 2] = 1.0f;
  }
  return taps;
}

std::vector<float> 
bartlett(const std::size_t ntaps) {
  std::vector<float> taps(ntaps);
  float M = static_cast<float>(ntaps - 1);

  for (int n = 0; n < ntaps / 2; n++)
    taps[n] = 2 * n / M;
  for (int n = ntaps / 2; n < ntaps; n++)
    taps[n] = 2 - 2 * n / M;

  return taps;
}

std::vector<float>
coswindow(int ntaps, std::span<const float> coeffs) {
  std::vector<float> taps(ntaps);
  const float M = static_cast<float>(ntaps - 1);
  for (int n = 0; n < ntaps; n++) {
    float sum = 0.0f;
    const float factor = 2.0f * kPi * n / M;
    for (std::size_t k = 0; k < coeffs.size(); k++) {
      const float sign = (k & 1) ? -1.0f : 1.0f;
      sum += sign * coeffs[k] * std::cos(k * factor);
    }
    taps[n] = sum;
  }
  return taps;
}

std::vector<float> 
blackman(const std::size_t ntaps) {
  return coswindow(ntaps, std::array{0.42f, 0.5f, 0.08f});
}

std::vector<float> 
blackman2(const std::size_t ntaps) {
  return coswindow(ntaps, std::array{0.34401f, 0.49755f, 0.15844f});
}

std::vector<float> 
blackman3(const std::size_t ntaps) {
  return coswindow(ntaps, std::array{0.21747f, 0.45325f, 0.28256f, 0.04672f});
}

std::vector<float> 
blackman4(const std::size_t ntaps) {
  return coswindow(ntaps, std::array{0.084037f, 0.29145f, 0.375696f, 0.20762f, 0.041194f});
}

std::vector<float> 
blackman_harris(const std::size_t ntaps, int atten) {
  switch (atten) {
    case 61:
      return coswindow(ntaps, std::array{0.42323f, 0.49755f, 0.07922f});
    case 67:
      return coswindow(ntaps, std::array{0.44959f, 0.49364f, 0.05677f});
    case 74:
      return coswindow(
          ntaps, std::array{0.40271f, 0.49703f, 0.09392f, 0.00183f});
    case 92:
      return coswindow(
          ntaps, std::array{0.35875f, 0.48829f, 0.14128f, 0.01168f});
    default:
      throw std::out_of_range("blackman_harris: unknown attenuation value "
                              "(must be 61, 67, 74, or 92)");
  }
}

std::vector<float> 
kaiser(const std::size_t ntaps, double beta) {
  if (beta < 0)
    throw std::invalid_argument("Kaiser window: beta must be >= 0");

  std::vector<float> taps(ntaps);
  double IBeta = 1.0 / Izero(beta);
  double inm1 = 1.0 / static_cast<double>(ntaps - 1);

  taps[0] = IBeta;
  for (int i = 1; i < ntaps - 1; i++) {
    double temp = 2 * i * inm1 - 1;
    taps[i] = Izero(beta * std::sqrt(1.0 - temp * temp)) * IBeta;
  }
  taps[ntaps - 1] = IBeta;
  return taps;
}

// Main window function
std::vector<float> 
get_window(std::string_view name, const size_t ntaps,
           WindowParams params, bool normalize) {
  // Convert to lowercase
  auto winname = [name] {
    std::string s(name);
    std::ranges::transform(
        s, s.begin(), [](unsigned char c) { return std::tolower(c); });
    return s;
  }();

  // Normalization helper
  auto normalize_window = [](std::vector<float>& win) {
    const double pwr = std::transform_reduce(win.begin(), win.end(), 0.0,
                           std::plus{}, [](float val) { return val * val; })
                       / win.size();

    const float norm_factor = static_cast<float>(std::sqrt(pwr));
    std::ranges::transform(
      win, win.begin(), [norm_factor](float tap) { return tap / norm_factor; });
  };

  // Get the window
  std::vector<float> win;
  if (winname == "hann") {
    win = hann(ntaps);
  } else if (winname == "rect") {
    win = rect(ntaps);
  } else if (winname == "hamming") {
    win = hamming(ntaps);
  } else if (winname == "") {
    win = bartlett(ntaps);
  } else if (winname == "blackman") {
    win = blackman(ntaps);
  } else if (winname == "blackman2") {
    win = blackman2(ntaps);
  } else if (winname == "blackman3") {
    win = blackman3(ntaps);
  } else if (winname == "blackman4") {
    win = blackman4(ntaps);
  } else if (winname == "blackman_harris") {
    int atten = kDefaultBlackHarrisAtten;
    if (auto* p = std::get_if<AttenParam>(&params)) {
      atten = p->atten;
    }
    win = blackman_harris(ntaps, atten);
  } else if (winname == "kaiser") {
    double beta = kDefaultKaiserBeta;
    if (auto* p = std::get_if<KaiserParam>(&params)) {
      beta = p->beta;
    }
    win = kaiser(ntaps, beta);
  } else {
    throw std::invalid_argument("Unknown window type: " + std::string(name));
  }

  // Apply normalization if requested
  if (normalize) {
    normalize_window(win);
  }

  return win;
}

StftAnlys
stft_analysis(const std::vector<float>& in,
              std::size_t frame_size,
              std::size_t hop_size,
              std::string_view win_name,
              WindowParams win_params) {

  StftAnlys spgram;

  auto win = get_window(win_name, frame_size, win_params, true);

  const std::size_t n_frames = (in.size() - frame_size) / hop_size + 1;

  std::vector<float> inpad((n_frames - 1) * hop_size + frame_size, 0.0f);

  std::copy(in.begin(), in.end(), inpad.begin());

  std::vector<cfloat> spectrum;
  std::vector<float> frame(frame_size);

  EigenFFTWrapper<float> fft(EigenFFTWrapper<float>::FFTMode::Real, frame_size);

  for (std::size_t i = 0; i < n_frames; ++i) {
    // Apply window and extract frame
    const std::size_t pos = i * hop_size;

    for (std::size_t n = 0; n < frame_size; ++n) {
      frame[n] = inpad[pos + n] * win[n];
    }
    // will return full N-length complex spectrum
    fft.runfft(frame, spectrum);

    spgram.emplace_back(spectrum);
  }

  return spgram;
}

StftSynth
stft_synth(const StftAnlys& spgram,
           std::size_t frame_size,
           std::size_t hop_size,
           std::string_view win_name,
           WindowParams win_params) {

  auto win = get_window(win_name, frame_size, win_params, true);

  const std::size_t synth_size =
    (spgram.size() - 1) * hop_size + frame_size;

  StftSynth synth(synth_size, 0.0f);
  std::vector<float> wsum(synth_size, 0.0f);

  EigenFFTWrapper<float> fft(EigenFFTWrapper<float>::FFTMode::Real, frame_size);

  std::vector<float> frame;

  for (std::size_t i = 0; i < spgram.size(); ++i) {
    const std::size_t start = i * hop_size;

    fft.run_ifft(spgram[i], frame);

    for (std::size_t n = 0; n < frame_size; ++n) {
      synth[start + n] += frame[n] * win[n];
      wsum[start + n]  += win[n] * win[n];
    }
  }

  // Compensates for overlapping window effects
  for (std::size_t n = 0; n < synth.size(); ++n) {
    if (wsum[n] > 1e-6f) {
      synth[n] /= wsum[n];
    }
  }

  return synth;
}

void StftAnlysInfo::derived_prop() {
  // Frequency resolution (bin width)
  freq_res = fs / static_cast<float>(frame_size);

  // Time resolution (seconds per frame)
  time_res= static_cast<float>(hop_size) / fs;

  // Maximum representable frequency
  max_freq = fs / 2.0f;

  // Total duration
  dur_secs = static_cast<float>(spgram.size()) * time_res;

  // Spectral statistics
  minval = INFINITY;
  maxval = -INFINITY;
  double sum = 0.0;
  std::size_t count = 0;

  for (const auto& frame : spgram) {
    for (const auto& bin : frame) {
      float magnitude = std::abs(bin);
      minval = std::min(minval, magnitude);
      maxval = std::max(maxval, magnitude);
      sum += magnitude;
      count++;
    }
  }
  mean = static_cast<float>(sum / count);
}

std::ostream& operator<<(std::ostream& os, const StftAnlysInfo& info) {
  // Header with basic info
  os << "STFT Analysis Information:\n"
     << "=========================\n"
     << std::fixed << std::setprecision(2);

  // Parameters section
  os << "Parameters:\n"
     << "  Sample rate:      " << info.fs << " Hz\n"
     << "  Frame size:       " << info.frame_size << " samples\n"
     << "  Hop size:         " << info.hop_size << " samples\n"
     << "  Window type:      " << info.win_name << "\n"
     << "  Frequency res:    " << info.freq_res << " Hz/bin\n"
     << "  Time res:         " << info.time_res * 1000 << " ms/frame\n"
     << "  Max frequency:    " << info.max_freq << " Hz\n"
     << "  Duration:         " << info.dur_secs << " s\n\n";

  // Data statistics
  os << "Spectrogram Statistics:\n"
     << "  Frames:           " << info.spgram.size() << "\n"
     << "  Bins per frame:   "
     << (info.spgram.empty() ? 0 : info.spgram[0].size()) << "\n"
     << "  Min magnitude:    " << info.minval << "\n"
     << "  Max magnitude:    " << info.maxval << "\n"
     << "  Mean magnitude:   " << info.mean << "\n\n";

  // Example frame output (first frame)
  if (!info.spgram.empty()) {
    os << "First Frame (bins 0-5):\n";
    for (size_t i = 0; i < std::min<size_t>(6, info.spgram[0].size()); ++i) {
      os << "  Bin " << i << " (" << (i * info.freq_res) << " Hz): "
         << "mag=" << std::abs(info.spgram[0][i])
         << ", phase=" << std::arg(info.spgram[0][i]) << " rad\n";
    }
  }

  return os;
}

template <typename T>
inline auto mag2(const T& v) {
  return std::norm(v);  // works for real & complex
}

template <typename T>
CrossCorrelationEval<T>
cross_correlation(const std::vector<T>& x, const std::vector<T>& y,
                  int max_lag, std::string_view scale, bool pos_lag) {

  if (x.empty() || y.empty())
    throw std::invalid_argument("cross_correlation: empty input");

  const int Nx = static_cast<int>(x.size());
  const int Ny = static_cast<int>(y.size());

  int lag_min = -(Ny - 1);
  int lag_max =  (Nx - 1);

  if (max_lag >= 0) {
    if (max_lag >= std::max(Nx, Ny))
      throw std::invalid_argument("cross_correlation: max_lag too large");
    lag_min = std::max(lag_min, -max_lag);
    lag_max = std::min(lag_max,  max_lag);
  }

  if (pos_lag)
    lag_min = std::max(lag_min, 0);

  const int n_lags = lag_max - lag_min + 1;

  CrossCorrelationEval<T> res;
  res.lags.resize(n_lags);
  res.corrs.resize(n_lags, T(0));

  double x_power = 0.0;
  double y_power = 0.0;

  if (scale == "coeff" || scale == "normalized") {
    for (const auto& v : x) x_power += mag2(v);
    for (const auto& v : y) y_power += mag2(v);
    if (x_power == 0.0 || y_power == 0.0)
      throw std::runtime_error("cross_correlation: zero norm");
  }

  const double norm = (scale == "coeff" || scale == "normalized")
                      ? std::sqrt(x_power * y_power) : 1.0;

  const int N = std::max(Nx, Ny);

  for (int idx = 0; idx < n_lags; ++idx) {
    const int k = lag_min + idx;
    res.lags[idx] = k;

    T sum = T(0);
    for (int n = 0; n < Nx; ++n) {
      const int m = n - k;
      if (m >= 0 && m < Ny) {
        if constexpr (std::is_floating_point_v<T>) {
          sum += x[n] * y[m];
        } else {
          sum += x[n] * std::conj(y[m]);
        }
      }
    }

    if (scale == "biased") {
      sum /= static_cast<double>(N);
    }
    else if (scale == "unbiased") {
      sum /= static_cast<double>(N - std::abs(k));
    }
    else if (scale == "coeff" || scale == "normalized") {
      sum /= norm;
    }
    else if (scale != "none") {
      throw std::invalid_argument("cross_correlation: invalid scale");
    }

    res.corrs[idx] = sum;
  }

  return res;
}

template <typename T>
CrossCorrelationEval<T>
auto_correlation(const std::vector<T>& x,
                int max_lag, std::string_view scale,
                bool pos_lag) {
  return cross_correlation(x, x, max_lag, scale, pos_lag);
}

template CrossCorrelationEval<float>
cross_correlation<float>(const std::vector<float>& x, const std::vector<float>& y, 
                        int max_lag, std::string_view scale, bool pos_lag);

template <typename T>
PsdInfo pwelch(const std::vector<T>& in,
               std::string_view win_name,
               int win_size,
               int nffts,
               int hop_size,
               float fs,
               WindowParams win_params,
               bool detrend,
               Scale scale,
               bool avg,
               bool two_side)
{
  const int in_size = static_cast<int>(in.size());

  // Defaults
  if (win_size <= 0)  win_size = in_size / 8;
  if (nffts <= 0)     nffts = win_size;
  if (hop_size <= 0)  hop_size = win_size / 2;

  constexpr bool is_cplx = is_complex_v<T>;
  if (is_cplx) two_side = true;

  const int psd_size = two_side ? nffts : nffts / 2 + 1;

  // Window
  auto win = get_window(win_name, win_size, win_params, false);
  const float U = std::inner_product(win.begin(), win.end(), win.begin(), 0.0f);
  
  using Scalar = typename scalar_of<T>::type;
  using FFTT   = EigenFFTWrapper<Scalar>;

  FFTT fft(
    is_cplx ? FFTT::FFTMode::Complex
            : FFTT::FFTMode::Real,
    static_cast<std::size_t>(nffts)
  );

  std::vector<std::vector<float>> psd_acc;

  // ============================================================
  // Frame loop
  // ============================================================

  for (int s = 0; s + win_size <= in_size; s += hop_size) {

    std::vector<cfloat> spectrum;

    if constexpr (is_complex_v<T>) {
      // ============================
      // Complex path
      // ============================
      std::vector<cfloat> frame(win_size);

      // detrend
      cfloat dc{0.0f, 0.0f};
      if (detrend) {
        for (int i = 0; i < win_size; ++i)
          dc += cfloat(in[s + i]);
        dc /= static_cast<float>(win_size);
      }

      for (int i = 0; i < win_size; ++i) {
        cfloat val = cfloat(in[s + i]);
        if (detrend) val -= dc;
        frame[i] = val * win[i];
      }

      frame.resize(nffts, cfloat(0.0f));
      fft.runfft(frame, spectrum);   // ✅ C2C
    }
    else {
      // ============================
      // Real path
      // ============================
      std::vector<float> frame(win_size);

      float dc = 0.0f;
      if (detrend) {
        for (int i = 0; i < win_size; ++i)
          dc += static_cast<float>(in[s + i]);
        dc /= static_cast<float>(win_size);
      }

      for (int i = 0; i < win_size; ++i) {
        float val = static_cast<float>(in[s + i]);
        if (detrend) val -= dc;
        frame[i] = val * win[i];
      }

      frame.resize(nffts, 0.0f);
      fft.runfft(frame, spectrum);   // ✅ R2C
    }

    // ============================
    // Periodogram
    // ============================
    std::vector<float> pgram(psd_size);

    const float denom =
      (scale == Scale::Density) ? (U * fs) : U;

    for (int k = 0; k < psd_size; ++k) {
      pgram[k] = std::norm(spectrum[k]) / denom;
    }

    // one-sided correction
    if (!two_side && !is_cplx && psd_size > 2) {
      for (int k = 1; k < psd_size - 1; ++k)
        pgram[k] *= 2.0f;
    }

    psd_acc.emplace_back(std::move(pgram));
  }

  // ============================================================
  // Average
  // ============================================================

  std::vector<float> psd(psd_size, 0.0f);

  for (const auto& vec : psd_acc) {
    for (int k = 0; k < psd_size; ++k) {
      psd[k] += vec[k];
    }
  }

  if (avg && !psd_acc.empty()) {
    const float inv = 1.0f / psd_acc.size();
    for (auto& v : psd) v *= inv;
  }

  // ============================================================
  // Frequency axis
  // ============================================================

  std::vector<float> freqs(psd_size);

  if (two_side) {
    for (int k = 0; k < psd_size; ++k) {
      freqs[k] = fs * (k > nffts / 2 ? k - nffts : k) / nffts;
    }
  } else {
    for (int k = 0; k < psd_size; ++k) {
      freqs[k] = fs * k / nffts;
    }
  }

  return PsdInfo{std::move(freqs), std::move(psd), fs};
}

int plot_psd (
  const PsdInfo& psd, const std::string& title,
  const std::string& prefix, float fs_override, 
  bool log_freq,bool linear) {
  const float fs = (fs_override > 0.0f) ? fs_override : psd.fs;
  // 1. dump CSV
  {
    std::ofstream f(prefix + "_freqs.csv");
    for (float v : psd.freqs)
      f << v << "\n";
  }

  {
    std::ofstream f(prefix + "_vals.csv");
    for (float v : psd.psd)
      f << v << "\n";
  }

  fs::path plt_script = fs::path(__FILE__).parent_path() / "adptplot.py";

  std::ostringstream cmd;
  cmd << "python3 " << plt_script.string()
      << " psd"                                
      << " --freqs " << prefix << "_freqs.csv"
      << " --psd "   << prefix << "_vals.csv"
      << " --fs "    << fs
      << " --title \"" << title << "\""
      << " --out "   << prefix << "_psd.png";
  if (log_freq)
    cmd << " --log-freq";

  if (linear)
    cmd << " --linear";

  std::cout << "[plot_psd] CMD:\n" << cmd.str() << std::endl;

  return std::system(cmd.str().c_str());
}


Zpk chebyshev1(const std::size_t ntaps, const float rp) {
  if (ntaps <= 0){
    return { {}, {}, 1.0f };
  }

  float eps = std::sqrt(std::pow(10.0f, 0.1f * rp) - 1.0f);
  float psi = 1.0f / ntaps * std::asinh(1.0f / eps);

  // Generate m = [-N+1, -N+3, ..., N-3, N-1]
  std::vector<float> m = linspace (
    static_cast<float>(-ntaps + 1),  // start: -N+1
    static_cast<float>(ntaps - 1),   // end: N-1  
    ntaps                            // size: N points
  );

  // Calculate poles
  std::vector<cfloat> poles;
  poles.reserve(ntaps);
  
  // p_k = -sinh(psi)·sin(θ_k) + j·cosh(psi)·cos(θ_k)
  // where θ_k = π(2k-1)/(2N), k = 1,2,...,N
  for (std::size_t i = 0; i < ntaps; ++i) {
    float theta = kPi * m[i] / (2.0f * ntaps);
    cfloat p = -std::sinh(cfloat(psi, theta));
    poles.emplace_back(p);
  }

  // Calculate gain
  float k = 1.0f;
  for (const auto& p : poles) {
    k *= -p.real(); // product(-p).real()
  }
  
  if (ntaps % 2 == 0) {
    k = k / std::sqrt(1.0f + eps * eps);
  }
  
  return { {}, std::move(poles), k };
}

Zpk butterworth(const std::size_t ntaps) {
  if (ntaps <= 0) {
    return { {}, {}, 1.0f };
  }
  
  const float start = kPi / 2 + kPi / (2 * ntaps);
  const float step = kPi / ntaps;

  std::vector<cfloat> poles;
 
  for (auto i : std::views::iota(0u, ntaps)) {
    poles.emplace_back(std::polar(1.0f, start + i * step));
  }
  return { {}, std::move(poles), 1.0f };
}

RootInfo 
uniq_roots(const std::vector<cfloat>& roots, const float tol) {
  if (roots.empty()) {
    return {{}, {}};
  }

  // Create sorted copy of roots (sort by magnitude)
  std::vector<cfloat> sortrts = roots;
  std::sort(sortrts.begin(), sortrts.end(),
      [](const cfloat& a, const cfloat& b) { 
        return std::abs(a) < std::abs(b); 
  });

  std::vector<cfloat> uniq;
  std::vector<int> mult;

  for (const auto& r : sortrts) {
    if (uniq.empty()) {
      uniq.push_back(r);
      mult.push_back(1);
      continue;
    }

    // Find minimum distance to existing unique roots
    float min_dist = std::numeric_limits<float>::max();
    std::size_t min_idx = 0;

    for (std::size_t i = 0; i < uniq.size(); ++i) {
      float dist = std::abs(r - uniq[i]);
      if (dist < min_dist) {
        min_dist = dist;
        min_idx = i;
      }
    }

    if (min_dist > tol) {
      // New unique root
      uniq.push_back(r);
      mult.push_back(1);
    } else {
      // Update existing root with weighted average
      int cur_mult = mult[min_idx];
      uniq[min_idx] = (uniq[min_idx] * static_cast<float>(cur_mult) + r)
                      / static_cast<float>(cur_mult + 1);
      mult[min_idx] = cur_mult + 1;
    }
  }

  return {uniq, mult};
}

int plot_zpk(const Zpk& zpk,
             const std::string& title,
             const std::string& prefix,
             const float tol) {
  using std::ofstream;
  using std::vector;

  auto [z, p, k] = zpk;

  auto [uniq_z, mult_z] = uniq_roots(z, tol);
  auto [uniq_p, mult_p] = uniq_roots(p, tol);

  // ---- export zeros ----
  {
    ofstream f(prefix + "_zeros.csv");
    f << "real,imag,mult\n";
    for (size_t i = 0; i < uniq_z.size(); ++i) {
      f << uniq_z[i].real() << ","
        << uniq_z[i].imag() << ","
        << mult_z[i] << "\n";
    }
  }

  // ---- export poles ----
  {
    ofstream f(prefix + "_poles.csv");
    f << "real,imag,mult\n";
    for (size_t i = 0; i < uniq_p.size(); ++i) {
      f << uniq_p[i].real() << ","
        << uniq_p[i].imag() << ","
        << mult_p[i] << "\n";
    }
  }

  // export metadata
  {
    ofstream f(prefix + "_meta.txt");
    f << "title=" << title << "\n";
    f << "k=" << k << "\n";
    f << "tol=" << tol << "\n";
  }

  // call python renderer
  fs::path script = fs::path(__FILE__).parent_path() / "adptplot.py";

  std::ostringstream cmd;
  cmd << "python3 " << script.string()
      << " zpk"
      << " --zeros " << prefix << "_zeros.csv"
      << " --poles " << prefix << "_poles.csv"
      << " --meta "  << prefix << "_meta.txt"
      << " --out "   << prefix << "_zpk.png";
      
  std::cout << "[plot_zpk] CMD:\n" << cmd.str() << std::endl;
  return std::system(cmd.str().c_str());
}


template PsdInfo pwelch<float>(const std::vector<float>& in,
  std::string_view win_name, int win_size, int nffts, int hop_size, float fs,
  WindowParams win_params, bool detrend, Scale scale, bool avg,
  bool two_side);

template PsdInfo pwelch<cfloat>(
  const std::vector<cfloat>& in, std::string_view win_name,
  int win_size, int nffts, int hop_size, float fs, WindowParams win_params,
  bool detrend, Scale scale, bool avg, bool two_side);

}  // namespace adptsysc
