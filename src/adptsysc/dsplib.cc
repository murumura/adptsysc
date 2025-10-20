#include <adptsysc/dsplib.hh>
#include <iostream>
#include <iomanip>

namespace adptsysc {

static constexpr double kIzeroEPSILON = 1E-21;

std::vector<float> fftshift_1d(const std::vector<float>& in) {
  if (in.empty()) return {};
  
  std::vector<float> out(in.size());
  size_t offset = in.size() / 2;
  if (offset & 1) 
    offset += 1;
  // Copy second half to beginning of output
  std::copy(in.begin() + offset, in.end(), out.begin());
  // Copy first half to end of output
  std::copy(in.begin(), in.begin() + offset, out.begin() + (in.size() - offset));
  
  return out;
}

std::vector<float> ifftshift_1d(const std::vector<float>& in) {
  if (in.empty()) return {};
  
  std::vector<float> out(in.size());
  size_t offset = (in.size()) / 2;
  
  // Copy second half to beginning of output
  std::copy(in.begin() + offset, in.end(), out.begin());
  // Copy first half to end of output
  std::copy(in.begin(), in.begin() + offset, out.begin() + (in.size() - offset));
  
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
    taps[n] = 0.54 - 0.46 * cos((2 * M_PI * n) / M);
  return taps;
}

std::vector<float> 
hann(const std::size_t ntaps) {
  std::vector<float> taps(ntaps);
  float M = static_cast<float>(ntaps - 1);

  for (int n = 0; n < ntaps; n++)
    taps[n] = 0.5 - 0.5 * cos((2 * M_PI * n) / M);
  // Force center to 1.0 if ntaps is odd
  if (ntaps % 2 == 1) {
    taps[ntaps / 2] = 1.0f;
  }
  return taps;
}

std::vector<float> 
bartlett(const std::size_t ntaps)
{
  std::vector<float> taps(ntaps);
  float M = static_cast<float>(ntaps - 1);

  for (int n = 0; n < ntaps / 2; n++)
    taps[n] = 2 * n / M;
  for (int n = ntaps / 2; n < ntaps; n++)
    taps[n] = 2 - 2 * n / M;

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
stft_analysis(const std::vector<float>& in, std::size_t frame_size, 
  std::size_t hop_size, std::string_view win_name, WindowParams win_params) {

  StftAnlys spgram;
  auto win = get_window(win_name, frame_size, win_params, true);
  std::size_t n_frames = (in.size() - frame_size) / hop_size + 1;
  std::vector<float> inpad((n_frames - 1) * hop_size + frame_size, 0.0f);
  std::copy(in.begin(), in.end(), inpad.begin());
  std::vector<cfloat> spectrum; // full spectrum
  std::vector<float> stft_slice(frame_size);
  FFT fft(false, true, frame_size);

  for (std::size_t i = 0; i < n_frames; i++) {
    // Apply window and extract frame
    std::size_t pos = i * hop_size;
    for (std::size_t n = 0; n < frame_size; n++) {
      stft_slice[n] = inpad[pos + n] * win[n];
    }

    // Compute FFT - will return full N-length complex spectrum
    fft.eval(stft_slice, spectrum);

    spgram.emplace_back(spectrum);
  }
  return spgram;
}

StftSynth 
stft_synth(const StftAnlys& spgram, std::size_t frame_size,
    std::size_t hop_size, std::string_view win_name, WindowParams win_params) {
  // Create window (must match analysis window)
  auto win = get_window(win_name, frame_size, win_params, true);

  // Initialize output
  const std::size_t synth_size = (spgram.size() - 1) * hop_size + frame_size;
  StftSynth synth(synth_size, 0.0f);
  std::vector<float> wsum_frame(synth_size, 0.0f);
 
  std::vector<float> frame(frame_size);
  FFT ifft(true, false, frame_size);  // Inverse FFT, complex input

  for (std::size_t i_frame = 0; i_frame < spgram.size(); ++i_frame) {
    std::size_t start = i_frame * hop_size;
    std::vector<float> frame(frame_size);
    ifft.eval(spgram[i_frame], frame);

    for (std::size_t n = 0; n < frame_size; ++n) {
      synth[start + n] += frame[n] * win[n];
      wsum_frame[start + n] += win[n] * win[n];
    }
  }
  // Compensates for overlapping window effects
  for (std::size_t n = 0; n < synth.size(); ++n) {
    if (wsum_frame[n] > 1e-6f) {
      synth[n] /= wsum_frame[n];
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

template <typename T> PsdInfo 
pwelch(const std::vector<T>& in, std::string_view win_name,
    int win_size, int nffts, int hop_size, float fs, 
    WindowParams win_params, bool detrend, 
    Scale scale, bool avg, bool two_side) {
  const int in_size = static_cast<int>(in.size());

  // Defaults
  if (win_size <= 0)
    win_size = in_size / 8;  // heuristic
  if (nffts <= 0)
    nffts = win_size;
  if (hop_size <= 0)
    hop_size = win_size / 2;  // 50% overlap

  const bool is_cplx = is_complex_v<T>;
  if (is_cplx) {
    // Force two-sided PSD for complex input
    two_side = true;
  }
  const int psd_size = two_side ? nffts : nffts / 2 + 1;

  // Get window and normalization factor (energy)
  auto win = get_window(win_name, win_size, win_params, true);
  const float U = std::inner_product(win.begin(), win.end(), win.begin(), 0.0f);

  // Prepare FFT and PSD accumulator
  FFT fft(false, !is_cplx, nffts);
  std::vector<std::vector<float>> psd_acc;

  // Frame processing
  for (int s = 0; s + win_size <= in_size; s += hop_size) {
    std::vector<cfloat> frame(win_size);

    if (detrend) {
      if constexpr (is_complex_v<T>) {
        // Complex mean removal
        cfloat dcval{0.0f, 0.0f};
        for (int i = 0; i < win_size; ++i) {
          dcval += cfloat(in[s + i]);
        }
        dcval /= static_cast<float>(win_size);

        for (int i = 0; i < win_size; ++i) {
          auto val = cfloat(in[s + i]) - dcval;
          frame[i] = val * win[i];
        }
      } else {
        // Real mean removal
        float dcval = 0.0f;
        for (int i = 0; i < win_size; ++i) {
          dcval += static_cast<float>(in[s + i]);
        }
        dcval /= static_cast<float>(win_size);

        for (int i = 0; i < win_size; ++i) {
          float val = static_cast<float>(in[s + i]) - dcval;
          frame[i] = val * win[i];
        }
      }
    } else {
      if constexpr (is_complex_v<T>) {
        for (int i = 0; i < win_size; ++i) {
          frame[i] = cfloat(in[s + i]) * win[i];
        }
      } else {
        for (int i = 0; i < win_size; ++i) {
          frame[i] = static_cast<float>(in[s + i]) * win[i];
        }
      }
    }

    // Zero-pad and FFT
    frame.resize(nffts, cfloat(0.0f, 0.0f));
    std::vector<cfloat> spectrum;
    fft.eval(frame, spectrum);

    // Periodogram
    std::vector<float> pgram(psd_size);
    
    const float denom = (scale == Scale::Density) ? (U * fs) : U; 
    for (int k = 0; k < psd_size; ++k) {
      pgram[k] = std::norm(spectrum[k]) / denom;
    }

    // One-sided correction (only for real input)
    if (!two_side && !is_cplx && psd_size > 2) {
      for (int k = 1; k < psd_size - 1; ++k)
        pgram[k] *= 2.0f;
    }

    psd_acc.push_back(std::move(pgram));
  }

  // Average PSD across frames
  std::vector<float> psd(psd_size, 0.0f);
  if (!psd_acc.empty()) {
    for (const auto& vec : psd_acc) {
      for (int k = 0; k < psd_size; ++k) {
        psd[k] += vec[k];
      }
    }
    if (avg) {  // Mean average
      const float norm = 1.0f / psd_acc.size();
      for (auto& val : psd)
        val *= norm;
    }
  }

  // Frequency axis
  std::vector<float> freqs(psd_size);
  if (two_side) {
    // [-Fs/2 ... 0 ... Fs/2) ordering
    for (int k = 0; k < psd_size; ++k) {
      freqs[k] = fs * (k > nffts / 2 ? k - nffts : k) / nffts;
    }
  } else {
    // [0 ... Fs/2]
    for (int k = 0; k < psd_size; ++k) {
      freqs[k] = fs * k / nffts;
    }
  }

  return PsdInfo{std::move(freqs), std::move(psd), fs};
}

void plot_psd(const PsdInfo& psd, const std::string& title, const std::string& fpath) {
#ifdef ENABLE_MATPLOT
  using namespace matplot;

  auto f = matplot::figure(true);
  f->title(title);
  
  // Set gray background
  f->color(color_array{0.9f, 0.9f, 0.9f, 1.0f}); // Light gray background

  if (psd.freqs.size() == psd.psd.size()) {
    if (psd.freqs.back() < 0) {  
      auto mid = std::find_if(
          psd.freqs.begin(), psd.freqs.end(), [](float f) { return f < 0; });
      std::size_t neg_start = std::distance(psd.freqs.begin(), mid);

      std::vector<float> pos_freqs(psd.freqs.begin(), mid);
      std::vector<float> pos_psd(psd.psd.begin(), psd.psd.begin() + neg_start);

      std::vector<float> neg_freqs(mid, psd.freqs.end());
      std::vector<float> neg_psd(psd.psd.begin() + neg_start, psd.psd.end());

      // Positive frequencies subplot
      matplot::subplot(2, 1, 0);
      auto ax1 = gca();
      ax1->color(color_array{0.9f, 0.9f, 0.9f, 1.0f}); // Light gray background
      
      auto pos_plot = ax1->plot(pos_freqs, pos_psd);
      pos_plot->color(color_array{0.0f, 0.0f, 0.0f, 1.0f}); // Black line
      pos_plot->line_width(2.5); // Thick line
      
      ax1->title("Positive Frequencies");
      ax1->xlabel("Frequency (Hz)");
      ax1->ylabel("Power/Frequency");
      
      // Set Y-axis to logarithmic scale
      ax1->y_axis().scale(axis_type::axis_scale::log);
      
      // Configure grid
      ax1->grid(true);
      ax1->grid_color(color_array{0.7f, 0.7f, 0.7f, 1.0f});

      // Negative frequencies subplot
      matplot::subplot(2, 1, 1);
      auto ax2 = gca();
      ax2->color(color_array{0.9f, 0.9f, 0.9f, 1.0f}); // Light gray background
      
      auto neg_plot = ax2->plot(neg_freqs, neg_psd);
      neg_plot->color(color_array{0.0f, 0.0f, 0.0f, 1.0f}); // Black line
      neg_plot->line_width(2.5); // Thick line
      
      ax2->title("Negative Frequencies");
      ax2->xlabel("Frequency (Hz)");
      ax2->ylabel("Power/Frequency");
      
      // Set Y-axis to logarithmic scale
      ax2->y_axis().scale(axis_type::axis_scale::log);
      
      // Configure grid
      ax2->grid(true);
      ax2->grid_color(color_array{0.7f, 0.7f, 0.7f, 1.0f});

    } else {
      // Single plot for all positive frequencies
      auto ax = gca();
      ax->color(color_array{0.9f, 0.9f, 0.9f, 1.0f}); // Light gray background
      
      auto main_plot = ax->plot(psd.freqs, psd.psd);
      main_plot->color(color_array{0.0f, 0.0f, 0.0f, 1.0f}); // Black line
      main_plot->line_width(2.5); // Thick line
      
      ax->title(title);
      ax->xlabel("Frequency (Hz)");
      ax->ylabel("Power/Frequency");
      
      // Set Y-axis to logarithmic scale
      ax->y_axis().scale(axis_type::axis_scale::log);
      
      // Configure grid
      ax->grid(true);
      ax->grid_color(color_array{0.7f, 0.7f, 0.7f, 1.0f});
    }
  }
  
  f->draw();
  bool success = f->save(fpath);
  if (!success) {
    std::cerr << "Failed to save PSD plot to: " << fpath << std::endl;
  } else {
    std::cout << "PSD plot saved successfully to: " << fpath << std::endl;
  }
#endif
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
    float theta = M_PI * m[i] / (2.0f * ntaps);
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

RootInfo uniq_roots(const std::vector<cfloat>& roots, float tol) {
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

void plot_zpk(const Zpk& zpk, const std::string& title,
              const std::string& fpath, const float tol) {
#ifdef ENABLE_MATPLOT
  using namespace matplot;
  auto [z, p, k] = zpk;
  auto [uniq_z, mult_z] = uniq_roots(z, tol);
  auto [uniq_p, mult_p] = uniq_roots(p, tol);

  // Create figure
  auto f = matplot::figure(true);
  f->title(title);
  
  // Set figure background to dark color using color_array
  f->color(color_array{0.1f, 0.1f, 0.15f, 1.0f});
  
  auto ax = gca();

  // Set axes background to dark color
  ax->color(color_array{0.1f, 0.1f, 0.15f, 1.0f});

  // Convert complex values to double vectors for plotting
  auto realpart = [](const std::vector<cfloat>& vec) {
    std::vector<double> rvec;
    for (const auto& c : vec) {
      rvec.push_back(static_cast<double>(c.real()));
    }
    return rvec;
  };

  auto imagpart = [](const std::vector<cfloat>& vec) {
    std::vector<double> ivec;
    for (const auto& c : vec) {
      ivec.push_back(static_cast<double>(c.imag()));
    }
    return ivec;
  };

  ax->xlabel("Real");
  ax->ylabel("Imaginary");

  // Set axis colors to white for contrast
  ax->x_axis().color("white");
  ax->y_axis().color("white");
  // Set title color using color_array
  ax->title_color(color_array{1.0f, 1.0f, 1.0f, 1.0f});

  matplot::hold(matplot::on);

  // Create unit circle using zcircle - CYAN color
  std::vector<double> circ_x = {0.0};
  std::vector<double> circ_y = {0.0};
  std::vector<double> circ_r = {1.0};
  std::vector<double> start_angle = {0.0};
  std::vector<double> end_angle = {360.0};
  std::vector<double> color = {1.0, 1.0, 1.0, 1.0}; 
  
  circles_handle circ = std::make_shared<class zcircle>(
    ax, circ_x, circ_y, circ_r, start_angle, end_angle, color
  );

  ax->emplace_object(circ);
  circ->display_name("Unit Circle");

  // Plot real and imaginary axes (dashed gray lines)
  double axis_limit = 1.8;
  std::vector<double> real_axis_x = {-axis_limit, axis_limit};
  std::vector<double> real_axis_y = {0.0, 0.0};
  std::vector<double> imag_axis_x = {0.0, 0.0};
  std::vector<double> imag_axis_y = {-axis_limit, axis_limit};
  
  auto real_axis = ax->plot(real_axis_x, real_axis_y);
  real_axis->color(color_array{0.4f, 0.4f, 0.4f, 0.5f});
  real_axis->line_width(0.5);
  real_axis->line_style("--");
  
  auto imag_axis = ax->plot(imag_axis_x, imag_axis_y);
  imag_axis->color(color_array{0.4f, 0.4f, 0.4f, 0.5f});
  imag_axis->line_width(0.5);
  imag_axis->line_style("--");

  // Plot zeros with ZePolA style - CYAN filled circles
  if (!uniq_z.empty()) {
    std::vector<double> z_real = realpart(uniq_z);
    std::vector<double> z_imag = imagpart(uniq_z);
    
    auto zeros = ax->scatter(z_real, z_imag);
    zeros->marker_face(false);
    zeros->marker_face_color(color_array{0.0f, 0.8f, 0.8f, 1.0f}); // CYAN fill
    zeros->marker_color(color_array{0.0f, 0.8f, 0.8f, 1.0f}); // CYAN border
    zeros->marker_size(12); // Larger for better visibility
    zeros->marker_style("o");
    zeros->line_width(2);
    zeros->display_name("Zeros");

    // Add multiplicity annotations in CYAN
    for (size_t i = 0; i < uniq_z.size(); ++i) {
      if (mult_z[i] > 1) {
        auto text_obj = ax->text(z_real[i] + 0.08, z_imag[i] + 0.08, std::to_string(mult_z[i]));
        text_obj->color(color_array{0.0f, 0.8f, 0.8f, 1.0f}); // Cyan text
        text_obj->font_size(12);
      }
    }
  }
  matplot::hold(matplot::off);
  matplot::hold(matplot::on);
  // Plot poles with ZePolA style - MAGENTA crosses
  if (!uniq_p.empty()) {
    std::vector<double> p_real = realpart(uniq_p);
    std::vector<double> p_imag = imagpart(uniq_p);
    
    auto poles = ax->scatter(p_real, p_imag);
    poles->marker_face(false); // No fill for crosses
    poles->marker_color(color_array{1.0f, 0.765f, 0.0f, 0.871f}); // MAGENTA color
    poles->marker_size(15); // Larger crosses
    poles->marker_style("x");
    poles->line_width(3); // Thicker crosses
    poles->display_name("Poles");

    // Add multiplicity annotations in MAGENTA
    for (size_t i = 0; i < uniq_p.size(); ++i) {
      if (mult_p[i] > 1) {
        auto text_obj = ax->text(p_real[i] + 0.08, p_imag[i] + 0.08, std::to_string(mult_p[i]));
        text_obj->color(color_array{1.0f, 0.0f, 1.0f, 1.0f}); // Magenta text
        text_obj->font_size(12);
      }
    }
  }

  // Configure subtle grid - remove grid_line_style call
  ax->grid(true);
  ax->grid_color(color_array{0.3f, 0.3f, 0.4f, 0.9f});
  // Remove the problematic grid_line_style call

  // Set equal aspect ratio and limits
  ax->axis(matplot::equal);
  ax->xlim({-axis_limit, axis_limit});
  ax->ylim({-axis_limit, axis_limit});

  // Remove box around plot
  ax->box(false);

  // Add legend
  auto leg = matplot::legend(ax);

  // Add title with gain information
  std::stringstream title_ss;
  title_ss << title << " (k=" << std::fixed << std::setprecision(3) << k << ")";
  ax->title(title_ss.str());

  matplot::hold(matplot::off);
  
  // Force the figure to render before saving
  f->draw();
  
  bool success = f->save(fpath);
  if (!success) {
    std::cerr << "Failed to save plot to: " << fpath << std::endl;
  } else {
    std::cout << "Plot saved successfully to: " << fpath << std::endl;
  }
#endif
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
