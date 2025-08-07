#include <adptsysc/dsplib.hh>
#include <iostream>
#include <iomanip>

namespace adptsysc {

static constexpr double kIzeroEPSILON = 1E-21;

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

std::vector<float> rect(const std::size_t ntaps) {
  return std::vector<float>(ntaps, 1.0f);
}

std::vector<float> hamming(const std::size_t ntaps) {
  std::vector<float> taps(ntaps);
  float M = static_cast<float>(ntaps - 1);

  for (int n = 0; n < ntaps; n++)
    taps[n] = 0.54 - 0.46 * cos((2 * M_PI * n) / M);
  return taps;
}

std::vector<float> hann(const std::size_t ntaps) {
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

std::vector<float> blackman(const std::size_t ntaps) {
  return coswindow(ntaps, std::array{0.42f, 0.5f, 0.08f});
}

std::vector<float> blackman2(const std::size_t ntaps) {
  return coswindow(ntaps, std::array{0.34401f, 0.49755f, 0.15844f});
}

std::vector<float> blackman3(const std::size_t ntaps) {
  return coswindow(ntaps, std::array{0.21747f, 0.45325f, 0.28256f, 0.04672f});
}

std::vector<float> blackman4(const std::size_t ntaps) {
  return coswindow(ntaps, std::array{0.084037f, 0.29145f, 0.375696f, 0.20762f, 0.041194f});
}

std::vector<float> blackman_harris(const std::size_t ntaps, int atten) {
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

std::vector<float> kaiser(const std::size_t ntaps, double beta) {
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
std::vector<float> get_window(std::string_view name, const size_t ntaps,
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
  std::vector<std::complex<float>> spectrum; // full spectrum
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
  // Temporary buffers
  std::vector<std::complex<float>> full_spectrum(frame_size);
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
  freq_res = sample_rate / static_cast<float>(frame_size);

  // Time resolution (seconds per frame)
  time_res= static_cast<float>(hop_size) / sample_rate;

  // Maximum representable frequency
  max_freq = sample_rate / 2.0f;

  // Total duration
  dursecs = static_cast<float>(spgram.size()) * time_res;

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
     << "  Sample rate:      " << info.sample_rate << " Hz\n"
     << "  Frame size:       " << info.frame_size << " samples\n"
     << "  Hop size:         " << info.hop_size << " samples\n"
     << "  Window type:      " << info.win_name << "\n"
     << "  Frequency res:    " << info.freq_res << " Hz/bin\n"
     << "  Time res:         " << info.time_res * 1000 << " ms/frame\n"
     << "  Max frequency:    " << info.max_freq << " Hz\n"
     << "  Duration:         " << info.dursecs << " s\n\n";

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


}  // namespace adptsysc