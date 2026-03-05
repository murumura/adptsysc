#include <adptsysc/dsplib.hh>
#include <vector>
#include <cmath>
#include <numbers>
#include <stdexcept>
#include <algorithm>

namespace adptsysc {

inline float sinc(float x) {
  if (std::abs(x) < 1e-6f) return 1.0f;
  return std::sin(x) / x;
}

std::vector<float> 
fir_lowpass_impl(std::size_t ntaps, float fc,
                 const std::vector<float>& w,
                 bool normalize) {
  if (ntaps == 0) return {};
  if (w.size() != ntaps) throw std::invalid_argument("window size mismatch");
  if (!(fc > 0.0f && fc < 0.5f)) throw std::invalid_argument("fc must be in (0, 0.5)");

  std::vector<float> h(ntaps);
  const float M = static_cast<float>(ntaps - 1);
  const float mid = M / 2.0f;
  const float scale = 2.0f * fc;

  for (std::size_t n = 0; n < ntaps; ++n) {
    const float m = static_cast<float>(n) - mid;
    const float x = 2.0f * kPi * fc * m;
    h[n] = scale * sinc(x) * w[n];
  }

  if (normalize) {
    float sum = std::accumulate(h.begin(), h.end(), 0.0f);
    if (std::abs(sum) > 1e-7f) {
      for (auto& v : h) v /= sum;
    }
  }

  return h;
}

// Designs FIR highpass filter coefficients using spectral inversion.
std::vector<float> 
fir_highpass_impl(std::size_t ntaps, float fc,
                  const std::vector<float>& w,
                  bool normalize) {
  // First design a lowpass with the same specs (without normalization)
  auto lp = fir_lowpass_impl(ntaps, fc, w, false);

  std::vector<float> hp(ntaps);
  const std::size_t c = ntaps / 2;

  // Spectral inversion: hp[n] = delta[n] - lp[n]
  for (std::size_t n = 0; n < ntaps; ++n) {
    hp[n] = -lp[n];
  }
  
  // for a highpass filter, ntaps should ideally be odd.
  hp[c] += 1.0f;

  if (normalize) {
    // Normalizing a Highpass is tricky; usually we target gain=1 at Nyquist (fs/2).
    // Sum of alternating coefficients: sum(h[n] * (-1)^n)
    float sum_alt = 0.0f;
    for (std::size_t n = 0; n < ntaps; ++n) {
      sum_alt += (n % 2 == 0) ? hp[n] : -hp[n];
    }
    if (std::abs(sum_alt) > 1e-7f) {
      for (auto& v : hp) v /= sum_alt;
    }
  }

  return hp;
}

std::vector<float>
fir_bandpass_impl(std::size_t ntaps, float f1, float f2,
                  const std::vector<float>& w,
                  bool normalize) {
  if (!(0.0f < f1 && f1 < f2 && f2 < 0.5f)) {
    throw std::invalid_argument("need 0 < f1 < f2 < 0.5");
  }

  // Bandpass designed as: LP(f2) - LP(f1)
  auto lp2 = fir_lowpass_impl(ntaps, f2, w, false);
  auto lp1 = fir_lowpass_impl(ntaps, f1, w, false);

  std::vector<float> bp(ntaps);
  for (std::size_t n = 0; n < ntaps; ++n) {
    bp[n] = lp2[n] - lp1[n];
  }

  if (normalize) {
    float wc = kPi * (f1 + f2);
    std::complex<float> H = 0.0f;
    for (size_t n = 0; n < ntaps; ++n) {
      H += bp[n] * std::exp(std::complex<float>(0.0f, -wc * n));
    }
    float g = std::abs(H);
    if (g > 1e-7f) {
      for (auto& v : bp) {
        v /= g;
      }
    }
  }

  return bp;
}

std::vector<float> 
fir_bandstop_impl(std::size_t ntaps, float f1, float f2,
                  const std::vector<float>& w,
                  bool normalize) {
  // Bandstop is Designed as: Delta - Bandpass(f1, f2)
  auto bp = fir_bandpass_impl(ntaps, f1, f2, w, false);

  std::vector<float> bs(ntaps);
  const std::size_t c = ntaps / 2;

  // Spectral inversion: bs = -bp
  for (std::size_t n = 0; n < ntaps; ++n) {
    bs[n] = -bp[n];
  }

  bs[c] += 1.0f;

  if (normalize) {
    float s = std::accumulate(bs.begin(), bs.end(), 0.0f);
    if (std::abs(s) > 1e-7f) {
      for (auto& v : bs) v /= s;
    }
  }

  return bs;
}

}