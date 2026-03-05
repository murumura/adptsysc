
#include <adptsysc/dsplib.hh>
#include <algorithm>
#include <cmath>
#include <complex>
#include <vector>
#include <gtest/gtest.h>
using namespace adptsysc;

static std::complex<float>
freq_response(const std::vector<float>& h, float omega) {
  std::complex<float> H = 0;

  for (size_t n = 0; n < h.size(); ++n)
    H += h[n] * std::exp(std::complex<float>(0, -omega * n));

  return H;
}

static std::vector<float>
rect_window(size_t N) {
  return std::vector<float>(N, 1.0f);
}

TEST(FIRDesign, LowpassDCGain) {
  const size_t N = 63;
  auto win = rect_window(N);
  auto h = fir_lowpass_impl(N, 0.2f, win);
  float sum = 0;
  for (auto v : h) sum += v;
  EXPECT_NEAR(sum, 1.0f, 1e-4);
}

TEST(FIRDesign, LowpassSymmetry) {
  const size_t N = 63;
  auto win = rect_window(N);

  auto h = fir_lowpass_impl(N, 0.25f, win);

  for (size_t i = 0; i < N/2; ++i)
    EXPECT_NEAR(h[i], h[N-1-i], 1e-6);
}

TEST(FIRDesign, HighpassDCZero) {
  const size_t N = 63;
  auto win = rect_window(N);

  auto h = fir_highpass_impl(N, 0.2f, win);

  float sum = 0;
  for (auto v : h) sum += v;

  EXPECT_NEAR(sum, 0.0f, 1e-2);
}

TEST(FIRDesign, HighpassNyquistGain) {
  const size_t N = 63;
  auto win = rect_window(N);

  auto h = fir_highpass_impl(N, 0.2f, win, true);

  auto H = freq_response(h, M_PI);

  EXPECT_NEAR(std::abs(H), 1.0f, 0.05f);
}

TEST(FIRDesign, BandpassDCZero) {
  const size_t N = 63;
  auto win = rect_window(N);

  auto h = fir_bandpass_impl(N, 0.2f, 0.3f, win);

  float sum = 0;
  for (auto v : h) sum += v;

  EXPECT_NEAR(sum, 0.0f, 1e-2);
}

TEST(FIRDesign, BandstopDCOne) {
  const size_t N = 63;
  auto win = rect_window(N);

  auto h = fir_bandstop_impl(N, 0.2f, 0.3f, win);

  float sum = 0;
  for (auto v : h) sum += v;

  EXPECT_NEAR(sum, 1.0f, 1e-3);
}