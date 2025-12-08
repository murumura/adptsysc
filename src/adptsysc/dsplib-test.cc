#include <Eigen/Dense>
#include <adptsysc/dsplib.hh>
#include <complex>
#include <gtest/gtest.h>
#include <iostream>

using namespace adptsysc;

TEST(Trait, Complex) {
  static_assert(is_complex_v<std::complex<float>>, "Should be true");
  static_assert(!is_complex_v<float>, "Should be false");
}

TEST(CrossCorrelation, Basic) {
  auto CrossCorrelation = CrossCorrelation<double>();  // Reset
  auto result = CrossCorrelation.eval({2, 3, 4}, {3, 4, 5}, -1, "none");
  std::vector<int> expected_lags = {2, 1, 0, -1, -2};
  std::vector<double> expected_corrs = {10, 23, 38, 25, 12};

  // Test mismatched signal lengths
  std::vector<double> x = {0.1, 0.2, -0.1, 4.1, -2, 1.5, 0};
  std::vector<double> y = {0.1, 4, -2.2, 1.6, 0.1, 0.1, 0.2};

  result = CrossCorrelation.eval({0.1, 0.2, -0.1, 4.1, -2, 1.5, 0},
      {0.1, 4, -2.2, 1.6, 0.1, 0.1, 0.2}, -1, "none");

  expected_lags = {6, 5, 4, 3, 2, 1, 0, -1, -2, -3, -4, -5, -6};
  expected_corrs = {
      0.02,    // k=6
      0.05,    // k=5
      0.01,    // k=4
      0.99,    // k=3
      0.1,     // k=2
      0.31,    // k=1
      7.54,    // k=0
      -12.45,  // k=-1
      23.19,   // k=-2
      -10.89,  // k=-3
      5.8,     // k=-4
      0.15,    // k=-5
      0.0      // k=-6
  };

  EXPECT_EQ(result.lags.size(), expected_lags.size());
  EXPECT_EQ(result.corrs.size(), expected_corrs.size());
  for (size_t i = 0; i < result.lags.size(); ++i) {
    EXPECT_EQ(result.lags[i], expected_lags[i]);
    EXPECT_NEAR(result.corrs[i], expected_corrs[i], 1e-6);
  }

  result = CrossCorrelation.eval(x, y, -1, "none", true);
  EXPECT_EQ(result.lags.size(), expected_lags.size() / 2 + 1);
  EXPECT_EQ(result.corrs.size(), expected_corrs.size() / 2 + 1);
  for (size_t i = 0; i < expected_lags.size() / 2 + 1; ++i) {
    EXPECT_EQ(result.lags[i], expected_lags[i]);
    EXPECT_NEAR(result.corrs[i], expected_corrs[i], 1e-6);
  }
}

TEST(CrossCorrelation, Normalize) {
  auto CrossCorrelation = CrossCorrelation<double>();  // Reset
  std::vector<double> x = {1.0, 0.5, 0.25, 0.125};
  std::vector<double> y = {0.125, 1.0, 0.5, 0.25};  // Shifted x
  auto result = CrossCorrelation.eval(x, y, 2, "normalized");

  // Norm = sqrt(1^2 + 0.5^2 + 0.25^2 + 0.125^2) * sqrt(0.125^2 + 1^2 + 0.5^2 + 0.25^2) ≈ 1.3280
  double normval = 1.3280;
  std::vector<double> expected_corrs = {
      0.625 / normval,    // k=2
      1.3125 / normval,   // k=1
      0.78125 / normval,  // k=0
      0.375 / normval,    // k=-1
      0.15625 / normval   // k=-2
  };
  std::vector<int> expected_lags_normalized = {2, 1, 0, -1, -2};

  EXPECT_EQ(result.lags.size(), expected_lags_normalized.size());
  EXPECT_EQ(result.corrs.size(), expected_corrs.size());
  for (size_t i = 0; i < result.lags.size(); ++i) {
    EXPECT_EQ(result.lags[i], expected_lags_normalized[i]);
    EXPECT_NEAR(result.corrs[i], expected_corrs[i], 1e-3);
  }
}

TEST(FFT, Generic) {
  constexpr float tol = 1e-4;
  Eigen::VectorXf rin(8);
  rin.setRandom();

  Eigen::VectorXcf fwdout;
  Eigen::VectorXcf invout;

  FFT fwdfft(false, true);  // real FFT
  fwdfft.eval(rin, fwdout);

  FFT invfft(true, false);  // complex IFFT
  invfft.eval(fwdout, invout);

  // Validate inverse result (should match original input)
  for (int i = 0; i < rin.size(); ++i) {
    EXPECT_NEAR(invout[i].real(), rin[i], tol);
    EXPECT_NEAR(invout[i].imag(), 0.0, tol);
  }
}

TEST(FFT, RealToComplexAndBack) {
  using T = float;
  const std::size_t N = 8;

  std::vector<T> rin(N);
  for (std::size_t i = 0; i < N; ++i)
    rin[i] = std::sin(2 * M_PI * i / N);  // A simple sine wave

  std::vector<std::complex<T>> fwdout;
  std::vector<std::complex<T>> invout;

  FFT fwdfft(/*inverse=*/false, /*real=*/true);
  fwdfft.eval(rin, fwdout);

  // Make sure forward FFT has expected size (N/2 + 1 for real input)
  EXPECT_EQ(fwdout.size(), N);

  // Inverse FFT expects complex input and outputs complex
  FFT invfft(/*inverse=*/true, /*real=*/false);
  invfft.eval(fwdout, invout);

  // Due to IFFT from real FFT, output should be real-dominant
  // Compare real part to original input (allow small error)
  ASSERT_EQ(invout.size(), N);

  for (std::size_t i = 0; i < N; ++i) {
    EXPECT_NEAR(invout[i].real(), rin[i], 1e-10) << "Mismatch at index " << i;
    EXPECT_NEAR(invout[i].imag(), 0.0, 1e-10)
        << "Imaginary part not near zero at index " << i;
  }
}

TEST(FFT, FFTSizeHandling) {
  using T = float;
  using Complex = std::complex<T>;
  constexpr float tolerance = 1e-4;

  // Test 1: Automatic size detection for complex FFT
  {
    FFT fft(false, false);  // Forward complex FFT
    const size_t input_size = 1024;
    std::vector<Complex> in(input_size, Complex(1.0, 0.0));  // DC signal
    std::vector<Complex> out;

    fft.eval(in, out);

    // Verify output size matches input size
    EXPECT_EQ(out.size(), input_size);

    // Verify FFT properties (DC signal should have energy at bin 0)
    EXPECT_NEAR(out[0].real(), input_size, tolerance);
    EXPECT_NEAR(out[0].imag(), 0.0, tolerance);

    // Other bins should be near zero (floating-point noise)
    for (size_t i = 1; i < out.size(); ++i) {
      EXPECT_NEAR(std::abs(out[i]), 0.0, tolerance);
    }
  }

  // Test 2: Fixed size with zero-padding
  {
    const size_t input_size = 512;
    const size_t fft_size = 2048;
    FFT fft_padded(false, false, fft_size);
    std::vector<Complex> in(input_size, Complex(1.0, 0.0));
    std::vector<Complex> out;

    fft_padded.eval(in, out);

    // Verify output size matches requested FFT size
    EXPECT_EQ(out.size(), fft_size);

    // Verify FFT properties (padded DC signal)
    EXPECT_NEAR(out[0].real(), input_size,
        tolerance);  // Not fft_size because of zero-padding
    EXPECT_NEAR(out[0].imag(), 0.0, tolerance);
  }
}

TEST(WINDOW, WindowCoeff) {
  // Test basic window properties
  const size_t ntaps = 32;

  // 1. Verify Rectangular Window (all ones)
  {
    auto win = get_window("rect", ntaps);
    ASSERT_EQ(win.size(), ntaps);
    for (float val : win) {
      EXPECT_FLOAT_EQ(val, 1.0f);
    }
  }

  // 2. Verify Hann Window symmetry and range
  {
    auto win = get_window("hann", ntaps);
    EXPECT_TRUE(std::is_sorted(win.begin(), win.begin() + ntaps / 2));
    EXPECT_TRUE(std::is_sorted(win.rbegin(), win.rbegin() + ntaps / 2));
    EXPECT_NEAR(win.front(), 0.0f, 1e-6f);
    EXPECT_NEAR(win.back(), 0.0f, 1e-6f);
    if (ntaps % 2 == 1)
      EXPECT_NEAR(win[ntaps / 2], 1.0f, 1e-6f);
    else
      EXPECT_NEAR(win[ntaps / 2], 1.0f, 1e-2f);
  }

  // 3. Test Normalization
  {
    auto win = get_window("hamming", ntaps, NoParam{}, true);
    float power = std::accumulate(win.begin(), win.end(), 0.0f,
                      [](float sum, float x) { return sum + x * x; })
                  / ntaps;
    EXPECT_NEAR(power, 1.0f, 1e-6f);
  }

  // 4. Verify Blackman-Harris with attenuation parameter
  {
    auto win61 = get_window("blackman_harris", ntaps, AttenParam{61});
    auto win74 = get_window("blackman_harris", ntaps, AttenParam{74});

    // Should produce different windows
    EXPECT_NE(win61, win74);

    // First and last samples should be near zero
    EXPECT_LT(win61.front(), 0.1f);
    EXPECT_LT(win61.back(), 0.1f);
  }

  // 5. Test Kaiser Window beta parameter
  {
    auto win3 = get_window("kaiser", ntaps, KaiserParam{3.0});
    auto win8 = get_window("kaiser", ntaps, KaiserParam{8.0});

    // Higher beta should have steeper drop-off
    float mid3 = win3[ntaps / 4];
    float mid8 = win8[ntaps / 4];
    EXPECT_GT(mid3, mid8);
  }

  // 6. Verify Invalid Window Throws
  EXPECT_THROW({ get_window("invalid_window", ntaps); }, std::invalid_argument);

  // 7. Verify Parameter Validation
  EXPECT_THROW({ get_window("kaiser", ntaps, KaiserParam{-1.0}); },
      std::invalid_argument);
}

// Verify reconstruction
void verify_reconstruction(const std::vector<float>& original,
    const std::vector<float>& reconstructed) {
  std::cout << "\nReconstruction verification:\n";
  float max_error = 0.0f;
  std::string pass;
  for (size_t i = 0; i < original.size(); ++i) {
    float error = std::abs(original[i] - reconstructed[i]);
    max_error = std::max(max_error, error);
    pass = (error < 1e-4) ? "--> pass" : "--> fail";
    if (i < 10) {  // Print first few samples
      std::cout << "Sample " << i << ": Original=" << original[i]
                << " Reconstructed=" << reconstructed[i] << " Error=" << error
                << pass << "\n";
    }
  }

  std::cout << "Max reconstruction error: " << max_error << "\n";
  assert(max_error < 1e-6f);
}

TEST(STFT, Analysis) {
  // Tiny test parameters
  const size_t fs = 20;
  const float freq = 5.0f;
  const size_t duration_sec = 1;
  const size_t frame_size = 10;
  const size_t hop_size = 5;
  const std::string win_name = "hann";
  // Create test signal (5Hz sine wave in 20Hz system)
  std::vector<float> signal(fs * duration_sec);
  for (size_t i = 0; i < signal.size(); ++i) {
    signal[i] = sin(2 * M_PI * freq * i / fs);
  }

  // Compute STFT
  auto spgram = stft_analysis(signal, frame_size, hop_size, win_name);
  auto reconsig = stft_synth(spgram, frame_size, hop_size, win_name);

  verify_reconstruction(signal, reconsig);
  //StftAnlysInfo info(spectrogram, fs, frame_size, hop_size, win_name);
  // Output to console
  //std::cout << info;
}

TEST(PWELCH, BasicFunctionality) {
  constexpr float fs = 1000.0f;
  constexpr float freq = 50.0f;
  constexpr int N = 1024;
  constexpr int nffts = 512;
  constexpr int win_size = 256;
  constexpr int hop_size = 128;

  // Real-valued test signal
  std::vector<float> signal(N);
  for (int i = 0; i < N; ++i)
    signal[i] = std::sin(2 * M_PI * freq * i / fs);
  std::string title = "complex-psd";
  std::string fname = "./psdplot.png";
  // --- Real-valued PSD ---
  {
    auto psd = pwelch(signal, "hann", win_size, nffts, hop_size, fs,
        WindowParams{}, true, Scale::Density, true, false);

    EXPECT_EQ(psd.freqs.size(), nffts / 2 + 1);
    EXPECT_EQ(psd.psd.size(), nffts / 2 + 1);
    EXPECT_FLOAT_EQ(psd.fs, fs);

    auto max_it = std::max_element(psd.psd.begin(), psd.psd.end());
    int peak_bin = std::distance(psd.psd.begin(), max_it);
    float peak_freq = psd.freqs[peak_bin];
    EXPECT_NEAR(peak_freq, freq, 5.0f);
    plot_psd(psd, title, fname);
  }

  // --- Complex-valued PSD (analytic signal from real) ---
  {
    std::vector<cfloat> csignal(N);
    for (int i = 0; i < N; ++i)
      csignal[i] = {signal[i], 0.0f};  // conjugate-symmetric case

    auto psd = pwelch(csignal, "hann", win_size, nffts, hop_size, fs,
        WindowParams{}, true, Scale::Density, true, true);

    EXPECT_EQ(psd.freqs.size(), nffts);
    EXPECT_EQ(psd.psd.size(), nffts);

    // Symmetry check is only valid here because input is conjugate-symmetric
    for (int k = 1; k < nffts / 2; ++k)
      EXPECT_NEAR(psd.psd[k], psd.psd[nffts - k], 1e-6f);
  }
}

TEST(FilterTest, ButterworthOrder1) {
  auto filter = butterworth(1);

  EXPECT_EQ(filter.zeros.size(), 0);
  EXPECT_EQ(filter.poles.size(), 1);
  EXPECT_EQ(filter.k, 1.0f);

  EXPECT_NEAR(std::real(filter.poles[0]), -1.0f, 1e-6f);
  EXPECT_NEAR(std::imag(filter.poles[0]), 0.0f, 1e-6f);
}

TEST(FilterTest, ButterworthOrder2) {
  auto filter = butterworth(2);

  EXPECT_EQ(filter.zeros.size(), 0);
  EXPECT_EQ(filter.poles.size(), 2);
  EXPECT_EQ(filter.k, 1.0f);

  // Should be at -0.7071 ± j0.7071
  for (const auto& pole : filter.poles) {
    EXPECT_NEAR(std::abs(pole), 1.0f, 1e-6f);
    EXPECT_NEAR(std::real(pole), -0.7071067811865476f, 1e-6f);
    EXPECT_NEAR(std::abs(std::imag(pole)), 0.7071067811865476f, 1e-6f);
  }
}

TEST(FilterTest, ButterworthOrder3) {
  auto filter = butterworth(3);

  EXPECT_EQ(filter.zeros.size(), 0);
  EXPECT_EQ(filter.poles.size(), 3);
  EXPECT_EQ(filter.k, 1.0f);

  // Should have one real pole and one complex conjugate pair
  int real_poles = 0;
  for (const auto& pole : filter.poles) {
    if (std::abs(std::imag(pole)) < 1e-6f) {
      real_poles++;
      EXPECT_NEAR(std::real(pole), -1.0f, 1e-6f);
    }
  }
  EXPECT_EQ(real_poles, 1);
}

// Chebyshev Type I Tests
TEST(FilterTest, Chebyshev1Order1) {
  float rp = 1.0f;  // 1dB ripple
  auto filter = chebyshev1(1, rp);

  EXPECT_EQ(filter.zeros.size(), 0);
  EXPECT_EQ(filter.poles.size(), 1);

  // For N=1, pole should be real and negative
  EXPECT_NEAR(std::imag(filter.poles[0]), 0.0f, 1e-6f);
  EXPECT_LT(std::real(filter.poles[0]), 0.0f);
}

TEST(FilterTest, Chebyshev1Order2) {
  float rp = 1.0f;
  auto filter = chebyshev1(2, rp);

  EXPECT_EQ(filter.zeros.size(), 0);
  EXPECT_EQ(filter.poles.size(), 2);

  // Poles should have negative real parts
  for (const auto& pole : filter.poles) {
    EXPECT_LT(std::real(pole), 0.0f);
  }
}

TEST(PlotZpkTest, CreatesOutputFile) {
  std::string title = "Test Plot";
  std::string fname = "./zpkplot.png";
  std::vector<cfloat> zeros = {cfloat(0.8f, 0.0f), cfloat(0.0f, 0.8f)};
  std::vector<cfloat> poles = {cfloat(-0.8f, 0.0f), cfloat(0.0f, -0.8f)};
  float gain = 1.0f;
  Zpk zpk = {zeros, poles, gain};

  // Call the function
  plot_zpk(zpk, title, fname);
}

TEST(FFTShiftTest, EvenSizeArray) {
  // Test data matching NumPy example
  std::vector<float> x = {0, 1, 2, 3, 4, 5, 6, 7};

  // Test fftshift
  auto fftshifted = fftshift_1d(x);
  std::vector<float> expected_fftshift = {4, 5, 6, 7, 0, 1, 2, 3};
  EXPECT_EQ(fftshifted, expected_fftshift);

  // Test ifftshift (should be same as fftshift for even size)
  auto ifftshifted = ifftshift_1d(x);
  std::vector<float> expected_ifftshift = {4, 5, 6, 7, 0, 1, 2, 3};
  EXPECT_EQ(ifftshifted, expected_ifftshift);

  // Test ifftshift(fftshift(x)) == x
  auto reconstructed1 = ifftshift_1d(fftshifted);
  EXPECT_EQ(reconstructed1, x);

  // Test fftshift(ifftshift(x)) == x
  auto reconstructed2 = fftshift_1d(ifftshifted);
  EXPECT_EQ(reconstructed2, x);
}

TEST(FFTShiftTest, OddSizeArray) {
  // Test with odd size array
  std::vector<float> x = {0, 1, 2, 3, 4, 5, 6};

  // Test fftshift
  auto fftshifted = fftshift_1d(x);
  std::vector<float> expected_fftshift = {4, 5, 6, 0, 1, 2, 3};
  EXPECT_EQ(fftshifted, expected_fftshift);

  // Test ifftshift (different from fftshift for odd size)
  auto ifftshifted = ifftshift_1d(x);
  std::vector<float> expected_ifftshift = {3, 4, 5, 6, 0, 1, 2};
  EXPECT_EQ(ifftshifted, expected_ifftshift);

  // Test ifftshift(fftshift(x)) == x
  auto reconstructed1 = ifftshift_1d(fftshifted);
  EXPECT_EQ(reconstructed1, x);

  // Test fftshift(ifftshift(x)) == x
  auto reconstructed2 = fftshift_1d(ifftshifted);
  EXPECT_EQ(reconstructed2, x);
}

