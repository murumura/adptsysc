#include <Eigen/Dense>
#include <adptsysc/dsplib.hh>
#include <adptsysc/design-lib.hh>
#include <complex>
#include <gtest/gtest.h>
#include <iostream>

using namespace adptsysc;

TEST(Trait, Complex) {
  static_assert(is_complex_v<std::complex<float>>, "Should be true");
  static_assert(!is_complex_v<float>, "Should be false");
}

TEST(CrossCorrelation, Basic) {
  std::vector<float> x = {2, 3, 4};
  std::vector<float> y = {3, 4, 5};

  auto result = cross_correlation(x, y);

  // SciPy/MATLAB full-mode lags: -(Ny-1) … (Nx-1) = -2 … 2
  std::vector<int> expected_lags = {-2, -1, 0, 1, 2};

  // r[k] = sum x[n] * y[n-k]
  std::vector<float> expected_corrs = {
    10,   // k = -2 : 2*5
    23,   // k = -1 : 2*4 + 3*5
    38,   // k =  0 : 2*3 + 3*4 + 4*5
    25,   // k =  1 : 3*3 + 4*4
    12    // k =  2 : 4*3
  };

  EXPECT_EQ(result.lags.size(), expected_lags.size());
  EXPECT_EQ(result.corrs.size(), expected_corrs.size());

  for (std::size_t i = 0; i < expected_lags.size(); ++i) {
    EXPECT_EQ(result.lags[i], expected_lags[i]);
    EXPECT_NEAR(result.corrs[i], expected_corrs[i], 1e-6);
  }
}

TEST(CrossCorrelation, MismatchedLengths) {
  std::vector<float> x = {0.1, 0.2, -0.1, 4.1, -2, 1.5, 0};
  std::vector<float> y = {0.1, 4, -2.2, 1.6, 0.1, 0.1, 0.2};

  auto result = cross_correlation(x, y);

  // Full lag range: -(Ny-1) … (Nx-1) = -6 … 6
  std::vector<int> expected_lags = {-6, -5, -4, -3, -2, -1, 0, 1, 2, 3, 4, 5, 6};

  std::vector<float> expected_corrs = {
    0.02f,
    0.05f,
    0.01f,
    0.99f,
    0.10f,
    0.31f,
    7.54f,
    -12.45f,
    23.19f,
    -10.89f,
    5.80f,
    0.15f,
    0.00f
  };

  EXPECT_EQ(result.lags.size(), expected_lags.size());
  EXPECT_EQ(result.corrs.size(), expected_corrs.size());

  for (size_t i = 0; i < expected_lags.size(); ++i) {
    EXPECT_EQ(result.lags[i], expected_lags[i]);
    EXPECT_NEAR(result.corrs[i], expected_corrs[i], 1e-5);
  }
}

TEST(CrossCorrelation, PositiveLagsOnly) {
  std::vector<float> x = {0.1, 0.2, -0.1, 4.1, -2, 1.5, 0};
  std::vector<float> y = {0.1, 4, -2.2, 1.6, 0.1, 0.1, 0.2};

  auto result = cross_correlation(x, y, -1, "none", true);

  std::vector<int> expected_lags = {0, 1, 2, 3, 4, 5, 6};
  std::vector<float> expected_corrs = {
    7.54f,
    -12.45f,
    23.19f,
    -10.89f,
    5.80f,
    0.15f,
    0.00f
  };

  EXPECT_EQ(result.lags.size(), expected_lags.size());
  EXPECT_EQ(result.corrs.size(), expected_corrs.size());

  for (size_t i = 0; i < expected_lags.size(); ++i) {
    EXPECT_EQ(result.lags[i], expected_lags[i]);
    EXPECT_NEAR(result.corrs[i], expected_corrs[i], 1e-5);
  }
}

TEST(CrossCorrelation, Normalize) {
  std::vector<float> x = {1.0, 0.5, 0.25, 0.125};
  std::vector<float> y = {0.125, 1.0, 0.5, 0.25};

  auto result = cross_correlation(x, y, 2, "normalized");

  // Lag range: -2 … 2
  std::vector<int> expected_lags = {-2,-1,0,1,2};
  
  // Norm = sqrt(1^2 + 0.5^2 + 0.25^2 + 0.125^2) * sqrt(0.125^2 + 1^2 + 0.5^2 + 0.25^2) ≈ 1.3280
  double normval =
    std::sqrt(1 + 0.25 + 0.0625 + 0.015625) *
    std::sqrt(1 + 0.25 + 0.0625 + 0.015625);

  std::vector<double> expected_corrs = {
      0.625 / normval,    // k=2
      1.3125 / normval,   // k=1
      0.78125 / normval,  // k=0
      0.375 / normval,    // k=-1
      0.15625 / normval   // k=-2
  };

  EXPECT_EQ(result.lags.size(), expected_lags.size());
  EXPECT_EQ(result.corrs.size(), expected_corrs.size());

  for (size_t i = 0; i < expected_lags.size(); ++i) {
    EXPECT_EQ(result.lags[i], expected_lags[i]);
    EXPECT_NEAR(result.corrs[i], expected_corrs[i], 1e-5);
  }
}

using cfloat = std::complex<float>;

TEST(EigenFFTWrapper, Real_RoundTrip) {
  constexpr float tol = 1e-4f;
  constexpr std::size_t N = 8;

  std::vector<float> rin(N);
  for (std::size_t i = 0; i < N; ++i) {
    rin[i] = static_cast<float>(i) / static_cast<float>(N);  // deterministic
  }

  std::vector<cfloat> spec;
  std::vector<float> rout;

  EigenFFTWrapper<float> fft(EigenFFTWrapper<float>::FFTMode::Real, N);

  fft.runfft(rin, spec);
  fft.runifft(spec, rout);

  ASSERT_EQ(spec.size(), N);
  ASSERT_EQ(rout.size(), N);

  for (std::size_t i = 0; i < N; ++i) {
    EXPECT_NEAR(rout[i], rin[i], tol);
  }
}

TEST(EigenFFTWrapper, Real_SineWave_Reconstruction) {
  constexpr float tol = 1e-4f;
  constexpr std::size_t N = 8;

  std::vector<float> rin(N);
  for (std::size_t i = 0; i < N; ++i) {
    rin[i] = std::sin(2.0f * static_cast<float>(M_PI) *
                      static_cast<float>(i) / static_cast<float>(N));
  }

  std::vector<cfloat> spec;
  std::vector<float> rout;

  EigenFFTWrapper<float> fft(EigenFFTWrapper<float>::FFTMode::Real, N);

  fft.runfft(rin, spec);
  ASSERT_EQ(spec.size(), N);  // Eigen full spectrum

  fft.runifft(spec, rout);

  ASSERT_EQ(rout.size(), N);

  for (std::size_t i = 0; i < N; ++i) {
    EXPECT_NEAR(rout[i], rin[i], tol);
  }
}

TEST(EigenFFTWrapper, Complex_RoundTrip) {
  constexpr float tol = 1e-4f;
  constexpr std::size_t N = 16;

  std::vector<cfloat> in(N);
  for (std::size_t i = 0; i < N; ++i) {
    in[i] = cfloat(std::cos(i), std::sin(i));
  }

  std::vector<cfloat> spec;
  std::vector<cfloat> out;

  EigenFFTWrapper<float> fft(EigenFFTWrapper<float>::FFTMode::Complex, N);

  fft.runfft(in, spec);
  fft.runifft(spec, out);

  ASSERT_EQ(out.size(), N);

  for (std::size_t i = 0; i < N; ++i) {
    EXPECT_NEAR(out[i].real(), in[i].real(), tol);
    EXPECT_NEAR(out[i].imag(), in[i].imag(), tol);
  }
}

TEST(EigenFFTWrapper, FFTSizeHandling) {
  using Complex = std::complex<float>;
  constexpr float tol = 1e-4f;

  // ============================
  // Case 1: no padding
  // ============================
  {
    const std::size_t N = 1024;
    EigenFFTWrapper<float> fft(EigenFFTWrapper<float>::FFTMode::Complex, N);
    std::vector<Complex> in(N, Complex(1.0f, 0.0f));
    std::vector<Complex> out;

    fft.runfft(in, out);

    ASSERT_EQ(out.size(), N);

    // DC bin = sum of all samples
    EXPECT_NEAR(out[0].real(), static_cast<float>(N), tol);
    EXPECT_NEAR(out[0].imag(), 0.0f, tol);

    // all other bins ≈ 0
    for (std::size_t i = 1; i < N; ++i) {
      EXPECT_NEAR(std::abs(out[i]), 0.0f, tol);
    }
  }

  // ============================
  // Case 2: zero padding
  // ============================
  {
    const std::size_t input_size = 512;
    const std::size_t fft_size   = 2048;

    EigenFFTWrapper<float> fft(EigenFFTWrapper<float>::FFTMode::Complex, fft_size);

    std::vector<Complex> in(input_size, Complex(1.0f, 0.0f));
    std::vector<Complex> out;

    fft.runfft(in, out);

    ASSERT_EQ(out.size(), fft_size);

    // DC still equals sum of original signal (not fft_size)
    EXPECT_NEAR(out[0].real(), static_cast<float>(input_size), tol);
    EXPECT_NEAR(out[0].imag(), 0.0f, tol);
  }
}

TEST(WINDOW, WindowCoeff) {
  // Test basic window properties
  const size_t ntaps = 32;

  // 1. Verify Rectangular Window (all ones)
  {
    auto win = get_window("rect", ntaps);
    EXPECT_EQ(win.size(), ntaps);
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

  // SciPy reference (computed offline with scipy.signal.welch)
  // fs=1000, freq=50, N=1024, hann, win=256, hop=128, nfft=512
  constexpr float scipy_ref_peak_psd = 0.08103163f;

  // Real-valued test signal
  std::vector<float> signal(N);
  for (int i = 0; i < N; ++i)
    signal[i] = std::sin(2 * M_PI * freq * i / fs);
    
  std::string prefix = "real_psd";
  std::string title = "real-psd";
  std::string fname = "./psdplot.png";

  // Real-valued PSD (one-sided)
  {
    auto psd = pwelch(signal, "hann",
                      win_size, nffts, hop_size, fs,
                      WindowParams{}, true, Scale::Density,
                      /*avg=*/true,
                      /*two_side=*/false);

    EXPECT_EQ(psd.freqs.size(), nffts / 2 + 1);
    EXPECT_EQ(psd.psd.size(),   nffts / 2 + 1);
    EXPECT_FLOAT_EQ(psd.fs, fs);

    // Peak frequency check
    auto max_it = std::max_element(psd.psd.begin(), psd.psd.end());
    int peak_bin = std::distance(psd.psd.begin(), max_it);
    float peak_freq = psd.freqs[peak_bin];

    EXPECT_NEAR(peak_freq, freq, 5.0f);

    // Peak PSD value check against SciPy
    EXPECT_NEAR(psd.psd[peak_bin],
                scipy_ref_peak_psd,
                0.01f * scipy_ref_peak_psd);  // 1% relative tolerance

    plot_psd(psd, title, prefix, 0.0f, false, false);

  }

  // Complex-valued PSD (two-sided)
  // NOTE: symmetry holds only because input is real embedded in complex
  {
    std::vector<cfloat> csignal(N);
    for (int i = 0; i < N; ++i)
      csignal[i] = {signal[i], 0.0f};  // conjugate-symmetric case

    auto psd = pwelch(csignal, "hann",
        win_size, nffts, hop_size, fs,
        WindowParams{}, true, Scale::Density,
        /*avg=*/true,
        /*two_side=*/true);

    EXPECT_EQ(psd.freqs.size(), nffts);
    EXPECT_EQ(psd.psd.size(),   nffts);

    // Symmetry check (valid only for conjugate-symmetric input)
    for (int k = 1; k < nffts / 2; ++k) {
      EXPECT_NEAR(psd.psd[k],
                  psd.psd[nffts - k],
                  1e-6f);
    }
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

