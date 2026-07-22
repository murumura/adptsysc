#include <adptsysc/design-lib.hh>
#include <gtest/gtest.h>
using namespace adptsysc;

TEST(OLSFFTConvTest, DeterministicExample) {
  constexpr int L = 16;
  constexpr int M = 5;
  constexpr int N = 8;

  Eigen::VectorXf x(L);
  x << 0.2f, -0.1f, 0.4f, 0.7f, -0.5f, 0.3f, 0.9f, -0.2f,
       0.1f, 0.6f, -0.4f, 0.8f, -0.3f, 0.5f, 0.2f, -0.6f;

  Eigen::VectorXf h(M);
  h << 0.3f, -0.2f, 0.5f, 0.1f, -0.4f;

  Eigen::VectorXf y_ref(20);
  y_ref << 
    0.06f, -0.07f, 0.24f, 0.10f, -0.18f,
    0.62f, -0.13f, -0.42f, 0.75f, 0.03f,
   -0.57f, 0.71f, -0.43f, 0.33f, 0.05f,
   -0.32f, 0.39f, -0.48f, -0.14f, 0.24f;

  auto y = olsfft_conv<float>(x, h, N);

  EXPECT_EQ(y.size(), y_ref.size());

  for (int i = 0; i < y.size(); ++i) {
    EXPECT_NEAR(y[i], y_ref[i], 1e-5)
        << "Mismatch at index " << i;
  }
}

TEST(DesignLibJsonTest, RealVectorRoundTrip) {
  const std::vector<float> in{1.25f, -2.5f, 0.0f};

  const json encoded = realvec_to_json(in);
  const auto out = realvec_from_json<float>(encoded);

  ASSERT_EQ(out.size(), in.size());
  for (std::size_t i = 0; i < in.size(); ++i) {
    EXPECT_FLOAT_EQ(out[i], in[i]);
  }
}

TEST(DesignLibJsonTest, LocalComplexVectorRoundTrip) {
  using CxT = std::complex<float>;
  const std::vector<CxT> in{
      CxT(1.25f, -0.5f),
      CxT(-2.0f, 3.5f),
  };

  const json encoded = cvec_to_local_json(in);
  const auto out = cvec_from_local_json<float>(encoded);

  ASSERT_EQ(out.size(), in.size());
  for (std::size_t i = 0; i < in.size(); ++i) {
    EXPECT_FLOAT_EQ(out[i].real(), in[i].real());
    EXPECT_FLOAT_EQ(out[i].imag(), in[i].imag());
  }
}

TEST(EigenFFTWrapperTest, ComplexRoundTrip) {
  using T = float;
  using CxT = std::complex<T>;
  using FFTT = EigenFFTWrapper<T>;
  using FFTMode = FFTT::FFTMode;

  const std::vector<CxT> in{
      CxT(1.0f, 0.0f),
      CxT(2.0f, -1.0f),
      CxT(-0.5f, 0.25f),
      CxT(0.0f, 0.75f),
      CxT(-1.0f, -0.5f),
      CxT(0.25f, 0.0f),
      CxT(0.5f, 1.0f),
      CxT(-0.75f, 0.5f),
  };

  FFTT fft(FFTMode::Complex, in.size());

  std::vector<CxT> freq;
  std::vector<CxT> recovered;
  fft.get_cmplxfft(in, freq);
  fft.get_cmplxifft(freq, recovered);

  ASSERT_EQ(recovered.size(), in.size());
  for (std::size_t i = 0; i < in.size(); ++i) {
    EXPECT_NEAR(recovered[i].real(), in[i].real(), 1e-5f);
    EXPECT_NEAR(recovered[i].imag(), in[i].imag(), 1e-5f);
  }
}
