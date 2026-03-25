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