#include <Eigen/Dense>
#include <adptsysc/dsplib.hh>
#include <complex>
#include <gtest/gtest.h>
#include <iostream>

using namespace adptsysc;

TEST(Xcorr, Basic) {
  auto xcorr = Xcorr<double>();  // Reset
  auto result = xcorr.eval({2, 3, 4}, {3, 4, 5}, -1, "none");
  std::vector<int> expected_lags = {2, 1, 0, -1, -2};
  std::vector<double> expected_corrs = {10, 23, 38, 25, 12};

  // Test mismatched signal lengths
  std::vector<double> x = {0.1, 0.2, -0.1, 4.1, -2, 1.5, 0};
  std::vector<double> y = {0.1, 4, -2.2, 1.6, 0.1, 0.1, 0.2};

  result = xcorr.eval({0.1, 0.2, -0.1, 4.1, -2, 1.5, 0},
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

  result = xcorr.eval(x, y, -1, "none", true);
  EXPECT_EQ(result.lags.size(), expected_lags.size() / 2 + 1);
  EXPECT_EQ(result.corrs.size(), expected_corrs.size() / 2 + 1);
  for (size_t i = 0; i < expected_lags.size() / 2 + 1; ++i) {
    EXPECT_EQ(result.lags[i], expected_lags[i]);
    EXPECT_NEAR(result.corrs[i], expected_corrs[i], 1e-6);
  }
}

TEST(Xcorr, Normalize) {
  auto xcorr = Xcorr<double>();  // Reset
  std::vector<double> x = {1.0, 0.5, 0.25, 0.125};
  std::vector<double> y = {0.125, 1.0, 0.5, 0.25};  // Shifted x
  auto result = xcorr.eval(x, y, 2, "normalized");

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