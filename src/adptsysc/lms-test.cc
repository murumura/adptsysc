#include <gtest/gtest.h>
#include <adptsysc/adpt-optimizer.hh>
#include <adptsysc/adpt-filter.hh>
#include <vector>
#include <numeric>

using namespace adptsysc;

class LMSFilterTest : public ::testing::Test {
protected:
  const std::size_t n_taps = 4;
  const float mu = 0.1f;

  std::unique_ptr<adptsysc::LMSFilter<float>> filter;
  std::unique_ptr<adptsysc::LMSOptimizer<float>> optimizer;

  void SetUp() override {
    filter = std::make_unique<adptsysc::LMSFilter<float>>(n_taps);
    
    json opt_params;
    opt_params["otype"] = "LMS";
    opt_params["mu"] = mu;

    optimizer.reset(reinterpret_cast<adptsysc::LMSOptimizer<float>*>(adptsysc::create_optimizer<float>(opt_params)));
    optimizer->allocate(n_taps);
  }
};

TEST_F(LMSFilterTest, InitializationTest) {
  EXPECT_EQ(filter->get_n_weights(), n_taps);
  EXPECT_EQ(optimizer->get_n_weights(), n_taps);
  
  auto weights = filter->get_weights_acc();
  for (int i = 0; i < weights.size(); ++i) {
    EXPECT_NEAR(weights[i], 0.0f, 1e-6f);
  }
}

TEST_F(LMSFilterTest, ForwardPassTest) {
  // Set manual weights: [0.5, -0.5, 1.0, 0.0]
  auto weights = filter->get_weights_acc();
  weights << 0.5f, -0.5f, 1.0f, 0.0f;

  Eigen::Matrix<float, Eigen::Dynamic, 1> x(n_taps);
  x << 1.0f, 2.0f, 0.5f, 10.0f;

  // Expect: (0.5*1.0) + (-0.5*2.0) + (1.0*0.5) + (0.0*10.0) = 0.5 - 1.0 + 0.5 = 0.0
  float y = filter->forward(x);
  EXPECT_NEAR(y, 0.0f, 1e-6f);
}

/**
 * @test Verify the optimizer converges to a known system.
 * This simulates a basic System Identification task.
 */
TEST_F(LMSFilterTest, ConvergenceTest) {
  // Target system: y = 0.8 * x[n] + 0.2 * x[n-1]
  Eigen::VectorXf target_w(n_taps);
  target_w << 0.8f, 0.2f, 0.0f, 0.0f;

  const int iterations = 1000;
  std::vector<float> error_sq;

  for (int i = 0; i < iterations; ++i) {
    // Generate random input vector
    Eigen::VectorXf x = Eigen::VectorXf::Random(n_taps);
    
    // Generate desired signal from target system
    float d = target_w.dot(x);

    // Filter output
    float y = filter->forward(x);

    // Perform optimization step
    AFStepState<float> state{x, d};
    auto w_acc = filter->get_weights_acc();
    optimizer->step_update(state, w_acc);

    float error = d - y;
    error_sq.push_back(error * error);
  }

  // Check that the final weights are close to the target system parameters
  auto final_weights = filter->get_weights_acc();
  for (int i = 0; i < n_taps; ++i) {
    EXPECT_NEAR(final_weights[i], target_w[i], 0.01f);
  }

  // Verify that the Mean Square Error has decayed significantly
  float initial_mse = std::accumulate(error_sq.begin(), error_sq.begin() + 20, 0.0f) / 20.0f;
  float final_mse = std::accumulate(error_sq.end() - 20, error_sq.end(), 0.0f) / 20.0f;
  
  EXPECT_LT(final_mse, initial_mse * 0.001f);
}

TEST_F(LMSFilterTest, HyperparameterTest) {
  json params = {{"mu", 0.05f}};
  optimizer->update_hyperparams(params);
  EXPECT_NEAR(optimizer->get_step_size(), 0.05f, 1e-6f);

  json h = optimizer->hyperparams();
  EXPECT_EQ(h["otype"], "lms");
  EXPECT_NEAR(h["mu"].get<float>(), 0.05f, 1e-6f);
  EXPECT_EQ(h["n_weights"], n_taps);
}

TEST_F(LMSFilterTest, ResetTest) {
  filter->get_weights_acc().setConstant(1.0f);
  
  filter->reset();
  
  auto weights = filter->get_weights_acc();
  for (int i = 0; i < weights.size(); ++i) {
    EXPECT_EQ(weights[i], 0.0f);
  }
}