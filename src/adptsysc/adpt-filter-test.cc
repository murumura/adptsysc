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

/**
 * @brief Test Fixture for APA Optimizer testing
 */
class APAOptimizerTest : public ::testing::Test {
protected:
  void SetUp() override {
    // Define hyperparameters in JSON
    config = {
        {"mu", 0.5},
        {"gamma", 1e-4},
        {"projection_order", 2}
    };
    
    n_weights = 4;
    optimizer = std::make_unique<APAOptimizer<float, float, float>>(config);
    optimizer->allocate(n_weights);
  }

  std::size_t n_weights;
  json config;
  std::unique_ptr<APAOptimizer<float, float, float>> optimizer;
};


TEST_F(APAOptimizerTest, InitializationTest) {
  EXPECT_EQ(optimizer->get_n_weights(), n_weights);
  EXPECT_EQ(optimizer->get_n_iterations(), 0);
  EXPECT_NEAR(optimizer->get_step_size(), 0.5f, 1e-6);
  
  auto params = optimizer->hyperparams();
  EXPECT_EQ(params["P"], 1);
}

TEST_F(APAOptimizerTest, ResetTest) {
  Eigen::VectorXf x = Eigen::VectorXf::Random(n_weights);
  AFStepState<float> state{x, 1.0f};
  
  Eigen::VectorXf w_acc = Eigen::VectorXf::Zero(n_weights);
  optimizer->step_update(state, w_acc);
  
  EXPECT_EQ(optimizer->get_n_iterations(), 1);
  optimizer->reset();
  EXPECT_EQ(optimizer->get_n_iterations(), 0);
}

/**
 * @brief Validates the convergence on a simple system.
 * The system is d = w_opt * x.
 */
TEST_F(APAOptimizerTest, ConvergenceTest) {
  Eigen::VectorXf w_opt(n_weights);
  w_opt << 0.5f, -0.2f, 0.1f, 0.8f;
  
  Eigen::VectorXf w_acc = Eigen::VectorXf::Zero(n_weights);

  for (int i = 0; i < 100; ++i) {
    Eigen::VectorXf x = Eigen::VectorXf::Random(n_weights);
    AFStepState<float> state{x, w_opt.dot(x)};
    
    optimizer->step_update(state, w_acc);
  }
  
  for (int i = 0; i < n_weights; ++i) {
    EXPECT_NEAR(w_acc[i], w_opt[i], 1e-2);
  }
}

TEST_F(APAOptimizerTest, UpdateHyperparamsTest) {
  json new_config = {
    {"mu", 0.1f},
    {"gamma", 0.01f},
    {"P", 3}
  };
  optimizer->update_hyperparams(new_config);
  EXPECT_NEAR(optimizer->get_step_size(), 0.1f, 1e-6);
  auto params = optimizer->hyperparams();
  EXPECT_NEAR(params["gamma"].get<float>(), 0.01f, 1e-6);
  EXPECT_EQ(params["P"].get<std::size_t>(), 3);
}

// Tests behavior with projection order P=0 (should behave like NLMS).
TEST_F(APAOptimizerTest, ProjectionOrderZeroTest) {
  json p0_config = {{"P", 0}, {"mu", 1.0f}};
  optimizer->update_hyperparams(p0_config);
  
  Eigen::VectorXf w_acc = Eigen::VectorXf::Zero(n_weights);
  Eigen::VectorXf x = Eigen::VectorXf::Ones(n_weights); // norm squared = n_weights
  AFStepState<float> state{x, 1.0f};
  
  // One step update
  optimizer->step_update(state, w_acc);
  
  // For P=0, update is roughly (mu * e * x) / (x'x + gamma)
  // Here e = 1 - 0 = 1. Update = (1 * 1 * ones) / (4 + 1e-4)
  float expected_val = 1.0f / (static_cast<float>(n_weights) + 1e-4f);
  EXPECT_NEAR(w_acc[0], expected_val, 1e-4);
}