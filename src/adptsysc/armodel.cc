#include <adptsysc/dsplib.hh>
#include <random>

namespace adptsysc {

template <typename T>
std::vector<T> 
ARModel<T>::eval(T drive_var, const int sample_size, 
                std::optional<int> seed) const {
  if (drive_var <= T(0)) {
    throw std::invalid_argument("Noise variance must be positive");
  }

  std::vector<T> x(sample_size, T(0));
  std::mt19937_64 rng(seed.value_or(std::random_device{}()));
  std::normal_distribution<T> dist(0, std::sqrt(drive_var));

  // Initialize first max(ar_ord, 1) samples with noise
  const int init_samples = std::max(ar_ord, 1);
  std::generate_n(x.begin(), init_samples, [&] { return dist(rng); });
  // AR recursion
  for (int n = init_samples; n < sample_size; ++n) {
    T sum = T(0);
    for (int k = 1; k <= ar_ord; ++k) {
      sum += a_params[k] * x[n - k];
    }
    x[n] = -sum + dist(rng);
  }

  return x;
}

template <typename T> void 
ARModel<T>::eval_to(std::span<T> output, T drive_var, 
                    std::optional<int> seed) const {
  if (output.size() < ar_ord) {
    throw std::invalid_argument("Output buffer too small");
  }

  std::mt19937_64 rng(seed.value_or(std::random_device{}()));
  std::normal_distribution<T> dist(0, std::sqrt(drive_var));

  // Initialize first samples
  std::generate_n(output.begin(), ar_ord, [&] { return dist(rng); });

  // AR recursion
  for (int n = ar_ord; n < output.size(); ++n) {
    T sum = T(0);
    for (int k = 1; k <= ar_ord; ++k) {
      sum += a_params[k] * output[n - k];
    }
    output[n] = -sum + dist(rng);
  }
}

}  // namespace adptsysc
