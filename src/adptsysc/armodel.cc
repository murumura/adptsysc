#include <adptsysc/dsplib.hh>
#include <random>

namespace adptsysc {

// a[1..p], AR coefficients
template <typename T>
std::vector<T>
ar_process(std::span<const T> a, T noise_var, 
           int N, std::optional<uint64_t> seed) {
  const int p = static_cast<int>(a.size());
  if (noise_var <= T(0))
    throw std::invalid_argument("Noise variance must be positive");

  std::vector<T> x(N, T(0));

  std::mt19937_64 rng(seed.value_or(std::random_device{}()));
  std::normal_distribution<T> dist(0, std::sqrt(noise_var));

  // init
  for (int n = 0; n < std::max(p, 1); ++n)
    x[n] = dist(rng);

  // recursion
  for (int n = p; n < N; ++n) {
    T sum = T(0);
    for (int k = 1; k <= p; ++k)
      sum += a[k-1] * x[n-k];

    x[n] = -sum + dist(rng);
  }

  return x;
}


}  // namespace adptsysc
