#pragma once

#include <adptsysc/object.hh>
#include <adptsysc/design-lib.hh>

#include <algorithm>
#include <cassert>
#include <complex>
#include <cstddef>
#include <memory>
#include <stdexcept>
#include <type_traits>
#include <vector>

#include <Eigen/Dense>

namespace adptsysc {

// SampleT: stream / desired-signal representation.
// CoeffT: deployed or quantized coefficient representation.
// WorkT: full-precision training and numerical-computation representation.
template <typename SampleT, typename CoeffT = SampleT, typename WorkT = SampleT>
class AdaptiveFilter : public ParametricObject<CoeffT> {
public:
  using SampleVec = Eigen::Matrix<SampleT, Eigen::Dynamic, 1>;
  using WorkVec = Eigen::Matrix<WorkT, Eigen::Dynamic, 1>;
  using WorkMat = Eigen::Matrix<WorkT, Eigen::Dynamic, Eigen::Dynamic>;
  using CoeffVec = Eigen::Matrix<CoeffT, Eigen::Dynamic, 1>;
  using RealT = typename Eigen::NumTraits<WorkT>::Real;

  // Source-compatible aliases for the pre-WorkT API.
  using DataVec = WorkVec;
  using DataMatrix = WorkMat;
  using AccVec = WorkVec;
  using ParamVec = CoeffVec;

  static_assert(std::is_floating_point_v<RealT>,
                "WorkT must be a floating-point or complex floating-point type");

  virtual ~AdaptiveFilter() = default;

  virtual std::size_t get_n_weights() const = 0;
  virtual void reset() = 0;

  // The prediction stays in WorkT so a SampleT such as int16_t does not
  // truncate the error calculation used by an adaptive optimizer.
  virtual WorkT forward(const Eigen::Ref<const SampleVec>& x) const = 0;

  std::size_t n_params() const override {
    return get_n_weights();
  }

  void init_params(float* params_full_precision) override {
    std::fill(params_full_precision,
              params_full_precision + get_n_weights(),
              0.0f);
  }

  virtual Eigen::Ref<WorkVec> train_weights() = 0;
  virtual Eigen::Ref<CoeffVec> coeffs() = 0;

  // Compatibility shims for existing callers. New code should use
  // train_weights() and coeffs(), which make the distinct domains explicit.
  Eigen::Ref<WorkVec> get_weights_acc() {
    return train_weights();
  }

  Eigen::Ref<CoeffVec> get_weights_q() {
    return coeffs();
  }

protected:
  using ParametricObject<CoeffT>::grads;
  using ParametricObject<CoeffT>::infer_params;
  using ParametricObject<CoeffT>::params;
};

template <typename SampleT, typename CoeffT = SampleT, typename WorkT = SampleT>
class LMSFilter : public AdaptiveFilter<SampleT, CoeffT, WorkT> {
public:
  using Base = AdaptiveFilter<SampleT, CoeffT, WorkT>;
  using SampleVec = typename Base::SampleVec;
  using WorkVec = typename Base::WorkVec;
  using CoeffVec = typename Base::CoeffVec;

  explicit LMSFilter(std::size_t n_weights)
      : n_weights_(n_weights),
        train_weights_(WorkVec::Zero(n_weights)),
        coeffs_(CoeffVec::Zero(n_weights)) {}

  std::size_t get_n_weights() const override {
    return n_weights_;
  }

  void reset() override {
    train_weights_.setZero();
    coeffs_.setZero();
  }

  WorkT forward(const Eigen::Ref<const SampleVec>& x) const override {
    assert(static_cast<std::size_t>(x.size()) == n_weights_);
    return train_weights_.dot(x.template cast<WorkT>());
  }

  void set_params_impl(CoeffT* input_coeffs,
                       CoeffT* /*inference_coeffs*/,
                       CoeffT* /*gradients*/) override {
    if (!input_coeffs) {
      return;
    }

    Eigen::Map<const CoeffVec> incoming(input_coeffs,
                                        static_cast<Eigen::Index>(n_weights_));
    coeffs_ = incoming;
    train_weights_ = coeffs_.template cast<WorkT>();
  }

  Eigen::Ref<WorkVec> train_weights() override {
    return train_weights_;
  }

  Eigen::Ref<CoeffVec> coeffs() override {
    return coeffs_;
  }

  json get_hyperparams() const override {
    return {
      {"otype", "lms_filter"},
      {"n_taps", n_weights_},
    };
  }

private:
  std::size_t n_weights_ = 0;
  WorkVec train_weights_;
  CoeffVec coeffs_;
};

template <typename SampleT, typename CoeffT = SampleT, typename WorkT = SampleT>
class RLSFilter : public AdaptiveFilter<SampleT, CoeffT, WorkT> {
public:
  using Base = AdaptiveFilter<SampleT, CoeffT, WorkT>;
  using SampleVec = typename Base::SampleVec;
  using WorkVec = typename Base::WorkVec;
  using CoeffVec = typename Base::CoeffVec;

  explicit RLSFilter(std::size_t n_weights)
      : n_weights_(n_weights),
        train_weights_(WorkVec::Zero(n_weights)),
        coeffs_(CoeffVec::Zero(n_weights)) {}

  std::size_t get_n_weights() const override {
    return n_weights_;
  }

  void reset() override {
    train_weights_.setZero();
    coeffs_.setZero();
  }

  WorkT forward(const Eigen::Ref<const SampleVec>& x) const override {
    assert(static_cast<std::size_t>(x.size()) == n_weights_);
    return train_weights_.dot(x.template cast<WorkT>());
  }

  void set_params_impl(CoeffT* input_coeffs,
                       CoeffT* /*inference_coeffs*/,
                       CoeffT* /*gradients*/) override {
    if (!input_coeffs) {
      return;
    }

    Eigen::Map<const CoeffVec> incoming(input_coeffs,
                                        static_cast<Eigen::Index>(n_weights_));
    coeffs_ = incoming;
    train_weights_ = coeffs_.template cast<WorkT>();
  }

  Eigen::Ref<WorkVec> train_weights() override {
    return train_weights_;
  }

  Eigen::Ref<CoeffVec> coeffs() override {
    return coeffs_;
  }

  json get_hyperparams() const override {
    return {
      {"otype", "rls_filter"},
      {"n_taps", n_weights_},
    };
  }

private:
  std::size_t n_weights_ = 0;
  WorkVec train_weights_;
  CoeffVec coeffs_;
};

template <typename SampleT, typename CoeffT = SampleT, typename WorkT = SampleT>
class APAFilter : public AdaptiveFilter<SampleT, CoeffT, WorkT> {
public:
  using Base = AdaptiveFilter<SampleT, CoeffT, WorkT>;
  using SampleVec = typename Base::SampleVec;
  using WorkVec = typename Base::WorkVec;
  using CoeffVec = typename Base::CoeffVec;

  explicit APAFilter(std::size_t n_weights)
      : n_weights_(n_weights),
        train_weights_(WorkVec::Zero(n_weights)),
        coeffs_(CoeffVec::Zero(n_weights)) {}

  std::size_t get_n_weights() const override {
    return n_weights_;
  }

  void reset() override {
    train_weights_.setZero();
    coeffs_.setZero();
  }

  WorkT forward(const Eigen::Ref<const SampleVec>& x) const override {
    assert(static_cast<std::size_t>(x.size()) == n_weights_);
    return train_weights_.dot(x.template cast<WorkT>());
  }

  void set_params_impl(CoeffT* input_coeffs,
                       CoeffT* /*inference_coeffs*/,
                       CoeffT* /*gradients*/) override {
    if (!input_coeffs) {
      return;
    }

    Eigen::Map<const CoeffVec> incoming(input_coeffs,
                                        static_cast<Eigen::Index>(n_weights_));
    coeffs_ = incoming;
    train_weights_ = coeffs_.template cast<WorkT>();
  }

  Eigen::Ref<WorkVec> train_weights() override {
    return train_weights_;
  }

  Eigen::Ref<CoeffVec> coeffs() override {
    return coeffs_;
  }

  json get_hyperparams() const override {
    return {
      {"otype", "affine_projection_filter"},
      {"n_taps", n_weights_},
    };
  }

private:
  std::size_t n_weights_ = 0;
  WorkVec train_weights_;
  CoeffVec coeffs_;
};

template <typename SampleT, typename CoeffT = SampleT, typename WorkT = SampleT>
class OverlapSaveFdaf : public AdaptiveFilter<SampleT, CoeffT, WorkT> {
public:
  using Base = AdaptiveFilter<SampleT, CoeffT, WorkT>;
  using SampleVec = typename Base::SampleVec;
  using WorkVec = typename Base::WorkVec;
  using CoeffVec = typename Base::CoeffVec;
  using RealT = typename Base::RealT;

  static_assert(!Eigen::NumTraits<WorkT>::IsComplex,
                "OverlapSaveFdaf expects a real WorkT; it constructs complex FFT samples internally");
  using CxT = std::complex<RealT>;

  OverlapSaveFdaf(std::size_t n_weights,
                  std::shared_ptr<EigenFFTWrapper<RealT>> fft_ptr)
      : n_weights_(n_weights),
        block_size_(n_weights),
        fft_size_(2 * n_weights),
        fft_(std::move(fft_ptr)),
        input_history_(WorkVec::Zero(block_size_)),
        last_output_(WorkVec::Zero(block_size_)),
        train_weights_(WorkVec::Zero(block_size_)),
        coeffs_(CoeffVec::Zero(block_size_)) {}

  std::size_t get_n_weights() const override {
    return n_weights_;
  }

  void reset() override {
    input_history_.setZero();
    last_output_.setZero();
    train_weights_.setZero();
    coeffs_.setZero();
    last_input_freq_.clear();
  }

  // Compatibility API: FDAF operates on blocks, so forward() returns the last
  // output sample from the most recently processed block.
  WorkT forward(const Eigen::Ref<const SampleVec>& x) const override {
    assert(static_cast<std::size_t>(x.size()) == block_size_);
    return last_output_[static_cast<Eigen::Index>(block_size_ - 1)];
  }

  void forward_block(const Eigen::Ref<const SampleVec>& input,
                     const std::vector<CxT>& frequency_weights,
                     Eigen::Ref<WorkVec> output) {
    if (static_cast<std::size_t>(input.size()) != block_size_) {
      throw std::invalid_argument("input block has the wrong size");
    }
    if (frequency_weights.size() != fft_size_) {
      throw std::invalid_argument("frequency_weights has the wrong size");
    }
    if (static_cast<std::size_t>(output.size()) != block_size_) {
      throw std::invalid_argument("output block has the wrong size");
    }

    WorkVec input_work = input.template cast<WorkT>();
    Eigen::Matrix<RealT, Eigen::Dynamic, 1> time_block(
        static_cast<Eigen::Index>(fft_size_));
    time_block << input_history_, input_work;

    last_input_freq_ = fft_->fft(time_block);

    std::vector<CxT> output_freq(fft_size_);
    for (std::size_t i = 0; i < fft_size_; ++i) {
      output_freq[i] = last_input_freq_[i] * frequency_weights[i];
    }

    const auto output_time = fft_->ifft(output_freq);
    output = output_time.tail(static_cast<Eigen::Index>(block_size_)).real();
    last_output_ = output;
    input_history_ = input_work;
  }

  const std::vector<CxT>& last_input_freq() const {
    return last_input_freq_;
  }

  void update_weight_cache(const std::vector<CxT>& frequency_weights) {
    if (frequency_weights.size() != fft_size_) {
      throw std::invalid_argument("frequency_weights has the wrong size");
    }

    const auto time_weights = fft_->ifft(frequency_weights);
    train_weights_ = time_weights.head(static_cast<Eigen::Index>(block_size_)).real();
    coeffs_ = train_weights_.template cast<CoeffT>();
  }

  void set_params_impl(CoeffT* input_coeffs,
                       CoeffT* /*inference_coeffs*/,
                       CoeffT* /*gradients*/) override {
    if (!input_coeffs) {
      return;
    }

    Eigen::Map<const CoeffVec> incoming(input_coeffs,
                                        static_cast<Eigen::Index>(n_weights_));
    coeffs_ = incoming;
    train_weights_ = coeffs_.template cast<WorkT>();
  }

  Eigen::Ref<WorkVec> train_weights() override {
    return train_weights_;
  }

  Eigen::Ref<CoeffVec> coeffs() override {
    return coeffs_;
  }

  json get_hyperparams() const override {
    return {
      {"otype", "overlap_save_fdaf"},
      {"n_taps", n_weights_},
      {"block_size", block_size_},
      {"fft_size", fft_size_},
    };
  }

private:
  std::size_t n_weights_ = 0;
  std::size_t block_size_ = 0;
  std::size_t fft_size_ = 0;

  WorkVec input_history_;
  WorkVec last_output_;
  WorkVec train_weights_;
  CoeffVec coeffs_;

  std::vector<CxT> last_input_freq_;
  std::shared_ptr<EigenFFTWrapper<RealT>> fft_;
};

}  // namespace adptsysc
