#pragma once

#include <json/json.hpp>
#include <memory>

namespace adptsysc {
using json = nlohmann::json;

class Object {
public:
  virtual ~Object() {}
  virtual json hyperparams() const = 0;
  std::string name() const {
    return hyperparams().value("otype", "<Unknown>");
  }
};

class ObjectWithMutableHyperparams : public Object {
public:
  virtual ~ObjectWithMutableHyperparams() {}
  virtual void update_hyperparams(const json& params) = 0;
};

// Optional parameterized interface for differentiable objects
template <typename PARAMS_T>
class ParametricObject : public Object {
public:
  virtual ~ParametricObject() = default;

  virtual void 
  set_params_impl(PARAMS_T* parameters, PARAMS_T* inference_params, PARAMS_T* gradients) = 0;

  void set_params(PARAMS_T* parameters, PARAMS_T* inference_params, PARAMS_T* gradients) {
    params = parameters;
    infer_params = infer_params;
    grads = gradients;
    set_params_impl(params, infer_params, grads);
  }

virtual void init_params(float* params_full_precision, float scale = 1) = 0;
virtual std::size_t n_params() const = 0;

PARAMS_T* parameters() const { return params; }
PARAMS_T* inference_params() const { return infer_params; }
PARAMS_T* gradients() const { return grads; }

protected:
  PARAMS_T* params = nullptr;
  PARAMS_T* infer_params = nullptr;
  PARAMS_T* grads = nullptr;
};

template <typename T>
class Filter : public Object {
public:
  virtual ~Filter() = default;

  virtual void eval(const std::vector<T>& input, std::vector<T>& output) = 0;

  virtual std::optional<T> 
  process_sample(std::optional<T> in_sample = std::nullopt) { return std::nullopt; }

  virtual void reset() = 0;
  virtual json freqresp(std::size_t n_points = 1024) const = 0;

  float sample_rate() const { return fs; }
  void set_sample_rate(float smpl_rate) { fs = smpl_rate; }

  // For wiring or system integration purposes
  virtual std::vector<std::pair<uint32_t, uint32_t>> 
  io_structure() const { return {}; }

protected:
  float fs = 1.0f;
};

// FIR filter
template <typename T>
class FIRFilter : public Filter<T> {
public:
  explicit FIRFilter(std::size_t n_taps = 64);

  void set_taps(const std::vector<T>& taps);
  const std::vector<T>& taps() const;

  void eval(const std::vector<T>& input, std::vector<T>& output) override;
  void reset() override;
  json freqresp(size_t n_points = 1024) const override;
  json hyperparams() const override;

protected:
  std::vector<T> taps;
  std::vector<T> states;
};

// Optional interface for hardware-like processing
class HardwareInterface {
public:
  virtual void process_sample(float input) = 0;
  virtual float get_output() const = 0;

  virtual std::string in_signal_name() const { return "input"; }
  virtual std::string out_signal_name() const { return "output"; }
  virtual std::string clk_signal_name() const { return "clk"; }

  virtual ~HardwareInterface() = default;
};

}
