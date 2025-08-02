#pragma once

#include <nlohmann/json.hpp>
#include <memory>
#include <iomanip>
#include <type_traits>

namespace adptsysc {
using json = nlohmann::json;

class Object {
public:
  virtual ~Object() {}
  virtual json hyperparams() const = 0;
  std::string objname() const {
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

  void 
  set_params(PARAMS_T* parameters, PARAMS_T* inference_params = nullptr, PARAMS_T* gradients = nullptr) {
    params = parameters;
    infer_params = inference_params;
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

}
