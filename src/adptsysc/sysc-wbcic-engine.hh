#pragma once

#include <adptsysc/arch.hh>

#include <systemc>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <deque>
#include <cstdint>
#include <limits>
#include <memory>
#include <string_view>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

namespace adptsysc {

template <typename E> struct Context;

enum class WbcicRounding { truncate_floor, nearest_even, nearest_away };

// Non-template arithmetic utilities have out-of-line implementations in engine.cc.
namespace wbcic_detail {
long double get_epsilon(int b);
long double get_scale(int b, std::size_t m, std::size_t k);
sc_dt::sc_fxval quantize_format(sc_dt::sc_fxval value, unsigned width, int integer_bits,
                                WbcicRounding rounding);
std::vector<long double> convolve(const std::vector<long double>& a, const std::vector<long double>& b);
std::vector<long double> compute_fig3b_fir_taps(std::size_t m, std::size_t k, int b);
}  // namespace wbcic_detail

// Values are numerical diagnostics, not registers and not cycle timestamps.
// A trace row exists for every accepted input sample; low-rate nodes are valid only on output_valid.
struct WbcicTrace {
  std::size_t input_index{0};
  std::size_t output_index{0};
  bool input_valid{false};
  bool output_valid{false};
  long double input{0};
  long double shared_integrator{0};
  long double upper_high_delay{0};
  long double upper_integrator{0};
  long double upper_comb{0};
  long double upper_comp{0};
  long double upper_scale{0};
  long double lower_delay{0};
  long double lower_times_two{0};
  long double sum{0};
  long double post_comb{0};
  long double post_comp{0};
  long double output{0};
};

template <typename T, std::size_t Stages>
class CicIntegrator {
public:
  static_assert(Stages > 0, "Integrator must have at least one stage");

  void reset() { states.fill(T{}); }

  T step(T sample) {
    for (auto& state : states) {
      state += sample;
      sample = state;
    }
    return sample;
  }

private:
  std::array<T, Stages> states{};
};

template <typename T, std::size_t Stages>
class CicComb {
public:
  static_assert(Stages > 0, "Comb must have at least one stage");

  void reset() { histories.fill(T{}); }

  T step(T sample) {
    for (auto& history : histories) {
      const T previous = history;
      history = sample;
      sample -= previous;
    }
    return sample;
  }

private:
  std::array<T, Stages> histories{};
};

template <typename T>
class CicCompensator {
public:
  explicit CicCompensator(T a) : a(a) {}

  void reset() {
    delay1 = T{};
    delay2 = T{};
  }

  // Unscaled polynomial 1 + A*z^-1 + z^-2; Scale S is a separate block.
  T step(T sample) {
    const T output = sample + a * delay1 + delay2;
    delay2 = delay1;
    delay1 = sample;
    return output;
  }

private:
  T a{};
  T delay1{};
  T delay2{};
};

template <typename T, std::size_t Length>
class SampleDelay {
public:
  void reset() { states.fill(T{}); position = 0; }

  T step(T sample) {
    if constexpr (Length == 0) {
      return sample;
    } else {
      const T delayed = states[position];
      states[position] = sample;
      position = (position + 1) % Length;
      return delayed;
    }
  }

private:
  std::array<T, Length> states{};
  std::size_t position{0};
};


// Paper notation: E::m = M, E::k = K; Fig. 3(b) uses k_fig = K/2.
// S = B/M^K, B = -2^-b/4, A = -(4/2^-b + 2).
// Fig. 3(b) as drawn: upper (+), lower (-); equivalent high-rate prototype
//                 H_fig3b(z) = -z^-k_fig * H_p(z).
template <typename E>
class WbcicMathEngine {
public:
  using Eval_T = typename E::Eval_T;
  using AccumT = long double;  // Finite precision; validate against independent FIR.
  using CountT = std::size_t;

  static constexpr CountT M = E::m;
  static constexpr CountT K = E::k;
  static constexpr CountT sharp_k = K / 2;
  static_assert(M > 1 && K > 0 && (K % 2) == 0, "Fig. 3(b) needs M > 1 and even K");
  static_assert(E::b >= -32 && E::b <= 32, "Compensator exponent outside supported range");
  static_assert(std::is_floating_point_v<Eval_T>, "This mathematical engine uses a floating Eval_T");

  struct StepResult {
    bool output_valid{false};
    Eval_T output{};
    CountT input_index{0};
    CountT output_index{0};
  };

  WbcicMathEngine() : upper_comp(get_compensator_a()), post_comp(get_compensator_a()) {
    reset();
  }

  static AccumT get_epsilon() { return wbcic_detail::get_epsilon(E::b); }

  static AccumT get_compensator_a() { return -(4.0L / get_epsilon() + 2.0L); }

  static AccumT get_compensator_b() { return -get_epsilon() / 4.0L; }

  static AccumT get_scale() { return wbcic_detail::get_scale(E::b, M, K); }
  static constexpr CountT get_tau() { return K * (M - 1) / 2 + M; }

  void reset() {
    shared_integrator.reset();
    upper_high_delay.reset();
    upper_integrator.reset();
    upper_comb.reset();
    upper_comp.reset();
    lower_low_delay.reset();
    post_comb.reset();
    post_comp.reset();
    input_count_state = output_count_state = phase_state = 0;
    last_trace = {};
  }

  // phase 0 samples accepted indices 0, M, 2M ...; a stall never advances any state.
  // Decimation phase zero selects accepted samples n = 0, M, 2M, ... .
  StepResult step(Eval_T sample, bool sample_fire = true) {
    if (!sample_fire) {
      last_trace = {};
      return {};
    }
    if (!std::isfinite(static_cast<double>(sample))) {
      throw std::invalid_argument("WbcicMathEngine: non-finite sample");
    }
    const CountT n = input_count_state++;
    const AccumT shared = shared_integrator.step(static_cast<AccumT>(sample));
    const AccumT upper_delayed = upper_high_delay.step(shared);
    const AccumT upper_high = upper_integrator.step(upper_delayed);
    last_trace = {};
    last_trace.input_valid = true;
    last_trace.input_index = n;
    last_trace.output_index = output_count_state;
    last_trace.input = sample;
    last_trace.shared_integrator = shared;
    last_trace.upper_high_delay = upper_delayed;
    last_trace.upper_integrator = upper_high;

    const bool fire = phase_state == 0;
    phase_state = (phase_state + 1) % M;
    if (!fire) return {};

    const AccumT upper_comb_out = upper_comb.step(upper_high);
    const AccumT upper_comp_out = upper_comp.step(upper_comb_out);
    const AccumT upper = get_scale() * upper_comp_out;
    const AccumT lower_delayed = lower_low_delay.step(shared);
    const AccumT lower = 2.0L * lower_delayed;
    const AccumT sum = upper - lower;
    const AccumT post_comb_out = post_comb.step(sum);
    const AccumT post_comp_out = post_comp.step(post_comb_out);
    const AccumT output = get_scale() * post_comp_out;
    last_trace.output_valid = true;
    last_trace.upper_comb = upper_comb_out;
    last_trace.upper_comp = upper_comp_out;
    last_trace.upper_scale = upper;
    last_trace.lower_delay = lower_delayed;
    last_trace.lower_times_two = lower;
    last_trace.sum = sum;
    last_trace.post_comb = post_comb_out;
    last_trace.post_comp = post_comp_out;
    last_trace.output = output;
    return {true, static_cast<Eval_T>(output), n, output_count_state++};
  }

  const WbcicTrace& get_last_trace() const { return last_trace; }
  CountT get_input_count() const { return input_count_state; }
  CountT get_output_count() const { return output_count_state; }
  CountT get_decimation_phase() const { return phase_state; }

private:
  CicIntegrator<AccumT, K> shared_integrator{};
  SampleDelay<AccumT, sharp_k> upper_high_delay{};
  CicIntegrator<AccumT, K> upper_integrator{};
  CicComb<AccumT, K> upper_comb{};
  CicCompensator<AccumT> upper_comp;
  SampleDelay<AccumT, sharp_k + 1> lower_low_delay{};
  CicComb<AccumT, K> post_comb{};
  CicCompensator<AccumT> post_comp;
  CountT input_count_state{0};
  CountT output_count_state{0};
  CountT phase_state{0};
  WbcicTrace last_trace{};
};


// Numeric policies use SystemC quantization and mandatory two's-complement SC_WRAP.
// Width and pruning configure the actual state-assignment precision.

template <typename E>
struct WbcicQuantConfig {
  static constexpr std::size_t K = E::k;
  bool enabled{false};
  bool quantize_input{true};
  bool quantize_output{false};
  unsigned output_word_bits{static_cast<unsigned>(E::input_word_bits)};
  unsigned output_integer_bits{static_cast<unsigned>(E::input_integer_bits)};
  unsigned register_bits{0};  // 0 -> E::planning_register_bits, finite-horizon only.
  WbcicRounding rounding{WbcicRounding::truncate_floor};
  std::array<unsigned, K> shared_integrator_bits{};
  std::array<unsigned, K> upper_integrator_bits{};
  std::array<unsigned, K> upper_comb_bits{};
  std::array<unsigned, K> post_comb_bits{};
  std::array<unsigned, K> shared_integrator_prune{};
  std::array<unsigned, K> upper_integrator_prune{};
  std::array<unsigned, K> upper_comb_prune{};
  std::array<unsigned, K> post_comb_prune{};
  unsigned upper_delay_bits{0};
  unsigned lower_delay_bits{0};
  unsigned upper_comp_bits{0};
  unsigned post_comp_bits{0};
  unsigned upper_scale_bits{0};
  unsigned sum_bits{0};
  unsigned output_scale_bits{0};
  unsigned upper_comp_prune{0};
  unsigned post_comp_prune{0};
  unsigned upper_scale_prune{0};
  unsigned sum_prune{0};
  unsigned output_scale_prune{0};

  unsigned get_effective_width(unsigned bits) const {
    return bits != 0 ? bits : register_bits != 0 ? register_bits : E::planning_register_bits;
  }

  void validate() const {
    if (output_word_bits < 2 || output_word_bits > 256 || output_integer_bits == 0 || output_integer_bits > output_word_bits) {
      throw std::invalid_argument("Invalid WBCIC output Q format");
    }
    if (!enabled) return;
    if ((E::m & (E::m - 1)) != 0) {
      throw std::invalid_argument("Fixed WBCIC scale requires power-of-two M");
    }
    if (E::scale_shift < 0 || E::input_frac_bits < 0) {
      throw std::invalid_argument("Invalid fixed-point scaling precision");
    }
    // Internal Q format has enough fractional precision for two Scale factors.
    // Width estimates are a finite-horizon planning budget, not a guaranteed minimum.
    const unsigned internal_frac_bits = static_cast<unsigned>(E::input_frac_bits + 2 * E::scale_shift);
    // The SystemC implementation may impose a stricter maximum word length.
    const auto check = [&](unsigned bits, unsigned prune) {
      const unsigned effective = get_effective_width(bits);
      if (effective < 2 || effective > 256 || effective <= internal_frac_bits ||
          prune > internal_frac_bits || prune > effective - 2) {
        throw std::invalid_argument("WBCIC stage width must retain sign; LSB prune cannot exceed fractional bits");
      }
    };
    check(0, 0);
    for (std::size_t i = 0; i < K; ++i) {
      check(shared_integrator_bits[i], shared_integrator_prune[i]);
      check(upper_integrator_bits[i], upper_integrator_prune[i]);
      check(upper_comb_bits[i], upper_comb_prune[i]);
      check(post_comb_bits[i], post_comb_prune[i]);
    }
    check(upper_delay_bits, 0);
    check(lower_delay_bits, 0);
    check(upper_comp_bits, upper_comp_prune);
    check(post_comp_bits, post_comp_prune);
    check(upper_scale_bits, upper_scale_prune);
    check(sum_bits, sum_prune);
    check(output_scale_bits, output_scale_prune);
  }
};

// sc_fxval is a SystemC arbitrary-precision fixed-point VALUE type, not a custom
// bigint. sc_fix is the finite-word-length boundary applying q/o at assignment.
// sc_fix_fast/sc_fxval_fast are deliberately avoided: they use floating-point
// representation and are not suitable for an exact wide-state fixed reference.
template <typename E>
class WbcicFixedEngine {
public:
  using Eval_T = typename E::Eval_T;
  using CountT = std::size_t;
  using Value = sc_dt::sc_fxval;
  using QuantConfig = WbcicQuantConfig<E>;
  using StepResult = typename WbcicMathEngine<E>::StepResult;
  static constexpr CountT M = E::m;
  static constexpr CountT K = E::k;
  static constexpr CountT sharp_k = K / 2;

  WbcicFixedEngine() { reset(); }

  void set_quant_config(const QuantConfig& config) {
    if (get_input_count() != 0) throw std::logic_error("Reset fixed engine before retuning");
    QuantConfig next = config;
    next.enabled = true;
    next.validate();
    cfg_state = next;
    reset();
  }

  const QuantConfig& get_quant_config() const { return cfg_state; }
  CountT get_input_count() const { return input_count_state; }
  CountT get_output_count() const { return output_count_state; }
  CountT get_decimation_phase() const { return phase_state; }

  void reset() {
    shared_integrators.fill(Value(0));
    upper_integrators.fill(Value(0));
    upper_high_delay.fill(Value(0));
    lower_low_delay.fill(Value(0));
    upper_combs.fill(Value(0));
    post_combs.fill(Value(0));
    upper_comp = {};
    post_comp = {};
    upper_delay_pos = lower_delay_pos = 0;
    phase_state = input_count_state = output_count_state = 0;
    last_trace = {};
  }

  StepResult step(Eval_T sample, bool sample_fire = true) {
    if (!sample_fire) {
      last_trace = {};
      return {};
    }
    if (!std::isfinite(static_cast<double>(sample))) {
      throw std::invalid_argument("WbcicFixedEngine: non-finite input");
    }
    const CountT n = input_count_state++;
    const Value input = cfg_state.quantize_input ? quantize_input(Value(static_cast<double>(sample)))
                                                 : Value(static_cast<double>(sample));
    const Value shared = run_integrators(shared_integrators, input,
                                          cfg_state.shared_integrator_bits, cfg_state.shared_integrator_prune);
    const Value upper_delayed = run_delay(upper_high_delay, upper_delay_pos, shared, cfg_state.upper_delay_bits);
    const Value upper_high = run_integrators(upper_integrators, upper_delayed,
                                             cfg_state.upper_integrator_bits, cfg_state.upper_integrator_prune);
    last_trace = {};
    last_trace.input_valid = true;
    last_trace.input_index = n;
    last_trace.output_index = output_count_state;
    last_trace.input = input.to_double();
    last_trace.shared_integrator = shared.to_double();
    last_trace.upper_high_delay = upper_delayed.to_double();
    last_trace.upper_integrator = upper_high.to_double();
    const bool fire = phase_state == 0;
    phase_state = (phase_state + 1) % M;
    if (!fire) return {};

    const Value upper_comb_out = run_combs(upper_combs, upper_high,
                                           cfg_state.upper_comb_bits, cfg_state.upper_comb_prune);
    const Value upper_comp_out = run_compensator(upper_comp, upper_comb_out,
                                                 cfg_state.upper_comp_bits, cfg_state.upper_comp_prune);
    const Value upper = quantize(-(upper_comp_out >> E::scale_shift),
                                  cfg_state.upper_scale_bits, cfg_state.upper_scale_prune);
    const Value lower_delayed = run_delay(lower_low_delay, lower_delay_pos, shared, cfg_state.lower_delay_bits);
    const Value lower = Value(2) * lower_delayed;
    const Value sum = quantize(upper - lower, cfg_state.sum_bits, cfg_state.sum_prune);
    const Value post_comb_out = run_combs(post_combs, sum, cfg_state.post_comb_bits, cfg_state.post_comb_prune);
    const Value post_comp_out = run_compensator(post_comp, post_comb_out,
                                                cfg_state.post_comp_bits, cfg_state.post_comp_prune);
    Value post = quantize(-(post_comp_out >> E::scale_shift),
                           cfg_state.output_scale_bits, cfg_state.output_scale_prune);
    if (cfg_state.quantize_output) {
      post = quantize_format(post, cfg_state.output_word_bits,
                             static_cast<int>(cfg_state.output_integer_bits));
    }
    last_trace.output_valid = true;
    last_trace.upper_comb = upper_comb_out.to_double();
    last_trace.upper_comp = upper_comp_out.to_double();
    last_trace.upper_scale = upper.to_double();
    last_trace.lower_delay = lower_delayed.to_double();
    last_trace.lower_times_two = lower.to_double();
    last_trace.sum = sum.to_double();
    last_trace.post_comb = post_comb_out.to_double();
    last_trace.post_comp = post_comp_out.to_double();
    last_trace.output = post.to_double();
    return {true, static_cast<Eval_T>(post.to_double()), n, output_count_state++};
  }

  const WbcicTrace& get_last_trace() const { return last_trace; }

private:
  struct CompHistory {
    Value delay1{0};
    Value delay2{0};
  };

  QuantConfig cfg_state{};
  std::array<Value, K> shared_integrators{};
  std::array<Value, K> upper_integrators{};
  std::array<Value, sharp_k> upper_high_delay{};
  std::array<Value, sharp_k + 1> lower_low_delay{};
  std::array<Value, K> upper_combs{};
  std::array<Value, K> post_combs{};
  CompHistory upper_comp{};
  CompHistory post_comp{};
  CountT upper_delay_pos{0};
  CountT lower_delay_pos{0};
  CountT phase_state{0};
  CountT input_count_state{0};
  CountT output_count_state{0};
  WbcicTrace last_trace{};

  Value quantize_format(Value value, unsigned width, int integer_bits) const {
    return wbcic_detail::quantize_format(value, width, integer_bits, cfg_state.rounding);
  }

  Value quantize(Value value, unsigned bits, unsigned prune = 0) const {
    const unsigned width = cfg_state.get_effective_width(bits);
    const int fractional_bits = E::input_frac_bits + 2 * E::scale_shift;
    // Remove fractional LSBs without reducing the signed integer range.
    return quantize_format(value, width - prune, static_cast<int>(width) - fractional_bits);
  }

  Value quantize_input(Value sample) const {
    return quantize_format(sample, E::input_word_bits, E::input_integer_bits);
  }

  Value run_integrators(std::array<Value, K>& states, Value sample,
                                      const std::array<unsigned, K>& bits,
                                      const std::array<unsigned, K>& prune) const {
    for (CountT stage = 0; stage < K; ++stage) {
      states[stage] = quantize(Value(states[stage] + sample), bits[stage], prune[stage]);
      sample = states[stage];
    }
    return sample;
  }

  Value run_combs(std::array<Value, K>& states, Value sample,
                                const std::array<unsigned, K>& bits,
                                const std::array<unsigned, K>& prune) const {
    for (CountT stage = 0; stage < K; ++stage) {
      const Value previous = states[stage];
      states[stage] = quantize(sample, bits[stage], prune[stage]);
      sample = quantize(Value(sample - previous), bits[stage], prune[stage]);
    }
    return sample;
  }

  template <std::size_t N>
  Value run_delay(std::array<Value, N>& states, CountT& position,
                                Value sample, unsigned bits) const {
    static_assert(N > 0);
    const Value previous = states[position];
    states[position] = quantize(sample, bits);
    position = (position + 1) % N;
    return previous;
  }

  Value run_compensator(CompHistory& state, Value sample,
                                      unsigned bits, unsigned prune) const {
    const Value a(static_cast<int>(WbcicMathEngine<E>::get_compensator_a()));
    const Value result = quantize(Value(sample + a * state.delay1 + state.delay2), bits, prune);
    state.delay2 = state.delay1;
    state.delay1 = quantize(sample, bits, prune);
    return result;
  }
};

// Both engines exist independently; mode selection changes only this wrapper.
template <typename E>
class WbcicEngine {
public:
  using Eval_T = typename E::Eval_T;
  using AccumT = typename WbcicMathEngine<E>::AccumT;
  using CountT = std::size_t;
  using StepResult = typename WbcicMathEngine<E>::StepResult;
  using QuantConfig = WbcicQuantConfig<E>;

  WbcicEngine() = default;  // Keep existing TLM/cycle member construction compatible.

  const std::string& get_name() const { return instance_name; }

  static AccumT get_epsilon() { return WbcicMathEngine<E>::get_epsilon(); }
  static AccumT get_compensator_a() { return WbcicMathEngine<E>::get_compensator_a(); }
  static AccumT get_compensator_b() { return WbcicMathEngine<E>::get_compensator_b(); }
  static AccumT get_scale() { return WbcicMathEngine<E>::get_scale(); }
  static constexpr CountT get_tau() { return WbcicMathEngine<E>::get_tau(); }

  void reset() { math_engine.reset(); fx_engine.reset(); }

  void set_quant_config(const QuantConfig& config) {
    if (get_input_count() != 0) throw std::logic_error("Reset WBCIC before retuning");
    config.validate();
    if (config.enabled) fx_engine.set_quant_config(config);
    cfg_state = config;
    reset();
  }

  const QuantConfig& get_quant_config() const { return cfg_state; }
  StepResult step(Eval_T sample, bool sample_fire = true) {
    return cfg_state.enabled ? fx_engine.step(sample, sample_fire) : math_engine.step(sample, sample_fire);
  }
  CountT get_input_count() const {
    return cfg_state.enabled ? fx_engine.get_input_count() : math_engine.get_input_count();
  }
  CountT get_output_count() const {
    return cfg_state.enabled ? fx_engine.get_output_count() : math_engine.get_output_count();
  }
  CountT get_decimation_phase() const {
    return cfg_state.enabled ? fx_engine.get_decimation_phase() : math_engine.get_decimation_phase();
  }
  const WbcicTrace& get_last_trace() const {
    return cfg_state.enabled ? fx_engine.get_last_trace() : math_engine.get_last_trace();
  }

  static bool run_testbench(Context<E>& ctx);
  static std::unique_ptr<WbcicEngine<E>> create(Context<E>& ctx, std::string_view name);

private:
  explicit WbcicEngine(std::string_view name) : instance_name(name) {}

  std::string instance_name{std::string(E::name)};
  QuantConfig cfg_state{};
  WbcicMathEngine<E> math_engine{};
  WbcicFixedEngine<E> fx_engine{};
};

// Independent transfer-function-derived FIR; does not call the Fig. 3(b) engine.
// h_fig[n] = -h_p[n - K/2], h_p = 2*z^-tau*h_c - h_c*h_c.
template <typename E>
class WidebandCicFirRef {
public:
  using Eval_T = typename E::Eval_T;
  using AccumT = typename WbcicMathEngine<E>::AccumT;
  using CountT = std::size_t;
  using StepResult = typename WbcicMathEngine<E>::StepResult;
  static constexpr CountT M = E::m;
  static constexpr CountT K = E::k;
  static constexpr CountT sharp_k = K / 2;
  static constexpr CountT tau = K * (M - 1) / 2 + M;

  WidebandCicFirRef() : taps_state(compute_taps()) { reset(); }

  void reset() { history.assign(taps_state.size(), AccumT{}); phase_state = input_count_state = output_count_state = 0; }

  StepResult step(Eval_T sample, bool sample_fire = true) {
    if (!sample_fire) return {};
    if (!std::isfinite(static_cast<double>(sample))) {
      throw std::invalid_argument("WidebandCicFirRef: non-finite sample");
    }
    const CountT index = input_count_state++;
    history.push_front(static_cast<AccumT>(sample));
    history.pop_back();
    const bool emit = phase_state == 0;
    phase_state = (phase_state + 1) % M;
    if (!emit) return {};
    AccumT sum{};
    for (CountT i = 0; i < taps_state.size(); ++i) sum += taps_state[i] * history[i];
    return {true, static_cast<Eval_T>(sum), index, output_count_state++};
  }

  const std::vector<AccumT>& get_taps() const { return taps_state; }
  CountT get_input_count() const { return input_count_state; }
  CountT get_output_count() const { return output_count_state; }

private:

  std::vector<AccumT> compute_taps() const {
    return wbcic_detail::compute_fig3b_fir_taps(M, K, E::b);
  }

  std::vector<AccumT> taps_state;
  std::deque<AccumT> history;
  CountT phase_state{0};
  CountT input_count_state{0};
  CountT output_count_state{0};
};


}  // namespace adptsysc