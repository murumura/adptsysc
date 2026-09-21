#include <adptsysc/config.hh>
#include <adptsysc/adptsysc.hh>
#include <adptsysc/sysc-wbcic-engine.hh>
#include <adptsysc/syscfx-utils.hh>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <iomanip>
#include <limits>
#include <memory>
#include <random>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace adptsysc {
using E = ADPT_TARGET;

namespace wbcic_detail {

long double get_epsilon(int b) {
  long double epsilon = 1.0L;
  for (int i = 0; i < (b < 0 ? -b : b); ++i) {
    epsilon *= b < 0 ? 2.0L : 0.5L;
  }
  return epsilon;
}

long double get_scale(int b, std::size_t m, std::size_t k) {
  long double gain = 1.0L;
  for (std::size_t stage = 0; stage < k; ++stage) gain *= static_cast<long double>(m);
  return (-get_epsilon(b) / 4.0L) / gain;
}

// Explicit SC_WRAP is required for CIC modulo arithmetic.
sc_dt::sc_fxval quantize_format(sc_dt::sc_fxval value, unsigned width, int integer_bits,
                                WbcicRounding rounding) {
  if (width < 2 || width > 256 || integer_bits < 1 || integer_bits > static_cast<int>(width)) {
    throw std::invalid_argument("Invalid WBCIC word length / integer bits");
  }
  sc_dt::sc_q_mode mode = sc_dt::SC_TRN;
  if (rounding == WbcicRounding::nearest_even) mode = sc_dt::SC_RND_CONV;
  if (rounding == WbcicRounding::nearest_away) {
    // Quantize abs(value) with positive SC_TRN, then restore the sign.
    // A W+1 temporary preserves |minimum_signed_value| before sign restoration.
    const int fractional_bits = static_cast<int>(width) - integer_bits;
    sc_dt::sc_fxval half(1);
    if (fractional_bits + 1 >= 0) half >>= fractional_bits + 1;
    else half <<= -(fractional_bits + 1);
    const bool negative = value < sc_dt::sc_fxval(0);
    sc_dt::sc_fxval magnitude = negative ? sc_dt::sc_fxval(-value) : value;
    magnitude += half;
    sc_dt::sc_fix positive(static_cast<int>(width) + 1, integer_bits + 1,
                            sc_dt::SC_TRN, sc_dt::SC_WRAP);
    positive = magnitude;
    value = sc_dt::sc_fxval(positive);
    if (negative) value = -value;
    mode = sc_dt::SC_TRN;
  }
  sc_dt::sc_fix boundary(static_cast<int>(width), integer_bits, mode, sc_dt::SC_WRAP);
  boundary = value;
  return sc_dt::sc_fxval(boundary);
}

std::vector<long double> convolve(const std::vector<long double>& a,
                                  const std::vector<long double>& b) {
  if (a.empty() || b.empty()) return {};
  std::vector<long double> out(a.size() + b.size() - 1, 0.0L);
  for (std::size_t i = 0; i < a.size(); ++i) {
    for (std::size_t j = 0; j < b.size(); ++j) out[i + j] += a[i] * b[j];
  }
  return out;
}

// Independent transfer-function FIR design, separate from the Fig. 3(b) engine.
std::vector<long double> 
compute_fig3b_fir_taps(std::size_t m, std::size_t k, int b) {
  if (m <= 1 || k == 0 || k % 2 != 0) 
    throw std::invalid_argument("Invalid WBCIC FIR parameters");
  const std::size_t sharp_k = k / 2;
  const std::size_t tau = k * (m - 1) / 2 + m;
  std::vector<long double> boxcar(m, 1.0L);
  std::vector<long double> cic{1.0L};
  for (std::size_t stage = 0; stage < k; ++stage) 
    cic = convolve(cic, boxcar);
  long double norm = 1.0L;
  for (std::size_t stage = 0; stage < k; ++stage) 
    norm *= static_cast<long double>(m);
  for (auto& tap : cic) 
    tap /= norm;
  const long double eps = get_epsilon(b);
  std::vector<long double> comp(2 * m + 1, 0.0L);
  comp[0] = comp[2 * m] = -eps / 4.0L;
  comp[m] = 1.0L + eps / 2.0L;
  const auto hc = convolve(cic, comp);
  auto hp = convolve(hc, hc);
  for (auto& tap : hp) tap = -tap;
  for (std::size_t i = 0; i < hc.size(); ++i) hp[tau + i] += 2.0L * hc[i];
  // Fig. 3(b) upper-minus-lower is -z^-sharp_k times sharpened H_p.
  std::vector<long double> fig3b_fir_taps(sharp_k + hp.size(), 0.0L);
  for (std::size_t i = 0; i < hp.size(); ++i) 
    fig3b_fir_taps[sharp_k + i] = -hp[i];
  return fig3b_fir_taps;
}

}  // namespace wbcic_detail

template <typename E>
std::unique_ptr<WbcicEngine<E>>
WbcicEngine<E>::create(Context<E>& ctx, std::string_view name) {
  (void)ctx;
  return std::unique_ptr<WbcicEngine<E>>(new WbcicEngine<E>(name));
}

namespace wbcic_detail {

// OutputFile writes the actual file on close(ctx); always close each export.
template <typename E>
std::unique_ptr<OutputFile<E>> 
open_export(Context<E>& ctx, const std::filesystem::path& path) {
  if (!path.parent_path().empty()) 
    std::filesystem::create_directories(path.parent_path());
  return OutputFile<E>::open(ctx, path.string(), 1 << 20, 0666);
}

inline std::string trace_line(const WbcicTrace& trace, const std::string& model) {
  std::ostringstream out;
  out << std::setprecision(std::numeric_limits<long double>::max_digits10)
      << "model=" << model << " n=" << trace.input_index
      << " m=" << trace.output_index << " valid=" << int(trace.output_valid)
      << " in=" << trace.input << " shared=" << trace.shared_integrator
      << " upper_delay=" << trace.upper_high_delay << " upper_integrator=" << trace.upper_integrator;
  if (trace.output_valid) {
    out << " upper_comb=" << trace.upper_comb << " upper_comp=" << trace.upper_comp
        << " upper_scale=" << trace.upper_scale << " lower_delay=" << trace.lower_delay
        << " lower_x2=" << trace.lower_times_two << " sum=" << trace.sum
        << " post_comb=" << trace.post_comb << " post_comp=" << trace.post_comp
        << " out=" << trace.output;
  }
  return out.str();
}

template <typename E>
std::vector<typename E::Eval_T> 
make_test_input(const std::string& mode) {
  using T = typename E::Eval_T;
  const std::size_t count = mode == "impulse" ? 256 : mode == "constant" ? 192 :
                            mode == "multitone" ? 320 : 384;
  std::vector<T> data(count, T{});
  std::mt19937 rng(0x5a17u);
  std::uniform_int_distribution<int> distribution(-6, 6);
  for (std::size_t n = 0; n < count; ++n) {
    if (mode == "impulse") data[n] = n == 0 ? T(0.5) : T(0);
    else if (mode == "constant") data[n] = T(0.125);
    else if (mode == "multitone") {
      const double pi = std::acos(-1.0);
      data[n] = static_cast<T>(0.125 * std::sin(2 * pi * n / 64.0) + 0.0625 * std::cos(2 * pi * n / 37.0));
    } 
      else data[n] = static_cast<T>(distribution(rng) / 16.0);
  }
  return data;
}

template <typename E>
void run_case(Context<E>& ctx, const std::filesystem::path& directory, const std::string& mode) {
  using Eval_T = typename E::Eval_T;
  using Engine = WbcicEngine<E>;
  using Fxpt_T = typename E::Fxpt_T;
  const auto samples = make_test_input<E>(mode);

  auto math_engine = Engine::create(ctx, "wbcic_math_" + mode);
  auto fixed_engine = Engine::create(ctx, "wbcic_fixed_" + mode);
  WidebandCicFirRef<E> fir;
  WidebandCicFirRef<E> fixed_fir;
  WidebandCicFirRef<E> mem_fir;
  WbcicQuantConfig<E> config;
  config.enabled = ctx.arg.fixedpoint_eval;
  if (config.enabled) fixed_engine->set_quant_config(config);

  const auto prefix = directory / ("wbcic_" + mode);
  std::unique_ptr<TraceFile<E>> trace;
  std::unique_ptr<TraceFile<E>> fixed_trace;
  if (ctx.arg.signal_trace) {
      const auto path = (mode == "impulse" && !ctx.arg.signal_trace_file.empty())
                        ? std::filesystem::path(ctx.arg.signal_trace_file) 
                        : std::filesystem::path(prefix.string() + ".trace");
    if (!path.parent_path().empty()) 
      std::filesystem::create_directories(path.parent_path());
    trace = std::make_unique<TraceFile<E>>(ctx);
    trace->open(path.string());
    trace->write_line("# M=" + std::to_string(E::m) + " K=" + std::to_string(E::k) +
                      " b=" + std::to_string(E::b) + " phase=0"
                      " H_fig3b(z)=-z^(-K/2)*H_p(z)");
    trace->write_line("# IO Q: width=" + std::to_string(E::fx_word_bits) +
                      " integer_bits=" + std::to_string(E::fx_integer_bits) +
                      "; .mem uses E::Fxpt_T (SC_TRN/SC_WRAP)");
    trace->write_line("# accepted-input sample trace; low-rate nodes valid only when valid=1");
    if (config.enabled) {
      fixed_trace = std::make_unique<TraceFile<E>>(ctx);
      fixed_trace->open(prefix.string() + "_fixed.trace");
      fixed_trace->write_line("# quantized engine diagnostic; numeric values may lose precision in trace");
    }
  }

  std::unique_ptr<OutputFile<E>> input_mem;
  std::unique_ptr<OutputFile<E>> golden_mem;
  if (ctx.arg.sv_trace) {
    const auto mem_dir = ctx.arg.sv_trace_dir.empty() ? directory / "sv" : std::filesystem::path(ctx.arg.sv_trace_dir);
    input_mem = open_export(ctx, mem_dir / ("wbcic_" + mode + "_input.mem"));
    golden_mem = open_export(ctx, mem_dir / ("wbcic_" + mode + "_golden.mem"));
  }

  std::size_t outputs = 0;
  long double max_error = 0;
  long double max_fixed_error = 0;
  std::size_t call_index = 0;
  for (std::size_t n = 0; n < samples.size(); ++n) {
    if (n % 23 == 7) {
      if (math_engine->step(Eval_T(0.25), false).output_valid || 
          fir.step(Eval_T(0.25), false).output_valid ||
          (config.enabled && fixed_engine->step(Eval_T(0.25), false).output_valid)) {
        throw std::runtime_error(mode + ": stall produced an output");
      }
      if (trace) trace->write_line("call=" + std::to_string(call_index) + " accepted=0");
      ++call_index;
    }
    const Eval_T sample = samples[n];
    typename WidebandCicFirRef<E>::StepResult mem_expected{};
    if (input_mem) {
      // Quantize once with the project's native IO Fxpt_T; share the exact sample
      // with the independent FIR and the emitted SV $readmemh testvector.
      const Fxpt_T q_input = sample;
      write_syscfx_hexword<E>(*input_mem, q_input);
      mem_expected = mem_fir.step(static_cast<Eval_T>(q_input.to_double()));
    }

    const auto actual = math_engine->step(sample);
    const auto expected = fir.step(sample);
    if (trace) {
      trace->write_line("call=" + std::to_string(call_index) + " accepted=1 " +
                        trace_line(math_engine->get_last_trace(), "math_engine"));
    }
    ++call_index;
    if (actual.output_valid != expected.output_valid ||
       (actual.output_valid && (actual.input_index != expected.input_index || actual.output_index != expected.output_index))) {
      throw std::runtime_error(mode + ": math_engine/FIR phase or output-index mismatch at n=" + std::to_string(n));
    }

    typename Engine::StepResult fx_result{};
    typename WidebandCicFirRef<E>::StepResult fx_expected{};
    if (config.enabled) {
      Eval_T q_input = sample;

      if (config.quantize_input) {
        const sc_dt::sc_fxval input_value(static_cast<double>(sample));

        const auto quantized = wbcic_detail::quantize_format(
          input_value,
          E::input_word_bits,
          E::input_integer_bits,
          config.rounding
        );

        q_input = static_cast<Eval_T>(quantized.to_double());
      }
      fx_expected = fixed_fir.step(q_input);
      fx_result = fixed_engine->step(sample);
      if (fixed_trace) {
        fixed_trace->write_line("call=" + std::to_string(call_index - 1) + " accepted=1 " +
                                trace_line(fixed_engine->get_last_trace(), "fixed"));
      }
      if (fx_result.output_valid != actual.output_valid || fx_result.output_valid != fx_expected.output_valid ||
          (fx_result.output_valid && fx_result.input_index != fx_expected.input_index)) {
        throw std::runtime_error(mode + ": fixed/FIR output-valid or phase mismatch");
      }
    }

    if (!actual.output_valid) continue;
    ++outputs;
    const long double error = std::fabs(static_cast<long double>(actual.output) - expected.output);
    max_error = std::max(max_error, error);
    if (!std::isfinite(static_cast<double>(error)) || error > 2e-6L) {
      throw std::runtime_error(mode + ": math_engine/FIR mismatch at n=" + std::to_string(n));
    }
    if (config.enabled) {
      const long double fixed_error = std::fabs(static_cast<long double>(fx_result.output) - fx_expected.output);
      max_fixed_error = std::max(max_fixed_error, fixed_error);
      if (!std::isfinite(static_cast<double>(fixed_error)) || fixed_error > ctx.arg.fixedpoint_tol + 1e-12) {
        throw std::runtime_error(mode + ": fixed/FIR error exceeds tolerance at n=" + std::to_string(n));
      }
    }
    if (golden_mem) {
      if (!mem_expected.output_valid || mem_expected.input_index != expected.input_index) {
        throw std::runtime_error(mode + ": MEM/FIR phase mismatch");
      }
      write_syscfx_hexword<E>(*golden_mem, mem_expected.output);
    }
    if (ctx.arg.trace_enabled && outputs <= 12) {
      Out(ctx) << "[WBCIC " << mode << "] n=" << n << " y=" << actual.output
               << " fir=" << expected.output << " err=" << static_cast<double>(error) << "\n";
    }
  }

  if (outputs != (samples.size() + E::m - 1) / E::m ||
      math_engine->get_input_count() != samples.size() || 
      math_engine->get_output_count() != outputs ||
      (config.enabled && fixed_engine->get_output_count() != outputs)) {
    throw std::runtime_error(mode + ": count mismatch");
  }
  if (trace) trace->close();
  if (fixed_trace) fixed_trace->close();
  if (input_mem) input_mem->close(ctx);
  if (golden_mem) golden_mem->close(ctx);
  Out(ctx) << "[WBCIC] PASS " << mode << " accepted=" << samples.size() << " outputs=" << outputs
           << " max_math_err=" << static_cast<double>(max_error)
           << (config.enabled ? " max_fixed_err=" + std::to_string(static_cast<double>(max_fixed_error)) : "")
           << "\n";
}

}  // namespace wbcic_detail

template <typename E>
bool WbcicEngine<E>::run_testbench(Context<E>& ctx) {
  try {
    // text_output selects the parent output directory for backward-compatible CLI.
    const std::filesystem::path output_dir = !ctx.arg.text_output.empty()
      ? std::filesystem::absolute(ctx.arg.text_output).parent_path()
      : std::filesystem::path("wbcic_testvectors");
    std::filesystem::create_directories(output_dir);
    if (ctx.arg.fixedpoint_tol < 0 || !std::isfinite(ctx.arg.fixedpoint_tol)) {
      throw std::invalid_argument("--fixedpoint-tol must be finite and nonnegative");
    }
    for (const std::string mode : {"impulse", "constant", "multitone", "random"}) {
      wbcic_detail::run_case<E>(ctx, output_dir, mode);
    }
    Out(ctx) << "[WBCIC] PASS all engine cases; .trace=" << ctx.arg.signal_trace
             << " .mem=" << ctx.arg.sv_trace << "\n";
    return true;
  } catch (const std::exception& error) {
    Out(ctx) << "[WBCIC] FAIL: " << error.what() << "\n";
    return false;
  }
}

#ifdef ADPT_ENABLE_WBCIC_ENGINE
using EngineArch = ADPT_TARGET;
template bool WbcicEngine<EngineArch>::run_testbench(Context<EngineArch>&);
template std::unique_ptr<WbcicEngine<EngineArch>>
WbcicEngine<EngineArch>::create(Context<EngineArch>&, std::string_view);
#endif

}  // namespace adptsysc