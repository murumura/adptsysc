#include <adptsysc/config.hh>

#include <tlm>
#include <tlm_utils/simple_initiator_socket.h>
#include <tlm_utils/simple_target_socket.h>

#include <adptsysc/adptsysc.hh>
#include <adptsysc/ovsfdaf-tlm.hh>

#include <algorithm>
#include <cmath>
#include <complex>
#include <limits>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <type_traits>
#include <utility>
#include <vector>

namespace adptsysc {

using E = ADPT_TARGET;

template <typename E>
std::unique_ptr<OverlapSaveFdafTLM<E>>
OverlapSaveFdafTLM<E>::create(Context<E>& ctx,
                              sc_core::sc_module_name name,
                              std::size_t filter_ncoeff,
                              const T* filter_initptr) {
  return std::unique_ptr<OverlapSaveFdafTLM<E>>(
      new OverlapSaveFdafTLM<E>(
          name, ctx, filter_ncoeff, filter_initptr));
}

template <typename E>
OverlapSaveFdafTLM<E>::OverlapSaveFdafTLM(
    sc_core::sc_module_name name,
    Context<E>& ctx,
    std::size_t ncoeff,
    const T* filter_initptr)
    : sc_core::sc_module(name),
      filter_ncoeff(ncoeff),
      fft_size(2 * ncoeff),
      block_size(ncoeff),
      initial_weights(ncoeff, T(0)) {
  static_assert(std::is_floating_point_v<T>,
                "OverlapSaveFdafTLM requires floating-point Eval_T");

  if (filter_initptr != nullptr) {
    std::copy(filter_initptr,
              filter_initptr + filter_ncoeff,
              initial_weights.begin());
  }

  if constexpr (requires { E::fdaf_block_latency; }) {
    block_delay = sc_core::sc_time(E::fdaf_block_latency,
                                   sc_core::SC_NS);
  }
  if constexpr (requires { E::fdaf_fft_latency; }) {
    fft_delay = sc_core::sc_time(E::fdaf_fft_latency,
                                 sc_core::SC_NS);
  }
  if constexpr (requires { E::fdaf_ifft_latency; }) {
    ifft_delay = sc_core::sc_time(E::fdaf_ifft_latency,
                                  sc_core::SC_NS);
  }

  targ_socket.register_b_transport(this,
                                   &OverlapSaveFdafTLM::b_transport);
  targ_socket.register_get_direct_mem_ptr(
      this, &OverlapSaveFdafTLM::get_direct_mem_ptr);
  targ_socket.register_transport_dbg(this,
                                     &OverlapSaveFdafTLM::transport_dbg);

  allocate_state(ctx);
}

template <typename E>
void OverlapSaveFdafTLM<E>::validate_config() const {
  if (filter_ncoeff == 0 || fft_size != 2 * filter_ncoeff ||
      block_size != filter_ncoeff) {
    throw std::invalid_argument(
        "FDAF requires M>0, N=2M and block_size=M");
  }

  if (!(mu >= T(0))) {
    throw std::invalid_argument("FDAF mu must be non-negative");
  }
  if (!(alpha >= T(0) && alpha < T(1))) {
    throw std::invalid_argument("FDAF alpha must satisfy 0 <= alpha < 1");
  }
  if (!(eps > T(0))) {
    throw std::invalid_argument("FDAF eps must be positive");
  }
}

template <typename E>
void OverlapSaveFdafTLM<E>::forward_fft(
    const std::vector<CxT>& in,
    std::vector<CxT>& out) const {
  if (!fft_engine) {
    throw std::runtime_error("FDAF FFT engine is not initialized");
  }
  if (in.size() != fft_size) {
    throw std::invalid_argument("FDAF forward FFT input size mismatch");
  }

  fft_engine->get_cmplxfft(in, out);

  if (out.size() != fft_size) {
    throw std::runtime_error("FDAF forward FFT output size mismatch");
  }
}

template <typename E>
void OverlapSaveFdafTLM<E>::inverse_fft(
    const std::vector<CxT>& in,
    std::vector<CxT>& out) const {
  if (!fft_engine) {
    throw std::runtime_error("FDAF FFT engine is not initialized");
  }
  if (in.size() != fft_size) {
    throw std::invalid_argument("FDAF inverse FFT input size mismatch");
  }

  fft_engine->get_cmplxifft(in, out);

  if (out.size() != fft_size) {
    throw std::runtime_error("FDAF inverse FFT output size mismatch");
  }
}

template <typename E>
void OverlapSaveFdafTLM<E>::reset_filter_state() {
  std::fill(x_hist.begin(), x_hist.end(), T(0));
  std::fill(pow_est.begin(), pow_est.end(), eps);

  std::vector<CxT> w_time(fft_size, CxT(0, 0));
  for (std::size_t i = 0; i < filter_ncoeff; ++i) {
    w_time[i] = CxT(initial_weights[i], T(0));
  }
  forward_fft(w_time, w_freq);

  sample_count = 0;
  block_count = 0;
}

template <typename E>
void OverlapSaveFdafTLM<E>::allocate_state(Context<E>&) {
  validate_config();

  w_freq.assign(fft_size, CxT(0, 0));
  pow_est.assign(fft_size, eps);

  x_block.assign(fft_size, CxT(0, 0));
  x_freq.assign(fft_size, CxT(0, 0));
  y_freq.assign(fft_size, CxT(0, 0));
  y_ifft.assign(fft_size, CxT(0, 0));

  d_block.assign(block_size, T(0));
  y_block.assign(block_size, T(0));
  e_block.assign(block_size, T(0));

  e_pad.assign(fft_size, CxT(0, 0));
  e_freq.assign(fft_size, CxT(0, 0));
  grad_freq.assign(fft_size, CxT(0, 0));
  grad_time.assign(fft_size, CxT(0, 0));

  x_hist.assign(block_size, T(0));

  using FFTMode = typename FFTT::FFTMode;
  fft_engine = std::make_unique<FFTT>(FFTMode::Complex, fft_size);

  reset_filter_state();
  is_init = true;
}

template <typename E>
void OverlapSaveFdafTLM<E>::state_reset(Context<E>& ctx) {
  allocate_state(ctx);
}

template <typename E>
void OverlapSaveFdafTLM<E>::constrain_weights() {
  inverse_fft(w_freq, grad_time);

  for (std::size_t i = filter_ncoeff; i < fft_size; ++i) {
    grad_time[i] = CxT(0, 0);
  }

  forward_fft(grad_time, w_freq);
}

template <typename E>
void OverlapSaveFdafTLM<E>::process_block(
  const std::vector<T>& x_in, const std::vector<T>& d_in,
  std::vector<T>& y_out, std::vector<T>& e_out, bool train) {
  if (!is_init) {
    throw std::runtime_error("OverlapSaveFdafTLM is not initialized");
  }
  if (x_in.size() != block_size || d_in.size() != block_size) {
    throw std::invalid_argument(
        "FDAF input and desired blocks must both have M samples");
  }

  for (std::size_t i = 0; i < block_size; ++i) {
    x_block[i] = CxT(x_hist[i], T(0));
    x_block[block_size + i] = CxT(x_in[i], T(0));
    d_block[i] = d_in[i];
  }

  forward_fft(x_block, x_freq);

  for (std::size_t k = 0; k < fft_size; ++k) {
    y_freq[k] = w_freq[k] * x_freq[k];
  }

  inverse_fft(y_freq, y_ifft);

  y_out.resize(block_size);
  e_out.resize(block_size);
  for (std::size_t i = 0; i < block_size; ++i) {
    y_block[i] = y_ifft[block_size + i].real();
    e_block[i] = d_block[i] - y_block[i];
    y_out[i] = y_block[i];
    e_out[i] = e_block[i];
  }

  if (train) {
    std::fill(e_pad.begin(), e_pad.end(), CxT(0, 0));
    for (std::size_t i = 0; i < block_size; ++i) {
      e_pad[block_size + i] = CxT(e_block[i], T(0));
    }

    forward_fft(e_pad, e_freq);

    for (std::size_t k = 0; k < fft_size; ++k) {
      const T xpow = static_cast<T>(std::norm(x_freq[k]));
      pow_est[k] = alpha * pow_est[k] + (T(1) - alpha) * xpow;

      CxT grad = std::conj(x_freq[k]) * e_freq[k];
      if (use_power_norm) {
        grad /= static_cast<T>(pow_est[k] + eps);
      }
      grad_freq[k] = grad;
      w_freq[k] += mu * grad;
    }

    constrain_weights();
  }

  x_hist = x_in;
  sample_count += block_size;
  ++block_count;
}

template <typename E>
std::vector<typename E::Eval_T>
OverlapSaveFdafTLM<E>::get_time_weights() const {
  if (!is_init) {
    return {};
  }

  std::vector<CxT> w_time;
  inverse_fft(w_freq, w_time);

  std::vector<T> out(filter_ncoeff, T(0));
  for (std::size_t i = 0; i < filter_ncoeff; ++i) {
    out[i] = w_time[i].real();
  }
  return out;
}

template <typename E>
void OverlapSaveFdafTLM<E>::update_hyperparams(const json& params) {
  if (params.contains("mu")) {
    mu = params.at("mu").template get<T>();
  }
  if (params.contains("alpha")) {
    alpha = params.at("alpha").template get<T>();
  }
  if (params.contains("eps")) {
    eps = params.at("eps").template get<T>();
  }
  if (params.contains("use_power_norm")) {
    use_power_norm =
        params.at("use_power_norm").template get<bool>();
  }
  validate_config();
}

template <typename E>
json OverlapSaveFdafTLM<E>::get_hyperparams() const {
  return {
    {"otype", "overlap_save_fdaf_tlm"},
    {"filter_ncoeff", filter_ncoeff},
    {"fft_size", fft_size},
    {"block_size", block_size},
    {"mu", mu},
    {"alpha", alpha},
    {"eps", eps},
    {"use_power_norm", use_power_norm},
  };
}

template <typename E>
json OverlapSaveFdafTLM<E>::serialize(Context<E>&) const {
  return {
    {"otype", "overlap_save_fdaf_tlm"},
    {"filter_ncoeff", filter_ncoeff},
    {"fft_size", fft_size},
    {"block_size", block_size},
    {"mu", mu},
    {"alpha", alpha},
    {"eps", eps},
    {"use_power_norm", use_power_norm},
    {"initial_weights", realvec_to_json(initial_weights)},
    {"w_freq", cvec_to_local_json(w_freq)},
    {"pow_est", realvec_to_json(pow_est)},
    {"x_hist", realvec_to_json(x_hist)},
    {"sample_count", sample_count},
    {"block_count", block_count},
    {"is_init", is_init},
  };
}

template <typename E>
void OverlapSaveFdafTLM<E>::deserialize(Context<E>& ctx,
                                        const json& data) {
  if (!data.is_object()) {
    throw std::invalid_argument("FDAF deserialize expects an object");
  }

  filter_ncoeff =
      data.at("filter_ncoeff").template get<std::size_t>();
  fft_size = 2 * filter_ncoeff;
  block_size = filter_ncoeff;

  if (data.contains("mu")) {
    mu = data.at("mu").template get<T>();
  }
  if (data.contains("alpha")) {
    alpha = data.at("alpha").template get<T>();
  }
  if (data.contains("eps")) {
    eps = data.at("eps").template get<T>();
  }
  if (data.contains("use_power_norm")) {
    use_power_norm =
        data.at("use_power_norm").template get<bool>();
  }
  if (data.contains("initial_weights")) {
    initial_weights =
        realvec_from_json<T>(data.at("initial_weights"));
  } else {
    initial_weights.assign(filter_ncoeff, T(0));
  }

  if (initial_weights.size() != filter_ncoeff) {
    throw std::runtime_error("FDAF initial weight size mismatch");
  }

  allocate_state(ctx);

  if (data.contains("w_freq")) {
    w_freq = cvec_from_local_json<T>(data.at("w_freq"));
  }
  if (data.contains("pow_est")) {
    pow_est = realvec_from_json<T>(data.at("pow_est"));
  }
  if (data.contains("x_hist")) {
    x_hist = realvec_from_json<T>(data.at("x_hist"));
  }
  if (data.contains("sample_count")) {
    sample_count = data.at("sample_count").template get<std::size_t>();
  }
  if (data.contains("block_count")) {
    block_count = data.at("block_count").template get<std::size_t>();
  }

  if (w_freq.size() != fft_size || pow_est.size() != fft_size || x_hist.size() != block_size) {
    throw std::runtime_error("FDAF serialized state size mismatch");
  }

  is_init = true;
}

template <typename E>
void OverlapSaveFdafTLM<E>::dump_state(
    Context<E>& ctx,
    const std::string& desc) const {
  if (!ctx.arg.verbose) {
    return;
  }

  Out(ctx) << "FDAF state"
           << (desc.empty() ? "" : " (" + desc + ")")
           << ": M=" << filter_ncoeff
           << " N=" << fft_size
           << " blocks=" << block_count
           << " samples=" << sample_count
           << " mu=" << mu
           << " alpha=" << alpha
           << " eps=" << eps
           << "\n";
}

template <typename E>
void OverlapSaveFdafTLM<E>::b_transport(
    tlm::tlm_generic_payload& trans,
    sc_core::sc_time& delay) {
  using Payload = Txn;

  trans.set_dmi_allowed(false);

  if (!is_init) {
    trans.set_response_status(tlm::TLM_GENERIC_ERROR_RESPONSE);
    return;
  }
  if (trans.get_command() != tlm::TLM_READ_COMMAND &&
      trans.get_command() != tlm::TLM_WRITE_COMMAND) {
    trans.set_response_status(tlm::TLM_COMMAND_ERROR_RESPONSE);
    return;
  }
  if (trans.get_address() != 0) {
    trans.set_response_status(tlm::TLM_ADDRESS_ERROR_RESPONSE);
    return;
  }
  if (trans.get_byte_enable_ptr() != nullptr) {
    trans.set_response_status(tlm::TLM_BYTE_ENABLE_ERROR_RESPONSE);
    return;
  }
  if (trans.get_data_ptr() == nullptr ||
      trans.get_data_length() != sizeof(Payload) ||
      trans.get_streaming_width() < sizeof(Payload)) {
    trans.set_response_status(tlm::TLM_BURST_ERROR_RESPONSE);
    return;
  }

  auto* txn = reinterpret_cast<Payload*>(trans.get_data_ptr());
  txn->ok = false;
  txn->error.clear();

  try {
    if (trans.get_command() == tlm::TLM_READ_COMMAND ||
        txn->op == Payload::Op::GET_WEIGHTS) {
      txn->weights = get_time_weights();
      txn->ok = true;
      trans.set_response_status(tlm::TLM_OK_RESPONSE);
      return;
    }

    if (txn->op == Payload::Op::RESET) {
      // Restore initial coefficients, power estimate, overlap and counters.
      reset_filter_state();
      txn->weights = get_time_weights();
      txn->ok = true;
      trans.set_response_status(tlm::TLM_OK_RESPONSE);
      return;
    }

    if (txn->op != Payload::Op::PROCESS) {
      throw std::runtime_error("Unknown FDAF transaction opcode");
    }

    process_block(txn->x_in, txn->d_in,
                  txn->y_out, txn->e_out, txn->train);
    txn->weights = get_time_weights();
    txn->ok = true;

    delay += block_delay + fft_delay + ifft_delay;
    if (txn->train) {
      delay += fft_delay + fft_delay + ifft_delay;
    }

    trans.set_response_status(tlm::TLM_OK_RESPONSE);
  } catch (const std::exception& e) {
    txn->error = e.what();
    txn->ok = false;
    trans.set_response_status(tlm::TLM_GENERIC_ERROR_RESPONSE);
  }
}

template <typename E>
bool OverlapSaveFdafTLM<E>::get_direct_mem_ptr(
    tlm::tlm_generic_payload&,
    tlm::tlm_dmi& dmi_data) {
  dmi_data.set_start_address(0);
  dmi_data.set_end_address(0);
  dmi_data.set_dmi_ptr(nullptr);
  return false;
}

template <typename E>
unsigned int OverlapSaveFdafTLM<E>::transport_dbg(
    tlm::tlm_generic_payload& trans) {
  using Payload = Txn;

  if (trans.get_command() != tlm::TLM_READ_COMMAND) {
    trans.set_response_status(tlm::TLM_COMMAND_ERROR_RESPONSE);
    return 0;
  }
  if (trans.get_address() != 0) {
    trans.set_response_status(tlm::TLM_ADDRESS_ERROR_RESPONSE);
    return 0;
  }
  if (trans.get_data_ptr() == nullptr ||
      trans.get_data_length() != sizeof(Payload)) {
    trans.set_response_status(tlm::TLM_BURST_ERROR_RESPONSE);
    return 0;
  }

  auto* txn = reinterpret_cast<Payload*>(trans.get_data_ptr());
  txn->weights = get_time_weights();
  txn->ok = true;
  txn->error.clear();

  trans.set_dmi_allowed(false);
  trans.set_response_status(tlm::TLM_OK_RESPONSE);
  return static_cast<unsigned int>(sizeof(Payload));
}

template <typename E>
class FdafTLMInitiator : public sc_core::sc_module {
public:
  using T = typename E::Eval_T;
  using Dut = OverlapSaveFdafTLM<E>;
  using Txn = typename Dut::Txn;

  tlm_utils::simple_initiator_socket<FdafTLMInitiator> init_socket{"init_socket"};

  Context<E>& ctx;
  std::size_t block_size;
  bool pass = true;

  FdafTLMInitiator(sc_core::sc_module_name name,
                   Context<E>& ctx,
                   std::size_t block_size)
      : sc_core::sc_module(name),
        ctx(ctx),
        block_size(block_size) {
    SC_THREAD(run);
  }

  void run() {
    Txn txn;
    txn.op = Txn::Op::PROCESS;
    txn.train = false;
    txn.x_in.resize(block_size);
    txn.d_in.resize(block_size);

    for (std::size_t i = 0; i < block_size; ++i) {
      txn.x_in[i] = static_cast<T>(i + 1) / static_cast<T>(16);
      txn.d_in[i] = txn.x_in[i];
    }

    tlm::tlm_generic_payload tr;
    sc_core::sc_time delay = sc_core::SC_ZERO_TIME;
    tr.set_command(tlm::TLM_WRITE_COMMAND);
    tr.set_address(0);
    tr.set_data_ptr(reinterpret_cast<unsigned char*>(&txn));
    tr.set_data_length(sizeof(Txn));
    tr.set_streaming_width(sizeof(Txn));
    tr.set_byte_enable_ptr(nullptr);
    tr.set_dmi_allowed(false);
    tr.set_response_status(tlm::TLM_INCOMPLETE_RESPONSE);

    init_socket->b_transport(tr, delay);
    wait(delay);

    pass = !tr.is_response_error() && txn.ok &&
           txn.y_out.size() == block_size;

    const T tol = static_cast<T>(1e-4);
    if (pass) {
      for (std::size_t i = 0; i < block_size; ++i) {
        if (std::abs(txn.y_out[i] - txn.x_in[i]) > tol) {
          pass = false;
          break;
        }
      }
    }

    if (ctx.arg.verbose) {
      Out(ctx) << sc_core::sc_time_stamp()
               << " FDAF TLM testbench done, pass="
               << (pass ? "true" : "false") << "\n";
    }

    sc_core::sc_stop();
  }
};

template <typename E>
bool OverlapSaveFdafTLM<E>::run_testbench(Context<E>& ctx) {
  constexpr std::size_t ncoeff = [] {
    if constexpr (requires { E::fdaf_filter_ncoeff; }) {
      return static_cast<std::size_t>(E::fdaf_filter_ncoeff);
    } else {
      return std::size_t{16};
    }
  }();
  std::vector<T> h(ncoeff, T(0));
  h[0] = T(1);

  auto dut = OverlapSaveFdafTLM<E>::create(ctx, "fdaf_dut", ncoeff, h.data());

  FdafTLMInitiator<E> tb("fdaf_tlm_tb", ctx, ncoeff);
  tb.init_socket.bind(dut->targ_socket);

  sc_core::sc_start();
  dut->dump_state(ctx, "end-of-testbench");
  return tb.pass;
}

template class OverlapSaveFdafTLM<E>;

}  // namespace adptsysc
