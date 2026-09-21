#ifdef ADPT_ENABLE_R2SDF
#include <adptsysc/config.hh>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <sstream>
#include <tlm>
#include <tlm_utils/simple_target_socket.h>
#include <tlm_utils/simple_initiator_socket.h>
#include <adptsysc/sysc-r2sdffft-tlm.hh>
#include <adptsysc/syscfx-utils.hh>

namespace adptsysc {

using E = ADPT_TARGET;

std::size_t clog2int (const std::size_t n) {
  std::size_t w = 0;
  std::size_t v = (n > 1) ? (n - 1) : 0;
  while (v > 0) {
    v >>= 1;
    ++w;
  }
  return w;
}

// =============================
// R2SdfCtrlTLM
// =============================
template <typename E>
std::shared_ptr<R2SdfCtrlTLM<E>>
R2SdfCtrlTLM<E>::create(Context<E>& ctx,
                        sc_core::sc_module_name name,
                        FFTFlowMode flow_mode, std::size_t fftsz) {
  return std::shared_ptr<R2SdfCtrlTLM<E>>(new R2SdfCtrlTLM<E>(name, flow_mode, fftsz));
}

template <typename E>
R2SdfCtrlTLM<E>::R2SdfCtrlTLM(sc_core::sc_module_name name,
                              FFTFlowMode fm,
                              std::size_t fftsz)
    : sc_core::sc_module(name),
      flow_mode(fm),
      fft_size(fftsz) {
  cnt_width = clog2int(this->fft_size);
  nstages   = cnt_width;
  ntwdls    = (nstages > 0) ? (nstages - 1) : 0;

  sigs_cur.s.resize(nstages, false);
  sigs_cur.twdlrom_en.resize(ntwdls, false);
  sigs_cur.tw_addr_local.resize(ntwdls, 0);
  sigs_cur.tw_addr_global.resize(ntwdls, 0);

  reset();
}

template <typename E>
void R2SdfCtrlTLM<E>::reset() {
  cnt_cur = 0;
  sigs_cur = compute_outputs(cnt_cur, false);
}

template <typename E>
int R2SdfCtrlTLM<E>::stage_to_tw_slot(unsigned p) const {
  if (flow_mode == FFTFlowMode::DIF) {
    if (p >= nstages - 1) return -1;
    return static_cast<int>(p);
  } else {
    if (p == 0) return -1;
    return static_cast<int>(p - 1);
  }
}

template <typename E>
const typename R2SdfCtrlTLM<E>::R2SdfCtrlSigs&
R2SdfCtrlTLM<E>::step(bool sample_fire) {
  sigs_cur = compute_outputs(cnt_cur, sample_fire);
  if (sample_fire) {
    CountT cnt_next = cnt_cur;
    cnt_next = cnt_next + CountT{1};
    cnt_cur = cnt_next;
  }
  return sigs_cur;
}

template <typename E>
typename R2SdfCtrlTLM<E>::R2SdfCtrlSigs
R2SdfCtrlTLM<E>::decode_from_count(CountT c) const {
  return compute_outputs(c, true);
}

template <typename E>
typename R2SdfCtrlTLM<E>::R2SdfCtrlSigs
R2SdfCtrlTLM<E>::compute_outputs(CountT c, bool sample_fire) const {
  R2SdfCtrlSigs o;
  o.cnt = c;
  o.s.resize(nstages, false);
  o.twdlrom_en.resize(ntwdls, false);
  o.tw_addr_local.resize(ntwdls, 0);
  o.tw_addr_global.resize(ntwdls, 0);

  if (!sample_fire) {
    return o;
  }

  for (unsigned p = 0; p < nstages; ++p) {
    const unsigned bit_idx = (flow_mode == FFTFlowMode::DIF) ? (nstages - 1 - p) : p;
    o.s[p] = static_cast<bool>((c >> bit_idx) & 0x1);
  }

  for (unsigned p = 0; p < nstages; ++p) {
    const int q = stage_to_tw_slot(p);
    if (q < 0) continue;

    o.twdlrom_en[static_cast<std::size_t>(q)] = o.s[p];

    unsigned valid_w = 0;
    unsigned shift   = 0;

    if (flow_mode == FFTFlowMode::DIF) {
      valid_w = nstages - 1 - p;
      shift   = p;
    } else {
      valid_w = p;
      shift   = nstages - 1 - p;
    }

    std::size_t local_addr = 0;
    for (unsigned b = 0; b < valid_w; ++b) {
      local_addr |= (((c >> b) & 0x1) << b);
    }

    o.tw_addr_local[static_cast<std::size_t>(q)]  = local_addr;
    o.tw_addr_global[static_cast<std::size_t>(q)] = (local_addr << shift);
  }

  o.frame_first = sample_fire && (c == 0);
  o.frame_last  = sample_fire && (c == fft_size - 1);
  return o;
}

template <typename E>
void R2SdfCtrlTLM<E>::update_hyperparams(const json& params) {
  (void)params;
}

template <typename E>
json R2SdfCtrlTLM<E>::get_hyperparams() const {
  return {
    {"otype", "r2sdf_ctrl_tlm"},
    {"fft_size", fft_size},
    {"flow_mode", flow_mode == FFTFlowMode::DIT ? "dit" : "dif"},
    {"nstages", nstages},
    {"ntwiddle_stages", ntwdls}
  };
}

// -----------------------------------------------------------------------------
// R2SdfStageTLM
// -----------------------------------------------------------------------------
template <typename E>
std::unique_ptr<R2SdfStageTLM<E>>
R2SdfStageTLM<E>::create(Context<E>& ctx,
                         sc_core::sc_module_name name,
                         std::size_t fftsz,
                         std::size_t std_idx,
                         FFTFlowMode fm,
                         SyscMemory<E>* twdlmem,
                         std::shared_ptr<TraceFile<E>> tf,
                         std::shared_ptr<R2SdfCtrlTLM<E>> ctrl,
                         bool use_ctrl) {
  return std::unique_ptr<R2SdfStageTLM<E>>(
    new R2SdfStageTLM<E>(ctx, name, fftsz, std_idx, fm, 
                         twdlmem, tf, ctrl, use_ctrl));
}

template <typename E>
R2SdfStageTLM<E>::R2SdfStageTLM(Context<E>&,
                                sc_core::sc_module_name name,
                                std::size_t fftsz,
                                std::size_t std_idx,
                                FFTFlowMode fm,
                                SyscMemory<E>* twdlmem,
                                std::shared_ptr<TraceFile<E>> tf,
                                std::shared_ptr<R2SdfCtrlTLM<E>> ctrl,
                                bool use_ctrl)
    : sc_core::sc_module(name),
      fft_size(fftsz),
      stage_idx(std_idx),
      flow_mode(fm),
      twiddle_mem(twdlmem),
      tracefile(tf),
      ctrl(ctrl),
      use_ctrl(use_ctrl) {}

template <typename E>
void R2SdfStageTLM<E>::set_tracefile(std::shared_ptr<TraceFile<E>> tf) {
  tracefile = tf;
}

template <typename E>
void R2SdfStageTLM<E>::set_ctrl(std::shared_ptr<R2SdfCtrlTLM<E>> c) {
  ctrl = c;
}

template <typename E>
void R2SdfStageTLM<E>::allocate_state(Context<E>& ctx) {
  // Stage submodules are functional helpers. R2SdfFFTTLM::b_transport()
  // owns the single frame-level timing annotation, so child calls are zero-time.
  shiftreg = std::make_unique<ComplexShiftRegisterTLM<T>>(
      sc_core::sc_gen_unique_name("shiftreg"),
      get_delay_len(),
      sc_core::SC_ZERO_TIME);

  cmul = std::make_unique<ComplexMultiplierTLM<T>>(
      sc_core::sc_gen_unique_name("cmul"),
      sc_core::SC_ZERO_TIME);
      
  cmul_init_socket.bind(cmul->targ_socket);
  shiftreg_init_socket.bind(shiftreg->targ_socket);
  reset_state();
  is_init = true;
}

template <typename E>
void R2SdfStageTLM<E>::reset_state() {
  if (shiftreg && is_init) {
    shiftreg_clear_tlm();
  } else if (shiftreg) {
    shiftreg->clear();
  }
}

template <typename E>
auto R2SdfStageTLM<E>::to_plain(const CxT& z) -> ComplexPlain<T> {
  return ComplexPlain<T>{z.real(), z.imag()};
}

template <typename E>
std::size_t R2SdfStageTLM<E>::get_nstages() const {
  std::size_t n = fft_size;
  std::size_t s = 0;
  while (n > 1) {
    n >>= 1;
    ++s;
  }
  return s;
}

template <typename E>
std::size_t R2SdfStageTLM<E>::get_span() const {
  if (flow_mode == FFTFlowMode::DIT) {
    return std::size_t(1) << (stage_idx + 1);
  }
  return std::size_t(1) << (get_nstages() - stage_idx);
}

template <typename E>
std::size_t R2SdfStageTLM<E>::get_delay_len() const {
  return get_span() >> 1;
}

template <typename E>
std::size_t R2SdfStageTLM<E>::get_twiddle_index_dit(std::size_t local_idx) const {
  const std::size_t group  = std::size_t(1) << (stage_idx + 1);
  const std::size_t stride = fft_size / group;
  return (local_idx * stride) & (fft_size - 1);
}

template <typename E>
std::size_t R2SdfStageTLM<E>::get_twiddle_index_dif(std::size_t local_idx) const {
  const std::size_t group  = std::size_t(1) << (get_nstages() - stage_idx);
  const std::size_t stride = fft_size / group;
  return (local_idx * stride) & (fft_size - 1);
}

template <typename E>
typename R2SdfStageTLM<E>::CxT
R2SdfStageTLM<E>::get_twiddle(std::size_t k, bool inverse) const {
  if (twiddle_mem == nullptr) {
    throw std::runtime_error("R2SdfStageTLM twiddle memory is null");
  }
  const std::size_t addr = 2 * k;
  const T re = twiddle_mem->data()[addr];
  const T im = twiddle_mem->data()[addr + 1];
  const CxT w(re, im);
  return inverse ? std::conj(w) : w;
}

template <typename E>
int R2SdfStageTLM<E>::get_ctrl_tw_slot() const {
  const std::size_t nstages = get_nstages();

  if (flow_mode == FFTFlowMode::DIT) {
    return (stage_idx == 0) ? -1 : static_cast<int>(stage_idx - 1);
  } else {
    return (stage_idx + 1 == nstages) ? -1 : static_cast<int>(stage_idx);
  }
}

template <typename E>
ComplexPlain<typename E::Eval_T>
R2SdfStageTLM<E>::shiftreg_step_tlm(const ComplexPlain<T>& in) {
  ShiftRegTLMTrans<T> txn{};
  txn.in = in;

  tlm::tlm_generic_payload trans;
  sc_core::sc_time delay = sc_core::SC_ZERO_TIME;

  trans.set_command(tlm::TLM_WRITE_COMMAND);
  trans.set_address(0);
  trans.set_data_ptr(reinterpret_cast<unsigned char*>(&txn));
  trans.set_data_length(sizeof(txn));
  trans.set_streaming_width(sizeof(txn));
  trans.set_byte_enable_ptr(nullptr);
  trans.set_dmi_allowed(false);
  trans.set_response_status(tlm::TLM_INCOMPLETE_RESPONSE);

  shiftreg_init_socket->b_transport(trans, delay);

  if (trans.is_response_error()) {
    throw std::runtime_error("shiftreg TLM step failed");
  }
  return txn.out;
}

template <typename E>
void R2SdfStageTLM<E>::shiftreg_clear_tlm() {
  ShiftRegTLMTrans<T> txn{};
  txn.op = ShiftRegTLMTrans<T>::Op::CLEAR;

  tlm::tlm_generic_payload trans;
  sc_core::sc_time delay = sc_core::SC_ZERO_TIME;

  trans.set_command(tlm::TLM_WRITE_COMMAND);
  trans.set_address(0);
  trans.set_data_ptr(reinterpret_cast<unsigned char*>(&txn));
  trans.set_data_length(sizeof(txn));
  trans.set_streaming_width(sizeof(txn));
  trans.set_byte_enable_ptr(nullptr);
  trans.set_dmi_allowed(false);
  trans.set_response_status(tlm::TLM_INCOMPLETE_RESPONSE);

  shiftreg_init_socket->b_transport(trans, delay);

  if (trans.is_response_error()) {
    throw std::runtime_error("R2SdfStageTLM: shift register TLM clear failed");
  }
}

template <typename E>
ComplexPlain<typename E::Eval_T>
R2SdfStageTLM<E>::cmul_mul_tlm(const ComplexPlain<T>& a,
                               const ComplexPlain<T>& b) {
  ComplexMulTLMTrans<T> txn{};
  txn.a = a;
  txn.b = b;

  tlm::tlm_generic_payload trans;
  sc_core::sc_time delay = sc_core::SC_ZERO_TIME;

  trans.set_command(tlm::TLM_WRITE_COMMAND);
  trans.set_address(0);
  trans.set_data_ptr(reinterpret_cast<unsigned char*>(&txn));
  trans.set_data_length(sizeof(txn));
  trans.set_streaming_width(sizeof(txn));
  trans.set_byte_enable_ptr(nullptr);
  trans.set_dmi_allowed(false);
  trans.set_response_status(tlm::TLM_INCOMPLETE_RESPONSE);

  cmul_init_socket->b_transport(trans, delay);

  if (trans.is_response_error()) {
    throw std::runtime_error("cmul TLM mul failed");
  }
  return txn.y;
}

template <typename E>
void R2SdfStageTLM<E>::process_block(const std::vector<CxT>& in,
                                     std::vector<CxT>& out,
                                     bool inverse) {
  if (!is_init) {
    throw std::runtime_error("R2SdfStageTLM not initialized");
  }
  if (in.size() != fft_size) {
    throw std::invalid_argument("R2SdfStageTLM input size mismatch");
  }

  auto trace_line = [&](const std::string& s) {
    if (tracefile && tracefile->enabled()) {
      tracefile->write_line(s);
    }
  };

  out.assign(fft_size, CxT(0, 0));

  const std::size_t span = get_span();
  const std::size_t half = span >> 1;

  const bool ctrl_mode = use_ctrl && static_cast<bool>(ctrl);

  using CtrlT = R2SdfCtrlTLM<E>;
  const int tw_slot = get_ctrl_tw_slot();
  const bool stage_has_nontrivial_tw = (tw_slot >= 0);

  {
    std::ostringstream oss;
    oss << "[STAGE " << stage_idx << "] begin"
        << " fft_size=" << fft_size
        << " span=" << span
        << " half=" << half
        << " flow_mode=" << (flow_mode == FFTFlowMode::DIT ? "DIT" : "DIF")
        << " use_ctrl=" << (use_ctrl ? 1 : 0)
        << " ctrl_mode=" << (ctrl_mode ? 1 : 0)
        << " tw_slot=" << tw_slot
        << " inverse=" << (inverse ? 1 : 0);
    trace_line(oss.str());
  }

  for (std::size_t base = 0; base < fft_size; base += span) {
    {
      std::ostringstream oss;
      oss << "[STAGE " << stage_idx << "] base=" << base
          << " clear shiftreg";
      trace_line(oss.str());
    }

    shiftreg_clear_tlm();

    // preload first half into delay line through TLM
    for (std::size_t i = 0; i < half; ++i) {
      const ComplexPlain<T> pin = to_plain(in[base + i]);
      (void)shiftreg_step_tlm(pin);

      std::ostringstream oss;
      oss << "[STAGE " << stage_idx << "] preload"
          << " base=" << base
          << " i=" << i
          << " in=" << plaincmplx_to_string(pin);
      trace_line(oss.str());
    }

    // process second half
    for (std::size_t i = 0; i < half; ++i) {
      const ComplexPlain<T> delayed_plain = shiftreg_step_tlm(to_plain(in[base + half + i]));

      const CxT a(delayed_plain.re, delayed_plain.im);
      const CxT b = in[base + half + i];

      // fallback decode
      bool use_twdl_fallback = false;
      std::size_t twdl_idx_fallback = 0;

      if (flow_mode == FFTFlowMode::DIT) {
        if (stage_idx > 0) {
          use_twdl_fallback = true;
          twdl_idx_fallback = get_twiddle_index_dit(i);
        }
      } else {
        if (stage_idx + 1 < get_nstages()) {
          use_twdl_fallback = true;
          twdl_idx_fallback = get_twiddle_index_dif(i);
        }
      }

      // ctrl decode
      bool use_twdl_ctrl = false;
      std::size_t twdl_idx_ctrl = 0;
      std::size_t sample_idx = 0;

      if (ctrl_mode && stage_has_nontrivial_tw) {
        sample_idx = static_cast<std::size_t>(base + half + i);

        const auto ctrlsigs =
            ctrl->decode_from_count(static_cast<typename CtrlT::CountT>(sample_idx));

        use_twdl_ctrl = ctrlsigs.twdlrom_en[static_cast<std::size_t>(tw_slot)];
        twdl_idx_ctrl =static_cast<std::size_t>(ctrlsigs.tw_addr_global[static_cast<std::size_t>(tw_slot)]);

        std::ostringstream oss;
        oss << "[STAGE " << stage_idx << "] ctrl_decode"
            << " base=" << base
            << " i=" << i
            << " sample_idx=" << sample_idx
            << " tw_slot=" << tw_slot
            << " ctrl_use_twdl=" << (use_twdl_ctrl ? 1 : 0)
            << " ctrl_twdl_idx=" << twdl_idx_ctrl
            << " fallback_use_twdl=" << (use_twdl_fallback ? 1 : 0)
            << " fallback_twdl_idx=" << twdl_idx_fallback;
        trace_line(oss.str());
      }

      bool use_twdl = false;
      std::size_t twdl_idx = 0;
      const char* tw_source = "fallback";

      if (ctrl_mode && stage_has_nontrivial_tw) {
        use_twdl = use_twdl_ctrl;
        twdl_idx = twdl_idx_ctrl;
        tw_source = "ctrl";
      } else {
        use_twdl = use_twdl_fallback;
        twdl_idx = twdl_idx_fallback;
        tw_source = "fallback";
      }

      {
        std::ostringstream oss;
        oss << "[STAGE " << stage_idx << "] butterfly_in"
            << " base=" << base
            << " i=" << i
            << " a=" << stdcmplx_to_string(a)
            << " b=" << stdcmplx_to_string(b)
            << " use_twdl=" << (use_twdl ? 1 : 0)
            << " twdl_idx=" << twdl_idx
            << " tw_source=" << tw_source;
        trace_line(oss.str());
      }

      if (flow_mode == FFTFlowMode::DIT) {
        CxT t = b;
        CxT w(1, 0);

        if (use_twdl) {
          w = get_twiddle(twdl_idx, inverse);
          const ComplexPlain<T> tb_plain =cmul_mul_tlm(to_plain(b), to_plain(w));
          t = CxT(tb_plain.re, tb_plain.im);

          std::ostringstream oss;
          oss << "[STAGE " << stage_idx << "] dit_twdl"
              << " base=" << base
              << " i=" << i
              << " w=" << stdcmplx_to_string(w)
              << " b_plain=" << plaincmplx_to_string(to_plain(b))
              << " t=" << stdcmplx_to_string(t);
          trace_line(oss.str());
        }

        out[base + i]        = a + t;
        out[base + half + i] = a - t;

        std::ostringstream oss;
        oss << "[STAGE " << stage_idx << "] dit_out"
            << " base=" << base
            << " i=" << i
            << " out_lo=" << stdcmplx_to_string(out[base + i])
            << " out_hi=" << stdcmplx_to_string(out[base + half + i]);
        trace_line(oss.str());

      } else {
        const CxT sum  = a + b;
        const CxT diff = a - b;

        CxT diff_tw = diff;
        CxT w(1, 0);

        if (use_twdl) {
          w = get_twiddle(twdl_idx, inverse);
          const ComplexPlain<T> prod =
              cmul_mul_tlm(to_plain(diff), to_plain(w));
          diff_tw = CxT(prod.re, prod.im);

          std::ostringstream oss;
          oss << "[STAGE " << stage_idx << "] dif_twdl"
              << " base=" << base
              << " i=" << i
              << " sum=" << stdcmplx_to_string(sum)
              << " diff=" << stdcmplx_to_string(diff)
              << " w=" << stdcmplx_to_string(w)
              << " diff_tw=" << stdcmplx_to_string(diff_tw);
          trace_line(oss.str());
        }

        out[base + i]        = sum;
        out[base + half + i] = diff_tw;

        std::ostringstream oss;
        oss << "[STAGE " << stage_idx << "] dif_out"
            << " base=" << base
            << " i=" << i
            << " out_lo=" << stdcmplx_to_string(out[base + i])
            << " out_hi=" << stdcmplx_to_string(out[base + half + i]);
        trace_line(oss.str());
      }
    }
  }

  {
    std::ostringstream oss;
    oss << "[STAGE " << stage_idx << "] end";
    trace_line(oss.str());
  }
}

// -----------------------------------------------------------------------------
// R2SdfFFTTLM
// -----------------------------------------------------------------------------
template <typename E>
std::unique_ptr<R2SdfFFTTLM<E>>
R2SdfFFTTLM<E>::create(Context<E>& ctx,
                       sc_core::sc_module_name name,
                       std::size_t fftsz,
                       FFTFlowMode fm,
                       FFTDirection fd) {
  return std::unique_ptr<R2SdfFFTTLM<E>>(
      new R2SdfFFTTLM<E>(ctx, name, fftsz, fm, fd));
}


template <typename E>
R2SdfFFTTLM<E>::R2SdfFFTTLM(Context<E>& ctx,
                            sc_core::sc_module_name name,
                            std::size_t fftsz,
                            FFTFlowMode fm,
                            FFTDirection fd)
    : sc_core::sc_module(name),
      fft_size(fftsz),
      flow_mode(fm),
      fft_dir(fd),
      scale_each_stage(E::scale_each_stage),
      use_ctrl(E::use_ctrl) {
  if (ctx.arg.trace_enabled) {
    tracefile = std::make_shared<TraceFile<E>>(ctx);

    // --output belongs to normal model output. Keep the diagnostic trace
    // independent until a dedicated --trace-file option is added.
    const std::string path = std::string(this->name()) + "_trace.log";

    tracefile->open(path, 1 << 20, 0777);
    tracefile->write_line("=== R2SdfFFTTLM trace start ===");
    tracefile->write_kvstr("name", this->name());
    tracefile->write_kvstr("fft_size", std::to_string(fft_size));
    tracefile->write_kvstr("flow_mode", flow_mode == FFTFlowMode::DIT ? "DIT" : "DIF");
    tracefile->write_kvstr("fft_dir", fft_dir == FFTDirection::FFT ? "FFT" : "IFFT");
    tracefile->write_kvstr("scale_each_stage", scale_each_stage ? "true" : "false");
    tracefile->write_kvstr("use_ctrl", use_ctrl ? "true" : "false");
  }

  targ_socket.register_b_transport(this, &R2SdfFFTTLM<E>::b_transport);
  targ_socket.register_get_direct_mem_ptr(this, &R2SdfFFTTLM<E>::get_direct_mem_ptr);
  targ_socket.register_transport_dbg(this, &R2SdfFFTTLM<E>::transport_dbg);
}
template <typename E>
R2SdfFFTTLM<E>::~R2SdfFFTTLM() {
  if (tracefile && tracefile->enabled()) {
    tracefile->close();
  }
}

template <typename E>
std::size_t R2SdfFFTTLM<E>::get_fftsize() const {
  return fft_size;
}

template <typename E>
void R2SdfFFTTLM<E>::get_realfft(const VecR& in, VecC& out) const {
  VecC in_c(in.size(), CxT(0, 0));
  for (std::size_t i = 0; i < in.size(); ++i) {
    in_c[i] = CxT(in[i], 0);
  }
  get_cmplxfft(in_c, out);
}

template <typename E>
void R2SdfFFTTLM<E>::get_cmplxfft(const VecC& in, VecC& out) const {
  process_frame(in, out, false);
}

template <typename E>
void R2SdfFFTTLM<E>::get_cmplxifft(const VecC& in, VecC& out) const {
  process_frame(in, out, true);
}

template <typename E>
void R2SdfFFTTLM<E>::allocate_state(Context<E>& ctx) {
  validate_fft_size();
  allocate_twiddle(ctx);

  if (use_ctrl && !ctrl) {
    ctrl = R2SdfCtrlTLM<E>::create(ctx, sc_core::sc_gen_unique_name("r2sdf_ctrl"), 
                                   flow_mode, fft_size);
  }

  r2sdfstgs.clear();
  r2sdfstgs.reserve(get_nstages());

  for (std::size_t i = 0; i < get_nstages(); ++i) {
    auto stg = R2SdfStageTLM<E>::create(
      ctx, sc_core::sc_gen_unique_name("r2sdf_stage"),
      fft_size, i, flow_mode, twiddle_mem.get(),
      tracefile, ctrl, use_ctrl);

    stg->allocate_state(ctx);
    r2sdfstgs.push_back(std::move(stg));
  }

  last_fftin.assign(fft_size, CxT(0, 0));
  last_fftout.assign(fft_size, CxT(0, 0));
  is_init = true;
}

template <typename E>
void R2SdfFFTTLM<E>::state_reset() {
  for (auto& stg : r2sdfstgs) {
    stg->reset_state();
  }
  if(ctrl) ctrl->reset();
  
  std::fill(last_fftin.begin(), last_fftin.end(), CxT(0, 0));
  std::fill(last_fftout.begin(), last_fftout.end(), CxT(0, 0));
}

template <typename E>
void R2SdfFFTTLM<E>::update_hyperparams(const json& params) {
  if (params.contains("scale_each_stage")) {
    scale_each_stage = params.at("scale_each_stage").template get<bool>();
  }
  if (params.contains("use_ctrl")) {
    use_ctrl = params.at("use_ctrl").template get<bool>();
  }
}

template <typename E>
json R2SdfFFTTLM<E>::get_hyperparams() const {
  return {
    {"otype", "r2sdf_fft_tlm"},
    {"fft_size", fft_size},
    {"flow_mode", flow_mode == FFTFlowMode::DIT ? "dit" : "dif"},
    {"scale_each_stage", scale_each_stage},
    {"use_ctrl", use_ctrl}
  };
}

template <typename E>
json R2SdfFFTTLM<E>::serialize(Context<E>& ctx) const {
  if (ctx.arg.verbose) {
    Out(ctx) << "Serializing FFT: " << name();
  }

  json j = {
    {"otype", "r2sdf_fft_tlm"},
    {"fft_size", fft_size},
    {"flow_mode", flow_mode == FFTFlowMode::DIT ? "dit" : "dif"},
    {"scale_each_stage", scale_each_stage},
    {"use_ctrl", use_ctrl},
    {"is_init", is_init},
    {"nstages", get_nstages()},
    {"last_fftin", cvec_to_json(last_fftin)},
    {"last_fftout", cvec_to_json(last_fftout)}
  };

  if (twiddle_mem) {
    j["twiddle_mem"] = twiddle_mem->serialize(ctx);
  } else {
    j["twiddle_mem"] = nullptr;
  }

  // Controller state is not serialized here, because current FFT model is
  // frame/block oriented and does not depend on in-flight sample pipeline state.
  return j;
}

template <typename E>
void R2SdfFFTTLM<E>::deserialize(Context<E>& ctx, const json& data) {
  if (!data.is_object()) {
    throw std::runtime_error("FFT deserialize expects a JSON object");
  }

  if (data.contains("fft_size")) {
    fft_size = data.at("fft_size").template get<std::size_t>();
  }

  if (data.contains("flow_mode")) {
    const auto fm = data.at("flow_mode").template get<std::string>();
    if (fm == "dit") {
      flow_mode = FFTFlowMode::DIT;
    } else if (fm == "dif") {
      flow_mode = FFTFlowMode::DIF;
    } else {
      throw std::runtime_error("Invalid flow_mode in FFT deserialization");
    }
  }

  if (data.contains("scale_each_stage")) {
    scale_each_stage = data.at("scale_each_stage").template get<bool>();
  }

  if (data.contains("use_ctrl")) {
    use_ctrl = data.at("use_ctrl").template get<bool>();
  }

  validate_fft_size();

  // Rebuild internal state so restored object is immediately usable
  twiddle_mem.reset();
  ctrl.reset();
  r2sdfstgs.clear();
  is_init = false;
  allocate_state(ctx);

  if (data.contains("twiddle_mem") && !data.at("twiddle_mem").is_null()) {
    twiddle_mem->deserialize(ctx, data.at("twiddle_mem"));
  }

  if (data.contains("last_fftin")) {
    last_fftin = cvec_from_json<T>(data.at("last_fftin"));
  } else {
    last_fftin.assign(fft_size, CxT(0, 0));
  }

  if (data.contains("last_fftout")) {
    last_fftout = cvec_from_json<T>(data.at("last_fftout"));
  } else {
    last_fftout.assign(fft_size, CxT(0, 0));
  }

  if (ctx.arg.verbose) {
    Out(ctx) << "Deserialized FFT: " << name()
             << " fft_size=" << fft_size
             << " flow_mode=" << (flow_mode == FFTFlowMode::DIT ? "dit" : "dif");
  }
}

template <typename E>
void R2SdfFFTTLM<E>::b_transport(tlm::tlm_generic_payload& trans,
                                 sc_core::sc_time& delay) {
  using Txn = FFTFrameTxn<T>;

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
      trans.get_data_length() != sizeof(Txn) ||
      trans.get_streaming_width() < sizeof(Txn)) {
    trans.set_response_status(tlm::TLM_BURST_ERROR_RESPONSE);
    return;
  }

  auto* txn = reinterpret_cast<Txn*>(trans.get_data_ptr());
  txn->ok = false;
  txn->error.clear();

  const auto nstg = get_nstages();
  const double btfly_delay = static_cast<double>(nstg * E::butterfly_latency);
  const double mem_delay = static_cast<double>(nstg * E::memory_latency);
  const double twdl_delay = static_cast<double>((nstg > 0 ? nstg - 1 : 0) * E::twiddle_latency);
  const double mul_delay = static_cast<double>((nstg > 0 ? nstg - 1 : 0) * E::cmplxmul_latency);
  const sc_core::sc_time frame_delay(btfly_delay + mem_delay + twdl_delay + mul_delay, sc_core::SC_NS);

  try {
    if (trans.get_command() == tlm::TLM_READ_COMMAND) {
      txn->out_cplx = last_fftout;
      txn->ok = true;
      delay += sc_core::sc_time(E::memory_latency, sc_core::SC_NS);
      trans.set_response_status(tlm::TLM_OK_RESPONSE);
      return;
    }

    switch (txn->op) {
      case Txn::Op::FFT_REAL: {
        auto in = maybepad_realvec(txn->in_real, fft_size);
        get_realfft(in, txn->out_cplx);
        break;
      }

      case Txn::Op::FFT_CPLX: {
        auto in = maybepad_cplxvec(txn->in_cplx, fft_size);
        get_cmplxfft(in, txn->out_cplx);
        break;
      }

      case Txn::Op::IFFT_CPLX: {
        auto in = maybepad_cplxvec(txn->in_cplx, fft_size);
        get_cmplxifft(in, txn->out_cplx);
        break;
      }

      default:
        throw std::runtime_error("Unknown FFT transaction opcode");
    }

    txn->ok = true;
    delay += frame_delay;
    trans.set_response_status(tlm::TLM_OK_RESPONSE);
  } catch (const std::exception& e) {
    txn->ok = false;
    txn->error = e.what();
    trans.set_response_status(tlm::TLM_GENERIC_ERROR_RESPONSE);
  }
}

template <typename E>
bool R2SdfFFTTLM<E>::get_direct_mem_ptr(tlm::tlm_generic_payload&,
                                        tlm::tlm_dmi& dmi_data) {
  // This model is command/compute oriented, not memory mapped.
  dmi_data.set_start_address(0);
  dmi_data.set_end_address(0);
  dmi_data.set_dmi_ptr(nullptr);
  dmi_data.set_read_latency(sc_core::sc_time(E::memory_latency, sc_core::SC_NS));
  dmi_data.set_write_latency(sc_core::sc_time(E::memory_latency, sc_core::SC_NS));
  return false;
}

template <typename E>
unsigned int R2SdfFFTTLM<E>::transport_dbg(
    tlm::tlm_generic_payload& trans) {
  using Txn = FFTFrameTxn<T>;

  if (trans.get_command() != tlm::TLM_READ_COMMAND) {
    trans.set_response_status(tlm::TLM_COMMAND_ERROR_RESPONSE);
    return 0;
  }

  if (trans.get_address() != 0) {
    trans.set_response_status(tlm::TLM_ADDRESS_ERROR_RESPONSE);
    return 0;
  }

  if (trans.get_byte_enable_ptr() != nullptr) {
    trans.set_response_status(tlm::TLM_BYTE_ENABLE_ERROR_RESPONSE);
    return 0;
  }

  if (trans.get_data_ptr() == nullptr ||
      trans.get_data_length() != sizeof(Txn)) {
    trans.set_response_status(tlm::TLM_BURST_ERROR_RESPONSE);
    return 0;
  }

  auto* txn = reinterpret_cast<Txn*>(trans.get_data_ptr());
  txn->out_cplx = last_fftout;
  txn->ok = true;
  txn->error.clear();

  trans.set_dmi_allowed(false);
  trans.set_response_status(tlm::TLM_OK_RESPONSE);
  return static_cast<unsigned int>(sizeof(Txn));
}

template <typename E>
void R2SdfFFTTLM<E>::dump_state(Context<E>& ctx, const std::string& desc) const {
  if (!ctx.arg.verbose) {
    return;
  }

  Out(ctx) << "==== R2SdfFFTTLM state dump ====";
  if (!desc.empty()) {
    Out(ctx) << "desc: " << desc;
  }

  Out(ctx) << "name            : " << name();
  Out(ctx) << "fft_size        : " << fft_size;
  Out(ctx) << "flow_mode       : " << (flow_mode == FFTFlowMode::DIT ? "DIT" : "DIF");
  Out(ctx) << "nstages         : " << get_nstages();
  Out(ctx) << "scale_each_stage: " << (scale_each_stage ? "true" : "false");
  Out(ctx) << "use_ctrl        : " << (use_ctrl ? "true" : "false");
  Out(ctx) << "is_init         : " << (is_init ? "true" : "false");
  Out(ctx) << "last_fftin size : " << last_fftin.size();
  Out(ctx) << "last_fftout size: " << last_fftout.size();

  const std::size_t preview = std::min<std::size_t>(4, last_fftout.size());
  for (std::size_t i = 0; i < preview; ++i) {
    Out(ctx) << "last_fftout[" << i << "] = " << stdcmplx_to_string(last_fftout[i]);
  }

  if (twiddle_mem) {
    Out(ctx) << "twiddle_mem     : present";
  } else {
    Out(ctx) << "twiddle_mem     : null";
  }

  Out(ctx) << "===============================";
}

template <typename E>
void R2SdfFFTTLM<E>::validate_fft_size() const {
  if (fft_size == 0 || ((fft_size & (fft_size - 1)) != 0)) {
    throw std::invalid_argument("R2SdfFFTTLM: fft_size must be power-of-2");
  }
}

template <typename E>
std::size_t R2SdfFFTTLM<E>::get_nstages() const {
  std::size_t n = fft_size;
  std::size_t s = 0;
  while (n > 1) {
    n >>= 1;
    ++s;
  }
  return s;
}

template <typename E>
void R2SdfFFTTLM<E>::allocate_twiddle(Context<E>& ctx) {
  const std::size_t mem_size = 2 * fft_size;
  std::vector<T> init(mem_size, T(0));

  const T pi = static_cast<T>(3.14159265358979323846);
  for (std::size_t k = 0; k < fft_size; ++k) {
    const T ang =static_cast<T>(-2) * pi * static_cast<T>(k) / static_cast<T>(fft_size);
    init[2 * k]     = std::cos(ang);
    init[2 * k + 1] = std::sin(ang);
  }

  twiddle_mem = SyscMemory<E>::create(
    ctx, sc_core::sc_gen_unique_name("twiddle_rom"),
    mem_size, init.data());
  twiddle_init_socket.bind(twiddle_mem->targ_socket);
}

template <typename E>
void R2SdfFFTTLM<E>::bit_reverse(VecC& data) const {
  std::size_t j = 0;
  for (std::size_t i = 1; i < data.size(); ++i) {
    std::size_t bit = data.size() >> 1;
    while (j & bit) {
      j ^= bit;
      bit >>= 1;
    }
    j |= bit;
    if (i < j) {
      std::swap(data[i], data[j]);
    }
  }
}

template <typename E>
void R2SdfFFTTLM<E>::process_frame(const VecC& in,
                                   VecC& out,
                                   bool inverse) const {
  if (!is_init) {
    throw std::runtime_error("R2SdfFFTTLM not initialized");
  }

  if (in.size() != fft_size) {
    throw std::invalid_argument("R2SdfFFTTLM frame size mismatch");
  }

  auto* self = const_cast<R2SdfFFTTLM<E>*>(this);
  self->last_fftin = in;

  VecC cur = in;

  if (flow_mode == FFTFlowMode::DIT) {
    bit_reverse(cur);
  }

  for (std::size_t s = 0; s < self->r2sdfstgs.size(); ++s) {
    VecC nxt;
    self->r2sdfstgs[s]->process_block(cur, nxt, inverse);

    if (tracefile && tracefile->enabled()) {
      std::ostringstream oss;
      oss << "[FFT] stage=" << s
          << " inverse=" << (inverse ? 1 : 0)
          << " scale_each_stage=" << (scale_each_stage ? 1 : 0);
      tracefile->write_line(oss.str());
    }

    // Only IFFT should receive 1/N normalization.
    // If scale_each_stage is true, radix-2 scaling is 1/2 per stage.
    if (inverse && scale_each_stage) {
      const T half = T(0.5);
      for (auto& z : nxt) {
        z *= half;
      }
    }

    cur.swap(nxt);
  }

  if (flow_mode == FFTFlowMode::DIF) {
    bit_reverse(cur);
  }

  // If not distributed per-stage scaling, apply final 1/N for IFFT.
  if (inverse && !scale_each_stage) {
    const T scale = T(1) / static_cast<T>(fft_size);
    for (auto& z : cur) {
      z *= scale;
    }
  }

  out = cur;
  self->last_fftout = out;
}

template <typename E>
class FFTTLMInitiator : public sc_core::sc_module {
public:
  using T       = typename E::Eval_T;
  using CxT     = std::complex<T>;
  using VecR    = std::vector<T>;
  using VecC    = std::vector<CxT>;
  using Txn     = FFTFrameTxn<T>;
  using FFTMode = typename FFTIntf<T>::FFTMode;
  using FxptT = typename E::Fxpt_T;

  tlm_utils::simple_initiator_socket<FFTTLMInitiator> init_socket{"init_socket"};

  Context<E>& ctx;
  std::size_t fftsize;
  bool pass{true};

  FFTTLMInitiator(sc_core::sc_module_name name, Context<E>& ctx, 
                  std::size_t fftsz) : sc_core::sc_module(name), ctx(ctx), fftsize(fftsz) {
    SC_THREAD(run);
  }

private:

  T get_syscfxlsb() const {
    return static_cast<T>(std::ldexp(1.0, -E::fx_frac_bits));
  }

  T get_cmprtol() const {
    if (ctx.arg.fixedpoint_eval) {
      return std::max(static_cast<T>(ctx.arg.fixedpoint_tol), get_syscfxlsb());
    }

    return static_cast<T>(1e-4);
  }


  bool compare_cvec(const VecC& got, const VecC& exp,
                    const std::string& tag, T tol) {
    if (got.size() != exp.size()) {
      std::ostringstream oss;
      oss << tag << ": size mismatch, got=" << got.size()
          << " expected=" << exp.size();
      SC_REPORT_ERROR("FFTTLMInitiator", oss.str().c_str());
      return false;
    }

    const T cmp_eps = std::max(
        static_cast<T>(1e-6),
        static_cast<T>(16) *
            std::numeric_limits<T>::epsilon() *
            std::max(static_cast<T>(1), tol));

    T max_err = T(0);
    std::size_t max_idx = 0;

    for (std::size_t i = 0; i < got.size(); ++i) {
      const T err_re = std::abs(got[i].real() - exp[i].real());
      const T err_im = std::abs(got[i].imag() - exp[i].imag());
      const T err = std::max(err_re, err_im);

      if (err > max_err) {
        max_err = err;
        max_idx = i;
      }

      if (err > tol + cmp_eps) {
        std::ostringstream oss;
        oss << tag << ": mismatch at i=" << i
            << " got=(" << got[i].real() << "," << got[i].imag() << ")"
            << " exp=(" << exp[i].real() << "," << exp[i].imag() << ")"
            << " err=" << err
            << " tol=" << tol
            << " cmp_eps=" << cmp_eps;
        SC_REPORT_ERROR("FFTTLMInitiator", oss.str().c_str());
        return false;
      }
    }

    if (ctx.arg.verbose) {
      Out(ctx) << tag << " PASS, max_err=" << max_err
              << " at index " << max_idx
              << " tol=" << tol
              << " cmp_eps=" << cmp_eps << "\n";
    }

    return true;
  }


  bool submit_job(Txn& job) {
    tlm::tlm_generic_payload tr;
    sc_core::sc_time delay = sc_core::SC_ZERO_TIME;

    tr.set_command(tlm::TLM_WRITE_COMMAND);
    tr.set_address(0);
    tr.set_data_ptr(reinterpret_cast<unsigned char*>(&job));
    tr.set_data_length(sizeof(Txn));
    tr.set_streaming_width(sizeof(Txn));
    tr.set_byte_enable_ptr(nullptr);
    tr.set_dmi_allowed(false);
    tr.set_response_status(tlm::TLM_INCOMPLETE_RESPONSE);

    init_socket->b_transport(tr, delay);
    wait(delay);

    if (tr.is_response_error() || !job.ok) {
      std::ostringstream oss;
      oss << "FFT WRITE failed";
      if (!job.error.empty()) {
        oss << ": " << job.error;
      }
      SC_REPORT_ERROR("FFTTLMInitiator", oss.str().c_str());
      return false;
    }
    return true;
  }

  bool readback_lastoutput(VecC& out) {
    Txn trbuf;
    tlm::tlm_generic_payload tr;
    sc_core::sc_time delay = sc_core::SC_ZERO_TIME;

    tr.set_command(tlm::TLM_READ_COMMAND);
    tr.set_address(0);
    tr.set_data_ptr(reinterpret_cast<unsigned char*>(&trbuf));
    tr.set_data_length(sizeof(Txn));
    tr.set_streaming_width(sizeof(Txn));
    tr.set_byte_enable_ptr(nullptr);
    tr.set_dmi_allowed(false);
    tr.set_response_status(tlm::TLM_INCOMPLETE_RESPONSE);

    init_socket->b_transport(tr, delay);
    wait(delay);

    if (tr.is_response_error() || !trbuf.ok) {
      std::ostringstream oss;
      oss << "FFT READ failed";
      if (!trbuf.error.empty()) {
        oss << ": " << trbuf.error;
      }
      SC_REPORT_ERROR("FFTTLMInitiator", oss.str().c_str());
      return false;
    }

    out = trbuf.out_cplx;
    return true;
  }

  bool verify_cmplxfft() {
    if (ctx.arg.verbose) {
      Out(ctx) << "[TB] verify_cmplxfft\n";
    }

    VecC in(fftsize, CxT(0, 0));
    in[0] = CxT(1, 0);
    in[1] = CxT(2, -1);
    in[2] = CxT(0, 0.5);
    in[3] = CxT(-1, 0.25);

    if (ctx.arg.fixedpoint_eval) {
      in = cmplxvec_from_archsyscfx<E>(in);
    }

    Txn job;
    job.op = Txn::Op::FFT_CPLX;
    job.in_cplx = in;

    if (!submit_job(job)) {
      return false;
    }

    EigenFFTWrapper<T> golden(FFTMode::Complex, fftsize);
    VecC golden_out;
    golden.get_cmplxfft(in, golden_out);

    if (ctx.arg.fixedpoint_eval) {
      golden_out = cmplxvec_from_archsyscfx<E>(golden_out);
    }

    const T tol = get_cmprtol();

    if (!compare_cvec(job.out_cplx, golden_out,
                      "FFT_CPLX/writeback",
                      tol)) {
      return false;
    }

    VecC readback;
    if (!readback_lastoutput(readback)) {
      return false;
    }

    return compare_cvec(readback, golden_out,
                        "FFT_CPLX/readback", tol);
  }

  bool verify_cmplxifft() {
    if (ctx.arg.verbose) {
      Out(ctx) << "[TB] verify_cmplxifft\n";
    }

    VecC in_freq(fftsize, CxT(0, 0));
    in_freq[0] = CxT(1, 0);
    in_freq[2] = CxT(0.5, -0.25);

    if (ctx.arg.fixedpoint_eval) {
      in_freq = cmplxvec_from_archsyscfx<E>(in_freq);
    }

    Txn job;
    job.op = Txn::Op::IFFT_CPLX;
    job.in_cplx = in_freq;

    if (!submit_job(job)) {
      return false;
    }

    EigenFFTWrapper<T> golden(FFTMode::Complex, fftsize);
    VecC golden_out;
    golden.get_cmplxifft(in_freq, golden_out);

    if (ctx.arg.fixedpoint_eval) {
      golden_out = cmplxvec_from_archsyscfx<E>(golden_out);
    }

    const T tol = get_cmprtol();

    if (!compare_cvec(job.out_cplx, golden_out,
                      "IFFT_CPLX/writeback",
                      tol)) {
      return false;
    }

    VecC readback;
    if (!readback_lastoutput(readback)) {
      return false;
    }

    return compare_cvec(readback, golden_out,
                        "IFFT_CPLX/readback", tol);
  }

  bool verify_realfft() {
    if (ctx.arg.verbose) {
      Out(ctx) << "[TB] verify_realfft\n";
    }

    VecR in(fftsize, T(0));
    in[0] = T(1);
    in[1] = T(2);
    in[2] = T(3);
    in[3] = T(4);

    if (ctx.arg.fixedpoint_eval) {
      in = vec_from_archsyscfx<E>(in);
    }

    Txn job;
    job.op = Txn::Op::FFT_REAL;
    job.in_real = in;

    if (!submit_job(job)) {
      return false;
    }

    EigenFFTWrapper<T> golden(FFTMode::Real, fftsize);
    VecC golden_out;
    golden.get_realfft(in, golden_out);

    if (ctx.arg.fixedpoint_eval) {
      golden_out = cmplxvec_from_archsyscfx<E>(golden_out);
    }

    const T tol = get_cmprtol();

    if (!compare_cvec(job.out_cplx, golden_out,
                      "FFT_REAL/writeback",
                      tol)) {
      return false;
    }

    VecC readback;
    if (!readback_lastoutput(readback)) {
      return false;
    }

    return compare_cvec(readback, golden_out,
                        "FFT_REAL/readback", tol);
  }

public:
  void run() {
    try {
      bool ok = true;

      if (ctx.arg.compute_ifft) {
        ok &= verify_cmplxifft();
      } else {
        ok &= verify_cmplxfft();
        ok &= verify_realfft();
      }

      pass = ok;
    } catch (const std::exception& e) {
      pass = false;
      SC_REPORT_ERROR("FFTTLMInitiator", e.what());
    }

    if (ctx.arg.verbose) {
      Out(ctx) << sc_core::sc_time_stamp()
              << (ctx.arg.compute_ifft ? " IFFT" : " FFT")
              << " testbench done, pass=" << (pass ? "true" : "false")
              << "\n";
    }

    sc_core::sc_stop();
  }
};

template <typename E>
bool R2SdfFFTTLM<E>::run_testbench(Context<E>& ctx) {
  constexpr std::size_t tb_fftsize = E::fft_size;

  const FFTDirection tb_dir = ctx.arg.compute_ifft ? FFTDirection::IFFT : FFTDirection::FFT;

  auto dut = R2SdfFFTTLM<E>::create(
    ctx, sc_core::sc_module_name("r2sdf_fft_dut"),
    tb_fftsize, E::use_dit ? FFTFlowMode::DIT : FFTFlowMode::DIF, tb_dir);

  dut->allocate_state(ctx);

  FFTTLMInitiator<E> tb("fft_tlm_tb", ctx, dut->get_fftsize());
  tb.init_socket.bind(dut->targ_socket);

  sc_core::sc_start();

  dut->dump_state(ctx, "end-of-testbench");

  if (dut->tracefile && dut->tracefile->enabled()) {
    dut->tracefile->close();
  }

  return tb.pass;
}


template class R2SdfFFTTLM<E>;
template class R2SdfCtrlTLM<E>;
template class R2SdfStageTLM<E>;

}

#endif