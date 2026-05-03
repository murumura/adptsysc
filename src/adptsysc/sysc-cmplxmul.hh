#pragma once

#include <systemc>
#include <tlm>
#include <tlm_utils/simple_target_socket.h>
#include <adptsysc/design-lib.hh>

namespace adptsysc {

template <typename T>
class ComplexMultiplierTLM : public sc_core::sc_module {
public:
  tlm_utils::simple_target_socket<ComplexMultiplierTLM> targ_socket{"targ_socket"};

  ComplexMultiplierTLM(
      sc_core::sc_module_name name,
      sc_core::sc_time latency = sc_core::sc_time(1, sc_core::SC_NS))
      : sc_core::sc_module(name),
        latency(latency) {
    targ_socket.register_b_transport(this, &ComplexMultiplierTLM::b_transport);
  }

  ComplexPlain<T> mul(const ComplexPlain<T>& a, const ComplexPlain<T>& b) const {
    ComplexPlain<T> y{};
    y.re = a.re * b.re - a.im * b.im;
    y.im = a.re * b.im + a.im * b.re;
    return y;
  }

private:
  sc_core::sc_time latency;

  void b_transport(tlm::tlm_generic_payload& trans, sc_core::sc_time& delay) {
    using Txn = ComplexMulTLMTrans<T>;

    if (trans.get_command() != tlm::TLM_WRITE_COMMAND) {
      trans.set_response_status(tlm::TLM_COMMAND_ERROR_RESPONSE);
      return;
    }

    if (trans.get_byte_enable_ptr() != nullptr) {
      trans.set_response_status(tlm::TLM_BYTE_ENABLE_ERROR_RESPONSE);
      return;
    }

    if (trans.get_data_ptr() == nullptr ||
        trans.get_data_length() != sizeof(Txn) ||
        trans.get_streaming_width() < sizeof(Txn)) {
      trans.set_response_status(tlm::TLM_GENERIC_ERROR_RESPONSE);
      return;
    }

    auto* txn = reinterpret_cast<Txn*>(trans.get_data_ptr());
    txn->y = mul(txn->a, txn->b);

    delay += latency;
    trans.set_dmi_allowed(false);
    trans.set_response_status(tlm::TLM_OK_RESPONSE);
  }
};

} // namespace adptsysc