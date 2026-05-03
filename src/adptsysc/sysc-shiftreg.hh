#pragma once

#include <systemc>
#include <tlm>
#include <tlm_utils/simple_target_socket.h>
#include <vector>
#include <adptsysc/design-lib.hh>

namespace adptsysc {

template <typename T>
class ComplexShiftRegisterTLM : public sc_core::sc_module {
public:
  tlm_utils::simple_target_socket<ComplexShiftRegisterTLM> targ_socket{"targ_socket"};

  ComplexShiftRegisterTLM(
      sc_core::sc_module_name name,
      std::size_t depth,
      sc_core::sc_time sample_period = sc_core::sc_time(1, sc_core::SC_NS))
      : sc_core::sc_module(name),
        sample_period(sample_period),
        reg(depth) {
    if (reg.empty()) {
      SC_REPORT_FATAL(this->name(), "depth must be > 0");
    }
    clear();
    targ_socket.register_b_transport(this, &ComplexShiftRegisterTLM::b_transport);
  }

  void clear() {
    for (auto& v : reg) {
      v = ComplexPlain<T>{};
    }
  }

  std::size_t size() const {
    return reg.size();
  }

  ComplexPlain<T> step(const ComplexPlain<T>& in) {
    ComplexPlain<T> out = reg.back();
    for (std::size_t i = reg.size() - 1; i > 0; --i) {
      reg[i] = reg[i - 1];
    }
    reg[0] = in;
    return out;
  }

private:
  sc_core::sc_time sample_period;
  std::vector<ComplexPlain<T>> reg;

  void b_transport(tlm::tlm_generic_payload& trans, sc_core::sc_time& delay) {
    using Txn = ShiftRegTLMTrans<T>;

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

    if (txn->op == Txn::Op::CLEAR) {
      clear();
      txn->out = ComplexPlain<T>{};
    } else {
      txn->out = step(txn->in);
    }

    delay += sample_period;
    trans.set_dmi_allowed(false);
    trans.set_response_status(tlm::TLM_OK_RESPONSE);
  }
};

} // namespace adptsysc