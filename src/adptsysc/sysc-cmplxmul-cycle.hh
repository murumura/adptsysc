#pragma once

#include <systemc>

#include <adptsysc/design-lib.hh>

#include <algorithm>
#include <cstddef>

namespace adptsysc {

template <typename E>
class ComplexMultiplierCycle : public sc_core::sc_module {
public:
  using T = typename E::Fxpt_T;

  sc_core::sc_in<bool> clk{"clk"};
  sc_core::sc_in<bool> rst_n{"rst_n"};

  sc_core::sc_in<bool> in_valid{"in_valid"};
  sc_core::sc_out<bool> in_ready{"in_ready"};

  sc_core::sc_in<T> a_re{"a_re"};
  sc_core::sc_in<T> a_im{"a_im"};
  sc_core::sc_in<T> b_re{"b_re"};
  sc_core::sc_in<T> b_im{"b_im"};
  sc_core::sc_in<bool> conj_a{"conj_a"};
  sc_core::sc_in<bool> conj_b{"conj_b"};

  sc_core::sc_out<bool> out_valid{"out_valid"};
  sc_core::sc_in<bool> out_ready{"out_ready"};
  sc_core::sc_out<T> y_re{"y_re"};
  sc_core::sc_out<T> y_im{"y_im"};

  explicit ComplexMultiplierCycle(sc_core::sc_module_name name)
      : sc_core::sc_module(name) {
    SC_CTHREAD(run, clk.pos());
    async_reset_signal_is(rst_n, false);
  }

  static ComplexPlain<T> mul(const ComplexPlain<T>& a,
                             const ComplexPlain<T>& b,
                             bool conj_a = false,
                             bool conj_b = false) {
    const T ai = conj_a ? T(-a.im) : a.im;
    const T bi = conj_b ? T(-b.im) : b.im;

    ComplexPlain<T> y{};
    y.re = T(a.re * b.re - ai * bi);
    y.im = T(a.re * bi + ai * b.re);
    return y;
  }

private:
  static constexpr unsigned latency_cycles() {
    if constexpr (requires { E::cmplxmul_latency; }) {
      return std::max(1u, static_cast<unsigned>(E::cmplxmul_latency));
    }
    return 1u;
  }

  void run() {
    bool busy = false;
    bool valid = false;
    unsigned count = 0;
    ComplexPlain<T> pending{};

    in_ready.write(true);
    out_valid.write(false);
    y_re.write(T(0));
    y_im.write(T(0));
    wait();

    while (true) {
      const bool consume = valid && out_ready.read();
      if (consume) {
        valid = false;
      }

      if (busy) {
        if (count > 1) {
          --count;
        } else {
          busy = false;
          valid = true;
          y_re.write(pending.re);
          y_im.write(pending.im);
        }
      }

      const bool ready = !busy && !valid;
      in_ready.write(ready);

      if (ready && in_valid.read()) {
        const ComplexPlain<T> a{a_re.read(), a_im.read()};
        const ComplexPlain<T> b{b_re.read(), b_im.read()};
        pending = mul(a, b, conj_a.read(), conj_b.read());
        count = latency_cycles();
        busy = true;
        in_ready.write(false);
      }

      out_valid.write(valid);
      wait();
    }
  }
};

}  // namespace adptsysc
