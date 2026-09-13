#pragma once

#include <systemc>

#include <algorithm>
#include <cstddef>
#include <vector>

namespace adptsysc {

template <typename E>
class ComplexShiftRegisterCycle : public sc_core::sc_module {
public:
  using T = typename E::Fxpt_T;

  sc_core::sc_in<bool> clk{"clk"};
  sc_core::sc_in<bool> rst_n{"rst_n"};
  sc_core::sc_in<bool> en{"en"};
  sc_core::sc_in<bool> clear{"clear"};

  sc_core::sc_in<T> din_re{"din_re"};
  sc_core::sc_in<T> din_im{"din_im"};
  sc_core::sc_out<T> dout_re{"dout_re"};
  sc_core::sc_out<T> dout_im{"dout_im"};

  ComplexShiftRegisterCycle(sc_core::sc_module_name name, std::size_t depth)
      : sc_core::sc_module(name), reg_re(depth), reg_im(depth) {
    if (depth == 0) {
      SC_REPORT_FATAL(this->name(), "depth must be > 0");
    }

    SC_CTHREAD(run, clk.pos());
    async_reset_signal_is(rst_n, false);
  }

  std::size_t size() const { return reg_re.size(); }

private:
  std::vector<T> reg_re;
  std::vector<T> reg_im;

  void clear_state() {
    std::fill(reg_re.begin(), reg_re.end(), T(0));
    std::fill(reg_im.begin(), reg_im.end(), T(0));
  }

  void run() {
    clear_state();
    dout_re.write(T(0));
    dout_im.write(T(0));
    wait();

    while (true) {
      if (clear.read()) {
        clear_state();
        dout_re.write(T(0));
        dout_im.write(T(0));
      } else if (en.read()) {
        const T old_re = reg_re.back();
        const T old_im = reg_im.back();

        for (std::size_t i = reg_re.size() - 1; i > 0; --i) {
          reg_re[i] = reg_re[i - 1];
          reg_im[i] = reg_im[i - 1];
        }

        reg_re[0] = din_re.read();
        reg_im[0] = din_im.read();
        dout_re.write(old_re);
        dout_im.write(old_im);
      }

      wait();
    }
  }
};

}  // namespace adptsysc
