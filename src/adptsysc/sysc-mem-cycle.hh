#pragma once

#include <systemc>

#include <adptsysc/adptsysc.hh>
#include <adptsysc/object.hh>
#include <adptsysc/syscfx-utils.hh>

#include <algorithm>
#include <cstddef>
#include <fstream>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace adptsysc {

template <typename E>
class SyscMemoryCycle : public sc_core::sc_module {
public:
  using T = typename E::Fxpt_T;
  using AddrT = sc_dt::sc_uint<64>;

  sc_core::sc_in<bool> clk{"clk"};
  sc_core::sc_in<bool> rst_n{"rst_n"};

  sc_core::sc_in<bool> req{"req"};
  sc_core::sc_in<bool> we{"we"};
  sc_core::sc_in<AddrT> addr{"addr"};
  sc_core::sc_in<T> wdata{"wdata"};

  sc_core::sc_out<bool> ready{"ready"};
  sc_core::sc_out<bool> rvalid{"rvalid"};
  sc_core::sc_out<T> rdata{"rdata"};


  static std::unique_ptr<SyscMemoryCycle<E>>
  create(Context<E>& ctx,
         sc_core::sc_module_name name,
         std::size_t depth,
         const T* initptr = nullptr);

  static bool run_testbench(Context<E>& ctx);

  SyscMemoryCycle(sc_core::sc_module_name name,
                  Context<E>& ctx,
                  std::size_t depth,
                  const T* initptr = nullptr);

  std::size_t size() const { return mem.size(); }
  T* data() { return mem.data(); }
  const T* data() const { return mem.data(); }

  void clear(T value = T(0)) { std::fill(mem.begin(), mem.end(), value); }

  void load_text(Context<E>& ctx, const std::string& path, bool hex);
  void save_text(Context<E>& ctx, const std::string& path, bool hex) const;

private:
  std::vector<T> mem;
  unsigned read_cycles = 1;
  unsigned write_cycles = 1;

  void run();
};

}  // namespace adptsysc
