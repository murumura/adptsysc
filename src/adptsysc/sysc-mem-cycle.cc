#include <adptsysc/config.hh>

#include <adptsysc/adptsysc.hh>
#include <adptsysc/sysc-mem-cycle.hh>

#include <iostream>
#include <sstream>
#include <string>

namespace adptsysc {

using E = ADPT_TARGET;

template <typename E>
std::unique_ptr<SyscMemoryCycle<E>>
SyscMemoryCycle<E>::create(Context<E>& ctx,
                           sc_core::sc_module_name name,
                           std::size_t depth,
                           const T* initptr) {
  return std::make_unique<SyscMemoryCycle<E>>(name, ctx, depth, initptr);
}

template <typename E>
SyscMemoryCycle<E>::SyscMemoryCycle(sc_core::sc_module_name name,
                                    Context<E>& ctx,
                                    std::size_t depth,
                                    const T* initptr)
    : sc_core::sc_module(name), mem(depth, T(0)) {
  if (depth == 0) {
    SC_REPORT_FATAL(this->name(), "memory depth must be > 0");
  }
  if (initptr != nullptr) {
    std::copy(initptr, initptr + depth, mem.begin());
  }

  read_cycles = static_cast<unsigned>(std::max(1, ctx.arg.mem_rddly_cycls));
  write_cycles = static_cast<unsigned>(std::max(1, ctx.arg.mem_wrdly_cycls));

  SC_CTHREAD(run, clk.pos());
  async_reset_signal_is(rst_n, false);
}

template <typename E>
void SyscMemoryCycle<E>::run() {
  bool active = false;
  bool pending_write = false;
  std::size_t pending_addr = 0;
  T pending_wdata = T(0);
  unsigned cycles_left = 0;

  ready.write(true);
  rvalid.write(false);
  rdata.write(T(0));
  wait();

  while (true) {
    // rvalid is a one-cycle response pulse.
    rvalid.write(false);

    if (active) {
      // Do not accept a second request while the current transaction is active.
      ready.write(false);

      if (cycles_left > 1) {
        --cycles_left;
      } else {
        if (pending_write) {
          mem[pending_addr] = pending_wdata;
        } else {
          rdata.write(mem[pending_addr]);
          rvalid.write(true);
        }

        active = false;
        ready.write(true);
      }
    } else {
      // A request is accepted only when the memory was already idle at this
      // rising edge.  This makes ready/req a conventional registered
      // request/accept protocol and avoids hidden same-edge re-acceptance.
      ready.write(true);

      if (req.read()) {
        const std::size_t a =
            static_cast<std::size_t>(addr.read().to_uint64());

        if (a >= mem.size()) {
          SC_REPORT_ERROR(this->name(), "address out of range");
        } else {
          pending_addr = a;
          pending_write = we.read();
          pending_wdata = wdata.read();
          cycles_left = pending_write ? write_cycles : read_cycles;

          active = true;
          ready.write(false);
        }
      }
    }

    wait();
  }
}

template <typename E>
void SyscMemoryCycle<E>::load_text(Context<E>&,
                                   const std::string& path,
                                   bool hex) {
  std::ifstream ifs(path);
  if (!ifs) {
    throw std::runtime_error("cannot open memory input: " + path);
  }

  std::string word;
  std::size_t i = 0;
  while (ifs >> word && i < mem.size()) {
    mem[i++] = hex ? archsyscfx_from_hexword<E>(word)
                   : archsyscfx_from_binword<E>(word);
  }
}

template <typename E>
void SyscMemoryCycle<E>::save_text(Context<E>&,
                                   const std::string& path,
                                   bool hex) const {
  std::ofstream ofs(path);
  if (!ofs) {
    throw std::runtime_error("cannot open memory output: " + path);
  }

  for (const auto& v : mem) {
    ofs << (hex ? archval_to_syscfx_hexword<E>(v)
                : archval_to_syscfx_binword<E>(v))
        << '\n';
  }
}

template <typename E>
class SyscMemoryCycleTB : public sc_core::sc_module {
public:
  using T = typename E::Fxpt_T;
  using AddrT = typename SyscMemoryCycle<E>::AddrT;

  sc_core::sc_in<bool> clk{"clk"};
  sc_core::sc_out<bool> rst_n{"rst_n"};
  sc_core::sc_out<bool> req{"req"};
  sc_core::sc_out<bool> we{"we"};
  sc_core::sc_out<AddrT> addr{"addr"};
  sc_core::sc_out<T> wdata{"wdata"};
  sc_core::sc_in<bool> ready{"ready"};
  sc_core::sc_in<bool> rvalid{"rvalid"};
  sc_core::sc_in<T> rdata{"rdata"};

  bool pass = true;

  explicit SyscMemoryCycleTB(sc_core::sc_module_name name)
      : sc_core::sc_module(name) {
    SC_THREAD(run);
    sensitive << clk.pos();
  }

private:
  void wait_until_ready() {
    while (!ready.read()) {
      wait();
    }
  }

  void write_word(std::size_t index, const T& value) {
    wait_until_ready();

    addr.write(static_cast<unsigned long long>(index));
    wdata.write(value);
    we.write(true);
    req.write(true);

    // Present the request for one complete clock interval.  At the following
    // rising edge the DUT samples req/we/addr/wdata.
    wait();

    req.write(false);
    we.write(false);

    // Do not immediately start another request in this same evaluation phase.
    // sc_signal writes become visible in the following update phase, so allow
    // the DUT's busy/ready state to propagate first.
    wait();

    wait_until_ready();
  }

  T read_word(std::size_t index) {
    wait_until_ready();

    addr.write(static_cast<unsigned long long>(index));
    we.write(false);
    req.write(true);

    // Request is sampled by the DUT on the following rising edge.
    wait();

    req.write(false);

    // rdata/rvalid are sc_signal outputs.  A DUT write performed at a clock
    // edge is not visible to this TB until the subsequent update phase.
    // Therefore wait for rvalid instead of checking it immediately after a
    // fixed number of clock edges.
    do {
      wait();
    } while (!rvalid.read());

    return rdata.read();
  }

  void check_word(std::size_t index, const T& expected) {
    const T got = read_word(index);

    if (got != expected) {
      pass = false;

      std::ostringstream oss;
      oss << "readback mismatch"
          << " addr=" << index
          << " expected=" << expected
          << " got=" << got
          << " time=" << sc_core::sc_time_stamp();

      SC_REPORT_ERROR("SyscMemoryCycleTB", oss.str().c_str());
    }
  }

  void run() {
    rst_n.write(false);
    req.write(false);
    we.write(false);
    addr.write(0);
    wdata.write(T(0));

    wait(2);

    rst_n.write(true);
    wait();

    // ------------------------------------------------------------------------
    // Test 1: sequential write/read
    // ------------------------------------------------------------------------
    for (std::size_t i = 0; i < 16; ++i) {
      write_word(i, T(100 + static_cast<int>(i)));
    }

    for (std::size_t i = 0; i < 16; ++i) {
      check_word(i, T(100 + static_cast<int>(i)));
    }

    // ------------------------------------------------------------------------
    // Test 2: signed values
    // ------------------------------------------------------------------------
    write_word(16, T(0));
    write_word(17, T(1));
    write_word(18, T(-1));
    write_word(19, T(7));
    write_word(20, T(-7));

    check_word(16, T(0));
    check_word(17, T(1));
    check_word(18, T(-1));
    check_word(19, T(7));
    check_word(20, T(-7));

    // ------------------------------------------------------------------------
    // Test 3: boundary address
    // ------------------------------------------------------------------------
    constexpr std::size_t last_idx = 127;
    write_word(last_idx, T(1234));
    check_word(last_idx, T(1234));

    std::cout << sc_core::sc_time_stamp()
              << " SyscMemoryCycle testbench done, pass="
              << (pass ? "true" : "false")
              << '\n';

    sc_core::sc_stop();
  }
};

template <typename E>
bool SyscMemoryCycle<E>::run_testbench(Context<E>& ctx) {
  sc_core::sc_clock clk("clk", sc_core::sc_time(10, sc_core::SC_NS));
  sc_core::sc_signal<bool> rst_n;
  sc_core::sc_signal<bool> req;
  sc_core::sc_signal<bool> we;
  sc_core::sc_signal<AddrT> addr;
  sc_core::sc_signal<T> wdata;
  sc_core::sc_signal<bool> ready;
  sc_core::sc_signal<bool> rvalid;
  sc_core::sc_signal<T> rdata;

  auto dut = SyscMemoryCycle<E>::create(ctx, "mem_cycle", 128);
  SyscMemoryCycleTB<E> tb("mem_cycle_tb");

  dut->clk(clk);
  dut->rst_n(rst_n);
  dut->req(req);
  dut->we(we);
  dut->addr(addr);
  dut->wdata(wdata);
  dut->ready(ready);
  dut->rvalid(rvalid);
  dut->rdata(rdata);

  tb.clk(clk);
  tb.rst_n(rst_n);
  tb.req(req);
  tb.we(we);
  tb.addr(addr);
  tb.wdata(wdata);
  tb.ready(ready);
  tb.rvalid(rvalid);
  tb.rdata(rdata);

  sc_core::sc_start();

  if (!ctx.arg.text_output.empty()) {
    dut->save_text(ctx, ctx.arg.text_output, ctx.arg.oformat_hex);
  }

  if (ctx.arg.verbose) {
    Out(ctx) << "SyscMemoryCycle testbench pass=" << (tb.pass ? "true" : "false") << "\n";
  }
  return tb.pass;
}

template class SyscMemoryCycle<E>;

}  // namespace adptsysc