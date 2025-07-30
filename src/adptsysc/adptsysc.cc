#include <adptsysc/adptsysc.hh>
#include <array>
#include <bit>
#include <cassert>
#include <concepts>
#include <cstdint>
#include <iostream>
#include <sysc/datatypes/fx/sc_fixed.h>
#include <systemc>

using namespace sc_core;
using namespace sc_dt;

// ======================== Fixed-Point Support ============================ //
template <int W, int F>
using fixed_t = sc_fixed<W, W - F>;

// ======================== FIR Filter Module ============================= //
template <typename T, size_t Order>
SC_MODULE(fir) {
  sc_in<bool> rst;
  sc_in<bool> clk;
  sc_in<T> in;
  sc_out<T> out;

  std::array<T, Order> coeffs;
  std::array<T, Order> delay{{}};

  SC_HAS_PROCESS(fir);
  fir(sc_module_name name, const std::array<T, Order>& h) : sc_module(name), coeffs(h) {
    SC_METHOD(process);
    sensitive << clk.pos();
    dont_initialize();
  }

  void process() {
    if (rst.read()) {
      delay.fill(0);
      out.write(0);
      return;
    }

    T acc = coeffs[0] * in.read();
    delay[0] = in.read();

    for (size_t i = 1; i < Order; ++i) {
      delay[i] = delay[i - 1];
      acc += coeffs[i] * delay[i - 1];
    }

    out.write(acc);
  }
};

// ===================== Polyphase Matrix Generator ====================== //
template <typename T, size_t Rows, size_t Cols>
std::array<std::array<T, Cols>, Rows> generate_polyphase_matrix(
    const std::array<T, Rows * Cols>& h) {
  std::array<std::array<T, Cols>, Rows> matrix{};
  for (size_t i = 0; i < Rows; ++i)
    for (size_t j = 0; j < Cols; ++j)
      matrix[i][j] = h[j * Rows + i];
  return matrix;
}

// =================== Polyphase Decimation Filter ======================= //
template <typename T, size_t N, size_t M>
class ppd : public sc_module {
 public:
  static constexpr size_t P = (N + M - 1) / M;

  sc_in<bool> clk;
  sc_in<bool> rst;
  sc_in<T> in;
  sc_out<T> out;

  std::array<sc_signal<T>, M> delay;
  std::array<sc_signal<T>, M> outputs;
  std::array<fir<T, P>*, M> filters;

  SC_HAS_PROCESS(ppd);

  ppd(sc_module_name name, const std::array<T, N>& h) : sc_module(name) {
    auto matrix = generate_polyphase_matrix<T, M, P>(h);

    for (size_t i = 0; i < M; ++i) {
      filters[i] = new fir<T, P>(sc_gen_unique_name("E"), matrix[i]);
      filters[i]->clk(clk);
      filters[i]->rst(rst);
      filters[i]->in(delay[i]);
      filters[i]->out(outputs[i]);
    }

    SC_METHOD(run);
    sensitive << clk.pos();
  }

  void run() {
    if (rst.read()) {
      for (auto& d : delay)
        d.write(0);
      out.write(0);
      return;
    }

    delay[0] = in.read();
    for (size_t i = 1; i < M; ++i)
      delay[i] = delay[i - 1];

    T sum = 0;
    for (size_t i = 0; i < M; ++i)
      sum += outputs[i].read();

    out.write(sum);
  }

  ~ppd() {
    for (auto* f : filters)
      delete f;
  }
};

// ============================ Example Top =============================== //
SC_MODULE(top) {
  static constexpr int N = 36;
  static constexpr int M = 3;
  using fxp_t = fixed_t<16, 12>;

  sc_clock clk{"clk", 10, SC_NS};
  sc_signal<bool> rst;
  sc_signal<fxp_t> sig_in, sig_out;

  std::array<fxp_t, N> coeffs = {0.0017, 0.0073, 0.0107, 0.0151, 0.0162, 0.0128, 0.0039,
      -0.0093, -0.0232, -0.0329, -0.0326, -0.0182, 0.0115, 0.0536, 0.1013, 0.1454, 0.1765,
      0.1877, 0.1765, 0.1454, 0.1013, 0.0536, 0.0115, -0.0182, -0.0326, -0.0329, -0.0232,
      -0.0093, 0.0039, 0.0128, 0.0162, 0.0151, 0.0107, 0.0073, 0.0017, 0.0};

  ppd<fxp_t, N, M> dut{"dut", coeffs};

  SC_CTOR(top) : dut("dut", coeffs) {
    dut.clk(clk);
    dut.rst(rst);
    dut.in(sig_in);
    dut.out(sig_out);

    SC_THREAD(test);
  }

  void test() {
    rst.write(true);
    wait(20, SC_NS);
    rst.write(false);

    for (int i = 0; i < 100; ++i) {
      sig_in.write((i % 2) ? 1.0 : -1.0);
      wait(10, SC_NS);
    }

    sc_stop();
  }
};

int sc_main(int argc, char* argv[]) {
  top t("t");
  sc_trace_file* tf = sc_create_vcd_trace_file("wave");
  tf->set_time_unit(1, SC_NS);

  // Trace primary signals
  sc_trace(tf, t.clk, "clk");
  sc_trace(tf, t.rst, "rst");
  sc_trace(tf, t.sig_in, "sig_in");
  sc_trace(tf, t.sig_out, "sig_out");

  // Optional: trace internal delays or outputs
  for (int i = 0; i < 3; ++i) {
    sc_trace(tf, t.dut.delay[i], ("delay" + std::to_string(i)).c_str());
    sc_trace(tf, t.dut.outputs[i], ("sum" + std::to_string(i)).c_str());
  }
  sc_close_vcd_trace_file(tf);

  return 0;
}
