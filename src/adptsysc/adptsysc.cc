#include <adptsysc/adptsysc.hh>
#include <systemc-ams.h>

SCA_TDF_MODULE(HelloSystemCAMS) {
  sca_tdf::sca_out<double> out;

  // Add sample period parameter
  sca_core::sca_time sample_period;

  // Modified constructor to accept sample period
  SCA_CTOR(HelloSystemCAMS) : sample_period(sca_core::sca_time(1.0, sc_core::SC_MS)) {
    // Set the sample period for this module
    set_timestep(sample_period);
  }

  void processing() {
    static int count = 0;
    out.write(1.0);
    std::cout << "Hello SystemC-AMS! (Step " << count++ << ")" << std::endl;
  }
};

SC_MODULE(Top) {
  sca_tdf::sca_signal<double> sig;
  HelloSystemCAMS hello;

  SC_CTOR(Top) : hello("hello") {
    hello.out(sig);
  }
};

int sc_main(int argc, char* argv[]) {
  Top top("top");
  sc_core::sc_start(10, sc_core::SC_SEC);
  return 0;
}
