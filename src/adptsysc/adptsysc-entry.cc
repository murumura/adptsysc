#include <iostream>
#include <systemc>
#include <tlm>
#include <tlm_utils/simple_target_socket.h>
#include <adptsysc/adptsysc.hh>
#include <adptsysc/config.hh>
using namespace sc_core;
using namespace sc_dt;
using namespace std;

int sc_main(int argc, char* argv[]) {
 
  return adptsysc::adptsysc_main<adptsysc::ADPT_FIRST_TARGET>(argc, argv);
}
