#include <systemc>
#include <tlm>
#include <tlm_utils/simple_initiator_socket.h>
#include <tlm_utils/simple_target_socket.h>
#include <iostream>
#include <adptsysc/generic-mem.hh>

using namespace sc_core;
using namespace sc_dt;
using namespace std;

SC_MODULE(TesterRTL) {
    sc_in<bool> clk;
    sc_out<bool> reset, ramen, writeen, invalid;
    sc_out<size_t> addr;
    sc_out<int> indata;
    sc_in<int> outdata;
    sc_in<bool> outready;
    sc_out<bool> rtl_done;

    int test_data[5] = {10, 20, 30, 40, 50};
    int read_data[5] = {0};

    void run() {
        reset.write(true);
        ramen.write(false);
        writeen.write(false);
        invalid.write(false);
        rtl_done.write(false);
        wait(20, SC_NS);
        reset.write(false);
        wait(10, SC_NS);

        // Write
        cout << "[RTL] Starting write operations..." << endl;
        for (int i = 0; i < 5; i++) {
            ramen.write(true);
            writeen.write(true);
            invalid.write(true);
            addr.write(i);
            indata.write(test_data[i]);
            cout << "[RTL] Write signal..." << test_data[i] << endl;
            wait(clk.posedge_event());
            wait(1, SC_NS); // Let memory commit write
        }
        ramen.write(false);
        writeen.write(false);
        invalid.write(false);
        wait(50, SC_NS);

        // Read
        // Read
        cout << "[RTL] Starting read operations..." << endl;
        for (int i = 0; i < 5; i++) {
            ramen.write(true);
            writeen.write(false);
            invalid.write(true);
            addr.write(i);
            wait(clk.posedge_event());  // Wait for address to be captured
            wait(1, SC_NS);             // Let memory process the address

            while (!outready.read())
                wait(clk.posedge_event());

            read_data[i] = outdata.read();
            cout << "[RTL] Read @" << i << " = " << read_data[i] << endl;

            if (read_data[i] != test_data[i]) {
                SC_REPORT_WARNING("TesterRTL", "Data mismatch");
                cout << "[RTL] Read @" << i << " = " << read_data[i] << "V.S test_data[" << i << "]=" << test_data[i] << endl;
            }

            wait(clk.posedge_event());
            ramen.write(false);
            invalid.write(false);
            wait(clk.posedge_event());  // Additional wait to ensure clean state
        }
        wait(clk.posedge_event());
        ramen.write(false);
        invalid.write(false);
        wait(clk.posedge_event());  // Additional wait to ensure clean state
        rtl_done.write(true);
        cout << "[RTL] Test sequence completed" << endl;
    }

    SC_CTOR(TesterRTL) {
        SC_THREAD(run);
        sensitive << clk.pos();
    }
};

SC_MODULE(TLMInitiator) {
    tlm_utils::simple_initiator_socket<TLMInitiator> initiator_socket;
    sc_in<bool> rtl_done;

    void run() {
        cout << "[TLM] Waiting for rtl_done..." << endl;
        wait(rtl_done.posedge_event());
        wait(50, SC_NS);

        cout << "[TLM] Starting TLM transactions..." << endl;

        for (int i = 0; i < 5; ++i) {
            int value = 100 + i;
            tlm::tlm_generic_payload trans;
            sc_time delay = SC_ZERO_TIME;

            // Write
            trans.set_command(tlm::TLM_WRITE_COMMAND);
            trans.set_address(i * sizeof(int));
            trans.set_data_ptr(reinterpret_cast<unsigned char*>(&value));
            trans.set_data_length(sizeof(int));
            trans.set_response_status(tlm::TLM_INCOMPLETE_RESPONSE);
            initiator_socket->b_transport(trans, delay);
            wait(10, SC_NS);

            if (trans.is_response_error())
                SC_REPORT_ERROR("TLMInitiator", "Write failed");

            // Read
            int readval = 0;
            trans.set_command(tlm::TLM_READ_COMMAND);
            trans.set_data_ptr(reinterpret_cast<unsigned char*>(&readval));
            initiator_socket->b_transport(trans, delay);
            wait(10, SC_NS);

            if (trans.is_response_error())
                SC_REPORT_ERROR("TLMInitiator", "Read failed");

            cout << "[TLM] Read @" << i << " = " << readval << endl;
        }

        wait(100, SC_NS);
        sc_stop();
    }

    SC_CTOR(TLMInitiator) {
        SC_THREAD(run);
    }
};

int sc_main(int argc, char* argv[]) {
    using namespace adptsysc;

    sc_clock clk("clk", 10, SC_NS);

    sc_signal<bool> reset, ramen, writeen, invalid, outready, rtl_done;
    sc_signal<size_t> addr;
    sc_signal<int> indata, outdata;

    // Memory instance
    Memory<int> memory("memory", 1024);
    memory.clk(clk);
    memory.reset(reset);
    memory.ramen(ramen);
    memory.writeen(writeen);
    memory.invalid(invalid);
    memory.outready(outready);
    memory.addr(addr);
    memory.indata(indata);
    memory.outdata(outdata);

    // RTL tester
    TesterRTL rtl("rtl");
    rtl.clk(clk);
    rtl.reset(reset);
    rtl.ramen(ramen);
    rtl.writeen(writeen);
    rtl.invalid(invalid);
    rtl.addr(addr);
    rtl.indata(indata);
    rtl.outdata(outdata);
    rtl.outready(outready);
    rtl.rtl_done(rtl_done);

    // TLM tester
    TLMInitiator tlm("tlm");
    tlm.initiator_socket.bind(memory.targ_socket);
    tlm.rtl_done(rtl_done);

    // Trace
    sc_trace_file* tf = sc_create_vcd_trace_file("memory_trace");
    sc_trace(tf, clk, "clk");
    sc_trace(tf, reset, "reset");
    sc_trace(tf, ramen, "ramen");
    sc_trace(tf, writeen, "writeen");
    sc_trace(tf, invalid, "invalid");
    sc_trace(tf, outready, "outready");
    sc_trace(tf, addr, "addr");
    sc_trace(tf, indata, "indata");
    sc_trace(tf, outdata, "outdata");
    sc_trace(tf, rtl_done, "rtl_done");

    cout << "Starting simulation..." << endl;
    sc_start();
    sc_close_vcd_trace_file(tf);
    cout << "Simulation completed." << endl;

    return 0;
}
