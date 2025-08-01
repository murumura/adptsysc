#include <systemc>
#include <iostream>
#include <adptsysc/generic-mem.hh>

using namespace sc_core;
using namespace std;

SC_MODULE(Tester) {
    // Interface signals
    sc_out<bool> clk;
    sc_out<bool> reset;
    sc_out<bool> ramen;
    sc_out<bool> writeen;
    sc_out<bool> invalid;
    sc_in<bool> outready;
    
    sc_out<size_t> addr;
    sc_out<int> indata;
    sc_in<int> outdata;

    // Test variables
    int test_data[5] = {10, 20, 30, 40, 50};
    int read_data[5] = {0};

    void test_sequence() {
        // Reset phase
        reset.write(true);
        ramen.write(false);
        writeen.write(false);
        invalid.write(false);
        wait(3, SC_NS);
        reset.write(false);
        wait(1, SC_NS);

        // Write sequence
        cout << "Starting write operations..." << endl;
        for (int i = 0; i < 5; i++) {
            ramen.write(true);
            writeen.write(true);
            invalid.write(true);
            addr.write(i);
            indata.write(test_data[i]);
            wait(1, SC_NS);
            
            while (!outready.read()) {
                wait(1, SC_NS);
            }
            
            cout << "Write @" << i << " = " << test_data[i] << endl;
            wait(clk.posedge_event());
        }
        ramen.write(false);
        writeen.write(false);
        invalid.write(false);
        wait(10, SC_NS);

        // Read sequence
        cout << "\nStarting read operations..." << endl;
        for (int i = 0; i < 5; i++) {
            ramen.write(true);
            writeen.write(false);
            invalid.write(true);
            addr.write(i);
            wait(1, SC_NS);
            
            while (!outready.read()) {
                wait(1, SC_NS);
            }
            
            read_data[i] = outdata.read();
            cout << "Read @" << i << " = " << read_data[i] << endl;
            
            // Verify data
            if (read_data[i] != test_data[i]) {
                cerr << "ERROR: Data mismatch at address " << i << endl;
            }
            wait(clk.posedge_event());
        }
        ramen.write(false);
        invalid.write(false);

        // End simulation
        wait(10, SC_NS);
        sc_stop();
    }

    void clock_gen() {
        while (true) {
            clk.write(false);
            wait(5, SC_NS);
            clk.write(true);
            wait(5, SC_NS);
        }
    }

    SC_CTOR(Tester) {
        SC_THREAD(clock_gen);
        SC_THREAD(test_sequence);
    }
};

int sc_main(int argc, char* argv[]) {
    // Create instances
    Memory<int> memory("memory", 1024);  // 1KB memory
    Tester tester("tester");

    // Signal declarations
    sc_clock clk_sig;
    sc_signal<bool> reset_sig;
    sc_signal<bool> ramen_sig;
    sc_signal<bool> writeen_sig;
    sc_signal<bool> invalid_sig;
    sc_signal<bool> outready_sig;
    sc_signal<size_t> addr_sig;
    sc_signal<int> indata_sig;
    sc_signal<int> outdata_sig;

    // Connections
    memory.clk(clk_sig);
    memory.reset(reset_sig);
    memory.ramen(ramen_sig);
    memory.writeen(writeen_sig);
    memory.invalid(invalid_sig);
    memory.outready(outready_sig);
    memory.addr(addr_sig);
    memory.indata(indata_sig);
    memory.outdata(outdata_sig);

    tester.clk(clk_sig);
    tester.reset(reset_sig);
    tester.ramen(ramen_sig);
    tester.writeen(writeen_sig);
    tester.invalid(invalid_sig);
    tester.outready(outready_sig);
    tester.addr(addr_sig);
    tester.indata(indata_sig);
    tester.outdata(outdata_sig);

    // Trace file
    sc_trace_file* tf = sc_create_vcd_trace_file("memory_trace");
    sc_trace(tf, clk_sig, "clk");
    sc_trace(tf, reset_sig, "reset");
    sc_trace(tf, ramen_sig, "ramen");
    sc_trace(tf, writeen_sig, "writeen");
    sc_trace(tf, invalid_sig, "invalid");
    sc_trace(tf, outready_sig, "outready");
    sc_trace(tf, addr_sig, "addr");
    sc_trace(tf, indata_sig, "indata");
    sc_trace(tf, outdata_sig, "outdata");

    // Start simulation
    cout << "Starting simulation..." << endl;
    sc_start(200, SC_NS);
    sc_close_vcd_trace_file(tf);

    cout << "Simulation completed." << endl;
    return 0;
}