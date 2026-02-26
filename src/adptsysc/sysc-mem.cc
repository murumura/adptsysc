#include <adptsysc/config.hh>
#include <adptsysc/integers.hh>
#include <stdint.h>
#include <tlm>
#include <tlm_utils/simple_target_socket.h>
#include <tlm_utils/simple_initiator_socket.h>
#include <adptsysc/adptsysc.hh>
#include <memory>
#include <adptsysc/sysc-mem.hh>

namespace adptsysc {

namespace fs = std::filesystem;

using E = ADPT_TARGET;

template <typename E>
class SyscMemoryRtlTester : public sc_core::sc_module {
  using T = typename E::Eval_T;

public:
  sc_core::sc_in<bool> clk;

  sc_core::sc_out<bool> reset, ram_en, write_en, in_valid;
  sc_core::sc_out<std::size_t> addr;
  sc_core::sc_out<T> in_data;

  sc_core::sc_in<T> out_data;
  sc_core::sc_in<bool> out_ready;

  sc_core::sc_out<bool> rtl_done;

  std::vector<T> test_data;
  std::vector<T> read_data;

  SyscMemoryRtlTester(sc_core::sc_module_name nm) : sc_module(nm) {
    test_data = {10,20,30,40,50};
    read_data.resize(test_data.size());
    SC_THREAD(run);
  }

  void run() {

    // -------------------------
    // INITIAL RESET
    // -------------------------
    reset.write(true);
    ram_en.write(false);
    write_en.write(false);
    in_valid.write(false);
    rtl_done.write(false);
    wait(5, sc_core::SC_NS);

    reset.write(false);
    wait(5, sc_core::SC_NS);

    // ===========================================
    // WRITE PHASE
    // ===========================================
    for (size_t i = 0; i < test_data.size(); i++) {

      ram_en.write(true);
      write_en.write(true);
      in_valid.write(true);
      addr.write(i);
      in_data.write(test_data[i]);

      wait(clk.posedge_event());
    }

    // de-assert
    ram_en.write(false);
    write_en.write(false);
    in_valid.write(false);
    wait(clk.posedge_event());


    // ===========================================
    // READ PHASE  (with variable read latency)
    // ===========================================
    for (size_t i = 0; i < test_data.size(); i++) {

      ram_en.write(true);
      write_en.write(false);
      in_valid.write(true);
      addr.write(i);

      wait(clk.posedge_event());  // issue read

      // Wait for pipeline output
      while (!out_ready.read())
        wait(clk.posedge_event());

      read_data[i] = out_data.read();

      if (read_data[i] != test_data[i])
        SC_REPORT_WARNING("RTL", "Read mismatch");

      // cleanup
      ram_en.write(false);
      in_valid.write(false);
      wait(clk.posedge_event());
    }

    rtl_done.write(true);
  }
};


template <typename E>
class SyscMemoryTLMInitiator : public sc_core::sc_module {
public:
  using T = typename E::Eval_T;

  tlm_utils::simple_initiator_socket<SyscMemoryTLMInitiator> init_socket;

  sc_core::sc_in<bool> rtl_done;
  sc_core::sc_out<bool> tlm_done;

  SyscMemoryTLMInitiator(sc_core::sc_module_name nm) : sc_module(nm) {
    SC_THREAD(run);
  }

  void run() {
    tlm_done.write(false);

    // wait until RTL tester fully finished
    wait(rtl_done.posedge_event());


    for (int i = 0; i < 5; i++) {
      T writeval = 100 + i;
      T readval  = 0;

      tlm::tlm_generic_payload tr;
      sc_core::sc_time delay = sc_core::SC_ZERO_TIME;

      // ---------------------
      // WRITE
      // ---------------------
      tr.set_command(tlm::TLM_WRITE_COMMAND);
      tr.set_address(i * sizeof(T));
      tr.set_data_ptr(reinterpret_cast<unsigned char*>(&writeval));
      tr.set_data_length(sizeof(T));
      tr.set_response_status(tlm::TLM_INCOMPLETE_RESPONSE);

      init_socket->b_transport(tr, delay);

      if (tr.is_response_error())
        SC_REPORT_ERROR("TLM", "Write failed");


      // ---------------------
      // READ
      // ---------------------
      tr.set_command(tlm::TLM_READ_COMMAND);
      tr.set_data_ptr(reinterpret_cast<unsigned char*>(&readval));
      tr.set_data_length(sizeof(T));
      tr.set_response_status(tlm::TLM_INCOMPLETE_RESPONSE);

      init_socket->b_transport(tr, delay);

      if (tr.is_response_error())
        SC_REPORT_ERROR("TLM", "Read failed");

      if (readval != writeval)
        SC_REPORT_WARNING("TLM", "TLM read mismatch");
    }

    tlm_done.write(true);
  }
};


template <typename E>
class TestbenchController : public sc_core::sc_module {
public:
  sc_core::sc_in<bool> rtl_done;
  sc_core::sc_in<bool> tlm_done;

  TestbenchController(sc_core::sc_module_name name) : sc_module(name) {
    SC_THREAD(run);
  }

  void run() {
    // Wait for both RTL and TLM to finish
    wait(rtl_done->posedge_event());
    wait(tlm_done->posedge_event());

    std::cout << sc_core::sc_time_stamp() 
              << " Stopping simulation\n";

    wait(20, sc_core::SC_NS);
    sc_core::sc_stop();
  }
};

template <typename E>
std::unique_ptr<SyscMemory<E>> 
SyscMemory<E>::create(Context<E> &ctx, 
                      sc_core::sc_module_name name,
                      std::size_t size, T* initptr) {
  return std::unique_ptr<SyscMemory<E>>(new SyscMemory<E>(name, ctx, size, initptr));
}

template <typename E>
SyscMemory<E>::SyscMemory(sc_core::sc_module_name name, 
                          Context<E> &ctx, std::size_t size,  T* initptr) 
                          : sc_module(name), targ_socket("targ_socket")
                          , mem_size(size), is_init(false), read_latency(0), write_latency(0) {
  
  targ_socket.register_b_transport(this, &SyscMemory::b_transport);
  targ_socket.register_get_direct_mem_ptr(this, &SyscMemory::get_direct_mem_ptr);
  targ_socket.register_transport_dbg(this, &SyscMemory::transport_dbg);
  
  // Configure latencies if this arch supports it
  if constexpr (support_rwlat<E>) {
    read_latency  = std::max(0, ctx.arg.mem_read_lat);
    write_latency = std::max(0, ctx.arg.mem_write_lat);
  } else {
    read_latency  = 0;
    write_latency = 0;
  }

  // RTL process registration
  SC_METHOD(memory_read);
  sensitive << clk.pos();
  dont_initialize();

  SC_METHOD(memory_write);
  sensitive << clk.pos();
  dont_initialize();

  if (size > 0) {
    // Use the allocate method to properly initialize
    allocate(size);
    if (initptr) {
      std::copy(initptr, initptr + size, mem_data.get());
    }
  }
}

template <typename E>
bool SyscMemory<E>::run_testbench(Context<E> &ctx) {
  using T = typename E::Eval_T;

  // Clock
  sc_core::sc_clock clk("clk", 10, sc_core::SC_NS);

  // Signals
  sc_core::sc_signal<bool> reset, ram_en, write_en, in_valid;
  sc_core::sc_signal<bool> out_ready;
  sc_core::sc_signal<bool> rtl_done, tlm_done;
  sc_core::sc_signal<std::size_t> addr;
  sc_core::sc_signal<T> in_data, out_data;

  // Memory under test
  auto mem = SyscMemory<E>::create(ctx, "mem", 128);
  mem->clk(clk);
  mem->reset(reset);
  mem->ram_en(ram_en);
  mem->write_en(write_en);
  mem->in_valid(in_valid);
  mem->out_ready(out_ready);
  mem->addr(addr);
  mem->in_data(in_data);
  mem->out_data(out_data);

  // RTL tester
  SyscMemoryRtlTester<E> rtl("rtl");
  rtl.clk(clk);
  rtl.reset(reset);
  rtl.ram_en(ram_en);
  rtl.write_en(write_en);
  rtl.in_valid(in_valid);
  rtl.addr(addr);
  rtl.in_data(in_data);
  rtl.out_data(out_data);
  rtl.out_ready(out_ready);
  rtl.rtl_done(rtl_done);

  // TLM tester
  SyscMemoryTLMInitiator<E> tlm("tlm");
  tlm.init_socket.bind(mem->targ_socket);
  tlm.rtl_done(rtl_done);
  tlm.tlm_done(tlm_done);

  // Controller
  TestbenchController<E> tb("tb");
  tb.rtl_done(rtl_done);
  tb.tlm_done(tlm_done);

  // Start simulation
  sc_core::sc_start();  // stops only when tb calls sc_stop()

  return true;
}

template <typename E>
void SyscMemory<E>::allocate(std::size_t size) {
  if (size == 0) {
    SC_REPORT_ERROR(name(), "Cannot allocate memory of size 0");
    return;
  }

  if (is_init && size == mem_size)
    return;

  mem_data = std::make_unique<T[]>(size);
  mem_size = size;
  is_init = true;
}

template <typename E>
void SyscMemory<E>::allocate(const std::shared_ptr<ParametricObject<T>>& target) {
  allocate(static_cast<std::size_t>(target->n_params()));
}

template <typename E>
void SyscMemory<E>::mem_reset(Context<E> &ctx) {
  if (is_init) {
    std::fill(mem_data.get(), mem_data.get() + mem_size, T{});
    if (ctx.arg.verbose) {
      Out(ctx) <<  "Reset memory: " << name();
    }
  }
}

template <typename E>
json SyscMemory<E>::serialize(Context<E> &ctx) const {
  if (ctx.arg.verbose) {
    Out(ctx) << "Serializing memory: " << name();
  }
  
  return {
    {"size", mem_size},
    {"data", std::vector<T>(mem_data.get(), mem_data.get() + mem_size)}
  };
}

template <typename E>
void SyscMemory<E>::deserialize(Context<E> &ctx, const json& data) {
  if (data.contains("size") && data.contains("data")) {
    auto new_size = data["size"].get<std::size_t>();
    auto vec = data["data"].get<std::vector<T>>();

    if (new_size != vec.size()) {
      throw std::runtime_error("Size mismatch in deserialization");
    }

    allocate(new_size);
    std::copy(vec.begin(), vec.end(), mem_data.get());
    
    if (ctx.arg.verbose) {
      Out(ctx) << "Deserialized memory: " << name() << " size=" << new_size;
    }
  }
}

template <typename E>
void SyscMemory<E>::load_from_file(Context<E> &ctx) {
  // Use memory-mapped file for efficient loading
  MappedFile *mf = open_file(ctx, ctx.arg.load_file);
  if (!mf) {
    Fatal(ctx) << "Cannot open file for memory load: " << ctx.arg.load_file;
  }
  int offset = ctx.arg.load_offset < 0 ? 0: ctx.arg.load_offset;
  if (offset + mem_size * sizeof(T) > mf->size) {
    Fatal(ctx) << "File too small for memory load";
  }

  // Copy data from memory-mapped file
  std::memcpy(mem_data.get(), mf->data + offset, mem_size * sizeof(T));
  
  if (ctx.arg.verbose) {
    Out(ctx) <<  "Loaded " << mem_size  << " elements from " << ctx.arg.load_file;
  }
}

template <typename E>
void SyscMemory<E>::save_to_file(Context<E> &ctx) {
  // Use OutputFile for efficient saving
  i64 file_size = mem_size * sizeof(T);
  ctx.output_file = OutputFile<E>::open(ctx, ctx.arg.output, file_size, 0777);
  if (!ctx.output_file || !ctx.output_file->buf) {
    Fatal(ctx) << "Cannot open file for memory file saving: " << ctx.output_file;
  }

  // Copy memory content to output buffer
  std::memcpy(ctx.output_file->buf, mem_data.get(), file_size);
  ctx.buf = ctx.output_file->buf;
  // Add additional metadata if needed
  if (ctx.arg.verbose) {
    std::string metadata = "Memory dump: " + std::to_string(mem_size) + " elements\n";
    ctx.output_file->buf2.assign(metadata.begin(), metadata.end());
  }
  
  ctx.output_file->close(ctx);
  
  if (ctx.arg.verbose) {
    Out(ctx) << "Saved " << mem_size  << " elements to " << ctx.arg.output;
  }
}


template <typename E>
void SyscMemory<E>::save_to_text_file(Context<E> &ctx) {
  std::ofstream ofs(ctx.arg.text_output);
  if (!ofs) {
    Fatal(ctx) << "Cannot open file for memory text saving: " << ctx.arg.text_output;
  }

  if (ctx.arg.oformat_hex) {
    ofs << std::hex << std::setfill('0');
    for (std::size_t i = 0; i < mem_size; ++i) {
      if constexpr (sizeof(T) > 1) {
        ofs << std::setw(sizeof(T)*2);
      }
      ofs << static_cast<uint64_t>(mem_data[i]) << '\n';
    }
  } else {
    for (std::size_t i = 0; i < mem_size; ++i) {
      ofs << mem_data[i] << '\n';
    }
  }
  
  if (ctx.arg.verbose) {
    Out(ctx) << "Saved " << mem_size  << " elements as text to " << ctx.arg.text_output;
  }
}

template <typename E>
void SyscMemory<E>::load_from_text_file(Context<E> &ctx) {
  std::ifstream ifs(ctx.arg.text_loadfile);
  if (!ifs) {
    Fatal(ctx) << "Cannot open file for memory text loading: " << ctx.arg.text_loadfile;
  }

  std::size_t i = 0;
  std::string line;
  while (i < mem_size && std::getline(ifs, line)) {
    if constexpr (std::is_integral_v<T>) {
      mem_data[i++] = static_cast<T>(std::stoull(line, nullptr, 0));
    } else {
      mem_data[i++] = static_cast<T>(std::stod(line));
    }
  }
  
  if (ctx.arg.verbose) {
    if (i != mem_size) {
      Out(ctx) << "Warning: Only loaded " << i << " elements from text file";
    } else {
      Out(ctx) << "Loaded"  << mem_size << " elements from text file " << ctx.arg.text_loadfile;
    }
  }
}

template <typename E>
json SyscMemory<E>::hyperparams() const {
  return {
    {"otype", "Memory"}, 
    {"size", mem_size},
    {"data_type", typeid(T).name()}
  };
}

  // Mutable configuration
template <typename E>
void SyscMemory<E>::update_hyperparams(const json& params) {
  if (params.contains("size")) {
    allocate(params["size"].get<std::size_t>());
  }
}

template <typename E>
void SyscMemory<E>::dump_memory(Context<E> &ctx, const std::string& desc) const {
  if (!ctx.arg.verbose) return;
  
  Out(ctx) << "Memory Dump " << (desc.empty() ? "" : "(" + desc + ")") 
           << ": size=" << mem_size << "\n";
  
  std::size_t limit = std::min(mem_size, static_cast<std::size_t>(16));
  for (std::size_t i = 0; i < limit; ++i) {
    Out(ctx) << " i= [" << i << "] = " << mem_data[i];
    if constexpr (std::is_integral_v<T>) {
      Out(ctx) << " (0x" << std::hex << static_cast<uint64_t>(mem_data[i]) << std::dec << ")";
    }
  }
  
  if (mem_size > limit) {
    Out(ctx) << "  ... and " << (mem_size - limit) << " more elements";
  }
}

template <typename E>
void SyscMemory<E>::b_transport(tlm::tlm_generic_payload& trans, sc_core::sc_time& delay) {
  auto cmd = trans.get_command();
  std::size_t addr = static_cast<std::size_t>(trans.get_address());
  std::size_t len = static_cast<std::size_t>(trans.get_data_length());
  u8* ptr = trans.get_data_ptr();

  if (addr + len > mem_size * sizeof(T) || addr % sizeof(T) != 0 || len % sizeof(T) != 0) {
    trans.set_response_status(tlm::TLM_ADDRESS_ERROR_RESPONSE);
    return;
  }

  T* base = reinterpret_cast<T*>(reinterpret_cast<u8*>(mem_data.get()) + addr);

  if (cmd == tlm::TLM_READ_COMMAND) {
    std::memcpy(ptr, base, len);
  } else if (cmd == tlm::TLM_WRITE_COMMAND) {
    std::memcpy(base, ptr, len);
  }

  trans.set_response_status(tlm::TLM_OK_RESPONSE);
}

template <typename E>
bool SyscMemory<E>::get_direct_mem_ptr(tlm::tlm_generic_payload& trans, tlm::tlm_dmi& dmi_data) {
  std::size_t addr = static_cast<std::size_t>(trans.get_address());
  if (addr % sizeof(T) != 0) {
      SC_REPORT_ERROR(name(), "DMI request with unaligned address");
      return false;
  }

  if (addr >= mem_size * sizeof(T)) {
    std::ostringstream msg;
    msg << "DMI request out of range (addr=0x" << std::hex << addr 
        << ", max=0x" << (mem_size * sizeof(T) - 1) << ")";
    SC_REPORT_WARNING(name(), msg.str().c_str());
    return false;
  }

  dmi_data.set_dmi_ptr(reinterpret_cast<u8*>(mem_data.get()));
  dmi_data.set_start_address(0);
  dmi_data.set_end_address(mem_size * sizeof(T) - 1);
  dmi_data.allow_read_write();
  dmi_data.set_read_latency(sc_core::sc_time(10, sc_core::SC_NS));
  dmi_data.set_write_latency(sc_core::sc_time(10, sc_core::SC_NS));

  return true;
}

template <typename E>
unsigned int SyscMemory<E>::transport_dbg(tlm::tlm_generic_payload& trans) {
  tlm::tlm_command cmd = trans.get_command();
  if (cmd != tlm::TLM_READ_COMMAND && cmd != tlm::TLM_WRITE_COMMAND) {
    trans.set_response_status(tlm::TLM_COMMAND_ERROR_RESPONSE);
    return 0;
  }

  std::size_t addr = static_cast<std::size_t>(trans.get_address());
  u8* ptr = trans.get_data_ptr();
  std::size_t len = trans.get_data_length();
  
  if (addr >= mem_size * sizeof(T)) {
    trans.set_response_status(tlm::TLM_ADDRESS_ERROR_RESPONSE);
    return 0;
  }

  std::size_t remains = static_cast<std::size_t>(mem_size * sizeof(T) - addr);
  std::size_t nbytes = (len < remains) ? len : remains;

  if (nbytes == 0) {
    trans.set_response_status(tlm::TLM_ADDRESS_ERROR_RESPONSE);
    return 0;
  }

  if (cmd == tlm::TLM_READ_COMMAND) {
    std::memcpy(ptr, reinterpret_cast<u8*>(mem_data.get()) + addr, nbytes);
  } else {
    std::memcpy(reinterpret_cast<u8*>(mem_data.get()) + addr, ptr, nbytes);
  }

  trans.set_response_status(tlm::TLM_OK_RESPONSE);
  return nbytes;
}

template <typename E>
void SyscMemory<E>::memory_write() {
  if (reset.read()) {
    write_q.clear();
    return;
  }

  // advance write pipeline & commit any ready writes
  if (!write_q.empty()) {
    for (auto &w : write_q) {
      if (w.cycles_left > 0)
        --w.cycles_left;
    }

    while (!write_q.empty() && write_q.front().cycles_left <= 0) {
      auto w = write_q.front();
      write_q.pop_front();

      if (is_valid_addr(w.addr)) {
        mem_data[w.addr] = w.data;
      } else {
        SC_REPORT_WARNING(name(), "Pipelined write addr out of bounds");
      }
    }
  }

  // capture new write request at the *end* of the cycle
  if (ram_en.read() && write_en.read() && in_valid.read()) {
    std::size_t waddr = addr.read();
    T wdata = in_data.read();

    if (!is_valid_addr(waddr)) {
      SC_REPORT_WARNING(name(), "Write addr out of bounds");
      return;
    }

    if (write_latency <= 0) {
      // Immediate write
      mem_data[waddr] = wdata;
    } else {
      write_q.push_back(WriteReq{waddr, wdata, write_latency});
    }
  }
}

template <typename E>
void SyscMemory<E>::memory_read() {
  if (reset.read()) {
    read_q.clear();
    out_ready.write(false);
    return;
  }

  bool produced = false;

  // advance read pipeline and possibly produce data
  if (!read_q.empty()) {
    for (auto &r : read_q) {
      if (r.cycles_left > 0)
        --r.cycles_left;
    }

    if (!read_q.empty() && read_q.front().cycles_left <= 0) {
      auto r = read_q.front();
      read_q.pop_front();

      if (is_valid_addr(r.addr)) {
        out_data.write(mem_data[r.addr]);
        produced = true;
      } else {
        SC_REPORT_WARNING(name(), "Pipelined read addr out of bounds");
      }
    }
  }

  // capture new read request at end of cycle
  if (ram_en.read() && !write_en.read() && in_valid.read()) {
    std::size_t raddr = addr.read();

    if (!is_valid_addr(raddr)) {
      SC_REPORT_WARNING(name(), "Read addr out of bounds");
    } else if (read_latency <= 0) {
      // Combinational read
      out_data.write(mem_data[raddr]);
      produced = true;
    } else {
      read_q.push_back(ReadReq{raddr, read_latency});
    }
  }

  out_ready.write(produced);
}


template bool SyscMemory<E>::run_testbench(Context<E>&);

template std::unique_ptr<SyscMemory<E>> 
SyscMemory<E>::create(Context<E>&, sc_core::sc_module_name, std::size_t, typename E::Eval_T*);

template class SyscMemory<E>;

}  // namespace adptsysc
