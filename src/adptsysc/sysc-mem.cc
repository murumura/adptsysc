#include <adptsysc/config.hh>
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
class SyscMemoryTLMInitiator : public sc_core::sc_module {
public:
  using T = typename E::Eval_T;

  tlm_utils::simple_initiator_socket<SyscMemoryTLMInitiator> init_socket;

  SyscMemoryTLMInitiator(sc_core::sc_module_name name)
      : sc_module(name) {
    SC_THREAD(run);
  }

  void run() {

    for (int i = 0; i < 5; i++) {

      T writeval = 100 + i;
      T readval  = 0;

      tlm::tlm_generic_payload tr;

      // ---------------------
      // WRITE
      // ---------------------
      {
        sc_core::sc_time delay = sc_core::SC_ZERO_TIME;

        tr.set_command(tlm::TLM_WRITE_COMMAND);
        tr.set_address(i * sizeof(T));
        tr.set_data_ptr(reinterpret_cast<unsigned char*>(&writeval));
        tr.set_data_length(sizeof(T));
        tr.set_response_status(tlm::TLM_INCOMPLETE_RESPONSE);

        init_socket->b_transport(tr, delay);
        wait(delay);

        if (tr.is_response_error())
          SC_REPORT_ERROR("TLM", "Write failed");
      }

      {
        sc_core::sc_time delay = sc_core::SC_ZERO_TIME;

        tr.set_command(tlm::TLM_READ_COMMAND);
        tr.set_data_ptr(reinterpret_cast<unsigned char*>(&readval));
        tr.set_data_length(sizeof(T));
        tr.set_response_status(tlm::TLM_INCOMPLETE_RESPONSE);

        init_socket->b_transport(tr, delay);
        wait(delay);

        if (tr.is_response_error())
          SC_REPORT_ERROR("TLM", "Read failed");

        if (readval != writeval)
          SC_REPORT_WARNING("TLM", "Mismatch");
      }
    }

    std::cout << sc_core::sc_time_stamp()
              << " Simulation done\n";

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
SyscMemory<E>::SyscMemory(sc_core::sc_module_name name, Context<E> &ctx, 
                          const std::size_t size, T* initptr) 
                          : sc_module(name), targ_socket("targ_socket"), 
                            mem_size(size), is_init(false) {
  targ_socket.register_b_transport(this, &SyscMemory::b_transport);
  targ_socket.register_get_direct_mem_ptr(this, &SyscMemory::get_direct_mem_ptr);
  targ_socket.register_transport_dbg(this, &SyscMemory::transport_dbg);
  
  // Configure latencies if this arch supports it
  if constexpr (support_rdwr_delay<E>) {
    int rddly_cycls = std::max(0, ctx.arg.mem_rddly_cycls);
    int wrdly_cycls = std::max(0, ctx.arg.mem_wrdly_cycls);
    sc_core::sc_time tclk = sc_core::sc_time(10, sc_core::SC_NS);
    this->read_delay  = rddly_cycls * tclk;
    this->write_delay = wrdly_cycls * tclk;
  } else {
    read_delay  = sc_core::SC_ZERO_TIME;
    write_delay = sc_core::SC_ZERO_TIME;
  }
  if (ctx.arg.verbose) {
    Out(ctx) << "Memory " << this->name()
             << " read_delay="  << this->read_delay
             << " write_delay=" << this->write_delay << "\n";
  }

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
  auto mem = SyscMemory<E>::create(ctx, "mem", 128);
  SyscMemoryTLMInitiator<E> tlm("tlm");
  tlm.init_socket.bind(mem->targ_socket);
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
json SyscMemory<E>::get_hyperparams() const {
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
    delay += read_delay;
  } else if (cmd == tlm::TLM_WRITE_COMMAND) {
    std::memcpy(base, ptr, len);
    delay += write_delay;
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

template bool SyscMemory<E>::run_testbench(Context<E>&);

template std::unique_ptr<SyscMemory<E>> 
SyscMemory<E>::create(Context<E>&, sc_core::sc_module_name, std::size_t, typename E::Eval_T*);

template class SyscMemory<E>;

}  // namespace adptsysc
