#ifdef ADPT_ENABLE_SYSC_MEM
#include <adptsysc/config.hh>

#include <algorithm>
#include <cstdint>
#include <cstring>
#include <limits>
#include <sstream>
#include <tlm>
#include <tlm_utils/simple_target_socket.h>
#include <tlm_utils/simple_initiator_socket.h>
#include <adptsysc/adptsysc.hh>
#include <adptsysc/common.hh>
#include <memory>
#include <adptsysc/sysc-mem-tlm.hh>
#include <adptsysc/syscfx-utils.hh>
#include <type_traits>

namespace adptsysc {

namespace fs = std::filesystem;

using E = ADPT_TARGET;

template <typename E>
class SyscMemoryTLMInitiator : public sc_core::sc_module {
public:
  using T = typename E::Eval_T;

  tlm_utils::simple_initiator_socket<SyscMemoryTLMInitiator> init_socket;
  bool pass = true;

  SyscMemoryTLMInitiator(sc_core::sc_module_name name) : sc_module(name) {
    SC_THREAD(run);
  }

  void run() {
    pass = true;

    auto write_trans = [&](std::size_t idx, T value) {
      tlm::tlm_generic_payload tr;
      sc_core::sc_time delay = sc_core::SC_ZERO_TIME;

      tr.set_command(tlm::TLM_WRITE_COMMAND);
      tr.set_address(idx * sizeof(T));
      tr.set_data_ptr(reinterpret_cast<unsigned char*>(&value));
      tr.set_data_length(sizeof(T));
      tr.set_streaming_width(sizeof(T));
      tr.set_byte_enable_ptr(nullptr);
      tr.set_dmi_allowed(false);
      tr.set_response_status(tlm::TLM_INCOMPLETE_RESPONSE);

      init_socket->b_transport(tr, delay);
      wait(delay);

      if (tr.is_response_error()) {
        std::ostringstream oss;
        oss << "Write failed at idx=" << idx;
        SC_REPORT_ERROR("TLM", oss.str().c_str());
        pass = false;
      }
    };

    auto do_read = [&](std::size_t idx) -> T {
      T value = 0;

      tlm::tlm_generic_payload tr;
      sc_core::sc_time delay = sc_core::SC_ZERO_TIME;

      tr.set_command(tlm::TLM_READ_COMMAND);
      tr.set_address(idx * sizeof(T));
      tr.set_data_ptr(reinterpret_cast<unsigned char*>(&value));
      tr.set_data_length(sizeof(T));
      tr.set_streaming_width(sizeof(T));
      tr.set_byte_enable_ptr(nullptr);
      tr.set_dmi_allowed(false);
      tr.set_response_status(tlm::TLM_INCOMPLETE_RESPONSE);

      init_socket->b_transport(tr, delay);
      wait(delay);

      if (tr.is_response_error()) {
        std::ostringstream oss;
        oss << "Read failed at idx=" << idx;
        SC_REPORT_ERROR("TLM", oss.str().c_str());
        pass = false;
      }

      return value;
    };

    auto read_trans = [&](std::size_t idx, T expected) {
      const T got = do_read(idx);

      if (got != expected) {
        std::ostringstream oss;
        oss << "Mismatch at idx=" << idx
            << " got=" << got
            << " expected=" << expected;
        SC_REPORT_WARNING("TLM", oss.str().c_str());
        pass = false;
      }
    };

    // --------------------------------------------------------------------------
    // Test 1: sequential positive writes
    // --------------------------------------------------------------------------
    for (std::size_t i = 0; i < 16; ++i) {
      write_trans(i, static_cast<T>(100 + i));
    }

    for (std::size_t i = 0; i < 16; ++i) {
      read_trans(i, static_cast<T>(100 + i));
    }

    // --------------------------------------------------------------------------
    // Test 2: zero / small positive / negative values
    // --------------------------------------------------------------------------
    write_trans(16, static_cast<T>(0));
    write_trans(17, static_cast<T>(1));
    write_trans(18, static_cast<T>(-1));
    write_trans(19, static_cast<T>(7));
    write_trans(20, static_cast<T>(-7));

    read_trans(16, static_cast<T>(0));
    read_trans(17, static_cast<T>(1));
    read_trans(18, static_cast<T>(-1));
    read_trans(19, static_cast<T>(7));
    read_trans(20, static_cast<T>(-7));

    // --------------------------------------------------------------------------
    // Test 3: boundary write/read
    // --------------------------------------------------------------------------
    const std::size_t last_idx = 127;

    write_trans(last_idx, static_cast<T>(1234));
    read_trans(last_idx, static_cast<T>(1234));

    std::cout << sc_core::sc_time_stamp()
              << " SyscMemory TLM testbench done, pass="
              << (pass ? "true" : "false")
              << "\n";

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
bool SyscMemory<E>::run_testbench(Context<E>& ctx) {
  auto mem = SyscMemory<E>::create(ctx, "mem", 128);

  if (!ctx.arg.load_file.empty()) {
    mem->load_from_file(ctx);
  }

  if (!ctx.arg.text_loadfile.empty()) {
    mem->load_from_text_file(ctx);
  }

  SyscMemoryTLMInitiator<E> tlm("tlm");
  tlm.init_socket.bind(mem->targ_socket);

  sc_core::sc_start();

  mem->dump_memory(ctx, "end-of-testbench");

  if (!ctx.arg.output.empty()) {
    mem->save_to_file(ctx);
  }

  if (!ctx.arg.text_output.empty()) {
    mem->save_to_text_file(ctx);
  }

  return tlm.pass;
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
void
SyscMemory<E>::load_from_file(Context<E>& ctx) {
  // Raw binary load path.
  //
  // This is for native/POD memory images only.
  // For SV/VCS-readable hex/bin text files, use load_from_text_file().
  if constexpr (!std::is_trivially_copyable_v<T>) {
    Fatal(ctx) << "Raw binary load is only supported for trivially-copyable "
                  "memory element types. Use load_from_text_file() instead.";
  } else {
    MappedFile* mf = open_file(ctx, ctx.arg.load_file);

    if (!mf) {
      Fatal(ctx) << "Cannot open file for memory load: " << ctx.arg.load_file;
    }

    const i64 offset = ctx.arg.load_offset < 0 ? 0 : ctx.arg.load_offset;
    const i64 nbytes = static_cast<i64>(mem_size * sizeof(T));

    if (offset < 0) {
      Fatal(ctx) << "Invalid negative load offset";
    }

    if (offset + nbytes > mf->size) {
      Fatal(ctx) << "File too small for memory load";
    }

    if (nbytes > 0) {
      std::memcpy(mem_data.get(),
                  mf->data + offset,
                  static_cast<std::size_t>(nbytes));
    }

    if (ctx.arg.verbose) {
      Out(ctx) << "Loaded " << mem_size
               << " raw elements from " << ctx.arg.load_file;
    }
  }
}


template <typename E>
void
SyscMemory<E>::save_to_file(Context<E>& ctx) {
  // Raw binary save path.
  //
  // This preserves the original behavior of save_to_file():
  //   mem_data bytes -> ctx.arg.output
  //
  // For SV-readable hex/bin text output, use save_to_text_file().
  if (ctx.arg.output.empty()) {
    Fatal(ctx) << "Cannot save memory: output path is empty";
  }

  if constexpr (!std::is_trivially_copyable_v<T>) {
    Fatal(ctx) << "Raw binary save is only supported for trivially-copyable "
                  "memory element types. Use save_to_text_file() instead.";
  } else {
    const i64 nbytes = static_cast<i64>(mem_size * sizeof(T));

    ctx.output_file = OutputFile<E>::open(ctx, ctx.arg.output, nbytes, 0777);

    if (!ctx.output_file) {
      Fatal(ctx) << "Cannot create OutputFile for memory saving";
    }

    if (nbytes > 0) {
      ctx.output_file->write_bytes(mem_data.get(), static_cast<std::size_t>(nbytes));
      ctx.buf = ctx.output_file->buf;
    }

    ctx.output_file->close(ctx);

    if (ctx.arg.verbose) {
      Out(ctx) << "Saved " << mem_size
               << " raw elements, " << nbytes
               << " bytes to " << ctx.arg.output;
    }
  }
}

template <typename E>
void SyscMemory<E>::save_to_text_file(Context<E>& ctx) {
  // Text save path.
  //
  // Modes:
  //   --oformat=hex
  //       arch value -> E::Fxpt_T -> SystemC to_hex() word
  //       suitable for SV $readmemh
  //
  //   --oformat=binary
  //       arch value -> E::Fxpt_T -> SystemC to_bin() word
  //       suitable for SV $readmemb
  //
  //   default
  //       decimal text
  if (ctx.arg.text_output.empty()) {
    Fatal(ctx) << "Cannot save memory text: text output path is empty";
  }

  ctx.output_file = OutputFile<E>::open(ctx, ctx.arg.text_output, 1 << 20, 0777);

  if (!ctx.output_file) {
    Fatal(ctx) << "Cannot open file for memory text saving: "
               << ctx.arg.text_output;
  }

  for (std::size_t i = 0; i < mem_size; ++i) {
    if (ctx.arg.oformat_hex) {
      ctx.output_file->write_line(archval_to_syscfx_hexword<E>(mem_data[i]));

    } else if (ctx.arg.oformat_binary) {
      ctx.output_file->write_line(archval_to_syscfx_binword<E>(mem_data[i]));

    } else {
      std::ostringstream oss;
      oss << mem_data[i];
      ctx.output_file->write_line(oss.str());
    }
  }

  ctx.output_file->close(ctx);

  if (ctx.arg.verbose) {
    const char* mode =
        ctx.arg.oformat_hex    ? "hex fixed-point text"
      : ctx.arg.oformat_binary ? "binary fixed-point text"
                               : "decimal text";

    Out(ctx) << "Saved " << mem_size
             << " elements as " << mode
             << " to " << ctx.arg.text_output;
  }
}

template <typename E>
void SyscMemory<E>::load_from_text_file(Context<E>& ctx) {
  // Text load path.
  //
  // Modes:
  //   --oformat=hex
  //       parse line as SV-style fixed-point hex word
  //       line -> E::Fxpt_T -> T
  //
  //   --oformat=binary
  //       parse line as SV-style fixed-point binary word
  //       line -> E::Fxpt_T -> T
  //
  //   default
  //       parse decimal text
  if (ctx.arg.text_loadfile.empty()) {
    Fatal(ctx) << "Cannot load memory text: text load path is empty";
  }

  std::ifstream ifs(ctx.arg.text_loadfile);

  if (!ifs) {
    Fatal(ctx) << "Cannot open file for memory text loading: "
               << ctx.arg.text_loadfile;
  }


  std::size_t i = 0;
  std::string line;

  while (i < mem_size && std::getline(ifs, line)) {
    line = strip_space(std::move(line));

    if (line.empty()) {
      continue;
    }

    if (ctx.arg.oformat_hex) {
      typename E::Fxpt_T q = archsyscfx_from_hexword<E>(line);
      mem_data[i++] = static_cast<T>(q);

    } else if (ctx.arg.oformat_binary) {
      typename E::Fxpt_T q = archsyscfx_from_binword<E>(line);
      mem_data[i++] = static_cast<T>(q);

    } else {
      if constexpr (std::is_integral_v<T>) {
        if constexpr (std::is_signed_v<T>) {
          mem_data[i++] = static_cast<T>(std::stoll(line, nullptr, 0));
        } else {
          mem_data[i++] = static_cast<T>(std::stoull(line, nullptr, 0));
        }
      } else {
        mem_data[i++] = static_cast<T>(std::stod(line));
      }
    }
  }

  if (ctx.arg.verbose) {
    if (i != mem_size) {
      Out(ctx) << "Warning: Only loaded " << i
               << " elements from text file "
               << ctx.arg.text_loadfile;
    } else {
      Out(ctx) << "Loaded " << mem_size
               << " elements from text file "
               << ctx.arg.text_loadfile;
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
void SyscMemory<E>::b_transport(tlm::tlm_generic_payload& trans,
                                sc_core::sc_time& delay) {
  const auto cmd = trans.get_command();
  const std::size_t addr = static_cast<std::size_t>(trans.get_address());
  const std::size_t len = static_cast<std::size_t>(trans.get_data_length());
  const std::size_t total = mem_size * sizeof(T);
  u8* ptr = trans.get_data_ptr();

  trans.set_dmi_allowed(false);

  if (!is_init || !mem_data) {
    trans.set_response_status(tlm::TLM_GENERIC_ERROR_RESPONSE);
    return;
  }

  if (cmd != tlm::TLM_READ_COMMAND &&
      cmd != tlm::TLM_WRITE_COMMAND) {
    trans.set_response_status(tlm::TLM_COMMAND_ERROR_RESPONSE);
    return;
  }

  if (trans.get_byte_enable_ptr() != nullptr) {
    trans.set_response_status(tlm::TLM_BYTE_ENABLE_ERROR_RESPONSE);
    return;
  }

  if (ptr == nullptr || len == 0 || trans.get_streaming_width() < len) {
    trans.set_response_status(tlm::TLM_BURST_ERROR_RESPONSE);
    return;
  }

  // Written this way to avoid overflow in addr + len.
  if (addr > total || len > total - addr ||
      addr % sizeof(T) != 0 || len % sizeof(T) != 0) {
    trans.set_response_status(tlm::TLM_ADDRESS_ERROR_RESPONSE);
    return;
  }

  u8* base = reinterpret_cast<u8*>(mem_data.get()) + addr;

  if (cmd == tlm::TLM_READ_COMMAND) {
    std::memcpy(ptr, base, len);
    delay += read_delay;
  } else {
    std::memcpy(base, ptr, len);
    delay += write_delay;
  }

  trans.set_response_status(tlm::TLM_OK_RESPONSE);
}

template <typename E>
bool SyscMemory<E>::get_direct_mem_ptr(
    tlm::tlm_generic_payload& trans,
    tlm::tlm_dmi& dmi_data) {
  if (!is_init || !mem_data || mem_size == 0) {
    return false;
  }

  const std::size_t addr =
      static_cast<std::size_t>(trans.get_address());
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
  dmi_data.set_read_latency(read_delay);
  dmi_data.set_write_latency(write_delay);

  return true;
}

template <typename E>
unsigned int SyscMemory<E>::transport_dbg(tlm::tlm_generic_payload& trans) {
  const auto cmd = trans.get_command();
  const std::size_t addr = static_cast<std::size_t>(trans.get_address());
  const std::size_t len = static_cast<std::size_t>(trans.get_data_length());
  const std::size_t total = mem_size * sizeof(T);
  u8* ptr = trans.get_data_ptr();

  if (!is_init || !mem_data) {
    trans.set_response_status(tlm::TLM_GENERIC_ERROR_RESPONSE);
    return 0;
  }

  if (cmd != tlm::TLM_READ_COMMAND &&
      cmd != tlm::TLM_WRITE_COMMAND) {
    trans.set_response_status(tlm::TLM_COMMAND_ERROR_RESPONSE);
    return 0;
  }

  if (ptr == nullptr || len == 0) {
    trans.set_response_status(tlm::TLM_BURST_ERROR_RESPONSE);
    return 0;
  }

  if (trans.get_byte_enable_ptr() != nullptr) {
    trans.set_response_status(tlm::TLM_BYTE_ENABLE_ERROR_RESPONSE);
    return 0;
  }

  if (addr >= total) {
    trans.set_response_status(tlm::TLM_ADDRESS_ERROR_RESPONSE);
    return 0;
  }

  const std::size_t nbytes = std::min(len, total - addr);
  u8* base = reinterpret_cast<u8*>(mem_data.get()) + addr;

  if (cmd == tlm::TLM_READ_COMMAND) {
    std::memcpy(ptr, base, nbytes);
  } else {
    std::memcpy(base, ptr, nbytes);
  }

  trans.set_dmi_allowed(false);
  trans.set_response_status(tlm::TLM_OK_RESPONSE);
  return static_cast<unsigned int>(std::min<std::size_t>(
      nbytes, std::numeric_limits<unsigned int>::max()));
}

template bool SyscMemory<E>::run_testbench(Context<E>&);

template std::unique_ptr<SyscMemory<E>> 
SyscMemory<E>::create(Context<E>&, sc_core::sc_module_name, std::size_t, typename E::Eval_T*);

template class SyscMemory<E>;

}  // namespace adptsysc

#endif